from pathlib import Path
import runpy

import pytest


SCRIPT = Path(__file__).parents[1] / "writes_hardware" / "run_right_box_pregrasp.py"
MODULE = runpy.run_path(str(SCRIPT))


def test_goal_is_the_reviewed_mujoco_relative_motion() -> None:
    start = {name: 10.0 for name in (*MODULE["MOVING_JOINTS"], "gripper")}
    goal = MODULE["build_goal"](start)
    for name, delta in MODULE["MUJOCO_DELTA_DEG"].items():
        assert goal[name] == pytest.approx(start[name] + delta)
    assert goal["gripper"] == start["gripper"]


def test_septic_profile_starts_and_ends_without_overshoot() -> None:
    weights = [MODULE["_septic_weight"](index / 100) for index in range(101)]
    assert weights[0] == 0.0
    assert weights[-1] == pytest.approx(1.0)
    assert weights == sorted(weights)


def test_goal_outside_calibration_is_rejected() -> None:
    calibration = {
        name: {"range_min": 1500, "range_max": 2500}
        for name in MODULE["MOVING_JOINTS"]
    }
    with pytest.raises(RuntimeError, match="calibrated range"):
        MODULE["validate_goal"](
            {name: 100.0 for name in MODULE["MOVING_JOINTS"]}, calibration
        )


def test_exact_confirmation_is_required(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(MODULE["os"], "isatty", lambda _fd: True)
    with pytest.raises(ValueError, match=MODULE["CONFIRMATION"]):
        MODULE["_require_request"]("wrong", True)
    MODULE["_require_request"](MODULE["CONFIRMATION"], True)


def test_motion_leg_uses_only_bounded_mujoco_joint_goals(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeBus:
        def __init__(self) -> None:
            self.position = {name: 0.0 for name in MODULE["MOVING_JOINTS"]}
            self.writes = []

        def sync_write(self, register, values, **_kwargs):
            assert register == "Goal_Position"
            assert set(values) == set(MODULE["MOVING_JOINTS"])
            self.position = dict(values)
            self.writes.append(dict(values))

        def sync_read(self, register, *_args, **_kwargs):
            if register == "Present_Position":
                return dict(self.position)
            value = 120 if register == "Present_Voltage" else 0
            if register == "Torque_Enable":
                value = 1
            return {name: value for name in MODULE["MOVING_JOINTS"]}

    monkeypatch.setattr(MODULE["time"], "monotonic", lambda: 0.0)
    monkeypatch.setattr(MODULE["time"], "sleep", lambda _seconds: None)
    bus = FakeBus()
    start = dict(bus.position)
    goal = MODULE["build_goal"](start)
    trace = []
    MODULE["run_leg"](bus, start, goal, trace)
    assert len(bus.writes) == round(MODULE["DURATION_S"] / MODULE["PERIOD_S"])
    assert bus.writes[-1] == pytest.approx(goal)
    assert trace[-1]["max_tracking_error_deg"] == 0.0


def test_reviewed_trajectory_stays_inside_explicit_limits() -> None:
    MODULE["validate_trajectory_limits"]()


def test_physical_execution_stays_blocked_after_role_mismatch() -> None:
    assert MODULE["PHYSICAL_ROLE_MAPPING_VERIFIED"] is False
