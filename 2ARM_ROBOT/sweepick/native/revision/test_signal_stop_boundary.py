"""NO_DEVICE checks that an operator STOP is a write boundary for arm and gripper execution."""
from pathlib import Path

import pytest

import sweepick_move_a as move
import sweepick_real_command as rc
from test_sweepick_grip import Jaws, OPEN, go as grip
from test_sweepick_move_a import Fake, MID, go as run


pytestmark = pytest.mark.skipif(
    not (Path.home() / "sweepick_261007_commission/inputs/CANDIDATE_Q12_CONFIG.json").exists(),
    reason="candidate config not on this machine",
)


def test_stop_before_grip_bootstrap_writes_nothing_and_is_stopped_holding(tmp_path):
    fake = Jaws(OPEN)
    code, log = grip(tmp_path, fake, stop_requested=lambda: True)
    assert code == move.EXIT["STOPPED_HOLDING"] and log["result"] == "STOPPED_HOLDING"
    assert log["stopped_by"]["gate"] == "operator_stop" and fake.writes == []
    assert not any(fake.reg["Torque_Enable"].values()) and "release" not in log


def test_stop_during_grip_gets_only_the_existing_fresh_hold_and_no_open_or_off(tmp_path):
    fake = Jaws(OPEN, block=3075)
    writes_at_stop = []

    def stopped():
        if len(fake.writes) >= 4:
            writes_at_stop.append(len(fake.writes))
            return True
        return False

    code, log = grip(tmp_path, fake, stop_requested=stopped)
    assert code == move.EXIT["STOPPED_HOLDING"] and log["result"] == "STOPPED_HOLDING"
    assert writes_at_stop and len(fake.writes) == writes_at_stop[0] + 1
    assert fake.writes[-1] == ("Goal_Position", "gripper", fake.reg["Present_Position"]["gripper"])
    assert not any(reg == "Torque_Enable" and value == 0 for reg, _joint, value in fake.writes)
    assert "release" not in log and fake.reg["Torque_Enable"]["gripper"] == 1


def test_bus_loss_during_grip_stop_hold_is_recorded_and_disconnected(tmp_path):
    fake = Jaws(OPEN, block=3075)

    def stopped():
        if len(fake.writes) >= 4:
            fake.dead = True
            return True
        return False

    code, log = grip(tmp_path, fake, stop_requested=stopped)
    assert code == move.EXIT["BUS_LOST"] and log["result"] == "BUS_LOST"
    assert log["stopped_by"]["gate"] == "operator_stop" and log["end"]["error"] == "no read possible"
    assert fake.disconnected is False


def test_stop_at_normal_return_boundary_sends_no_off_or_following_goal(tmp_path):
    fake = Fake(MID)
    code, log = run(tmp_path / "out", fake, target=dict(elbow_flex=2000), execute=True)
    assert code == 0 and log["result"] == "HOLDING"
    writes_at_stop = []

    def stopped():
        if fake.reg["Present_Position"]["elbow_flex"] == MID["elbow_flex"]:
            writes_at_stop.append(len(fake.writes))
            return True
        return False

    code, log = run(tmp_path / "return", fake, mode="return", rest=dict(elbow_flex=MID["elbow_flex"]),
                    execute=True, stop_requested=stopped)
    assert code == move.EXIT["STOPPED_HOLDING"] and log["result"] == "STOPPED_HOLDING"
    assert writes_at_stop and len(fake.writes) == writes_at_stop[0]
    assert fake.reg["Torque_Enable"]["elbow_flex"] == 1 and "release" not in log


def test_normal_return_without_stop_still_releases(tmp_path):
    fake = Fake(MID)
    assert run(tmp_path / "out", fake, target=dict(elbow_flex=2000), execute=True)[0] == 0
    code, log = run(tmp_path / "return", fake, mode="return", rest=dict(elbow_flex=MID["elbow_flex"]), execute=True)
    assert code == 0 and log["result"] == "RETURNED_RELEASED"
    assert not any(fake.reg["Torque_Enable"].values())


def test_stop_after_first_return_off_preserves_mixed_torque_without_goal_or_more_off(tmp_path):
    fake = Fake(MID)
    code, log = run(tmp_path / "out", fake, target=dict(elbow_flex=2000, gripper=3000),
                    torque_joints=["elbow_flex", "gripper"], execute=True)
    assert code == 0 and log["result"] == "HOLDING"
    writes_at_stop = []

    def stopped():
        off = [write for write in fake.writes if write[0] == "Torque_Enable" and write[2] == 0]
        if off:
            if not writes_at_stop:
                still_on = next(joint for joint, value in fake.reg["Torque_Enable"].items() if value)
                fake.reg["Present_Position"][still_on] -= 1
            writes_at_stop.append(len(fake.writes))
            return True
        return False

    code, log = run(tmp_path / "return", fake, mode="return",
                    rest=dict(elbow_flex=MID["elbow_flex"], gripper=MID["gripper"]),
                    torque_joints=["elbow_flex", "gripper"], execute=True, stop_requested=stopped)
    assert code == move.EXIT["STOPPED_HOLDING"] and log["result"] == "STOPPED_HOLDING"
    assert writes_at_stop and len(fake.writes) == writes_at_stop[0] + 1
    assert len([write for write in fake.writes if write[0] == "Torque_Enable" and write[2] == 0]) == 1
    assert sum(fake.reg["Torque_Enable"].values()) == 1
    assert log["end"]["torque_on"] in (["elbow_flex"], ["gripper"])
    off_joint = next(joint for reg, joint, value in fake.writes if reg == "Torque_Enable" and value == 0)
    assert fake.writes[-1][0] == "Goal_Position" and fake.writes[-1][1] != off_joint
