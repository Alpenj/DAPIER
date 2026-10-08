"""Regression tests of the R6 field path (no hardware, no ROS): each one blocks a defect found in the R5 tools.
Fake bus / fake camera objects stand for the devices; the calibration files are the project's real lerobot files."""
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from sweepick.perception import sweepick_observation_capture as f5
from sweepick.perception import sweepick_feedback_observation as f6

CAL = Path("~/.config/dapier/lerobot-calibration").expanduser()
pytestmark = pytest.mark.skipif(not (CAL / "dapier_dual_follower_left.json").exists(), reason="the project's lerobot calibration files are not on this machine")


class FakeBus:
    """Ticks around the calibrated mid range; the command (Goal_Position) differs from the measured position."""

    def __init__(self, cal, *, gripper_fraction=0.3, nan=False, no_current=True, drift=0.0):
        self.cal, self.g, self.nan, self.no_current, self.drift, self.k, self.written, self.disconnect_arg = cal, gripper_fraction, nan, no_current, drift, 0, [], None

    def sync_read(self, reg, normalize=False):
        if reg == "Present_Current" and self.no_current:
            raise KeyError("no such register on this firmware")
        self.k += 1
        mid = {n: (c["range_min"] + c["range_max"]) / 2 for n, c in self.cal.items()}
        g = self.cal["gripper"]
        present = dict(mid, gripper=g["range_min"] + self.g * (g["range_max"] - g["range_min"]) + self.drift * self.k)
        if self.nan:
            present["elbow_flex"] = float("nan")
        return {"Present_Position": present, "Goal_Position": {n: v - 12.0 for n, v in present.items()}, "Present_Velocity": {n: 0.0 for n in mid}, "Present_Load": {n: 40.0 for n in mid},
                "Torque_Enable": {n: 1.0 for n in mid}}[reg]

    def write(self, *a):
        self.written.append(a)

    def disconnect(self, disable_torque=True):
        self.disconnect_arg = disable_torque


@pytest.mark.parametrize(("failure", "is_open"), [(ConnectionError("handshake failed"), False), (KeyboardInterrupt(), True)])
def test_read_only_bus_cleans_up_failed_connect_without_motor_writes(failure, is_open):
    class Port:
        def __init__(self):
            self.is_open, self.closed = is_open, False

        def closePort(self):
            if not self.is_open:
                raise AttributeError("'NoneType' object has no attribute 'close'")
            self.is_open = False
            self.closed = True

    class Bus:
        def __init__(self):
            self.port_handler, self.write_called, self.disconnect_arg = Port(), False, None

        def connect(self):
            raise failure

        def write(self, *args):
            self.write_called = True

        def disconnect(self, disable_torque=True):
            self.disconnect_arg = disable_torque

    raw = Bus()
    bus = f5.ReadOnlyBus(raw)
    with pytest.raises(type(failure)) as caught:
        bus.connect()
    assert caught.value is failure and raw.port_handler.closed is is_open
    with pytest.raises(f5.ReadOnlyViolation):
        bus.write("Goal_Position", "shoulder_pan", 1)
    assert bus.refused == ["write"] and not raw.write_called
    bus.disconnect()
    assert raw.disconnect_arg is False


def contract(aligned=True, **over):
    c = f6.template()
    if aligned:                                                               # synthetic alignment, for these tests only
        for side in ("left", "right"):
            for i, j in enumerate(c["q12"][side]["joints"]):
                j.update(sign=-1.0 if i == 1 else 1.0, offset_rad=0.1 * i, verified=True)
            c["q12"][side]["gripper"].update(driver_percent_closed=10.0, driver_percent_open=90.0, verified=True)
    c.update(over)
    return c


def cal_of(c):
    return {s: v["joints"] for s, v in f6.load_calibration(c).items()}


def sample(tmp_path, c, *, bus_kw=None, stamps=(100.0, 100.01, 100.02), fb_time=100.0, K=(500.0, 0, 320.0, 0, 500.0, 240.0, 0, 0, 1.0), depth=False, tf="READ"):
    img = lambda s: np.full((480, 640, 3), 80, dtype=np.uint8)
    frames = dict(top_color=[dict(data=img(0), stamp_s=stamps[0])], left_wrist=[dict(data=img(0), stamp_s=stamps[1])], right_wrist=[dict(data=img(0), stamp_s=stamps[2])])
    f = lambda rate: dict(status="READ", encoding="rgb8", kind="COLOUR", rate_hz=rate, stamp_source="test")
    facts = dict(top_color=f(30.0), left_wrist=f(30.0), right_wrist=f(30.0), top_info=dict(status="READ", K=list(K), width=640, height=480))
    if depth:
        frames["top_depth"] = [dict(data=np.full((480, 640), 600, dtype=np.uint16), stamp_s=stamps[0])]
        facts["top_depth"] = dict(status="READ", encoding="16UC1", unit="millimetres (uint16, REP 118)", rate_hz=30.0)
    f5.save_sample(tmp_path, frames, facts, dict(contract=c, joints={}, tf=dict(status=tf, reason="test")))
    cals = cal_of(c)
    clock = iter(np.arange(fb_time, fb_time + 100, 0.001))
    rows = {s: f6.read_feedback_series(f5.ReadOnlyBus(FakeBus(cals[s], **(bus_kw or {}))), samples=5, period_s=0.0, clock=lambda: float(next(clock)), sleep=lambda x: None) for s in ("left", "right")}
    (tmp_path / "feedback.json").write_text(json.dumps(f6.feedback_document("fake bus (test)", rows, f6.load_calibration(c)), default=str))
    return tmp_path


def test_bus_feedback_without_jointstate_becomes_q12_through_the_real_calibration(tmp_path):
    c = contract()
    info, arrays = f6.model_input(sample(tmp_path, c))
    q = arrays["q12"]
    assert q.shape == (12,) and np.isfinite(q).all()
    assert np.allclose(q[:5], [0.0, 0.1, 0.2, 0.3, 0.4], atol=1e-6)            # mid-range ticks = 0 deg -> sign * 0 + offset
    assert abs(q[5] - (30.0 - 10.0) / 80.0) < 1e-6                             # 30 % of the calibrated gripper range, closed 10 %, open 90 %
    left = info["q12"]["left"]
    assert left["status"] == "MAPPED" and left["calibration"]["used"].startswith("calibration snapshot bound to the sample") and left["source"] == "fake bus (test)"
    assert info["calibration_comparison"]["left"]["used"] == "snapshot" and info["calibration_comparison"]["left"]["same_content"] is True
    assert left["goal_raw_ticks"]["gripper"] == left["raw_ticks"]["gripper"] - 12.0    # the command channel is kept, and is not the measured one
    assert info["readiness"]["policy_tensor_possible"]["ok"] and info["readiness"]["q_mapping_verified"]["ok"] and info["ready_for"]["policy_input_shadow"]
    assert info["ready_for"]["manipulation"] is False


def test_feedback_series_keeps_every_sample_with_its_read_window_and_channel_roles(tmp_path):
    c = contract()
    doc = json.loads((sample(tmp_path, c) / "feedback.json").read_text())
    arm = doc["arms"]["left"]
    assert arm["samples"] == 5 and [r["seq"] for r in arm["rows"]] == [0, 1, 2, 3, 4] and all(r["t_end"] > r["t_start"] for r in arm["rows"])
    assert arm["availability"]["current_raw"] == "UNAVAILABLE" and arm["availability"]["load_raw"] == "READ" and arm["command_and_measured_are_separate_channels"] and arm["command_echo"] is False
    assert arm["rows"][0]["registers"]["goal_raw"]["role"] == "command" and arm["rows"][0]["registers"]["present_raw"]["role"] == "measured"


def test_empty_alignment_keeps_driver_units_and_makes_no_q12(tmp_path):
    info, arrays = f6.model_input(sample(tmp_path, contract(aligned=False)))
    assert "q12" not in arrays and info["q12"]["left"]["status"] == "UNVERIFIED" and len(info["q12"]["left"]["driver_arm_rad"]) == 5 and "gripper closed / open" in info["q12"]["left"]["empty"]
    assert not info["readiness"]["policy_tensor_possible"]["ok"] and not info["ready_for"]["shape_only_shadow"]


def test_nan_feedback_is_invalid_not_ready(tmp_path):
    info, arrays = f6.model_input(sample(tmp_path, contract(), bus_kw=dict(nan=True)))
    assert "q12" not in arrays and info["q12"]["left"]["status"] == "INVALID" and not info["readiness"]["policy_tensor_possible"]["ok"]


def test_out_of_range_gripper_is_recorded_not_clipped(tmp_path):
    info, arrays = f6.model_input(sample(tmp_path, contract(), bus_kw=dict(gripper_fraction=1.2)))
    assert arrays["q12"][5] > 1.0 and "gripper opening outside [0, 1]" in info["q12"]["left"]["flags"] and "gripper outside its calibrated range" in info["q12"]["left"]["flags"]


def test_zero_focal_K_and_missing_transform_fail_geometry_only(tmp_path):
    info, _ = f6.model_input(sample(tmp_path, contract(), K=(0.0, 0, 320.0, 0, 0.0, 240.0, 0, 0, 1.0), tf="MISSING"))
    assert not info["geometry"]["top_K"]["valid"] and "focal" in info["geometry"]["top_K"]["reason"] and not info["geometry"]["camera_to_base"]["valid"]
    assert not info["readiness"]["geometry_valid"]["ok"] and info["readiness"]["policy_tensor_possible"]["ok"]      # the tensors exist; the geometry does not: two different answers


def test_twenty_second_skew_is_not_time_aligned(tmp_path):
    info, _ = f6.model_input(sample(tmp_path, contract(), stamps=(100.0, 120.0, 100.01)))
    assert abs(info["pairing"]["skew_s"]["left"] - 20.0) < 1e-6 and info["pairing"]["within_limit"]["left"] is False
    assert not info["readiness"]["time_aligned"]["ok"] and not info["ready_for"]["policy_input_shadow"] and info["ready_for"]["shape_only_shadow"]


def test_stale_feedback_is_not_time_aligned(tmp_path):
    info, _ = f6.model_input(sample(tmp_path, contract(), fb_time=80.0))
    assert info["pairing"]["within_limit"]["joints_left"] is False and not info["readiness"]["time_aligned"]["ok"]


def test_depth_of_the_same_size_is_not_used_unless_registration_is_declared(tmp_path):
    info, arrays = f6.model_input(sample(tmp_path / "a", contract(), depth=True))
    assert "top_depth_m" not in arrays and not info["geometry"]["depth"]["valid"] and "would not prove registration" in info["geometry"]["depth"]["reason"]
    c = contract()
    c["depth"]["registered_to_color"] = True
    info, arrays = f6.model_input(sample(tmp_path / "b", c, depth=True))
    assert info["geometry"]["depth"]["valid"] and arrays["top_depth_m"].shape == (120, 160) and abs(float(arrays["top_depth_m"].mean()) - 0.6) < 1e-6


def test_devices_and_bus_are_read_in_the_same_window_not_one_after_the_other(tmp_path):
    class Cam:
        def __init__(self, device):
            self.n = 0

        def isOpened(self):
            return True

        def read(self):
            time.sleep(0.02)
            self.n += 1
            return True, np.full((48, 64, 3), self.n % 255, dtype=np.uint8)

        def release(self):
            pass

    c = contract()
    cals = cal_of(c)
    out = f6.capture(c, tmp_path, duration_s=0.5, samples=10, with_arms=True, open_camera=Cam, open_bus=lambda side, port: FakeBus(cals[side]), ros=False)
    meta = json.loads((out / "sample.json").read_text())
    raw = np.load(out / "raw.npz")
    fb = json.loads((out / "feedback.json").read_text())
    l, r = raw["left_wrist_stamp"], raw["right_wrist_stamp"]
    bt = [x["t_start"] for x in fb["arms"]["left"]["rows"]]
    assert max(l[0], r[0], bt[0]) < min(l[-1], r[-1], bt[-1])                   # the three reading intervals overlap
    assert "NOT a capture time" in meta["streams"]["left_wrist"]["stamp_source"] and sorted(meta["concurrent"]["devices"]) == ["left_wrist", "right_wrist"]
    assert meta["commands_sent"] if "commands_sent" in meta else True


def test_calibration_problems_are_reported(tmp_path):
    c = contract()
    c["lerobot"]["calibration_dir"] = str(tmp_path)
    assert f6.load_calibration(c)["left"]["status"] == "MISSING"
    bad = json.loads((CAL / "dapier_dual_follower_left.json").read_text())
    bad["gripper"]["range_max"] = bad["gripper"]["range_min"]
    (tmp_path / "dapier_dual_follower_left.json").write_text(json.dumps(bad))
    assert f6.load_calibration(c)["left"]["status"] == "INVALID"


def test_missing_top_stream_is_reported_and_nothing_is_ready(tmp_path):
    c = contract()
    cals = cal_of(c)

    class Cam:
        def __init__(self, device):
            pass

        def isOpened(self):
            return True

        def read(self):
            time.sleep(0.02)
            return True, np.zeros((48, 64, 3), dtype=np.uint8)

        def release(self):
            pass

    out = f6.capture(c, tmp_path, duration_s=0.3, samples=5, with_arms=True, open_camera=Cam, open_bus=lambda side, port: FakeBus(cals[side]), ros=False)
    info, arrays = f6.model_input(out)
    assert "top" not in arrays and not info["readiness"]["arrays_present"]["ok"] and not info["readiness"]["time_aligned"]["ok"] and not any(v for k, v in info["ready_for"].items() if k != "manipulation_note")


# ---- R6.1 / 1A: the sample's own calibration decides; another file of the same name on the analysing machine does not ----
def shifted_dir(tmp_path, ticks=100):
    d = tmp_path / "other_calibration"
    d.mkdir(parents=True, exist_ok=True)
    for side in ("left", "right"):
        c = json.loads((CAL / f"dapier_dual_follower_{side}.json").read_text())
        if side == "left":
            c["shoulder_pan"]["range_min"] += ticks                               # midpoint + 100 ticks
            c["shoulder_pan"]["range_max"] += ticks
        (d / f"dapier_dual_follower_{side}.json").write_text(json.dumps(c))
    return d


def test_a_different_calibration_file_on_the_analysing_machine_does_not_change_the_sample(tmp_path):
    c = contract()
    s = sample(tmp_path / "s", c)
    ref, a0 = f6.model_input(s)
    meta = json.loads((s / "sample.json").read_text())
    meta["contract"]["lerobot"]["calibration_dir"] = str(shifted_dir(tmp_path))  # as if the same path held other numbers on the server
    (s / "sample.json").write_text(json.dumps(meta))
    info, a1 = f6.model_input(s)
    assert np.array_equal(a0["q12"], a1["q12"]) and info["readiness"]["q_mapping_verified"]["ok"]
    assert info["calibration_comparison"]["left"]["same_content"] is False and info["calibration_comparison"]["left"]["used"] == "snapshot"
    meta["contract"]["lerobot"]["calibration_dir"] = str(tmp_path / "nowhere")   # and with no file at all on the analysing machine
    (s / "sample.json").write_text(json.dumps(meta))
    info, a2 = f6.model_input(s)
    assert np.array_equal(a0["q12"], a2["q12"]) and info["calibration_comparison"]["left"]["file_on_this_machine"] == "MISSING"


def test_an_explicit_override_is_a_separate_unverified_artifact(tmp_path):
    c = contract()
    s = sample(tmp_path / "s", c)
    ref, a0 = f6.model_input(s)
    with pytest.raises(ValueError):
        f6.model_input(s, shifted_dir(tmp_path))
    info, a1 = f6.model_input(s, shifted_dir(tmp_path), "test: midpoint of the left pan moved by 100 ticks")
    assert abs(float(a1["q12"][0] - a0["q12"][0]) - (-100 * 2 * math.pi / 4095)) < 1e-5          # about -0.1534 rad
    assert info["artifact"].startswith("model_input_r6_override_") and (s / (info["artifact"] + ".json")).exists() and (s / "model_input_r6.json").exists()
    assert not info["readiness"]["q_mapping_verified"]["ok"] and not info["ready_for"]["policy_input_shadow"]
    assert info["calibration_override"]["previous"]["left"] != info["calibration_override"]["new"]["left"] and info["q12"]["left"]["verification_note"].startswith("NOT verified")
    again, a2 = f6.model_input(s)                                                                # the default artifact is still the capture interpretation
    assert np.array_equal(a0["q12"], a2["q12"]) and again["artifact"] == "model_input_r6"


def test_a_snapshot_whose_content_does_not_match_its_recorded_sha_is_invalid(tmp_path):
    c = contract()
    s = sample(tmp_path / "s", c)
    fb = json.loads((s / "feedback.json").read_text())
    assert fb["calibration_at_capture"]["left"]["content_sha256"] == f6.content_sha(fb["calibration_at_capture"]["left"]["joints"])
    fb["calibration_at_capture"]["left"]["joints"]["shoulder_pan"]["range_min"] += 100            # the numbers were altered after the capture; the label still says READ
    (s / "feedback.json").write_text(json.dumps(fb))
    info, arrays = f6.model_input(s)
    assert "q12" not in arrays and info["calibration_applied"]["left"]["status"] == "INVALID" and not info["readiness"]["q_mapping_verified"]["ok"]


def test_sample_without_a_snapshot_is_not_converted_with_a_local_file(tmp_path):
    c = contract()
    s = sample(tmp_path / "s", c)
    fb = json.loads((s / "feedback.json").read_text())
    fb.pop("calibration_at_capture")
    (s / "feedback.json").write_text(json.dumps(fb))
    info, arrays = f6.model_input(s)
    assert "q12" not in arrays and info["q12"]["left"]["status"] == "MISSING" and info["calibration_comparison"]["left"]["file_on_this_machine"] == "READ"


def test_stamp_agreement_is_reported_apart_from_exposure_simultaneity(tmp_path):
    info, _ = f6.model_input(sample(tmp_path, contract()))
    ta = info["readiness"]["time_aligned"]
    assert ta["ok"] and ta["exposure_simultaneity"]["verified"] is False and any("one after the other" in r for r in ta["exposure_simultaneity"]["reasons"])


def test_corrupted_stationary_wrist_frames_are_never_paired_and_all_corrupted_stops_the_input():
    rng = np.random.default_rng(0)
    base = (rng.integers(0, 255, (120, 160, 3))).astype(np.uint8)
    clean = [np.clip(base.astype(np.int16) + rng.integers(-1, 2, base.shape), 0, 255).astype(np.uint8) for _ in range(8)]
    bad = np.roll(base, 37, axis=1)
    frames = clean[:3] + [bad] + clean[3:]
    mask, diff = f6.stream_integrity(np.array(frames))
    assert mask.tolist() == [True, True, True, False, True, True, True, True, True] and diff[3] > f6.INTEGRITY_MAX_NEIGHBOUR_DIFF
    allbad = np.array([np.roll(base, 11 * (i + 1), axis=1) for i in range(6)])
    assert not f6.stream_integrity(allbad)[0].any()
    still = dict(arms=dict(left=dict(rows=[dict(registers=dict(present_raw=dict(values={n: 100.0 for n in f6.JOINTS})))] * 3)))
    moving = dict(arms=dict(left=dict(rows=[dict(registers=dict(present_raw=dict(values={n: 100.0 + 5 * i for n in f6.JOINTS}))) for i in range(3)])))
    assert f6.arms_stationary(still) is True and f6.arms_stationary(moving) is False and f6.arms_stationary(dict(arms={})) is None
