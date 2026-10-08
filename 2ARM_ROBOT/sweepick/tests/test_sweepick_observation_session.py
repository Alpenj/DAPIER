"""Live source and live-read-only session against fake devices (no hardware). The models are loaded once for the module."""
import json, threading, time, types
from pathlib import Path
import numpy as np, pytest
from sweepick.integration import sweepick_observation_session as L; from sweepick.control import sweepick_joint_command_mapping as rc

pytestmark = pytest.mark.skipif(not (L.RT / "checkpoints/START_3V/step/model.safetensors").exists(), reason="local runtime package not on this machine")
TICKS = dict(left=dict(shoulder_pan=2040, shoulder_lift=1038, elbow_flex=3149, wrist_flex=2043, wrist_roll=2057, gripper=3372), right=dict(shoulder_pan=2103, shoulder_lift=1056, elbow_flex=3147, wrist_flex=2004, wrist_roll=2099, gripper=2981))
LIM = L.tt.load_profile()


class Bus:
    def __init__(self, side, log, fail_read_after=None):
        self.side, self.log, self.n, self.fail = side, log, 0, fail_read_after
        log.append(("open", side))
    def sync_read(self, reg, normalize=True):
        assert normalize is False
        self.n += 1
        if self.fail is not None and self.n > self.fail: raise ConnectionError("no status packet")
        return dict(TICKS[self.side]) if reg in ("Present_Position", "Goal_Position") else {n: 0 for n in rc.JOINTS}
    def write(self, *a, **k): raise AssertionError("a write reached the bus")
    def disconnect(self): self.log.append(("disconnect", self.side))


class Cam:
    def __init__(self, name, log, period=0.033):
        self.name, self.log, self.n, self.t0, self.period, self.info, self.frozen = name, log, 0, time.monotonic(), period, dict(fake=True), False
        self.intervals = [period] * 10
        log.append(("open", name))
        self.img = np.full((240, 320, 3), 120, np.uint8)
    def get(self):
        if not self.frozen:
            self.n = int((time.monotonic() - self.t0) / self.period) + 1; self.t = self.t0 + (self.n - 1) * self.period
        return dict(frame=self.img, recv_monotonic_s=self.t, n=self.n), dict(frame=self.img, recv_monotonic_s=self.t - self.period, n=self.n - 1)
    def close(self):
        self.log.append(("close", self.name)); return dict(frames_read=self.n, thread_alive=False)


class Sdk:
    K, rect_sha, cmd, error = [818.6, 0, 626.3, 0, 818.6, 462.4, 0, 0, 1], "x", ["fake-helper"], None
    def __init__(self, log, fail=False, period=0.033):
        if fail: raise RuntimeError("camera not found")
        self.log, self.t0, self.period, self.done = log, time.monotonic(), period, threading.Event()
        self.intervals = [period] * 10
        log.append(("open", "sdk")); self.bgr = np.full((920, 1280, 3), 90, np.uint8); self.zd = np.zeros((460, 640), np.uint16)
    def get(self):
        n = int((time.monotonic() - self.t0) / self.period) + 1; t = self.t0 + (n - 1) * self.period
        return dict(n=n, frame_number=[n, n], sdk_color_stamp_s=1.7e9 + t, sdk_depth_stamp_s=1.7e9 + t + 0.001, helper_receipt_monotonic_s=t, recv_monotonic_s=t + 0.005, bgr=self.bgr, zd=self.zd)
    def close(self):
        self.log.append(("close", "sdk")); return dict(ended_by="itself (normal close path)")


def source(log, tmp, **kw):
    return L.LiveSource(LIM, tmp, open_bus=kw.pop("open_bus", lambda s: Bus(s, log)), open_wrist=lambda name, dev: Cam(name, log), open_sdk=kw.pop("open_sdk", lambda: Sdk(log)), top_wait_s=1.0, **kw)


def test_second_bus_failing_releases_the_first_and_touches_nothing_else(tmp_path):
    log = []
    def open_bus(side):
        if side == "right": raise RuntimeError("FeetechMotorsBus motor check failed on port '/dev/dapier/right_arm'")
        return Bus(side, log)
    with pytest.raises(L.SourceUnavailable) as e:
        source(log, tmp_path, open_bus=open_bus)
    d = e.value.devices["devices"]
    assert log == [("open", "left"), ("disconnect", "left")]
    assert d["left_arm"]["state"] == "opened" and d["right_arm"]["state"] == "failed" and all(d[k]["state"] == "not-attempted" for k in ("left_wrist", "right_wrist", "top_sdk"))


def test_sdk_failing_after_buses_and_cameras_releases_all_of_them(tmp_path):
    log = []
    with pytest.raises(L.SourceUnavailable) as e:
        source(log, tmp_path, open_sdk=lambda: Sdk(log, fail=True))
    assert set(log) == {("open", "left"), ("open", "right"), ("open", "left_wrist"), ("open", "right_wrist"), ("close", "left_wrist"), ("close", "right_wrist"), ("disconnect", "left"), ("disconnect", "right")}
    assert e.value.devices["devices"]["top_sdk"]["state"] == "failed"


def test_observation_keeps_the_real_readback_and_refuses_a_stale_frame(tmp_path):
    log = []; src = source(log, tmp_path)
    o = src.next()
    assert o["usable"] and o["ticks"] == TICKS and o["frame_n"]["top"] >= 1 and o["K"] == Sdk.K and o["depth_mm"].shape == (460, 640)
    assert set(o["age_ms"]) == {"top_since_helper_receipt", "top_since_this_process_received", "left_wrist", "right_wrist"} and "sdk_color_stamp_s" in o["top"]
    assert o["freshness_limit_ms"]["left_wrist"] == pytest.approx(66.0, abs=0.5)                  # two of the camera's own frame intervals; no fixed time
    src.cam["left"].frozen = True; time.sleep(0.12)
    o2 = src.next()
    assert not o2["usable"] and any("left_wrist" in w and "stale" in w for w in o2["why"]) and "left" not in o2
    assert src.close() is src.close() and ("disconnect", "right") in log and ("close", "sdk") in log


@pytest.fixture(scope="module")
def rt():
    return L.load_runtime()


def args(**k):
    return types.SimpleNamespace(seconds=k.get("seconds", 0.8), branch_exercise_at=k.get("branch"))


def test_live_session_uses_the_real_measurement_never_the_planned_goal_and_writes_nothing(tmp_path, rt):
    log = []
    code, s = L.run_live(args(), tmp_path, rt, source_factory=lambda: source(log, tmp_path))
    rows = [json.loads(x) for x in open(tmp_path / "LIVE_READONLY.jsonl")]
    body = [r for r in rows if "seq" in r]
    assert code == 0 and s["writes"] == dict(goal=0, torque=0, base=0) and s["owners"] == {"ACT:START_3V": s["consumed"]} and "fixed START" in s["owner_selection"]
    assert all(r["measured_ticks"] == TICKS and r["measurement_source"].startswith("REAL") for r in body)                 # the planned goal never replaces the readback
    moved = [r for r in body if r.get("consumed") and r["planned"]["would_write"]["left"]["gripper"] != TICKS["left"]["gripper"]]
    assert moved and all(r["planned"]["sent"] is False and r["planned"]["plant"].startswith("SYNTHETIC") for r in moved)
    assert s["tracking_with_the_real_arm"].startswith("not evaluated") and s["forwards"] >= 1 and s["unique"]["top_framesets"] < s["consumed"] and s["reused_in_consecutive_consumed_cycles"]["top"] > 0
    assert rows[-1]["released"]["left_arm"].startswith("disconnected") and ("close", "sdk") in log


def test_branch_exercise_is_labelled_and_off_by_default(tmp_path, rt):
    log = []
    code, s = L.run_live(args(seconds=0.6, branch=10), tmp_path, rt, source_factory=lambda: source(log, tmp_path))
    assert code == 0 and "BRANCH EXERCISE" in s["owner_selection"] and "NOT a task transition" in s["owner_selection"] and set(s["owners"]) == {"ACT:START_3V", "ACT:RECEIVE_3V"}


def test_an_exception_inside_the_session_still_releases_everything(tmp_path, rt):
    log = []
    code, s = L.run_live(args(seconds=2.0), tmp_path, rt, source_factory=lambda: source(log, tmp_path, open_bus=lambda side: Bus(side, log, fail_read_after=40)))
    rows = [json.loads(x) for x in open(tmp_path / "LIVE_READONLY.jsonl")]
    assert code == 0 and s["refused"] > 0 and any("bus read failed" in k for k in s["refusal_reasons"])           # a failed read is a refused observation, not a crash
    assert {("disconnect", "left"), ("disconnect", "right"), ("close", "sdk"), ("close", "left_wrist")} <= set(log) and "released" in rows[-1]
