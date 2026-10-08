"""The continuous record, on synthetic readers and the fake arm of the PICK02 session tests. Wiring only: nothing here is a real episode."""
import json, threading, time
from pathlib import Path

import numpy as np
import pytest

from sweepick.recording import sweepick_episode_recorder as se
from sweepick.integration import sweepick_manipulation_session as p1
import test_sweepick_manipulation_session as T


class Cam:
    """a reader that makes a new small frame every `dt` (synthetic)"""
    def __init__(self, name, dt=0.01, dead_after=None):
        self.name, self.n, self.dt, self.t0, self.dead_after, self.closed = name, 0, dt, time.monotonic(), dead_after, False

    def get(self):
        n = int((time.monotonic() - self.t0) / self.dt) + 1
        if self.dead_after is not None: n = min(n, self.dead_after)
        return dict(n=n, recv_monotonic_s=self.t0 + (n - 1) * self.dt, frame=np.full((4, 4, 3), n % 255, np.uint8)), None

    def close(self): self.closed = True


class Top(Cam):
    count, error, K = 0, None, [1, 0, 0, 0, 1, 0, 0, 0, 1]
    def get(self):
        f, _ = Cam.get(self); return dict(n=f["n"], bgr=f["frame"], zd=np.ones((4, 4), np.uint16), sdk_color_stamp_s=1.0 * f["n"], sdk_depth_stamp_s=1.0 * f["n"], helper_receipt_monotonic_s=f["recv_monotonic_s"], recv_monotonic_s=f["recv_monotonic_s"])


class Bus:
    opened = []
    def __init__(self, side): self.side = side; Bus.opened.append(side); self.live = True
    def sync_read(self, name, normalize=False):
        assert self.live; return {n: 2048.0 if name == "Present_Position" else 0 for n in ("shoulder_pan", "gripper")}
    def disconnect(self): self.live = False; Bus.opened.remove(self.side)


def episode(tmp_path, **k):
    kw = dict(wrists=dict(left_wrist=Cam("left_wrist"), right_wrist=Cam("right_wrist")), top=Top("top"), open_arm=Bus, source_kind="SYNTHETIC", arm_period_s=0.01, save_image=lambda path, im: True); kw.update(k)
    return se.Episode(tmp_path, **kw)


def rows(tmp_path, name): return [json.loads(l) for l in (tmp_path / f"episode/{name}.jsonl").read_text().splitlines()]


def test_three_views_and_both_arms_under_one_session_id(tmp_path):
    ep = episode(tmp_path); ep.mark("pregrasp", skill="START", owner="executor"); time.sleep(0.25); doc = ep.close(task_outcome="PARTIAL_OR_DIAGNOSTIC: test")
    assert doc["data_quality"]["three_views_recorded"] and all(doc["cameras"][c]["frames"] > 5 for c in ("top", "left_wrist", "right_wrist")) and doc["joints"]["left"] > 3 and doc["joints"]["right"] > 3
    fr = rows(tmp_path, "frames"); assert {f["camera"] for f in fr} == {"top", "left_wrist", "right_wrist"} and all(f["step"] in ("start", "pregrasp") for f in fr)
    top = [f for f in fr if f["camera"] == "top"]; assert all("sdk_color_stamp_s" in f and "recv_monotonic_s" in f and "helper_receipt_monotonic_s" in f for f in top)      # the three clocks stay apart
    assert sum(1 for f in top if f.get("depth_file")) < len(top) and doc["top_depth_saved_every_n_framesets"] == 5
    assert doc["data_quality"]["training_eligibility"].startswith("NOT_ELIGIBLE") and doc["source_kind"] == "SYNTHETIC"     # a synthetic record is never passed on as a real episode
    assert all(j["kind"] == "MEASURED" and "commanded" not in j and "goal" not in j for j in rows(tmp_path, "joints"))       # the recorder's reads are measurements only


def test_the_recorder_lets_go_of_an_arm_for_a_stage_and_takes_it_back(tmp_path):
    ep = episode(tmp_path); time.sleep(0.05); assert "left" in Bus.opened
    with ep.lease("left"):
        assert "left" not in Bus.opened; n0 = ep.counts.get("joints_left", (0,))[0]; r0 = ep.counts.get("joints_right", (0,))[0]
        with ep.lease("left", "right"):                                        # a look's read inside the stage's lease
            assert "right" not in Bus.opened
        time.sleep(0.08); assert "left" not in Bus.opened and ep.counts.get("joints_left", (0,))[0] == n0 and ep.counts["joints_right"][0] > r0      # the inner lease's end did not end the stage's; the other arm is read on
    time.sleep(0.08); assert ep.counts["joints_left"][0] > n0; ep.close()
    assert Bus.opened == []


def test_a_camera_that_stops_is_a_gap_not_repeated_frames(tmp_path):
    ep = episode(tmp_path, wrists=dict(left_wrist=Cam("left_wrist"), right_wrist=Cam("right_wrist", dead_after=3))); time.sleep(0.2); doc = ep.close()
    assert doc["cameras"]["right_wrist"]["frames"] == 3 and doc["cameras"]["left_wrist"]["frames"] > 10          # nothing is repeated to fill the stream
    ep2 = episode(tmp_path / "b", wrists=dict(left_wrist=Cam("left_wrist"))); time.sleep(0.05); d2 = ep2.close(); assert d2["data_quality"]["missing_views"] == ["right_wrist"] and not d2["data_quality"]["three_views_recorded"]


def test_a_failing_save_does_not_stop_the_record_or_block(tmp_path):
    def bad(path, im): raise OSError("disk")
    ep = episode(tmp_path, save_image=bad); t0 = time.monotonic()
    with ep.lease("left"): pass
    assert time.monotonic() - t0 < 1.0; time.sleep(0.05); doc = ep.close(); assert doc["data_quality"]["recorder_errors"] > 0 and doc["cameras"]["top"]["frames"] == 0


def cell(tmp_path, monkeypatch, **k):
    sc = json.loads(T.PLACED.read_text()); mp = T.rc.load_mapping(T.IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", T.IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json"); plan = p1.plan_pick(sc); tk = T.rc.q12_to_ticks(mp, np.array(sc["q12"]))
    b = dict(scene=sc, mp=mp, plan=plan, model=p1.model((np.array(sc["object"]["centre_world"]), sc["object"]["yaw_rad"])), home={n: float(round(v)) for n, v in tk["left"].items()}, right={n: float(round(v)) for n, v in tk["right"].items()})
    return T.cell(tmp_path, monkeypatch, b, delta=8.0, k=0.9, flicker=True, block=3075, **k)


def test_the_real_session_with_the_recorder_on(tmp_path, monkeypatch):
    """the PICK02 session (real session / stage / planner / executor on the fake arm) with the recorder: same stages as without it, one record with every stage's rows"""
    out, f, stage_fn, run = cell(tmp_path, monkeypatch); ep = episode(out); monkeypatch.setattr(p1, "EPISODE", ep)
    r = run(); doc = ep.close(task_outcome="PARTIAL_OR_DIAGNOSTIC: " + r["result"]); st = T.S(out)
    assert r["result"] == "PICKED_LIFTED_AND_PUT_BACK" and [n for n in dict.fromkeys(st["order"])] == ["pregrasp", "align", "descend_a", "descend", "close", "lift", "lower", "open", "retreat", "home"]
    assert [s["stage"] for s in doc["stages"]] == st["order"] and all(Path(s["folder"]).exists() for s in doc["stages"])
    ev = rows(out, "events"); ph = [e for e in ev if e["kind"] == "phase"]; assert [e["skill"] for e in ph if e["step"] == "lift"] == ["START"] and [e["skill"] for e in ph if e["step"] == "lower"] == ["PUT_BACK"]
    al = se.alignment(out); lift = next(s for s in al["stages"] if s["stage"] == "lift"); assert lift["rows"] > 10 and lift["has_command"] and set(lift["cameras"]) == {"top", "left_wrist", "right_wrist"}
    assert doc["task_outcome"].startswith("PARTIAL") and r.get("put_back_evidence") is not None


def test_the_looks_use_the_recorders_readers_and_only_frames_received_after_the_call(tmp_path, monkeypatch):
    """with the recorder on, a look opens no camera of its own: it takes the next frames of the session's readers"""
    import cv2
    img = cv2.imread(str(T.PLACED.parent.parent / "01_pregrasp_1/left_wrist.jpg")); assert img is not None
    class Wrist(Cam):
        def get(self):
            f, _ = Cam.get(self); return dict(f, frame=img), None
    w = Wrist("left_wrist", dt=0.03); ep = episode(tmp_path, wrists=dict(left_wrist=w)); monkeypatch.setattr(p1, "EPISODE", ep)
    from sweepick.integration import sweepick_observation_session as ll
    monkeypatch.setattr(ll, "WristCam", lambda *a, **k: (_ for _ in ()).throw(AssertionError("a second reader was opened")), raising=True); monkeypatch.setattr(ll, "SdkTop", lambda *a, **k: (_ for _ in ()).throw(AssertionError("a second SDK reader was opened")), raising=True)
    n0 = w.get()[0]["n"]; r = p1.wrist_frame(tmp_path / "w.jpg"); assert r.get("camera_tries")[-1]["camera_failed"] is False and [f["n"] for f in r["frames"]][0] > n0 and not w.closed
    t0 = ep.top.get()["n"]; top = p1.top_frame(tmp_path / "sdk", wait_s=5.0, depth_frames=6); assert top["depth_frames"] == 6 and "resident_reader" in top["close"] and top["stamps"]["sdk_color_stamp_s"] > t0
    ep.close()
