"""SYNTHETIC fixture for the full-chain tests: the real session / stage / executor / RECEIVE session / teacher run; only the two buses, the cameras and the field records are fakes. Nothing made with it is a real result."""
import json, time
from pathlib import Path
import numpy as np
import cv2
from sweepick.manipulation import sweepick_bimanual_handover as fc; from sweepick.integration import sweepick_manipulation_session as p1; from sweepick.control import sweepick_trajectory_executor as mv
import test_sweepick_manipulation_session as T
from test_sweepick_episode_recorder import cell, episode, rows
from test_sweepick_trajectory_executor import Clock

IMG = cv2.imread(str(T.PLACED.parent.parent / "01_pregrasp_1/left_wrist.jpg"))


def refs(tmp, *, right_jaw=True, right_wrist=True, place=True, support=True):
    """SYNTHETIC field records for the wiring test (marked as such in their evidence)."""
    tmp = Path(tmp); tmp.mkdir(parents=True, exist_ok=True); out = {}
    docs = dict(right_jaw=dict(json.loads(mv.JAW_PROFILE.read_text()), name="SYNTHETIC_RIGHT_FIXTURE", source_kind="REAL", device_identity="SYNTHETIC fixture", measurement_evidence="SYNTHETIC TEST FIXTURE: not a measurement"),
                right_wrist=dict(source_kind="REAL", measurement_evidence="SYNTHETIC TEST FIXTURE", block_between_jaws_centre_px=float(p1.wrist_object(IMG)["centre_px"]), centre_tolerance_px=20.0),
                support=dict(source_kind="REAL", measurement_evidence="SYNTHETIC TEST FIXTURE", right_lift_m=0.005, load_joints=["shoulder_lift"], drop_raw=40.0),
                place=dict(source_kind="REAL", measurement_evidence="SYNTHETIC TEST FIXTURE", field_confirmed=True, name="place_candidate", centre_xy_m=[0.21, -0.06], surface_z_m=p1.BOARD_Z_IN_WORLD, radius_m=0.04, approach_height_m=0.05))
    want = dict(right_jaw=right_jaw, right_wrist=right_wrist, place=place, support=support)
    for n, (path, what, blocks) in fc.FIELD.items():
        f = tmp / path.name; ok = want[n]
        if ok: f.write_text(json.dumps(docs[n]))
        out[n] = dict(file=str(f), present=ok, usable=ok, doc=docs[n] if ok else None, what=what, blocks=blocks)
    return out


def chain(tmp_path, monkeypatch, *, field=None, check="real", seen=True, right_block=2627, scope="full", right_lift_m=None, stale=(), seen_shift=(0.0, 0.0, 0.0), seen_centre_m=0.005, **cellkw):
    if not getattr(T.World, "_full_look", False):                                # the fake look gets the fields a real look has (stamps, both arms, camera frame)
        raw_scene = T.World.scene
        def scene(self, tag):
            s = raw_scene(self, tag); s["arms"] = dict(s.get("arms") or {}); s["arms"].setdefault("left", dict(ticks=s["ticks"]["left"], torque={})); s["arms"]["right"] = dict(s["arms"].get("right") or {}, ticks=s["ticks"]["right"])
            s.update(joint_reads=dict(static=True, joints_host_s=time.time()), stamps=dict(sdk_color_stamp_s=time.time(), sdk_depth_stamp_s=time.time(), recv_monotonic_s=time.monotonic()), T_world_camera=np.eye(4).tolist(), K=[600, 0, 320, 0, 600, 240, 0, 0, 1]); return s
        monkeypatch.setattr(T.World, "scene", scene); monkeypatch.setattr(T.World, "_full_look", True, raising=False)
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, **cellkw); st = T.S(out); mp = T.rc.load_mapping(T.IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", T.IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")
    right = T.Arm({n: float(v) for n, v in st["right_ticks"].items()}, block=right_block); c = Clock(); c.t = time.time()                    # like the real executor: wall time, ahead of the monotonic stamps of the observations
    for reg in ("Min_Position_Limit", "Max_Position_Limit"):
        if reg in right.reg: right.reg[reg] = {n: (mp["right"][n]["lo"] if reg.startswith("Min") else mp["right"][n]["hi"]) for n in p1.ALL}
    kw = dict(open_bus=lambda port: f if "left" in port else right, owner=lambda p: "", mapping=mp, mapping_sha="m", sleep=c.sleep, clock=c.now)
    def read():
        g = lambda b, r: {n: float(b.reg[r][n]) for n in p1.ALL}
        return dict(ticks=dict(left=g(f, "Present_Position"), right=g(right, "Present_Position")), goal=dict(left=g(f, "Goal_Position") if f.reg.get("Goal_Position") else g(f, "Present_Position"), right={n: float(right.reg.get("Goal_Position", {}).get(n, right.reg["Present_Position"][n])) for n in p1.ALL}),
                    torque=dict(left=g(f, "Torque_Enable"), right=g(right, "Torque_Enable")), status=dict(left=g(f, "Status"), right=g(right, "Status")), load=dict(left={n: (100.0 if "load_transfer" not in owner.done else 20.0) if n == "shoulder_lift" else 0.0 for n in p1.ALL}, right={n: 0.0 for n in p1.ALL}))     # the fake left shoulder is unloaded once the right hand has lifted
    k = [0]
    def frames():
        k[0] += 1; mk = lambda extra=None, cam=None: dict(n=k[0] if cam not in stale else 1, recv_monotonic_s=time.monotonic() - (5.0 if cam in stale else 0.0), frame=IMG, cadence_limit_s=0.2, **(extra or {}))      # a camera in `stale` stopped delivering 5 s ago
        return dict(left_wrist=dict(mk(cam="left_wrist"), frame=IMG if "left_retreat" not in owner.done else np.full_like(IMG, 255)), right_wrist=mk(cam="right_wrist"), top=mk(dict(depth=np.ones((4, 4), np.uint16)), cam="top"))
    placed = []
    def look(o, tag):
        if tag.startswith(("05_place", "06_area")):
            return dict(tag=tag, ok=True, support_region_at=lambda region: dict(valid=True, state="OCCUPIED" if region.region_id != "source" else "CLEAR"), arms={}, wrist={})
        raise AssertionError(f'unexpected look {tag}')
    ep = episode(out); monkeypatch.setattr(p1, "EPISODE", ep)
    handoff, owner = fc.build(out, ep, run_kw=kw, read=read, frames=frames, look=look, refs=field if field is not None else refs(tmp_path / "field"), clock=time.monotonic, sleep=lambda s: None, scope=scope, right_lift_m=right_lift_m,
                              check=(None if check == "real" else (lambda doc, side, raw: dict(identity=__import__("sweepick.control.sweepick_trajectory_profile", fromlist=["*"]).identity(doc["side"], doc["trajectory"], "m"), nominal_verdict=check, nominal_steps_checked=doc["trajectory"]["steps"]))))
    if seen:
        def seen_fn(depth, K, Twc, predicted, **k_):
            P = np.array(predicted, float); P[:3, 3] += np.array(seen_shift, float)                      # the synthetic 'observation' puts the block this far from where the estimate had it
            return dict(object_pose_model=P.tolist(), points=300, moved_from_estimate_m=list(map(float, seen_shift)), components=dict(centre_xy=dict(cls="OBSERVED"), height=dict(cls="OBSERVED"), yaw=dict(cls="ASSUMED")), object_pose_uncertainty=dict(measured=True, centre_m=seen_centre_m))
        owner.held_seen_fn = seen_fn
    return dict(out=out, left=f, right=right, run=run, handoff=handoff, owner=owner, ep=ep, kw=kw)
