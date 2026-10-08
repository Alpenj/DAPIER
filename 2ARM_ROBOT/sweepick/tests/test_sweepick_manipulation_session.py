"""PICK02 through the REAL session(), stage(), planner (plan_pick / correct), executors (run / grip) and judgements.
Replaced: the device (a fake bus), the collision answers (stubs: CLEAR for the path they are given) and the camera (an
image model: the block's offset in the wrist image follows the tool's real position from the fake arm's joints, the block
standing `delta` mm along the jaw axis from where the model has it; its width grows as the tool comes down)."""
import json
from pathlib import Path
import numpy as np, mujoco, pytest
from sweepick.integration import sweepick_manipulation_session as p1; from sweepick.control import sweepick_gripper_contact_hold as tg; from sweepick.control import sweepick_joint_command_mapping as rc; from sweepick.control import sweepick_trajectory_profile as tt; from sweepick.control import sweepick_joint_kinematics as kin
from test_sweepick_trajectory_executor import Clock, IN
from test_sweepick_gripper_execution import Jaws

SRC = Path.home() / "sweepick_261007_commission/pick02_plan2/looks/00_observe/SCENE.json"
pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists() or not SRC.exists(), reason="candidate config / stored observation not on this machine")
REF = dict(readable=True, inner_px=[65, 244], midline_px=154.5, gap_px=179); W0, CAM0 = 150.0, 150.0
DESK = dict(class_match=True)


class Arm(Jaws):
    """the fake bus, with two behaviours of the real one: a held joint's reading flickers by a tick, and a Goal_Position write powers the joint"""
    def sync_read(self, reg, normalize=True):
        r = super().sync_read(reg, normalize)
        if reg == "Present_Position" and self.fail.get("flicker") and self.reg["Torque_Enable"]["shoulder_lift"]:
            self.k = getattr(self, "k", 0) + 1; r = dict(r); r["shoulder_lift"] += self.k % 2
        return r
    def write(self, reg, n, v, normalize=True):
        stiff = self.fail.get("stiff") or {}
        if reg == "Goal_Position" and n in stiff and self.reg["Torque_Enable"][n]:      # a loaded joint as seen on 2026-10-08: it does not start until the goal is `dead` ticks away and then rests `rest` ticks short of it
            dead, rest = stiff[n]; p = self.reg["Present_Position"][n]; self.n_goal += 1; self.writes.append((reg, n, v)); self.reg[reg][n] = v
            if abs(v - p) > dead:
                self.reg["Present_Position"][n] = round(v - (rest if v > p else -rest))
            return
        super().write(reg, n, v, normalize)
        if reg == "Goal_Position":
            self.reg["Torque_Enable"][n] = 1


@pytest.fixture(scope="module")
def base():
    sc = json.loads(SRC.read_text()); mp = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")
    plan = p1.plan_pick(sc); assert plan["ok"], plan
    m = p1.model((np.array(sc["object"]["centre_world"]), sc["object"]["yaw_rad"])); tk = rc.q12_to_ticks(mp, np.array(sc["q12"]))
    return dict(scene=sc, mp=mp, plan=plan, model=m, home={n: float(round(v)) for n, v in tk["left"].items()}, right={n: float(round(v)) for n, v in tk["right"].items()})


class World:
    def __init__(self, base, fake, delta=0.0, k=1.0, **over):
        self.b, self.f, self.delta, self.k, self.over = base, fake, delta, k, over
        self.c = np.array(base["scene"]["object"]["centre_world"]); self.yaw = base["scene"]["object"]["yaw_rad"]; self.z0 = None
    def scene(self, tag):
        b = self.b; key = next(x for x in ("01a", "01b", "01_", "02", "03", "04", "05", "06") if tag.startswith(x)); d = mujoco.MjData(b["model"])
        left = {n: float(v) for n, v in self.f.reg["Present_Position"].items()}; q12 = rc.ticks_to_q12(b["mp"], dict(left=left, right=b["right"])); kin.set_q12(b["model"], d, q12); mujoco.mj_forward(b["model"], d)
        pl = p1.pad_placement(b["model"], d, self.c, self.yaw); tool = d.site_xpos[b["model"].site("left_cube_grasp").id].copy()
        self.z0 = tool[2] if self.z0 is None else self.z0
        width = W0 * CAM0 / max(CAM0 - 1000 * (self.z0 - tool[2]), 40.0); off_mm = self.k * (pl["tool_along_jaw_mm"] - self.delta)       # the tool against where the block REALLY is, seen along the jaw axis
        o = dict(visible=True, stable=True, touches_border=False, width_px=width, mm_per_px=40.0 / width, centre_px=154.5 + off_mm * width / 40.0, far_edge_slope_deg=p1.WRIST["aligned_slope_deg"]); o["edges_px"] = [o["centre_px"] - width / 2, o["centre_px"] + width / 2]
        tips = dict(left_px=None, right_px=244) if key in ("01_", "01a", "01b") else dict(left_px=None, right_px=None)
        desk, region = (dict(DESK, centre_xy=list(self.c[:2])), "OCCUPIED") if key in ("01_", "05") else (None, "CLEAR" if key in ("04", "06") else "UNKNOWN (not fully observed)")
        if getattr(self, "true_yaw_deg", None) is not None:                      # the far top edge as the wrist camera shows it: square to the jaws at the recorded slope, turning with the jaws against the block
            ja = np.degrees(np.arctan2(pl["jaw_axis_xy"][1], pl["jaw_axis_xy"][0])); rel = (ja - 90.0 - self.true_yaw_deg + 45.0) % 90.0 - 45.0; o["far_edge_slope_deg"] = p1.WRIST["aligned_slope_deg"] + 0.8 * rel
        if key == "03": self.in_hand = dict(o)
        if key == "04": o = dict(getattr(self, "in_hand", o))                    # lifted: the block goes up with the hand, so it looks as it did when the jaws closed
        if key in ("05", "06"): o = dict(visible=False, stable=False)
        if key in self.over: o, tips, desk, region = self.over[key](o, tips, desk, region)
        return dict(tag=tag, ok=True, q12=[float(v) for v in q12], ticks=dict(left=left, right=b["right"]), wrist=dict(object=o, tips=tips, offset=p1.wrist_offset(o, REF, tips), frames=[dict(n=1), dict(n=2), dict(n=3)]), object=desk, support_region=dict(state=region, valid=not region.startswith("UNKNOWN")),
                    tool_left=[float(v) for v in tool], arms=dict(left=dict(torque={n: 0 for n in p1.ALL})), **p1.tool_axis(b["model"], d))


def cell(tmp_path, monkeypatch, base, delta=0.0, k=1.0, over=None, collision=None, **fail):
    out = tmp_path / "s"; (out / "looks/00").mkdir(parents=True); (out / "looks/00/SCENE.json").write_text(json.dumps(base["scene"]))
    st = dict(ready=True, why=None, scene="looks/00/SCENE.json", stages={}, evidence=[], order=[], plan=json.loads(json.dumps(base["plan"])), home_ticks=base["home"], right_ticks=base["right"], jaw_reference=REF)
    (out / "STATE.json").write_text(json.dumps(st))
    for i in range(4):
        (out / f"scene_block{i}.mjb").write_text("placeholder: the collision check is a stub in this test")
    fake = Arm(dict(base["home"]), **fail); c = Clock(); w = World(base, fake, delta=delta, k=k, **(over or {})); verdict = collision or (lambda tag: "CLEAR")
    kw = dict(open_bus=lambda port: fake, owner=lambda p: "", mapping=base["mp"], mapping_sha="m", sleep=c.sleep, clock=c.now)
    cert = lambda traj_file, out_file, scene_file: dict(identity=json.loads(Path(traj_file).read_text())["identity"], nominal_verdict=verdict(Path(traj_file).name))
    monkeypatch.setattr(p1, "object_check", lambda doc, block, **k_: dict(identity=tt.identity("left", doc["trajectory"], "m") if "profile" in doc else None, nominal_verdict="CLEAR", nominal_steps_checked=doc["trajectory"]["steps"], scene="stub", allowed=["left pad - block", "block - desk"]))
    monkeypatch.setattr(p1, "read_ticks", lambda: dict(left=dict(ticks={n: float(v) for n, v in fake.reg["Present_Position"].items()}, torque=dict(fake.reg["Torque_Enable"]), status=dict(fake.reg["Status"])), right=dict(ticks=base["right"], torque={n: 0 for n in p1.ALL}, status={n: 0 for n in p1.ALL})))
    stage_fn = lambda o, name, execute=False: p1.stage(o, name, execute=execute, cert=cert, run_kw=kw, torque_now=lambda: fake.reg["Torque_Enable"])
    run = lambda **k_: p1.session(out, execute=True, stage_fn=stage_fn, look_fn=lambda o, tag: w.scene(tag), observe_fn=lambda o: S(out), **k_)
    return out, fake, stage_fn, run


S = lambda out: json.loads((out / "STATE.json").read_text())
trail = lambda out, step=None: [r for r in json.loads((out / "SESSION.json").read_text())["trail"] if step is None or r.get("step") == step]
grip_writes = lambda f: [w for w in f.writes if w[1] == "gripper"]
all_off = lambda f: not any(f.reg["Torque_Enable"].values())
FULL = ["descend", "close", "lift", "lower", "open", "retreat", "home"]


def test_plan_uses_the_pad_meshes_and_keeps_the_pads_above_the_top_face_at_the_first_stop(base):
    p = base["plan"]
    assert p["placement_at_grasp"]["tool_minus_pad_low_mm"] > 12                                    # PICK01 / run 5 assumed 5 mm; the bump at the first stop came from that
    assert p["descend_a"]["lowest_pad_vertex_above_top_mm"] > p["joint_residual_moves_pads_mm"]["vertical"] > 0 and min(p["placement_at_grasp"]["gaps_mm"]) > 0
    assert all(p["placement_at_grasp"]["pads"][n]["lowest_above_top_mm"] < 0 for n in p1.PAD_GEOMS) and "descend_a" in p["ticks"]


def test_the_whole_session_with_a_flickering_readback_and_a_second_stop_from_the_measured_error(tmp_path, monkeypatch, base):
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, flicker=True, block=3075)
    r = run(second_stop=True); order = S(out)["order"]
    assert r["result"] == "PICKED_LIFTED_AND_PUT_BACK" and all_off(f), (r["result"], order, trail(out)[-3:])
    assert order[0] == "pregrasp" and order[-7:] == FULL and "descend_a" in order and "descend_b" in order and order.index("descend_a") < order.index("descend_b") < order.index("descend")
    nd = trail(out, "next_descent")[0]; assert nd["next_stop_pads_above_top_mm"] == pytest.approx(nd["measured_vertical_error_mm"] + 1.0) and nd["next_stop_pads_above_top_mm"] < nd["pads_above_top_now_mm"]
    ev = {e["what"]: e for e in S(out)["evidence"]}
    assert ev["ready_to_close"]["placement"].startswith("PADS_BESIDE") and ev["held_after_lift"]["label"] == tg.HELD_CONFIRMED and not S(out).get("refusals")
    moves = [json.loads(p_.read_text()) for p_ in sorted(out.glob("*/run*/move.json"))]
    assert any(isinstance(e, dict) and e.get("start_changed_since_plan") and e["collision"] == "CLEAR" for m_ in moves for e in m_.get("events", []))      # a one-tick change at the start: the path from there was checked again, not refused, not ignored
    first = json.loads((out / "pregrasp/run1/move.json").read_text())
    assert first["drive_enable"]["torque_read_after_the_goal_write"] == {n: 1.0 for n in p1.ARM} and "Goal_Position write" in first["drive_enable"]["note"]


def test_a_real_lateral_error_is_removed_by_the_visual_loop_and_the_block_stays_where_it_was_observed(tmp_path, monkeypatch, base):
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, delta=4.0, k=0.8, block=3075)
    r = run(); st = S(out); v = trail(out, "visual")
    assert r["result"] == "PICKED_LIFTED_AND_PUT_BACK", (r["result"], st["order"], v[-1])
    assert abs(v[0]["lateral_error_mm"]) > 2 and "correction" in v[0] and v[0]["correction"]["tool_move_along_jaw_mm"] > 0 and st["order"][:2] == ["pregrasp", "align"]
    hand = [c for c in st["correction"] if c["along_jaw_mm"]]; assert hand and all(c["kind"].startswith("HAND TARGET OFFSET") for c in hand)
    assert list(p1.state_object(out, st)[0]) == base["scene"]["object"]["centre_world"] and abs(sum(c["along_jaw_mm"] for c in hand) - 4.0) < 1.5
    assert (st["plan"]["tilt_deg"], st["plan"]["jaw"]) == (base["plan"]["tilt_deg"], base["plan"]["jaw"])       # run 6 of 2026-10-08: the re-plan of a correction chose the other jaw direction and turned the wrist a quarter turn instead of shifting 9 mm
    moves = [json.loads(p_.read_text())["trajectory"]["travel_ticks"] for p_ in sorted(out.glob("align*/run*/move.json"))]
    assert moves and all(abs(v) < 200 for m_ in moves for v in m_.values()), moves


def test_wrong_sign_or_no_feature_goes_home_with_the_arm_only(tmp_path, monkeypatch, base):
    out, f, stage_fn, run = cell(tmp_path / "a", monkeypatch, base, delta=8.0, k=-0.8, block=3075)
    r = run(); assert r["result"] == "NOT_ALIGNED_RETURNED_HOME" and S(out)["order"] == ["pregrasp", "align", "home_arm"] and all_off(f) and grip_writes(f) == [], (r["result"], S(out)["order"])
    for i, ov in enumerate((lambda o, t, d, g: (dict(visible=False, stable=False), t, d, g), lambda o, t, d, g: (dict(o, stable=False), t, d, g), lambda o, t, d, g: (o, dict(left_px=None, right_px=None), d, g))):
        out, f, stage_fn, run = cell(tmp_path / f"b{i}", monkeypatch, base, over={"01_": ov}, block=3075)
        r = run(); assert r["result"] == "NOT_ALIGNED_RETURNED_HOME" and S(out)["order"] == ["pregrasp", "home_arm"] and all_off(f)


@pytest.mark.parametrize("case", ["turned", "shifted", "not readable"])
def test_a_block_disturbed_by_the_last_descent_is_not_closed_on_and_the_arm_backs_up_despite_the_flicker(tmp_path, monkeypatch, base, case):
    ov = dict(turned=lambda o, t, d, g: (dict(o, far_edge_slope_deg=-1.0), t, d, g), shifted=lambda o, t, d, g: (dict(o, centre_px=o["centre_px"] + 4.0 / o["mm_per_px"]), t, d, g), **{"not readable": lambda o, t, d, g: (dict(visible=False, stable=False), t, d, g)})[case]
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, over={"02": ov}, flicker=True, block=3075)
    r = run(); order = S(out)["order"]
    assert r["result"] == "PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME", (case, r["result"], order)
    assert order[-3:] == ["descend", "retreat_arm", "home_arm"] and "close" not in order and all_off(f) and grip_writes(f) == []      # run 5 of 2026-10-08: this retreat was refused twice over one tick


def test_a_block_turned_at_the_first_stop_ends_the_descent_there(tmp_path, monkeypatch, base):
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, over={"01a": lambda o, t, d, g: (dict(o, far_edge_slope_deg=-1.0), t, d, g)}, block=3075)
    r = run(); assert r["result"] == "NOT_READY_ABOVE_THE_BLOCK_RETURNED_HOME" and S(out)["order"] == ["pregrasp", "descend_a", "retreat_arm", "home_arm"] and all_off(f), (r["result"], S(out)["order"])


def test_a_request_refused_before_any_write_is_kept_as_a_refusal_and_does_not_close_the_way_home(tmp_path, monkeypatch, base):
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, flicker=True, block=3075, collision=lambda name: "CONTACT" if name.startswith("start") else "CLEAR")     # the re-check from the changed start says the path is not clear
    assert stage_fn(out, "pregrasp", execute=True)["controller_result"] == "HOLDING"                 # the first move starts from rest: no flicker yet
    st = S(out); st["evidence"] = [dict(what="ready_to_descend", ok=True, wrist_object={})]; (out / "STATE.json").write_text(json.dumps(st))
    x = stage_fn(out, "descend_a", execute=True)
    assert x["controller_result"] == "REFUSED" and x["writes"] == dict(Goal_Position=0, Torque_Enable=0) and S(out)["order"] == ["pregrasp"] and len(S(out)["refusals"]) == 1
    ok, why = p1.allowed(S(out), "home_arm"); assert ok                                               # the arm still holds the pre-grasp pose: the arm-only way home is open
    f.fail["flicker"] = False
    assert stage_fn(out, "home_arm", execute=True)["controller_result"] == "RETURNED_RELEASED" and all_off(f)


def test_no_contact_block_not_seen_lost_block_and_unobserved_desk(tmp_path, monkeypatch, base):
    out, f, stage_fn, run = cell(tmp_path / "a", monkeypatch, base)
    r = run(); assert r["result"] == "NO_CONTACT_RETURNED_HOME" and S(out)["order"][-4:] == ["close", "open", "retreat", "home"] and all_off(f)
    out, f, stage_fn, run = cell(tmp_path / "b", monkeypatch, base, over={"03": lambda o, t, d, g: (dict(visible=False, stable=False), t, d, g)}, block=3075)
    r = run(); assert r["result"] == "CONTACT_WITHOUT_THE_BLOCK_SEEN_RETURNED_HOME" and "lift" not in S(out)["stages"] and all_off(f)
    out, f, stage_fn, run = cell(tmp_path / "c", monkeypatch, base, over={"04": lambda o, t, d, g: (o, t, None, "UNKNOWN (not fully observed)")}, block=3075)
    r = run(); ev = next(e for e in S(out)["evidence"] if e["what"] == "held_after_lift")
    assert ev["support_region"]["on_support"] is None and ev["evidence_not_available"] and ev["label"] == tg.HELD_CONFIRMED and (out / "HANDOFF_SNAPSHOT.json").exists()
    out, f, stage_fn, run = cell(tmp_path / "d", monkeypatch, base, over={"04": lambda o, t, d, g: (dict(visible=False, stable=False), t, dict(DESK, centre_xy=base["scene"]["object"]["centre_xy"]), "OCCUPIED")}, block=3075)
    r = run(); assert r["result"] == "LIFT_NOT_CONFIRMED_RETURNED_HOME" and all_off(f)


def test_a_fault_ends_the_session_there_and_a_camera_that_disappears_is_an_observation_failure(tmp_path, monkeypatch, base):
    out, f, stage_fn, run = cell(tmp_path / "a", monkeypatch, base, block=3075, fault_from=30)
    r = run(); assert r["result"] == "STOPPED_IN_CLOSE_HOLDING_NEEDS_A_PERSON" and S(out)["order"][-1] == "close"
    for name in ("open", "retreat", "home", "retreat_arm", "home_arm", "lift", "align"):
        x = stage_fn(out, name, execute=True); assert x["result"] == "NOT_RUN" and "ended as FAULT_HOLDING" in x["why"], name
    out, f, stage_fn, run = cell(tmp_path / "b", monkeypatch, base, block=3075)
    def gone(o, tag):
        raise RuntimeError("left_wrist: /dev/dapier/left_wrist_rgb could not be opened")
    r = p1.session(out, execute=True, stage_fn=stage_fn, look_fn=gone, observe_fn=lambda o: S(out))
    assert r["result"] == "NOT_ALIGNED_RETURNED_HOME" and S(out)["order"] == ["pregrasp", "home_arm"] and all_off(f)


def test_read_only_paths_write_no_goal():
    src = Path(p1.__file__).read_text()
    for fn in ("def read_ticks", "def look(", "def observe(", "def top_frame", "def wrist_frame", "def read_now"):
        body = src[src.index(fn):]; body = body[:body.index("\ndef ", 5)]
        assert "Goal_Position\", " not in body.replace("sync_read(\"Goal_Position\", ", "") and ".write(" not in body, fn


def test_the_jaw_reference_is_confirmed_by_an_unchanged_gripper_reading_when_the_background_hides_the_tips():
    o = dict(visible=True, stable=True, touches_border=False, centre_px=152.5, width_px=165.0, mm_per_px=40 / 165.0); ref = dict(REF, gripper_tick=3361.0)
    over_board = dict(left_px=73, right_px=234)                                   # run 8: dark squares of the printed board next to the tips
    assert not p1.wrist_offset(o, REF, over_board)["reference_confirmed_in_this_frame"]
    r = p1.wrist_offset(o, ref, over_board, gripper_tick=3361.0); assert r["reference_confirmed_in_this_frame"] and "gripper reading unchanged" in r["confirmed_by"][0]
    assert not p1.wrist_offset(o, ref, over_board, gripper_tick=3300.0)["reference_confirmed_in_this_frame"]      # the jaws moved: the reference no longer holds


def test_an_alignment_move_that_settles_short_of_its_joint_target_is_judged_by_the_next_look_not_ended(base):
    """runs 10 and 11 of 2026-10-08: the shoulder stood 16 / 17 ticks short after a small alignment move (tolerance 14) and the session stopped holding"""
    st = dict(stages=dict(pregrasp=dict(controller_result="HOLDING"), align=dict(controller_result="TARGET_NOT_REACHED_HOLDING")), order=["pregrasp", "align"], height="pregrasp", pending_align=False, plan=base["plan"], correction=[dict()])
    assert p1.allowed(st, "descend_a")[0] and p1.allowed(st, "home_arm")[0]
    st["stages"]["align"]["controller_result"] = "NO_PROGRESS_HOLDING"; assert not p1.allowed(st, "descend_a")[0] and not p1.allowed(st, "home_arm")[0]
    st = dict(stages=dict(pregrasp=dict(controller_result="TARGET_NOT_REACHED_HOLDING")), order=["pregrasp"], height=None, plan=base["plan"]); assert not p1.allowed(st, "home_arm")[0]      # only for the alignment move


REAL_ARM = dict(shoulder_lift=(33, 16), elbow_flex=(20, 12), wrist_flex=(8, 5), shoulder_pan=(6, 4), wrist_roll=(6, 4))     # start threshold and resting shortfall per joint, of the size the real traces showed


PLACED = Path.home() / "sweepick_261007_commission/sweepick_261008_pick02_run10/looks/00_observe/SCENE.json"     # the block where it was put for runs 10 and 11 (x 0.21, y +0.05), read by the cameras


def test_the_whole_session_on_an_arm_with_the_real_servo_behaviour(tmp_path, monkeypatch):
    """every real run of 2026-10-08 ended on one more consequence of loaded, proportional-only servos; this runs the whole session on a fake arm that has them, at the placement in use"""
    sc = json.loads(PLACED.read_text()); mp = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")
    plan = p1.plan_pick(sc); assert plan["ok"], plan
    tk = rc.q12_to_ticks(mp, np.array(sc["q12"]))
    placed = dict(scene=sc, mp=mp, plan=plan, model=p1.model((np.array(sc["object"]["centre_world"]), sc["object"]["yaw_rad"])), home={n: float(round(v)) for n, v in tk["left"].items()}, right={n: float(round(v)) for n, v in tk["right"].items()})
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, placed, delta=8.0, k=0.9, flicker=True, block=3075, stiff=REAL_ARM)
    r = run(); st = S(out); got = [(n, st["stages"][n]["controller_result"]) for n in dict.fromkeys(st["order"])]
    assert [n for n, _ in got] == ["pregrasp", "align", "descend_a", "descend", "descend_settle", "close", "lift", "lower", "open", "retreat", "home"], (r["result"], got)
    assert dict(got)["descend"] == "TARGET_NOT_REACHED_HOLDING" and dict(got)["descend_settle"] == "HOLDING" and dict(got)["close"] == "GRIP_CONTACT_HOLDING" and dict(got)["lift"] == "HOLDING", got
    assert (out / "HANDOFF_SNAPSHOT.json").exists()
    assert dict(got)["home"] in ("RETURNED_RELEASED", "RETURN_NOT_CONFIRMED_HOLDING"), got       # this fake leaves every joint short at the folded pose too; the real arm has released there in each run that reached it

def test_a_wrist_camera_that_fails_to_open_once_is_read_again(tmp_path, monkeypatch):
    """run 12b of 2026-10-08: one refused open of the wrist camera at the first stop ended the session although the next open worked"""
    monkeypatch.setattr(p1.time, "sleep", lambda s: None); n = []
    def once(path, frames):
        n.append(1)
        if len(n) == 1: raise RuntimeError("left_wrist: could not be opened")
        return dict(readable=True, visible=True)
    r = p1.wrist_frame(tmp_path / "w.jpg", once=once); assert r["readable"] and [t["camera_failed"] for t in r["camera_tries"]] == [True, False]
    n.clear(); r = p1.wrist_frame(tmp_path / "w.jpg", once=lambda *a: (n.append(1), dict(readable=False, why="the frames disagree"))[1]); assert len(n) == 1 and not r["readable"]      # an image that was delivered and is not usable is not asked again
    def dead(path, frames): n.append(1); raise RuntimeError("gone")
    n.clear(); r = p1.wrist_frame(tmp_path / "w.jpg", once=dead); assert len(n) == 3 and not r["readable"] and r["camera_failed"]


def test_the_wrist_is_turned_to_the_block_as_the_wrist_image_shows_it(tmp_path, monkeypatch):
    """run 13 of 2026-10-08: the top depth gave the block's yaw 10 deg wrong, the wrist followed it and the pads met the block's corners at the first stop"""
    sc = json.loads(PLACED.read_text()); true = float(np.degrees(sc["object"]["yaw_rad"])); sc["object"]["yaw_rad"] = float(np.radians(true - 13.0))
    mp = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json"); plan = p1.plan_pick(sc); assert plan["ok"], plan
    tk = rc.q12_to_ticks(mp, np.array(sc["q12"]))
    b = dict(scene=sc, mp=mp, plan=plan, model=p1.model((np.array(sc["object"]["centre_world"]), sc["object"]["yaw_rad"])), home={n: float(round(v)) for n, v in tk["left"].items()}, right={n: float(round(v)) for n, v in tk["right"].items()})
    monkeypatch.setattr(World, "true_yaw_deg", true, raising=False)
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, b, delta=8.0, k=0.9, flicker=True, block=3075)
    r = run(); st = S(out); sq = [x["square"] for x in trail(out) if x.get("square")]
    assert abs(sq[0]["error_deg"]) > 8 and "correction" in sq[0] and abs(sq[-1]["error_deg"]) <= p1.WRIST["slope_tolerance_deg"], [(q["edge_slope_deg"], q["jaw_axis_deg"]) for q in sq]
    assert abs(np.degrees(st["object_updates"][-1]["yaw_rad"]) - true) < 4.0 and "wrist" in st["object_updates"][0]["kind"].lower()
    assert r["result"] == "PICKED_LIFTED_AND_PUT_BACK", (r["result"], [(n, st["stages"][n]["controller_result"]) for n in dict.fromkeys(st["order"])])
