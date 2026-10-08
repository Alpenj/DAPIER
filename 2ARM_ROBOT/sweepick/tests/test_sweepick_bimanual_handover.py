"""The FULL-CHAIN mode through its real entry: sweepick_pick01.session -> RECEIVE FullEpisodeHandoff -> TaskManager ->
ReceiveSession -> sweepick_full_chain.Owner -> sweepick_move_a.run / grip, with the teacher's planners. Only the two motor
buses, the cameras and the field records are fakes (sweepick_chain_fixture). Wiring and state transfer; not a hand-over."""
import json
from pathlib import Path

import numpy as np
import pytest

from sweepick.recording import sweepick_episode_recorder as se
from sweepick.manipulation import sweepick_bimanual_handover as fc
from sweepick.integration import sweepick_manipulation_session as p1
import test_sweepick_manipulation_session as T
from sweepick_261008_manipulation_chain_fixture import chain, refs

R14 = Path.home() / "sweepick_261007_commission/sweepick_261008_pick02_run14"
pytestmark = pytest.mark.skipif(not fc.CODEX.exists(), reason="RECEIVE receive modules are not on this machine")
ran = lambda c: [n["name"] + ":" + n["side"] for n in c["owner"].notes if n.get("kind") == "segment" and n.get("wrote", True)]
after_lift = lambda out: T.S(out)["order"][T.S(out)["order"].index("lift") + 1:]


@pytest.mark.parametrize("check", ["CLEAR", "real"])
def test_the_whole_chain_runs_through_the_existing_executor(tmp_path, monkeypatch, check):
    """with every field record there (SYNTHETIC): one state machine (RECEIVE task manager) and one writer (the executor) from the lift to the stow"""
    c = chain(tmp_path, monkeypatch, check=check); r = c["run"](handoff_fn=c["handoff"]); h = r["handoff"]; doc = c["ep"].close(task_outcome=r["result"])
    assert r["result"] == "FULL_CHAIN_DONE", (r["result"], r["handoff"].get("why"), r["handoff"].get("missing")[-2:], [n for n in c["owner"].notes if n["kind"] in ("held_pose_not_observed", "owner_stop")][:2])
    assert h["object_owner"] == "PLACED" and after_lift(c["out"]) == []                       # the old put-back stages were not run
    assert [s["stage"] for s in h["stages"] if s["status"] == "SUCCEEDED"] == ["COORDINATE", "RELEASE", "PLACE", "ARM_STOW"]
    assert ran(c) == ["right_take_open:right", "right_open_gripper:right", "carry_right:right", "carry_left:left", "right_approach:right", "right_insert:right", "load_transfer:right", "left_gripper:left", "left_retreat:left",
                      "place_above:right", "place_down:right", "place_open:right", "place_retreat:right", "stow:right", "stow:left"]
    assert any(n["kind"] == "right_close" and n["result"] == "GRIP_CONTACT_HOLDING" for n in c["owner"].notes)                 # the right close was the Jaw's, through grip
    assert T.all_off(c["left"]) and not any(c["right"].reg["Torque_Enable"].values())
    # ONE manager made the final area check and the request; the session reports it and did not look again
    assert r["clean_resume_request"] is True and h["clean_resume_request_emitted"] is True and r["cleaning_succeeded"] is False and not any(t.get("gate") == "area_after_the_task" for t in r["trail"])
    assert [a["tag"] for a in c["handoff"].area.records] == ["05_place", "06_area"]
    # one episode: the START stages and the runs after the lift, each with the executor's rows; each cycle's observation -> proposal -> run
    rec = se.chain_record(c["out"]); assert rec["session_id"] == c["ep"].id and not rec["missing_links"], rec["missing_links"]
    assert [s["stage"] for s in rec["start_stages"]][:2] == ["pregrasp", "align"] and all(s["rows"] and s["measured"] and s["goal_sent"] for s in rec["start_stages"] + rec["after_lift_stages"])
    assert len(rec["after_lift_stages"]) == len(ran(c)) and rec["cycles_that_sent"] >= 8 and rec["acknowledgements"] >= 3 and rec["cycles_with_proposal"] >= rec["cycles_that_sent"]
    assert doc["data_quality"]["training_eligibility"].startswith("NOT_ELIGIBLE")                                                   # synthetic readers: never an episode to learn from


def test_without_the_field_records_the_session_is_not_started_and_only_field_items_are_named(tmp_path, monkeypatch):
    c = chain(tmp_path, monkeypatch, field=fc.field_refs() if not any(p.exists() for p, *_ in fc.FIELD.values()) else refs(tmp_path / "none", right_jaw=False, right_wrist=False, place=False, support=False))
    r = c["run"](handoff_fn=c["handoff"]); assert r["result"] == "NOT_STARTED_FULL_CHAIN_NOT_READY" and T.S(c["out"]).get("order", []) == [] and T.all_off(c["left"]) and ran(c) == []
    assert len(r["missing"]) == 4 and all(m.startswith("FIELD ") for m in r["missing"]), r["missing"]                               # no code reference is missing: every function is bound


@pytest.mark.parametrize("lacking", ["right_jaw", "right_wrist", "support", "place"])
def test_each_missing_field_record_keeps_the_session_from_starting(tmp_path, monkeypatch, lacking):
    c = chain(tmp_path, monkeypatch, field=refs(tmp_path / "f", **{lacking: False})); r = c["run"](handoff_fn=c["handoff"])
    assert r["result"] == "NOT_STARTED_FULL_CHAIN_NOT_READY" and len(r["missing"]) == 1 and lacking in r["missing"][0] or fc.FIELD[lacking][0].name in r["missing"][0]


def test_the_right_insert_waits_for_the_block_observed_in_the_hand(tmp_path, monkeypatch):
    """the coarse estimate carries and approaches; the right jaws are not put around the block on it"""
    c = chain(tmp_path, monkeypatch, check="CLEAR", seen=False); r = c["run"](handoff_fn=c["handoff"]); o = c["owner"]
    assert r["result"].startswith("HANDOFF_") and r["result"].endswith("_HOLDING_NEEDS_A_PERSON") and "RIGHT_APPROACH" in o.done and "RIGHT_INSERT" not in o.done and not o.right_closed
    assert any(n["kind"] == "held_pose_not_observed" for n in o.notes) and after_lift(c["out"]) == [] and c["left"].reg["Torque_Enable"]["gripper"] == 1      # still holding; nothing lowered or opened by itself
    h = json.loads((c["out"] / "HELD_POSE.json").read_text()); assert h["object_pose_model"] is None and h["status"] == "MISSING" and h["coarse_preview_pose"] is not None
    assert {k: v["cls"] for k, v in h["components"].items()} == dict(along_jaw_axis="ASSUMED", across_jaw_axis="ESTIMATED", height="ESTIMATED", yaw="ASSUMED", tilt="ASSUMED", slip_after_close="UNIDENTIFIED")
    assert set(h["missing"]) == {"held object height/tool offset", "held across-pad-width offset", "held orientation in model frame"}      # RECEIVE own wording of what is not identified


def test_without_the_load_going_over_the_left_hand_is_not_opened(tmp_path, monkeypatch):
    c = chain(tmp_path, monkeypatch, check="CLEAR"); o = c["owner"]; read0 = o._read
    def read():
        r = read0(); r["load"]["left"]["shoulder_lift"] = 100.0; return r                                 # the left shoulder stays loaded after the right-hand lift
    o._read = read; o.idle_limit_s = 1.0; hold = None
    r = c["run"](handoff_fn=c["handoff"])
    assert r["result"] == "HANDOFF_FAILED_HOLDING_NEEDS_A_PERSON" and "nothing changed" in r["handoff"]["why"] and "load_transfer" in o.done and "left_retreat" not in o.done
    assert not any(n.get("name") == "left_gripper" for n in o.notes if n["kind"] == "segment") and r["handoff"]["object_owner"] in ("LEFT", "UNKNOWN", "BOTH") and after_lift(c["out"]) == []


def test_a_stop_during_the_hand_over_ends_it_there(tmp_path, monkeypatch):
    c = chain(tmp_path, monkeypatch, check="CLEAR"); o = c["owner"]; seg0 = o.segment
    def seg(name, side, **k):
        if name == "right_approach": Path(str(c["out"]) + ".STOP").write_text("stop")                 # the operator's stop file, read by the executor
        return seg0(name, side, **k)
    o.segment = seg; r = c["run"](handoff_fn=c["handoff"]); Path(str(c["out"]) + ".STOP").unlink()
    assert r["result"] == "HANDOFF_FAILED_HOLDING_NEEDS_A_PERSON" and o.stopped is not None and "RIGHT_INSERT" not in o.done and after_lift(c["out"]) == []
    n = len(ran(c)); assert ran(c)[-1].startswith("right_approach") and c["left"].reg["Torque_Enable"]["shoulder_lift"] == 1 and len(ran(c)) == n        # nothing was sent after the stop; the arms hold


def test_a_path_that_touches_something_is_not_run(tmp_path, monkeypatch):
    c = chain(tmp_path, monkeypatch, check="CONTACT"); r = c["run"](handoff_fn=c["handoff"])
    assert r["result"].endswith("_HOLDING_NEEDS_A_PERSON") and ran(c) == [] and "touches something" in str(r["handoff"]["why"]) + str(c["owner"].stopped) + str(r["handoff"].get("preparation"))


def test_a_record_that_cannot_be_written_does_not_change_what_the_robot_did(tmp_path, monkeypatch):
    """events.jsonl fails from the first stage on: every stage still returns its result, the chain still ends, and the loss is in the data quality"""
    c = chain(tmp_path, monkeypatch, check="CLEAR"); ep = c["ep"]
    class Dead:
        def write(self, *_): raise OSError("disk full")
        def close(self): pass
    ep.files["events"] = Dead(); r = c["run"](handoff_fn=c["handoff"]); doc = ep.close(task_outcome=r["result"])
    assert r["result"] == "FULL_CHAIN_DONE" and T.all_off(c["left"]) and doc["data_quality"]["events_not_recorded"] > 10 and doc["data_quality"]["training_eligibility"].startswith("NOT_ELIGIBLE")
    assert [s["stage"] for s in doc["stages"]][:3] == ["pregrasp", "align", "descend_a"] and all(Path(s["folder"]).exists() for s in doc["stages"])      # the control traces are separate files and are there


def test_a_bus_fault_is_not_taken_for_a_recording_error(tmp_path, monkeypatch):
    c = chain(tmp_path, monkeypatch, check="CLEAR"); o = c["owner"]; seg0 = o.segment
    def seg(name, side, **k):
        if name == "carry_left": c["left"].reg["Status"]["elbow_flex"] = 32                               # a servo fault on the left arm
        return seg0(name, side, **k)
    o.segment = seg; r = c["run"](handoff_fn=c["handoff"])
    assert r["result"] == "HANDOFF_FAILED_HOLDING_NEEDS_A_PERSON" and o.stopped is not None and "CARRY_LEFT" not in o.done and after_lift(c["out"]) == []


@pytest.mark.skipif(not R14.exists(), reason="stored run 14 is not on this machine")
def test_stored_run14_coarse_estimate_gives_a_carry_and_approach_preview_inside_the_real_ranges():
    st = json.loads((R14 / "STATE.json").read_text()); sc = json.loads((R14 / "looks/04_lifted/SCENE.json").read_text()); snap = json.loads((R14 / "HANDOFF_SNAPSHOT.json").read_text()); mp = p1.mapping(); cx = fc.codex()
    q_close = p1.rc.ticks_to_q12(mp, dict(left={n: float(v) for n, v in st["stages"]["close"]["end"]["ticks"].items()}, right=sc["arms"]["right"]["ticks"])); c, yaw = p1.state_object(R14, st)
    h = fc.held_pose(q_close, sc["q12"], c, yaw, model=p1.model(), pad_geoms=p1.PAD_GEOMS, source="run 14"); assert h["object_pose_model"] is None and np.array(h["coarse_preview_pose"])[2, 3] > 0.05
    import hashlib
    scene = fc.scene_bindings(p1.model(), sc, p1.BOARD_Z_IN_WORLD, model_sha=hashlib.sha256(p1.SCENE.read_bytes()).hexdigest(), source_sha=hashlib.sha256((fc.RUNTIME / "source/production_source_g000/integration_scenes.py").read_bytes()).hexdigest())
    assert np.allclose(np.array(scene["T_model_table"])[:3, 3], [0.2955, 0.0, p1.BOARD_Z_IN_WORLD], atol=1e-3)
    b = dict(source_kind="REAL", object_pose_model=h["coarse_preview_pose"], object_pose_frame=scene["frame"], object_pose_source="COARSE PREVIEW ESTIMATE", object_pose_uncertainty=dict(h["object_pose_uncertainty"], components=h["components"]), q_mapping_verified=False, geometry_verified=False, right_jaw_profile=None,
             scene={k: v for k, v in scene.items() if k not in ("provenance", "geometry_verified")})
    state = cx["receive_real_adapter"].adapt_snapshot(snap, b); plan = cx["receive_plan"].plan_receive(state); assert state["status"] == "INPUT_VALID" and plan["status"] == "PLAN_PREVIEW" and plan["mj_step_calls"] == 0
    for w in plan["waypoints"]:
        tk = p1.rc.q12_to_ticks(mp, np.array(w["q12"])); assert all(mp[s][n]["lo"] <= v <= mp[s][n]["hi"] for s in ("left", "right") for n, v in tk[s].items()), w["label"]


def test_put_back_evidence_revision_on_run14_and_on_missing_observations():
    if not R14.exists(): pytest.skip("stored run 14 is not on this machine")
    st = json.loads((R14 / "STATE.json").read_text()); sc = json.loads((R14 / "looks/05_home/SCENE.json").read_text()); e = p1.put_back_evidence(st, sc)
    assert e["verdict"].startswith("ON_THE_DESK_AT_THE_PLACE") and "size class NOT confirmed" in e["verdict"] and e["size_class"] is False
    assert p1.put_back_evidence(st, dict(sc, support_region=dict(sc["support_region"], valid=False)))["verdict"] == "UNKNOWN" and p1.put_back_evidence(st, dict(sc, object=None))["verdict"] == "UNKNOWN"
    st2 = json.loads(json.dumps(st)); st2["stages"]["open"]["controller_result"] = "STOPPED_HOLDING"; assert p1.put_back_evidence(st2, sc)["verdict"] == "NOT_CONFIRMED"


@pytest.mark.skipif(not R14.exists(), reason="stored run 14 is not on this machine")
def test_the_default_binding_of_the_real_entry_lacks_only_field_records(tmp_path):
    """what `session --record --full-chain` builds with no argument of a test: RECEIVE FullEpisodeHandoff with this process's owner; before any device is touched it names the field records and no missing function"""
    from test_sweepick_episode_recorder import episode
    ep = episode(tmp_path); h, o = fc.build(tmp_path, ep); st = json.loads((R14 / "STATE.json").read_text())
    assert type(h.inner).__name__ == "FullEpisodeHandoff" and h.inner.recorder is ep and o.run is __import__("sweepick.control.sweepick_trajectory_executor", fromlist=["*"]).run and o.grip is __import__("sweepick.control.sweepick_trajectory_executor", fromlist=["*"]).grip
    (tmp_path / "looks/00_observe").mkdir(parents=True); (tmp_path / "looks/00_observe/SCENE.json").write_text((R14 / "looks/00_observe/SCENE.json").read_text())
    r = h.ready(tmp_path, st); ep.close()
    assert r["ready"] is False and sorted(m.split(":")[0] for m in r["missing"]) == ["FIELD place", "FIELD right_jaw", "FIELD right_wrist", "FIELD support"] and r["items"]["code_references"]["missing"] == []


# ---- 2026-10-08 final connections: observe scope, insert from the observed relation, close sweep, camera time -------------------

def test_the_observe_scope_reaches_the_load_transfer_observation_without_the_support_or_place_records(tmp_path, monkeypatch):
    """the circle is open: the session that measures what the support reference is made from does not ask for that reference; the left hand is never released in it"""
    f = refs(tmp_path / "f", right_wrist=False, support=False, place=False); c = chain(tmp_path, monkeypatch, check="CLEAR", field=f, scope="observe", right_lift_m=0.005); o = c["owner"]; o.observe_s = 0.3      # only the right gripper's own profile is there
    r = c["run"](handoff_fn=c["handoff"]); names = ran(c)
    assert r["scope"] == "OBSERVE" and r["held_after_lift"] and after_lift(c["out"]) == ["lower", "open", "retreat", "home"] and T.all_off(c["left"]) and not any(c["right"].reg["Torque_Enable"].values())
    assert "load_transfer:right" in names and names[-5:] == ["observe_right_lower:right", "observe_right_open:right", "observe_right_back:right", "observe_right_home:right", "observe_left_back_to_lift:left"]
    assert not any(n.startswith(("left_gripper", "left_retreat", "place_", "stow")) for n in names) and "left_retreat" not in o.done            # nothing of the release, the place or the stow was run
    doc = json.loads((c["out"] / "SUPPORT_OBSERVATION.json").read_text()); whens = [x["when"] for x in doc["rows"]]
    assert whens[0].startswith("right closed, before") and whens[-1].startswith("after the right-hand lift") and doc["end"]["left_at_lift"] and doc["end"]["left_still_loaded"] and doc["end"]["right_released"]
    assert doc["rows"][0]["left_load_raw"]["shoulder_lift"] == 100.0 and doc["rows"][-1]["left_load_raw"]["shoulder_lift"] == 20.0 and "judgement" in doc["note"]      # raw numbers; no threshold was needed or made
    assert doc["rows"][0]["object_at_right_tool"] is True and doc["rows"][0]["frames"]["right_wrist"]["n"] >= 1                                # 'at the right tool' from the observed pose and the joints; the right wrist frames are recorded for the reference to be made from
    assert not any(n["kind"] == "support_evaluated" for n in o.notes) and not fc.FIELD["support"][0].exists()                                 # no support judgement, and no reference file written by the tool
    assert any(t.get("step") == "handoff_observed" for t in r["trail"]) and not any(t.get("step") == "handoff_abandon" for t in r["trail"])


def test_the_observe_scope_needs_its_lift_named_and_the_full_scope_keeps_its_conditions(tmp_path, monkeypatch):
    f = refs(tmp_path / "f", right_wrist=False, support=False, place=False)
    c = chain(tmp_path / "a", monkeypatch, check="CLEAR", field=f, scope="observe"); r = c["run"](handoff_fn=c["handoff"])
    assert r["result"] == "NOT_STARTED_FULL_CHAIN_NOT_READY" and len(r["missing"]) == 1 and r["missing"][0].startswith("PLAN right_lift_m") and ran(c) == []
    c = chain(tmp_path / "b", monkeypatch, check="CLEAR", field=f, scope="full"); r = c["run"](handoff_fn=c["handoff"])
    assert r["result"] == "NOT_STARTED_FULL_CHAIN_NOT_READY" and sorted(m.split(":")[0] for m in r["missing"]) == ["FIELD place", "FIELD right_wrist", "FIELD support"] and ran(c) == []
    c = chain(tmp_path / "c", monkeypatch, check="CLEAR", field=refs(tmp_path / "g", right_jaw=False, support=False, place=False), scope="observe", right_lift_m=0.005); r = c["run"](handoff_fn=c["handoff"])
    assert r["result"] == "NOT_STARTED_FULL_CHAIN_NOT_READY" and r["missing"][0].startswith("FIELD right_jaw")                                # what the right close itself needs is still needed


def test_a_release_command_in_the_observe_scope_is_not_sent(tmp_path, monkeypatch):
    c = chain(tmp_path, monkeypatch, check="CLEAR", field=refs(tmp_path / "f", support=False, place=False), scope="observe", right_lift_m=0.005); o = c["owner"]
    class S_: phase = "RELEASE"; proposals = [dict(q12=[0.0] * 5 + [1.0] + [0.0] * 6)]
    o.session = S_()
    with pytest.raises(RuntimeError, match="OBSERVE scope"):
        o.apply_dispatch(None, dict(q12=[0.0] * 12))
    assert ran(c) == []


def test_the_insert_is_planned_again_from_the_block_as_observed(tmp_path, monkeypatch):
    """the block is seen 6 mm from where the estimate had it: the insert that is run is the teacher's re-plan for the observed block, not the waypoint planned on the estimate"""
    c = chain(tmp_path, monkeypatch, check="CLEAR", seen_shift=(0.0, 0.0, 0.006), seen_centre_m=0.003); o = c["owner"]; r = c["run"](handoff_fn=c["handoff"])      # with the 5 mm of the top depth the room (6.6 mm a side) would not cover this case: see the next test
    chk = [n for n in o.notes if n["kind"] == "insertion_check"]; assert chk and chk[-1]["ready"] is True and chk[-1]["planner"].startswith("HandoverTeacher._plan_right_grasp") and min(chk[-1]["room_each_side_m"]) > chk[-1]["not_resolved_m"]
    wp = next(w for w in o.session.corrector.plan["waypoints"] if w["label"] == "RIGHT_INSERT"); assert wp.get("replanned_from") and not np.allclose(wp["q12"][6:11], wp["previous_q12"][6:11], atol=1e-3)
    assert o.session.state["right_insert_q12"] == wp["q12"] and "right_insert:right" in ran(c) and o.right_closed and "RIGHT_INSERT" in o.done
    # (with the block this much higher in the left hand the teacher's check then refuses the load-transfer lift: the pads of the two hands would meet. The session ends holding there; that is the check working, and it is not what this test is about)
    assert r["result"] in ("FULL_CHAIN_DONE", "HANDOFF_NOT_READY_HOLDING_NEEDS_A_PERSON", "HANDOFF_FAILED_HOLDING_NEEDS_A_PERSON")
    tk = p1.rc.q12_to_ticks(o.mp, np.array(wp["q12"])); log = json.loads(next(Path(n["folder"]).glob("run/move.json")).read_text()) if (n := next(x for x in o.notes if x["kind"] == "segment" and x["name"] == "right_insert")) else None
    assert all(abs(log["end"]["goal_register"][j] - round(tk["right"][j])) <= 1 for j in fc.ARM)                                                # what the executor was sent to is the re-planned insert


def test_seeing_the_block_is_not_enough_when_the_room_does_not_cover_what_is_not_resolved(tmp_path, monkeypatch):
    c = chain(tmp_path, monkeypatch, check="CLEAR", seen_centre_m=0.012); o = c["owner"]; o.idle_limit_s = 1.0; r = c["run"](handoff_fn=c["handoff"])
    chk = [n for n in o.notes if n["kind"] == "insertion_check"][-1]; assert chk["observed_in_hand"] is True and chk["ready"] is False and min(chk["room_each_side_m"]) < chk["not_resolved_m"]
    assert "RIGHT_INSERT" not in o.done and not o.right_closed and r["result"].endswith("_HOLDING_NEEDS_A_PERSON") and after_lift(c["out"]) == []
    assert next(w for w in o.session.corrector.plan["waypoints"] if w["label"] == "RIGHT_INSERT").get("replanned_from") is None             # a re-plan that did not pass is not put into the plan


def test_the_right_close_sweep_is_checked_by_the_collision_check_and_a_contact_stops_the_close(tmp_path, monkeypatch):
    seen = []
    c = chain(tmp_path, monkeypatch, check="CLEAR"); o = c["owner"]; chk0 = o._check
    def chk(doc, side, raw):
        if doc["trajectory"].get("moving") == ["gripper"] and side == "right" and "start" in doc["trajectory"] and len(doc["trajectory"]["goals"]["gripper"]) > 100:
            seen.append(len(doc["trajectory"]["goals"]["gripper"])); return dict(chk0(doc, side, raw), nominal_verdict="CONTACT", first_nominal_contact=dict(step=40, pairs=[["right_gripper_housing", "red_block_geom"]]))
        return chk0(doc, side, raw)
    o._check = chk; r = c["run"](handoff_fn=c["handoff"])
    assert len(seen) == 1 and seen[0] > 1000 and not o.right_closed and not any(n["kind"] == "right_close" for n in o.notes) and r["result"] == "HANDOFF_FAILED_HOLDING_NEEDS_A_PERSON" and "closing sweep" in str(o.stopped)
    assert not hasattr(o, "_grip_cert")


def test_the_real_collision_check_is_what_passes_the_close_sweep_in_the_whole_chain(tmp_path, monkeypatch):
    c = chain(tmp_path, monkeypatch, check="real"); o = c["owner"]; r = c["run"](handoff_fn=c["handoff"]); d = next(Path(c["out"] / "handoff").glob("*_right_close")); s = json.loads((d / "sweep.CHECK.json").read_text())
    assert r["result"] == "FULL_CHAIN_DONE" and s["nominal_verdict"] == "CLEAR" and s["nominal_steps_checked"] > 1000 and s["check"].startswith("sim_data_factory.teacher.Planner.collisions") and "right pad - block" in s["allowed"]


def test_an_old_camera_frame_keeps_its_own_time_and_is_not_this_cycles_image(tmp_path, monkeypatch):
    """the right wrist camera stopped 5 s ago: its packet keeps its own receipt time and sequence, is not fresh, and nothing that needs it is decided"""
    c = chain(tmp_path, monkeypatch, check="CLEAR", stale=("right_wrist",)); o = c["owner"]; o.idle_limit_s = 1.0; r = c["run"](handoff_fn=c["handoff"]); im = o.last_images
    assert im["right_wrist"]["fresh"] is False and im["right_wrist"]["seq"] is None and im["right_wrist"]["feedback_stamp"] == im["right_wrist"]["recv_monotonic_s"] and im["right_wrist"]["age_at_bus_read_s"] > 4.0 and im["right_wrist"]["frame_seq"] == 1
    assert im["left_wrist"]["fresh"] is True and im["left_wrist"]["seq"] == o.last_obs["seq"] and im["left_wrist"]["recv_monotonic_s"] <= o.last_obs["feedback_stamp"] and im["left_wrist"]["validity"] == "reader cadence rule"
    assert o.last_obs["object_evidence"]["evidence_valid"] is False and "left_retreat" not in o.done and not any(n.get("name") == "left_gripper" for n in o.notes if n["kind"] == "segment")      # no grasp label, no support, no release on an old image
    assert r["result"].endswith("_HOLDING_NEEDS_A_PERSON") and after_lift(c["out"]) == []
