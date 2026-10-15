"""Approval pauses use resident frames and feedback; no device or motion is constructed."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import sweepick_full_chain as fc

spec = importlib.util.spec_from_file_location("approval_wait_launcher", Path(__file__).resolve().parents[1] / "RUN_APPROVED_SESSION.py")
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)

DELAY = "frame exceeds the reader's measured-cadence freshness limit"


def owner_fixture(tmp_path, monkeypatch, images):
    owner = fc.Owner.__new__(fc.Owner)
    owner.out, owner.scope, owner.stopped = tmp_path, "contact", None
    owner.session = SimpleNamespace(stopped_dispatch=None)
    owner.contact_close_executed = False
    owner.approach_state = {"left_hold_q": [0.] * 6}
    owner.mp = {"left": {"gripper": {"lo": 0, "hi": 100}}}
    owner.left_profile = {"floor": .17}
    profile = json.loads((launcher.REV / "sweepick_jaw_profile_left.json").read_text())
    owner.idle_limit_s = 100 * profile["confirm_s"]["value"]
    owner.idle_basis = "100 times the Jaw profile's confirmation time"
    owner.seq = 0
    owner.time = 28695.791521693
    owner.clock = lambda: owner.time
    owner.sleep = lambda dt: setattr(owner, "time", owner.time + dt)
    owner.notes, owner.detected, owner.reads = [], [], []
    owner.note = lambda kind, **fields: owner.notes.append({"kind": kind, **fields})
    raw = {key: {s: {n: 0 for n in fc.ALL} for s in ("left", "right")}
           for key in ("ticks", "goal", "status", "torque")}
    raw["ticks"]["left"]["gripper"], raw["goal"]["left"]["gripper"] = 3042., 3017.
    raw["torque"]["left"] = dict.fromkeys(fc.ALL, 1)
    raw["torque"]["right"] = dict.fromkeys(fc.ARM, 1) | {"gripper": 0}
    owner.registers = raw

    def read():
        owner.reads.append(owner.time)
        return copy.deepcopy(owner.registers)

    def frames():
        image = images.pop(0) if len(images) > 1 else images[0]
        result = dict(n=6266, recv_monotonic_s=28695.523576045, frame="block",
                      reader_valid=False, fresh=False, reader_reason=DELAY, freshness_limit_s=.06408226200073841)
        if image == "fresh":
            result.update(n=6267 + len(owner.reads), recv_monotonic_s=owner.clock(),
                          reader_valid=True, fresh=True, reader_reason="")
        elif isinstance(image, dict):
            result.update(image)
        return {"left_wrist": result}

    def detect(image):
        owner.detected.append(image)
        return {"visible": image == "block"}

    owner._read, owner._frames = read, frames
    owner.p1 = SimpleNamespace(wrist_object=detect, rc=SimpleNamespace(
        ticks_to_q12=lambda *_: [0.] * 12,
        q12_to_ticks=lambda *_: {"left": {"gripper": 3017.}}))
    monkeypatch.setattr(launcher, "_STOP_SIGNAL", None)
    return owner


def approve_when_request_created(owner, approved=True):
    note = owner.note

    def on_note(kind, **fields):
        note(kind, **fields)
        if kind == "awaiting_observation_move_approval":
            request = json.loads((owner.out / "OBSERVATION_MOVE_REQUEST.json").read_text())
            (owner.out / "OBSERVATION_MOVE_APPROVAL.json").write_text(json.dumps(dict(
                approved=approved, request_sha256=fields["sha256"], session_out=str(owner.out), scope=request["scope"])))
    owner.note = on_note


@pytest.mark.parametrize("approved", [True, False])
def test_delayed_frame_cannot_consume_decision_until_fresh_support(tmp_path, monkeypatch, approved):
    owner = owner_fixture(tmp_path, monkeypatch, ["delay", "delay", "fresh"])
    approve_when_request_created(owner, approved)
    assert launcher.authorize_observation_plan(owner, {"ready": True}, {}, {"id": "view"}) is approved
    checks = [e for e in owner.notes if e["kind"] == "observation_return_check"]
    assert [e["ok"] for e in checks] == [False, False, True]
    assert [e["left_wrist_detection"] for e in checks] == [None, None, {"visible": True}]
    assert all(e["register_checks_ok"] and e["left_jaw_actual_tick"] == 3042. for e in checks)
    assert all(e["register_identity"]["bus_seq"] == i + 1 for i, e in enumerate(checks))
    assert owner.detected == ["block"] and len(owner.reads) == 3
    assert owner.notes[-1]["kind"] == "observation_move_approval_consumed"
    assert any(e["kind"] == "observation_approval_frame_wait" and e["support"] == "UNCONFIRMED" for e in owner.notes)


@pytest.mark.parametrize("change", ["status", "torque", "goal", "unloaded"])
def test_register_failure_is_not_hidden_by_delayed_frame(tmp_path, monkeypatch, change):
    owner = owner_fixture(tmp_path, monkeypatch, ["delay"])
    if change == "status":
        owner.registers["status"]["right"]["elbow_flex"] = 1
    elif change == "torque":
        owner.registers["torque"]["left"]["shoulder_pan"] = 0
    elif change == "goal":
        owner.registers["goal"]["left"]["gripper"] = 3016
    else:
        owner.registers["ticks"]["left"]["gripper"] = 3017
    approve_when_request_created(owner)
    with pytest.raises(RuntimeError, match="lost current support/bus"):
        launcher.authorize_observation_plan(owner, {}, {}, {})
    assert len(owner.reads) == 1
    assert owner.last_recovery_check["register_checks_ok"] is False
    assert not any(e["kind"] == "observation_move_approval_consumed" for e in owner.notes)


@pytest.mark.parametrize("image", [
    {"reader_reason": "reader reported an error: disconnected"},
    {"reader_reason": "reader stream has ended"},
    {"reader_reason": "reader sequence is malformed"},
    {"reader_valid": True, "fresh": True, "reader_reason": "", "frame": "occluded"},
])
def test_reader_failure_or_current_detection_failure_stops_immediately(tmp_path, monkeypatch, image):
    owner = owner_fixture(tmp_path, monkeypatch, [image])
    approve_when_request_created(owner)
    with pytest.raises(RuntimeError, match="lost current support/bus"):
        launcher.authorize_observation_plan(owner, {}, {}, {})
    assert len(owner.reads) == 1
    assert not any(e["kind"] == "observation_approval_frame_wait" for e in owner.notes)
    assert not any(e["kind"] == "observation_move_approval_consumed" for e in owner.notes)


def test_frame_wait_expires_using_existing_owner_budget(tmp_path, monkeypatch):
    owner = owner_fixture(tmp_path, monkeypatch, ["delay"])
    approve_when_request_created(owner)
    started = owner.clock()
    with pytest.raises(RuntimeError, match="lost current support/bus"):
        launcher.authorize_observation_plan(owner, {}, {}, {})
    assert owner.clock() - started >= owner.idle_limit_s
    assert owner.detected == []
    assert not any(e["kind"] == "observation_move_approval_consumed" for e in owner.notes)


def test_restored_frame_must_show_block_even_if_prior_hold_was_confirmed(tmp_path, monkeypatch):
    owner = owner_fixture(tmp_path, monkeypatch, ["delay", {
        "reader_valid": True, "fresh": True, "reader_reason": "", "frame": "occluded"}])
    owner.held = {"status": "HELD_CONFIRMED"}
    approve_when_request_created(owner)
    with pytest.raises(RuntimeError, match="lost current support/bus"):
        launcher.authorize_observation_plan(owner, {}, {}, {})
    assert len(owner.reads) == 2 and owner.detected == ["occluded"]
    assert owner.last_recovery_check["left_wrist_detection"] == {"visible": False}
    assert not any(e["kind"] == "observation_move_approval_consumed" for e in owner.notes)


@pytest.mark.parametrize("fault", ["bus", "stop"])
def test_bus_loss_and_stop_interrupt_frame_wait_without_retransmission(tmp_path, monkeypatch, fault):
    owner = owner_fixture(tmp_path, monkeypatch, ["delay"])

    def interrupt(_dt):
        if fault == "stop":
            Path(str(owner.out) + ".STOP").touch()
        else:
            owner._read = lambda: (_ for _ in ()).throw(RuntimeError("BUS_LOST ID3"))
    owner.sleep = interrupt
    with pytest.raises(RuntimeError, match="BUS_LOST|lost current support/bus"):
        launcher.authorize_observation_plan(owner, {}, {}, {})
    assert owner.detected == []
    assert not any(e["kind"] == "observation_move_approval_consumed" for e in owner.notes)


def test_healthy_human_wait_has_no_software_idle_deadline(tmp_path, monkeypatch):
    owner = owner_fixture(tmp_path, monkeypatch, ["fresh"])
    started, sleep = owner.clock(), owner.sleep

    def later_decision(dt):
        sleep(dt)
        if owner.clock() - started > owner.idle_limit_s:
            import hashlib
            content = (owner.out / "OBSERVATION_MOVE_REQUEST.json").read_bytes()
            request = json.loads(content)
            (owner.out / "OBSERVATION_MOVE_APPROVAL.json").write_text(json.dumps(dict(
                approved=True, request_sha256=hashlib.sha256(content).hexdigest(),
                session_out=str(owner.out), scope=request["scope"])))
    owner.sleep = later_decision
    assert launcher.authorize_observation_plan(owner, {}, {}, {}) is True
    assert owner.clock() - started > owner.idle_limit_s
    assert len(owner.reads) == len(owner.detected) > 1
    assert not any(e["kind"] == "observation_approval_frame_wait" for e in owner.notes)
