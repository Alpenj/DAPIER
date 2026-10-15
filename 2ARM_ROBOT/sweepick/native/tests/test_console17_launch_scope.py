"""Console-17 launcher: LEFT_START_TO_LIFT has its own approval and manifest scope, the full chain is unchanged, and the
session call is chosen by the approved scope. No device, SDK or executor module is imported (all are stubbed)."""
import hashlib
import json
import sys
from types import SimpleNamespace, ModuleType
from pathlib import Path
import pytest
import RUN_APPROVED_SESSION as runner

APPROVED = dict(approved=True, field_ready=True, permission_to_execute=True, permission_to_open_devices=True, unchanged_wrist_mount_and_routes=True)


def bind(tmp_path, monkeypatch, *, approval, manifest_scope, execution_scope=None, files=None):
    out = tmp_path / "run/session"
    src = tmp_path / "src.py"; src.write_text("x = 1\n")
    files = files if files is not None else {str(src): hashlib.sha256(src.read_bytes()).hexdigest(),
        **{str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(runner.__file__).resolve(), runner.REV / "sweepick_pick01.py", runner.REV / "sweepick_move_a.py")}}
    manifest = dict(scope=manifest_scope, files=files, **({"execution_scope": execution_scope} if execution_scope else {}))
    mpath = tmp_path / "MANIFEST.json"; mpath.write_text(json.dumps(manifest))
    a = dict(APPROVED, session_out=str(out), prepared_manifest_sha256=hashlib.sha256(mpath.read_bytes()).hexdigest(), **approval)
    apath = tmp_path / "APPROVAL.json"; apath.write_text(json.dumps(a))
    monkeypatch.setattr(runner, "APPROVAL", apath); monkeypatch.setattr(runner, "MANIFEST", mpath)
    monkeypatch.setenv("SWEEPICK_SESSION_OUT", str(out))
    return out, src


LIMITED = dict(scope="LEFT_START_TO_LIFT", execution_scope="LEFT_START_TO_LIFT", right_arm_commands_permitted=False, automatic_return_after_lift=False)


def test_limited_approval_and_manifest_bind_together(tmp_path, monkeypatch):
    out, _ = bind(tmp_path, monkeypatch, approval=LIMITED, manifest_scope="LEFT_START_TO_LIFT", execution_scope="LEFT_START_TO_LIFT")
    approval, plan, got, manifest = runner.validate_launch()
    assert got == out and runner.execution_scope(approval) == "LEFT_START_TO_LIFT" and manifest["execution_scope"] == "LEFT_START_TO_LIFT"


@pytest.mark.parametrize("approval, manifest_scope, execution_scope, words", [
    (dict(scope="RIGHT_RECEIVE_NONCONTACT_OBSERVATION", execution_scope="LEFT_START_TO_LIFT", right_arm_commands_permitted=False, automatic_return_after_lift=False), "LEFT_START_TO_LIFT", "LEFT_START_TO_LIFT", "approval does not match"),
    (LIMITED, "RIGHT_RECEIVE_NONCONTACT_OBSERVATION", None, "not a LEFT_START_TO_LIFT manifest"),
    (dict(LIMITED, automatic_return_after_lift=True), "LEFT_START_TO_LIFT", "LEFT_START_TO_LIFT", "automatic_return_after_lift=false"),
    (dict(LIMITED, right_arm_commands_permitted=None), "LEFT_START_TO_LIFT", "LEFT_START_TO_LIFT", "right_arm_commands_permitted=false"),
    (dict(LIMITED, execution_scope="LEFT_LIFT_AND_CARRY"), "LEFT_START_TO_LIFT", "LEFT_START_TO_LIFT", "unsupported execution_scope"),
])
def test_a_full_chain_or_mismatched_approval_never_opens_the_limited_scope(tmp_path, monkeypatch, approval, manifest_scope, execution_scope, words):
    bind(tmp_path, monkeypatch, approval=approval, manifest_scope=manifest_scope, execution_scope=execution_scope)
    with pytest.raises(RuntimeError, match=words):
        runner.validate_launch()


def test_limited_manifest_must_bind_the_launcher_and_executors(tmp_path, monkeypatch):
    src = tmp_path / "only.py"; src.write_text("y = 2\n")
    bind(tmp_path, monkeypatch, approval=LIMITED, manifest_scope="LEFT_START_TO_LIFT", execution_scope="LEFT_START_TO_LIFT", files={str(src): hashlib.sha256(src.read_bytes()).hexdigest()})
    with pytest.raises(RuntimeError, match="does not bind"):
        runner.validate_launch()


def test_a_changed_source_after_preparation_stops_the_limited_launch(tmp_path, monkeypatch):
    out, src = bind(tmp_path, monkeypatch, approval=LIMITED, manifest_scope="LEFT_START_TO_LIFT", execution_scope="LEFT_START_TO_LIFT")
    src.write_text("x = 2\n")
    with pytest.raises(RuntimeError, match="changed before launch"):
        runner.validate_launch()


def test_full_chain_approval_without_execution_scope_keeps_the_existing_checks(tmp_path, monkeypatch):
    bind(tmp_path, monkeypatch, approval=dict(scope="RIGHT_RECEIVE_NONCONTACT_OBSERVATION"), manifest_scope="RIGHT_RECEIVE_NONCONTACT_OBSERVATION")
    approval, plan, out, manifest = runner.validate_launch()
    assert runner.execution_scope(approval) == "FULL_CHAIN_RECEIVE_CONTACT" and plan["observation_only"] is True


def _stub_main(monkeypatch, scope_approval, out):
    calls = []
    p1 = SimpleNamespace(LEFT_TO_LIFT="LEFT_START_TO_LIFT", model=lambda: calls.append("face_model") or object(), SCENE="s", CAND="c", CAL="c", ASM="a", __file__=__file__,
                         run_session=lambda o, **k: calls.append(("run_session", k)) or 0)
    fc = SimpleNamespace(build=lambda *a, **k: (None, None), __file__=__file__)
    monkeypatch.setattr(runner, "validate_launch", lambda: (scope_approval, dict(face_geometry_source_xml="x", wrist_route_basis="b", model_assisted_optical_contract={}), out, dict(files={})))
    monkeypatch.setattr(runner, "managed_tmux_owner", lambda: True)
    monkeypatch.setattr(runner, "preload_sources", lambda o: (fc, p1))
    monkeypatch.setattr(sys, "argv", ["RUN_APPROVED_SESSION.py", "--execute", "--managed-tmux"])
    monkeypatch.delenv("SWEEPICK_CONSOLE_FEED", raising=False)
    mods = {"sweepick_episode": dict(Episode=object), "sweepick_local_loop": dict(WristCam=object, SdkTop=object, f5=None),
            "sweepick_commission": dict(real_bus=None), "confirmed_wrist_routes": dict(bind_wrist_readers=lambda *a: dict(bound=True)),
            "bus_packet_ring": dict(install=lambda d: SimpleNamespace(flush=lambda: None), __file__=str(runner.REV / "bus_packet_ring.py")),
            "inner_pad_face_contract": dict(prepare_face_geometry=lambda *a, **k: calls.append("face_geometry"))}
    for name, attrs in mods.items():
        m = ModuleType(name); m.__dict__.update(attrs); monkeypatch.setitem(sys.modules, name, m)
    return calls


def test_main_runs_the_limited_session_without_the_receive_preparation(tmp_path, monkeypatch):
    out = tmp_path / "run/session"; calls = _stub_main(monkeypatch, dict(LIMITED), out)
    assert runner.main() == 0
    run = [c for c in calls if isinstance(c, tuple)]
    assert run == [("run_session", dict(execute=True, record=True, full_chain=False, limit="LEFT_START_TO_LIFT", episode_fn=run[0][1]["episode_fn"]))]
    assert "face_geometry" not in calls and "face_model" not in calls
    events = [json.loads(l) for l in (out / "review/lifetime.jsonl").read_text().splitlines()]
    assert any(e["event"] == "execution_scope" and e["scope"] == "LEFT_START_TO_LIFT" for e in events) and events[-1]["event"] == "owner_end"


def test_main_full_chain_call_is_unchanged(tmp_path, monkeypatch):
    out = tmp_path / "run/session"; calls = _stub_main(monkeypatch, dict(scope="RIGHT_RECEIVE_NONCONTACT_OBSERVATION"), out)
    assert runner.main() == 0
    run = [c for c in calls if isinstance(c, tuple)]
    k = run[0][1]
    assert k["full_chain"] is True and k["scope"] == "RECEIVE_CONTACT" and "limit" not in k and k["record"] is True and "face_geometry" in calls
