"""RIGHT_X_FINITE_METROLOGY launcher: its own approval/manifest scope, refused before any device, per-move decisions."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import RUN_APPROVED_SESSION as runner
import right_x_metrology as rx

APPROVED = dict(approved=True, field_ready=True, permission_to_execute=True, permission_to_open_devices=True)
CFG = dict(t_pose='AFTER_RIGHT_PREGRASP', x_basis_tool=[1.0, 0.0, 0.0], x_basis_source='A model tool +X; physical direction unconfirmed',
           fit_displacement_m=-0.004, verify_displacement_m=-0.004, verify_from='F')


def bind(tmp_path, monkeypatch, *, approval, manifest_scope=rx.SCOPE, execution_scope=rx.SCOPE, plan_extra=None, files=None):
    out = tmp_path / 'run/session'
    required = (Path(runner.__file__).resolve(), runner.REV / 'sweepick_full_chain.py', runner.REV / 'sweepick_pick01.py',
                runner.REV / 'sweepick_move_a.py', runner.REV / 'right_x_metrology.py', runner.REV / 'current_view_candidate.py')
    files = files if files is not None else {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in required}
    mpath = tmp_path / 'MANIFEST.json'; mpath.write_text(json.dumps(dict(scope=manifest_scope, execution_scope=execution_scope, files=files)))
    a = dict(APPROVED, session_out=str(out), prepared_manifest_sha256=hashlib.sha256(mpath.read_bytes()).hexdigest(), **approval)
    apath = tmp_path / 'APPROVAL.json'; apath.write_text(json.dumps(a))
    plan = dict(json.loads(runner.PLAN.read_text()), **(plan_extra or {}))
    ppath = tmp_path / 'PLAN.json'; ppath.write_text(json.dumps(plan))
    monkeypatch.setattr(runner, 'APPROVAL', apath); monkeypatch.setattr(runner, 'MANIFEST', mpath); monkeypatch.setattr(runner, 'PLAN', ppath)
    monkeypatch.setenv('SWEEPICK_SESSION_OUT', str(out))
    return out


def right_x(out=None, **over):
    a = dict(scope=rx.SCOPE, execution_scope=rx.SCOPE, declared_motor_writes=list(rx.DECLARED_WRITES), excluded=list(rx.EXCLUDED),
             right_gripper_commands_permitted=False, insertion_permitted=False, close_permitted=False, left_release_permitted=False,
             right_x_metrology=dict(CFG))
    a.update(over); return a


def test_right_x_approval_binds_its_config_into_the_plan(tmp_path, monkeypatch):
    out = bind(tmp_path, monkeypatch, approval=right_x())
    approval, plan, got, manifest = runner.validate_launch()
    assert got == out and runner.execution_scope(approval) == rx.SCOPE and plan['right_x_metrology'] == CFG and plan['observation_only'] is True


@pytest.mark.parametrize('approval, manifest_scope, plan_extra, words', [
    (right_x(right_x_metrology=dict(CFG, fit_displacement_m=0)), rx.SCOPE, None, 'non-zero'),
    (right_x(declared_motor_writes=[]), rx.SCOPE, None, 'declared_motor_writes'),
    (right_x(observation_view_preapproval={'approved': True}), rx.SCOPE, None, 'pre-approval'),
    (right_x(left_release_permitted=True), rx.SCOPE, None, 'left release'),
    (right_x(), 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION', None, 'not a RIGHT_X_FINITE_METROLOGY manifest'),
    (right_x(), rx.SCOPE, {'right_x_metrology': CFG}, 'must not carry right_x_metrology'),
    (dict(scope='RIGHT_RECEIVE_NONCONTACT_OBSERVATION'), 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION', {'right_x_metrology': CFG}, 'must not carry right_x_metrology'),
])
def test_incomplete_or_foreign_approval_is_refused_before_any_device(tmp_path, monkeypatch, approval, manifest_scope, plan_extra, words):
    bind(tmp_path, monkeypatch, approval=approval, manifest_scope=manifest_scope, plan_extra=plan_extra)
    with pytest.raises(RuntimeError, match=words):
        runner.validate_launch()


def test_full_chain_approval_carries_no_metrology(tmp_path, monkeypatch):
    bind(tmp_path, monkeypatch, approval=dict(scope='RIGHT_RECEIVE_NONCONTACT_OBSERVATION'), manifest_scope='RIGHT_RECEIVE_NONCONTACT_OBSERVATION', execution_scope=None)
    approval, plan, out, manifest = runner.validate_launch()
    assert runner.execution_scope(approval) == runner.FULL_CHAIN and 'right_x_metrology' not in plan


def test_right_x_manifest_must_bind_its_sources(tmp_path, monkeypatch):
    src = tmp_path / 'only.py'; src.write_text('y = 1\n')
    bind(tmp_path, monkeypatch, approval=right_x(), files={str(src): hashlib.sha256(src.read_bytes()).hexdigest()})
    with pytest.raises(RuntimeError, match='does not bind'):
        runner.validate_launch()


def waiting_owner(tmp_path, *, decide):
    clock = [0.0]; notes = []
    owner = SimpleNamespace(out=tmp_path, clock=lambda: clock[0], notes=notes, idle_limit_s=5.0, idle_basis='test', last_recovery_check={},
                            note=lambda kind, **v: notes.append(dict(kind=kind, **v)), raw=lambda: {'ticks': {}, 'status': {}},
                            recovery_ready=lambda *a, **k: (True, []))

    def sleep(seconds):
        clock[0] += seconds; decide(owner, clock[0])
    owner.sleep = sleep
    return owner


def test_move_request_has_its_own_scope_and_ignores_an_earlier_decision_file(tmp_path):
    (tmp_path / 'OBSERVATION_MOVE_APPROVAL.json').write_text(json.dumps(dict(approved=False, request_sha256='earlier-view', scope=runner.PREAPPROVAL_SCOPE, session_out=str(tmp_path))))

    def decide(owner, now):
        request = json.loads((tmp_path / 'OBSERVATION_MOVE_REQUEST.json').read_text())
        assert request['scope'] == rx.MOVE_SCOPE
        digest = hashlib.sha256((tmp_path / 'OBSERVATION_MOVE_REQUEST.json').read_bytes()).hexdigest()
        if now > 1.0:
            (tmp_path / 'OBSERVATION_MOVE_APPROVAL.json').write_text(json.dumps(dict(approved=True, request_sha256=digest, scope=rx.MOVE_SCOPE, session_out=str(tmp_path))))
    owner = waiting_owner(tmp_path, decide=decide)
    pre = dict(approved=True)                                    # a START pre-approval never covers an X move
    assert runner.authorize_observation_plan(owner, {'forward': []}, {}, rx.move_candidate('F', CFG), preapproval=pre, scope=rx.MOVE_SCOPE) is True
    assert owner.clock() > 1.0 and not any(n['kind'].startswith('observation_move_preapproval') for n in owner.notes)


def test_view_decision_for_another_scope_is_not_an_x_approval(tmp_path):
    def decide(owner, now):
        digest = hashlib.sha256((tmp_path / 'OBSERVATION_MOVE_REQUEST.json').read_bytes()).hexdigest()
        (tmp_path / 'OBSERVATION_MOVE_APPROVAL.json').write_text(json.dumps(dict(approved=True, request_sha256=digest, scope=runner.PREAPPROVAL_SCOPE, session_out=str(tmp_path))))
    owner = waiting_owner(tmp_path, decide=decide)
    assert runner.authorize_observation_plan(owner, {'forward': []}, {}, rx.move_candidate('V', CFG), scope=rx.MOVE_SCOPE) is False
