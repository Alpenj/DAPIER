"""2026-10-15 START pre-approval of the single non-contact observation move. Input: the stored REAL request of
20261015_151937_81d00c (plan, candidate, actual feedback). Fake owner / fake bus only; nothing opens a device."""
import copy, hashlib, json
from pathlib import Path
import pytest
import RUN_APPROVED_SESSION as runner

REQ = json.loads((Path(__file__).parent / "review/preapproval_20261015/REQUEST_20261015_151937_81d00c.json").read_text())
MSHA = "a" * 64


def pre(out, **over):
    p = dict(approved=True, scope=runner.PREAPPROVAL_SCOPE, candidate_id=runner.PREAPPROVAL_CANDIDATE, candidate_count=1, purpose=runner.PREAPPROVAL_PURPOSE,
             insertion_permitted=False, close_permitted=False, session_out=str(out), prepared_manifest_sha256=MSHA, granted_at="t")
    p.update(over); return p


def inputs():
    return copy.deepcopy(REQ["plan"]), copy.deepcopy(REQ["actual_feedback"]), copy.deepcopy(REQ["candidate"])


class Owner:
    """the attributes authorize_observation_plan uses; decision=None: no operator answer; else the answer written on the first wait"""
    def __init__(self, out, decision=None, supported=True):
        self.out, self.notes, self.decision, self.supported, self.t, self.waits = Path(out), [], decision, supported, 0.0, 0
        self.idle_limit_s, self.idle_basis, self.last_recovery_check = 5.0, "test", {"register_checks_ok": True, "left_wrist_frame": {}}
    def note(self, kind, **k): self.notes.append(dict(kind=kind, **k))
    def clock(self): self.t += 0.1; return self.t
    def raw(self): return copy.deepcopy(REQ["actual_feedback"])
    def recovery_ready(self, why, observation=None): return (self.supported, None if self.supported else "left support unconfirmed")
    def sleep(self, s):
        self.waits += 1
        if self.decision is None:
            raise TimeoutError("no operator answer (test)")
        digest = hashlib.sha256((self.out / "OBSERVATION_MOVE_REQUEST.json").read_bytes()).hexdigest()
        (self.out / "OBSERVATION_MOVE_APPROVAL.json").write_text(json.dumps(dict(approved=self.decision, request_sha256=digest, session_out=str(self.out), scope=runner.PREAPPROVAL_SCOPE)))


def authorize(owner, preapproval):
    plan, raw, cand = inputs()
    return runner.authorize_observation_plan(owner, plan, raw, cand, preapproval=preapproval, manifest_sha=MSHA)


def test_the_stored_request_is_covered_by_the_start_preapproval(tmp_path):
    plan, raw, cand = inputs()
    covered, checks = runner.preapproval_covers(pre(tmp_path), tmp_path, MSHA, plan, raw, cand, used=False)
    assert covered and all(checks.values()), checks


@pytest.mark.parametrize("change, key", [
    (lambda p, plan, raw, c, out: (p, plan, raw, dict(c, id="OTHER_VIEW"), out), "candidate_kind"),
    (lambda p, plan, raw, c, out: (p, plan, raw, c, out.parent / "other_session"), "same_session"),
    (lambda p, plan, raw, c, out: (dict(p, prepared_manifest_sha256="b" * 64), plan, raw, c, out), "same_manifest"),
    (lambda p, plan, raw, c, out: (p, dict(plan, forward=[dict(plan["forward"][0], certificate=dict(plan["forward"][0]["certificate"], nominal_verdict="CONTACT"))] + plan["forward"][1:]), raw, c, out), "every_step_certified_clear"),
    (lambda p, plan, raw, c, out: (p, plan, dict(raw, ticks=dict(raw["ticks"], right=dict(raw["ticks"]["right"], elbow_flex=raw["ticks"]["right"]["elbow_flex"] + 40))), c, out), "planned_from_current_actual"),
    (lambda p, plan, raw, c, out: (p, plan, dict(raw, status=dict(raw["status"], right=dict(raw["status"]["right"], elbow_flex=32.0))), c, out), "bus_status_zero"),
    (lambda p, plan, raw, c, out: (p, dict(plan, forward=[dict(plan["forward"][0], side="left")]), raw, c, out), "finite_right_noncontact_steps"),
    (lambda p, plan, raw, c, out: (p, dict(plan, ready=False), raw, c, out), "plan_ready"),
])
def test_anything_outside_the_preapproved_scope_is_not_covered(tmp_path, change, key):
    plan, raw, cand = inputs()
    p, plan, raw, cand, out = change(pre(tmp_path), plan, raw, cand, tmp_path)
    covered, checks = runner.preapproval_covers(p, out, MSHA, plan, raw, cand, used=False)
    assert not covered and checks[key] is False


def test_preapproval_is_single_use(tmp_path):
    plan, raw, cand = inputs()
    assert runner.preapproval_covers(pre(tmp_path), tmp_path, MSHA, plan, raw, cand, used=True)[1]["not_used_yet"] is False


def test_covered_request_proceeds_without_the_mid_session_wait(tmp_path):
    owner = Owner(tmp_path, decision=None)
    assert authorize(owner, pre(tmp_path)) is True and owner.waits == 0
    decision = json.loads((tmp_path / "OBSERVATION_MOVE_APPROVAL.json").read_text())
    assert decision["operator_source"] == "START_PREAPPROVAL" and decision["request_sha256"] == hashlib.sha256((tmp_path / "OBSERVATION_MOVE_REQUEST.json").read_bytes()).hexdigest()
    kinds = [n["kind"] for n in owner.notes]
    assert "observation_move_preapproval_applied" in kinds and "awaiting_observation_move_approval" not in kinds
    s2 = tmp_path / "s2"; s2.mkdir()
    second = Owner(s2, decision=None); second.preapproval_used = True                         # the same owner never uses it twice
    with pytest.raises(TimeoutError):
        authorize(second, pre(s2))
    assert "awaiting_observation_move_approval" in [n["kind"] for n in second.notes]


@pytest.mark.parametrize("why", ["support", "candidate"])
def test_uncovered_or_unsupported_request_falls_back_to_the_existing_operator_decision(tmp_path, why):
    owner = Owner(tmp_path, decision=False, supported=why != "support")
    p = pre(tmp_path, candidate_id="OTHER") if why == "candidate" else pre(tmp_path)
    if why == "candidate":
        with pytest.raises(RuntimeError):                                                    # a mismatched pre-approval is rejected at launch
            runner.preapproval_record(dict(observation_view_preapproval=p), tmp_path, MSHA)
        plan, raw, cand = inputs()
        result = runner.authorize_observation_plan(owner, plan, raw, dict(cand, id="OTHER"), preapproval=pre(tmp_path), manifest_sha=MSHA)
    else:
        with pytest.raises(RuntimeError):                                                    # unsupported: the existing wait loop refuses (no auto move)
            authorize(owner, p)
        return
    assert result is False and owner.waits >= 1
    kinds = [n["kind"] for n in owner.notes]
    assert kinds.index("observation_move_preapproval_not_applicable") < kinds.index("awaiting_observation_move_approval")


def test_manual_mode_is_unchanged(tmp_path):
    owner = Owner(tmp_path, decision=True)
    assert authorize(owner, None) is True and owner.waits == 1
    assert [n["kind"] for n in owner.notes] == ["awaiting_observation_move_approval", "observation_move_approval_consumed"]


def test_stop_before_a_preapproved_move_raises(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, "_STOP_SIGNAL", 2)
    with pytest.raises(RuntimeError, match="STOP"):
        authorize(Owner(tmp_path), pre(tmp_path))
    assert not (tmp_path / "OBSERVATION_MOVE_APPROVAL.json").exists()


@pytest.mark.parametrize("field, value", [("session_out", "/elsewhere"), ("prepared_manifest_sha256", "c" * 64), ("insertion_permitted", True),
                                          ("close_permitted", True), ("candidate_count", 2), ("scope", "RIGHT_INSERT")])
def test_launch_rejects_a_preapproval_that_widens_or_moves_scope(tmp_path, field, value):
    with pytest.raises(RuntimeError, match=field):
        runner.preapproval_record(dict(observation_view_preapproval=pre(tmp_path, **{field: value})), tmp_path, MSHA)
    assert runner.preapproval_record({}, tmp_path, MSHA) is None
