"""Console-17: the legacy model-level trim test omits other_arm_read; production stage() always supplies one
(sweepick_pick01.stage: kw.setdefault("other_arm_read", ...)). The same trim checks with a fresh other-arm reader."""
import json
from test_sweepick_grip import MID
from test_sweepick_move_a import Fake, go as run


def test_model_trim_with_a_fresh_other_arm_reader_is_validated_like_its_stage(tmp_path):
    tgt = dict(elbow_flex=1900); seen, reads = [], []
    def plan_cert(d):
        f = Fake(MID, sag=23); run(d, f, mode="plan", level="model", target=tgt, other_arm_ticks=dict(MID))
        return dict(identity=json.loads((d / "TRAJECTORY.json").read_text())["identity"], nominal_verdict="CLEAR")
    def other():
        reads.append(1); return dict(MID)
    for verdict, applied in (("CONTACT", False), ("CLEAR", True)):
        d = tmp_path / verdict; cert = plan_cert(d / "plan"); f = Fake(MID, sag=23)
        def check(doc, v=verdict):
            seen.append(doc["identity"]); assert doc["other_arm_ticks"] == dict(MID); return dict(identity=doc["identity"], nominal_verdict=v)
        code, log = run(d / "run", f, target=tgt, level="model", cert=cert, other_arm_ticks=dict(MID), other_arm_read=other, execute=True, settle_trim=["elbow_flex"], trim_check=check)
        t = log["settle_trim"][0]
        assert "check_error" not in t, t
        assert t["applied"] is applied and (code == 0) is applied and (f.reg["Goal_Position"]["elbow_flex"] == 1877) is applied, (verdict, t)
        assert applied or any(g["gate"] in ("collision_candidate_frame", "certificate_binding") for g in t["provisional"])
    assert len(seen) >= 2 and reads


def test_model_trim_without_an_other_arm_reader_is_cancelled_without_a_write(tmp_path):
    """what the legacy test actually exercises: no reader -> check_error, no trim Goal; it is not a KeyError in the executor"""
    tgt = dict(elbow_flex=1900); d = tmp_path / "x"
    f = Fake(MID, sag=23); run(d / "plan", f, mode="plan", level="model", target=tgt, other_arm_ticks=dict(MID))
    cert = dict(identity=json.loads((d / "plan/TRAJECTORY.json").read_text())["identity"], nominal_verdict="CLEAR")
    f = Fake(MID, sag=23); code, log = run(d / "run", f, target=tgt, level="model", cert=cert, other_arm_ticks=dict(MID), execute=True, settle_trim=["elbow_flex"], trim_check=lambda doc: dict(identity=doc["identity"], nominal_verdict="CLEAR"))
    t = log["settle_trim"][0]
    assert t["applied"] is False and "other-arm actual reader is required" in t["check_error"] and f.reg["Goal_Position"]["elbow_flex"] != 1877
