import json, sys
import pytest
import sweepick_vservo_shadow as vs


def write_session(tmp_path):
    (tmp_path / "looks/01_pregrasp_0").mkdir(parents=True); (tmp_path / "looks/01_pregrasp_0/left_wrist.jpg").write_bytes(b"jpg")
    row = dict(schema=vs.INPUT_SCHEMA, observation_identity=dict(session_out=str(tmp_path), look_tag="01_pregrasp_0"),
               files=dict(left_wrist="looks/01_pregrasp_0/left_wrist.jpg", top="looks/01_pregrasp_0/top.png", scene="looks/01_pregrasp_0/SCENE.json"),
               q12=[0.0] * 12, timing_monotonic_s=dict(look_called=1.0, look_returned=2.0, judged=2.1), a_feature=dict(offset_px=3.0), a_correction=None,
               executable=False, motor_write_authorized=False)
    (tmp_path / vs.INPUT).write_text(json.dumps(row) + "\n" + "{not json\n")
    return row


def test_inputs_and_observation_are_offline_and_non_executable(tmp_path):
    write_session(tmp_path)
    rows, errors = vs.inputs(tmp_path)
    assert len(rows) == 1 and len(errors) == 1
    obs = vs.observation(tmp_path, rows[0])
    assert obs["views"]["left_wrist"]["available"] and not obs["views"]["top"]["available"] and not obs["views"]["right_wrist"]["available"]
    assert obs["executable"] is False and obs["reference_times"]["loop"]["judged"] == 2.1
    import subprocess
    probe = "import sys, sweepick_vservo_shadow; print(sorted(m for m in ('sweepick_move_a','sweepick_commission','sweepick_pick01','scservo_sdk','lerobot','cv2') if m in sys.modules))"
    assert subprocess.run([sys.executable, "-c", probe], capture_output=True, text=True, cwd=str(vs.Path(vs.__file__).parent)).stdout.strip() == "[]"


@pytest.mark.parametrize("bad", [dict(executable=True, motor_write_authorized=False), dict(executable=False), dict(executable=False, motor_write_authorized=False)])
def test_only_named_non_executable_proposals_are_recorded(tmp_path, bad):
    with pytest.raises(ValueError):
        vs.record_proposal(tmp_path, bad)
    assert not (tmp_path / vs.PROPOSALS).exists()


def test_a_named_non_executable_proposal_is_recorded(tmp_path):
    line = vs.record_proposal(tmp_path, dict(executable=False, motor_write_authorized=False, observation_identity=dict(observation_id="01_pregrasp_0")))
    assert line["role"] == "SHADOW_PROPOSAL_RECORD_ONLY" and (tmp_path / vs.PROPOSALS).read_text().count("\n") == 1
