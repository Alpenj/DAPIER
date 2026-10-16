"""LEFT CLOSE FOV view prediction: synthetic evidence rows shaped like STATE.json. No device, no executor run."""
import hashlib
from pathlib import Path

import pytest

import left_fov_check as L

CONTEXT = dict(camera_side='left_wrist', mount_binding_sha256=hashlib.sha256(b'synthetic left wrist mount fixture').hexdigest(),
               image_size_px=[320, 240], object_width_mm=40.0)
FRAME = dict(camera_side='left_wrist', frame_id='synthetic-frame-1', raw_rgb_sha256=hashlib.sha256(b'synthetic raw RGB fixture').hexdigest())


def look(what, ok, offset, target, width, touches=False, context=CONTEXT, frame=FRAME):
    return dict(what=what, ok=ok, lateral_error_mm=None if offset is None else offset - target, lateral_target_mm=target,
                wrist_object=dict(visible=True, stable=True, width_px=width, centre_px=154.0, edges_px=[0, 0], touches_border=touches),
                wrist_offset=dict(readable=offset is not None, offset_mm=offset, reference_confirmed_in_this_frame=True),
                fov_context=context, frame_binding=frame)


def state(pre_offset, pre_width, final_offset, final_width, final_ok=True, target=0.0):
    return {'evidence': [look('ready_to_descend', True, pre_offset, target, pre_width),
                         look('ready_to_close', final_ok, final_offset if final_ok else None, target, final_width, touches=not final_ok)],
            'jaw_reference': {'midline_px': 154.0}}


REF = L.reference_from_states([('a', state(-1.0, 170, -1.5, 263)), ('b', state(1.0, 170, 0.5, 263)), ('c', state(2.0, 160, 2.0, 245)),
                               ('failed', state(-5.0, 160, None, 262, final_ok=False))])


def test_the_reference_holds_only_readable_real_finals_and_uses_the_raw_offset():
    assert [r['session'] for r in REF['runs']] == ['a', 'b', 'c'] and REF['schema'].endswith('v2')
    assert REF['pre_offset_mm_range'] == [-1.0, 2.0] and REF['offset_change_mm_median'] == -0.5
    assert REF['role'].startswith('OBSERVED_REAL_RUNS_WITH_A_READABLE_FINAL_LOOK')


def test_raw_offset_and_lateral_error_are_kept_apart():
    # same raw image position, different targets: the predicted image edges must not move with the target
    a = L.predict_final_view(look('ready_to_descend', True, -2.0, 0.0, 160), 154.0, REF)
    b = L.predict_final_view(look('ready_to_descend', True, -2.0, -1.5, 160), 154.0, REF)
    assert a['pred_edges_px'] == b['pred_edges_px']
    assert (a['inputs']['raw_offset_mm'], a['inputs']['lateral_error_mm'], b['inputs']['lateral_error_mm']) == (-2.0, -2.0, -0.5)


def test_an_out_of_support_look_never_requests_an_extrapolated_correction():
    c = L.candidate(look('ready_to_descend', True, -5.15, 0.0, 160), 154.0, REF)
    assert c['status'] == 'OUT_OF_SUPPORT' and c['executable'] is False and c['prediction']['pred_edges_px'][0] <= 1
    assert c['prediction']['inside_observed_pre_offset_range'] is False


def test_a_look_inside_changes_nothing_and_a_failed_check_is_left_to_the_existing_path():
    assert L.candidate(look('ready_to_descend', True, 0.3, 0.0, 165), 154.0, REF)['status'] == 'NO_CHANGE'
    assert L.candidate(look('ready_to_descend', False, -8.0, 0.0, 160), 154.0, REF)['status'] == 'EXISTING_CHECK_ALREADY_REALIGNS'


def test_a_clipped_or_unreadable_look_is_never_given_a_centre():
    assert L.predict_final_view(look('ready_to_descend', False, -1.0, 0.0, 262, touches=True), 154.0, REF) is None
    assert L.predict_final_view(look('ready_to_descend', False, None, 0.0, 160), 154.0, REF) is None
    assert L.candidate(look('ready_to_descend', False, None, 0.0, 160), 154.0, REF)['status'] == 'UNREADABLE'


def test_context_resolution_object_width_and_raw_frame_binding_must_match():
    good = look('ready_to_descend', True, 0.3, 0.0, 165)
    assert not L.context_issues(good, REF)
    for changed in (dict(CONTEXT, image_size_px=[640, 480]), dict(CONTEXT, object_width_mm=39.0),
                    dict(CONTEXT, camera_side='right_wrist'), dict(CONTEXT, mount_binding_sha256='other-mount')):
        assert L.context_issues(dict(good, fov_context=changed), REF)
    assert L.context_issues(dict(good, frame_binding=None), REF)
    incompatible = dict(CONTEXT, camera_side='right_wrist', image_size_px=[640, 480], object_width_mm=80.0)
    malformed = dict(camera_side='', mount_binding_sha256='', image_size_px=[], object_width_mm='')
    for context, frame in ((incompatible, dict(FRAME, camera_side='right_wrist')),
                           (malformed, dict(camera_side='', frame_id='', raw_rgb_sha256='garbage'))):
        assert L.context_issues(look('ready_to_descend', True, 0.3, 0.0, 165, context=context, frame=frame), dict(REF, context=context))


def test_no_real_success_means_no_reference_and_a_missing_file_is_no_reference(tmp_path):
    with pytest.raises(ValueError):
        L.reference_from_states([('failed', state(-5.0, 160, None, 262, final_ok=False))])
    assert L.load_reference(tmp_path / 'missing.json') is None
    corrupt = tmp_path / 'corrupt.json'; corrupt.write_text('{bad')
    incomplete = tmp_path / 'incomplete.json'; incomplete.write_text('{"schema":"sweepick.left-fov-reference.v2","runs":[{}]}')
    assert L.load_reference(corrupt) is None and L.load_reference(incomplete) is None


def test_only_the_pre_grasp_gate_of_the_executor_uses_the_prediction():
    here = Path(__file__).resolve().parent
    src = (here / 'sweepick_pick01.py').read_text()
    assert 'FOV_GATES = ("ready_to_descend",)' in src and 'import left_fov_check as lf' in src
    for name in ('sweepick_full_chain.py', 'sweepick_move_a.py', 'sweepick_episode.py'):
        assert 'left_fov_check' not in (here / name).read_text()
