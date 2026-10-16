"""LEFT CLOSE FOV offline candidate: synthetic evidence rows shaped like STATE.json. No device, no executor import."""
from pathlib import Path

import pytest

import left_fov_check as L


def state(pre_err, pre_width, final_err, final_width, final_ok=True, first_ok=True):
    pre = dict(what='ready_to_descend', ok=first_ok, lateral_error_mm=pre_err,
               wrist_object=dict(visible=True, width_px=pre_width, edges_px=[0, 0], touches_border=False))
    final = dict(what='ready_to_close', ok=final_ok, lateral_error_mm=final_err if final_ok else None,
                 wrist_object=dict(visible=True, width_px=final_width, edges_px=[10, 270], touches_border=not final_ok))
    return {'evidence': [pre, final], 'jaw_reference': {'midline_px': 154.0}}


REF = L.reference_from_states([('a', state(-1.0, 170, -1.5, 263)), ('b', state(1.0, 170, 0.5, 263)), ('c', state(2.0, 160, 2.0, 245)),
                               ('failed', state(-5.0, 160, None, 262, final_ok=False))])


def test_the_reference_holds_only_readable_real_finals():
    assert [r['session'] for r in REF['runs']] == ['a', 'b', 'c']
    assert REF['pre_err_mm_range'] == [-1.0, 2.0] and REF['role'].startswith('OBSERVED_REAL_SUCCESSES_ONLY')


def test_a_centred_look_that_would_clip_after_the_descent_asks_for_the_existing_realign_only():
    look = dict(ok=True, lateral_error_mm=-5.15, wrist_object=dict(visible=True, width_px=160, touches_border=False))
    c = L.candidate(look, 154.0, REF)
    assert c['status'] == 'FOV_REALIGN_CANDIDATE' and c['executable'] is False
    assert 'EXISTING visual_align' in c['ask'] and c['prediction']['pred_edges_px'][0] <= 1
    assert c['prediction']['inside_observed_pre_err_range'] is False


def test_a_look_that_stays_inside_changes_nothing_and_a_failed_check_is_left_to_the_existing_path():
    inside = dict(ok=True, lateral_error_mm=0.3, wrist_object=dict(visible=True, width_px=165, touches_border=False))
    assert L.candidate(inside, 154.0, REF)['status'] == 'NO_CHANGE'
    failed = dict(ok=False, lateral_error_mm=-8.0, wrist_object=dict(visible=True, width_px=160, touches_border=False))
    assert L.candidate(failed, 154.0, REF)['status'] == 'EXISTING_CHECK_ALREADY_REALIGNS'


def test_a_clipped_or_unreadable_look_is_never_given_a_centre():
    clipped = dict(ok=False, lateral_error_mm=None, wrist_object=dict(visible=True, width_px=262, edges_px=[0, 262], touches_border=True))
    assert L.predict_final_view(clipped, 154.0, REF) is None
    assert L.candidate(clipped, 154.0, REF)['status'] == 'UNREADABLE'


def test_no_real_success_means_no_reference():
    with pytest.raises(ValueError):
        L.reference_from_states([('failed', state(-5.0, 160, None, 262, final_ok=False))])


def test_the_executor_does_not_import_the_candidate():
    here = Path(__file__).resolve().parent
    for name in ('sweepick_pick01.py', 'sweepick_full_chain.py', 'sweepick_move_a.py', 'sweepick_episode.py'):
        assert 'left_fov_check' not in (here / name).read_text()
    assert 'left_fov_check' not in (here.parent / 'RUN_APPROVED_SESSION.py').read_text()
