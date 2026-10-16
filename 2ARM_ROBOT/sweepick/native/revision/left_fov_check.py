"""2026-10-16 LEFT CLOSE FOV — OFFLINE CANDIDATE, NOT EXECUTABLE. Nothing in the executor imports this module.

Run 20261016_112020_4d7c2b: the pre-grasp look passed the existing centring check at -5.15 mm (allowed 5.56 mm, the pad room
of the wider 3492-tick opening), no re-centring ran, and after the final descent the real block ran off the LEFT image border
(edges [0, 262], ready_to_close=false). Every stored REAL run whose final look stayed readable had entered the final descent
from a pre-grasp error inside [-2.80, +2.87] mm.

This module predicts, from a pre-grasp look alone, where the block's edges will be in the final descended look, using ONLY
what the stored REAL successes show (final/pre-grasp width ratio and the lateral change between the two looks; medians).
'Touching the border' is the existing wrist_object definition (xl <= 1 or xr >= width - 2). It adds no torque, speed,
tolerance or collision value, it does not relax ready_to_close and it never treats a clipped centroid as the block centre.
The only action it can name is the EXISTING visual_align at the pre-grasp height toward the EXISTING lateral target."""
import json
import statistics
from pathlib import Path

IMAGE_WIDTH_PX = 320                      # the left wrist stream (YUYV 320x240), as recorded in CAMERA_BINDING.json
OBJECT_WIDTH_MM = 40.0                    # the existing A block model (half extent 0.02 m), as wrist_object uses it


def reference_from_states(states):
    """The pre-grasp -> final relation of stored REAL runs whose final look was readable (ready_to_close ok)."""
    rows = []
    for name, st in states:
        ev = st.get('evidence') or []
        final = next((e for e in ev if e.get('what') == 'ready_to_close'), None)
        pre = [e for e in ev if e.get('what') == 'ready_to_descend']
        if not final or not final.get('ok') or not pre or final.get('lateral_error_mm') is None:
            continue
        p = pre[-1]
        rows.append(dict(session=name, pre_err_mm=p['lateral_error_mm'], pre_width_px=p['wrist_object']['width_px'],
                         final_err_mm=final['lateral_error_mm'], final_width_px=final['wrist_object']['width_px'],
                         final_edges_px=final['wrist_object']['edges_px']))
    if not rows:
        raise ValueError('no stored REAL run with a readable final look')
    ratios = [r['final_width_px'] / r['pre_width_px'] for r in rows]
    drifts = [r['final_err_mm'] - r['pre_err_mm'] for r in rows]
    return dict(schema='sweepick.left-fov-reference.v1', role='OBSERVED_REAL_SUCCESSES_ONLY; not a tolerance',
                runs=rows, width_ratio_median=statistics.median(ratios), width_ratio_range=[min(ratios), max(ratios)],
                lateral_change_mm_median=statistics.median(drifts), lateral_change_mm_range=[min(drifts), max(drifts)],
                pre_err_mm_range=[min(r['pre_err_mm'] for r in rows), max(r['pre_err_mm'] for r in rows)])


def predict_final_view(pre_look, midline_px, reference):
    """Predicted final-look edges from a pre-grasp evidence row (wrist_object + lateral_error_mm). None when unreadable."""
    obj = pre_look.get('wrist_object') or {}
    err = pre_look.get('lateral_error_mm')
    if not obj.get('visible') or obj.get('touches_border') or err is None or midline_px is None:
        return None
    width = reference['width_ratio_median'] * obj['width_px']
    px_per_mm = width / OBJECT_WIDTH_MM
    centre = midline_px + (err + reference['lateral_change_mm_median']) * px_per_mm
    left, right = centre - width / 2, centre + width / 2
    return dict(pred_width_px=width, pred_edges_px=[left, right],
                pred_touches_border=bool(left <= 1 or right >= IMAGE_WIDTH_PX - 2),
                inside_observed_pre_err_range=reference['pre_err_mm_range'][0] <= err <= reference['pre_err_mm_range'][1],
                basis='median width ratio and median lateral change of the stored REAL successes')


def candidate(pre_look, midline_px, reference):
    """What the candidate would ask for. Always executable=False."""
    pred = predict_final_view(pre_look, midline_px, reference)
    if pred is None:
        status, ask = 'UNREADABLE', 'none (the existing checks already stop on an unreadable look)'
    elif pre_look.get('ok') is False:
        status, ask = 'EXISTING_CHECK_ALREADY_REALIGNS', 'none (the existing centring check already asks for visual_align)'
    elif pred['pred_touches_border']:
        status, ask = 'FOV_REALIGN_CANDIDATE', 'the EXISTING visual_align at the pre-grasp height toward the EXISTING lateral target'
    else:
        status, ask = 'NO_CHANGE', 'none'
    return dict(status=status, ask=ask, prediction=pred, executable=False,
                not_changed='ready_to_close, the centring tolerance, the lateral target, IK, collision checks, torque and speed')


def load_states(paths):
    return [(str(p), json.loads(Path(p).read_text())) for p in paths]
