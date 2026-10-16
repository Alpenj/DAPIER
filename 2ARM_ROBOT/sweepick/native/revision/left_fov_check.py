"""2026-10-16 LEFT CLOSE FOV — will the block stay inside the left wrist image after the descent from the pre-grasp height?

Run 20261016_112020_4d7c2b: the pre-grasp look passed the existing centring check (-5.15 mm, allowed 5.56 mm), no re-centring
ran, and after the final descent the real block ran off the LEFT image border (edges [0, 262], ready_to_close=false).

The prediction uses ONLY what stored REAL runs with a readable final look show: the final/pre-grasp width ratio and the change
of the block's RAW image offset from the jaw midline between the two looks (medians). Inputs are kept apart:
  raw image position   = jaw midline px + wrist_offset.offset_mm (where the block IS in the image)
  lateral_error_mm     = offset_mm - lateral_target_mm (what the existing 1-axis loop corrects)
'Touching the border' is the existing wrist_object definition (xl <= 1 or xr >= width - 2). No torque, speed, tolerance or
collision value is added; ready_to_close is not relaxed; a clipped centroid is never taken as the block centre."""
import json
import statistics
from pathlib import Path

IMAGE_WIDTH_PX = 320                      # the left wrist stream (YUYV 320x240), as recorded in CAMERA_BINDING.json
OBJECT_WIDTH_MM = 40.0                    # the existing A block model (half extent 0.02 m), as wrist_object uses it
REFERENCE_FILE = Path(__file__).resolve().parent / 'sweepick_left_fov_reference.json'


def _raw_offset(row):
    off = row.get('wrist_offset') or {}
    return off.get('offset_mm') if off.get('readable') else None


def reference_from_states(states):
    """The pre-grasp -> final relation of stored REAL runs whose final look was readable (ready_to_close ok)."""
    rows = []
    for name, st in states:
        ev = st.get('evidence') or []
        final = next((e for e in ev if e.get('what') == 'ready_to_close'), None)
        pre = [e for e in ev if e.get('what') == 'ready_to_descend']
        if not final or not final.get('ok') or not pre or _raw_offset(final) is None or _raw_offset(pre[-1]) is None:
            continue
        p = pre[-1]
        rows.append(dict(session=name, pre_offset_mm=_raw_offset(p), pre_lateral_error_mm=p.get('lateral_error_mm'),
                         pre_lateral_target_mm=p.get('lateral_target_mm'), pre_width_px=p['wrist_object']['width_px'],
                         final_offset_mm=_raw_offset(final), final_width_px=final['wrist_object']['width_px'],
                         final_edges_px=final['wrist_object']['edges_px'], midline_px=(st.get('jaw_reference') or {}).get('midline_px')))
    if not rows:
        raise ValueError('no stored REAL run with a readable final look')
    ratios = [r['final_width_px'] / r['pre_width_px'] for r in rows]
    changes = [r['final_offset_mm'] - r['pre_offset_mm'] for r in rows]
    return dict(schema='sweepick.left-fov-reference.v2', role='OBSERVED_REAL_RUNS_WITH_A_READABLE_FINAL_LOOK; not a tolerance',
                runs=rows, width_ratio_median=statistics.median(ratios), width_ratio_range=[min(ratios), max(ratios)],
                offset_change_mm_median=statistics.median(changes), offset_change_mm_range=[min(changes), max(changes)],
                pre_offset_mm_range=[min(r['pre_offset_mm'] for r in rows), max(r['pre_offset_mm'] for r in rows)])


def load_reference(path=REFERENCE_FILE):
    try:
        ref = json.loads(Path(path).read_text())
        return ref if ref.get('schema') == 'sweepick.left-fov-reference.v2' and ref.get('runs') else None
    except (OSError, ValueError, AttributeError):
        return None


def predict_final_view(pre_look, midline_px, reference):
    """Predicted final-look edges from a pre-grasp evidence row (wrist_object + RAW wrist_offset). None when unreadable."""
    obj = pre_look.get('wrist_object') or {}
    offset = _raw_offset(pre_look)
    if not obj.get('visible') or not obj.get('stable', True) or obj.get('touches_border') or offset is None or midline_px is None or not reference:
        return None
    width = reference['width_ratio_median'] * obj['width_px']
    px_per_mm = width / OBJECT_WIDTH_MM
    centre = midline_px + (offset + reference['offset_change_mm_median']) * px_per_mm
    left, right = centre - width / 2, centre + width / 2
    return dict(inputs=dict(raw_centre_px=obj.get('centre_px'), jaw_midline_px=midline_px, raw_offset_mm=offset,
                            lateral_target_mm=pre_look.get('lateral_target_mm'), lateral_error_mm=pre_look.get('lateral_error_mm')),
                pred_width_px=width, pred_edges_px=[left, right],
                pred_touches_border=bool(left <= 1 or right >= IMAGE_WIDTH_PX - 2),
                inside_observed_pre_offset_range=reference['pre_offset_mm_range'][0] <= offset <= reference['pre_offset_mm_range'][1],
                basis='median width ratio and median raw-offset change of the stored REAL runs with a readable final look')


def candidate(pre_look, midline_px, reference):
    """Offline classification of one pre-grasp look. Always executable=False (the executor has its own path)."""
    pred = predict_final_view(pre_look, midline_px, reference)
    if pred is None:
        status, ask = 'UNREADABLE', 'none (the existing checks already stop on an unreadable look)'
    elif pre_look.get('ok') is False:
        status, ask = 'EXISTING_CHECK_ALREADY_REALIGNS', 'none (the existing centring check already asks for visual_align)'
    elif pred['pred_touches_border']:
        status, ask = 'FOV_REALIGN_CANDIDATE', 'reduce the existing lateral error at the pre-grasp height through correct()/plan_pick/align'
    else:
        status, ask = 'NO_CHANGE', 'none'
    return dict(status=status, ask=ask, prediction=pred, executable=False,
                not_changed='ready_to_close, the centring tolerance, the lateral target, IK, collision checks, torque and speed')


def load_states(paths):
    return [(str(p), json.loads(Path(p).read_text())) for p in paths]
