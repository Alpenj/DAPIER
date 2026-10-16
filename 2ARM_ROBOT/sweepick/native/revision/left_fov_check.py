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
import math
import statistics
from pathlib import Path

IMAGE_WIDTH_PX = 320                      # the left wrist stream (YUYV 320x240), as recorded in CAMERA_BINDING.json
OBJECT_WIDTH_MM = 40.0                    # the existing A block model (half extent 0.02 m), as wrist_object uses it
REFERENCE_FILE = Path(__file__).resolve().parent / 'sweepick_left_fov_reference.json'


def _sha256(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def _raw_offset(row):
    off = row.get('wrist_offset') or {}
    return off.get('offset_mm') if off.get('readable') else None


def reference_from_states(states):
    """The pre-grasp -> final relation of stored REAL runs whose final look was readable (ready_to_close ok)."""
    rows, contexts = [], []
    for name, st in states:
        ev = st.get('evidence') or []
        final = next((e for e in ev if e.get('what') == 'ready_to_close'), None)
        pre = [e for e in ev if e.get('what') == 'ready_to_descend']
        if not final or not final.get('ok') or not pre or _raw_offset(final) is None or _raw_offset(pre[-1]) is None:
            continue
        p = pre[-1]
        contexts.append(p.get('fov_context'))
        rows.append(dict(session=name, pre_offset_mm=_raw_offset(p), pre_lateral_error_mm=p.get('lateral_error_mm'),
                         pre_lateral_target_mm=p.get('lateral_target_mm'), pre_width_px=p['wrist_object']['width_px'],
                         final_offset_mm=_raw_offset(final), final_width_px=final['wrist_object']['width_px'],
                         final_edges_px=final['wrist_object']['edges_px'], midline_px=(st.get('jaw_reference') or {}).get('midline_px')))
    if not rows:
        raise ValueError('no stored REAL run with a readable final look')
    ratios = [r['final_width_px'] / r['pre_width_px'] for r in rows]
    changes = [r['final_offset_mm'] - r['pre_offset_mm'] for r in rows]
    reference = dict(schema='sweepick.left-fov-reference.v2', role='OBSERVED_REAL_RUNS_WITH_A_READABLE_FINAL_LOOK; not a tolerance',
                runs=rows, width_ratio_median=statistics.median(ratios), width_ratio_range=[min(ratios), max(ratios)],
                offset_change_mm_median=statistics.median(changes), offset_change_mm_range=[min(changes), max(changes)],
                pre_offset_mm_range=[min(r['pre_offset_mm'] for r in rows), max(r['pre_offset_mm'] for r in rows)])
    if contexts and contexts[0] is not None and all(c == contexts[0] for c in contexts):
        reference['context'] = contexts[0]
    return reference


def load_reference(path=REFERENCE_FILE):
    try:
        ref = json.loads(Path(path).read_text())
        ranges = [ref.get(k) for k in ('width_ratio_range', 'offset_change_mm_range', 'pre_offset_mm_range')]
        scalars = [ref.get(k) for k in ('width_ratio_median', 'offset_change_mm_median')]
        valid = (isinstance(ref, dict) and ref.get('schema') == 'sweepick.left-fov-reference.v2' and isinstance(ref.get('runs'), list) and ref['runs']
                 and all(isinstance(r, list) and len(r) == 2 and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in r) and r[0] <= r[1] for r in ranges)
                 and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in scalars)
                 and ref['width_ratio_median'] > 0)
        return ref if valid else None
    except (OSError, ValueError, AttributeError, TypeError, OverflowError):
        return None


def predict_final_view(pre_look, midline_px, reference):
    """Predicted final-look edges from a pre-grasp evidence row (wrist_object + RAW wrist_offset). None when unreadable."""
    obj = pre_look.get('wrist_object') or {}
    offset = _raw_offset(pre_look)
    values = (offset, midline_px, obj.get('width_px'), (reference or {}).get('width_ratio_median'), (reference or {}).get('offset_change_mm_median'))
    if (not obj.get('visible') or not obj.get('stable', True) or obj.get('touches_border') or not (pre_look.get('wrist_offset') or {}).get('reference_confirmed_in_this_frame')
            or any(not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v) for v in values) or obj.get('width_px', 0) <= 0):
        return None
    width = reference['width_ratio_median'] * obj['width_px']
    px_per_mm = width / OBJECT_WIDTH_MM
    centre = midline_px + (offset + reference['offset_change_mm_median']) * px_per_mm
    left, right = centre - width / 2, centre + width / 2
    if not all(math.isfinite(v) for v in (width, centre, left, right)):
        return None
    return dict(inputs=dict(raw_centre_px=obj.get('centre_px'), jaw_midline_px=midline_px, raw_offset_mm=offset,
                            lateral_target_mm=pre_look.get('lateral_target_mm'), lateral_error_mm=pre_look.get('lateral_error_mm')),
                pred_width_px=width, pred_edges_px=[left, right],
                pred_touches_border=bool(left <= 1 or right >= IMAGE_WIDTH_PX - 2),
                inside_observed_pre_offset_range=reference['pre_offset_mm_range'][0] <= offset <= reference['pre_offset_mm_range'][1],
                basis='median width ratio and median raw-offset change of the stored REAL runs with a readable final look')


def context_issues(pre_look, reference):
    """Missing or mismatched capture identity stays diagnostic UNKNOWN; it never validates a prediction."""
    expected, current = (reference or {}).get('context'), pre_look.get('fov_context')
    issues = []
    for label, context in (('reference', expected), ('current look', current)):
        if not isinstance(context, dict):
            issues.append(f'{label} context is missing')
        else:
            if context.get('camera_side') != 'left_wrist':
                issues.append(f'{label} camera_side is not left_wrist')
            if not _sha256(context.get('mount_binding_sha256')):
                issues.append(f'{label} mount_binding_sha256 is not a SHA-256 identity')
            if context.get('image_size_px') != [IMAGE_WIDTH_PX, 240]:
                issues.append(f'{label} image_size_px does not match the 320x240 prediction')
            if context.get('object_width_mm') != OBJECT_WIDTH_MM:
                issues.append(f'{label} object_width_mm does not match the 40 mm prediction model')
    if isinstance(expected, dict) and isinstance(current, dict):
        issues.extend(f'{field} differs from the reference' for field in ('camera_side', 'mount_binding_sha256', 'image_size_px', 'object_width_mm') if expected.get(field) != current.get(field))
    frame = pre_look.get('frame_binding')
    if not isinstance(frame, dict):
        issues.append('current RAW RGB frame binding is missing')
    else:
        if not _sha256(frame.get('raw_rgb_sha256')):
            issues.append('current raw_rgb_sha256 is not a SHA-256 identity')
        if not isinstance(frame.get('frame_id'), str) or not frame['frame_id'].strip():
            issues.append('current frame_id is missing or malformed')
        if frame.get('camera_side') != 'left_wrist':
            issues.append('frame camera side is not left_wrist')
        elif isinstance(current, dict) and frame.get('camera_side') != current.get('camera_side'):
            issues.append('frame camera side differs from the current context')
    return issues


def candidate(pre_look, midline_px, reference):
    """Offline classification of one pre-grasp look. Always executable=False (the executor has its own path)."""
    pred = predict_final_view(pre_look, midline_px, reference)
    if pred is None:
        status, ask = 'UNREADABLE', 'none (the existing checks already stop on an unreadable look)'
    elif pre_look.get('ok') is False:
        status, ask = 'EXISTING_CHECK_ALREADY_REALIGNS', 'none (the existing centring check already asks for visual_align)'
    elif not pred['inside_observed_pre_offset_range']:
        status, ask = 'OUT_OF_SUPPORT', 'none (no correction is inferred outside the observed pre-offset range)'
    elif context_issues(pre_look, reference):
        status, ask = 'UNKNOWN', 'none (capture/reference identity is not confirmed)'
    elif pred['pred_touches_border']:
        status, ask = 'FOV_REALIGN_CANDIDATE', 'reduce the existing lateral error at the pre-grasp height through correct()/plan_pick/align'
    else:
        status, ask = 'NO_CHANGE', 'none'
    return dict(status=status, ask=ask, prediction=pred, executable=False,
                not_changed='ready_to_close, the centring tolerance, the lateral target, IK, collision checks, torque and speed')


def load_states(paths):
    return [(str(p), json.loads(Path(p).read_text())) for p in paths]
