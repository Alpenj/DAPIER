"""Read A metrology artifacts into B; candidate FK never becomes physical X evidence."""
import copy
import json
from pathlib import Path

import cv2
import numpy as np

from .right_relative import extract_right_features
from .target_image import (CURRENT_TARGET, TargetImageServoProposal, _json, _number,
                           _sample, snapshot_observation, source_blob)

A_CONTRACT_COMMIT = '0cb3ad4141ea2dcab970daed30235ed0f6e84567'
B_CONSUMER_COMMIT = '29cdf06a457274553cbcb85f76231ff43fc2fa3c'
ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
VIEWS = ('top', 'left_wrist', 'right_wrist')


def _object(value, label):
    if not isinstance(value, dict):
        raise ValueError(label + '_DOCUMENT_REQUIRED')
    return value


def _arm5(value):
    value = _object(value, 'ARM5_TICKS')
    if not all(_number(value.get(n)) for n in ARM):
        raise ValueError('FINITE_ARM5_TICKS_REQUIRED')
    return value


def _endpoint(root, row, moves, read, run_id, current_evidence, reference, jacobian, a_source):
    role = row['role']
    folder = row.get('decision_record')
    if not isinstance(folder, str) or not folder:
        raise ValueError('DECISION_RECORD_SOURCE_UNKNOWN')
    record_path = Path(folder) / 'OBSERVATION.json'
    record = read(record_path); d = _json(record)
    if d['stage'] != 'RIGHT_X_' + role:
        raise ValueError('ENDPOINT_STAGE_IDENTITY_MISMATCH')
    cameras = _object(row['cameras'], 'ENDPOINT_CAMERA')
    actual = _object(row['actual'], 'ENDPOINT_ACTUAL')
    feedback = _object(d['actual_feedback'], 'RECORDED_ACTUAL')
    for name in ('ticks', 'goal', 'torque', 'status', 'bus_seq', 't_bus_monotonic', 'cycle_start'):
        if name not in actual or name not in feedback or actual[name] != feedback[name]:
            raise ValueError('ENDPOINT_ACTUAL_SOURCE_MISMATCH: ' + name)
    if actual['q12'] != d['q12']:
        raise ValueError('ENDPOINT_Q12_SOURCE_MISMATCH')
    recorded_cameras = _object(d['camera_frames'], 'RECORDED_CAMERA')
    for name in VIEWS:
        if name not in cameras or cameras[name] != recorded_cameras.get(name):
            raise ValueError('ENDPOINT_CAMERA_SOURCE_MISMATCH: ' + name)
    _object(feedback['ticks'], 'PRESENT_TICKS')
    _object(feedback['goal'], 'GOAL_TICKS')
    opening = _object(row['opening'], 'ENDPOINT_OPENING')
    for side in ('left', 'right'):
        _object(feedback['ticks'][side], 'PRESENT_TICKS_SIDE')
        _object(feedback['goal'][side], 'GOAL_TICKS_SIDE')
        if (not _number(opening.get(side + '_present_tick'))
                or not _number(opening.get(side + '_goal_tick'))
                or opening.get(side + '_present_tick') != feedback['ticks'][side]['gripper']
                or opening.get(side + '_goal_tick') != feedback['goal'][side]['gripper']):
            raise ValueError('ENDPOINT_OPENING_SOURCE_MISMATCH')
    images = {}
    for name in VIEWS:
        image_name = d['files'][name]['rgb']
        images[name] = read(Path(folder) / image_name)
    right_name = d['files']['right_wrist']['rgb']
    sample = dict(record=record, image=dict(images['right_wrist'], name=right_name),
        observation_identity=dict(run_id=run_id, observation_id=Path(folder).name))
    observation, binding, measured_opening = snapshot_observation(sample)
    features = extract_right_features(observation, {}, {}, a_source=a_source)
    context_status = 'UNKNOWN'; context_error = None; context = None
    if current_evidence is not None:
        sample['evidence'] = current_evidence
        try:
            context = _sample(sample, role=CURRENT_TARGET, a_source=a_source)['context']
            context_status = 'VERIFIED_CURRENT_EVIDENCE'
        except (KeyError, ValueError, TypeError) as exc:
            context_error = str(exc)
    stage = dict(status='UNKNOWN', reason='T has no metrology move source; earlier approach is not inferred')
    if role != 'T':
        move = moves[role]; executor = _object(row['executor'], 'ENDPOINT_EXECUTOR')
        command = _object(row['command'], 'ENDPOINT_COMMAND')
        _object(move['candidate'], 'MOVE_CANDIDATE')
        if not isinstance(move['forward'], list) or len(move['forward']) != 1:
            raise ValueError('ONE_FORWARD_MOVE_REQUIRED')
        _object(move['forward'][0], 'FORWARD_MOVE')
        _arm5(command['target_ticks'])
        if (move['role'] != role or len(move['forward']) != 1
                or command['target_ticks'] != move['forward'][0]['target_ticks']
                or command['commanded_tool_displacement_m'] != move['candidate']['commanded_tool_displacement_m']):
            raise ValueError('COMMAND_SOURCE_MISMATCH')
        if (move.get('result') != executor.get('result') or move.get('writes') != executor.get('writes')):
            raise ValueError('STAGE_MOTION_SOURCE_MISMATCH')
        view = move.get('observation_view') or {}
        _object(view, 'STAGE_OBSERVATION_VIEW')
        if any(executor.get(n) != view.get(n) for n in ('nominal_target_reached', 'actual_settled', 'command_state_resolved')):
            raise ValueError('STAGE_VIEW_STATE_SOURCE_MISMATCH')
        static = (move.get('writes') is None and view.get('settling_evidence') ==
                  'existing POST_READS stationary register window; no command')
        if static:
            log_blob = read(Path(move['folder']) / 'STATIC_VIEW_READS.json')
            reads = json.loads(log_blob['bytes'])
            if not isinstance(reads, list) or not reads or not all(isinstance(r, dict) for r in reads):
                raise ValueError('STATIC_READ_SOURCE_REQUIRED')
            last_raw = reads[-1]
            present = _arm5(_object(last_raw['ticks'], 'STATIC_TICKS')['right'])
            last = dict(present=present, t=last_raw.get('t_bus_monotonic'))
            target = _arm5(view['nominal_target'])
            result, writes, status = move.get('result'), None, 'STATIC_READ_SOURCE_BOUND'
            row_clock = 't_bus_monotonic (static native read)'
        else:
            log_blob = read(Path(move['folder']) / 'run/move.json'); log = _json(log_blob)
            if (log.get('side') != 'right' or log.get('result') != move.get('result')
                    or log.get('writes') != move.get('writes')):
                raise ValueError('STAGE_MOTION_SOURCE_MISMATCH')
            target = _arm5(_object(log['trajectory'], 'STAGE_TRAJECTORY')['target'])
            rows = log.get('rows', [])
            if not isinstance(rows, list):
                raise ValueError('STAGE_ROWS_REQUIRED')
            last = _object(rows[-1], 'STAGE_ROW') if rows else {}
            result, writes, status = log['result'], log['writes'], 'SOURCE_BOUND'
            row_clock = 'UNKNOWN_FROM_ARTIFACT'
        if any(target[n] != command['target_ticks'][n] for n in ARM):
            raise ValueError('STAGE_COMMAND_TARGET_SOURCE_MISMATCH')
        stage = dict(status=status, source_sha256=log_blob['sha256'], result=result,
            writes=writes, last_present=copy.deepcopy(last.get('present')),
            nominal_target_ticks=copy.deepcopy(target),
            last_row_time=last.get('t'), row_clock=row_clock,
            zero_command_static_contract=static,
            endpoint_feedback_is_separate_sample=True, endpoint_minus_stage_ticks=None)
        if last.get('present') is not None:
            present = _arm5(last['present'])
            stage['endpoint_minus_stage_ticks'] = {n: feedback['ticks']['right'][n] - present[n] for n in ARM}
    b_output = TargetImageServoProposal(sample, reference, jacobian, a_source=a_source)
    return dict(status='SOURCE_BOUND', role=role, snapshot=sample, binding=binding,
        actual=copy.deepcopy(actual), opening=copy.deepcopy(opening), physical_opening_mm=None,
        cameras=copy.deepcopy(cameras), image_sha256={name: blob['sha256'] for name, blob in images.items()},
        exposure_time='UNKNOWN', time_basis=row.get('time_basis', 'UNKNOWN'), stage_motion=stage,
        command=copy.deepcopy(row.get('command')), candidate_fk_from_previous=copy.deepcopy(row.get('actual_fk_from_previous')),
        candidate_fk_from_T=copy.deepcopy(row.get('actual_fk_from_T')), physical_X_mm=None,
        candidate_right_tool_pose_fk=copy.deepcopy(row.get('right_tool_pose_fk')), fk_role=row.get('fk_role', 'UNKNOWN'),
        source_kind=d['source_kind'], current_context=context,
        runtime_producer_revision='UNKNOWN', raw_centroid_u=features['object_component_centroid_uv']['u'],
        component_area_px=features['visible_approach_region']['component_area_px'],
        centroid_usable=features['feature_validity']['centroid_usable_for_control'],
        current_context_status=context_status, current_context_error=context_error,
        measured_opening_tick=measured_opening, B_output=b_output)


def _response(before, after, *, independent=False):
    output = dict(before=before.get('role'), after=after.get('role'),
        purpose='FIT_EXCLUDED_RESPONSE_CHECK' if independent else 'FIT_CANDIDATE',
        status='UNKNOWN', observed_delta_u_px=None, actual_delta_ARM5_ticks=None, actual_delta_right_q_rad=None,
        commanded_tool_displacement_m=None, physical_X_mm=None, detector_jitter_px=None,
        jitter_status='UNKNOWN_NO_STATIONARY_BOUND', observability='UNRESOLVED',
        REAL_J_VALIDATED=False, physical_px_per_mm_J=None)
    if before.get('status') != 'SOURCE_BOUND' or after.get('status') != 'SOURCE_BOUND':
        output['status'] = 'SOURCE_INCOMPLETE'; return output
    key = lambda value: (value['snapshot']['observation_identity']['run_id'], value['cameras']['right_wrist']['n'])
    if key(before) == key(after):
        output['status'] = 'CAPTURE_REUSED'; return output
    delta = np.asarray([after['actual']['ticks']['right'][n] - before['actual']['ticks']['right'][n] for n in ARM], float)
    q_delta = [after['actual']['q12'][n] - before['actual']['q12'][n] for n in range(6,11)]
    if not np.isfinite(delta).all() or not all(_number(n) for n in q_delta):
        output['status'] = 'NONFINITE_RESPONSE_DIFFERENCE'; return output
    output.update(actual_delta_ARM5_ticks=delta.tolist(), actual_delta_right_q_rad=q_delta,
        commanded_tool_displacement_m=(after.get('command') or {}).get('commanded_tool_displacement_m'),
        candidate_fk_only=copy.deepcopy(after.get('candidate_fk_from_previous')),
        area_ratio=None if not before['component_area_px'] else after['component_area_px'] / before['component_area_px'])
    flags = []
    if any(after['actual']['ticks']['left'][n] != before['actual']['ticks']['left'][n] for n in (*ARM, 'gripper')):
        flags.append('LEFT_HELD_OBJECT_MOTION_OR_OPENING_CHANGED')
    b, a = before['cameras']['right_wrist'], after['cameras']['right_wrist']
    if a['n'] <= b['n'] or a['recv_monotonic_s'] <= b['recv_monotonic_s']:
        flags.append('CAMERA_RESPONSE_ORDER_UNVERIFIED')
    if (after['actual']['bus_seq'] <= before['actual']['bus_seq']
            or after['actual']['t_bus_monotonic'] <= before['actual']['t_bus_monotonic']):
        flags.append('FEEDBACK_RESPONSE_ORDER_UNVERIFIED')
    if not before['centroid_usable'] or not after['centroid_usable']:
        flags.append('CLIPPED_OR_MISSING_COMPONENT')
    if before['current_context_status'] != 'VERIFIED_CURRENT_EVIDENCE' or after['current_context_status'] != 'VERIFIED_CURRENT_EVIDENCE':
        flags.append('OCCLUSION_OR_OBJECT_CONTEXT_UNVERIFIED')
    elif (before['current_context'] != after['current_context']
            or any(before['binding']['camera_state_binding'][k] != after['binding']['camera_state_binding'][k]
                   for k in ('image_shape', 'image_dtype'))):
        flags.append('CURRENT_OBJECT_FEATURE_MOUNT_MODE_OR_OPENING_CONTEXT_CHANGED')
    if before['measured_opening_tick'] != after['measured_opening_tick']:
        flags.append('OPENING_CHANGED')
    fk = after.get('candidate_fk_from_previous')
    rotation = fk.get('rotation_deg') if isinstance(fk, dict) else None
    output['candidate_rotation_deg'] = rotation
    if rotation is None:
        flags.append('ROTATION_UNKNOWN')
    elif not _number(rotation) or rotation != 0:
        flags.append('CANDIDATE_ROTATION_PRESENT_NOT_PHYSICAL_BOUND')
    if _number(before['raw_centroid_u']) and _number(after['raw_centroid_u']):
        output['observed_delta_u_px'] = after['raw_centroid_u'] - before['raw_centroid_u']
    else:
        flags.append('IMAGE_RESPONSE_UNKNOWN')
    if output['area_ratio'] is not None and output['area_ratio'] != 1:
        flags.append('APPARENT_AREA_CHANGED_SCALE_OR_OCCLUSION_UNRESOLVED')
    output.update(confounds=flags, status='CONFOUNDED_DIAGNOSTIC_ONLY' if flags else 'ENCODER_PIXEL_RESPONSE_RECORDED')
    return output


def bridge_metrology(session_root, *, run_id, expected_sha256, reference=None,
                     current_evidence_by_role=None, jacobian_evidence=None, a_source=None):
    """expected_sha256 maps session-relative source filenames to independently pinned hashes."""
    root = Path(session_root).resolve(); sources = {}
    output = dict(schema='sweepick.b20.a-metrology-bridge/1', status='NEEDS_SOURCE',
        A_schema_contract_commit=A_CONTRACT_COMMIT, B_consumer_commit=B_CONSUMER_COMMIT,
        runtime_producer_revision='UNKNOWN', endpoints={}, responses={}, missing_evidence=[],
        target_reference_created=False, MEASURED_PHYSICAL_X_RESPONSE_created=False,
        REAL_J_VALIDATED=False, Correction=None, executable=False, motor_write_authorized=False)

    def read(value):
        path = Path(value); path = path if path.is_absolute() else root / path
        path = path.resolve()
        if not path.is_relative_to(root):
            raise ValueError('SOURCE_PATH_OUTSIDE_SESSION')
        name = str(path.relative_to(root))
        if name not in expected_sha256:
            raise ValueError('SOURCE_SHA_UNKNOWN: ' + name)
        blob = source_blob(path.read_bytes())
        if blob['sha256'] != expected_sha256[name]:
            raise ValueError('SOURCE_SHA_MISMATCH: ' + name)
        sources[name] = blob['sha256']; return blob

    try:
        if not isinstance(run_id, str) or not run_id.strip() or not isinstance(expected_sha256, dict):
            raise ValueError('RUN_ID_AND_PINNED_SOURCE_MANIFEST_REQUIRED')
        if current_evidence_by_role is not None:
            _object(current_evidence_by_role, 'CURRENT_EVIDENCE_BY_ROLE')
        record = _json(read('RIGHT_X_METROLOGY.json'))
        if (record.get('schema') != 'sweepick.right-x-metrology.v1'
                or record.get('scope') != 'RIGHT_X_FINITE_METROLOGY'
                or record.get('source_kind') not in ('REAL', 'TEST_FIXTURE')):
            raise ValueError('A_METROLOGY_SCHEMA_OR_SOURCE_MISMATCH')
        output.update(A_record_status=record.get('status', 'UNKNOWN'), source_kind=record['source_kind'])
        endpoints = {}; moves = {}
        for item in record['endpoints']:
            _object(item, 'ENDPOINT')
            if item.get('role') not in ('T', 'F', 'V') or item['role'] in endpoints:
                raise ValueError('ENDPOINT_ROLE_DUPLICATED_OR_UNKNOWN')
            endpoints[item['role']] = item
        for item in record['moves']:
            _object(item, 'MOVE')
            if item.get('role') not in ('F', 'V') or item['role'] in moves:
                raise ValueError('MOVE_ROLE_DUPLICATED_OR_UNKNOWN')
            moves[item['role']] = item
        config = _object(record['config'], 'METROLOGY_CONFIG')
        complete = (config.get('verify_from') == 'F' and list(endpoints) == ['T', 'F', 'V']
                    and record.get('status') == 'ENDPOINTS_RECORDED')
        if not complete:
            output['missing_evidence'].append('INCOMPLETE_ORDERED_T_F_V_OR_VERIFY_FROM_F')
        for role in ('T', 'F', 'V'):
            try:
                row = _endpoint(root, endpoints[role], moves, read, run_id,
                    (current_evidence_by_role or {}).get(role), reference, jacobian_evidence, a_source)
                if row['snapshot']['record']['bytes'] and _json(row['snapshot']['record'])['source_kind'] != record['source_kind']:
                    raise ValueError('METROLOGY_OBSERVATION_PROVENANCE_MISMATCH')
                output['endpoints'][role] = row
            except (KeyError, ValueError, TypeError, OSError, cv2.error) as exc:
                output['endpoints'][role] = dict(role=role, status='UNKNOWN', missing_evidence=[str(exc)])
        t, f, v = (output['endpoints'][role] for role in ('T', 'F', 'V'))
        fit, check = _response(t, f), _response(f, v, independent=True)
        if all(point.get('status') == 'SOURCE_BOUND' for point in (t, f, v)):
            used = {t['cameras']['right_wrist']['n'], f['cameras']['right_wrist']['n']}
            if v['cameras']['right_wrist']['n'] in used:
                check.update(status='FIT_CAPTURE_REUSED', observed_delta_u_px=None)
        diagnostic = None
        if complete and fit['status'] == 'ENCODER_PIXEL_RESPONSE_RECORDED':
            delta = np.asarray(fit['actual_delta_ARM5_ticks'], float); norm = float(np.linalg.norm(delta))
            if np.isfinite(norm) and norm != 0:
                direction = delta / norm
                diagnostic = dict(role='ENCODER_PROJECTION_DIAGNOSTIC_NOT_CARTESIAN_CONTROL', ARM5_order=list(ARM),
                    encoder_basis=direction.tolist(), unit='pixel per Euclidean raw ARM5 tick-norm',
                    fit_px_per_encoder_tick_norm=fit['observed_delta_u_px'] / norm, fit_coordinate_tick_norm=norm,
                    independent_status='UNRESOLVED', physical_axis_verified=False, validated=False,
                    may_enter_REAL_px_per_mm_Jacobian=False, observable_for_control=False)
                if check['status'] == 'ENCODER_PIXEL_RESPONSE_RECORDED':
                    dv = np.asarray(check['actual_delta_ARM5_ticks'], float)
                    coordinate = float(dv @ direction); prediction = diagnostic['fit_px_per_encoder_tick_norm'] * coordinate
                    observed = check['observed_delta_u_px']
                    consistent = prediction != 0 and observed != 0 and np.sign(prediction) == np.sign(observed)
                    off_basis = float(np.linalg.norm(dv - coordinate * direction))
                    diagnostic.update(independent_coordinate_tick_norm=coordinate,
                        independent_off_basis_tick_norm=off_basis,
                        independent_observed_delta_u_px=observed, independent_predicted_delta_u_px=prediction,
                        independent_status=('OFF_BASIS_CONFOUNDED_DIAGNOSTIC_ONLY' if off_basis != 0 else
                            'DIAGNOSTIC_SIGN_CONSISTENT_NO_NOISE_BOUND' if consistent else 'SIGN_UNRESOLVED_OR_REVERSED'))
        output.update(status='SOURCE_BOUND' if complete and all(p.get('status') == 'SOURCE_BOUND' for p in (t, f, v)) else 'PARTIAL_SOURCE',
                      responses=dict(T_to_F=fit, F_to_V=check), encoder_diagnostic=diagnostic)
    except (KeyError, ValueError, TypeError, OSError) as exc:
        output['missing_evidence'].append(str(exc))
    output['source_sha256'] = sources
    return output
