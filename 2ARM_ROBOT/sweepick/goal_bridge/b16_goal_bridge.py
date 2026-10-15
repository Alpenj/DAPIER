"""Stored-observation goal contracts and diagnostics over A's unchanged pure planner."""

from __future__ import annotations

import copy

import hashlib

import importlib.util

import json

import math

from pathlib import Path
from decimal import Decimal

import sys

import numpy as np

HERE = Path(__file__).resolve().parent

MODEL_FRAME = 'assembled dual model world (left base at its model pose)'

SIDES = ('left', 'right')

def read_json(path):
    return json.loads(Path(path).read_text())

def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')

def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module

def stamp_difference(later, earlier):
    """Signed seconds only for qualified, identical clocks; units may differ."""
    scales = {'s': '1', 'ms': '.001', 'us': '.000001', 'ns': '.000000001'}
    for stamp in (later, earlier):
        if not isinstance(stamp, dict):
            raise ValueError('timestamp must be a mapping')
        value = stamp.get('value')
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)):
            raise ValueError('finite numeric timestamp required')
    if any(s.get('value') is None for s in (later, earlier)):
        return dict(seconds=None, reason='TIME_UNKNOWN', comparable=False)
    clock = later.get('clock')
    if (not isinstance(clock, str) or not clock or 'UNKNOWN' in clock.upper()
            or 'UNAVAILABLE' in clock.upper() or clock != earlier.get('clock')
            or any(s.get('unit') not in scales for s in (later, earlier))):
        return dict(seconds=None, reason='CLOCKS_UNCOMPARABLE', comparable=False)
    delta = (Decimal(str(later['value'])) * Decimal(scales[later['unit']])
             - Decimal(str(earlier['value'])) * Decimal(scales[earlier['unit']]))
    return dict(seconds=float(delta), reason=None, comparable=True, clock=clock)


def temporal_alignment(times):
    """Receipt chronology is diagnostic; unknown measurement/exposure leaves pose uncertainty."""
    feedback = times.get('joint_state', {})
    receipt_delta = stamp_difference(feedback.get('right', {}), feedback.get('left', {}))
    measurement = {s: feedback.get(s, {}).get('measurement_time') or {} for s in SIDES}
    measured_delta = stamp_difference(measurement['right'], measurement['left'])
    diagnostics, uncertainty = [], []
    if receipt_delta['seconds'] is None:
        diagnostics.append('JOINT_TIME_UNKNOWN' if receipt_delta['reason'] == 'TIME_UNKNOWN'
                           else 'JOINT_CLOCKS_UNCOMPARABLE')
    elif receipt_delta['seconds'] != 0:
        diagnostics.append('ASYNCHRONOUS_BIMANUAL_FEEDBACK')
    if measured_delta['seconds'] is None:
        uncertainty.append('JOINT_MEASUREMENT_TIME_UNKNOWN_OR_UNMAPPED')
    cameras = []
    for sensor in times.get('sensors', []):
        refs = sensor.get('state_references')
        if refs is None:
            refs = {'UNQUALIFIED_LEGACY_REFERENCE': sensor.get('state_reference') or {}}
        relations = {side: stamp_difference(sensor, stamp) for side, stamp in refs.items()}
        for relation in relations.values():
            if relation['seconds'] is None:
                diagnostics.append('SENSOR_STATE_ALIGNMENT_UNKNOWN' if relation['reason'] == 'TIME_UNKNOWN'
                                   else 'SENSOR_STATE_CLOCKS_UNCOMPARABLE')
                uncertainty.append('CAMERA_STATE_TIME_UNMAPPED')
            elif relation['seconds'] < 0:
                diagnostics.append('CAMERA_RECEIVED_BEFORE_REFERENCED_STATE')
        exposure = sensor.get('exposure_time') or {}
        exposure_relations = {side: stamp_difference(exposure, measurement.get(side, {})) for side in refs}
        exposure_verified = sensor.get('native_exposure_known') is True
        if not exposure_verified or any(not x['comparable'] for x in exposure_relations.values()):
            uncertainty.append('NATIVE_EXPOSURE_ALIGNMENT_UNKNOWN')
        cameras.append(dict(view=sensor.get('view'), receipt=copy.deepcopy(sensor),
            receipt_minus_state_s=relations, exposure_minus_joint_measurement_s=exposure_relations,
            actual_exposure_verified=exposure_verified))
    if not cameras:
        uncertainty.append('SENSOR_STATE_ALIGNMENT_UNKNOWN')
    return dict(joint_state=copy.deepcopy(feedback),
        right_minus_left_receipt_s=receipt_delta, right_minus_left_measurement_s=measured_delta,
        cameras=cameras, diagnostics=list(dict.fromkeys(diagnostics)),
        geometric_uncertainty=list(dict.fromkeys(uncertainty)),
        unknown_motion_displacement=True, physical_error_bound=None,
        uncertainty_role='no motion/latency bound inferred from receipt ordering',
        blocks_model_preview=False, new_numeric_hard_gate=False)


def timing_deficits(times):
    """Compatibility diagnostic API; callers must not use receipt ordering as a goal gate."""
    report = temporal_alignment(times)
    return report['geometric_uncertainty']

class GoalProposalBridge:
    """P0: retain missing geometry; only consume an evidenced A relative-view target."""

    def __init__(self, a_contract=None):
        self.seen = set()
        self.a_contract = a_contract or load_module(
            'b17_a_goal_contract', HERE.parent/'native/revision/sweepick_full_chain.py')

    def propose(self, record, preview=None):
        r = copy.deepcopy(record)
        identity = (r.get('run_id'), r.get('observation_id'))
        invalid, missing = [], []
        if not all(isinstance(x, str) and x for x in identity):
            raise ValueError('nonempty run_id and observation_id strings are required')
        if identity in self.seen:
            invalid.append('DUPLICATE_OBSERVATION_ID_WITHIN_RUN')
        self.seen.add(identity)
        if r.get('measurement_source_kinds') != ['REAL']:
            invalid.append('REAL_SIM_OR_UNKNOWN_MEASUREMENT_PROVENANCE')
        if not r.get('target_object'):
            missing.append('TARGET_OBJECT_IDENTITY_MISSING')
        try:
            state = np.asarray(r.get('canonical_state'), float)
            if state.shape != (12,) or not np.isfinite(state).all():
                raise ValueError('finite measured q12 is required')
        except (ValueError, TypeError):
            invalid.append('MEASURED_STATE_MALFORMED')
        frame = r.get('coordinate_frame')
        if frame is None:
            missing.append('TARGET_COORDINATE_FRAME_MISSING')
        elif frame != MODEL_FRAME:
            invalid.append('UNSUPPORTED_A_COORDINATE_FRAME')
        held = r.get('held_pose') or {}
        geometry = r.get('geometry_evidence') or {}
        if held.get('status') == 'MISSING':
            missing.append('HELD_POSE_MISSING')
        if held.get('object_pose_model') is None:
            missing.append('OBJECT_POSITION_MISSING')
        else:
            try:
                pose = np.asarray(held['object_pose_model'], float)
                if pose.shape != (4, 4) or not np.isfinite(pose).all():
                    raise ValueError('finite rigid object transform is required')
            except (ValueError, TypeError):
                invalid.append('OBJECT_POSE_MALFORMED')
        if geometry.get('centre_identified') is not True:
            missing.append('HELD_CENTRE_NOT_OBSERVED')
        if geometry.get('full_rotation_observed') is not True:
            missing.append('HELD_ORIENTATION_NOT_OBSERVED')
        if geometry.get('pad_object_relation_status') != 'OK':
            missing.append('INNER_PAD_OBJECT_RELATION_MISSING')
        times = r.get('reference_times') or {}
        try:
            for stamp in list(times.get('joint_state', {}).values()) + times.get('sensors', []):
                value = stamp.get('value')
                if value is not None and (isinstance(value, bool) or not math.isfinite(value)):
                    raise ValueError('finite timestamp required')
            time_report = temporal_alignment(times)
        except (TypeError, ValueError):
            invalid.append('REFERENCE_TIME_MALFORMED')
            times = {}
            time_report = dict(status='INVALID', diagnostics=[], geometric_uncertainty=['REFERENCE_TIME_MALFORMED'])
        candidate = r.get('a_observation_candidate')
        if candidate is None:
            missing.append('EVIDENCED_A_OBSERVATION_TARGET_MISSING')
        else:
            candidate_errors = self.a_contract.observation_candidate_errors(candidate)
            invalid.extend(candidate_errors)
            if not candidate_errors and any(t['reference_frame'] != 'CURRENT_TOOL' for t in candidate['targets']):
                invalid.append('P0_REQUIRES_A_CURRENT_TOOL_TARGETS')
        result = dict(
            schema='sweepick.b17.goal-proposal/1', type='GoalProposal',
            origin='B_GOAL_PROPOSAL', proposal_id=':'.join(str(x) for x in identity),
            task_goal='OBSERVE_HELD_OBJECT before right receive', primitive='OBSERVE_HELD_OBJECT',
            target_object=copy.deepcopy(r.get('target_object')),
            target_arm='UNKNOWN', coordinate_frame=frame,
            target_reference_frame='MISSING', goal_expression='MISSING', targets=[],
            observation_policy='FINAL_TARGET_ONLY',
            reference_times=times, canonical_state=None if 'MEASURED_STATE_MALFORMED' in invalid else r.get('canonical_state'),
            provenance=copy.deepcopy(r.get('provenance', {})),
            geometry_evidence=geometry,
            temporal_alignment=time_report,
            physical_execution_requirements=dict(
                calibration_verified=geometry.get('calibration_verified') is True,
                temporal_uncertainty=time_report['geometric_uncertainty'],
                geometry_deficits=list(dict.fromkeys(missing)), physical_approval=False),
            goal_evidence_role='OBSERVATION_INCOMPLETE' if missing else 'UNAPPROVED_OBSERVATION_INPUT',
            model_view_candidate=None,
            unknown_geometry=list(dict.fromkeys(missing)),
            required_additional_observations=copy.deepcopy(r.get('required_additional_observations', [])),
            source=dict(run_id=r.get('run_id'), observation_id=r.get('observation_id'),
                        source_sha256=r.get('source_sha256'), source_files=r.get('source_files', [])),
            planning_preview=dict(status='NOT_RUN', reason='missing or invalid evidence', model_only=True),
            status='EXCLUDED' if invalid else 'NEEDS_OBSERVATION', invalid_reasons=invalid,
            shadow_only=True, executable=False, motor_write_authorized=False,
            insertion_requested=False, motion_requested=False,
        )
        if invalid:
            return result
        basic_missing = {'TARGET_OBJECT_IDENTITY_MISSING', 'TARGET_COORDINATE_FRAME_MISSING',
                         'EVIDENCED_A_OBSERVATION_TARGET_MISSING'}
        model_context = r.get('model_preview_context') or {}
        if basic_missing.intersection(missing) or (missing and model_context.get('available') is not True):
            return result
        targets = copy.deepcopy(candidate['targets'])
        result['model_view_candidate'] = dict(role='MODEL_VIEW_CANDIDATE', targets=targets,
            context=copy.deepcopy(model_context), measured_goal=False,
            information_gain_proven=False, executable=False)
        if not missing:
            result['targets'] = targets
        if not missing:
            arms = set(t['side'] for t in targets)
            result.update(target_arm=next(iter(arms)) if len(arms) == 1 else 'both',
                          target_reference_frame='CURRENT_TOOL', goal_expression='A_RELATIVE_TOOL',
                          status='PREVIEW_UNSUPPORTED')
        if preview is None:
            result['planning_preview']['reason'] = 'A pure preview callback not supplied'
            return result
        plan = preview(copy.deepcopy(targets))
        result['planning_preview'] = dict(plan, model_only=True)
        if not missing:
            result['status'] = ('MODEL_PREVIEW_ONLY' if plan.get('ik_solution_exists') is True
                                and plan.get('collision_result') == 'CLEAR' else 'PREVIEW_FAILED')
        return result

def b15_times(prediction):
    feedback = prediction['actual_feedback']['per_arm']
    clock = f"{prediction['source_clock']['host_id']}:{prediction['source_clock']['run_id']}:HOST_MONOTONIC"
    joints = {s: dict(value=feedback[s]['feedback_received_ns'], unit='ns', clock=clock,
                     kind='HOST_FEEDBACK_RECEIPT', measurement_time=None) for s in SIDES}
    sensors = []
    for view, camera in prediction['camera_provenance'].items():
        arm = 'left' if view in ('left', 'left_wrist') else 'right' if view in ('right', 'right_wrist') else None
        sensors.append(dict(view=view, value=camera.get('received_ns'), unit='ns', clock=clock,
            kind='HOST_CAMERA_RECEIPT',
            state_references={s: copy.deepcopy(joints[s]) for s in ((arm,) if arm else SIDES)},
            native_exposure_known=False, exposure_time=None,
            native_capture=camera.get('capture_native')))
    return dict(joint_state=joints, sensors=sensors,
                pose=dict(value=None, unit=None, clock='UNKNOWN', status='MISSING'))

def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value
