"""A-shaped file fixtures; no A runtime, devices or physical confirmations."""
import copy
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib

import cv2
import numpy as np

from .metrology_bridge import ARM, VIEWS, bridge_metrology
from .target_image import CURRENT_TARGET, FIELD_TARGET, source_blob
from .test_target_image import document, evidence, json_blob, rebind, snapshot_fixture

RUN_ID = 'SYNTHETIC_METROLOGY_FIXTURE'


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(json_blob(value)['bytes'])


def fixture(root, *, shifts=(0, 4, 6), pans=(100, 110, 115)):
    endpoints, moves, snapshots = [], [], {}
    for i, (role, shift, pan) in enumerate(zip(('T', 'F', 'V'), shifts, pans)):
        sample = snapshot_fixture(role, shift)
        d = document(sample); d['stage'] = 'RIGHT_X_' + role
        folder = Path('decision_observations') / f'{i+1:06d}_RIGHT_X_{role}'
        sample['observation_identity'] = dict(run_id=RUN_ID, observation_id=folder.name)
        d['q12'][6] = (pan - 100) / 100; d['observation']['q12'] = copy.deepcopy(d['q12'])
        d['actual_feedback']['ticks']['right']['shoulder_pan'] = pan
        d['observation']['present_ticks'] = copy.deepcopy(d['actual_feedback']['ticks'])
        d['actual_feedback'].update(bus_seq=i+1, t_bus_monotonic=10.0+i, cycle_start=9.9+i,
            goal=copy.deepcopy(d['actual_feedback']['ticks']),
            torque={side: {n: 1 for n in d['actual_feedback']['ticks'][side]} for side in ('left', 'right')},
            status={side: {n: 0 for n in d['actual_feedback']['ticks'][side]} for side in ('left', 'right')})
        d['actual_feedback']['goal']['right']['shoulder_pan'] = pan + 2
        d['observation'].update(seq=i+1, feedback_stamp=10.0+i)
        d['files'] = {name: dict(rgb=name + '_rgb.png') for name in VIEWS}
        d['camera_frames'] = {name: dict(n=100+i, fresh=True, reader_valid=True,
            recv_monotonic_s=10.05+i) for name in VIEWS}
        sample['record'] = json_blob(d); rebind(sample)
        snapshots[role] = sample
        write_json(root / folder / 'OBSERVATION.json', d)
        for name in VIEWS:
            (root / folder / d['files'][name]['rgb']).write_bytes(sample['image']['bytes'])
        raw = d['actual_feedback']
        row = dict(role=role, decision_record=str(folder),
            actual={**{k: copy.deepcopy(raw[k]) for k in ('ticks', 'goal', 'torque', 'status', 'bus_seq', 't_bus_monotonic', 'cycle_start')},
                    'q12': d['q12']}, cameras=d['camera_frames'],
            opening={**{side+'_present_tick': raw['ticks'][side]['gripper'] for side in ('left', 'right')},
                     **{side+'_goal_tick': raw['goal'][side]['gripper'] for side in ('left', 'right')}},
            right_tool_pose_fk=np.eye(4).tolist(), fk_role='candidate mapping verified=false',
            time_basis='separate camera receipt and bus clocks; exposure unknown')
        if role != 'T':
            commanded = 0.005 if role == 'F' else 0.0025
            target = {n: raw['goal']['right'][n] for n in ARM}
            fk = dict(translation_tool_m=[commanded*0.8, 0, 0], along_x_m=commanded*0.8,
                off_axis_tool_m=[0, 0, 0], off_axis_norm_m=0, rotation_deg=0,
                frame='previous right_cube_grasp candidate FK')
            row.update(actual_fk_from_previous=fk, actual_fk_from_T=copy.deepcopy(fk),
                command=dict(commanded_tool_displacement_m=commanded, target_ticks=target,
                    command_fk_from_planning_actual=copy.deepcopy(fk)),
                executor=dict(result='OBSERVATION_VIEW_HELD', writes=dict(Goal_Position=1)))
            move_folder = 'stage_right_x_' + role
            log = dict(side='right', result=row['executor']['result'], writes=row['executor']['writes'],
                trajectory=dict(target=copy.deepcopy(target)),
                rows=[dict(t=0.5, present={n: raw['ticks']['right'][n]-1 for n in ARM})])
            write_json(root / move_folder / 'run/move.json', log)
            moves.append(dict(role=role, folder=move_folder, forward=[dict(target_ticks=target)],
                candidate=dict(commanded_tool_displacement_m=commanded),
                result=log['result'], writes=log['writes']))
        endpoints.append(row)
    record = dict(schema='sweepick.right-x-metrology.v1', scope='RIGHT_X_FINITE_METROLOGY',
        source_kind='TEST_FIXTURE', config=dict(verify_from='F'), status='ENDPOINTS_RECORDED',
        endpoints=endpoints, moves=moves)
    write_json(root / 'RIGHT_X_METROLOGY.json', record)
    target = copy.deepcopy(snapshots['T']); e = evidence(target); e['role'] = FIELD_TARGET
    rebind(target, e)
    return dict(root=root, record=record, snapshots=snapshots,
        reference=dict(role=FIELD_TARGET, reference_id='SYNTHETIC_NOT_REAL_TARGET', snapshot=target))


def pins(root):
    return {str(p.relative_to(root)): source_blob(p.read_bytes())['sha256']
            for p in root.rglob('*') if p.is_file()}


def run_fixture(case, *, reference=True, current=True, expected=None, **kw):
    return bridge_metrology(case['root'], run_id=RUN_ID, expected_sha256=pins(case['root']) if expected is None else expected,
        reference=case['reference'] if reference else None,
        current_evidence_by_role={k: v['evidence'] for k, v in case['snapshots'].items()} if current else None, **kw)


class MetrologyBridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.case = fixture(self.root)

    def save_record(self):
        write_json(self.root / 'RIGHT_X_METROLOGY.json', self.case['record'])

    def update_observation(self, role, mutate):
        row = next(r for r in self.case['record']['endpoints'] if r['role'] == role)
        sample = self.case['snapshots'][role]; d = document(sample); mutate(d)
        sample['record'] = json_blob(d); rebind(sample)
        write_json(self.root / row['decision_record'] / 'OBSERVATION.json', d)
        return row, sample, d

    def test_positive_source_map_and_encoder_diagnostic_never_physical_X(self):
        original = pins(self.root); out = run_fixture(self.case)
        self.assertEqual(out['status'], 'SOURCE_BOUND')
        self.assertEqual(original, pins(self.root))
        self.assertEqual([out['endpoints'][r]['B_output']['feature_error_px']['u'] for r in ('T','F','V')], [0,4,6])
        self.assertEqual(out['responses']['T_to_F']['actual_delta_ARM5_ticks'], [10,0,0,0,0])
        self.assertEqual(out['responses']['F_to_V']['observed_delta_u_px'], 2)
        self.assertAlmostEqual(out['encoder_diagnostic']['fit_px_per_encoder_tick_norm'], 0.4)
        self.assertEqual(out['encoder_diagnostic']['independent_status'], 'DIAGNOSTIC_SIGN_CONSISTENT_NO_NOISE_BOUND')
        f = out['endpoints']['F']
        self.assertEqual(f['stage_motion']['endpoint_minus_stage_ticks']['shoulder_pan'], 1)
        self.assertNotEqual(f['actual']['ticks']['right']['shoulder_pan'], f['command']['target_ticks']['shoulder_pan'])
        self.assertIsNone(out['Correction']); self.assertFalse(out['REAL_J_VALIDATED'])
        self.assertFalse(out['MEASURED_PHYSICAL_X_RESPONSE_created'])
        for row in out['endpoints'].values():
            self.assertIsNone(row['physical_X_mm']); self.assertIsNone(row['physical_opening_mm'])
            self.assertIsNone(row['B_output']['estimated_tool_correction'])

    def test_no_reference_is_invented_and_current_evidence_is_not_copied(self):
        out = run_fixture(self.case, reference=False, current=False)
        self.assertFalse(out['target_reference_created']); self.assertIsNone(out['encoder_diagnostic'])
        for row in out['endpoints'].values():
            self.assertNotIn('evidence', row['snapshot'])
            self.assertIsNone(row['B_output']['feature_error_px']['u'])
        copied = self.case['reference']['snapshot']['evidence']
        for sample in self.case['snapshots'].values(): sample['evidence'] = copied
        out = run_fixture(self.case)
        self.assertTrue(all(r['current_context_status'] == 'UNKNOWN' for r in out['endpoints'].values()))
        self.assertIsNone(out['encoder_diagnostic'])

    def test_missing_or_changed_original_bytes_fail_source_binding(self):
        expected = pins(self.root)
        path = self.root / self.case['record']['endpoints'][1]['decision_record'] / 'right_wrist_rgb.png'
        data = path.read_bytes(); path.write_bytes(data + b'changed')
        out = run_fixture(self.case, expected=expected)
        self.assertIn('SOURCE_SHA_MISMATCH', str(out['endpoints']['F']))
        path.unlink(); out = run_fixture(self.case, expected=expected)
        self.assertEqual(out['endpoints']['F']['status'], 'UNKNOWN')
        self.assertIsNone(out['encoder_diagnostic'])

    def test_q12_camera_opening_and_command_metadata_mismatch(self):
        for fault in ('q12', 'camera', 'opening', 'command'):
            with self.subTest(fault=fault):
                record = copy.deepcopy(self.case['record']); row = self.case['record']['endpoints'][1]
                if fault == 'q12': row['actual']['q12'][6] += 1
                elif fault == 'camera': row['cameras']['right_wrist']['n'] += 1
                elif fault == 'opening': row['opening']['right_present_tick'] += 1
                else: row['command']['target_ticks'] = dict(row['command']['target_ticks'], shoulder_pan=999)
                self.save_record(); out = run_fixture(self.case)
                self.assertEqual(out['endpoints']['F']['status'], 'UNKNOWN')
                self.case['record'] = record

    def test_stage_result_and_actual_are_separate_required_sources(self):
        path = self.root / 'stage_right_x_F/run/move.json'; original = path.read_bytes()
        log = json.loads(original); log['side'] = 'left'; write_json(path, log)
        out = run_fixture(self.case); self.assertEqual(out['endpoints']['F']['status'], 'UNKNOWN')
        log = json.loads(original); log['trajectory']['target']['shoulder_pan'] += 1; write_json(path, log)
        out = run_fixture(self.case); self.assertIn('STAGE_COMMAND_TARGET_SOURCE_MISMATCH', str(out['endpoints']['F']))
        path.write_bytes(original); path.unlink(); out = run_fixture(self.case)
        self.assertEqual(out['endpoints']['F']['status'], 'UNKNOWN')

    def test_A_zero_command_static_view_reads_preserve_actual_image_and_quantized_response(self):
        self.case = fixture(self.root, pans=(100,100,115))
        row = self.case['record']['endpoints'][1]; move = self.case['record']['moves'][0]
        target = {n: row['actual']['ticks']['right'][n] for n in ARM}
        row['command']['target_ticks'] = target; move['forward'][0]['target_ticks'] = target
        view = dict(nominal_target_reached=True, actual_settled=True, command_state_resolved=True,
            nominal_target=target, settling_evidence='existing POST_READS stationary register window; no command')
        move.update(writes=None, result='HOLDING', observation_view=view)
        row['executor'] = dict(writes=None, result='HOLDING', **{n: view[n] for n in
            ('nominal_target_reached','actual_settled','command_state_resolved')})
        (self.root / 'stage_right_x_F/run/move.json').unlink()
        raw = document(self.case['snapshots']['F'])['actual_feedback']
        write_json(self.root / 'stage_right_x_F/STATIC_VIEW_READS.json', [raw])
        self.save_record(); out = run_fixture(self.case)
        self.assertEqual(out['status'], 'SOURCE_BOUND')
        self.assertEqual(out['endpoints']['F']['stage_motion']['status'], 'STATIC_READ_SOURCE_BOUND')
        self.assertIsNone(out['endpoints']['F']['stage_motion']['writes'])
        self.assertEqual(out['endpoints']['F']['B_output']['feature_error_px']['u'], 4)
        self.assertEqual(out['responses']['T_to_F']['actual_delta_ARM5_ticks'], [0,0,0,0,0])
        self.assertIsNone(out['encoder_diagnostic']); self.assertIsNone(out['Correction'])

    def test_occlusion_and_rotation_prevent_encoder_fit_interpretation(self):
        sample = self.case['snapshots']['F']; e = evidence(sample); e['feature_unoccluded'] = False
        sample['evidence'] = json_blob(e)
        out = run_fixture(self.case); self.assertIsNone(out['encoder_diagnostic'])
        rebind(sample); e['feature_unoccluded'] = True; rebind(sample, e)
        self.case['record']['endpoints'][1]['actual_fk_from_previous']['rotation_deg'] = 0.2
        self.save_record(); out = run_fixture(self.case)
        self.assertIn('CANDIDATE_ROTATION_PRESENT_NOT_PHYSICAL_BOUND', out['responses']['T_to_F']['confounds'])
        self.assertIsNone(out['encoder_diagnostic'])

    def test_independent_V_capture_cannot_be_reused_from_fit(self):
        row, _, d = self.update_observation('V', lambda d: d['camera_frames']['right_wrist'].update(n=100))
        row['cameras'] = copy.deepcopy(d['camera_frames']); self.save_record()
        out = run_fixture(self.case)
        self.assertEqual(out['responses']['F_to_V']['status'], 'FIT_CAPTURE_REUSED')
        self.assertEqual(out['encoder_diagnostic']['independent_status'], 'UNRESOLVED')
        self.assertFalse(out['encoder_diagnostic']['validated'])

    def test_small_response_without_jitter_bound_remains_unresolved(self):
        case = fixture(self.root, shifts=(0,0,0)); out = run_fixture(case)
        self.assertEqual(out['responses']['T_to_F']['observed_delta_u_px'], 0)
        self.assertEqual(out['responses']['T_to_F']['observability'], 'UNRESOLVED')
        self.assertIsNone(out['responses']['T_to_F']['detector_jitter_px'])
        self.assertFalse(out['encoder_diagnostic']['observable_for_control'])

    def test_left_actual_motion_is_an_unresolved_image_response_confound(self):
        def mutate(d):
            d['actual_feedback']['ticks']['left']['shoulder_pan'] += 1
            d['observation']['present_ticks'] = copy.deepcopy(d['actual_feedback']['ticks'])
        row, _, d = self.update_observation('F', mutate)
        row['actual']['ticks'] = copy.deepcopy(d['actual_feedback']['ticks']); self.save_record()
        out = run_fixture(self.case)
        self.assertIn('LEFT_HELD_OBJECT_MOTION_OR_OPENING_CHANGED', out['responses']['T_to_F']['confounds'])
        self.assertIsNone(out['encoder_diagnostic'])

    def test_V_motion_outside_fit_encoder_basis_is_not_a_1D_sign_validation(self):
        def mutate(d):
            d['actual_feedback']['ticks']['right']['wrist_roll'] += 1
            d['observation']['present_ticks'] = copy.deepcopy(d['actual_feedback']['ticks'])
        row, _, d = self.update_observation('V', mutate)
        row['actual']['ticks'] = copy.deepcopy(d['actual_feedback']['ticks']); self.save_record()
        out = run_fixture(self.case)
        self.assertEqual(out['encoder_diagnostic']['independent_status'], 'OFF_BASIS_CONFOUNDED_DIAGNOSTIC_ONLY')
        self.assertFalse(out['encoder_diagnostic']['validated'])

    def test_scale_clipping_and_changed_object_context_are_confounds(self):
        sample = self.case['snapshots']['F']; e = evidence(sample)
        e['context']['object_id'] = 'OTHER_OBJECT'; sample['evidence'] = json_blob(e)
        out = run_fixture(self.case); self.assertIsNone(out['encoder_diagnostic'])
        for clipped in (False, True):
            with self.subTest(clipped=clipped):
                image = np.full((240,320,3), 255, np.uint8)
                image[70:160, :110 if clipped else 220] = [0,0,255]
                if not clipped: image[:,:100] = 255
                sample['image'] = dict(source_blob(cv2.imencode('.png',image)[1].tobytes()), name='right_wrist_rgb.png')
                e['context']['object_id'] = 'FIXTURE_OBJECT'; rebind(sample, e)
                row = self.case['record']['endpoints'][1]
                (self.root / row['decision_record'] / 'right_wrist_rgb.png').write_bytes(sample['image']['bytes'])
                out = run_fixture(self.case); self.assertIsNone(out['encoder_diagnostic'])

    def test_missing_V_and_fault_status_are_not_complete_sources(self):
        self.case['record']['endpoints'].pop(); self.save_record()
        out = run_fixture(self.case); self.assertEqual(out['status'], 'PARTIAL_SOURCE')
        self.assertIsNone(out['encoder_diagnostic'])
        self.case = fixture(self.root); self.case['record']['status'] = 'STOPPED_BY_FAULT'; self.save_record()
        out = run_fixture(self.case); self.assertEqual(out['status'], 'PARTIAL_SOURCE')
        self.assertIsNone(out['encoder_diagnostic'])

    def test_source_paths_cannot_escape_session(self):
        self.case['record']['endpoints'][1]['decision_record'] = '../elsewhere'
        self.save_record(); out = run_fixture(self.case)
        self.assertIn('SOURCE_PATH_OUTSIDE_SESSION', str(out['endpoints']['F']))

    def test_malformed_PNG_header_returns_unknown_without_large_decode(self):
        row = self.case['record']['endpoints'][1]
        path = self.root / row['decision_record'] / 'right_wrist_rgb.png'
        data = bytearray(path.read_bytes()); data[16:24] = struct.pack('>II',65536,65536)
        data[29:33] = struct.pack('>I',zlib.crc32(data[12:29])); path.write_bytes(data)
        out = run_fixture(self.case)
        self.assertEqual(out['endpoints']['F']['status'], 'UNKNOWN')
        self.assertIsNone(out['Correction']); self.assertIsNone(out['encoder_diagnostic'])

    def test_malformed_documents_return_missing_evidence(self):
        for field in ('config', 'endpoints', 'moves'):
            with self.subTest(field=field):
                original = copy.deepcopy(self.case['record']); self.case['record'][field] = None
                self.save_record(); out = run_fixture(self.case)
                self.assertIsNone(out['Correction']); self.assertTrue(out['missing_evidence'])
                self.case['record'] = original
        self.save_record()
        out = bridge_metrology(self.root, run_id=RUN_ID, expected_sha256=pins(self.root), current_evidence_by_role=[])
        self.assertIn('CURRENT_EVIDENCE_BY_ROLE_DOCUMENT_REQUIRED', str(out['missing_evidence']))

    def test_unverified_J_cannot_be_created_from_command_or_candidate_FK(self):
        out = run_fixture(self.case, jacobian_evidence=dict(jacobian=dict(source='REAL_MEASURED', validated=True)))
        self.assertFalse(out['REAL_J_VALIDATED']); self.assertIsNone(out['Correction'])
        self.assertFalse(out['executable']); self.assertFalse(out['motor_write_authorized'])
        self.assertTrue(all(r['B_output']['estimated_tool_correction'] is None for r in out['endpoints'].values()))


if __name__ == '__main__':
    unittest.main()
