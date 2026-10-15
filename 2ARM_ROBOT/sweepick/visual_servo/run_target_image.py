"""File-only target-image regression; raw observations stay in the private output."""
import argparse
import io
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'goal_bridge'))
from b16_goal_bridge import sha256
from run_b17 import audit_side_effects


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    counts, forbidden = audit_side_effects()
    from visual_servo.target_image import FIELD_TARGET, TargetImageServoProposal, source_blob, snapshot_observation
    from visual_servo.a_shadow import proposal_from_A_record
    from visual_servo.run_b19 import saved_observation
    from visual_servo.test_target_image import fixture_pair
    from visual_servo import test_b19, test_b20, test_b20_recovery, test_target_image
    inputs = json.loads(args.inputs.read_text())
    checked = {str(args.inputs): sha256(args.inputs)}
    checked.update({str(p): sha256(p) for p in (ROOT / 'visual_servo').glob('*.py')})
    for source in [ROOT / 'native/revision', *[Path(item['a_source']) for item in inputs['sessions']]]:
        for name in ('sweepick_pick01.py', 'right_wrist_relative_geometry.py', 'sweepick_261009_offsite_relative_features.py'):
            checked[str(source / name)] = sha256(source / name)
    real = []
    for item in inputs['sessions']:
        session = Path(item['session'])
        for folder in sorted((session / 'decision_observations').iterdir()):
            if not (folder / 'OBSERVATION.json').is_file():
                continue
            d, observation, path = saved_observation(folder, session.parent.name, checked)
            image_path = folder / d['files']['right_wrist']['rgb']
            sample = dict(record=source_blob(path.read_bytes()),
                image=dict(source_blob(image_path.read_bytes()), name=image_path.name),
                observation_identity=observation['observation_identity'])
            independent_source_error = None
            try:
                _, binding, opening = snapshot_observation(sample)
                assert binding['record_sha256'] == checked[str(path)]
                assert opening == d['actual_feedback']['ticks']['right']['gripper']
            except ValueError as exc:
                independent_source_error = str(exc)
            reference = dict(role=FIELD_TARGET, reference_id='UNVERIFIED_ARCHIVE_REFERENCE', snapshot=sample)
            proposal = TargetImageServoProposal(sample, reference, a_source=item['a_source'])
            legacy = proposal_from_A_record(d,
                {k: v['image_bgr'] for k, v in observation['views'].items()},
                record_identity=observation['observation_identity'], record_sha256=checked[str(path)],
                image_sha256={k: v['source_sha256'] for k, v in observation['views'].items()}, a_source=item['a_source'])
            real.append(dict(anonymous_record=f'REAL_{len(real)+1:02}', record_sha256=checked[str(path)],
                proposal=proposal, legacy_control_dimensions=legacy['observed_control_dimensions'],
                legacy_correction=legacy['estimated_tool_correction'], independently_checked_source_error=independent_source_error))
    current, reference = fixture_pair()
    positive = TargetImageServoProposal(current, reference)

    class StoredRealTests(unittest.TestCase):
        def test_actual_REAL_without_physical_target_never_supplies_error_or_correction(self):
            self.assertTrue(real)
            for row in real:
                p = row['proposal']
                self.assertIsNone(p['feature_error_px']['u']); self.assertIsNone(p['estimated_tool_correction'])
                self.assertFalse(p['REAL_J_VALIDATED']); self.assertFalse(p['executable'])
                self.assertFalse(p['motor_write_authorized']); self.assertTrue(p['missing_evidence'])
        def test_existing_REAL_pad_path_remains_unapproved(self):
            for row in real:
                self.assertEqual(row['legacy_control_dimensions'], 0)
                self.assertIsNone(row['legacy_correction'])

    suite = unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromModule(module)
        for module in (test_target_image, test_b19, test_b20, test_b20_recovery)] +
        [unittest.defaultTestLoader.loadTestsFromTestCase(StoredRealTests)])
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    loaded = [name for name in sys.modules if any(name == f or name.startswith(f + '.') for f in forbidden)]
    original_changes = [path for path, digest in checked.items() if sha256(path) != digest]
    report = dict(status='PASS' if result.wasSuccessful() and not loaded and not original_changes else 'FAIL',
        tests=dict(run=result.testsRun, failures=len(result.failures), errors=len(result.errors), skipped=len(result.skipped)),
        synthetic_positive=dict(source_kind='TEST_FIXTURE', physical_measurement=False,
            expected_X_error_px=4.0, actual_X_error_px=positive['feature_error_px']['u'],
            Y_error_px=positive['feature_error_px']['v'], Correction=positive['estimated_tool_correction'],
            graph_requested=positive['graph_requested']),
        REAL_negative_records=len(real), all_REAL_errors_and_corrections_null=all(
            row['proposal']['feature_error_px']['u'] is None and row['proposal']['estimated_tool_correction'] is None for row in real),
        REAL_original_feedback_and_image_packets_validated=sum(row['independently_checked_source_error'] is None for row in real),
        required_cases=['missing target verification', 'independent reference/current bindings', 'object identity',
            'Opening difference', 'clipped/occluded/missing feature', 'actual feedback source', 'camera mount/mode',
            'MODEL/REAL/fixture mixing', 'unvalidated Jacobian', 'fit-independent sign and actual-X source',
            'UNKNOWN contexts', 'reencoded fit capture reuse', 'finite feedback/frame identity',
            'malformed nested documents', 'independent new-capture identical-pixel revisit'],
        existing_PAD_fast_path_LEFT_HELD='unit regressions run; existing implementations unchanged',
        side_effect_checks=dict(counts, forbidden_modules_loaded=loaded), original_source_changes=original_changes,
        REAL_J_VALIDATED=False, FIELD_PHYSICAL_CONFIRMATION_REQUIRED=True, motor_write_authorized=False,
        environment=dict(python=sys.version.split()[0], numpy=np.__version__),
        code_sha256={name: sha256(ROOT / 'visual_servo' / name)
                     for name in ('target_image.py', 'test_target_image.py', 'run_target_image.py')},
        raw_images_calibration_identity_in_public_report=False)
    private = dict(report, REAL_records=real, source_manifest=checked, unittest_log=stream.getvalue())
    (args.output / 'execution.private.json').write_text(json.dumps(private, indent=2, allow_nan=False) + '\n')
    (args.output / 'TARGET_IMAGE_CONSUMER_REGRESSION.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
