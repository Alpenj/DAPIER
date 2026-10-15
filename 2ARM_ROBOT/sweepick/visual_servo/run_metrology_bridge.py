"""Run the file-only bridge checks and attach the preserved B20 regression receipt."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'goal_bridge'))
from run_b17 import audit_side_effects


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def without_bytes(value):
    if isinstance(value, bytes):
        return dict(sha256=hashlib.sha256(value).hexdigest(), bytes=len(value))
    if isinstance(value, dict):
        return {k: without_bytes(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [without_bytes(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    counts, forbidden = audit_side_effects()
    from visual_servo import test_metrology_bridge
    from visual_servo.test_metrology_bridge import fixture, run_fixture
    baseline = json.loads(args.baseline_report.read_text())
    assert baseline['status'] == 'PASS' and baseline['tests'] == dict(run=57, failures=0, errors=0, skipped=0)
    assert baseline['original_source_changes'] == [] and baseline['REAL_original_feedback_and_image_packets_validated'] == 14
    assert baseline['all_REAL_errors_and_corrections_null']
    assert all(sha(ROOT / 'visual_servo' / name) == digest for name, digest in baseline['code_sha256'].items())
    sources = {p.name: sha(p) for p in (ROOT / 'visual_servo').glob('*.py')}
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromModule(test_metrology_bridge))
    with tempfile.TemporaryDirectory() as folder:
        case = fixture(Path(folder)); positive = run_fixture(case)
        unconfirmed = run_fixture(case, reference=False, current=False)
    loaded = [name for name in sys.modules if any(name == f or name.startswith(f + '.') for f in forbidden)]
    changed = [name for name, digest in sources.items() if sha(ROOT / 'visual_servo' / name) != digest]
    ok = result.wasSuccessful() and not loaded and not changed and not counts['forbidden_attempts']
    tests = dict(run=result.testsRun, failures=len(result.failures), errors=len(result.errors), skipped=len(result.skipped))
    report = dict(schema='sweepick.b20.a-metrology-bridge-regression/1', status='PASS' if ok else 'FAIL',
        A_schema_contract_commit=positive['A_schema_contract_commit'], B_consumer_commit=positive['B_consumer_commit'],
        bridge_tests=tests, preserved_B20_tests=baseline['tests'],
        total_tests=dict(run=57+result.testsRun, failures=len(result.failures), errors=len(result.errors), skipped=len(result.skipped)),
        preserved_B20_receipt_sha256=sha(args.baseline_report),
        REAL_original_feedback_and_image_packets_validated=14,
        all_REAL_target_errors_and_corrections_null=baseline['all_REAL_errors_and_corrections_null'],
        synthetic_TFV=dict(source_kind='TEST_FIXTURE', physical_measurement=False,
            target_image_X_error_px={r: positive['endpoints'][r]['B_output']['feature_error_px']['u'] for r in ('T','F','V')},
            T_to_F=dict(actual_delta_ARM5_ticks=positive['responses']['T_to_F']['actual_delta_ARM5_ticks'],
                observed_delta_u_px=positive['responses']['T_to_F']['observed_delta_u_px']),
            F_to_V=dict(purpose=positive['responses']['F_to_V']['purpose'], observed_delta_u_px=positive['responses']['F_to_V']['observed_delta_u_px']),
            encoder_diagnostic=positive['encoder_diagnostic']),
        actual_A_TFV='REAL_TFV_UNAVAILABLE_NOT_EXECUTED',
        observation_limit='No stationary detector-noise bound; moving T/F/V alone cannot establish control observability.',
        required_cases=['original SHA and endpoint actual/camera/opening/command source mismatch',
            'Stage command-target mismatch and separate later endpoint actual', 'missing source and path escape',
            'A zero-command STATIC_VIEW_READS source without move.json', 'malformed PNG header decoder error',
            'occlusion, clipping, scale/area and rotation confounds', 'left actual motion confound',
            'fit-independent V capture reuse and encoder off-basis confound', 'missing target and copied reference evidence',
            'small response without detector-jitter bound', 'partial/fault/malformed source',
            'candidate FK and command are never physical X evidence'],
        physical_X_mm=None, MEASURED_PHYSICAL_X_RESPONSE_created=False,
        target_reference_created=unconfirmed['target_reference_created'], REAL_J_VALIDATED=False,
        Correction=None, executable=False, motor_write_authorized=False,
        side_effect_checks=dict(counts, forbidden_modules_loaded=loaded), original_source_changes=changed,
        code_sha256={n: sources[n] for n in ('metrology_bridge.py', 'test_metrology_bridge.py', 'run_metrology_bridge.py')},
        raw_REAL_images_ticks_calibration_device_identity_in_public_report=False,
        environment=dict(python=sys.version.split()[0]))
    private = dict(report, unittest_log=stream.getvalue(), synthetic_positive=without_bytes(positive),
                   no_confirmation=without_bytes(unconfirmed), source_sha256=sources)
    (args.output / 'execution.private.json').write_text(json.dumps(private, indent=2, allow_nan=False)+'\n')
    (args.output / 'A_METROLOGY_TARGET_IMAGE_REGRESSION.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
