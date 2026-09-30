"""Offline long-edge predictions; no motor/serial imports or hardware execution."""
import argparse
import json
import math


def edge_angle(elbow_model_deg):
    if not math.isfinite(elbow_model_deg):
        raise ValueError('Non-finite model angle')
    # Both selected STL edges are parallel to mesh/body X. The lower body has
    # a fixed +90-degree Z rotation, so a folded 90-degree elbow has parallel lines.
    theta = math.radians(90 + elbow_model_deg)
    directed = math.degrees(math.atan2(math.sin(theta), math.cos(theta)))
    return (directed + 90) % 180 - 90


def compatible_candidates(measured, measurement_error, predictions, model_error, acute=False):
    """Interval overlap, conditional on declared bounded errors and matching pose."""
    values = [measured, measurement_error, model_error, *predictions.values()]
    if not predictions or not all(math.isfinite(x) for x in values):
        raise ValueError('Finite measurement, errors, and predictions required')
    if measurement_error <= 0 or model_error < 0:
        raise ValueError('Measured uncertainty must be positive; model error non-negative')
    if acute and any(x < 0 or x > 90 for x in [measured, *predictions.values()]):
        raise ValueError('Acute line angles must be between 0 and 90 degrees')
    # Undirected lines are periodic over 180 degrees. Acute magnitudes instead
    # compare on [0, 90]; wrapping those magnitudes would hide a sign/mode error.
    compatible = [name for name, value in predictions.items()
                  if (abs(measured - value) if acute else abs((measured - value + 90) % 180 - 90))
                  <= measurement_error + model_error]
    status = ('one_candidate_consistent' if len(compatible) == 1 else
              'insufficient_discrimination' if len(compatible) > 1 else
              'neither_candidate_consistent')
    return dict(status=status, compatible=compatible, physically_verified=False)


def demo():
    # Numerical regression only: these assertions are not physical tolerances.
    assert abs(edge_angle(90)) < 1e-12
    assert abs(edge_angle(95.47252747252747) - 5.472527472527474) < 1e-12
    assert abs(edge_angle(89.20879120879121) + .791208791208789) < 1e-12
    synthetic = {'a': 6., 'b': 0.}
    assert compatible_candidates(0., 1., synthetic, 0.)['compatible'] == ['b']
    assert compatible_candidates(3., 3., synthetic, 0.)['status'] == 'insufficient_discrimination'
    assert compatible_candidates(12., 1., synthetic, 0.)['status'] == 'neither_candidate_consistent'
    assert compatible_candidates(89., 3., {'wrapped': -89., 'other': 0.}, 0.)['compatible'] == ['wrapped']
    assert compatible_candidates(1., 1., {'near': .5, 'far': 5.}, 0., acute=True)['compatible'] == ['near']
    for value in (float('nan'), float('inf')):
        try:
            edge_angle(value)
        except ValueError:
            pass
        else:
            raise AssertionError('Non-finite angle accepted')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', help='Saved 14 mapping-comparison.json; no raw conversion rerun')
    parser.add_argument('--self-check', action='store_true')
    args = parser.parse_args()
    if args.self_check:
        demo()
        print('numerical and interval self-check PASS; physical measurement not performed')
    if args.comparison:
        from pathlib import Path
        cache = json.loads(Path(args.comparison).read_text())
        result = {}
        for side, index in [('left', 2), ('right', 7)]:
            result[side] = {}
            for name, case in cache['cases'].items():
                angle = edge_angle(math.degrees(case['arm10_model_rad'][index]))
                result[side][name] = dict(signed_edge_angle_deg=angle,
                                          acute_edge_angle_deg=abs(angle))
        print(json.dumps(result, indent=2, allow_nan=False))
