"""Numerically compare two SONIC six-CSV directories, without control I/O."""
import argparse
import json
from pathlib import Path

import numpy as np


def compare(expected, actual, tolerance=2e-6):
    results = {}
    for name in ('joint_pos', 'joint_vel', 'body_pos', 'body_quat', 'body_lin_vel', 'body_ang_vel'):
        left = np.loadtxt(Path(expected)/(name+'.csv'), delimiter=',', skiprows=1, ndmin=2)
        right = np.loadtxt(Path(actual)/(name+'.csv'), delimiter=',', skiprows=1, ndmin=2)
        same_shape = left.shape == right.shape
        error = float(np.max(np.abs(left-right))) if same_shape and np.isfinite(left).all() and np.isfinite(right).all() else None
        results[name+'.csv'] = dict(expected_shape=list(left.shape), actual_shape=list(right.shape),
                                    max_abs_error=error, passed=error is not None and error <= tolerance)
    return dict(passed=all(x['passed'] for x in results.values()), tolerance=tolerance, files=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('expected', type=Path)
    parser.add_argument('actual', type=Path)
    parser.add_argument('--atol', type=float, default=2e-6)
    args = parser.parse_args()
    if not np.isfinite(args.atol) or args.atol < 0:
        parser.error('--atol must be finite and nonnegative')
    result = compare(args.expected, args.actual, args.atol)
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
