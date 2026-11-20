"""Audit native .trt cache keys and engine payloads; never builds or controls."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from onboardmimic.backends.sonic.native_cache import verify_native_caches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT, help='OnboardMimic installation root')
    parser.add_argument('--gpu', default='Orin', help='Exact native CUDA device name; qualified G1 profile is Orin')
    args = parser.parse_args()
    try:
        records = verify_native_caches(args.root/'onboardmimic/backends/sonic/deploy', args.gpu)
    except (OSError, ValueError) as exc:
        parser.exit(1, str(exc)+'\nNo inference or engine build was performed.\n')
    print(json.dumps(dict(passed=True, motor_output=False, engine_build=False,
                         caches=records), indent=2))


if __name__ == '__main__':
    main()
