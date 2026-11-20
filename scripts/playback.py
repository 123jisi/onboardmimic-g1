"""Explicit file-player API client. Never prepares or starts SONIC."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from onboardmimic.core.client import OnboardClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'load', 'play', 'pause', 'resume', 'stop'])
    parser.add_argument('motion', nargs='?', help='Relative CSV directory or SMPL .npz below runtime --motion-root')
    parser.add_argument('--fps', type=float)
    parser.add_argument('--joint-order', choices=['sonic_reference'])
    parser.add_argument('--root-body-index', type=int, default=0)
    parser.add_argument('--entry-checks', choices=['on', 'off'],
                        help='play only: first-frame velocity/tilt checks (default on); feedback checks always apply')
    parser.add_argument('--url', default='http://127.0.0.1:18921')
    parser.add_argument('--token-file', type=Path, default=Path.home()/'.config/onboardmimic/http.token')
    args = parser.parse_args()
    if args.entry_checks is not None and args.action != 'play':
        parser.error('--entry-checks applies only to play')
    if args.action == 'load' and (not args.motion or (not args.motion.endswith('.npz') and (args.fps is None or args.joint_order is None))):
        parser.error('load requires motion; CSV also requires --fps and --joint-order')
    if args.token_file.stat().st_mode & 0o077:
        parser.error('token file must be private (0600)')
    client = OnboardClient(args.url, args.token_file.read_text().strip(), timeout=10)
    if args.action == 'status':
        status = client.status()
    else:
        parameters = dict(motion=args.motion, fps=args.fps, joint_order=args.joint_order,
                          root_body_index=args.root_body_index) if args.action == 'load' else {}
        if args.action == 'play':
            parameters['entry_checks'] = args.entry_checks or 'on'
        status = client.command(args.action, path='/api/playback', **parameters)
    # No request credentials in terminal output.
    print(json.dumps({k: status.get(k) for k in
                     ('state','pids','workflow_error','input_source','reference_representation','mode_switch_pending',
                      'mode_switch_error','backend_feedback','playback_entry_checks','playback',
                      'operator','revision','motion_task','conversion')}, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
