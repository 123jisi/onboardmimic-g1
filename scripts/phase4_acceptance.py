#!/usr/bin/env python3
"""Field helper: inspect, activate idle phase4 services, or remove its overrides.

Never connects hardware, prepares posture, starts SONIC, or sends reference data.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from onboardmimic.core.client import OnboardClient
from onboardmimic.backends.sonic.native_cache import verify_native_caches

NAMES = ('onboardmimic.service', 'onboardmimic-console.service',
         'onboardmimic-console-connect.service')
OVERRIDE = '99-phase4-acceptance.conf'


def idle(client):
    status = client.status()
    if status.get('mode') != 'onboard_real' or status.get('state') != 'stopped' or status.get('pids') != []:
        raise RuntimeError('Require mode=onboard_real, state=stopped, pids=[]; no services changed')
    return status


def verify(root):
    expected = json.loads((root/'phase4-source-sha256.json').read_text())
    changed = []
    for name, checksum in expected.items():
        path = (root/name).resolve(strict=True)
        if root not in path.parents:
            raise RuntimeError('Manifest path outside candidate')
        if hashlib.sha256(path.read_bytes()).hexdigest() != checksum:
            changed.append(name)
    if changed:
        raise RuntimeError('Source/native mismatch: '+', '.join(changed))
    records = verify_native_caches(root/'onboardmimic/backends/sonic/deploy')
    print(json.dumps(dict(verified_files=len(expected), native_caches=len(records), motor_output=False)))


def overrides(root):
    # Preserve bridge/readers and their validated binaries. The explicit
    # connection job must use the new source so it writes the new status path.
    common = ' --token-file %h/.config/onboardmimic/http.token --robot unitree_g1 --backend sonic'
    return {
        NAMES[0]: '[Service]\nWorkingDirectory='+str(root)+'\nExecStart=\nExecStart='+str(root/'.venv/bin/python')+
            ' -m onboardmimic.core.runtime --physical'+common+' --motion-root '+str(root/'var/motions')+
            ' --motion-source-root '+str(root/'var')+'\n',
        NAMES[1]: '[Service]\nWorkingDirectory='+str(root)+'\nExecStart=\nExecStart='+str(root/'.venv/bin/python')+
            ' -m onboardmimic.console.server --host 0.0.0.0 --port 8769'+common+'\n',
        NAMES[2]: '[Service]\nWorkingDirectory='+str(root)+'\nExecStart=\nExecStart=/bin/bash '+
            str(root/'onboardmimic/robots/unitree_g1/connect_services.sh')+'\n',
    }


def systemctl(*args):
    return subprocess.run(['systemctl', '--user', *args], check=True)


def wait_idle(client):
    error = ''
    for _ in range(40):
        try:
            idle(client)
            print('Idle API ready: state=stopped, pids=[]; no controller started')
            return
        except Exception as exc:
            error = str(exc)
            time.sleep(.25)
    raise RuntimeError('API not ready: '+error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', nargs='?', choices=('check','activate','rollback'), default='check')
    args = parser.parse_args()
    if not ROOT.name == 'teleop-phase4' or any(c.isspace() for c in str(ROOT)):
        raise RuntimeError('Run from the installed teleop-phase4 candidate')
    token = Path.home()/'.config/onboardmimic/http.token'
    if token.stat().st_mode & 0o077:
        raise RuntimeError('Token must be private (0600)')
    client = OnboardClient('http://127.0.0.1:18921', token.read_text().strip(), timeout=2)
    idle(client)
    units = Path.home()/'.config/systemd/user'
    if args.action == 'check':
        verify(ROOT)
        systemctl('show', *NAMES, '--property=WorkingDirectory,ExecStart', '--no-pager')
        return
    wanted = overrides(ROOT)
    if args.action == 'activate':
        verify(ROOT)
        for name in NAMES:
            if (units/(name+'.d')/OVERRIDE).exists():
                raise RuntimeError('Overrides already exist; check status instead of reactivating')
        backup = Path.home()/'onboardmimic-backups'/('phase4-'+datetime.now().strftime('%Y%m%dT%H%M%S'))
        backup.mkdir(parents=True, exist_ok=False)
        for name in NAMES:
            for original in (units/name, units/(name+'.d')):
                if original.is_dir():
                    shutil.copytree(original, backup/original.name)
                elif original.exists():
                    shutil.copy2(original, backup/original.name)
        print('Unit backup:', backup, flush=True)
    else:
        # Remove only our exact overrides; never erase an operator's edits.
        for name, data in wanted.items():
            path = units/(name+'.d')/OVERRIDE
            if not path.exists() or path.read_text() != data:
                raise RuntimeError('Missing/modified acceptance override; refusing automatic rollback: '+str(path))
    # Recheck immediately before stopping idle services. Stop the old API first
    # so a previously opened browser cannot queue preparation during migration.
    idle(client)
    systemctl('stop', NAMES[0])
    systemctl('stop', NAMES[1], NAMES[2])
    for name, data in wanted.items():
        path = units/(name+'.d')/OVERRIDE
        if args.action == 'activate':
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('x') as stream:
                stream.write(data)
        else:
            path.unlink()
    systemctl('daemon-reload')
    systemctl('start', NAMES[0], NAMES[1])
    wait_idle(client)
    print('Result:', args.action, '; bridge/readers unchanged; connect/prepare/start/play NOT requested')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        raise SystemExit('Not completed: '+str(exc))
