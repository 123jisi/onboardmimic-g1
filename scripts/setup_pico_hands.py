#!/usr/bin/env python3
"""Install optional PICO->BrainCo hand intent adapter while SONIC is stopped.

Never connects the body bridge, prepares posture, starts SONIC, or sends a hand
command. The worker waits for a subsequent explicit PICO session and release.
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
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from onboardmimic.core.client import OnboardClient
from onboardmimic.core.service_units import quote,unit_path
from scripts.phase5_acceptance import overrides,idle,verify

NAME='onboardmimic-pico-hands.service'
DROP='zz-pico-hands.conf'


def render(root):
    unit='''[Unit]
Description=OnboardMimic PICO BrainCo hand adapter
[Service]
Type=simple
WorkingDirectory={cwd}
ExecStart=/bin/bash {script}
Restart=on-failure
RestartSec=1
KillMode=control-group
UMask=0077
TimeoutStopSec=5

[Install]
WantedBy=default.target
'''.format(cwd=unit_path(root),script=quote(root/'onboardmimic/robots/unitree_g1/hands/run.sh'))
    api=overrides(root)['onboardmimic.service'].rstrip()+' --pico-hands-status '+quote(root/'var/pico-hands.json')+'\n'
    return unit,api


def verify_build(root):
    folder=root/'onboardmimic/robots/unitree_g1/hands'
    m=json.loads((root/'var/pico-hands-build.json').read_text())
    if set(m['sources'])!=set(('core.hpp','core_test.cpp','bridge.cpp','CMakeLists.txt','build.sh')):
        raise RuntimeError('Incomplete hand build provenance')
    for name,wanted in m['sources'].items():
        if name not in ('core.hpp','core_test.cpp','bridge.cpp','CMakeLists.txt','build.sh'):
            raise RuntimeError('Invalid hand build source')
        if hashlib.sha256((folder/name).read_bytes()).hexdigest()!=wanted:
            raise RuntimeError('Hand source changed; rebuild: '+name)
    binary=folder/'build/pico_hand_bridge';data=binary.read_bytes()
    if data[:4]!=b'\x7fELF' or int.from_bytes(data[18:20],'little')!=183:
        raise RuntimeError('Hand adapter must be a local ARM64 build')
    if hashlib.sha256(data).hexdigest()!=m['binary_sha256']:
        raise RuntimeError('Hand adapter binary mismatch')
    deps=root/'var/deps/pico-hands-json-sha256.json'
    if deps.exists():
        for name,wanted in json.loads(deps.read_text())['sha256'].items():
            path=(root/'var/deps/include'/name).resolve()
            if (root/'var/deps/include').resolve() not in path.parents:
                raise RuntimeError('Invalid header path')
            if hashlib.sha256(path.read_bytes()).hexdigest()!=wanted:raise RuntimeError('Header mismatch: '+name)
    return m


def run(*args,**kw):return subprocess.run(['systemctl','--user',*args],check=True,**kw)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('check','install','rollback'),default='check',nargs='?');a=p.parse_args()
    if ROOT.name!='teleop-phase5':raise RuntimeError('Run from the current teleop-phase5 candidate')
    token=Path.home()/'.config/onboardmimic/http.token'
    if token.stat().st_mode&0o077:raise RuntimeError('Token must be private')
    client=OnboardClient('http://127.0.0.1:18921',token.read_text().strip(),timeout=3)
    idle(client);source=verify(ROOT);build=verify_build(ROOT)
    unit,drop=render(ROOT);units=Path.home()/'.config/systemd/user'
    target=units/NAME;override=units/'onboardmimic.service.d'/DROP
    for path,text in ((target,unit),(override,drop)):
        if path.exists() and path.read_text()!=text:raise RuntimeError('Modified existing hand unit: '+str(path))
    if a.action=='check':
        print(json.dumps(dict(source=source,binary_sha256=build['binary_sha256'],motor_output=False)));return
    if a.action=='rollback':
        if not target.exists() or not override.exists():raise RuntimeError('Hand installation incomplete; inspect manually')
        idle(client);run('disable','--now',NAME);override.unlink();target.unlink();run('daemon-reload');idle(client)
        run('restart','onboardmimic.service')
    else:
        backup=Path.home()/'onboardmimic-backups'/('pico-hands-'+datetime.now().strftime('%Y%m%dT%H%M%S'));backup.mkdir(parents=True)
        for path in (target,override):
            if path.exists():shutil.copy2(path,backup/path.name)
        idle(client);target.write_text(unit);override.parent.mkdir(parents=True,exist_ok=True);override.write_text(drop)
        run('daemon-reload')
        effective=run('show','onboardmimic.service','--property=ExecStart','--value',capture_output=True,text=True).stdout
        if '--pico-hands-status ' not in effective or str(ROOT/'var/pico-hands.json') not in effective:
            raise RuntimeError('Effective API command does not enable hand status; inspect override')
        idle(client);run('enable','--now',NAME)
        # Stopped gateway sends no active hand intent; starting worker is receive-only.
        ready=False
        for _ in range(50):
            try:
                s=json.loads((ROOT/'var/pico-hands.json').read_text())
                ready=s.get('running') is True and s.get('observe_only') is False and 0<=(time.monotonic_ns()-s['updated_monotonic_ns'])/1e9<.5
                if ready:break
            except (OSError,ValueError,KeyError):pass
            time.sleep(.1)
        if not ready:raise RuntimeError('Hand worker did not become ready; controller remains stopped')
        idle(client);run('restart','onboardmimic.service');print('Backup:',backup)
    for _ in range(40):
        try:s=idle(client);break
        except Exception:time.sleep(.25)
    else:raise RuntimeError('Idle API did not return; inspect services')
    print(json.dumps(dict(state=s['state'],pids=s['pids'],pico_hands=s.get('pico_hands'),hand_action_sent=False,body_controller_started=False),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
