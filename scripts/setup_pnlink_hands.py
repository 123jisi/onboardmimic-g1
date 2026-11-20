#!/usr/bin/env python3
"""Install the shared PICO/PN continuous hand worker with the body stopped.

Preserves the effective API arguments and PN receiver configuration. Never
starts capture, calibrates, prepares posture or enters a teleoperation session.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from onboardmimic.core.client import OnboardClient
from onboardmimic.core.service_units import quote,unit_path
from scripts.setup_pico_hands import verify_build

NAME='onboardmimic-pico-hands.service'  # The existing, single physical hand owner.
DROP='zzz-pnlink-hands.conf'


def idle(client):
    s=client.status()
    if s.get('mode')!='onboard_real' or s.get('state')!='stopped' or s.get('pids')!=[] or s.get('mode_switch_pending'):
        raise RuntimeError('Require onboard_real + stopped + pids=[]')
    h=s.get('pico_hands',{})
    if h.get('active') or any(v.get('commanding') for v in h.get('sides',{}).values()):
        raise RuntimeError('Hand worker must be inactive before installation')
    if s.get('pnlink_command_pending'):
        raise RuntimeError('Wait for PN device command completion')
    return s


def render(root,effective):
    commands=[line.split('=',1)[1].strip() for line in effective.splitlines()
              if line.startswith('ExecStart=') and line.split('=',1)[1].strip()]
    if not commands:raise ValueError('Missing effective API command')
    argv=shlex.split(commands[-1])
    if argv[1:3]!=['-m','onboardmimic.core.runtime'] or '--pnlink-status' not in argv:
        raise ValueError('Require current PN Link runtime')
    if argv[argv.index('--pnlink-status')+1]!=str(root/'var/pnlink/status.json'):
        raise ValueError('API PN receiver belongs to another deployment')
    while '--pico-hands-status' in argv:
        i=argv.index('--pico-hands-status');del argv[i:i+2]
    argv[0]=str(root/'.venv/bin/python')
    argv+=['--pico-hands-status',str(root/'var/pico-hands.json')]
    # Existing systemd specifiers such as %h must keep their meaning.
    command=' '.join('"'+a.replace('\\','\\\\').replace('"','\\"')+'"' for a in argv)
    drop='[Service]\nWorkingDirectory='+unit_path(root)+'\nExecStart=\nExecStart='+command+'\n'
    unit='''[Unit]
Description=OnboardMimic shared PICO and PN BrainCo hand adapter
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
    return unit,drop


def verify_sources(root,manifest):
    source=json.loads(Path(manifest).read_text())
    if not source:raise ValueError('Empty source manifest')
    for name,wanted in source.items():
        path=(root/name).resolve(strict=True)
        if root.resolve() not in path.parents or hashlib.sha256(path.read_bytes()).hexdigest()!=wanted:
            raise ValueError('Source mismatch: '+name)
    return len(source)


def systemctl(*args):
    return subprocess.check_output(['systemctl','--user',*args],text=True).strip()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('check','install','rollback'))
    p.add_argument('--source-manifest',type=Path,default=ROOT/'source-sha256.json')
    p.add_argument('--backup',type=Path)
    a=p.parse_args()
    token=Path.home()/'.config/onboardmimic/http.token'
    if token.stat().st_mode&0o077:raise ValueError('Token must be private')
    client=OnboardClient('http://127.0.0.1:18921',token.read_text().strip(),timeout=3)
    idle(client)
    units=Path.home()/'.config/systemd/user';target=units/NAME;override=units/'onboardmimic.service.d'/DROP
    if a.action=='rollback':
        if not a.backup:raise ValueError('Rollback requires --backup')
        saved=json.loads((a.backup/'units.json').read_text())
        if set(saved)!=set((str(target),str(override))):raise ValueError('Backup unit paths differ')
        idle(client);systemctl('stop',NAME)
        for name,text in saved.items():
            path=Path(name)
            if text is None:path.unlink(missing_ok=True)
            else:path.write_text(text)
        systemctl('daemon-reload');idle(client);systemctl('restart','onboardmimic.service')
        systemctl('start',NAME)
    else:
        count=verify_sources(ROOT,a.source_manifest);build=verify_build(ROOT)
        unit,drop=render(ROOT,systemctl('cat','onboardmimic.service','--no-pager'))
        if override.exists() and override.read_text()!=drop:raise ValueError('Existing custom PN hand override; inspect first')
        if a.action=='check':
            print(json.dumps(dict(source_files=count,binary_sha256=build['binary_sha256'],motor_output=False)));return
        backup=Path.home()/'onboardmimic-backups'/('pnlink-hands-'+datetime.now().strftime('%Y%m%dT%H%M%S'))
        backup.mkdir(parents=True,mode=0o700)
        (backup/'units.json').write_text(json.dumps({str(f):f.read_text() if f.exists() else None for f in (target,override)},indent=2))
        idle(client);systemctl('stop',NAME)
        target.write_text(unit);override.parent.mkdir(parents=True,exist_ok=True);override.write_text(drop)
        systemctl('daemon-reload');idle(client);systemctl('restart','onboardmimic.service')
        systemctl('start',NAME)
        print('Backup:',backup)
    for _ in range(50):
        try:
            s=idle(client);h=s.get('pico_hands',{})
            if h.get('fresh') and (a.action=='rollback' or 2 in h.get('intent_protocols',[]) and h.get('finish_open_supported') is True and set(h.get('continuous_sides',[]))=={'left','right'}):break
        except (OSError,ValueError,RuntimeError):pass
        time.sleep(.1)
    else:raise RuntimeError('Idle API/hand worker did not return; inspect service logs')
    print(json.dumps(dict(state=s['state'],pids=s['pids'],hand_worker=h.get('bridge_id'),
        protocols=h.get('intent_protocols'),hand_command_counts={k:v.get('commands_sent') for k,v in h.get('sides',{}).items()},
        motor_output=False,body_controller_started=False),indent=2))

if __name__=='__main__':main()
