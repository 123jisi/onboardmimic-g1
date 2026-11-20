#!/usr/bin/env python3
"""Idle-only phase5 deployment. Never prepares/starts SONIC or sends motion.

check/preview are read-only. receive-setup installs only a receive-only unit;
receive-start enables that unit. activate changes only idle API/console units.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from onboardmimic.core.client import OnboardClient
from onboardmimic.backends.sonic.native_cache import verify_native_caches
from onboardmimic.inputs.pico.install_service import render,NAME,IMAGE
from scripts.phase4_acceptance import overrides as phase4_overrides,NAMES

OVERRIDE='99-phase5-acceptance.conf'

def overrides(root):
    result=phase4_overrides(root)
    status=root/'onboardmimic/inputs/pico/runs/live/status.json'
    for name in NAMES[:2]:result[name]=result[name].rstrip()+' --pose-status '+str(status)+'\n'
    return result

def run(*argv,**kw):return subprocess.run(list(argv),check=True,**kw)

def idle(client):
    s=client.status()
    if s.get('mode')!='onboard_real' or s.get('state')!='stopped' or s.get('pids')!=[]:
        raise RuntimeError('Require onboard_real + stopped + pids=[]; services unchanged')
    return s

def verify(root):
    manifest=json.loads((root/'phase5-source-sha256.json').read_text())
    for name,expected in manifest.items():
        path=(root/name).resolve(strict=True)
        if root not in path.parents:raise RuntimeError('Source manifest path outside candidate')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise RuntimeError('Source mismatch: '+name)
    caches=verify_native_caches(root/'onboardmimic/backends/sonic/deploy')
    return dict(verified_files=len(manifest),native_caches=len(caches),motor_output=False)

def image_ready():
    label=run('sudo','docker','image','inspect',IMAGE,'--format','{{ index .Config.Labels "onboardmimic.pico.sdk_patch" }}',capture_output=True,text=True).stdout.strip()
    if label!='phase5-controller-clock-v1':raise RuntimeError('PICO image lacks controller timestamp patch')
    arch=run('sudo','docker','image','inspect',IMAGE,'--format','{{.Architecture}}',capture_output=True,text=True).stdout.strip()
    if arch!='arm64':raise RuntimeError('PICO image must be arm64')

def wait_idle(client):
    for _ in range(40):
        try:return idle(client)
        except Exception:time.sleep(.25)
    raise RuntimeError('Idle API did not become ready; use recorded unit backup to restore')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',nargs='?',choices=('check','preview','receive-setup','receive-start','activate','rollback'),default='check')
    a=p.parse_args()
    if ROOT.name!='teleop-phase5' or any(c.isspace() for c in str(ROOT)):
        raise RuntimeError('Run from the installed teleop-phase5 candidate')
    wanted=overrides(ROOT)
    if a.action=='preview':
        print(json.dumps(wanted,indent=2));return
    token=Path.home()/'.config/onboardmimic/http.token'
    if token.stat().st_mode&0o077:raise RuntimeError('Token must be private')
    client=OnboardClient('http://127.0.0.1:18921',token.read_text().strip(),timeout=2)
    idle(client)
    if a.action=='check':
        print(json.dumps(verify(ROOT)))
        run('systemctl','--user','show',*NAMES,'-p','WorkingDirectory,ExecStart,DropInPaths','--no-pager')
        return
    if a.action in ('receive-setup','receive-start'):
        verify(ROOT);image_ready();idle(client)
        unit,_=render(ROOT,Path.home()/'.config/onboardmimic')
        if a.action=='receive-start':
            installed=run('systemctl','cat',NAME,'--no-pager',capture_output=True,text=True).stdout
            if unit not in installed:raise RuntimeError('Receive unit is not this candidate; run receive-setup')
            run('sudo','systemctl','enable','--now',NAME)
            run('systemctl','is-active',NAME)
            print('Receive-only service started; SONIC untouched');return
        state=run('systemctl','show',NAME,'-p','ActiveState','--value',capture_output=True,text=True).stdout.strip()
        if state not in ('inactive','failed'):raise RuntimeError('Stop old receive-only service before replacing its configuration')
        pico=ROOT/'onboardmimic/inputs/pico'
        devices=pico/'devices.json'
        if not devices.exists():shutil.copy2(pico/'devices.example.json',devices)
        json.loads(devices.read_text())
        run('sudo','install','-d','-o','root','-g','root','-m','0755',str(pico/'runs/live'))
        run('sudo','chgrp','root',str(devices));devices.chmod(0o640)
        target='/etc/systemd/system/'+NAME
        if Path(target).exists():
            backup=Path.home()/'onboardmimic-backups'/('phase5-receiver-'+datetime.now().strftime('%Y%m%dT%H%M%S'))
            backup.mkdir(parents=True);run('sudo','cp','-p',target,str(backup/NAME))
            print('Receive unit backup:',backup)
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/NAME;f.write_text(unit);run('sudo','install','-m','0644',str(f),target)
        run('sudo','systemctl','daemon-reload')
        print('Receive-only service configured; nothing started');return
    units=Path.home()/'.config/systemd/user'
    if a.action=='activate':
        verify(ROOT)
        # Preserve phase4 and all bridge/readers; reject later unknown overrides.
        for name in NAMES:
            folder=units/(name+'.d')
            if (folder/OVERRIDE).exists():raise RuntimeError('Phase5 overrides already exist')
            if any(f.name>OVERRIDE for f in folder.glob('*.conf')):raise RuntimeError('Later custom override requires manual merge')
        backup=Path.home()/'onboardmimic-backups'/('phase5-'+datetime.now().strftime('%Y%m%dT%H%M%S'))
        backup.mkdir(parents=True,exist_ok=False)
        for name in NAMES:
            for original in (units/name,units/(name+'.d')):
                if original.is_dir():shutil.copytree(original,backup/original.name)
                elif original.exists():shutil.copy2(original,backup/original.name)
        print('Unit backup:',backup,flush=True)
    else:
        for name,data in wanted.items():
            f=units/(name+'.d')/OVERRIDE
            if not f.exists() or f.read_text()!=data:raise RuntimeError('Modified/missing override; refusing automatic rollback')
    idle(client)
    run('systemctl','--user','stop',NAMES[0]);run('systemctl','--user','stop',*NAMES[1:])
    for name,data in wanted.items():
        f=units/(name+'.d')/OVERRIDE
        if a.action=='activate':
            f.parent.mkdir(parents=True,exist_ok=True)
            with f.open('x') as out:out.write(data)
        else:f.unlink()
    run('systemctl','--user','daemon-reload')
    # Validate effective commands before starting the idle API.
    if a.action=='activate':
        for name in NAMES[:2]:
            effective=run('systemctl','--user','show',name,'-p','ExecStart','--value',capture_output=True,text=True).stdout
            if str(ROOT) not in effective or '--pose-status '+str(ROOT/'onboardmimic/inputs/pico/runs/live/status.json') not in effective:
                raise RuntimeError('Effective command is not phase5; restore the printed unit backup before restarting')
    run('systemctl','--user','start',*NAMES[:2]);wait_idle(client)
    print('Idle API/console:',a.action,'; connect/prepare/start/play NOT requested')

if __name__=='__main__':
    try:main()
    except Exception as exc:raise SystemExit('Not completed: '+str(exc))
