"""Idle-only candidate enable/rollback. Never connect, prepare, start or play."""
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
from onboardmimic.inputs.pnlink.state import read_state, fresh
from onboardmimic.backends.sonic.native_cache import verify_native_caches

NAMES=('onboardmimic.service','onboardmimic-console.service')
CONNECT='onboardmimic-console-connect.service'
UNIT_NAMES=(*NAMES,CONNECT)
OVERRIDE='zz-pnlink.conf'


def run(*args):return subprocess.check_output(args,text=True).strip()


def idle(client):
    s=client.status()
    if s.get('mode')!='onboard_real' or s.get('state')!='stopped' or s.get('pids')!=[]:
        raise RuntimeError('Require onboard_real + stopped + pids=[]; no services changed')


def overrides(root, current_units):
    result={}
    for name,text in current_units.items():
        if name==CONNECT:
            # The console reads installation-relative connection status. Keep
            # the explicit connection entrypoint in the same candidate, but
            # never run it as part of deployment (it can release vendor mode).
            command=shlex.join(['/bin/bash',str(root/'onboardmimic/robots/unitree_g1/connect_services.sh')])
            result[name]='[Service]\nWorkingDirectory='+str(root)+'\nExecStart=\nExecStart='+command+'\n'
            continue
        commands=[line.split('=',1)[1].strip() for line in text.splitlines() if line.startswith('ExecStart=') and line.split('=',1)[1].strip()]
        if not commands:raise ValueError('No existing ExecStart for '+name)
        argv=shlex.split(commands[-1])
        expected='onboardmimic.core.runtime' if name==NAMES[0] else 'onboardmimic.console.server'
        if argv[1:3]!=['-m',expected]:raise ValueError('Unexpected service entrypoint: '+name)
        if any(flag in argv for flag in ('--pnlink-status','--pnlink-socket')):raise ValueError('PN Link override already present')
        argv[0]=str(root/'.venv/bin/python')
        argv+=['--pnlink-status',str(root/'var/pnlink/status.json')]
        if name==NAMES[0]:argv+=['--pnlink-socket',str(root/'var/pnlink/command.sock')]
        # Preserve old receiver/hand status, motion roots, robot/backend and bind address.
        result[name]='[Service]\nWorkingDirectory='+str(root)+'\nExecStart=\nExecStart='+shlex.join(argv)+'\n'
    return result


def verify(root):
    manifest=json.loads((root/'phase6-source-sha256.json').read_text())
    for name,expected in manifest.items():
        path=(root/name).resolve(strict=True)
        if root not in path.parents:raise ValueError('Source path outside candidate')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise ValueError('Source hash mismatch: '+name)
    return dict(source_files=len(manifest),native_caches=len(verify_native_caches(root/'onboardmimic/backends/sonic/deploy')))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('check','preview','activate','rollback'))
    p.add_argument('--receive-validation',type=Path,default=ROOT/'var/pnlink/receive-600s.json')
    a=p.parse_args()
    if ROOT.name not in ('teleop-phase6','teleop-phase7') or any(c.isspace() for c in str(ROOT)):raise ValueError('Run inside the installed phase6/7 candidate')
    current={name:run('systemctl','--user','cat',name,'--no-pager') for name in UNIT_NAMES}
    units=Path.home()/'.config/systemd/user'
    if a.action=='preview':print(json.dumps(overrides(ROOT,current),indent=2));return
    token=Path.home()/'.config/onboardmimic/http.token'
    if token.stat().st_mode&0o077:raise ValueError('Token must be private')
    client=OnboardClient('http://127.0.0.1:18921',token.read_text().strip(),timeout=2)
    idle(client)
    if a.action=='check':print(json.dumps(verify(ROOT)));return
    if a.action=='activate':
        verify(ROOT)
        report=json.loads(a.receive_validation.read_text())
        if (report.get('duration_seconds',0)<600 or report.get('observed_frames',0)<2 or report.get('invalid_reads')!=0
                or report.get('receiver_generations')!=1 or report.get('sdk_timestamp_regressions')!=0 or report.get('errors')
                or report.get('receive_interval_ms',{}).get('max',999)>250):
            raise ValueError('Stage6 needs a passing 600s real direct-receive record before enabling stage7')
        data=read_state(ROOT/'var/pnlink/status.json')
        if not fresh(data):raise ValueError('Real PN Link receiver must currently be fresh')
        wanted=overrides(ROOT,current)
        for name in UNIT_NAMES:
            folder=units/(name+'.d')
            if (folder/OVERRIDE).exists() or any(f.name>OVERRIDE for f in folder.glob('*.conf')):raise ValueError('Existing/later override requires inspection')
        backup=Path.home()/'onboardmimic-backups'/('pnlink-'+datetime.now().strftime('%Y%m%dT%H%M%S'))
        backup.mkdir(parents=True)
        for name,text in current.items():(backup/(name+'.effective.txt')).write_text(text)
        idle(client)
        for name,text in wanted.items():
            folder=units/(name+'.d');folder.mkdir(parents=True,exist_ok=True);(folder/OVERRIDE).write_text(text)
        print('Previous effective configuration:',backup)
    else:
        for name in UNIT_NAMES:
            path=units/(name+'.d')/OVERRIDE
            if path.exists():
                if 'WorkingDirectory='+str(ROOT)+'\n' not in path.read_text():raise ValueError('Override belongs to another candidate')
        for name in UNIT_NAMES:(units/(name+'.d')/OVERRIDE).unlink(missing_ok=True)
    run('systemctl','--user','daemon-reload')
    for name in NAMES:run('systemctl','--user','restart',name)
    for _ in range(40):
        try:idle(client);break
        except (OSError,RuntimeError):time.sleep(.25)
    else:raise RuntimeError('Idle API failed to return; restore the recorded configuration')
    print('Idle API/console '+a.action+'; connect/prepare/start/play not requested')


if __name__=='__main__':main()
