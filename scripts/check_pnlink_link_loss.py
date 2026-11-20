"""Stopped-robot receive-only UDP loss test. Never changes robot control.

Temporarily filters only the named PN source/UDP destination port, then removes
that exact uniquely tagged rule in finally. Requires root and a stopped API.
"""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import secrets
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from onboardmimic.core.client import OnboardClient
from onboardmimic.inputs.pnlink.state import read_state, fresh


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-ip',required=True)
    p.add_argument('--port',type=int,default=8002)
    p.add_argument('--status',type=Path,default=ROOT/'var/pnlink/status.json')
    p.add_argument('--token-file',type=Path,required=True)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    ipaddress.IPv4Address(a.source_ip)
    if os.geteuid()!=0 or not 1<=a.port<=65535:p.error('Requires root and a valid UDP port')
    if a.token_file.stat().st_mode&0o077:p.error('Token file must be private')
    client=OnboardClient('http://127.0.0.1:18921',a.token_file.read_text().strip())
    state=client.status()
    if state.get('mode')!='onboard_real' or state.get('state')!='stopped' or state.get('pids')!=[]:
        raise RuntimeError('Require stopped robot and pids=[]; network unchanged')
    before=read_state(a.status)
    if not fresh(before):raise RuntimeError('Require fresh capture before testing link loss')
    tag='onboard-pnlink-test-'+secrets.token_hex(8)
    rule=['INPUT','-p','udp','-s',a.source_ip,'--dport',str(a.port),'-m','comment','--comment',tag,'-j','DROP']
    lost=None;start=time.monotonic()
    subprocess.run(['iptables','-w','3','-I',*rule],check=True)
    try:
        while time.monotonic()-start<2:
            state=client.status()
            if state.get('state')!='stopped' or state.get('pids')!=[]:
                raise RuntimeError('Robot state changed during receive-only test')
            if not fresh(read_state(a.status)) and lost is None:lost=time.monotonic()-start
            time.sleep(.05)
    finally:
        subprocess.run(['iptables','-w','3','-D',*rule],check=True)
    recovery=time.monotonic();after=read_state(a.status)
    while time.monotonic()-recovery<5:
        after=read_state(a.status)
        if fresh(after):break
        time.sleep(.05)
    result=dict(receive_loss_detected=lost is not None,loss_detect_seconds=lost,
                receive_recovered=fresh(after),receiver_changed=before.get('receiver_id')!=after.get('receiver_id'),
                rule_removed=True,robot_state=client.status().get('state'),robot_output_enabled=False,
                scope='Receive freshness only; not active-policy fallback or physical sensor-loss validation')
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if lost is None:raise SystemExit(1)


if __name__=='__main__':main()
