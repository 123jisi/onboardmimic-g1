"""Render/install a receive-only user service; never start robot control."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys

NAME='onboardmimic-pnlink-receiver.service'


def render(root, config):
    root=Path(root).resolve()
    if any(c.isspace() for c in str(root)):raise ValueError('Installation path cannot contain whitespace')
    import ipaddress
    for key in ('local_ip','server_ip'):ipaddress.IPv4Address(config[key])
    for key in ('local_port','server_port'):
        if type(config.get(key,8002 if key=='local_port' else 8080)) is not int or not 1<=config.get(key,8002 if key=='local_port' else 8080)<=65535:raise ValueError('Invalid UDP port')
    if type(config.get('auto_capture',False)) is not bool:raise ValueError('auto_capture must be boolean')
    state=root/'var/pnlink'
    argv=['/usr/bin/python3','-m','onboardmimic.inputs.pnlink.receiver',
          '--sdk-root',str(root/'onboardmimic/inputs/pnlink/vendor'),
          '--local-ip',config['local_ip'],'--server-ip',config['server_ip'],
          '--local-port',str(config.get('local_port',8002)),'--server-port',str(config.get('server_port',8080)),
          '--status',str(state/'status.json'),'--socket',str(state/'command.sock')]
    if config.get('auto_capture',False):argv.append('--auto-capture')
    return '\n'.join(['[Unit]','Description=OnboardMimic PN Link receive-only SDK','After=network-online.target',
        'StartLimitIntervalSec=0',
        '[Service]','Type=simple','WorkingDirectory='+str(root),'ExecStart='+shlex.join(argv),
        'Restart=on-failure','RestartSec=5','TimeoutStopSec=10','UMask=0077','NoNewPrivileges=true',
        'PrivateTmp=true','ProtectSystem=strict','ProtectHome=read-only','ReadWritePaths='+str(state),
        'RestrictAddressFamilies=AF_INET AF_UNIX AF_NETLINK','Environment=PYTHONDONTWRITEBYTECODE=1',
        '[Install]','WantedBy=default.target',''])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[3])
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--install',action='store_true')
    a=p.parse_args();unit=render(a.root,json.loads(a.config.read_text()))
    if not a.install:print(unit);return
    destination=Path.home()/'.config/systemd/user'/NAME
    if destination.exists():raise RuntimeError('Back up/remove the existing receiver unit before replacing it')
    (a.root/'var/pnlink').mkdir(parents=True,exist_ok=True,mode=0o700)
    destination.parent.mkdir(parents=True,exist_ok=True);destination.write_text(unit)
    subprocess.run(['systemctl','--user','daemon-reload'],check=True)
    print('Receive-only unit installed, not started; no robot services changed')


if __name__=='__main__':main()
