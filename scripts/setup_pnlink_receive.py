"""Persist only the G1 receive address; install a manually started receiver.

Preview by default. Never enables the receiver at boot, calibrates, or starts
robot control. Source/SDK deployment is separate from this machine configuration.
"""
import argparse
from datetime import datetime
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from onboardmimic.inputs.pnlink.install_service import render, NAME
from onboardmimic.core.client import OnboardClient


def run(*args):
    return subprocess.check_output(args, text=True).strip()


def require_idle(client):
    value = client.status()
    if (value.get('mode') != 'onboard_real' or value.get('robot_id') != 'unitree_g1'
            or value.get('state') != 'stopped' or value.get('pids') != []):
        raise RuntimeError('Require a stopped G1 with pids=[]; configuration unchanged')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface', default='eth0')
    p.add_argument('--local-ip', required=True)
    p.add_argument('--server-ip', required=True)
    p.add_argument('--apply', action='store_true')
    a = p.parse_args()
    local, server = ipaddress.IPv4Address(a.local_ip), ipaddress.IPv4Address(a.server_ip)
    network = ipaddress.IPv4Network(str(local)+'/24', strict=False)
    if server not in network or local == server:
        p.error('Require distinct receiver/device addresses in the same /24 LAN')
    config_path = ROOT/'var/pnlink/receiver-config.json'
    config = json.loads(config_path.read_text()) if config_path.exists() else {}
    config.update(local_ip=str(local), server_ip=str(server), auto_capture=True)
    unit = render(ROOT, config)
    if not a.apply:
        print(json.dumps(dict(interface=a.interface, append_address=str(local)+'/24',
                              receiver_config=config, service=NAME,
                              start_at_boot=False, start_now=False, unit=unit), indent=2))
        return
    if platform.machine() not in ('aarch64', 'arm64') or os.geteuid() == 0:
        raise RuntimeError('Run as the G1 deployment user, not root or an x86 host')
    interface = Path('/sys/class/net')/a.interface
    if '/' in a.interface or not interface.is_dir() or (interface/'wireless').exists():
        raise ValueError('Select the existing physical wired G1 interface')
    token = Path.home()/'.config/onboardmimic/http.token'
    if token.stat().st_mode & 0o077:
        raise ValueError('HTTP token must be private')
    client = OnboardClient('http://127.0.0.1:18921', token.read_text().strip(), timeout=2)
    require_idle(client)
    status_path = ROOT/'var/pnlink/status.json'
    if status_path.exists():
        command = json.loads(status_path.read_text()).get('command', {})
        if command.get('stage') in ('running', 'unknown'):
            raise RuntimeError('Wait for the pending device command before configuring reception')
    profile = run('nmcli', '-g', 'GENERAL.CON-UUID', 'device', 'show', a.interface)
    if profile in ('', '--'):
        raise ValueError('Interface must have an active NetworkManager profile')
    keys = ('ipv4.method', 'ipv4.addresses', 'ipv4.gateway', 'ipv4.routes', 'connection.autoconnect')
    before = {key: run('nmcli', '-g', key, 'connection', 'show', 'uuid', profile) for key in keys}
    if before['connection.autoconnect'] != 'yes':
        raise ValueError('Existing robot network profile must already autoconnect; preserved')
    sdk = ROOT/'onboardmimic/inputs/pnlink/vendor'
    manifest = json.loads((sdk/'sdk-install.json').read_text())
    for name, expected in manifest['installed_sha256'].items():
        path = (sdk/name).resolve(strict=True)
        if sdk.resolve() not in path.parents or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('SDK installation hash mismatch: '+name)
    target = Path.home()/'.config/systemd/user'/NAME
    dropins = run('systemctl', '--user', 'show', NAME, '-p', 'DropInPaths', '--value')
    if dropins:
        raise ValueError('Inspect existing receiver service overrides before replacing configuration')
    if target.exists() and 'onboardmimic.inputs.pnlink.receiver' not in target.read_text():
        raise ValueError('Existing service has another entrypoint; preserved')
    backup = Path.home()/'onboardmimic-backups'/('pnlink-receive-'+datetime.now().strftime('%Y%m%dT%H%M%S'))
    backup.mkdir(parents=True, mode=0o700)
    record = dict(interface=a.interface, profile_uuid=profile, before=before,
                  service_enabled_before=run('systemctl', '--user', 'show', NAME, '-p', 'UnitFileState', '--value'))
    if target.exists():shutil.copy2(target, backup/NAME)
    if config_path.exists():shutil.copy2(config_path, backup/'receiver-config.json')
    (backup/'network.json').write_text(json.dumps(record, indent=2)+'\n')
    subprocess.run(['sudo', '-v'], check=True)
    require_idle(client)
    cidr = str(local)+'/24'
    if cidr not in [x.strip() for x in before['ipv4.addresses'].split(',')]:
        subprocess.run(['sudo', 'nmcli', 'connection', 'modify', 'uuid', profile,
                        '+ipv4.addresses', cidr], check=True)
    active = json.loads(run('ip', '-j', '-4', 'address', 'show', 'dev', a.interface))
    if str(local) not in {row['local'] for device in active for row in device['addr_info']}:
        subprocess.run(['sudo', 'nmcli', 'device', 'modify', a.interface, '+ipv4.addresses', cidr], check=True)
    after = {key: run('nmcli', '-g', key, 'connection', 'show', 'uuid', profile) for key in keys}
    for key in keys:
        if key != 'ipv4.addresses' and before[key] != after[key]:
            raise RuntimeError('Unrelated network property changed; inspect '+str(backup))
    subprocess.run(['systemctl', '--user', 'stop', NAME], check=True)
    config_path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    config_path.write_text(json.dumps(config, indent=2)+'\n'); config_path.chmod(0o600)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(unit)
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', '--user', 'disable', NAME], check=True)
    record.update(after=after, configured_receiver=config, start_at_boot=False,
                  unit_state=run('systemctl', '--user', 'show', NAME, '-p', 'ActiveState,UnitFileState', '--no-pager'),
                  backup=str(backup), robot_started=False)
    (backup/'result.json').write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
