#!/usr/bin/python3
"""Move the PN Link receive address between G1 Ethernet and Wi-Fi.

status is read-only. wifi/wired/restore preview unless --apply is supplied.
Run as unitree with /usr/bin/python3, not sudo; only network commands use sudo.
Never starts a receiver, capture, calibration, bridge, policy, or robot motion.
"""
import argparse
import copy
from datetime import datetime
import fcntl
import ipaddress
import json
import os
from pathlib import Path
import platform
import re
import secrets
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from onboardmimic.core.client import OnboardClient
from onboardmimic.inputs.pnlink.install_service import render as render_receiver
from scripts.install_services import write_atomic

# Documentation-only test addresses. The CLI reads the local receiver configuration.
LOCAL = '192.0.2.10'
CIDR = LOCAL + '/24'
SERVER = '192.0.2.20'
ROBOT = '192.168.123.161'
INTERNAL_NET = ipaddress.IPv4Network('192.168.123.0/24')
WIRED, WIFI = 'eth0', 'wlan0'
SERVICE = 'onboardmimic-pnlink-receiver.service'
ROUTE = SERVER + '/32 0.0.0.0 50'
STATE = ROOT / 'var/pnlink/network-switch'
NM = 'org.freedesktop.NetworkManager'
RECEIVER_CONFIG = ROOT/'var/pnlink/receiver-config.json'


class Site:
    def __init__(self, cidr=CIDR, server=SERVER, previous=None):
        interface = ipaddress.IPv4Interface(cidr)
        device = ipaddress.IPv4Address(server)
        if (device == interface.ip or device not in interface.network
                or device in (interface.network.network_address, interface.network.broadcast_address)
                or interface.ip in (interface.network.network_address, interface.network.broadcast_address)
                or interface.network.overlaps(INTERNAL_NET)):
            raise ValueError('接收地址和动捕盒必须为同一外部局域网内不同主机地址，且不能与内部 DDS 网段重叠')
        self.cidr, self.local, self.server = str(interface), str(interface.ip), str(device)
        self.accepted = {self.local: {self.cidr}}
        self.servers = {self.server}
        if previous is not None:
            self.accepted.setdefault(previous.local, set()).add(previous.cidr)
            self.servers.add(previous.server)

    @classmethod
    def from_receiver(cls, data, previous=None):
        local = data.get('local_cidr', data['local_ip']+'/24')
        if str(ipaddress.IPv4Interface(local).ip) != data['local_ip']:
            raise ValueError('动捕接收 IP 与网段参数不一致')
        return cls(local, data['server_ip'], previous)


DEFAULT_SITE = Site()


# A bounded ARP-only helper. No addresses, routes, device settings or files change.
# Passed as an argv item to sudo/python (no shell, no credentials in arguments).
ARP_HELPER = r'''
import ipaddress,json,socket,struct,sys,time,select
mode,iface,ip,source=sys.argv[1:]
if mode not in ('probe','announce') or iface not in ('eth0','wlan0'):raise SystemExit('Invalid ARP request')
ipaddress.IPv4Address(ip);ipaddress.IPv4Address(source)
mac=bytes.fromhex(open('/sys/class/net/'+iface+'/address').read().strip().replace(':',''))
s=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(0x0806));s.bind((iface,0));s.setblocking(False)
found=set()
for _ in range(3):
    for op in ((1,2) if mode=='announce' else (1,)):
        sender=ip if mode=='announce' else source
        arp=struct.pack('!HHBBH',1,0x0800,6,4,op)+mac+socket.inet_aton(sender)+(bytes([255])*6 if op==2 else bytes(6))+socket.inet_aton(ip)
        s.send(bytes([255])*6+mac+struct.pack('!H',0x0806)+arp)
    time.sleep(.05)
end=time.monotonic()+(1.2 if mode=='probe' else .1)
while time.monotonic()<end:
    if not select.select([s],[],[],.05)[0]:continue
    p=s.recv(2048)
    if len(p)>=42 and p[12:14]==b'\x08\x06' and p[20:22]==b'\x00\x02' and socket.inet_ntoa(p[28:32])==ip:
        found.add(':'.join('%02x'%v for v in p[22:28]))
print(json.dumps(sorted(found)))
'''


def run(*args, timeout=20):
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=timeout)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or
                           'Command failed: ' + args[0])
    return result.stdout.strip()


def private_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temp = path.with_name(path.name + '.tmp')
    fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    temp.replace(path)


def owned_route(route, site=DEFAULT_SITE):
    return (route.get('dest') in site.servers and route.get('prefix') == 32 and
            route.get('next-hop', '0.0.0.0') == '0.0.0.0' and route.get('metric') == 50 and
            not set(route) - {'dest', 'prefix', 'next-hop', 'metric'})


def other_routes(profile, site=DEFAULT_SITE):
    return [r for r in profile['routes'] if not owned_route(r, site)]


def desired_profiles(before, target, site=DEFAULT_SITE):
    desired = copy.deepcopy(before)
    for iface, profile in desired.items():
        profile['addresses'] = [a for a in profile['addresses']
                                if str(ipaddress.ip_interface(a).ip) not in site.accepted]
        profile['routes'] = other_routes(profile, site)
        if iface == target:
            profile['addresses'].append(site.cidr)
            profile['routes'].append(dict(dest=site.server, prefix=32, metric=50))
    return desired


def require_idle(value):
    if (value.get('mode') != 'onboard_real' or value.get('robot_id') != 'unitree_g1' or
            value.get('state') != 'stopped' or value.get('pids') != [] or
            value.get('mode_switch_pending') or value.get('pnlink_command_pending')):
        raise RuntimeError('先在网页停止机器人控制：必须 stopped、pids=[] 且无在途切换/动捕命令')
    for key in ('pico_hands', 'pnlink_hands'):
        hand = value.get(key) or {}
        if hand.get('active') or hand.get('sampling'):
            raise RuntimeError('请先退出手部控制与标定')
        worker = hand.get('worker') or hand
        if any(s.get('commanding') or s.get('admitted') for s in worker.get('sides', {}).values()):
            raise RuntimeError('手部仍在控制中，拒绝改网络')


def client():
    token = Path.home() / '.config/onboardmimic/http.token'
    if token.stat().st_mode & 0o077:
        raise RuntimeError('HTTP token must be private')
    return OnboardClient('http://127.0.0.1:18921', token.read_text().strip(), timeout=2)


class Network:
    def __init__(self, ssid, site=DEFAULT_SITE, wired_connection=None):
        try:
            import dbus
        except ImportError as exc:
            raise RuntimeError('请用 /usr/bin/python3 运行；需要系统自带 python3-dbus') from exc
        self.dbus, self.ssid = dbus, ssid
        self.site, self.wired_connection = site, wired_connection
        self.bus = dbus.SystemBus()
        self.manager = self.interface('/org/freedesktop/NetworkManager', NM)

    def interface(self, path, name):
        return self.dbus.Interface(self.bus.get_object(NM, path), name)

    def active_uuid(self, iface):
        device = self.manager.GetDeviceByIpIface(iface)
        active = self.interface(device, 'org.freedesktop.DBus.Properties').Get(NM + '.Device', 'ActiveConnection')
        if str(active) == '/':
            return None
        return str(self.interface(active, 'org.freedesktop.DBus.Properties').Get(NM + '.Connection.Active', 'Uuid'))

    def profiles(self):
        settings = self.interface('/org/freedesktop/NetworkManager/Settings', NM + '.Settings')
        rows = []
        for path in settings.ListConnections():
            # GetSettings excludes Wi-Fi secrets; GetSecrets is never called.
            data = self.interface(path, NM + '.Settings.Connection').GetSettings()
            connection = data['connection']
            wifi = data.get('802-11-wireless', {})
            ssid = bytes(wifi.get('ssid', [])).decode('utf-8', errors='replace')
            ipv4 = json.loads(json.dumps(data.get('ipv4', {})))
            addresses = ipv4.get('address-data', [])
            if any(set(a) - {'address', 'prefix'} for a in addresses):
                # Do not flatten and lose address labels/attributes.
                if str(connection['uuid']) in (self.active_uuid(WIRED), self.active_uuid(WIFI)) or ssid == self.ssid:
                    raise RuntimeError('网络地址包含自定义属性，需要人工检查后再切换')
                continue
            rows.append(dict(uuid=str(connection['uuid']), name=str(connection['id']),
                             type=str(connection['type']), ssid=ssid,
                             addresses=[a['address'] + '/' + str(a['prefix']) for a in addresses],
                             routes=ipv4.get('route-data', []),
                             protected={k: v for k, v in ipv4.items()
                                        if k not in ('addresses', 'address-data', 'routes', 'route-data')},
                             autoconnect=bool(connection.get('autoconnect', True))))
        return rows

    def snapshot(self):
        rows = self.profiles()
        wired_uuid, wifi_uuid = self.active_uuid(WIRED), self.active_uuid(WIFI)
        wired = [r for r in rows if r['uuid'] == wired_uuid and r['type'] == '802-3-ethernet']
        wifi = [r for r in rows if r['type'] == '802-11-wireless' and r['ssid'] == self.ssid]
        active_wifi = [r for r in wifi if r['uuid'] == wifi_uuid]
        wifi = active_wifi or wifi
        if self.wired_connection is not None:
            wired = [r for r in wired if self.wired_connection in (r['name'], r['uuid'])]
        if len(wired) != 1 or len(wifi) > 1:
            raise RuntimeError('需要唯一的已保存 Wi-Fi 配置 ' + self.ssid + '，以及已连接的 eth0 内部网配置')
        selected = wired + wifi
        for row in rows:
            if row not in selected and any(
                    str(ipaddress.ip_interface(address).ip) in self.site.accepted
                    for address in row['addresses']):
                raise RuntimeError('其他已保存的网络配置持有接收地址，请先用原配置切回或恢复：'+row['name'])
        return {WIRED: wired[0], **({WIFI: wifi[0]} if wifi else {})}

    def apply_profile(self, iface, wanted):
        current = self.snapshot()[iface]
        if current['uuid'] != wanted['uuid'] or current['protected'] != wanted['protected'] or current['autoconnect'] != wanted['autoconnect']:
            raise RuntimeError('网络配置被其他操作更改，拒绝覆盖：' + iface)
        if other_routes(current, self.site) != other_routes(wanted, self.site):
            raise RuntimeError('存在其他静态路由变更，拒绝覆盖：' + iface)
        have = {r['dest'] for r in current['routes'] if owned_route(r, self.site)}
        want = {r['dest'] for r in wanted['routes'] if owned_route(r, self.site)}
        if current['addresses'] == wanted['addresses'] and have == want:
            return
        args = ['sudo', '-n', 'nmcli', '--wait', '10', 'connection', 'modify', 'uuid', wanted['uuid'],
                'ipv4.addresses', ','.join(wanted['addresses'])]
        for address in sorted(have - want):
            args += ['-ipv4.routes', address+'/32 0.0.0.0 50']
        for address in sorted(want - have):
            args += ['+ipv4.routes', address+'/32 0.0.0.0 50']
        run(*args)
        if self.active_uuid(iface) == wanted['uuid']:
            # Reapply without disconnect/down/up; preserve DHCP and internal NIC.
            run('sudo', '-n', 'nmcli', '--wait', '10', 'device', 'reapply', iface)

    def addresses(self):
        return json.loads(run('ip', '-j', '-4', 'address', 'show'))

    def route(self, destination, source=None):
        args = ['ip', '-j', '-4', 'route', 'get', destination]
        if source:
            args += ['from', source]
        return json.loads(run(*args))[0]

    def arp(self, iface, address, source='0.0.0.0', announce=False):
        return json.loads(run('sudo', '-n', '/usr/bin/python3', '-c', ARP_HELPER,
                              'announce' if announce else 'probe', iface, address, source, timeout=5))


def address_owners(network, address=LOCAL):
    return [d['ifname'] for d in network.addresses()
            if any(a.get('local') == address for a in d.get('addr_info', []))]


def internal_ok(network):
    active = {a['local'] for d in network.addresses() if d['ifname'] == WIRED
              for a in d.get('addr_info', [])
              if a.get('local') and ipaddress.ip_address(a['local']) in INTERNAL_NET}
    route = network.route(ROBOT)
    if route.get('prefsrc') not in active or route.get('dev') != WIRED or route.get('gateway'):
        raise RuntimeError('G1 内部地址/直连路由异常，拒绝切换')
    return route


def internal_addresses(profile):
    return {address for address in profile['addresses']
            if ipaddress.ip_interface(address).ip in INTERNAL_NET}


def validate_profiles(profiles, site=DEFAULT_SITE):
    if WIRED not in profiles or not internal_addresses(profiles[WIRED]):
        raise RuntimeError('备份或当前配置必须保留 eth0 上的内部 DDS 地址')
    if profiles[WIRED]['protected'].get('method') != 'manual':
        raise RuntimeError('eth0 必须是已有的静态内部控制配置')
    if WIFI in profiles and profiles[WIFI]['protected'].get('method') != 'auto':
        raise RuntimeError('Wi-Fi 请保留自动 DHCP；脚本只附加固定接收地址')
    owners = []
    for iface, profile in profiles.items():
        for address in profile['addresses']:
            parsed = ipaddress.ip_interface(address)
            ip = str(parsed.ip)
            if ip in site.accepted:
                if address not in site.accepted[ip]:
                    raise RuntimeError('接收地址已有不同前缀，请先核对')
                owners.append(ip)
            if iface == WIFI and parsed.ip in INTERNAL_NET:
                raise RuntimeError('Wi-Fi 静态地址与 G1 内部控制网冲突')
        for route in profile['routes']:
            if route.get('dest') in site.servers and route.get('prefix') == 32 and not owned_route(route, site):
                raise RuntimeError('已有自定义动捕盒 /32 路由，请先人工检查')
    if len(set(owners)) != len(owners):
        raise RuntimeError('接收地址重复，请先人工核对')


def transition(network, wanted, site=DEFAULT_SITE):
    # Remove old addresses/routes first so either receive IP has at most one owner.
    for iface, current in network.snapshot().items():
        clean = copy.deepcopy(current)
        clean['addresses'] = [a for a in current['addresses']
                              if str(ipaddress.ip_interface(a).ip) not in site.accepted
                              or a in wanted[iface]['addresses']]
        wanted_routes = {r['dest'] for r in wanted[iface]['routes'] if owned_route(r, site)}
        clean['routes'] = [r for r in current['routes']
                           if not owned_route(r, site) or r['dest'] in wanted_routes]
        if clean != current:
            network.apply_profile(iface, clean)
    for iface, profile in wanted.items():
        network.apply_profile(iface, profile)


class SwitchFailure(RuntimeError):
    def __init__(self, error, rollback_error=None):
        self.rollback_error = rollback_error
        super().__init__(str(error) + ('；自动回退失败：' + str(rollback_error) if rollback_error else '；已恢复切换前的网络配置'))


def transaction(network, before, wanted, verify, site=DEFAULT_SITE):
    try:
        transition(network, wanted, site)
        return verify()
    except BaseException as exc:
        rollback_error = None
        try:
            transition(network, before, site)
            verify_profiles(network, before, site)
            for address in site.accepted:
                for iface in address_owners(network, address):
                    network.arp(iface, address, announce=True)
        except BaseException as rollback:
            rollback_error = str(rollback)
        raise SwitchFailure(exc, rollback_error) from exc


def verify_profiles(network, wanted, site=DEFAULT_SITE):
    actual = network.snapshot()
    for iface, b in wanted.items():
        a = actual[iface]
        def routes(profile):
            return {r['dest'] for r in profile['routes'] if owned_route(r, site)}
        if (a['addresses'] != b['addresses'] or a['protected'] != b['protected']
                or a['autoconnect'] != b['autoconnect'] or other_routes(a, site) != other_routes(b, site)
                or routes(a) != routes(b)):
            raise RuntimeError('持久化配置读回不一致：' + iface)
    verify_owners(network, wanted, site)
    return internal_ok(network)


def verify_owners(network, profiles, site):
    for address, cidrs in site.accepted.items():
        expected = [iface for iface, profile in profiles.items() if cidrs.intersection(profile['addresses'])
                    and (not hasattr(network, 'active_uuid') or
                         network.active_uuid(iface) == profile['uuid'])]
        if address_owners(network, address) != expected:
            raise RuntimeError('实际接收地址与目标网卡不一致: '+address)


def receiver_check(config):
    start = run('systemctl', '--user', 'show', SERVICE, '-p', 'ExecStart', '--value')
    for argument, value in (('local-ip', config['local_ip']), ('server-ip', config['server_ip']),
                            ('local-port', config.get('local_port', 8002))):
        if not re.search(r'--'+argument+r'\s+'+re.escape(str(value))+r'(?:\s|;)', start):
            raise RuntimeError('实际接收服务参数与配置不一致: '+argument)
    active = run('systemctl', '--user', 'show', SERVICE, '-p', 'ActiveState', '--value')
    path = ROOT/'var/pnlink/status.json'
    if active != 'inactive' and path.exists():
        command = json.loads(path.read_text()).get('command', {})
        if command.get('stage') in ('sending', 'running', 'unknown'):
            raise RuntimeError('接收器有在途/结果未知的设备命令，请先处理')
    return active


def update_receiver(before, after):
    if before == after:
        return
    target = Path.home()/'.config/systemd/user'/SERVICE
    if run('systemctl', '--user', 'show', SERVICE, '-p', 'DropInPaths', '--value'):
        raise RuntimeError('接收服务有自定义覆盖，请先核对')
    old_unit = target.read_text()
    old_config = RECEIVER_CONFIG.read_text()
    new_unit = render_receiver(ROOT, after)
    try:
        write_atomic(target, new_unit)
        write_atomic(RECEIVER_CONFIG, json.dumps(after, indent=2)+'\n', 0o600)
        run('systemctl', '--user', 'daemon-reload')
        receiver_check(after)
    except BaseException:
        write_atomic(target, old_unit)
        write_atomic(RECEIVER_CONFIG, old_config, 0o600)
        run('systemctl', '--user', 'daemon-reload')
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('status', 'wifi', 'wired', 'restore'))
    parser.add_argument('--apply', action='store_true', help='实际切换；省略则只预览')
    parser.add_argument('--ssid', required=True, help='现场已保存的 Wi-Fi SSID')
    parser.add_argument('--wired-connection', help='核对有线活动配置名称或 UUID')
    parser.add_argument('--local-cidr', help='G1 接收地址和前缀，例如 192.0.2.10/24')
    parser.add_argument('--server-ip', help='动捕盒地址')
    parser.add_argument('--local-port', type=int, help='接收 UDP 端口')
    parser.add_argument('--backup', type=Path, help='restore 使用的 backup.json')
    parser.add_argument('--offline', action='store_true', help='允许动捕盒关机时配置；不代表链路验收通过')
    args = parser.parse_args()
    if os.geteuid() == 0:
        parser.error('请以 unitree 运行 /usr/bin/python3，不要 sudo 整个脚本')
    if args.mode == 'status' and args.apply:
        parser.error('status 不接受 --apply')
    current_receiver = json.loads(RECEIVER_CONFIG.read_text())
    previous = Site.from_receiver(current_receiver)
    wanted_receiver = dict(current_receiver)
    if args.local_cidr:
        wanted_receiver.update(local_cidr=args.local_cidr,
                               local_ip=str(ipaddress.IPv4Interface(args.local_cidr).ip))
    if args.server_ip:
        wanted_receiver['server_ip'] = args.server_ip
    if args.local_port is not None:
        wanted_receiver['local_port'] = args.local_port
    backup = None
    if args.mode == 'restore':
        if not args.backup:
            parser.error('restore 需要 --backup PATH/backup.json')
        backup = json.loads(args.backup.read_text())
        if backup.get('schema') not in (1, 2) or backup.get('root') != str(ROOT) or backup.get('uid') != os.getuid():
            raise RuntimeError('备份与本部署/用户不匹配')
        wanted_receiver = backup.get('receiver_config', current_receiver)
        args.ssid = backup.get('ssid', args.ssid)
    if args.mode == 'status':
        wanted_receiver = current_receiver
    site = Site.from_receiver(wanted_receiver, previous)
    render_receiver(ROOT, wanted_receiver)
    network = Network(args.ssid, site, args.wired_connection)
    before = network.snapshot()
    validate_profiles(before, site)
    verify_owners(network, before, site)
    internal = internal_ok(network)
    if args.mode == 'status':
        try:
            value = client().status()
            control = {k: value.get(k) for k in ('mode', 'state', 'pids', 'input_source', 'pnlink_command_pending')}
        except Exception as exc:
            control = dict(error=str(exc))
        result = dict(mode='status', receive_address=site.cidr, active_owners=address_owners(network, site.local),
                      wifi_connected=WIFI in before and network.active_uuid(WIFI) == before[WIFI]['uuid'],
                      ssid=args.ssid, profiles=before, addresses=network.addresses(),
                      device_route=network.route(site.server), internal_route=internal,
                      controller=control, receiver_state=receiver_check(current_receiver))
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return
    if backup is not None:
        wanted = backup['profiles']
        validate_profiles(wanted, site)
        if internal_addresses(wanted[WIRED]) != internal_addresses(before[WIRED]):
            raise RuntimeError('恢复会更改内部 DDS 地址，已拒绝')
        if set(wanted) != set(before) or any(wanted[i]['uuid'] != before[i]['uuid'] for i in before):
            raise RuntimeError('原 NetworkManager 配置已被替换，拒绝恢复其他连接')
    else:
        if args.mode == 'wifi' and WIFI not in before:
            raise RuntimeError('请先通过系统网络菜单保存并连接 '+args.ssid)
        wanted = desired_profiles(before, WIFI if args.mode == 'wifi' else WIRED, site)
    target = next((i for i, profile in wanted.items() if site.cidr in profile['addresses']), None)
    if target == WIFI and network.active_uuid(WIFI) != before[WIFI]['uuid'] and backup is None:
        raise RuntimeError('请先通过系统网络菜单连接 ' + args.ssid + '，再切换接收地址')
    live_target = target if target and network.active_uuid(target) == before[target]['uuid'] else None
    if not args.apply:
        print(json.dumps(dict(preview=True, mode=args.mode, target=target,
                              before=before, after=wanted, receiver_config=wanted_receiver, receiver_after='inactive',
                              apply_command='加 --apply 执行；需要机器人 stopped、pids=[]',
                              note='同一 Wi-Fi 的 DHCP 管理地址保留；只迁移动捕接收地址与动捕盒 /32 路由'),
                         indent=2, ensure_ascii=False))
        return
    if platform.machine() not in ('aarch64', 'arm64'):
        raise RuntimeError('实际切换只允许在 G1 ARM64 上执行')
    STATE.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (STATE / 'switch.lock').open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('另一个网络切换正在运行') from exc
        require_idle(client().status())
        active = receiver_check(current_receiver)
        # Prompt once before any network mutation. Never accept a password argument.
        subprocess.run(['sudo', '-v'], check=True)
        require_idle(client().status())
        if network.snapshot() != before:
            raise RuntimeError('预检期间配置已变化，请重新执行')
        own_macs = {Path('/sys/class/net', i, 'address').read_text().strip().lower() for i in before}
        if live_target:
            collisions = set(network.arp(live_target, site.local)) - own_macs
            if collisions:
                raise RuntimeError(site.local+' 被其他设备使用，配置未改：' + ', '.join(sorted(collisions)))
            if not args.offline and not network.arp(live_target, site.server):
                raise RuntimeError('目标链路未收到动捕盒 ARP 应答，配置未改；请开机/检查 LAN，或用 --offline 仅配置')
        folder = STATE / (datetime.now().strftime('%Y%m%dT%H%M%S') + '-' + secrets.token_hex(3))
        record = dict(schema=2, root=str(ROOT), uid=os.getuid(), ssid=args.ssid,
                      receiver_config=current_receiver,
                      profiles=before, receiver_before=active, internal_route=internal,
                      actual_addresses=network.addresses())
        private_json(folder / 'backup.json', record)
        require_idle(client().status())
        receiver_check(current_receiver)
        # Survive loss of the SSH address being moved. Results are also kept on disk.
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        run('systemctl', '--user', 'stop', SERVICE)

        def verify():
            internal_route = verify_profiles(network, wanted, site)
            device_route = None
            device_macs = []
            if live_target:
                device_route = network.route(site.server, site.local)
                if device_route.get('dev') != live_target or device_route.get('gateway'):
                    raise RuntimeError('动捕盒实际路由没有切到目标接口')
                network.arp(live_target, site.local, announce=True)
                if not args.offline:
                    device_macs = network.arp(live_target, site.server, source=site.local)
                    if not device_macs:
                        raise RuntimeError('切换后动捕盒没有回应，执行回退')
            update_receiver(current_receiver, wanted_receiver)
            return dict(internal_route=internal_route, device_route=device_route,
                        device_macs=device_macs, device_link_verified=bool(device_macs))

        try:
            evidence = transaction(network, before, wanted, verify, site)
        except SwitchFailure as exc:
            result = dict(ok=False, error=str(exc), rollback_error=exc.rollback_error,
                          backup=str(folder / 'backup.json'), receiver_after='inactive')
            private_json(folder / 'result.json', result)
            private_json(STATE / 'last-result.json', result)
            raise
        result = dict(ok=True, mode=args.mode, receive_address=site.cidr, interface=target,
                      persistent=True, backup=str(folder / 'backup.json'), evidence=evidence,
                      receiver_after='inactive', robot_started=False,
                      web_url='http://' + site.local + ':8769/remote' if live_target else None,
                      next='网页手动启动动捕接收；预览/确认后人工进入遥操。未启动机器人，未验证真机运动。')
        private_json(folder / 'result.json', result)
        private_json(STATE / 'last-result.json', result)
        print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        print('PN Link 网络切换失败：' + str(error), file=sys.stderr)
        sys.exit(1)
