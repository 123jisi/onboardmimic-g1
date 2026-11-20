"""Build a fresh G1 JP5 clone from online sources and start the idle console."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import tempfile

if __name__ == "__main__":
    sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from onboardmimic.backends.sonic import setup
from onboardmimic.backends.sonic.fetch_dependencies import checkout_sdk, download
from onboardmimic.backends.sonic.native_cache import verify_native_caches
from onboardmimic.core.client import OnboardClient
from onboardmimic.inputs.pico import install_service as pico
from onboardmimic.inputs.pnlink import install_service as pnlink
from onboardmimic.robots.unitree_g1.bridge import network
from onboardmimic.robots.unitree_g1.install_dependencies import build_workspace, install_apt
from onboardmimic.robots.unitree_g1.setup_support import env_value, probe_ros, require_idle, run
from scripts import install_services
from scripts.install_pnlink_sdk import install as install_sdk
from scripts.switch_pnlink_network import require_idle as require_idle_status

SONIC = ROOT/'onboardmimic/backends/sonic'
DEPLOY = SONIC/'deploy'
G1 = ROOT/'onboardmimic/robots/unitree_g1'
PICO = ROOT/'onboardmimic/inputs/pico'
PNLINK = ROOT/'onboardmimic/inputs/pnlink'
ROS = Path('/opt/ros/foxy/setup.bash')
TRTEXEC = Path('/usr/src/tensorrt/bin/trtexec')
FEATURES = frozenset(('playback', 'pico', 'pnlink', 'brainco'))
HAND_UNIT = 'onboardmimic-pico-hands.service'
PACKAGES = ('g++-10', 'curl', 'libmsgpack-dev', 'nlohmann-json3-dev', 'libzmq3-dev',
            'libgtest-dev', 'libeigen3-dev', 'zlib1g-dev', 'pkg-config', 'python3-dev')
HAND_SDK = {
    'url': 'https://github.com/unitreerobotics/unitree_sdk2.git',
    'revision': '07493e4b5b46ced303ffa6acb426b4c365c291f0',
}


def output(*command):
    return subprocess.check_output(list(map(str, command)), text=True, timeout=30).strip()


def check_platform():
    if os.geteuid() == 0 or platform.machine() != 'aarch64':
        raise RuntimeError('Run on an ARM64 G1 as the deployment user, without sudo.')
    release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
    if sys.version_info[:2] != (3, 8) or release.get('VERSION_ID', '').strip('"') != '20.04':
        raise RuntimeError('Ubuntu 20.04 and system Python 3.8 are required.')
    if not Path('/etc/nv_tegra_release').read_text().startswith('# R35 (release), REVISION: 3.1,'):
        raise RuntimeError('JetPack 5.1.1 / L4T 35.3.1 is required.')
    device = Path('/proc/device-tree/model').read_text().rstrip('\0')
    if 'Orin NX' not in device or int(Path('/proc/meminfo').read_text().split()[1]) < 12*1024**2:
        raise RuntimeError('The supported target is G1 EDU with Orin NX 16 GB.')
    for package, version in (('cuda-toolkit-11-4', '11.4'), ('libnvinfer8', '8.5.2')):
        if not output('dpkg-query', '-W', '-f=${Version}', package).startswith(version):
            raise RuntimeError('Factory package version mismatch: '+package)
    if not TRTEXEC.is_file():
        raise RuntimeError('The factory TensorRT trtexec is required: '+str(TRTEXEC))
    if not re.fullmatch(r'/[a-zA-Z0-9_./-]+', str(ROOT)) or not os.access(ROOT, os.W_OK):
        raise RuntimeError('Use a writable clone path without whitespace or special characters.')


def user_unit_options(content, arguments):
    return '\n'.join(line+' '+arguments if line.startswith('ExecStart=') else line
                     for line in content.split('\n'))


def render_units(config, features, access, receiver):
    units = install_services.render_units(ROOT, config)
    runtime, console = install_services.BOOT_UNITS
    quote = install_services.quote
    if access == 'lan':
        units[console] = units[console].replace('--host 127.0.0.1', '--host 0.0.0.0')
    if 'playback' not in features:
        units[runtime] = user_unit_options(units[runtime], '--no-playback')
    if 'pico' in features:
        for name in (runtime, console):
            units[name] = user_unit_options(units[name], '--pose-status '+quote(PICO/'runs/live/status.json'))
    if receiver is not None:
        state = ROOT/'var/pnlink'
        for name in (runtime, console):
            units[name] = user_unit_options(units[name], '--pnlink-status '+quote(state/'status.json'))
        units[runtime] = user_unit_options(units[runtime], '--pnlink-socket '+quote(state/'command.sock'))
        units[pnlink.NAME] = pnlink.render(ROOT, receiver)
    if 'brainco' in features:
        if features & {'pico', 'pnlink'}:
            units[runtime] = user_unit_options(units[runtime], '--pico-hands-status '+quote(ROOT/'var/pico-hands.json'))
        units[HAND_UNIT] = ('[Unit]\nDescription=OnboardMimic BrainCo hand adapter\n'
                            'Requires=brainco_hand.service\nAfter=brainco_hand.service\n'
                            '[Service]\nType=simple\nWorkingDirectory='+install_services.unit_path(ROOT)+'\n'
                            'ExecStart=/bin/bash '+quote(G1/'hands/run.sh')+'\n'
                            'KillMode=control-group\nRestart=no\nUMask=0077\n')
    return units


def read_receiver(features, supplied):
    if 'pnlink' not in features:
        return None
    local = ROOT/'var/pnlink/receiver-config.json'
    source = supplied or local
    if not source.is_file():
        raise RuntimeError('PN Link needs --pn-config FILE with local_ip and server_ip. See README.')
    value = json.loads(source.read_text())
    pnlink.render(ROOT, value)
    if value['local_ip'] == value['server_ip'] or value['local_ip'] == '0.0.0.0' or value['server_ip'] == '0.0.0.0':
        raise ValueError('PN Link needs distinct, configured IPv4 addresses.')
    if local.is_file() and json.loads(local.read_text()) != value:
        raise RuntimeError('Existing PN Link configuration preserved. Configure network changes separately.')
    return value


def require_idle_api(config, required=False):
    token = config/'http.token'
    if token.exists() and token.stat().st_mode & 0o077:
        raise RuntimeError('Existing API token must have mode 0600.')
    service = install_services.BOOT_UNITS[0]
    state = output('systemctl', '--user', 'show', service, '-p', 'ActiveState', '--value')
    if required or state == 'active':
        if not token.is_file():
            raise RuntimeError('Running API has no recognized local token file.')
        client = OnboardClient('http://127.0.0.1:18921', token.read_text().strip(), timeout=3)
        require_idle_status(client.status())


def preflight(args, config, units):
    check_platform()
    require_idle()
    require_idle_api(config)
    interface = network.detect()
    if min(shutil.disk_usage(ROOT).free, shutil.disk_usage('/').free) < (12 if args.features else 8)*1024**3:
        raise RuntimeError('Not enough disk space for downloads and native builds.')
    for option in (args.pn_sdk_source, args.hand_sdk_root):
        if option is not None and not option.is_dir():
            raise RuntimeError('Dependency directory does not exist: '+str(option))
    if 'brainco' in args.features and output('systemctl', 'show', 'brainco_hand.service', '-p', 'LoadState', '--value') != 'loaded':
        raise RuntimeError('BrainCo requires the installed factory brainco_hand.service.')
    unit_dir = Path.home()/'.config/systemd/user'
    for name in set(units) | {HAND_UNIT, pnlink.NAME}:
        target = unit_dir/name
        properties = output('systemctl', '--user', 'show', name, '-p', 'FragmentPath,DropInPaths')
        systemd = dict(line.split('=', 1) for line in properties.splitlines() if '=' in line)
        fragment = systemd.get('FragmentPath')
        if (systemd.get('DropInPaths') or (fragment and fragment != str(target))
                or target.is_symlink() or (target.exists() and target.read_text() != units.get(name))):
            raise RuntimeError('Existing service or override preserved: '+name)
    installed = Path('/etc/systemd/system')/pico.NAME
    if installed.exists():
        content, _ = pico.render(ROOT, config)
        if 'pico' not in args.features or installed.read_text() != content:
            raise RuntimeError('Existing Pico service preserved.')
    return interface


def build_core(features):
    extras = PACKAGES
    if 'pico' in features:
        extras += ('docker.io',)
    if 'pnlink' in features:
        extras += ('python3-dbus',)
    install_apt(extras)

    venv = ROOT/'.venv'
    python = venv/'bin/python'
    if venv.is_symlink():
        raise RuntimeError('Use a local .venv, without symlinking the environment directory.')
    if not python.is_file():
        run(['/usr/bin/python3', '-m', 'venv', '--system-site-packages', venv])
    pip = [python, '-m', 'pip', '--isolated', '--disable-pip-version-check', 'install',
           '--index-url', 'https://pypi.org/simple', '--only-binary=:all:']
    pip_env = dict(os.environ, PIP_CONFIG_FILE=os.devnull)
    run(pip+['pip==25.0.1'], env=pip_env)
    playback_deps = ['joblib==1.4.2'] if 'playback' in features else []
    run(pip+['-r', SONIC/'requirements-g1-jp5.txt', *playback_deps], env=pip_env)
    run([python, '-c', 'import onnxruntime, scipy, tensorrt; assert tensorrt.__version__.startswith("8.5.2")'])

    workspace = ROOT/'var/g1-ros2'
    workspace.mkdir(parents=True, exist_ok=True)
    overlay = workspace/'install/setup.bash'
    if not probe_ros(ROS, overlay)[0]:
        build_workspace(workspace, ROS)
    run([python, ROOT/'install_scripts/fetch_dependencies.py'])
    run(['bash', DEPLOY/'build_orin.sh'])
    run(['bash', G1/'native_bridge/build.sh'],
        env=dict(os.environ, ROS_SETUP=str(ROS), UNITREE_ROS_SETUP=str(overlay)))
    run(['bash', DEPLOY/'onboard/build_models.sh'], env=dict(os.environ, PYTHON=str(python)))
    verify_native_caches(DEPLOY)
    verify_libraries(overlay, DEPLOY/'target/release/g1_deploy_onnx_ref',
                     G1/'native_bridge/build/sonic_state_reader', G1/'native_bridge/build/sonic_command_bridge')
    return overlay


def verify_libraries(overlay, *paths):
    for binary in paths:
        dependencies = output('bash', '--noprofile', '--norc', '-c',
                              'source "$1" >/dev/null && source "$2" >/dev/null && '
                              'exec /lib/ld-linux-aarch64.so.1 --list "$3"',
                              'check-shared-libraries', ROS, overlay, binary)
        if 'not found' in dependencies:
            raise RuntimeError('Unresolved ARM64 library for '+str(binary))


def prepare_pnlink(source):
    if source is not None:
        install_sdk(source, PNLINK/'vendor', machine='aarch64')
        return
    spec = json.loads((PNLINK/'sdk_sources.json').read_text())
    repository = spec['upstream'].split('github.com/', 1)[1]
    url = 'https://raw.githubusercontent.com/'+repository+'/'+spec['upstream_revision']+'/'
    with tempfile.TemporaryDirectory(dir=ROOT/'var') as temporary:
        stage = Path(temporary)
        for name in ('mocap_api.py', 'lib/librobotapi_arm64.so'):
            destination = stage/name
            destination.parent.mkdir(exist_ok=True)
            expected = spec['upstream_binding_sha256'] if name == 'mocap_api.py' else spec['sdk_files'][name]
            download(url+name, destination, expected)
        install_sdk(stage, PNLINK/'vendor', machine='aarch64')


def build_features(args, overlay):
    if 'playback' in args.features:
        run(['bash', ROOT/'install_scripts/install_smpl_import_jp5.sh'])
    if 'pico' in args.features:
        run(['sudo', 'systemctl', 'start', 'docker.service'])
        run(['/usr/bin/python3', PICO/'fetch_dependencies.py'])
        run(['sudo', 'docker', 'build', '-t', pico.IMAGE, PICO])
    if 'pnlink' in args.features:
        prepare_pnlink(args.pn_sdk_source)
    if 'brainco' in args.features:
        sdk = args.hand_sdk_root or ROOT/'var/brainco-sdk2'
        if args.hand_sdk_root is None:
            checkout_sdk(sdk, HAND_SDK)
        run(['bash', G1/'hands/build.sh', '-DUNITREE_SDK_ROOT='+str(sdk)])
        verify_libraries(overlay, G1/'hands/build/pico_hand_bridge')


def configure_services(config, overlay, interface, units, args, receiver):
    config.mkdir(parents=True, exist_ok=True, mode=0o700)
    env = config/'bridge.env'
    xml = config/'cyclonedds.xml'
    if not xml.exists():
        network.configure(G1/'systemd/cyclonedds.xml.example', xml, interface)
    values = dict(ROS_SETUP=ROS, UNITREE_ROS_SETUP=overlay, CYCLONEDDS_URI=xml.as_uri())
    old = env.read_text().splitlines() if env.exists() else []
    lines = [line for line in old if line.partition('=')[0].strip() not in values]
    lines.extend(key+'='+env_value(value) for key, value in values.items())
    install_services.write_atomic(env, '\n'.join(lines)+'\n', 0o600)
    if receiver is not None:
        settings = ROOT/'var/pnlink/receiver-config.json'
        settings.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        if not settings.exists():
            install_services.write_atomic(settings, json.dumps(receiver, indent=2)+'\n', 0o600)
    run(['sudo', 'loginctl', 'enable-linger', output('id', '-un')])
    install_services.install(service_units=units, ensure_linger=False)
    if receiver is not None:
        run(['systemctl', '--user', 'disable', pnlink.NAME])
    if 'pico' in args.features:
        devices = PICO/'devices.json'
        if not devices.exists():
            shutil.copy2(PICO/'devices.example.json', devices)
        run(['sudo', 'chgrp', 'root', devices])
        run(['chmod', '0640', devices])
        run(['sudo', 'install', '-d', '-o', 'root', '-g', 'root', '-m', '0755', PICO/'runs/live'])
        content, _ = pico.render(ROOT, config)
        with tempfile.TemporaryDirectory() as temporary:
            unit = Path(temporary)/pico.NAME
            unit.write_text(content)
            run(['sudo', 'install', '-m', '0644', unit, Path('/etc/systemd/system')/unit.name])
        run(['sudo', 'systemctl', 'daemon-reload'])
        run(['sudo', 'systemctl', 'enable', '--now', pico.NAME])
    run(['systemctl', '--user', 'start', *install_services.BOOT_UNITS])
    setup.wait_console()
    require_idle_api(config, required=True)
    setup.show_urls(args.console_access)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Read-only platform and installation prerequisite checks.')
    parser.add_argument('--features', default='', help='Comma-separated subset of playback,pico,pnlink,brainco.')
    parser.add_argument('--console-access', choices=('local', 'lan'), default='local')
    parser.add_argument('--pn-config', type=Path, help='Local PN Link receiver JSON.')
    parser.add_argument('--pn-sdk-source', type=Path, help='Optional local copy of the pinned PN Link SDK.')
    parser.add_argument('--hand-sdk-root', type=Path, help='Optional existing compatible ARM64 Unitree SDK2.')
    args = parser.parse_args(argv)
    args.features = frozenset(name.strip() for name in args.features.split(',')) if args.features else frozenset()
    if not args.features <= FEATURES:
        parser.error('--features must select playback,pico,pnlink,brainco')
    if (args.pn_config or args.pn_sdk_source) and 'pnlink' not in args.features:
        parser.error('PN Link options require --features pnlink')
    if args.hand_sdk_root and 'brainco' not in args.features:
        parser.error('--hand-sdk-root requires --features brainco')
    config = Path.home()/'.config/onboardmimic'
    receiver = read_receiver(args.features, args.pn_config)
    units = render_units(config, args.features, args.console_access, receiver)
    preflight(args, config, units)
    if args.check:
        print('Read-only G1 preflight passed.')
        return
    # Units are per user, so concurrent installs in different clones use the same lock.
    config.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (config/'g1-online-install.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        preflight(args, config, units)
        run(['sudo', '-v'])
        overlay = build_core(args.features)
        build_features(args, overlay)
        interface = preflight(args, config, units)
        configure_services(config, overlay, interface, units, args, receiver)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, KeyError, subprocess.SubprocessError) as exc:
        sys.exit('G1 online setup incomplete: '+str(exc)+'\nResolve the reported stage, then rerun with the same options.')
