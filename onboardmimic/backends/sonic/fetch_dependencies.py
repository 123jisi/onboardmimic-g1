"""Fetch the pinned native toolchain dependencies for the source build."""
from hashlib import sha256
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from zipfile import ZipFile

from onboardmimic.core.paths import ROOT

DEPLOY = ROOT / 'onboardmimic/backends/sonic/deploy'
DEPS = DEPLOY / 'onboard/deps'
ORT_DIRECTORY = 'onnxruntime-linux-aarch64-1.16.3'
ORT_PACKAGE = (
    'https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/1.16.3/'
    'microsoft.ml.onnxruntime.1.16.3.nupkg',
    '84d247f3650fa15c00a4568a80f304050e1da4dca540bbe6d17493da8350242a',
)
CPPZMQ = {
    'zmq.hpp': '1f8b641161dcf12641ae4951c2c49552de425be88d66c988ec5e85046f1320f6',
    'zmq_addon.hpp': '1dc1d551d7eca43ce31031b2104c0acb204e97fe0921e75741be5a8448ce8814',
    'LICENSE': '7e6db90e2e805c72b3daa77849d1cb5f1f92da0e69750c8a4aa1d86d70704748',
}


def checksum(path):
    digest = sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(data)
    return digest.hexdigest()


def download(url, destination, expected):
    if destination.is_file() and checksum(destination) == expected:
        return
    with tempfile.TemporaryDirectory(dir=destination.parent) as directory:
        temporary = Path(directory)/destination.name
        subprocess.run(['curl', '--fail', '--location', '--retry', '3',
                        '--connect-timeout', '20', '--max-time', '300',
                        '-o', str(temporary), url], check=True)
        if checksum(temporary) != expected:
            raise ValueError('Dependency checksum mismatch: '+destination.name)
        temporary.replace(destination)


def install_ort(destination):
    if destination.exists():
        if not (destination/'include/onnxruntime_cxx_api.h').is_file() or not (
                destination/'lib/libonnxruntime.so.1.16.3').is_file():
            raise ValueError('Move incomplete ONNX Runtime directory aside and retry: '+str(destination))
        return
    with tempfile.TemporaryDirectory(dir=destination.parent) as directory:
        stage = Path(directory)
        package = stage/'onnxruntime.nupkg'
        download(ORT_PACKAGE[0], package, ORT_PACKAGE[1])
        tree = stage/'ort'
        (tree/'include').mkdir(parents=True)
        (tree/'lib').mkdir()
        with ZipFile(package) as archive:
            for member in archive.namelist():
                path = Path(member)
                if path.parent == Path('build/native/include') and path.suffix == '.h':
                    (tree/'include'/path.name).write_bytes(archive.read(member))
            for name in ('LICENSE', 'ThirdPartyNotices.txt'):
                (tree/name).write_bytes(archive.read(name))
            library = tree/'lib/libonnxruntime.so.1.16.3'
            with archive.open('runtimes/linux-arm64/native/libonnxruntime.so') as source, library.open('wb') as target:
                shutil.copyfileobj(source, target)
        (tree/'lib/libonnxruntime.so').symlink_to(library.name)
        tree.rename(destination)


def checkout_sdk(destination, spec):
    if not destination.exists():
        destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=destination.parent) as directory:
            stage = Path(directory)/'sdk'
            subprocess.run(['git', 'init', str(stage)], check=True)
            subprocess.run(['git', '-C', str(stage), 'remote', 'add', 'origin', spec['url']], check=True)
            subprocess.run(['git', '-C', str(stage), 'fetch', '--depth', '1', 'origin', spec['revision']], check=True)
            subprocess.run(['git', '-C', str(stage), 'checkout', '--detach', 'FETCH_HEAD'], check=True)
            stage.rename(destination)
    head = subprocess.check_output(['git', '-C', str(destination), 'rev-parse', 'HEAD'], text=True).strip()
    if head != spec['revision']:
        raise ValueError('Existing SDK2 checkout differs from the revision in resources.json')


def main():
    spec = json.loads((ROOT/'onboardmimic/backends/sonic/resources.json').read_text())['unitree_sdk2']
    checkout_sdk(DEPLOY/'thirdparty/unitree_sdk2', spec)
    DEPS.mkdir(exist_ok=True)
    install_ort(DEPS/ORT_DIRECTORY)
    cpp = DEPS/'cppzmq'
    cpp.mkdir(exist_ok=True)
    for name, expected in CPPZMQ.items():
        download('https://raw.githubusercontent.com/zeromq/cppzmq/v4.10.0/'+name, cpp/name, expected)
    print('Native build dependencies ready.')


if __name__ == '__main__':
    main()
