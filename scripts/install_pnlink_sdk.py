"""Install pinned SDK assets locally; no SDK loading, network or robot I/O."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil

ROOT=Path(__file__).resolve().parents[1]


def install(source, destination, machine=None):
    manifest=json.loads((ROOT/'onboardmimic/inputs/pnlink/sdk_sources.json').read_text())
    architecture=machine or platform.machine()
    library={'aarch64':'librobotapi_arm64.so','arm64':'librobotapi_arm64.so','x86_64':'librobotapi_x86-64.so'}.get(architecture)
    if library is None:raise ValueError('Unsupported PN Link SDK architecture')
    files=['mocap_api.py','lib/'+library]
    source_hashes = {}
    for name in files:
        data = (source/name).read_bytes()
        expected = {manifest['sdk_files'][name]}
        # The pinned public binding differs from the deployed copy by whitespace.
        if name == 'mocap_api.py':
            expected.add(manifest['upstream_binding_sha256'])
        source_hashes[name] = hashlib.sha256(data).hexdigest()
        if source_hashes[name] not in expected:
            raise ValueError('PN Link SDK source hash mismatch: '+name)
        if name.startswith('lib/') and data[:4]!=b'\x7fELF':raise ValueError('SDK library is not ELF')
    destination.mkdir(parents=True,exist_ok=True)
    (destination/'lib').mkdir(exist_ok=True)
    # Upstream imports this unused docutils symbol. Removing only that line
    # keeps the receive process stdlib-only; record both original and installed hashes.
    binding=(source/'mocap_api.py').read_text().replace('from docutils.parsers.rst.directives import encoding\n','')
    (destination/'mocap_api.py').write_text(binding)
    shutil.copy2(source/'lib'/library,destination/'lib'/library)
    result=dict(upstream=manifest['upstream'],source_sha256=source_hashes,
                installed_sha256={n:hashlib.sha256((destination/n).read_bytes()).hexdigest() for n in files},
                transform='Remove unused docutils encoding import only',architecture=architecture)
    (destination/'sdk-install.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True,help='Trusted pinned mocapapi_python SDK directory')
    p.add_argument('--destination',type=Path,default=ROOT/'onboardmimic/inputs/pnlink/vendor')
    a=p.parse_args();print(json.dumps(install(a.source,a.destination),indent=2))


if __name__=='__main__':main()
