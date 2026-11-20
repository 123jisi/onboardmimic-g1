"""Read-only deployment inventory. Never reads tokens or contacts motor interfaces."""
import argparse
import hashlib
import json
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def command(argv):
    result = subprocess.run(argv, capture_output=True, text=True, timeout=15)
    return dict(code=result.returncode, stdout=result.stdout.strip(), stderr=result.stderr.strip())


def capture(root):
    root = root.resolve()
    manifest = root / 'prebuilt_manifest.json'
    release = json.loads(manifest.read_text()) if manifest.exists() else {}
    deploy = root / 'onboardmimic/backends/sonic/deploy'
    resources = [p for p in deploy.rglob('*') if p.is_file() and
                 (p.suffix in ('.onnx', '.engine', '.trt') or p.name in
                  ('g1_deploy_onnx_ref', 'native_cache_manifest.json', 'baseline_q.json'))]
    sources = [p for folder in ('onboardmimic', 'scripts', 'tests')
               for p in (root / folder).rglob('*.py') if '__pycache__' not in p.parts]
    return dict(captured_utc=datetime.now(timezone.utc).isoformat(), root=str(root),
                system=dict(machine=platform.machine(), python=platform.python_version(),
                            kernel=platform.release(), os=platform.platform()),
                release={k: release.get(k) for k in ('version', 'source_commit', 'platform', 'provenance')},
                resources={str(p.relative_to(root)): sha256(p) for p in sorted(resources)},
                python_sources={str(p.relative_to(root)): sha256(p) for p in sorted(sources)},
                services=command(['systemctl', '--user', 'show', 'onboardmimic.service',
                                  'onboardmimic-console.service', '-p', 'ActiveState',
                                  '-p', 'ExecStart', '-p', 'WorkingDirectory']),
                pico_service=command(['systemctl', 'is-active', 'onboardmimic-pico-receiver.service']),
                addresses=command(['ip', '-br', 'address']), routes=command(['ip', 'route']),
                credential_paths=[str(Path.home()/'.config/onboardmimic/http.token'),
                                  str(Path.home()/'.ssh')])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(capture(args.root), indent=2, ensure_ascii=False))
