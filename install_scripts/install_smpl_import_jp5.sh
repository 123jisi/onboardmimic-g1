#!/usr/bin/env bash
# Install an offline PT importer; never starts or contacts a controller.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == "--help" ]]; then
    echo "Usage: bash $0 [WHEELHOUSE]"
    echo "G1 JP5 / Python 3.8 / aarch64 only; installs into .venv_import."
    echo "With WHEELHOUSE, installation uses local wheels without network."
    exit 0
fi
if (( $# > 1 )); then
    echo "Expected at most one wheelhouse directory." >&2
    exit 2
fi
/usr/bin/python3 - <<'PY'
import platform, sys
if (sys.version_info[:2], platform.system(), platform.machine()) != ((3, 8), 'Linux', 'aarch64'):
    raise SystemExit('This installer requires G1 JP5: Linux aarch64, /usr/bin/python3 3.8.')
PY
VENV="$ROOT/.venv_import"
if [[ -L "$VENV" ]]; then
    echo "Refusing a symlink at .venv_import; use a separate importer environment." >&2
    exit 1
fi
if [[ ! -e "$VENV" ]]; then
    /usr/bin/python3 -m venv "$VENV"
fi
"$VENV/bin/python" - "$VENV" <<'PY'
from pathlib import Path
import sys
target = Path(sys.argv[1]).resolve()
config = (target/'pyvenv.cfg').read_text().lower()
if Path(sys.prefix).resolve() != target or sys.version_info[:2] != (3, 8) or 'include-system-site-packages = false' not in config:
    raise SystemExit('Expected an isolated Python 3.8 .venv_import; existing environment was not modified.')
PY
PIP_ARGS=(--index-url https://pypi.org/simple --only-binary=:all:)
if (( $# == 1 )); then
    [[ -d "$1" ]] || { echo "Wheelhouse not found: $1" >&2; exit 1; }
    PIP_ARGS=(--no-index --find-links "$1" --only-binary=:all:)
fi
PIP_CONFIG_FILE=/dev/null PIP_EXTRA_INDEX_URL='' "$VENV/bin/python" -m pip install \
    "${PIP_ARGS[@]}" -r "$ROOT/onboardmimic/backends/sonic/requirements-smpl-import-jp5.txt"
"$VENV/bin/python" -m pip check
"$VENV/bin/python" - <<'PY'
import io, json, torch, numpy, scipy
if torch.version.cuda is not None:
    raise SystemExit('Expected CPU Torch; CUDA build is not used by this importer.')
buf = io.BytesIO()
torch.save({'body_pose': torch.zeros(2, 63)}, buf)
buf.seek(0)
assert torch.load(buf, map_location='cpu', weights_only=True)['body_pose'].shape == (2, 63)
print(json.dumps(dict(torch=torch.__version__, cuda=torch.version.cuda,
                      numpy=numpy.__version__, scipy=scipy.__version__,
                      weights_only_load=True, controller_started=False)))
PY
echo "Ready: $VENV/bin/python -m onboardmimic.backends.sonic.motion_import smpl INPUT.pt OUTPUT.npz --fps SOURCE_FPS --group body_params_global"
