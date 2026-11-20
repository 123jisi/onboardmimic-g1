#!/usr/bin/env bash
# Compile only. Never run the physical hand adapter or a vendor example.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cmake -S "$root" -B "$root/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_DDS_HANDS=ON "$@"
cmake --build "$root/build" --clean-first --parallel 2
(cd "$root/build" && ctest --output-on-failure)
python3 - "$root" <<'PY'
import hashlib,json,pathlib,platform,sys,subprocess
from datetime import datetime,timezone
root=pathlib.Path(sys.argv[1]);repo=root.parents[3]
files=('core.hpp','core_test.cpp','bridge.cpp','CMakeLists.txt','build.sh')
record=dict(built_at_utc=datetime.now(timezone.utc).isoformat(),compiler=subprocess.check_output(['c++','--version'],text=True).splitlines()[0],architecture=platform.machine(),sources={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in files},binary_sha256=hashlib.sha256((root/'build/pico_hand_bridge').read_bytes()).hexdigest(),motor_output=False)
(repo/'var').mkdir(exist_ok=True);(repo/'var/pico-hands-build.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record,indent=2))
PY
