#!/bin/bash
set -euo pipefail
# Receive only: no robot SDK, motor devices, control token or publisher mounts.
mkdir -p /capture/service
cp -a --no-preserve=ownership /opt/xrt/opt/apps/roboticsservice/. /capture/service/
cd /capture/service
service_pid=''
receiver_pid=''
cleanup() {
 trap - EXIT TERM INT
 for pid in "$receiver_pid" "$service_pid"; do
  if [[ -n "$pid" ]]; then kill "$pid" 2>/dev/null || true; fi
 done
 for pid in "$receiver_pid" "$service_pid"; do
  if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
 done
 if [[ -d /root/.local/share/PICOBusinessSuitData/log ]]; then
  cp -r /root/.local/share/PICOBusinessSuitData/log /capture/service-internal-log || true
 fi
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT
./RoboticsServiceProcess > /capture/xrt-service.log 2>&1 &
service_pid=$!
# Check the receive listener without opening a competing device connection.
ready=0
for ((i=0;i<100;i++)); do
 kill -0 "$service_pid" 2>/dev/null || { echo 'XR service exited during startup' >&2; exit 1; }
 if python3 - <<'PY'
from pathlib import Path
port=f'{63901:04X}'
rows=Path('/proc/net/tcp').read_text().splitlines()[1:]+Path('/proc/net/tcp6').read_text().splitlines()[1:]
raise SystemExit(0 if any(r.split()[1].endswith(':'+port) and r.split()[3]=='0A' for r in rows) else 1)
PY
 then ready=1; break; fi
 sleep .2
done
[[ "$ready" == 1 ]] || { echo 'XR listener 63901 not ready after 20s' >&2; exit 1; }
record_args=()
if [[ "${PICO_RECORD:-0}" == 1 ]]; then record_args=(--record "/capture/frames-$(date +%s).jsonl"); fi
python3 /app/receiver.py --source sdk --config /config/devices.json --status /capture/status.json "${record_args[@]}" --duration "${PICO_CAPTURE_SECONDS:-60}" &
receiver_pid=$!
# Either process dying tears down the other. systemd restarts the whole pair.
set +e
wait -n "$service_pid" "$receiver_pid"
result=$?
set -e
if ! kill -0 "$service_pid" 2>/dev/null; then exit 1; fi
exit "$result"
