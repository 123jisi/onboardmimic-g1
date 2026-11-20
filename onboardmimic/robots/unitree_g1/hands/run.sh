#!/usr/bin/env bash
# Starts only the hand adapter; it waits for explicit gateway PICO authority.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "$root"
interface="$(.venv/bin/python -c 'from onboardmimic.robots.unitree_g1.bridge.network import detect; print(detect())')"
exec "$root/onboardmimic/robots/unitree_g1/hands/build/pico_hand_bridge" --physical --interface "$interface" --status "$root/var/pico-hands.json"
