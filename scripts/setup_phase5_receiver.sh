#!/bin/bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
[[ "$(uname -m)" == aarch64 ]] || { echo 'Run on G1 ARM64'; exit 1; }
[[ "$(basename "$PWD")" == teleop-phase5 ]] || { echo 'Run from teleop-phase5 candidate'; exit 1; }
.venv/bin/python scripts/phase5_acceptance.py check
echo 'Build and start receive-only PICO service. SONIC is not started.'
# sudo prompts only in the operator terminal; never put a password in a file.
sudo -v
python3 onboardmimic/inputs/pico/fetch_dependencies.py
sudo docker build -t onboardmimic-pico-receiver:20261113 onboardmimic/inputs/pico
.venv/bin/python scripts/phase5_acceptance.py receive-setup
.venv/bin/python scripts/phase5_acceptance.py receive-start
echo 'PICO receiver ready. In XRoboToolkit connect to G1 Wi-Fi IP, TCP 63901.'
echo 'No connect-services, posture preparation, policy start or motion requested.'
