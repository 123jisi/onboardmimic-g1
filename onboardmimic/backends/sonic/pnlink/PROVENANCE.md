# PN Link hand mapping provenance

`hand_mapping.py` adapts the pure `Mapping`, pose validation and relative-rotation
math from `GR00T-WholeBodyControl/host_setup/g1_hands/mocap.py` (source SHA256
`95c2b60d4f74c6fccbad3bf5ea0047f824e7f0a9140e1736e4bae1cc2e87e78a`).
The source repository licenses its source code under Apache-2.0; the license and
copyright notice are retained in this repository's root `LICENSE`.

The onboard module removes host ZMQ/DDS imports, CLI and process ownership. It
retains v1/v2 calibration semantics, local SO(3) projection, per-finger overrides,
minimum measured span, grasp averaging, dead zone and six Revo2 channel caps.
Right-hand SDK joint ordering comes from the same repository's
`gear_sonic/scripts/mocap_live_play_smplx.py`.

`hands.py` adds onboard-only persistence, explicit authority, fresh frame capture
and no-output previews. The shared BrainCo worker implements host continuous
smoothing (0.12 s), 0.8/s slew, speed 0.3 and 20 Hz output; legacy PICO binary
commands keep their previously verified finger-before-thumb behavior.

`scripts/check_pnlink_hand_parity.py` reads only the actual host math AST and
creates synthetic fixtures; it never imports the host SDK, ROS or control code.
