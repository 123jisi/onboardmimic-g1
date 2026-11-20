"""Compile the production native receive-boundary filter; no robot/network I/O."""
from pathlib import Path
import subprocess
import tempfile
import unittest

class StreamEpochTest(unittest.TestCase):
    def test_delayed_packets_across_joint_smpl_replays(self):
        root=Path(__file__).resolve().parents[1]
        include=root/'onboardmimic/backends/sonic/deploy/src/g1/g1_deploy_onnx_ref/include'
        with tempfile.TemporaryDirectory() as area:
            binary=Path(area)/'probe'
            subprocess.run(['g++','-std=c++17','-O3','-ffast-math','-Wall','-Wextra','-Werror',
                            '-I'+str(include),str(root/'tests/stream_epoch_probe.cpp'),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)
