import unittest
from pathlib import Path
from unittest.mock import Mock,patch
from scripts.phase5_acceptance import overrides,idle,NAME,IMAGE
from onboardmimic.inputs.pico.patch_sdk import patch as patch_sdk

class Phase5DeploymentTest(unittest.TestCase):
    def test_effective_override_after_phase4_keeps_all_runtime_options(self):
        root=Path('/opt/test-deployments/teleop-phase5');units=overrides(root)
        runtime=units['onboardmimic.service'];console=units['onboardmimic-console.service']
        status=str(root/'onboardmimic/inputs/pico/runs/live/status.json')
        self.assertIn('--pose-status '+status,runtime);self.assertIn('--pose-status '+status,console)
        self.assertIn('--motion-root '+str(root/'var/motions'),runtime)
        self.assertIn('--motion-source-root '+str(root/'var'),runtime)
        self.assertIn('--host 0.0.0.0 --port 8769',console)
        self.assertEqual(set(units),{'onboardmimic.service','onboardmimic-console.service','onboardmimic-console-connect.service'})
        from scripts.phase5_acceptance import OVERRIDE
        self.assertGreater(OVERRIDE,'99-phase4-acceptance.conf')
        self.assertGreater(OVERRIDE,'60-pico.conf')
    def test_every_mutating_action_requires_idle_no_pids(self):
        for s in (dict(mode='onboard_real',state='active',pids=[123]),dict(mode='onboard_real',state='stopped',pids=[123]),dict(mode='onboard_isolated',state='stopped',pids=[])):
            client=Mock();client.status.return_value=s
            with self.assertRaises(RuntimeError):idle(client)
        client=Mock();client.status.return_value=dict(mode='onboard_real',state='stopped',pids=[])
        self.assertEqual(idle(client)['pids'],[])
    def test_image_patch_and_arch_both_required(self):
        from scripts.phase5_acceptance import image_ready
        for label,arch,valid in (('phase5-controller-clock-v1','arm64',True),('old','arm64',False),('phase5-controller-clock-v1','amd64',False)):
            with patch('scripts.phase5_acceptance.run',side_effect=[Mock(stdout=label),Mock(stdout=arch)]):
                if valid:image_ready()
                else:
                    with self.assertRaises(RuntimeError):image_ready()
        self.assertIn('20261113',IMAGE)
    def test_complete_controller_clock_only_updates_in_matching_mutex(self):
        root=Path(__file__).parents[1];src=patch_sdk((root/'onboardmimic/inputs/pico/reference/xrt_py_bindings.cpp').read_text())
        for side in ('Left','Right'):
            low=side.lower();start=src.index('std::lock_guard<std::mutex> lock('+low+'Mutex);')
            end=src.index('\n                    }',start);part=src[start:end]
            self.assertLess(part.index(side+'ControllerTimeStampNs = 0;'),part.index(side+'ControllerPose ='))
            self.assertLess(part.index(side+'SecondaryButton ='),part.index(side+'ControllerTimeStampNs = '+low+'.value'))
            self.assertIn('get_'+low+'_controller_timestamp_ns',src)
