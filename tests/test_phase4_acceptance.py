"""Field helper must reject active controllers and keep connection path aligned."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('acceptance',ROOT/'scripts/phase4_acceptance.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class AcceptanceTest(unittest.TestCase):
    def test_only_stopped_physical_api_is_accepted(self):
        client=Mock()
        for status in ({'mode':'onboard_real','state':'active','pids':[12]},
                       {'mode':'onboard_real','state':'error','pids':[]},
                       {'mode':'onboard_mock','state':'stopped','pids':[]}):
            client.status.return_value=status
            with self.assertRaises(RuntimeError):module.idle(client)
        client.status.return_value={'mode':'onboard_real','state':'stopped','pids':[]}
        module.idle(client)

    def test_only_idle_api_ui_and_connection_job_are_overridden(self):
        root=Path('/opt/test-deployments/teleop-phase4')
        units=module.overrides(root)
        self.assertEqual(set(units),set(module.NAMES))
        for data in units.values():
            self.assertIn('WorkingDirectory='+str(root),data)
            self.assertIn('ExecStart=\nExecStart=',data)
        self.assertIn(str(root/'var/motions'),units['onboardmimic.service'])
        self.assertIn(str(root/'onboardmimic/robots/unitree_g1/connect_services.sh'),units['onboardmimic-console-connect.service'])
        self.assertFalse(any('prepare_pose' in x or 'start_policy' in x for x in units.values()))
