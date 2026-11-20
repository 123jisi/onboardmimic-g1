import unittest
from pathlib import Path
from unittest.mock import Mock
from scripts.phase67_acceptance import overrides, idle, NAMES, CONNECT, UNIT_NAMES
from scripts.configure_pnlink_target import CaptureFields


class PNDeploymentTest(unittest.TestCase):
    def test_candidate_preserves_pico_hands_motion_assets_and_only_changes_idle_api(self):
        root=Path('/opt/test-deployments/teleop-phase6')
        current={NAMES[0]:'ExecStart=/old/.venv/bin/python -m onboardmimic.core.runtime --physical --pose-status /old/pico.json --pico-hands-status /old/hands.json --motion-root /old/motions --token-file %h/token\n',
                 NAMES[1]:'ExecStart=/old/.venv/bin/python -m onboardmimic.console.server --host 0.0.0.0 --port 8769 --pose-status /old/pico.json --token-file %h/token\n',
                 CONNECT:'ExecStart=/bin/bash /old/onboardmimic/robots/unitree_g1/connect_services.sh\n'}
        units=overrides(root,current)
        self.assertEqual(set(units),set(UNIT_NAMES))
        self.assertIn('--pose-status /old/pico.json',units[NAMES[0]])
        self.assertIn('--pico-hands-status /old/hands.json',units[NAMES[0]])
        self.assertIn('--motion-root /old/motions',units[NAMES[0]])
        self.assertIn('--pnlink-socket '+str(root/'var/pnlink/command.sock'),units[NAMES[0]])
        self.assertIn('WorkingDirectory='+str(root),units[CONNECT])
        self.assertIn(str(root/'onboardmimic/robots/unitree_g1/connect_services.sh'),units[CONNECT])
        self.assertNotIn('/old/',units[CONNECT])
        self.assertNotIn('--pnlink-',units[CONNECT])
        self.assertNotIn(CONNECT,NAMES)  # Only API/console may restart on deploy.
        for state in (dict(mode='onboard_real',state='active',pids=[1]),dict(mode='onboard_real',state='stopped',pids=[1])):
            client=Mock();client.status.return_value=state
            with self.assertRaises(RuntimeError):idle(client)

    def test_device_capture_parser_never_collects_wifi_credentials_and_preserves_wire_types(self):
        html='''<input name="pose-mode" value="human" checked><input name="protocol-mode" value="udp" checked>
        <select id="rotation-order"><option value="yxz" selected></option></select>
        <input id="group-tag" value="1234"><input id="udp-local-port" value="8080"><input id="udp-remote-port" value="8002">
        <input id="capture-udp-local-ip" value="192"><input id="capture-udp-local-ip" value="0"><input id="capture-udp-local-ip" value="2"><input id="capture-udp-local-ip" value="20">
        <input id="capture-udp-remote-ip" value="192"><input id="capture-udp-remote-ip" value="0"><input id="capture-udp-remote-ip" value="2"><input id="capture-udp-remote-ip" value="10">
        <input id="sensor-host-ip" value="255"><input id="sensor-host-ip" value="255"><input id="sensor-host-ip" value="255"><input id="sensor-host-ip" value="255">
        <input class="password-input" type="password" value="must-never-collect">'''
        parser=CaptureFields();parser.feed(html);config=parser.config()
        self.assertEqual(config['groupTag'],'1234')
        self.assertEqual(config['mocap']['udp']['remoteAddress'],'192.0.2.10')
        self.assertEqual(config['mocap']['tcp']['remoteAddress'],'')
        self.assertNotIn('must-never-collect',str(parser.values)+str(config))


if __name__=='__main__':unittest.main()
