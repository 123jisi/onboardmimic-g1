"""Exercise the source installer without sudo, network services or a robot."""
from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from scripts import setup_g1_online as online


class OnlineSetupTest(unittest.TestCase):
    def test_fresh_clone_check_does_not_write_or_run_installer(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            with patch.object(Path, 'home', return_value=path), patch.object(online, 'preflight') as check, \
                    patch.object(online, 'run') as run, redirect_stdout(StringIO()):
                online.main(['--check'])
            check.assert_called_once()
            run.assert_not_called()
            self.assertEqual(list(path.iterdir()), [])
            self.assertIn('--no-playback', check.call_args.args[2]['onboardmimic.service'])

    def test_optional_user_units_keep_receivers_and_hardware_manual(self):
        units = online.render_units(Path('/tmp/test-config'), online.FEATURES, 'lan',
                                    {'local_ip': '192.0.2.10', 'server_ip': '192.0.2.20'})
        runtime = units['onboardmimic.service']
        self.assertNotIn('--no-playback', runtime)
        for flag in ('--pose-status', '--pnlink-status', '--pnlink-socket', '--pico-hands-status'):
            self.assertIn(flag, runtime)
        self.assertIn('--host 0.0.0.0', units['onboardmimic-console.service'])
        self.assertNotIn('[Install]', units[online.HAND_UNIT])
        self.assertNotIn(online.pnlink.NAME, online.install_services.BOOT_UNITS)
        self.assertNotIn('[Install]', units['onboardmimic-bridge.service'])

    def test_pnlink_requires_machine_specific_config(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(online, 'ROOT', Path('/opt/test-clone')):
            missing = Path(folder)/'absent.json'
            with self.assertRaisesRegex(RuntimeError, 'PN Link needs --pn-config'):
                online.read_receiver({'pnlink'}, missing)
            missing.write_text(json.dumps({'local_ip': '192.0.2.10', 'server_ip': '192.0.2.20'}))
            self.assertEqual(online.read_receiver({'pnlink'}, missing)['local_ip'], '192.0.2.10')

    def test_build_failure_preserves_services_and_can_retry(self):
        failed = subprocess.CalledProcessError(1, 'build-models')
        overlay = Path('/tmp/test-overlay/setup.bash')
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(Path, 'home', return_value=Path(folder)), \
                    patch.object(online, 'preflight', return_value='eth0'), patch.object(online, 'run'), \
                    patch.object(online, 'build_core', side_effect=[failed, overlay]), \
                    patch.object(online, 'build_features') as build_features, \
                    patch.object(online, 'configure_services') as configure:
                with self.assertRaises(subprocess.CalledProcessError):
                    online.main(['--features', 'playback'])
                build_features.assert_not_called()
                configure.assert_not_called()
                online.main(['--features', 'playback'])
                build_features.assert_called_once()
                configure.assert_called_once()
                self.assertEqual(configure.call_args.args[1:3], (overlay, 'eth0'))


if __name__ == '__main__':
    unittest.main()
