import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from scripts.setup_pnlink_hands import render,idle,verify_sources

class SetupTest(unittest.TestCase):
    def test_preserves_effective_pn_pico_motion_and_token_flags(self):
        root=Path('/opt/test-deployment')
        source='ExecStart=/old/python -m onboardmimic.core.runtime --physical --token-file %h/.config/token --pnlink-status /opt/test-deployment/var/pnlink/status.json --pnlink-socket /data/socket --pose-status /data/pico --motion-root /data/motions --pico-hands-status /old/hands.json\n'
        unit,drop=render(root,source)
        self.assertIn('shared PICO and PN',unit);self.assertIn('WorkingDirectory=/opt/test-deployment',unit)
        for flag in ('%h/.config/token','/data/socket','/data/pico','/data/motions'):
            self.assertIn(flag,drop)
        self.assertNotIn('/old/hands',drop);self.assertEqual(drop.count('--pico-hands-status'),1)
        self.assertIn('/opt/test-deployment/var/pico-hands.json',drop)
    def test_wrong_root_or_non_pn_runtime_refused(self):
        for source in ('ExecStart=python -m other','ExecStart=python -m onboardmimic.core.runtime --pnlink-status /wrong/status'):
            with self.assertRaises(ValueError):render(Path('/candidate'),source)
    def test_requires_stopped_body_and_inactive_hands(self):
        s=dict(mode='onboard_real',state='stopped',pids=[],pico_hands=dict(active=False,sides={}))
        client=SimpleNamespace(status=lambda:s);self.assertEqual(idle(client),s)
        for change in (dict(state='active'),dict(pids=[123]),dict(pico_hands=dict(active=True)),dict(pnlink_command_pending=True)):
            before=s.copy();s.update(change)
            with self.assertRaises(RuntimeError):idle(client)
            s.clear();s.update(before)
    def test_manifest_checks_actual_bytes(self):
        import hashlib,json
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'source.py').write_text('actual')
            manifest=root/'manifest.json';manifest.write_text(json.dumps({'source.py':hashlib.sha256(b'actual').hexdigest()}))
            self.assertEqual(verify_sources(root,manifest),1);(root/'source.py').write_text('changed')
            with self.assertRaises(ValueError):verify_sources(root,manifest)
