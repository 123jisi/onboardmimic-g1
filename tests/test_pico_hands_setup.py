import unittest
from pathlib import Path
from scripts.setup_pico_hands import render,DROP,NAME

class HandSetupTest(unittest.TestCase):
    def test_optional_units_only_enable_hand_adapter_and_idle_api_parameter(self):
        root=Path('/opt/test-deployments/teleop-phase5')
        unit,api=render(root)
        self.assertIn('onboardmimic/robots/unitree_g1/hands/run.sh',unit)
        self.assertIn('WantedBy=default.target',unit)
        self.assertIn('--pose-status ',api);self.assertIn('--pico-hands-status ',api)
        self.assertNotIn('start_policy',unit+api);self.assertNotIn('prepare_pose',unit+api)
        self.assertNotIn('native_bridge',unit);self.assertNotIn('brainco_hand_server',unit)
        self.assertGreater(DROP,'99-phase5-acceptance.conf')
    def test_changed_sources_or_binary_are_rejected_before_install(self):
        import hashlib,json,tempfile
        from scripts.setup_pico_hands import verify_build
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'onboardmimic/robots/unitree_g1/hands'
            (folder/'build').mkdir(parents=True);(root/'var').mkdir()
            names=('core.hpp','core_test.cpp','bridge.cpp','CMakeLists.txt','build.sh')
            for name in names:(folder/name).write_text('source')
            binary=folder/'build/pico_hand_bridge';binary.write_bytes(b'\x7fELF'+bytes(14)+(183).to_bytes(2,'little'))
            record=dict(sources={n:hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in names},binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
            (root/'var/pico-hands-build.json').write_text(json.dumps(record))
            self.assertEqual(verify_build(root),record)
            (folder/'core.hpp').write_text('changed')
            with self.assertRaisesRegex(RuntimeError,'source changed'):verify_build(root)
            (folder/'core.hpp').write_text('source');binary.write_bytes(binary.read_bytes()+b'changed')
            with self.assertRaisesRegex(RuntimeError,'binary mismatch'):verify_build(root)
