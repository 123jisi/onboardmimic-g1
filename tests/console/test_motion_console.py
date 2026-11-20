"""Real console -> authenticated HTTP runtime integration; no hardware adapters run."""
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from onboardmimic.core.client import OnboardClient
from onboardmimic.core.gateway import make_server as gateway_server
from onboardmimic.console.client import OnboardBackend
from onboardmimic.console.server import Console, make_server
from tests.fixtures import NativeLifecycle
from tests.runtime.test_local_playback import write_clip

class MotionConsoleTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        root=Path(self.temp.name);write_clip(root/'motion')
        self.g=NativeLifecycle([0.]*29,Mock(),lambda:'',lambda:None,physical=True)
        self.g.enable_playback(root)
        api=gateway_server(self.g,'x'*40);self.start_server(api)
        backend=OnboardBackend(OnboardClient('http://127.0.0.1:'+str(api.server_port),'x'*40))
        connection=Mock();connection.status.side_effect=lambda status,runner:status
        self.console=Console(backend,Path(__file__).resolve().parents[2]/'onboardmimic/console/static',connection=connection)
        web=make_server(self.console);self.start_server(web)
        self.url='http://127.0.0.1:'+str(web.server_port)
        self.actor='operator-browser-a'

    def start_server(self,server):
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(thread.join,2);self.addCleanup(server.server_close);self.addCleanup(server.shutdown)

    def post(self,route,**data):
        state=self.g.status()
        request=dict(actor=self.actor,operator_generation=state['operator']['generation'],expected_revision=state['revision'],**data)
        return json.load(urlopen(Request(self.url+route,data=json.dumps(request).encode(),headers={
            'X-Control-Token':self.console.token,'Content-Type':'application/json'}),timeout=3))

    def test_catalog_refresh_register_and_load_never_start_controller(self):
        self.post('/api/operator',action='acquire')
        self.post('/api/motions',action='refresh')
        for _ in range(200):
            catalog=json.load(urlopen(self.url+'/api/motions'))
            if catalog['state']!='scanning':break
            time.sleep(.01)
        self.assertEqual(catalog['entries'][0]['status'],'needs_confirmation')
        self.post('/api/imports',action='register',motion_id='cache/motion',fps=50,joint_order='sonic_reference')
        for _ in range(200):
            if self.g.conversion['stage']!='running' and self.g.library.state!='scanning':break
            time.sleep(.01)
        self.assertEqual(self.g.conversion['stage'],'finished')
        self.post('/api/playback',action='load_async',motion_id='cache/motion')
        for _ in range(200):
            state=json.load(urlopen(self.url+'/api/controller'))
            if state['motion_task']['stage'] in ('finished','error'):break
            time.sleep(.01)
        self.assertEqual(state['motion_task']['stage'],'finished')
        self.assertEqual(state['state'],'stopped');self.assertEqual(state['pids'],[])
        self.g.launch.assert_not_called()

    def test_new_routes_keep_origin_and_token_protection(self):
        for route in ('/api/operator','/api/motions','/api/imports','/api/playback','/api/inputs/pnlink'):
            for headers in ({},{'X-Control-Token':self.console.token,'Origin':'http://evil.example'}):
                with self.subTest(route=route,headers=headers),self.assertRaises(HTTPError) as caught:
                    urlopen(Request(self.url+route,data=b'{"action":"stop"}',headers=headers))
                self.assertEqual(caught.exception.code,403)

    def test_takeover_and_stop_are_visible_across_clients(self):
        self.post('/api/operator',action='acquire')
        self.actor='operator-browser-b'
        with self.assertRaises(HTTPError):self.post('/api/playback',action='load_async',motion='motion')
        self.post('/api/playback',action='stop')
        self.post('/api/operator',action='takeover')
        self.assertEqual(self.g.operator,self.actor)

    def test_both_layouts_serve_motion_controls_before_activation(self):
        for path in ('/','/remote'):
            html=urlopen(self.url+path).read().decode()
            self.assertIn('id="motion-library"',html)
            self.assertIn('/motions.js',html)
            self.assertIn('id="pnlink-panel"',html)
            self.assertIn('/pnlink.js',html)
            self.assertLess(html.index('id="motion-library"'),html.index('id="workspace"'))

    def test_named_conversion_through_console_replaces_without_starting(self):
        import joblib
        import numpy as np
        from onboardmimic.backends.sonic.motion_library import scan_catalog
        source_root=Path(self.temp.name)/'sources';(source_root/'gmr').mkdir(parents=True)
        joblib.dump(dict(fps=30,dof_pos=np.zeros((4,29)),root_pos=np.zeros((4,3)),root_rot=np.tile([0,0,0,1],(4,1))),source_root/'gmr/wave.pkl')
        self.g.library.source_root=source_root
        self.g.library.entries=scan_catalog(self.g.library.root,source_root)
        self.post('/api/operator',action='acquire')
        for name in ['gmr_初次','gmr_新动作']:
            self.post('/api/imports',action='convert',motion_id='gmr/wave.pkl',joint_order='mujoco',output_name=name)
            for _ in range(300):
                if self.g.conversion['stage']!='running' and self.g.library.state!='scanning':break
                time.sleep(.02)
            self.assertEqual(self.g.conversion['stage'],'finished',self.g.conversion)
            self.assertEqual(self.g.conversion['result']['motion'],name)
            if name=='gmr_初次':
                from onboardmimic.backends.sonic.motion_library import validated_cache
                self.g.playback.load(validated_cache(self.g.library.root,name))
        self.assertEqual(self.g.playback.state,'unloaded');self.assertIsNone(self.g.playback.clip)
        self.assertFalse((self.g.library.root/'gmr_初次').exists())
        self.assertTrue((self.g.library.root/'gmr_新动作').is_dir())
        self.assertEqual(self.g.status()['state'],'stopped');self.assertEqual(self.g.status()['pids'],[])
        self.g.launch.assert_not_called()
