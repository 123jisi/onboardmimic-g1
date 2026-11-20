"""No hardware: hand intent must follow gateway authority and fresh release."""
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from tests.sonic import test_pico_session as sessions
from onboardmimic.backends.sonic.pico.hands import PicoHandLink,TOPIC


class HandSessionTest(unittest.TestCase):
    setUp=sessions.SessionTest.setUp
    update=sessions.SessionTest.update
    release=sessions.SessionTest.release
    def test_trigger_threshold_sides_and_grip_does_not_close(self):
        self.release()
        self.assertEqual(self.p.hand_intent(),dict(left=False,right=False))
        self.left['trigger']=.51;self.right['grip']=1.;self.update()
        self.assertEqual(self.p.hand_intent(),dict(left=True,right=False))
        self.left['trigger']=.5;self.right['trigger']=1.;self.update()
        self.assertEqual(self.p.hand_intent(),dict(left=False,right=True))
    def test_entry_with_trigger_held_requires_release_per_side(self):
        self.release();self.left['trigger']=1.;self.update()
        self.assertEqual(self.p.hand_intent(),dict(left=None,right=False))
        self.left['trigger']=0.;self.update();self.p.hand_intent()
        self.left['trigger']=1.;self.update();self.assertTrue(self.p.hand_intent()['left'])
    def test_pause_freezes_hand_intent_and_loss_disarms(self):
        self.release();self.p.hand_intent();self.left['trigger']=1.;self.update();self.p.hand_intent()
        self.p.reference(self.now);self.left['menu']=True;self.update();self.p.events()
        self.left['trigger']=0.;self.update();self.assertTrue(self.p.hand_intent()['left'])
        self.now+=.3;self.assertEqual(self.p.hand_intent(),dict(left=None,right=None))
    def test_mode_keys_keep_hand_identity_but_revoke_and_new_session_reset(self):
        import numpy as np
        self.release();self.p.hand_intent();old=self.p.hand_session
        for mode in ('planner','frozen','vr3pt','pose'):
            self.p.enter(mode,np.zeros(29));self.assertEqual(self.p.hand_session,old)
        self.p.revoke();self.assertNotEqual(self.p.hand_session,old)
        self.assertEqual(self.p.hand_intent(),dict(left=None,right=None))


class HandWireTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'hands.json';self.now=1000000000
        self.path.write_text(json.dumps(dict(protocol=1,running=True,bridge_id='a'*32,updated_monotonic_ns=self.now)))
        self.link=PicoHandLink(self.path,lambda:self.now)
        self.p=SimpleNamespace(hand_session='b'*32,ready=lambda:True,hand_intent=lambda:dict(left=True,right=False))
        self.g=SimpleNamespace(pico_session=self.p,phase='active',pico_owned=lambda:True,mode_pending=False,task_active=lambda:False)
    def packet(self):return json.loads(self.link.message(self.g)[len(TOPIC):])
    def test_active_intent_and_inactive_barriers(self):
        self.assertTrue(self.packet()['active']);self.assertTrue(self.packet()['left'])
        for change in ({'phase':'stopped'},{'mode_pending':True},{'task_active':lambda:True},{'pico_owned':lambda:False}):
            before={k:getattr(self.g,k) for k in change}
            for k,v in change.items():setattr(self.g,k,v)
            v=self.packet();self.assertFalse(v['active']);self.assertIsNone(v['left']);self.assertEqual(v['session'],'')
            for k,v in before.items():setattr(self.g,k,v)
    def test_stale_worker_produces_no_packet(self):
        self.now+=500000001;self.assertFalse(self.link.status()['fresh']);self.assertIsNone(self.link.message(self.g))
    def test_stopped_gateway_needs_no_pico_session(self):
        self.g.pico_session=None;self.g.phase='stopped';self.assertFalse(self.packet()['active'])
