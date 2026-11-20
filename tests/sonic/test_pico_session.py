import copy
import json
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from onboardmimic.backends.sonic.pico.pose_reader import PoseReader
from onboardmimic.backends.sonic.pico.pose_mapping import UP,BASE,FLIP
from onboardmimic.backends.sonic.pico.session import PicoSession
from onboardmimic.backends.sonic.pico.kinematics import key_frames,raw_three_points,ThreePointCalibration
from onboardmimic.inputs.pico.receiver import Monitor
from tests.pico.test_receiver import config,frame

def controls():
    return dict(axes=[0.,0.],menu=False,click=False,primary=False,secondary=False,trigger=0.,grip=0.)

def body():
    p=np.zeros((24,7));p[:,3:]=(UP.inv()*BASE*FLIP.inv()).as_quat()
    p[22,:3]=[.1,.6,.1];p[23,:3]=[-.1,.6,.1];p[12,:3]=[0,.4,0]
    return p

def feed(reader,now,left=None,right=None,poses=None,receiver_id='test',left_stamp=None,right_stamp=None):
    stamp=1000000000+round(now*1e9)
    streams=dict(body=dict(fresh=True,age_ms=0,source_timestamp_ns=stamp,payload=(body() if poses is None else poses).tolist()))
    for side,payload,override in (('left',left,left_stamp),('right',right,right_stamp)):
        streams[side+'_controller']=dict(fresh=True,age_ms=0,source_timestamp_ns=stamp if override is None else override,payload=payload or controls())
    reader.feed(dict(updated_at_unix=100.,data_source='sdk',receiver_running=True,receiver_id=receiver_id,streams=streams),wall=100.)

def unpack(wire):
    h=json.loads(wire[4:1284].rstrip(b'\0 '));at=1284;out={}
    for f in h['fields']:
        dtype=np.dtype({'f32':'<f4','f64':'<f8','i64':'<i8','i32':'<i4','u8':'u1'}.get(f['dtype'],f['dtype']))
        n=int(np.prod(f['shape']));out[f['name']]=np.frombuffer(wire,dtype=dtype,count=n,offset=at).reshape(f['shape']);at+=n*dtype.itemsize
    return out

class SessionTest(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.r=PoseReader('/unused',lambda:self.now);self.left=controls();self.right=controls()
        self.p=PicoSession(self.r,max_speed=.5)
        for i in range(8):self.update(i*.02)
        self.p.enter('pose',np.zeros(29),True)
    def update(self,now=None,**kw):
        self.now=self.now+.02 if now is None else now
        feed(self.r,self.now,self.left,self.right,**kw)
    def release(self):self.assertIsNone(self.p.events());self.assertTrue(self.p.released)
    def test_controller_duplicate_and_other_stream_cannot_renew(self):
        stamp=self.r.controls['left']['stamp']
        for i in range(15):self.update(left_stamp=stamp)
        self.assertTrue(self.r.snapshot()['ready']);self.assertFalse(self.r.snapshot()['controllers_ready'])
        self.assertEqual(self.p.events(),'lost')
    def test_controller_regression_latched_until_receiver_generation(self):
        self.update(left_stamp=1);self.assertFalse(self.p.ready())
        self.assertTrue(self.r.snapshot()['control_fault'])
        self.assertIn('时间戳回退',self.r.snapshot()['controller_errors']['left'])
        self.update();self.assertFalse(self.p.ready())
        for i in range(8):self.update(receiver_id='new')
        self.assertTrue(self.p.ready());self.assertEqual(self.p.events(),'lost')
        self.assertEqual(self.r.snapshot()['controller_errors'],{})
    def test_buttons_held_on_entry_require_release(self):
        self.left['primary']=self.right['primary']=True;self.update()
        self.assertIsNone(self.p.events());self.assertFalse(self.p.released)
        self.left['primary']=self.right['primary']=False;self.update();self.release()
        self.left['primary']=self.right['primary']=True;self.update()
        self.assertEqual(self.p.events(),'planner');self.assertIsNone(self.p.events())
    def test_abxy_stop_precedes_ax_by_and_click(self):
        self.release()
        for c in (self.left,self.right):c.update(primary=True,secondary=True,click=True)
        self.update();self.assertEqual(self.p.events(),'stop')
        self.p.revoke();self.assertIsNone(self.p.events());self.assertFalse(self.p.active)
    def test_single_held_button_cannot_arm_new_session(self):
        for side in ('left','right'):
            for key in ('primary','secondary','click','menu'):
                with self.subTest(side=side,key=key):
                    self.left=controls();self.right=controls()
                    self.p.enter('planner',np.zeros(29),True)
                    getattr(self,side)[key]=True;self.update()
                    self.assertIsNone(self.p.events());self.assertFalse(self.p.released)
                    self.left['axes']=[0,1];self.update();self.p.events()
                    self.assertEqual(self.p.reference(self.now)['speed'],0.)
                    self.left=controls();self.right=controls();self.update();self.release()
    def test_sticks_held_at_entry_cannot_move(self):
        self.p.enter('planner',np.zeros(29),True);self.left['axes']=[0,1];self.update()
        self.assertIsNone(self.p.events());self.assertEqual(self.p.reference(self.now)['speed'],0.)
        self.left['axes']=[0,0];self.update();self.release()
    def test_original_button_chains(self):
        self.release();self.left['secondary']=self.right['secondary']=True;self.update()
        self.assertEqual(self.p.events(),'frozen')
        self.p.enter('frozen',np.zeros(29));self.left=controls();self.right=controls();self.update();self.p.events()
        self.left['click']=True;self.update();self.assertEqual(self.p.events(),'vr3pt')
        self.p.enter('vr3pt',np.zeros(29));self.left['click']=False;self.update();self.p.events()
        self.left['click']=True;self.update();self.assertEqual(self.p.events(),'frozen')
    def test_pose_pause_keeps_reference_but_tracking_required(self):
        self.release();before=unpack(self.p.reference(self.now))
        self.left['menu']=True;self.update();self.p.events();self.assertTrue(self.p.paused)
        during=unpack(self.p.reference(self.now))
        np.testing.assert_allclose(before['smpl_pose'],during['smpl_pose'])
        self.assertGreater(during['frame_index'][-1],before['frame_index'][-1])
        self.left['menu']=False;self.update();self.assertIsNone(self.p.events());self.assertFalse(self.p.paused)
        self.now+=.3;self.assertEqual(self.p.events(),'lost')
    def test_resume_large_jump_rejected(self):
        self.release();self.p.reference(self.now);self.left['menu']=True;self.update();self.p.events()
        moved=body();moved[0,3:]=(R.from_euler('y',70,degrees=True)*R.from_quat(moved[0,3:])).as_quat()
        for i in range(8):self.update(poses=moved)
        self.left['menu']=False;self.update(poses=moved)
        self.assertEqual(self.p.events(),'resume_rejected')
    def test_vr_and_frozen_fields_and_speed_bound(self):
        self.release();q=np.arange(29)*.001
        self.p.enter('frozen',q);self.left['axes']=[0,1];self.update();self.p.gait=3
        ref=self.p.reference(self.now);self.assertEqual(len(ref['upper_body_position']),17);self.assertEqual(ref['speed'],.5)
        self.p.enter('vr3pt',q);ref=self.p.reference(self.now)
        self.assertEqual(len(ref['vr_3pt_position']),9);self.assertEqual(len(ref['vr_3pt_orientation']),12)
    def test_pose_contains_three_points_and_increment(self):
        self.release();self.p.reference(self.now);self.right['axes']=[.4,0];self.update()
        ref=unpack(self.p.reference(self.now))
        self.assertAlmostEqual(float(ref['heading_increment'][0]),-.012,places=6)
        self.assertEqual(ref['vr_position'].shape,(9,));self.assertNotIn('left_hand_joints',ref)
    def test_planner_mapping_matches_original(self):
        self.release();self.p.enter('planner',np.zeros(29));self.p.gait=1
        self.p.reference(self.now);self.left['axes']=[.4,.7];self.right['axes']=[-.3,0];self.update()
        ref=self.p.reference(self.now)
        yaw=1.5*.3*.02;raw=np.hypot(.4,.7);mag=(raw-.15)/.85
        facing=[np.cos(yaw),np.sin(yaw)]
        expected=np.array([[-facing[1],facing[0]],[facing[0],facing[1]]])@np.array([-.4,.7])*mag/raw
        np.testing.assert_allclose(ref['movement_direction'][:2],expected,atol=1e-7)
        self.assertAlmostEqual(ref['speed'],min(.5,.1+.5*mag))

class KinematicsTest(unittest.TestCase):
    def test_host_torch_pinocchio_golden_fixture(self):
        from onboardmimic.backends.sonic.pico.pose_mapping import map_body
        data=json.loads((Path(__file__).parents[1]/'fixtures/pico_host_parity.json').read_text())
        self.assertEqual(len(data['rows']),8)
        for row in data['rows']:
            c=ThreePointCalibration();c.capture(row['body'],row['q'])
            for actual_pose,expected_pose in ((key_frames(row['q']),row['fk']),(c.apply(row['moved']),row['vr'])):
                expected_pose=np.asarray(expected_pose)
                np.testing.assert_allclose(actual_pose[:,:3],expected_pose[:,:3],atol=2e-6)
                a=R.from_quat(actual_pose[:,3:][:,[1,2,3,0]])
                b=R.from_quat(expected_pose[:,3:][:,[1,2,3,0]])
                self.assertLess(float(np.max((a.inv()*b).magnitude())),2e-6)
            actual=map_body(row['body'])
            for key in row['expected']:
                expected=np.asarray(row['expected'][key])
                if key=='body_quat_w' and np.dot(actual[key],expected)<0:expected=-expected
                np.testing.assert_allclose(actual[key],expected,atol=5e-5)
    def test_calibration_matches_measured_wrists(self):
        q=np.linspace(-.1,.1,29);c=ThreePointCalibration();c.capture(body(),q)
        np.testing.assert_allclose(c.apply(body())[:2,:3],key_frames(q)[:2,:3],atol=1e-6)
        np.testing.assert_allclose(c.apply(body())[2,:3],[0,0,.4],atol=1e-6)
    def test_root_translation_does_not_change_three_points(self):
        b=body();a=raw_three_points(b);b[:,:3]+=[1,2,3]
        np.testing.assert_allclose(raw_three_points(b),a,atol=1e-6)
    def test_bad_q_fails_closed(self):
        for q in ([0]*28,[float('nan')]*29):
            with self.assertRaises(ValueError):key_frames(q)

class ControllerMonitorTest(unittest.TestCase):
    def test_body_does_not_refresh_controller(self):
        m=Monitor(config());f=frame();f['left_controller']=dict(timestamp_ns=100,**controls());f['right_controller']=dict(timestamp_ns=100,**controls())
        m.ingest(f,0);f['body']['timestamp_ns']=200
        s=m.ingest(f,.3)
        self.assertTrue(s['body_capture_ready']);self.assertFalse(s['streams']['left_controller']['fresh'])
    def test_invalid_button_and_axis_rejected(self):
        for key,v in (('axes',[float('nan'),0]),('primary',1),('grip',2)):
            f=frame();f['left_controller']=dict(timestamp_ns=100,**controls());f['left_controller'][key]=v
            self.assertFalse(Monitor(config()).ingest(f,0)['streams']['left_controller']['fresh'])
