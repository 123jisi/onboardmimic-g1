"""SMPL file source uses the same admission/feedback/mux as joint files."""
from tests.runtime import test_local_playback as local
from tests.runtime.test_local_playback import unpack
from tests.sonic.test_motion_import import write_smpl
import unittest
import numpy as np


class SMPLPlaybackTest(unittest.TestCase):
    setUp = local.LocalPlaybackTest.setUp
    call = local.LocalPlaybackTest.call
    activate = local.LocalPlaybackTest.activate
    refresh = local.LocalPlaybackTest.refresh
    packet = local.LocalPlaybackTest.packet
    command = local.LocalPlaybackTest.command
    step = local.LocalPlaybackTest.step

    def feedback(self, mode=None, **kw):
        mode = mode or self.g.input_mode
        local.LocalPlaybackTest.feedback(self,'pose' if mode=='smpl_playback' else mode,**kw)

    def begin(self):
        write_smpl(self.root/'human.npz')
        self.call('load',motion='human.npz')
        local.LocalPlaybackTest.begin(self)

    def test_smpl_confirm_pause_finish_and_replay_joint(self):
        self.begin()
        self.assertEqual(self.g.input_mode,'smpl_playback')
        self.assertEqual(self.g.status()['input_source'],'file_smpl')
        self.step(8)
        self.assertFalse(self.g.mode_pending)
        packets=self.step()
        self.assertEqual(unpack(packets[-1])[0]['v'],3)
        self.call('pause');position=self.g.playback.position
        self.step(50)
        self.assertEqual(self.g.playback.position,position)
        self.call('resume');self.step(50)
        self.assertEqual(self.g.playback.state,'finished')
        self.assertEqual(self.g.input_mode,'planner')
        self.step(8)
        self.assertFalse(self.g.mode_pending)

    def test_wrong_joint_ack_cannot_admit_smpl(self):
        self.begin()
        for _ in range(8):
            self.feedback('joint_playback');self.step(ack=False)
        self.assertTrue(self.g.mode_pending)
        self.assertEqual(self.g.playback.position,0)
        self.call('stop');self.step(8)
        self.assertEqual(self.g.input_mode,'planner')

    def test_smpl_entry_checks_off_allows_tilt_and_keeps_smpl_ack(self):
        write_smpl(self.root/'human.npz')
        self.call('load',motion='human.npz')
        clip=self.g.playback.clip
        clip.quat.flags.writeable=True
        clip.quat[0]=[np.cos(.2),np.sin(.2),0,0]
        self.activate();self.now+=.6;self.refresh();self.g.tick();self.feedback('planner')
        with self.assertRaisesRegex(ValueError,'倾斜'):self.call('play')
        result=self.call('play',entry_checks='off')
        self.assertEqual(result['playback_entry_checks'],'off')
        self.g.playback_packet()
        self.step(8)
        self.assertEqual(self.g.input_mode,'smpl_playback')
        self.assertFalse(self.g.mode_pending)

    def test_imported_kimodo_cache_uses_existing_mux_and_pause_stop(self):
        from onboardmimic.backends.sonic.kimodo_import import import_kimodo
        from onboardmimic.backends.sonic.pico.pose_mapping import UP, BASE
        orient=(UP.inv()*BASE).as_rotvec()
        np.savez(self.root/'amass.npz',pose_body=np.zeros((61,63)),
                 root_orient=np.tile(orient,(61,1)),mocap_frame_rate=30.)
        import_kimodo(self.root/'amass.npz',self.root/'kimodo.npz')
        self.call('load',motion='kimodo.npz')
        local.LocalPlaybackTest.begin(self)
        self.step(8)
        self.assertFalse(self.g.mode_pending)
        self.assertEqual(self.g.status()['input_source'],'file_smpl')
        header,fields=unpack(self.step()[-1])
        self.assertEqual(header['v'],3)
        self.assertEqual(fields['smpl_joints'].shape,(64,24,3))
        self.call('pause');pos=self.g.playback.position;self.step(20)
        self.assertEqual(self.g.playback.position,pos)
        self.call('resume');self.step(5)
        self.assertGreater(self.g.playback.position,pos)
        self.call('stop');self.step(8)
        self.assertEqual(self.g.input_mode,'planner')
        self.assertFalse(self.g.mode_pending)
