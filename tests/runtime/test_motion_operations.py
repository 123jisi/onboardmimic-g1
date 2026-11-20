"""Handoff races and browser authority, using isolated file loaders and fake motors."""
import threading
import time
from unittest.mock import patch

from tests.runtime import test_local_playback as local
write_clip = local.write_clip


class MotionOperationsTest(local.LocalPlaybackTest):
    def feedback(self, mode=None, **kw):
        mode = mode or self.g.input_mode
        return super().feedback('pose' if mode == 'smpl_playback' else mode, **kw)

    def acquire(self, actor='operator-aaaaaaaa'):
        self.g.request('/api/operator', self.packet(action='acquire', actor=actor))
        return actor

    def browser(self, action, actor='operator-aaaaaaaa', **kw):
        return self.g.request('/api/playback',self.packet(action=action,actor=actor,
            operator_generation=self.g.operator_generation,expected_revision=self.g.revision,**kw))

    def ready(self):
        for _ in range(250):
            if self.g.motion_task['stage'] != 'checking': return
            time.sleep(.01)
        self.fail('worker did not finish')

    def target(self):
        write_clip(self.root/'next')
        # Browser paths must be cataloged and declared; never silently infer CSV order.
        from onboardmimic.backends.sonic.motion_worker import execute
        execute(dict(action='register',root=str(self.root),motion='next',fps=50,
                     joint_order='sonic_reference',root_body_index=0),self.root)
        self.g.library.entries=[dict(id='cache/next',name='next',motion='next',kind='cache',status='ready')]

    def test_switch_does_not_restart_controller_and_preserves_clock_until_ack(self):
        self.target();self.begin();self.step(8);self.acquire()
        pid=self.g.process.pid
        self.browser('switch',motion_id='cache/next',request_id='once')
        self.assertEqual(self.g.input_mode,'joint_playback')
        self.ready();self.step()
        self.assertEqual(self.g.input_mode,'planner')
        self.assertEqual(self.g.motion_task['stage'],'waiting_planner')
        self.step(45)
        self.assertEqual(self.g.motion_task['stage'],'finished')
        self.assertEqual(self.g.input_mode,'joint_playback')
        self.assertEqual(self.g.playback.clip.path.name,'next')
        self.assertEqual(self.g.process.pid,pid)
        self.assertEqual(self.launches,1)

    def test_failed_target_keeps_old_reference(self):
        self.begin();self.step(8);self.acquire()
        self.g.library.entries=[dict(id='cache/bad',motion='missing',kind='cache',status='ready')]
        old=self.g.playback.clip
        self.browser('switch',motion_id='cache/bad')
        self.ready()
        self.assertEqual(self.g.motion_task['stage'],'error')
        self.assertIs(self.g.playback.clip,old)
        self.assertEqual(self.g.input_mode,'joint_playback')

    def test_stop_invalidates_late_loader_result(self):
        self.target();self.begin();self.step(8);self.acquire()
        entered,release=threading.Event(),threading.Event()
        clip=self.g.playback.clip
        def slow(*args):entered.set();release.wait(2);return clip
        with patch.object(self.g.backend,'prepare_clip',side_effect=slow):
            self.browser('switch',motion_id='cache/next')
            self.assertTrue(entered.wait(1))
            self.browser('stop')
            release.set();time.sleep(.03)
        self.assertEqual(self.g.motion_task['stage'],'cancelled')
        self.assertIsNone(self.g.prepared_clip)
        self.assertIs(self.g.playback.clip,clip)
        self.assertEqual(self.g.input_mode,'planner')

    def test_ownership_blocks_cli_and_other_browser_but_never_stop(self):
        self.acquire()
        with self.assertRaisesRegex(ValueError,'操作权'):self.call('play')
        with self.assertRaises(ValueError):self.browser('load_async',actor='operator-bbbbbbbb',motion='short')
        self.g.request('/api/playback',self.packet(action='stop'))
        self.g.request('/api/controller',{'action':'estop'})

    def test_stale_revision_and_generation_are_rejected(self):
        self.acquire();old=self.g.operator_generation
        self.g.request('/api/operator',self.packet(action='takeover',actor='operator-bbbbbbbb'))
        with self.assertRaises(ValueError):self.g.request('/api/playback',self.packet(action='play',
            actor='operator-aaaaaaaa',operator_generation=old,expected_revision=self.g.revision))
        with self.assertRaises(ValueError):self.g.request('/api/playback',self.packet(action='play',
            actor='operator-bbbbbbbb',operator_generation=self.g.operator_generation,expected_revision=-1))

    def test_operator_loss_before_ready_cancels_but_playing_is_independent(self):
        self.target();self.begin();self.step(8);self.acquire()
        blocker=threading.Event()
        def slow(*args):blocker.wait(1);return self.g.playback.clip
        with patch.object(self.g.backend,'prepare_clip',side_effect=slow):
            self.browser('switch',motion_id='cache/next')
            # Keep transport alive while browser lease expires. A is lengthened.
            self.g.playback.clip.duration=20
            self.step(255)
            blocker.set()
        self.assertIsNone(self.g.operator)
        self.assertEqual(self.g.motion_task['stage'],'cancelled')
        self.assertEqual(self.g.input_mode,'joint_playback')

    def test_active_load_cli_is_async_and_remains_planner(self):
        self.target();self.activate();self.step(40)
        self.call('load',motion='next',fps=50,joint_order='sonic_reference')
        self.ready();self.step(40)
        self.assertEqual(self.g.motion_task['stage'],'finished')
        self.assertEqual(self.g.playback.clip.path.name,'next')
        self.assertEqual(self.g.input_mode,'planner')

    def test_conversion_running_blocks_prepare_even_on_legacy_cli(self):
        self.g.conversion=dict(stage='running')
        with self.assertRaisesRegex(ValueError,'转换'):self.command('prepare_pose')
        self.assertEqual(self.launches,0)

    def test_browser_disconnect_after_start_does_not_end_motion(self):
        self.target();self.activate();self.step(40);self.acquire()
        self.browser('switch',motion_id='cache/next',entry_checks='off')
        self.ready();self.step(40)
        self.assertEqual(self.g.motion_task['stage'],'finished')
        self.g.playback.clip.duration=20
        self.step(255)
        self.assertIsNone(self.g.operator)
        self.assertEqual(self.g.input_mode,'joint_playback')
        self.assertEqual(self.g.playback_entry_checks,'off')

    def test_duplicate_request_after_completion_does_not_replay(self):
        self.target();self.activate();self.step(40);self.acquire()
        self.browser('switch',motion_id='cache/next',request_id='deduplicate')
        self.ready();self.step(40)
        task=self.g.motion_task
        self.assertEqual(task['stage'],'finished')
        self.browser('switch',motion_id='cache/next',request_id='deduplicate')
        self.assertIs(self.g.motion_task,task)

    def test_static_entry_failure_does_not_interrupt_old_clip(self):
        import numpy as np
        self.target();self.begin();self.step(8);self.acquire()
        clip=self.g.backend.load_clip(self.root/'next',fps=50,joint_order='sonic_reference')
        dq=clip.dq.copy();dq[0]=2.;clip.dq=dq
        with patch.object(self.g.backend,'prepare_clip',return_value=clip):
            self.browser('switch',motion_id='cache/next')
            self.ready()
        self.assertEqual(self.g.motion_task['stage'],'error')
        self.assertEqual(self.g.input_mode,'joint_playback')
        self.assertEqual(self.g.playback.clip.path.name,'short')

    def test_dynamic_entry_failure_remains_planner_and_no_auto_retry(self):
        self.target();self.begin();self.step(8);self.acquire()
        self.browser('switch',motion_id='cache/next')
        self.ready()
        with patch.object(self.g.backend,'validate_first_frame',side_effect=ValueError('tilt rejected')):
            self.step(45)
        self.assertEqual(self.g.motion_task['stage'],'error')
        self.assertEqual(self.g.input_mode,'planner')
        self.assertIn('tilt',self.g.motion_task['error'])
        self.step(10)
        self.assertEqual(self.g.input_mode,'planner')

    def test_cancel_after_play_issued_requests_planner(self):
        self.target();self.activate();self.step(40);self.acquire()
        self.browser('switch',motion_id='cache/next')
        self.ready()
        for _ in range(40):
            self.step()
            if self.g.motion_task['stage']=='starting':break
        self.assertEqual(self.g.motion_task['stage'],'starting')
        self.browser('cancel')
        self.assertEqual(self.g.motion_task['stage'],'cancelled')
        self.assertEqual(self.g.input_mode,'planner')

    def test_joint_smpl_smpl_joint_handoffs_keep_same_controller(self):
        from tests.sonic.test_motion_import import write_smpl
        from tests.runtime.test_local_playback import unpack
        self.target();write_smpl(self.root/'human.npz');write_smpl(self.root/'other.npz')
        for name in ('human.npz','other.npz'):
            self.g.library.entries.append(dict(id='cache/'+name,name=name,motion=name,kind='cache',status='ready'))
        self.begin();self.step(8);self.acquire()
        pid=self.g.process.pid
        for name,version in [('human.npz',3),('other.npz',3),('next',1)]:
            self.browser('switch',motion_id='cache/'+name)
            self.ready()
            for _ in range(60):
                packets=self.step()
                if self.g.motion_task['stage']=='finished':break
            self.assertEqual(self.g.motion_task['stage'],'finished',self.g.motion_task)
            self.assertEqual(unpack(packets[-1])[0]['v'],version)
            self.assertEqual(self.g.process.pid,pid)
        self.assertEqual(self.launches,1)
