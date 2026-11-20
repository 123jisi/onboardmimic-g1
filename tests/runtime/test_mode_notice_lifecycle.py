"""Controller-session switch notices expire; real failures remain visible."""
import unittest
from tests.runtime import test_native_lifecycle as fixture

class ModeNoticeLifecycleTest(unittest.TestCase):
 setUp=fixture.NativeTest.setUp
 refresh=fixture.NativeTest.refresh
 packet=fixture.NativeTest.packet
 command=fixture.NativeTest.command
 activate=fixture.NativeTest.activate
 def test_estop_retires_pico_and_file_notices_without_erasing_history(self):
  for reason in ['PICO 身体或手柄断流，已请求回退','文件播放结束，已请求零速 Planner']:
   with self.subTest(reason=reason):
    self.setUp();self.activate();self.g.mode_error=reason;self.g.mode_requested_at=self.now
    self.g.motion_task=dict(id='history',kind='pico',stage='error',error='historical event')
    self.assertEqual(self.g.status()['mode_switch_error'],reason)
    result=self.command('estop')
    self.assertEqual(result['state'],'stopped');self.assertEqual(result['pids'],[])
    self.assertEqual(result['mode_switch_error'],'');self.assertEqual(self.g.mode_requested_at,0.)
    self.assertEqual(result['motion_task']['error'],'historical event')
    self.assertEqual(self.proc.kills,1)
 def test_prepare_starts_fresh_notice_scope_even_for_legacy_leftovers(self):
  self.g.mode_error='PICO 身体或手柄断流，已请求回退'
  result=self.command('prepare_pose')
  self.assertEqual(result['state'],'loading');self.assertEqual(result['mode_switch_error'],'')
  self.logs='Init Done';self.g.tick();self.assertEqual(self.g.status()['state'],'prepared')
  self.assertEqual(self.g.status()['mode_switch_error'],'')
 def test_live_notice_and_real_controller_fault_are_not_hidden(self):
  self.activate();self.g.mode_error='正在返回 Planner'
  self.assertEqual(self.g.status()['mode_switch_error'],'正在返回 Planner')
  self.g.halt('真实状态反馈故障')
  result=self.g.status();self.assertEqual(result['state'],'error')
  self.assertEqual(result['workflow_error'],'真实状态反馈故障')
  self.assertEqual(result['mode_switch_error'],'')
