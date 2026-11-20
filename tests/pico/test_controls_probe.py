"""Read-only hardware probe must not count stale or initially held inputs."""
import copy
import unittest
from scripts.check_pico_controls import Observation,inputs
from tests.sonic.test_pico_session import controls


def sample(stamp=1):
    return dict(data_source='sdk',receiver_running=True,receiver_id='one',updated_at_unix=100.,
        streams={k:dict(fresh=True,age_ms=0,source_timestamp_ns=stamp,payload=controls())
                 for k in ('body','left_controller','right_controller')})


class ControlsProbeTest(unittest.TestCase):
    def test_held_input_requires_neutral_then_rising_and_release(self):
        p=Observation();s=sample();c=s['streams']['right_controller']['payload']
        c['primary']=True;p.sample(s,100.,0);self.assertEqual(p.observed['a'],0)
        c['primary']=False;p.sample(s,100.,.01)
        c['primary']=True;p.sample(s,100.,.02);p.sample(s,100.,.03)
        self.assertEqual(p.observed['a'],1)
        self.assertIn('a',p.report(1)['missing'])
        c['primary']=False;p.sample(s,100.,.04)
        self.assertNotIn('a',p.report(1)['missing'])
    def test_body_updates_do_not_refresh_frozen_controller(self):
        p=Observation();s=sample();p.sample(s,100.,0)
        s['streams']['body']['source_timestamp_ns']=2
        p.sample(s,100.,.3)
        self.assertEqual(p.stream_losses['left_controller'],1)
        self.assertIsNone(p.previous)
        self.assertFalse(p.report(1)['input_coverage_complete'])
    def test_reconnect_reestablishes_neutral(self):
        p=Observation();s=sample();p.sample(s,100.,0)
        s['streams']['right_controller']['payload']['primary']=True
        s['receiver_id']='two';p.sample(s,100.,.01)
        self.assertEqual(p.observed['a'],0);self.assertEqual(p.generations,2)
    def test_four_button_chord_retains_raw_and_subset_observations(self):
        l=controls();r=controls()
        l.update(primary=True,secondary=True);r.update(primary=True,secondary=True)
        self.assertTrue({'a','b','x','y','ax','by','ab','xy','abxy'}<=inputs(l,r))
    def test_axis_signs_and_return_to_neutral(self):
        l=controls();r=controls();l['axes']=[-1,1];r['axes']=[1,-1]
        self.assertEqual(inputs(l,r),{'left_axis_left','left_axis_up','right_axis_right','right_axis_down'})
    def test_bad_samples_fail_closed(self):
        for mutate in (lambda s:s.update(updated_at_unix=float('nan')),
                       lambda s:s['streams']['left_controller'].update(age_ms=True),
                       lambda s:s['streams']['right_controller']['payload'].update(primary=1)):
            p=Observation();s=sample();mutate(s);p.sample(s,100.,0)
            self.assertEqual(p.stale_samples,1);self.assertEqual(p.fresh_samples,0)
