import tempfile
from pathlib import Path
import sys
import time
import unittest

from recovery_ocr_policy import decide, run_once
from recovery_ocr_session import child_job


def rows(count=5):
    return [dict(slot=i, status='recognized', image_header={'frame_id':'camera',
                 'stamp':{'sec':10+i//2,'nanosec':500000000*(i%2)}},
                 image_size_wh=[1920,1200], raw_text='123가4567', recognition_confidence=.95,
                 observation={'x':100.,'y':100.,'width':150.,'height':50.,
                              'confidence':.3,'status':'low_confidence','usable':False}) for i in range(count)]


def capture(count=5):
    return dict(capture_fault=None, clock_advanced=True, sampling={'count':count})


class PolicyTests(unittest.TestCase):
    def test_low_detector_score_can_succeed_without_known_text(self):
        values=rows()
        for r in values: r['raw_text']=' 98나7654 '
        decision=decide(capture(),values)
        self.assertTrue(decision['success'])
        self.assertEqual(decision['text'],'98나7654')

    def test_last_observations_must_still_agree(self):
        for status in ('not_detected','missing_pair','ocr_error','input_invalid'):
            values=rows(); values[-1]['status']=status
            self.assertFalse(decide(capture(),values)['success'])
        values=rows(); values[-1]['raw_text']='999나9999'
        self.assertEqual(decide(capture(),values)['trailing_count'],1)

    def test_ocr_confidence_and_format_are_independent_gates(self):
        for key,value in [('recognition_confidence',.79),('recognition_confidence',float('nan')),
                          ('raw_text','123A4567'),('raw_text','123가456')]:
            values=rows(); values[-1][key]=value
            self.assertFalse(decide(capture(),values)['success'])

    def test_missing_barrier_and_box_switch_break_streak(self):
        for change in ('missing','barrier','box'):
            values=rows()
            if change=='missing': values[2]['status']='missing_pair'
            if change=='barrier': values[3]['barrier_since_previous']=True
            if change=='box': values[3]['observation']['x']=500.
            self.assertFalse(decide(capture(),values)['success'])

    def test_duplicate_reordered_and_stream_changed_abort(self):
        for index,sec in [(2,10),(3,9)]:
            values=rows(); values[index]['image_header']['stamp']={'sec':sec,'nanosec':0}
            self.assertEqual(decide(capture(),values)['reason'],'duplicate_or_reordered_source')
        values=rows(); values[-1]['image_header']['frame_id']='other'
        self.assertEqual(decide(capture(),values)['reason'],'stream_changed')

    def test_long_source_gap_resets_and_short_window_fails(self):
        values=rows()
        for r in values[-2:]: r['image_header']['stamp']['sec']+=10
        self.assertFalse(decide(capture(),values)['success'])
        self.assertFalse(decide(capture(2),rows(2))['success'])

    def test_fault_or_incomplete_slots_cannot_pass(self):
        c=capture(); c['capture_fault']='clock_rollback'
        self.assertEqual(decide(c,rows())['reason'],'capture_invalid')
        self.assertEqual(decide(capture(),rows(4))['reason'],'incomplete_slot_sequence')
        c=capture();c['clock_advanced']=False
        self.assertFalse(decide(c,rows())['success'])

    def test_no_detection_is_not_input_failure(self):
        values=rows()
        for r in values:r['status']='not_detected'
        self.assertEqual(decide(capture(),values)['reason'],'not_detected')
        for r in values:r['status']='missing_pair'
        self.assertEqual(decide(capture(),values)['reason'],'no_valid_observations')


class FlowTests(unittest.TestCase):
    def run_flow(self, before, after=None, failure=None, execute=True, candidate='candidate'):
        self.events=[]
        def action(name,value=None):
            self.events.append(name)
            if failure==name:raise RuntimeError(name)
            return value
        return run_once(lambda phase:action(phase,before if phase=='before' else after),
                        lambda:action('prepare'),lambda:action('select',candidate),
                        lambda c:action('move'),lambda s:None,execute=execute)

    def test_initial_success_skips_direction_and_motion(self):
        good=dict(success=True,text='12가3456',reason='consistent_ocr')
        r=self.run_flow(good)
        self.assertTrue(r['success']);self.assertEqual(r['candidate_attempts'],0)
        self.assertEqual(self.events,['before'])

    def test_exactly_one_candidate_success_or_failure_no_return(self):
        bad=dict(success=False,text=None,reason='not_detected')
        for after in (bad,dict(success=True,text='123가4567',reason='consistent_ocr')):
            r=self.run_flow(bad,after)
            self.assertEqual(r['success'],after['success'])
            self.assertEqual(r['candidate_attempts'],1)
            self.assertEqual(self.events,['before','prepare','select','move','after'])

    def test_faults_never_proceed_or_retry(self):
        bad=dict(success=False,text=None,reason='not_detected')
        for fault in ('before','prepare','select','move','after'):
            with self.assertRaisesRegex(RuntimeError,fault):self.run_flow(bad,bad,failure=fault)
            self.assertEqual(self.events[-1],fault)
            self.assertLessEqual(self.events.count('move'),1)
        for reason in ('capture_invalid','no_valid_observations'):
            with self.assertRaisesRegex(RuntimeError,reason):
                self.run_flow(dict(success=False,text=None,reason=reason))
            self.assertEqual(self.events,['before'])
            with self.assertRaisesRegex(RuntimeError,reason):
                self.run_flow(bad,dict(success=False,text=None,reason=reason))

    def test_no_candidate_and_dry_selection_never_move(self):
        bad=dict(success=False,text=None,reason='not_detected')
        for args in ({'execute':False},{'candidate':None}):
            r=self.run_flow(bad,**args)
            self.assertFalse(r['success']);self.assertNotIn('move',self.events)


class ChildTests(unittest.TestCase):
    def test_timeout_and_health_failure_stop_child(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)
            script=path/'sleeper.py'
            script.write_text('import time\ntime.sleep(20)\n')
            with self.assertRaisesRegex(RuntimeError,'child_timeout'):
                child_job([sys.executable,script],path/'timeout.log',.1,lambda:time.sleep(.01))
            def unhealthy():raise RuntimeError('clock_stalled')
            with self.assertRaisesRegex(RuntimeError,'clock_stalled'):
                child_job([sys.executable,script],path/'health.log',2,unhealthy)


if __name__=='__main__':unittest.main()
