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

    def test_first_three_latch_even_if_seventh_image_is_missing(self):
        values=rows(8)
        values[6].update(status='missing_image', image_header=None)
        decision=decide(capture(8),values)
        self.assertTrue(decision['success'])
        self.assertEqual(decision['supporting_slots'],[0,1,2])
        self.assertEqual(decision['rows_considered'],3)

    def test_first_confirmed_text_wins(self):
        values=rows(6)
        for row in values[3:]:
            row['raw_text']='999나9999'
        self.assertEqual(decide(capture(6),values)['text'],'123가4567')

    def test_missing_before_confirmation_restarts_streak(self):
        values=rows(6)
        values[2].update(status='missing_image', image_header=None)
        self.assertEqual(decide(capture(6),values)['supporting_slots'],[3,4,5])

    def test_ocr_confidence_and_format_are_independent_gates(self):
        for key,value in [('recognition_confidence',.79),('recognition_confidence',float('nan')),
                          ('raw_text','123A4567'),('raw_text','123가456')]:
            values=rows(3); values[-1][key]=value
            self.assertFalse(decide(capture(3),values)['success'])

    def test_missing_barrier_and_box_switch_break_streak(self):
        for change in ('missing','barrier','box'):
            values=rows()
            if change=='missing': values[2]['status']='missing_pair'
            if change=='barrier': values[2]['barrier_since_previous']=True; values=values[:4]
            if change=='box': values[2]['observation']['x']=500.
            self.assertFalse(decide(capture(len(values)),values)['success'])

    def test_duplicate_reordered_and_stream_changed_abort(self):
        for index,sec in [(2,10),(3,9)]:
            values=rows(); values[index]['image_header']['stamp']={'sec':sec,'nanosec':0}
            self.assertEqual(decide(capture(),values)['reason'],'duplicate_or_reordered_source')
        values=rows(); values[-1]['image_header']['frame_id']='other'
        self.assertEqual(decide(capture(),values)['reason'],'stream_changed')

    def test_long_source_gap_resets_and_short_window_fails(self):
        values=rows(4)
        for r in values[-2:]: r['image_header']['stamp']['sec']+=10
        self.assertFalse(decide(capture(4),values)['success'])
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
    def test_normal_candidate_never_triggers_corner_followup(self):
        miss = dict(success=False, text=None, reason='not_detected')
        phases = []
        def observe(phase):
            phases.append(phase)
            return miss
        result = run_once(observe, lambda: None, lambda: {'label':'end_face'},
                          lambda candidate: None, lambda state: None,
                          select_after_probe=lambda: self.fail('unexpected second attempt'))
        self.assertEqual(phases, ['before', 'after'])
        self.assertEqual(result['candidate_attempts'], 1)

    def test_corner_followup_is_bounded_and_only_runs_after_a_clean_miss(self):
        bad = dict(success=False, text=None, reason='not_detected')
        good = dict(success=True, text='123가4567', reason='consistent_ocr')
        fault = dict(success=False, text=None, reason='capture_invalid')
        for corner, second, final, status, expected in (
                (good, None, None, 'succeeded_after_motion', ['corner_probe','corner']),
                (bad, None, None, 'no_candidate_after_probe', ['corner_probe','corner','refresh']),
                (bad, 'end', good, 'succeeded_after_motion', ['corner_probe','corner','refresh','end','after']),
                (bad, 'end', bad, 'candidate_exhausted', ['corner_probe','corner','refresh','end','after']),
                (fault, 'end', None, 'candidate_ocr_capture_invalid', ['corner_probe','corner'])):
            with self.subTest(status=status):
                calls = []
                def observe(phase):
                    calls.append(phase)
                    return {'before': bad, 'corner': corner, 'after': final}[phase]
                def refresh():
                    calls.append('refresh')
                    return {'label': second} if second else None
                def run():
                    return run_once(observe, lambda: None, lambda: {'label':'corner_probe'},
                                    lambda c: calls.append(c['label']), lambda s: None,
                                    select_after_probe=refresh)
                if corner == fault:
                    with self.assertRaisesRegex(RuntimeError, status): run()
                else:
                    result = run()
                    self.assertEqual(result['status'], status)
                    self.assertEqual(result['candidate_attempts'], 1 + ('end' in expected))
                self.assertEqual(calls, ['before', *expected])

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
