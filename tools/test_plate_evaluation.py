import copy
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

from plate_evaluation import (compare_groups, match_status, merge_observation,
                              source_key, summarize)
from evaluate_plate_recovery import evaluate_sample, load_captures
from diagnose_plate_capture import diagnose


def observation(sec=10, **changes):
    return dict(header={'frame_id':'camera', 'stamp':{'sec':sec,'nanosec':0}},
                image_width=100, image_height=30, input_valid=True, detected=True,
                usable=False, status='low_confidence', confidence=.3,
                x=5., y=5., width=90., height=20., **changes)


def sample():
    return {'image':'frame.png','capture_status':'image_saved', 'image_header':observation()['header'],
            'image_size_wh':[100,30], 'id':'sample', 'slot':0}


class EvaluationTests(unittest.TestCase):
    def test_independent_witness_distinguishes_evidence_not_assumed_cause(self):
        item=sample()
        capture={'samples':[item]}
        for records,expected in [([],'no_result_seen_by_either_subscriber'),
                ([{'kind':'debug','header':item['image_header']}],'debug_seen_but_result_not_seen'),
                ([{'kind':'observation','header':item['image_header']}],'result_seen_by_independent_witness')]:
            self.assertEqual(diagnose(capture,records)['missing'][0]['evidence'],expected)

    def test_match_requires_exact_source_not_latest(self):
        self.assertEqual(match_status(sample(),observation(11)), 'source_mismatch')
        wrong = observation(); wrong['header']['frame_id']='other'
        self.assertEqual(match_status(sample(),wrong), 'source_mismatch')
        self.assertEqual(match_status(sample(),None), 'missing_observation')

    def test_low_confidence_does_not_select_out_frames(self):
        self.assertEqual(match_status(sample(),observation()), 'ready_for_ocr')

    def test_resolution_box_and_timeout_rejected(self):
        for changes, expected in [({'image_width':101},'resolution_mismatch'),
                                  ({'width':float('nan')},'invalid_bbox'),
                                  ({'width':-2},'invalid_bbox'),
                                  ({'input_valid':False,'status':'input_timeout'},'input_invalid')]:
            value=observation(); value.update(changes)
            self.assertEqual(match_status(sample(),value),expected)

    def test_no_detection_and_missing_input_never_call_ocr(self):
        missing=sample(); missing.update(image=None,capture_status='missing_image')
        no_detection=sample(); obs=observation(); obs['detected']=False
        no_detection['observation']=obs
        with patch('evaluate_plate_recovery.recognize', side_effect=AssertionError('must not run')):
            for value, expected in [(missing,'missing_image'),(no_detection,'not_detected')]:
                result=evaluate_sample(value,None,Path('/tmp'),'123가4567')
                self.assertEqual(result['status'],expected)
                self.assertFalse(result['ocr_attempted'])
                self.assertFalse(result['end_to_end_exact'])

    def test_timeout_revokes_source_and_cannot_be_recovered_by_duplicate(self):
        good=observation(); bad=observation();bad.update(input_valid=False,status='input_timeout')
        revoked=merge_observation(good,bad)
        self.assertFalse(revoked['input_valid'])
        self.assertFalse(merge_observation(revoked,good)['input_valid'])
        self.assertEqual(merge_observation(good,good),good)

    def test_conflicting_duplicate_and_different_stamp(self):
        changed=observation(); changed['x']=20.
        self.assertEqual(merge_observation(observation(),changed)['status'],'conflicting_observation')
        with self.assertRaises(ValueError):
            merge_observation(observation(),observation(11))

    def test_denominator_includes_misses_and_streak_resets(self):
        rows=[dict(slot=i, image='a' if i!=2 else None, ocr_attempted=i!=2,
                   end_to_end_exact=i!=2,status='exact_match' if i!=2 else 'missing_image')
              for i in range(5)]
        stats=summarize(rows,temporal=True)
        self.assertEqual(stats['end_to_end_exact_fraction'],.8)
        self.assertEqual(stats['exact_fraction_given_ocr'],1.)
        self.assertEqual(stats['longest_exact_sample_streak'],2)
        rows[4]['barrier_since_previous']=True
        rows[1]['barrier_since_previous']=True
        self.assertEqual(summarize(rows,temporal=True)['longest_exact_sample_streak'],1)
        self.assertIsNone(summarize(rows,temporal=False)['longest_exact_sample_streak'])

    def test_slot_gap_is_not_consecutive(self):
        rows=[dict(slot=i, end_to_end_exact=True,status='exact_match') for i in (0,2)]
        self.assertEqual(summarize(rows,temporal=True)['longest_exact_sample_streak'],1)

    def test_phase_pairing_and_capture_failures(self):
        row=dict(slot=0,image='a',status='not_detected',end_to_end_exact=False)
        before=dict(trial_id='trial',phase='before',sampling={'hz':2},source_kind='raw_camera',
                    detector_parameters={'/detector': {'threshold': .5}},
                    clock_advanced=True,clock_first_ns=1,clock_last_ns=5,rows=[row],
                    summary=summarize([row],temporal=True))
        after=copy.deepcopy(before);after.update(phase='after',clock_first_ns=10,clock_last_ns=15)
        self.assertEqual(compare_groups([before,after])[0]['comparison_status'],'paired')
        for field,value,reason in [('sampling',{'hz':3},'sampling_mismatch'),
                                   ('capture_fault','clock_regression','capture_fault'),
                                   ('clock_first_ns',3,'phase_clock_order_invalid'),
                                   ('detector_parameters',{'threshold':.9},'detector_parameters_mismatch')]:
            changed=copy.deepcopy(after);changed[field]=value
            self.assertEqual(compare_groups([before,changed])[0]['comparison_status'],reason)
        self.assertEqual(compare_groups([before])[0]['comparison_status'],'missing_phase')
        with self.assertRaises(ValueError):
            compare_groups([before,before])

        for missing in ({}, {'/detector': {}}, None):
            changed=copy.deepcopy(after);changed['detector_parameters']=missing
            self.assertEqual(compare_groups([before,changed])[0]['comparison_status'],
                             'detector_parameters_unavailable')
            first=copy.deepcopy(before);first['detector_parameters']=missing
            result=compare_groups([first,changed])[0]
            self.assertEqual(result['comparison_status'],'detector_parameters_unavailable')
            self.assertIsNone(result['after_minus_before_exact_fraction'])

    def test_fixed_preprocessing_and_truth_used_only_after_ocr(self):
        import cv2
        import numpy as np
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)
            image=np.full((30,100,3),150,np.uint8)
            path=output/'source.png';cv2.imwrite(str(path),image)
            value=sample();value.update(image=str(path),observation=observation())
            seen=[]
            def engine(model,path):
                seen.append(cv2.imread(str(path)).copy())
                return dict(raw_text='123가4567',recognition_confidence=.7,recognition_ms=1.)
            with patch('evaluate_plate_recovery.recognize',side_effect=engine):
                first=evaluate_sample(value,None,output,'123가4567')
                second=evaluate_sample(value,None,output,'999나9999')
            self.assertEqual(seen[0].shape,(110,520,3))
            np.testing.assert_array_equal(seen[0],seen[1])
            self.assertEqual(first['raw_text'],second['raw_text'])
            self.assertTrue(first['end_to_end_exact'])
            self.assertFalse(second['end_to_end_exact'])

    def test_saved_source_twice_is_rejected(self):
        import json
        d=dict(schema='plate_evaluation_v1',source_kind='raw_camera',trial_id='test',phase='before',
               sampling={'count':2},samples=[sample(),sample()])
        d['samples'][1]['slot']=1
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'capture.json';path.write_text(json.dumps(d))
            with self.assertRaisesRegex(ValueError,'Repeated source'):
                load_captures([path])


if __name__ == '__main__':
    unittest.main()
