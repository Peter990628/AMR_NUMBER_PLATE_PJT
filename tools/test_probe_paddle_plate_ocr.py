#!/usr/bin/env python3
"""Check that the local probe preserves inputs and keeps truth out of inference."""
from pathlib import Path
import unittest

from probe_paddle_plate_ocr import recognize, select_inputs
from probe_plate_ocr import score_text


class ProbePaddleTests(unittest.TestCase):
    def test_fixed_cohort_excludes_skipped_before_images(self):
        rows=[{'id':str(i), 'kind':'recorded_camera', 'phase':'after', 'ocr_attempted':True} for i in range(5)]
        rows += [{'id':'texture', 'kind':'texture_control', 'phase':'control', 'ocr_attempted':True},
                 {'id':'before', 'kind':'recorded_camera', 'phase':'before', 'ocr_attempted':False}]
        selected=select_inputs({'rows':rows})
        self.assertEqual(len(selected),6)
        self.assertEqual(selected[0]['id'],'texture')
        with self.assertRaises(ValueError):
            select_inputs({'rows':rows[:-2]})

    def test_recognition_gets_only_image_and_preserves_wrong_character(self):
        calls=[]
        class Model:
            def predict(self, **kwargs):
                calls.append(kwargs)
                return iter([{'rec_text':'12374567', 'rec_score':0.99}])
        result=recognize(Model(),Path('plate.png'))
        self.assertEqual(calls,[{'input':'plate.png','batch_size':1}])
        self.assertEqual(result['raw_text'],'12374567')
        self.assertFalse(score_text(result['raw_text'],'123가4567')['exact_match'])


if __name__=='__main__':
    unittest.main()
