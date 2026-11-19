"""Scoring must not silently repair incorrect registration numbers."""
import unittest
from probe_plate_ocr import crop_bounds, normalize_text, score_text


class OCRScoringTest(unittest.TestCase):
    def test_whitespace_and_unicode_only(self):
        self.assertEqual(normalize_text(' 123\u1100\u11614567\n'), '123가4567')
        self.assertTrue(score_text('123가 4567', '123가4567')['exact_match'])

    def test_wrong_plate_is_not_corrected(self):
        for text in ['123가4561', '1237 4567', '123가4567.', 'I23가4567']:
            result = score_text(text, '123가4567')
            self.assertFalse(result['exact_match'])
            self.assertGreater(result['edit_distance'], 0)
        self.assertTrue(score_text('123나4567', '123가4567')['simple_plate_format'])
        self.assertFalse(score_text('123나4567', '123가4567')['exact_match'])

    def test_no_text_is_not_success(self):
        result = score_text('', '123가4567')
        self.assertFalse(result['exact_match'])
        self.assertEqual(result['character_error_rate'], 1.0)

    def test_crop_rounding_clipping(self):
        self.assertEqual(crop_bounds(dict(x=-.5,y=2.1,width=10,height=6.4),20,20),
                         ([0,2,10,9],True))

    def test_bad_crop_rejected(self):
        for box in [dict(x=float('nan'),y=0,width=2,height=2),
                    dict(x=25,y=0,width=2,height=2),dict(x=1,y=0,width=0,height=2)]:
            with self.assertRaises(ValueError): crop_bounds(box,20,20)


if __name__ == '__main__':
    unittest.main()
