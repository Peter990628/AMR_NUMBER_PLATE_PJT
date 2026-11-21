"""Regression tests for identity and content checks at the collector boundary."""
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from amr_reobservation.srv import InspectPlateFrame
from inspect_plate_window import inspect


class NumberedFrameTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.output = Path(directory.name)
        self.path = self.output/'original.png'
        self.image = np.arange(20*10*3, dtype=np.uint8).reshape(10, 20, 3)
        cv2.imwrite(str(self.path), self.image)
        self.row = dict(id='sample', sample_id='run_before_017', image=str(self.path),
                        image_sha256=hashlib.sha256(self.path.read_bytes()).hexdigest(),
                        capture_status='image_saved', image_size_wh=[20, 10],
                        image_header=dict(frame_id='camera', stamp=dict(sec=10, nanosec=0)))

    def response(self):
        reply = InspectPlateFrame.Response()
        reply.sample_id = self.row['sample_id']
        o = reply.observation
        o.header.frame_id = 'camera'
        o.header.stamp.sec = 10
        o.input_valid = True
        o.image_width, o.image_height = 20, 10
        o.x, o.y, o.width, o.height, o.confidence = .5, 1.2, 4.7, 2.5, .35
        o.detected = True
        reply.status, reply.text, reply.ocr_confidence = 'recognized', '123가4567', .99
        reply.crop_pixels_sha256 = hashlib.sha256(self.image[1:4, :6].tobytes()).hexdigest()
        return reply

    def run_reply(self, reply):
        with patch('inspect_plate_window.call', return_value=reply):
            inspect(None, None, self.row, self.output)

    def test_low_detector_confidence_with_exact_crop(self):
        self.run_reply(self.response())
        self.assertEqual(self.row['crop_bounds_xyxy'], [0, 1, 6, 4])
        self.assertEqual(self.row['raw_text'], '123가4567')

    def test_wrong_id_stamp_and_crop(self):
        for change, message in [('id', 'source_mismatch'), ('stamp', 'source_mismatch'),
                                ('pixels', 'pixels_mismatch')]:
            with self.subTest(change=change):
                reply = self.response()
                if change == 'id':
                    reply.sample_id = 'previous_run_before_017'
                elif change == 'stamp':
                    reply.observation.header.stamp.nanosec = 1
                else:
                    reply.crop_pixels_sha256 = 'wrong'
                with self.assertRaisesRegex(ValueError, message):
                    self.run_reply(reply)

    def test_no_box_versus_failed_ocr(self):
        reply = self.response()
        reply.observation.detected = False
        reply.status = 'not_detected'
        self.run_reply(reply)
        self.assertEqual(self.row['status'], 'not_detected')
        reply = self.response()
        reply.status = 'ocr_timeout'
        with self.assertRaisesRegex(ValueError, 'ocr_timeout'):
            self.run_reply(reply)
