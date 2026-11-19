#!/usr/bin/env python3
"""Offline contract tests; never call an external API."""
import base64
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from probe_google_plate_ocr import (legacy_bounds, legacy_normalize, load_key,
                                    make_request, parse_response, redact, run, summarize)
from probe_plate_ocr import score_text, sha256


class GoogleProbeTests(unittest.TestCase):
    def test_original_padding_uses_truncation_and_clips(self):
        self.assertEqual(legacy_bounds({'x': 20.9, 'y': 40.9, 'width': 50.9, 'height': 30.9}, 100, 80),
                         [0, 10, 100, 80])

    def test_cloud_request_contains_only_pixels_and_configuration(self):
        payload = make_request(b'image bytes')
        req = payload['requests'][0]
        self.assertEqual(set(req), {'image', 'features', 'imageContext'})
        self.assertEqual(base64.b64decode(req['image']['content']), b'image bytes')
        self.assertEqual(req['features'], [{'type': 'DOCUMENT_TEXT_DETECTION'}])
        self.assertEqual(req['imageContext'], {'languageHints': ['ko']})

    def test_parse_text_and_mean_symbol_confidence(self):
        word = {'symbols': [{'confidence': 0.8}, {'confidence': 1.0}, {}]}
        payload = {'responses': [{'textAnnotations': [{'description': '123가4567\n'}],
                   'fullTextAnnotation': {'pages': [{'blocks': [{'paragraphs': [{'words': [word]}]}]}]}}]}
        result = parse_response(payload)
        self.assertEqual(result['raw_text'], '123가4567\n')
        self.assertAlmostEqual(result['mean_symbol_confidence'], 0.6)
        self.assertEqual(result['symbol_count'], 3)

    def test_api_failure_is_not_a_scored_ocr_mismatch(self):
        failed = parse_response({'responses': [{'error': {'code': 7, 'message': 'denied'}}]})
        self.assertEqual(failed['status'], 'api_error')
        self.assertNotIn('raw_text', failed)
        summary = summarize([dict(failed, condition='same_crop', kind='recorded_camera',
                                  request_sent=True, exact_match=None)])
        self.assertEqual(summary['same_crop/recorded_camera']['scored'], 0)
        self.assertEqual(parse_response({'responses': [{}]})['status'], 'no_text')

    def test_legacy_sanitizing_is_separate_from_strict_scoring(self):
        text = '[123가4567]'
        self.assertFalse(score_text(text, '123가4567')['exact_match'])
        self.assertEqual(legacy_normalize(text), '123가 4567')
        self.assertTrue(score_text(legacy_normalize(text), '123가4567')['exact_match'])
        self.assertFalse(score_text(legacy_normalize('12374567'), '123가4567')['exact_match'])

    def test_key_source_is_parsed_not_executed_and_redacted(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'source.py'
            path.write_text("raise RuntimeError('must not execute')\nGCP_API_KEY = 'fake-secret'\n")
            with patch.dict('os.environ', {}, clear=True):
                key = load_key(path)
            self.assertEqual(key, 'fake-secret')
            redacted = redact({'message': 'key=' + key, 'nested': [{key: key}]}, key)
            self.assertNotIn(key, json.dumps(redacted))

    def test_access_denied_stops_remaining_requests_without_scoring(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            image = directory/'image.png'
            image.write_bytes(b'fake-image-bytes')
            manifest = directory/'manifest.json'
            manifest.write_text(json.dumps({'rows': [
                {'id': str(i), 'image': str(image), 'image_sha256': sha256(image),
                 'condition': 'same_crop', 'kind': 'recorded_camera', 'expected': '123가4567'}
                for i in range(11)]}))
            response = SimpleNamespace(status_code=403, json=lambda: {
                'error': {'code': 403, 'message': 'denied fake-secret'}})
            args = SimpleNamespace(manifest=manifest, output=None, key_source_python=None)
            with patch('requests.post', return_value=response) as post, \
                    patch('probe_google_plate_ocr.load_key', return_value='fake-secret'), \
                    redirect_stdout(io.StringIO()):
                run(args)
                self.assertEqual(post.call_count, 1)
                self.assertFalse(post.call_args.kwargs['allow_redirects'])
                self.assertEqual(post.call_args.kwargs['headers']['x-goog-api-key'], 'fake-secret')
                with self.assertRaisesRegex(ValueError, 'Run already exists'):
                    run(args)
                self.assertEqual(post.call_count, 1)
            report = json.loads((directory/'run/report.json').read_text())
            self.assertTrue(report['halted_on_error'])
            self.assertEqual(report['summary']['same_crop/recorded_camera']['scored'], 0)
            self.assertEqual(sum(r['status']=='not_attempted_after_error' for r in report['rows']), 10)
            self.assertNotIn('fake-secret', (directory/'run/report.json').read_text())


if __name__ == '__main__':
    unittest.main()
