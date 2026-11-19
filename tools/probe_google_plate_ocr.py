#!/usr/bin/env python3
"""Compare Google Vision with saved OCR inputs; no ROS or robot commands.

prepare freezes and hashes images before any cloud request. run sends only those
images plus the original language hint, once each, without retries. Expected
text is used solely for scoring returned results. Credentials stay in memory.
"""
import argparse
import ast
import base64
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
import time

from probe_plate_ocr import crop_bounds, score_text, sha256

ENDPOINT = 'https://vision.googleapis.com/v1/images:annotate'


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def legacy_bounds(box, width, height):
    """Original detector path: int xyxy, then 30 px padding and clipping."""
    crop_bounds(box, width, height)  # Validate before truncating.
    x, y, w, h = [float(box[k]) for k in ('x', 'y', 'width', 'height')]
    return [max(0, int(x)-30), max(0, int(y)-30),
            min(width, int(x+w)+30), min(height, int(y+h)+30)]


def legacy_normalize(text):
    """Reproduce the external node; strict scoring is also retained separately."""
    text = re.sub(r'[^0-9가-힣]', '', text.replace('\n', '').replace(' ', ''))
    if re.fullmatch(r'[0-9]{2,3}[가-힣][0-9]{4}', text):
        return text[:-4] + ' ' + text[-4:]
    return text


def make_request(image_bytes):
    return {'requests': [{'image': {'content': base64.b64encode(image_bytes).decode('ascii')},
                          'features': [{'type': 'DOCUMENT_TEXT_DETECTION'}],
                          'imageContext': {'languageHints': ['ko']}}]}


def parse_response(payload):
    if payload.get('error'):
        return {'status': 'api_error', 'api_error': payload['error']}
    responses = payload.get('responses', [])
    if len(responses) != 1:
        return {'status': 'invalid_response'}
    response = responses[0]
    if response.get('error'):
        return {'status': 'api_error', 'api_error': response['error']}
    annotations = response.get('textAnnotations', [])
    text = annotations[0].get('description', '') if annotations else ''
    confidence = []
    for page in response.get('fullTextAnnotation', {}).get('pages', []):
        for block in page.get('blocks', []):
            for paragraph in block.get('paragraphs', []):
                for word in paragraph.get('words', []):
                    confidence.extend(float(s.get('confidence', 0)) for s in word.get('symbols', []))
    return {'status': 'text' if text else 'no_text', 'raw_text': text,
            'symbol_count': len(confidence),
            'mean_symbol_confidence': sum(confidence)/len(confidence) if confidence else None,
            'legacy_normalized_text': legacy_normalize(text)}


def redact(value, secret):
    if isinstance(value, str):
        return value.replace(secret, '[REDACTED]') if secret else value
    if isinstance(value, list):
        return [redact(v, secret) for v in value]
    if isinstance(value, dict):
        return {redact(k, secret): redact(v, secret) for k, v in value.items()}
    return value


def load_key(source):
    if os.environ.get('GOOGLE_VISION_API_KEY'):
        return os.environ['GOOGLE_VISION_API_KEY']
    if source is not None:
        # Parse a literal only. Never import/execute the external ROS node.
        tree = ast.parse(source.read_text())
        for node in tree.body:
            if isinstance(node, ast.Assign) and any(
                    isinstance(t, ast.Name) and t.id == 'GCP_API_KEY' for t in node.targets):
                value = ast.literal_eval(node.value)
                if isinstance(value, str) and value.strip():
                    return value
    raise ValueError('No API key configured; use GOOGLE_VISION_API_KEY or --key-source-python')


def prepare(args):
    import cv2
    import numpy as np
    baseline = json.loads(args.baseline.read_text())
    selected = [r for r in baseline['rows'] if r.get('ocr_attempted') and
                (r['phase'] == 'after' or r['kind'] == 'texture_control')]
    if len(selected) != 6 or sum(r['kind'] == 'recorded_camera' for r in selected) != 5:
        raise ValueError('Expected the fixed five after images and one texture control')
    output = args.output.resolve()
    if output.exists():
        raise ValueError('Use a new directory; existing prepared inputs are preserved')
    output.mkdir(parents=True)
    (output/'inputs').mkdir()
    rows = []
    for condition in ('same_crop', 'legacy_preprocess'):
        for s in selected:
            if condition == 'legacy_preprocess' and s['kind'] == 'texture_control':
                continue  # Texture is already 2080x440; it is only a native-resolution control.
            if not re.fullmatch(r'[a-zA-Z0-9_-]+', s['id']):
                raise ValueError('Invalid sample ID')
            if sha256(s['image']) != s['image_sha256'] or sha256(s['crop_image']) != s['crop_sha256']:
                raise ValueError('Baseline input hash mismatch: ' + s['id'])
            image = cv2.imread(s['image'])
            if image is None:
                raise ValueError('Cannot read source image')
            row_id = s['id'] + '__' + condition
            path = output/'inputs'/(row_id + ('.png' if condition == 'same_crop' else '.jpg'))
            if condition == 'same_crop':
                bounds, _ = crop_bounds(s['bbox'], image.shape[1], image.shape[0])
                x1, y1, x2, y2 = bounds
                crop = cv2.imread(s['crop_image'])
                if not np.array_equal(crop, image[y1:y2, x1:x2]):
                    raise ValueError('Baseline crop differs from source pixels')
                shutil.copyfile(s['crop_image'], path)
            else:
                bounds = legacy_bounds(s['bbox'], image.shape[1], image.shape[0])
                x1, y1, x2, y2 = bounds
                crop = cv2.resize(image[y1:y2, x1:x2], None, fx=8, fy=8, interpolation=cv2.INTER_CUBIC)
                if not cv2.imwrite(str(path), crop):
                    raise ValueError('Cannot save prepared JPEG')
            rows.append({'id': row_id, 'sample_id': s['id'], 'kind': s['kind'],
                         'condition': condition, 'source_image': s['image'],
                         'source_sha256': s['image_sha256'], 'crop_bounds_xyxy': bounds,
                         'image': str(path), 'image_sha256': sha256(path),
                         'image_size_wh': [crop.shape[1], crop.shape[0]], 'expected': s['expected'],
                         'baseline_raw_text': s['raw_text'],
                         'baseline_exact_match': s['exact_match']})
    manifest = {'prepared_at_utc': datetime.now(timezone.utc).isoformat(),
                'baseline_report': str(args.baseline.resolve()), 'baseline_sha256': sha256(args.baseline),
                'endpoint': ENDPOINT, 'feature': 'DOCUMENT_TEXT_DETECTION', 'language_hints': ['ko'],
                'legacy_preprocess': 'Recorded box (not original YOLO), int xyxy + 30px margin, 8x INTER_CUBIC, OpenCV default JPEG quality 95.',
                'opencv_version': cv2.__version__, 'rows': rows}
    write_json(output/'manifest.json', manifest)
    print(json.dumps({'prepared_images': len(rows), 'manifest': str(output/'manifest.json')}))


def summarize(rows):
    result = {}
    for condition in sorted({r['condition'] for r in rows}):
        for kind in sorted({r['kind'] for r in rows if r['condition'] == condition}):
            group = [r for r in rows if r['condition'] == condition and r['kind'] == kind]
            scored = [r for r in group if r.get('exact_match') is not None]
            result[condition + '/' + kind] = {
                'planned': len(group), 'requests_sent': sum(r['request_sent'] for r in group),
                'scored': len(scored), 'strict_exact_matches': sum(r['exact_match'] for r in scored),
                'legacy_normalized_exact_matches': sum(r['legacy_scoring']['exact_match'] for r in scored)}
    return result


def run(args):
    import requests
    manifest = json.loads(args.manifest.read_text())
    rows = manifest['rows']
    if len(rows) != 11 or len({r['id'] for r in rows}) != 11:
        raise ValueError('Expected exactly 11 unique prepared requests')
    for r in rows:
        if not re.fullmatch(r'[a-zA-Z0-9_-]+', r['id']):
            raise ValueError('Invalid request ID')
        if sha256(r['image']) != r['image_sha256']:
            raise ValueError('Prepared image changed: ' + r['id'])
    output = args.output.resolve() if args.output else args.manifest.resolve().parent/'run'
    if output.exists():
        raise ValueError('Run already exists; no implicit retries or overwrite')
    key = load_key(args.key_source_python)
    output.mkdir()
    completed = []
    halted = False
    for item in rows:
        row = dict(item, request_sent=False, exact_match=None)
        if halted:
            row['status'] = 'not_attempted_after_error'
        else:
            started = time.perf_counter()
            row['started_at_utc'] = datetime.now(timezone.utc).isoformat()
            row['request_sent'] = True
            try:
                response = requests.post(ENDPOINT,
                    headers={'x-goog-api-key': key, 'Content-Type': 'application/json'},
                    json=make_request(Path(row['image']).read_bytes()),
                    timeout=(10, 60), allow_redirects=False)
                row['http_status'] = response.status_code
                try:
                    body = redact(response.json(), key)
                except ValueError:
                    body = None
                if body is not None:
                    write_json(output/(row['id'] + '_response.json'), body)
                if response.status_code != 200:
                    row.update(status='http_error', api_error=body.get('error') if isinstance(body, dict) else None)
                elif not isinstance(body, dict):
                    row['status'] = 'invalid_response'
                else:
                    row.update(parse_response(body))
            except requests.RequestException as error:
                # Exception text/URLs/headers are not persisted, to protect credentials.
                row.update(status='network_error', error_type=type(error).__name__)
            row['request_ms'] = (time.perf_counter()-started)*1000
            if row['status'] in ('text', 'no_text'):
                row.update(score_text(row['raw_text'], row['expected']))
                row['legacy_scoring'] = score_text(row['legacy_normalized_text'], row['expected'])
            else:
                halted = True
        completed.append(redact(row, key))
        write_json(output/'rows.partial.json', completed)
        print(json.dumps({k: row.get(k) for k in ('id', 'status', 'raw_text', 'exact_match', 'request_ms')}, ensure_ascii=False), flush=True)
    report = {'engine': 'Google Cloud Vision', 'feature': 'DOCUMENT_TEXT_DETECTION',
              'language_hints': ['ko'], 'endpoint': ENDPOINT, 'model': 'service default (version not returned)',
              'manifest': str(args.manifest.resolve()), 'manifest_sha256': sha256(args.manifest),
              'generator_sha256': sha256(__file__), 'halted_on_error': halted,
              'summary': summarize(completed), 'rows': completed}
    write_json(output/'report.json', report)
    (output/'rows.partial.json').unlink()
    print(json.dumps({'summary': report['summary'], 'report': str(output/'report.json')}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    prep = sub.add_parser('prepare')
    prep.add_argument('--baseline', type=Path, required=True)
    prep.add_argument('--output', type=Path, required=True)
    execute = sub.add_parser('run')
    execute.add_argument('--manifest', type=Path, required=True)
    execute.add_argument('--key-source-python', type=Path)
    execute.add_argument('--output', type=Path, help='New run directory; never overwrites or retries an existing run')
    args = parser.parse_args()
    (prepare if args.command == 'prepare' else run)(args)
