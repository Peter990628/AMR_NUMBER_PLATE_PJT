#!/usr/bin/env python3
"""Run a local PaddleOCR recognizer on the six frozen plate crops.

No training, ROS publishing, motion, character substitutions, or truth-based
selection. Ground truth is passed only to the scorer after inference.
"""
import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import os
from pathlib import Path
import re
import resource
import time

from probe_plate_ocr import score_text, sha256

MODEL = 'korean_PP-OCRv5_mobile_rec'


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def select_inputs(baseline):
    rows = [r for r in baseline['rows'] if r.get('ocr_attempted') and
            (r['phase'] == 'after' or r['kind'] == 'texture_control')]
    if len(rows) != 6 or sum(r['kind'] == 'texture_control' for r in rows) != 1:
        raise ValueError('Expected five after images and one texture control')
    if len({r['id'] for r in rows}) != 6:
        raise ValueError('Duplicate input IDs')
    # A clean-texture control is evaluated first; every input is still evaluated.
    return sorted(rows, key=lambda r: r['kind'] != 'texture_control')


def recognize(model, crop_path):
    started = time.perf_counter()
    predictions = list(model.predict(input=str(crop_path), batch_size=1))
    if len(predictions) != 1:
        raise RuntimeError('Expected exactly one recognition result')
    result = predictions[0]
    return {'raw_text': str(result['rec_text']),
            'recognition_confidence': float(result['rec_score']),
            'recognition_ms': (time.perf_counter()-started)*1000}


def run(args):
    output = args.output.resolve()
    if output.exists():
        raise ValueError('Use a new directory; preserve earlier runs')
    baseline = json.loads(args.baseline.read_text())
    inputs = select_inputs(baseline)
    for row in inputs:
        if not re.fullmatch(r'[a-zA-Z0-9_-]+', row['id']):
            raise ValueError('Invalid sample ID')
        if sha256(row['crop_image']) != row['crop_sha256'] or sha256(row['image']) != row['image_sha256']:
            raise ValueError('Saved input changed: ' + row['id'])
    model_dir = args.model_dir.resolve()
    model_hashes = {name: sha256(model_dir/name)
                    for name in ('inference.json', 'inference.pdiparams', 'inference.yml')}
    output.mkdir(parents=True)
    # Protect the running desktop/Isaac Sim from an unexpected process allocation.
    resource.setrlimit(resource.RLIMIT_AS, (5 * 1024**3, 5 * 1024**3))
    for name in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        os.environ[name] = '2'
    os.environ['PADDLE_PDX_CACHE_HOME'] = str(output/'cache/paddlex')
    os.environ['PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK'] = 'True'
    os.environ['HF_HOME'] = str(output/'cache/huggingface')
    os.environ['HF_HUB_OFFLINE'] = '1'
    os.environ['XDG_CACHE_HOME'] = str(output/'cache')
    # Inference is executed without network escalation, with an explicit local model.
    import cv2
    from paddleocr import TextRecognition
    cv2.setNumThreads(2)
    config = {'model_name': MODEL, 'model_dir': str(model_dir), 'device': 'cpu',
              'cpu_threads': 2, 'enable_mkldnn': False, 'enable_hpi': False}
    manifest = {'created_at_utc': datetime.now(timezone.utc).isoformat(),
                'baseline_report': str(args.baseline.resolve()), 'baseline_sha256': sha256(args.baseline),
                'model_hashes': model_hashes, 'config': config,
                'inputs': [{k: r[k] for k in ('id', 'kind', 'case', 'phase', 'crop_image',
                           'crop_sha256', 'image', 'image_sha256', 'expected')} for r in inputs]}
    write_json(output/'manifest.json', manifest)
    started = time.perf_counter()
    model = TextRecognition(**config)
    init_sec = time.perf_counter()-started
    rows = []
    for sample in manifest['inputs']:
        prediction = recognize(model, sample['crop_image'])
        scoring = score_text(prediction['raw_text'], sample['expected'])
        row = dict(sample, **prediction, **scoring)
        row['status'] = 'exact_match' if row['exact_match'] else 'text_mismatch'
        rows.append(row)
        write_json(output/'rows.partial.json', rows)
        print(json.dumps({k: row[k] for k in ('id', 'status', 'raw_text',
                         'recognition_confidence', 'recognition_ms')}, ensure_ascii=False), flush=True)
    summary = {kind: {'images': sum(r['kind']==kind for r in rows),
                     'exact_matches': sum(r['kind']==kind and r['exact_match'] for r in rows)}
               for kind in ('recorded_camera', 'texture_control')}
    report = {'engine': 'PaddleOCR', 'model': MODEL, 'mode': 'whole_box',
              'versions': {name: importlib.metadata.version(name) for name in
                           ('paddleocr', 'paddlex', 'paddlepaddle', 'numpy', 'opencv-contrib-python')},
              'config': config, 'model_hashes': model_hashes, 'init_sec': init_sec,
              'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
              'address_space_limit_gib': 5, 'generator_sha256': sha256(__file__),
              'manifest_sha256': sha256(output/'manifest.json'), 'summary': summary, 'rows': rows,
              'limitations': ['Same synthetic plate; five after images include three revisits.',
                             'Camera crops contain debug detector overlays.',
                             'No live OCR, before/after movement comparison, or robot motion in this probe.',
                             'NFC and whitespace normalization only; no substitutions or custom allowlist.']}
    write_json(output/'report.json', report)
    (output/'rows.partial.json').unlink()
    print(json.dumps({'summary': summary, 'report': str(output/'report.json')}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--model-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    run(parser.parse_args())
