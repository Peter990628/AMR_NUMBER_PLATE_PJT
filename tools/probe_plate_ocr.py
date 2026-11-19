#!/usr/bin/env python3
"""Offline OCR on recorded plate boxes; never publishes ROS messages or commands.

The recognition function receives pixels only. Ground truth is used exclusively
by the subsequent scorer. No allowlist, character substitutions or best-of-N
selection based on the expected registration are applied.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import resource
import time
import unicodedata


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def normalize_text(text):
    """Remove whitespace and compose Unicode; preserve all other characters."""
    return ''.join(unicodedata.normalize('NFC', text).split())


def edit_distance(a, b):
    row = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        nxt = [i]
        for j, cb in enumerate(b, 1):
            nxt.append(min(nxt[-1] + 1, row[j] + 1, row[j-1] + (ca != cb)))
        row = nxt
    return row[-1]


def score_text(raw_text, expected):
    text = normalize_text(raw_text)
    label = normalize_text(expected)
    if not label:
        raise ValueError('Empty ground truth')
    distance = edit_distance(label, text)
    return {'normalized_text': text, 'expected': label, 'exact_match': text == label,
            'edit_distance': distance, 'character_error_rate': distance / len(label),
            'simple_plate_format': re.fullmatch(r'[0-9]{2,3}[가-힣][0-9]{4}', text) is not None}


def crop_bounds(box, width, height):
    values = [float(box[k]) for k in ('x', 'y', 'width', 'height')]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite bounding box')
    x, y, w, h = values
    if w <= 0 or h <= 0:
        raise ValueError('Nonpositive bounding box')
    raw = [math.floor(x), math.floor(y), math.ceil(x+w), math.ceil(y+h)]
    clipped = [max(0, raw[0]), max(0, raw[1]), min(width, raw[2]), min(height, raw[3])]
    if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
        raise ValueError('Bounding box outside image')
    return clipped, clipped != raw


def recognize(reader, bgr, mode='whole_box'):
    # EasyOCR internally resizes to its recognition height; use one whole-plate
    # region and greedy decoding consistently, without a second text detector.
    started = time.perf_counter()
    if mode == 'whole_box':
        predictions = reader.recognize(bgr, decoder='greedy', batch_size=1, workers=0,
                                      detail=1, paragraph=False, rotation_info=None)
        if len(predictions) != 1:
            raise RuntimeError('Expected one recognition result for one plate crop')
    else:
        predictions = reader.readtext(bgr, decoder='greedy', batch_size=1, workers=0,
                                      detail=1, paragraph=False, rotation_info=None,
                                      canvas_size=1024, mag_ratio=1.0)
        # This probe evaluates single-line plates. Keep every region, in image
        # order; never select the string closest to the expected registration.
        predictions = sorted(predictions, key=lambda r: sum(float(p[0]) for p in r[0])/len(r[0]))
    segments = [{'polygon': [[float(x),float(y)] for x,y in box], 'text': text,
                 'confidence': float(conf)} for box,text,conf in predictions]
    text = ' '.join(s['text'] for s in segments)
    confidence = min((s['confidence'] for s in segments), default=0.0)
    return {'raw_text': text, 'recognition_confidence': confidence, 'segments': segments,
            'recognition_ms': (time.perf_counter()-started)*1000}


def run(args):
    import cv2
    import torch
    import easyocr
    import numpy as np

    output = args.output.resolve()
    if output.exists():
        raise ValueError('Use a new output directory; existing results are preserved')
    model_dir = args.models.resolve()
    checkpoint = model_dir/'korean_g2.pth'
    if not checkpoint.is_file():
        raise ValueError('Missing Korean recognition model')
    manifest = json.loads(args.manifest.read_text())
    samples = manifest['samples']
    if len({s['id'] for s in samples}) != len(samples):
        raise ValueError('Duplicate sample IDs')
    for s in samples:
        if not re.fullmatch(r'[a-zA-Z0-9_-]+', s['id']):
            raise ValueError('Invalid sample ID')
        if sha256(s['image']) != s['image_sha256']:
            raise ValueError('Input image hash mismatch: '+s['id'])
    output.mkdir(parents=True)
    os.environ['EASYOCR_MODULE_PATH'] = str(model_dir)
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    cv2.setNumThreads(2)
    started = time.perf_counter()
    reader = easyocr.Reader(['ko', 'en'], gpu=False, detector=args.mode == 'text_regions',
                            model_storage_directory=str(model_dir),
                            user_network_directory=str(output/'user_network'),
                            download_enabled=False, verbose=False, quantize=True)
    init_sec = time.perf_counter()-started
    rows = []
    for sample in samples:
        image = cv2.imread(sample['image'])
        if image is None:
            raise ValueError('Cannot read image: '+sample['image'])
        row = dict(sample)
        row['image_size_wh'] = [image.shape[1], image.shape[0]]
        if sample.get('bbox') is None:
            row.update(status='no_plate_detection', ocr_attempted=False,
                       raw_text=None, normalized_text=None, exact_match=None,
                       end_to_end_exact=False, recognition_confidence=None)
        else:
            bounds, clipped = crop_bounds(sample['bbox'], image.shape[1], image.shape[0])
            x1, y1, x2, y2 = bounds
            crop = image[y1:y2, x1:x2].copy()
            crop_path = output/(sample['id']+'_crop.png')
            if not cv2.imwrite(str(crop_path), crop):
                raise RuntimeError('Cannot write crop')
            prediction = recognize(reader, crop, args.mode)
            # Expected text is intentionally not passed to the engine.
            scoring = score_text(prediction['raw_text'], sample['expected'])
            row.update(prediction)
            row.update(scoring)
            row.update(ocr_attempted=True, crop_bounds_xyxy=bounds,
                       crop_clipped=clipped, crop_image=str(crop_path),
                       crop_sha256=sha256(crop_path),
                       status='exact_match' if scoring['exact_match'] else 'text_mismatch',
                       end_to_end_exact=scoring['exact_match'])
        rows.append(row)
        print(json.dumps({k: row.get(k) for k in ['id','status','raw_text',
              'recognition_confidence','recognition_ms']}, ensure_ascii=False), flush=True)
        (output/'rows.partial.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2))
    stats = {}
    for phase in sorted({s['phase'] for s in rows if s['kind'] == 'recorded_camera'}):
        selected = [r for r in rows if r['kind']=='recorded_camera' and r['phase']==phase]
        stats[phase] = {'images': len(selected), 'ocr_attempted': sum(r['ocr_attempted'] for r in selected),
                        'exact_matches': sum(r['end_to_end_exact'] for r in selected)}
    report = {'engine': 'EasyOCR', 'engine_version': easyocr.__version__,
              'torch_version': torch.__version__, 'opencv_version': cv2.__version__,
              'numpy_version': np.__version__, 'model': 'korean_g2',
              'checkpoint_sha256': sha256(checkpoint), 'device': 'cpu', 'threads': 2,
              'mode': args.mode,
              'text_detector_sha256': sha256(model_dir/'craft_mlt_25k.pth') if args.mode == 'text_regions' else None,
              'init_sec': init_sec, 'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
              'manifest': str(args.manifest.resolve()), 'manifest_sha256': sha256(args.manifest),
              'generator_sha256': sha256(__file__),
              'method': 'Recorded plate detector boxes; '+args.mode+' greedy recognition; no custom allowlist; NFC/whitespace normalization only.',
              'limitations': manifest.get('limitations', []), 'summary_by_phase': stats, 'rows': rows}
    (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    (output/'rows.partial.json').unlink()
    print(json.dumps({'summary': stats, 'report': str(output/'report.json')}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--models', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--mode', choices=['whole_box','text_regions'], default='whole_box')
    run(parser.parse_args())
