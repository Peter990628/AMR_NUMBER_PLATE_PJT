#!/usr/bin/env python3
"""Freeze geometry-only variants, then compare them with unchanged PaddleOCR."""
import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import os
from pathlib import Path
import resource
import time

from probe_plate_ocr import sha256, score_text
from probe_paddle_plate_ocr import recognize, write_json


def prepare(args):
    import cv2
    import numpy as np
    from rectify_plate_crop import PARAMETERS, prepare_variants
    output = args.output.resolve()
    if output.exists():
        raise ValueError('Use a new output directory')
    baseline = json.loads(args.baseline.read_text())
    if len(baseline['rows']) != 6:
        raise ValueError('Expected the six frozen baseline crops')
    for row in baseline['rows']:
        if sha256(row['crop_image']) != row['crop_sha256']:
            raise ValueError('Source crop changed')
    output.mkdir(parents=True)
    (output/'inputs').mkdir()
    rows = []
    for sample in baseline['rows']:
        image = cv2.imread(sample['crop_image'])
        if image is None:
            raise ValueError('Cannot read crop')
        started = time.perf_counter()
        variants, geometry = prepare_variants(image, (520, 110))
        geometry['preprocessing_ms'] = (time.perf_counter()-started)*1000
        preview = image.copy()
        if geometry['corners_xy'] is not None:
            points = np.asarray(geometry['corners_xy'], np.int32)
            cv2.polylines(preview, [points], True, (0, 0, 255), 1)
            for i, (x, y) in enumerate(points):
                cv2.circle(preview, (int(x), int(y)), 2, (255, 0, 0), -1)
        preview_path = output/'inputs'/(sample['id']+'_corners.png')
        cv2.imwrite(str(preview_path), preview)
        files = {}
        for name, variant in variants.items():
            path = output/'inputs'/(sample['id']+'__'+name+'.png')
            if not cv2.imwrite(str(path), variant):
                raise ValueError('Cannot save variant')
            files[name] = {'image': str(path), 'sha256': sha256(path),
                           'size_wh': [variant.shape[1], variant.shape[0]]}
        rows.append({'id': sample['id'], 'case': sample['case'], 'kind': sample['kind'],
                     'expected': sample['expected'], 'source_crop': sample['crop_image'],
                     'source_crop_sha256': sample['crop_sha256'], 'geometry': geometry,
                     'corner_preview': str(preview_path), 'variants': files,
                     'baseline_raw_text': sample['raw_text']})
    manifest = {'prepared_at_utc': datetime.now(timezone.utc).isoformat(),
                'baseline': str(args.baseline.resolve()), 'baseline_sha256': sha256(args.baseline),
                'model_config': baseline['config'], 'model_hashes': baseline['model_hashes'],
                'geometry_parameters': PARAMETERS,
                'geometry_code_sha256': sha256(Path(__file__).with_name('rectify_plate_crop.py')),
                'aspect_prior': '520/110, known synthetic plate dimensions 0.52m/0.11m from assets/license_plate/mount.json; not inferred from pixels.',
                'opencv_version': cv2.__version__, 'rows': rows}
    write_json(output/'manifest.json', manifest)
    print(json.dumps({'prepared': len(rows), 'quad_found': sum(r['geometry']['status']=='quad_found' for r in rows),
                      'manifest': str(output/'manifest.json')}, ensure_ascii=False))


def run(args):
    manifest = json.loads(args.manifest.read_text())
    output = args.manifest.resolve().parent/'run'
    if output.exists():
        raise ValueError('Preserve existing run')
    for sample in manifest['rows']:
        for variant in sample['variants'].values():
            if sha256(variant['image']) != variant['sha256']:
                raise ValueError('Prepared variant changed')
    config = manifest['model_config']
    for name, expected_hash in manifest['model_hashes'].items():
        if sha256(Path(config['model_dir'])/name) != expected_hash:
            raise ValueError('Model changed from baseline')
    output.mkdir()
    resource.setrlimit(resource.RLIMIT_AS, (5*1024**3, 5*1024**3))
    for name in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        os.environ[name] = '2'
    os.environ['PADDLE_PDX_CACHE_HOME'] = str(output/'cache/paddlex')
    os.environ['PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK'] = 'True'
    os.environ['HF_HOME'] = str(output/'cache/huggingface')
    os.environ['HF_HUB_OFFLINE'] = '1'
    os.environ['XDG_CACHE_HOME'] = str(output/'cache')
    import cv2
    from paddleocr import TextRecognition
    cv2.setNumThreads(2)
    model = TextRecognition(**config)
    rows = []
    for sample in manifest['rows']:
        row = dict(sample, results={})
        for name, variant in sample['variants'].items():
            prediction = recognize(model, variant['image'])
            row['results'][name] = dict(prediction, **score_text(prediction['raw_text'], sample['expected']))
            print(json.dumps({'id': sample['id'], 'condition': name,
                              'geometry': sample['geometry']['status'], **row['results'][name]}, ensure_ascii=False), flush=True)
        rows.append(row)
        write_json(output/'rows.partial.json', rows)
    summary = {}
    for kind in ('recorded_camera', 'texture_control'):
        selected = [r for r in rows if r['kind']==kind]
        summary[kind] = {'images': len(selected),
                         'quad_found': sum(r['geometry']['status']=='quad_found' for r in selected),
                         'exact_matches': {name: sum(r['results'][name]['exact_match'] for r in selected)
                                           for name in ('original','resize_only','recrop_resize','perspective')}}
    report = {'manifest': str(args.manifest.resolve()), 'manifest_sha256': sha256(args.manifest),
              'versions': {n: importlib.metadata.version(n) for n in ('paddleocr','paddlex','paddlepaddle','opencv-contrib-python')},
              'model_config': config, 'model_hashes': manifest['model_hashes'],
              'summary': summary, 'rows': rows, 'generator_sha256': sha256(__file__),
              'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024}
    write_json(output/'report.json', report)
    (output/'rows.partial.json').unlink()
    print(json.dumps(summary, ensure_ascii=False))


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    prep = sub.add_parser('prepare')
    prep.add_argument('--baseline', type=Path, required=True)
    prep.add_argument('--output', type=Path, required=True)
    execute = sub.add_parser('run')
    execute.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args()
    (prepare if args.command=='prepare' else run)(args)
