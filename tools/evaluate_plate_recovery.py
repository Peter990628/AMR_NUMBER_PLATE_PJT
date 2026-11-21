#!/usr/bin/env python3
"""Compare fixed before/after capture windows with local, fixed PaddleOCR.

Also imports old representative snapshots, explicitly without temporal metrics.
Expected text is used only after recognition; no control or OCR-based selection.
"""
import argparse
import csv
from datetime import datetime, timezone
import html
import importlib.metadata
import json
import os
from pathlib import Path
import resource
import time

from plate_evaluation import (SCHEMA, PREPROCESS, compare_groups, identifier,
                              match_status, source_key, summarize)
from probe_paddle_plate_ocr import recognize, write_json
from probe_plate_ocr import crop_bounds, normalize_text, score_text, sha256

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASELINE = ROOT/'local_artifacts/paddle_ocr_validation_2026-11-19/whole_box_01/report.json'


def load_captures(paths):
    groups = []
    for path in paths:
        capture = json.loads(path.read_text())
        if capture.get('schema') != SCHEMA or capture.get('source_kind') != 'raw_camera':
            raise ValueError('Not a raw evaluation capture: '+str(path))
        identifier(capture['trial_id'])
        if capture['phase'] not in ('before','corner','after','returned'):
            raise ValueError('Invalid phase')
        samples = capture['samples']
        count = capture['sampling']['count']
        if not 1 <= count <= 120 or [s['slot'] for s in samples] != list(range(count)):
            raise ValueError('Incomplete scheduled slot list')
        keys = [source_key(s['image_header']) for s in samples if s.get('image')]
        if len(keys) != len(set(keys)):
            raise ValueError('Repeated source image cannot count twice')
        groups.append({**capture, 'provenance': {'capture': str(path.resolve()), 'sha256': sha256(path)}})
    return groups


def load_recorded(path):
    """Import every old camera snapshot; never infer consecutive frames from it."""
    manifest = json.loads(path.read_text())
    groups = {}
    for sample in manifest['samples']:
        if sample.get('kind') != 'recorded_camera':
            continue
        case, phase = identifier(sample['case']), sample['phase']
        if phase not in ('before','after','returned'):
            raise ValueError('Invalid recorded phase')
        if sha256(sample['source_report']) != sample['source_report_sha256']:
            raise ValueError('Recorded source report changed')
        observation = sample['original_observation']
        if source_key(observation['header'])[1] != sample['source_stamp_ns']:
            raise ValueError('Recorded source stamp mismatch')
        group = groups.setdefault((case,phase), {'schema': SCHEMA, 'trial_id': case, 'phase': phase,
              'source_kind': sample['source_kind'], 'capture_fault': None,
              'sampling': {'mode': 'representative_snapshots', 'temporal_metrics': False},
              'samples': [], 'provenance': {'manifest': str(path.resolve()), 'sha256': sha256(path)},
              'limitations': manifest.get('limitations', [])})
        group['samples'].append({**sample, 'slot': len(group['samples']),
                                 'capture_status': 'recorded_image',
                                 'image_header': observation['header'],
                                 'image_size_wh': [observation['image_width'], observation['image_height']],
                                 'observation': observation})
    if not groups:
        raise ValueError('No recorded camera samples')
    return list(groups.values())


def validate_inputs(groups):
    from PIL import Image
    ids = set()
    for group in groups:
        for sample in group['samples']:
            identifier(sample['id'])
            if sample['id'] in ids:
                raise ValueError('Duplicate sample ID')
            ids.add(sample['id'])
            if sample.get('image'):
                if sha256(sample['image']) != sample['image_sha256']:
                    raise ValueError('Source image changed: '+sample['id'])
                with Image.open(sample['image']) as image:
                    if list(image.size) != sample['image_size_wh']:
                        raise ValueError('Image dimensions disagree: '+sample['id'])
                    if image.width*image.height > 16_000_000:
                        raise ValueError('Image too large')
    # Reject duplicate phases before spending inference time.
    seen = [(g['trial_id'],g['phase']) for g in groups]
    if len(seen) != len(set(seen)):
        raise ValueError('Duplicate trial and phase')


def recognize_sample(sample, model, output):
    """Recognize an exact image/result pair without any ground-truth input."""
    import cv2
    row = dict(sample, ocr_attempted=False, end_to_end_exact=False,
               raw_text=None, recognition_confidence=None, recognition_ms=None,
               exact_match=None, character_error_rate=None)
    status = match_status(sample, sample.get('observation'))
    row['status'] = status
    if status != 'ready_for_ocr':
        return row
    image = cv2.imread(sample['image'])
    if image is None:
        raise ValueError('Cannot read validated image')
    bounds, clipped = crop_bounds(sample['observation'], image.shape[1], image.shape[0])
    x1,y1,x2,y2 = bounds
    started = time.perf_counter()
    crop = image[y1:y2,x1:x2]
    # Fixed on previous experiment; no warp, truth-based retries, or confidence gate.
    processed = cv2.resize(crop, (PREPROCESS['width'], PREPROCESS['height']), interpolation=cv2.INTER_CUBIC)
    elapsed = (time.perf_counter()-started)*1000
    raw_path, processed_path = output/(sample['id']+'_crop.png'), output/(sample['id']+'_ocr.png')
    if not cv2.imwrite(str(raw_path), crop) or not cv2.imwrite(str(processed_path), processed):
        raise RuntimeError('Cannot save OCR input')
    row.update(crop_image=str(raw_path), crop_sha256=sha256(raw_path),
               ocr_image=str(processed_path), ocr_image_sha256=sha256(processed_path),
               crop_bounds_xyxy=bounds, crop_clipped=clipped, preprocessing_ms=elapsed,
               ocr_attempted=True)
    try:
        prediction = recognize(model, processed_path)
    except Exception as error:
        # Failed inference is a failed scheduled sample, not silently excluded.
        row.update(status='ocr_error', error=f'{type(error).__name__}: {error}')
        return row
    row.update(**prediction, status='recognized')
    return row


def evaluate_sample(sample, model, output, expected):
    row = recognize_sample(sample, model, output)
    if row['status'] == 'recognized':
        scoring = score_text(row['raw_text'], expected)
        row.update(**scoring, end_to_end_exact=scoring['exact_match'],
                   status='exact_match' if scoring['exact_match'] else 'text_mismatch')
    return row


def write_outputs(output, report):
    write_json(output/'report.json', report)
    with (output/'frames.csv').open('w', newline='') as stream:
        columns = ['trial_id','phase','slot','id','status','detector_status','detector_confidence',
                   'ocr_attempted','raw_text','recognition_confidence','end_to_end_exact',
                   'character_error_rate','recognition_ms','source_stamp_ns']
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for group in report['groups']:
            for row in group['rows']:
                observation = row.get('observation') or {}
                values = {**row, 'trial_id': group['trial_id'], 'phase': group['phase'],
                          'detector_status': observation.get('status'),
                          'detector_confidence': observation.get('confidence')}
                writer.writerow({k:values.get(k) for k in columns})
    def esc(value):
        return html.escape(str(value))
    lines = ['<!doctype html><meta charset="utf-8"><title>번호판 이동 전후 평가</title>',
             '<style>body{font:17px sans-serif;max-width:1400px;margin:30px auto;color:#192b3b;background:#f6f8fa}table{border-collapse:collapse;background:white;width:100%;margin-bottom:25px}td,th{padding:12px;border:1px solid #ccd5df;text-align:left}img{max-width:420px;max-height:150px}.good{color:#146b35}.bad{color:#a12626}details{margin:12px 0}code{word-break:break-all}</style>',
             '<h1>번호판 이동 전후 평가</h1>',
             '<p>동일 PaddleOCR + 전체 crop 520×110 크기·비율 조정. 정답은 채점에만 사용. 미검출·입력 누락도 분모에 포함.</p>',
             '<p>이 결과의 판독률은 저장/예약 표본 기준이며 전체 카메라 프레임의 판독률이 아닙니다.</p>',
             '<table><tr><th>실험</th><th>단계</th><th>전체 일치 / 표본</th><th>OCR 시도</th><th>최장 연속 표본 일치</th><th>입력 종류</th></tr>']
    for group in report['groups']:
        s = group['summary']
        cells = [group['trial_id'], group['phase'], f"{s['exact_matches']} / {s['scheduled_samples']}",
                 s['ocr_attempts'], s['longest_exact_sample_streak'] if s['longest_exact_sample_streak'] is not None else '계산 불가(대표 사진)', group['source_kind']]
        lines.append('<tr>'+''.join('<td>'+esc(c)+'</td>' for c in cells)+'</tr>')
    lines.append('</table><h2>전후 비교</h2>')
    for pair in report['comparisons']:
        delta = pair['after_minus_before_exact_fraction']
        lines.append('<p>'+esc(pair['trial_id'])+': '+esc(pair['comparison_status'])+
                     (f' · 일치 비율 변화 {delta*100:+.1f}%p' if delta is not None else ' · 비교 수치 산출 안 함')+'</p>')
    lines.append('<h2>각 표본 — 점수가 좋은 표본만 선택하지 않음</h2>')
    for group in report['groups']:
        lines.append('<details open><summary>'+esc(group['trial_id']+'/'+group['phase'])+'</summary><table>')
        for row in group['rows']:
            picture = row.get('ocr_image') or row.get('image')
            image_html = ''
            if picture:
                relative = esc(os.path.relpath(picture, output))
                image_html = f'<a href="{relative}"><img src="{relative}"></a>'
            color = 'good' if row['end_to_end_exact'] else 'bad'
            lines.append(f'<tr><td>{esc(row["id"])}</td><td>{image_html}</td><td class="{color}">{esc(row["status"])}<br>{esc(row.get("raw_text") or "OCR 미실행")}<br>OCR 점수: {esc(row.get("recognition_confidence"))}</td></tr>')
        lines.append('</table></details>')
    lines.append('<h2>범위와 한계</h2><ul>'+''.join('<li>'+esc(s)+'</li>' for s in report['limitations'])+'</ul>')
    (output/'comparison.html').write_text('\n'.join(lines), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--captures', nargs='+', type=Path)
    source.add_argument('--recorded-manifest', type=Path)
    parser.add_argument('--expected', required=True, help='Ground truth used only by scorer')
    parser.add_argument('--baseline', type=Path, default=DEFAULT_BASELINE)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not normalize_text(args.expected) or args.output.exists():
        parser.error('Supply expected text and a new output directory')
    groups = load_captures(args.captures) if args.captures else load_recorded(args.recorded_manifest)
    validate_inputs(groups)
    baseline = json.loads(args.baseline.read_text())
    config = baseline['config']
    for name, digest in baseline['model_hashes'].items():
        if sha256(Path(config['model_dir'])/name) != digest:
            raise ValueError('Model changed from fixed baseline')
    output = args.output.resolve(); output.mkdir(parents=True)
    frozen = {'groups': groups, 'config': config, 'preprocessing': PREPROCESS,
              'expected_for_scoring_only': args.expected, 'model_hashes': baseline['model_hashes'],
              'baseline_sha256': sha256(args.baseline)}
    write_json(output/'inputs.json', frozen)
    resource.setrlimit(resource.RLIMIT_AS, (5*1024**3, 5*1024**3))
    for name in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS'):
        os.environ[name] = '2'
    os.environ.update(PADDLE_PDX_CACHE_HOME=str(output/'cache/paddlex'), PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK='True',
                      HF_HOME=str(output/'cache/hf'), HF_HUB_OFFLINE='1', XDG_CACHE_HOME=str(output/'cache'))
    import cv2
    from paddleocr import TextRecognition
    cv2.setNumThreads(2)
    model = TextRecognition(**config)
    for group in groups:
        rows = []
        for sample in group['samples']:
            row = evaluate_sample(sample, model, output, args.expected)
            rows.append(row)
            print(json.dumps({'id':row['id'], 'status':row['status'], 'raw_text':row['raw_text']}, ensure_ascii=False), flush=True)
            write_json(output/'progress.json', {'group': group['trial_id']+'/'+group['phase'], 'rows': rows})
        group['rows'] = rows
        group['summary'] = summarize(rows, temporal=group['sampling']['mode']=='fixed_wall_slots')
        del group['samples']
    report = {'schema': SCHEMA, 'created_at_utc': datetime.now(timezone.utc).isoformat(),
              'groups': groups, 'comparisons': compare_groups(groups), 'config':config,
              'expected_for_scoring_only': args.expected,
              'preprocessing':PREPROCESS, 'model_hashes':baseline['model_hashes'],
              'inputs_sha256':sha256(output/'inputs.json'),
              'code_hashes':{name:sha256(Path(__file__).with_name(name)) for name in
                             ('evaluate_plate_recovery.py','plate_evaluation.py','probe_paddle_plate_ocr.py','probe_plate_ocr.py')},
              'versions':{name:importlib.metadata.version(name) for name in
                          ('paddleocr','paddlex','paddlepaddle','opencv-contrib-python','numpy')},
              'peak_rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
              'limitations':['One synthetic plate; not accuracy on independent vehicles.',
                             'Offline OCR is an evaluation tool, not a robot stopping rule.',
                             'Missed frames, unmatched detector outputs, and OCR errors remain in scheduled denominators.',
                             'Streaks describe consecutive scheduled samples, not every sensor frame.',
                             'Representative historical images include debug overlays and cannot establish temporal stability.',
                             'No robot movement or ROS command publishing by this evaluator.']}
    write_outputs(output, report)
    (output/'progress.json').unlink()
    print(json.dumps({'comparisons':report['comparisons'], 'report':str(output/'report.json')},ensure_ascii=False))


if __name__ == '__main__':
    main()
