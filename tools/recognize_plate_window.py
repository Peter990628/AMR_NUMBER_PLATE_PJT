#!/usr/bin/env python3
"""Bounded local OCR + truth-free consensus on a newly captured stationary window."""
import argparse
import json
import os
from pathlib import Path
import resource

from evaluate_plate_recovery import load_captures, validate_inputs, recognize_sample
from probe_plate_ocr import sha256
from recovery_ocr_policy import decide


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output directory')
    captures = load_captures([args.capture])
    validate_inputs(captures)
    capture = captures[0]
    baseline = json.loads(args.baseline.read_text())
    config = baseline['config']
    for name, digest in baseline['model_hashes'].items():
        if sha256(Path(config['model_dir'])/name) != digest:
            raise ValueError('ocr_model_changed')
    args.output.mkdir(parents=True)
    resource.setrlimit(resource.RLIMIT_AS, (5*1024**3, 5*1024**3))
    for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        os.environ[key] = '2'
    os.environ.update(PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK='True', HF_HUB_OFFLINE='1',
                      PADDLE_PDX_CACHE_HOME=str(args.output/'cache'),
                      HF_HOME=str(args.output/'cache/hf'), XDG_CACHE_HOME=str(args.output/'cache'))
    import cv2
    from paddleocr import TextRecognition
    cv2.setNumThreads(2)
    model = TextRecognition(**config)
    rows = []
    for sample in capture['samples']:
        row = recognize_sample(sample, model, args.output)
        # Offline scoring fields are intentionally absent from the stopping rule/report.
        for key in ('end_to_end_exact', 'exact_match', 'character_error_rate'):
            row.pop(key, None)
        rows.append(row)
    report = {'decision': decide(capture, rows), 'rows': rows,
              'capture_sha256': sha256(args.capture), 'baseline_sha256': sha256(args.baseline),
              'config': config, 'model_hashes': baseline['model_hashes'],
              'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024}
    (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report['decision'], ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
