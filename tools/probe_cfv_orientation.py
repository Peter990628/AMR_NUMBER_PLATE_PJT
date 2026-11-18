#!/usr/bin/env python3
"""Offline CFV checkpoint evaluation. Reads images; never connects to ROS.

Manifest: {"samples": [{"id": "...", "image": "path", "group": "...",
                       "cfv_label_deg": 180, "note": "..."}]}
Image paths are relative to --image-root. cfv_label_deg is optional and must
only be supplied for independently labeled CFV-compatible reference images.
"""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace


def circular_distance(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def angle_bin(angle):
    """Use upstream AngleDataset boundaries, including their exact endpoints."""
    angle %= 360.0
    if angle <= 22.5 or angle >= 337.5:
        return 0
    return math.ceil((angle - 22.5) / 45.0)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--image-root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--deps', type=Path, help='Optional local timm dependency directory')
    parser.add_argument('--threads', type=int, default=4)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('--output must be a new directory')
    if args.threads < 1:
        parser.error('--threads must be positive')
    if args.deps:
        sys.path.insert(0, str(args.deps.resolve()))
    import cv2
    import numpy as np
    import torch
    import timm
    import torchvision
    if timm.__version__ != '0.6.12':
        parser.error('Use timm==0.6.12 to match the official checkpoint architecture')
    torch.set_num_threads(args.threads)
    spec = importlib.util.spec_from_file_location(
        'cfv_official_multitask', args.upstream / 'models/multitask_models.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = module.ConvNextSmallMultiModel(SimpleNamespace(pretrained=False))
    state = torch.load(args.checkpoint, map_location='cpu', weights_only=True)
    model.load_state_dict(state, strict=True)
    model.eval()
    manifest = json.loads(args.manifest.read_text())
    samples = manifest['samples']
    if not samples or len({s['id'] for s in samples}) != len(samples):
        parser.error('Manifest needs nonempty samples with unique IDs')
    args.output.mkdir(parents=True)
    # Warm-up is excluded from timing. No downloads or external inference APIs.
    with torch.inference_mode():
        model(torch.zeros(1, 3, 224, 224))
    results = []
    for sample in samples:
        path = (args.image_root / sample['image']).resolve()
        original = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if original is None:
            raise ValueError(f'Cannot read image: {path}')
        # Match prepare_data.py resize + PIL RGB + ToTensor. No Normalize.
        rgb = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (224, 224), interpolation=cv2.INTER_LINEAR)
        tensor = torch.from_numpy(resized.transpose(2, 0, 1).copy())
        tensor = tensor.to(torch.float32).div(255).unsqueeze(0)
        start = time.perf_counter()
        with torch.inference_mode():
            logits, normalized_angle = model(tensor)
        elapsed = (time.perf_counter() - start) * 1000
        probs = logits.softmax(dim=1)[0]
        angle = float(normalized_angle.item()) * 360.0
        if not math.isfinite(angle) or not torch.isfinite(probs).all():
            raise ValueError(f'Nonfinite model output: {path}')
        predicted_class = int(probs.argmax().item())
        result = {
            **sample, 'image_absolute': str(path), 'image_sha256': sha256(path),
            'image_size_wh': [original.shape[1], original.shape[0]],
            'angle_deg': angle, 'rear_deviation_deg': circular_distance(angle, 180),
            'class_index': predicted_class,
            'class_center_deg': 45 * predicted_class,
            'class_softmax': [float(p) for p in probs],
            'class_probability_uncalibrated': float(probs.max()),
            'angle_class_index': angle_bin(angle),
            'class_angle_bins_agree': angle_bin(angle) == predicted_class,
            'inference_ms_cpu': elapsed,
        }
        if sample.get('cfv_label_deg') is not None:
            label = float(sample['cfv_label_deg'])
            if not math.isfinite(label):
                raise ValueError('Reference label must be finite')
            result['circular_error_deg'] = circular_distance(angle, label)
        if sample.get('geometric_reference_deg') is not None:
            reference = float(sample['geometric_reference_deg'])
            if not math.isfinite(reference):
                raise ValueError('Geometric reference must be finite')
            # Controlled camera position around a defined vehicle center. Keep
            # separate from CFV's human/interpolated view-angle annotations.
            result['geometric_difference_deg'] = circular_distance(angle, reference)
        results.append(result)
        print(f"{sample['id']}: {angle:.2f} deg, rear deviation "
              f"{result['rear_deviation_deg']:.2f} deg, {elapsed:.1f} ms", flush=True)
    groups = {}
    for group in dict.fromkeys(r.get('group', 'default') for r in results):
        rows = [r for r in results if r.get('group', 'default') == group]
        angles = np.radians([r['angle_deg'] for r in rows])
        mean = math.degrees(math.atan2(np.sin(angles).mean(), np.cos(angles).mean())) % 360
        groups[group] = {
            'count': len(rows), 'circular_mean_deg': mean,
            'max_pairwise_angle_difference_deg': max(
                circular_distance(a['angle_deg'], b['angle_deg']) for a in rows for b in rows),
            'mean_rear_deviation_deg': float(np.mean([r['rear_deviation_deg'] for r in rows])),
            'mean_inference_ms_cpu': float(np.mean([r['inference_ms_cpu'] for r in rows])),
        }
        errors = [r['circular_error_deg'] for r in rows if 'circular_error_deg' in r]
        if errors:
            groups[group]['labeled_sample_count'] = len(errors)
            groups[group]['circular_mae_deg'] = float(np.mean(errors))
            # An average direction is not useful for a full-circle reference set.
            groups[group].pop('circular_mean_deg')
        geometric_errors = [r['geometric_difference_deg'] for r in rows
                            if 'geometric_difference_deg' in r]
        if geometric_errors:
            groups[group]['geometric_reference_sample_count'] = len(geometric_errors)
            groups[group]['mean_geometric_difference_deg'] = float(np.mean(geometric_errors))
    report = {
        'model': 'official CFV ConvNextSmallMultiModel / convnext-multi-weak',
        'checkpoint_sha256': sha256(args.checkpoint),
        'runner_sha256': sha256(Path(__file__)),
        'manifest_sha256': sha256(args.manifest),
        'upstream_commit': subprocess.check_output(
            ['git', '-C', str(args.upstream), 'rev-parse', 'HEAD'], text=True).strip(),
        'versions': {'torch': torch.__version__, 'torchvision': torchvision.__version__,
                     'timm': timm.__version__, 'opencv': cv2.__version__},
        'device': 'cpu', 'threads': args.threads,
        'preprocessing': 'Entire supplied image OpenCV INTER_LINEAR resize 224x224; RGB float32 /255; no mean/std normalization. Source crop, if any, recorded in manifest/sample metadata.',
        'input_protocol': {key: manifest[key] for key in
                           ('primary_condition', 'sensitivity_conditions', 'crop_protocol',
                            'source_manifest_sha256', 'boxes_sha256', 'limitations')
                           if key in manifest},
        'angle_convention': 'CFV paper: front=0, right=90, rear=180, left=270. Upstream dataset code comment reverses side names. Numeric bins retained; official labeled image sequence checked separately.',
        'limitations': [
            'No ROS connection, motion commands, or inference of navigation goals.',
            'Softmax probabilities are not calibrated correctness probabilities.',
            'Simulation samples have no exact CFV ground-truth angle; no accuracy claimed for them.',
            'Small saved-image sample cannot establish live temporal stability or generalization.',
            'Full frames include debug overlays if indicated in the manifest.',
            'Public labeled training examples are a wiring check, not held-out model evaluation.',
            'Official preparation re-saves resized JPEGs; this probe avoids lossy recompression of source images.',
            'Geometric references, when present, use the stated synthetic setup center, not CFV manual labels or plate-plane incidence.',
        ],
        'groups': groups, 'samples': results,
    }
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    columns = ['id', 'group', 'angle_deg', 'rear_deviation_deg', 'class_index',
               'class_probability_uncalibrated', 'class_angle_bins_agree',
               'cfv_label_deg', 'circular_error_deg', 'geometric_reference_deg',
               'geometric_difference_deg', 'inference_ms_cpu', 'image']
    with (args.output / 'predictions.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)


if __name__ == '__main__':
    main()
