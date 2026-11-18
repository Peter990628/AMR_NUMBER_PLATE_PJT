#!/usr/bin/env python3
"""Prepare reproducible manual whole-car crop controls; no model or ROS access.

Boxes JSON: {"boxes_xyxy_exclusive": {"sample_id": [x0, y0, x1, y1]}}.
Padding is a fraction of the original box width/height on each respective side.
The primary comparison is fixed at 10%; 0% and 20% are sensitivity controls.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def padded_box(box, size, margin):
    if (len(box) != 4 or any(type(value) is not int for value in box)
            or not math.isfinite(margin) or margin < 0):
        raise ValueError('Expected four integer box coordinates and finite nonnegative padding')
    x0, y0, x1, y1 = box
    width, height = size
    if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
        raise ValueError(f'Box {box} is outside image {size} or empty')
    dx, dy = (x1 - x0) * margin, (y1 - y0) * margin
    return [max(0, math.floor(x0 - dx)), max(0, math.floor(y0 - dy)),
            min(width, math.ceil(x1 + dx)), min(height, math.ceil(y1 + dy))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--boxes', type=Path, required=True)
    parser.add_argument('--image-root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('--output must be a new directory')
    manifest = json.loads(args.manifest.read_text())
    box_spec = json.loads(args.boxes.read_text())
    boxes = box_spec['boxes_xyxy_exclusive']
    samples = manifest['samples']
    if (not samples or len({sample['id'] for sample in samples}) != len(samples)
            or set(boxes) != {sample['id'] for sample in samples}):
        parser.error('Boxes must match all unique source sample IDs exactly')
    # Validate every input before creating any output.
    for sample in samples:
        with Image.open(args.image_root / sample['image']) as image:
            padded_box(boxes[sample['id']], image.size, 0)
    args.output.mkdir(parents=True)
    args.output = args.output.resolve()
    (args.output / 'boxes.json').write_text(args.boxes.read_text())
    result_samples = []
    preview = Image.new('RGB', (960, 330 * math.ceil(len(samples) / 2)), '#222222')
    preview_draw = ImageDraw.Draw(preview)
    for index, sample in enumerate(samples):
        source = (args.image_root / sample['image']).resolve()
        source_id = sample['id']
        with Image.open(source) as opened:
            image = opened.convert('RGB')
        box = boxes[source_id]
        shared = {
            **sample, 'crop_source_id': source_id, 'crop_source_image': str(source),
            'crop_source_sha256': sha256(source), 'manual_vehicle_box_xyxy': box,
            'crop_source_size_wh': list(image.size),
            'note': 'Same saved camera frame; geometric reference from fixture metadata, '
                    'not a CFV manual label. Manual whole-car box frozen before crop inference.',
        }
        result_samples.append({
            **shared, 'id': f'{source_id}_full', 'group': f'{source_id}_full',
            'image': str(source), 'input_condition': 'full_frame',
            'crop_margin_fraction': None, 'crop_xyxy': [0, 0, *image.size],
        })
        for percentage in (0, 10, 20):
            margin = percentage / 100
            crop_box = padded_box(box, image.size, margin)
            condition = f'crop_p{percentage:02d}'
            sample_id = f'{source_id}_{condition}'
            path = args.output / f'{sample_id}.png'
            crop = image.crop(crop_box)
            crop.save(path)
            result_samples.append({
                **shared, 'id': sample_id, 'group': sample_id, 'image': str(path),
                'image_size_wh': list(crop.size), 'input_condition': condition,
                'crop_margin_fraction': margin, 'crop_xyxy': crop_box,
            })
        # Preview original pixels with fixed box overlays; never used by the model.
        annotated = image.copy()
        draw = ImageDraw.Draw(annotated)
        for percentage, color in ((20, '#ffc857'), (10, '#00ffff'), (0, '#ff55ff')):
            x0, y0, x1, y1 = padded_box(box, image.size, percentage / 100)
            draw.rectangle([x0, y0, x1 - 1, y1 - 1], outline=color, width=4)
        annotated.thumbnail((480, 300))
        x, y = (index % 2) * 480, (index // 2) * 330
        preview.paste(annotated, (x, y + 25))
        preview_draw.text((x + 8, y + 6), f'{source_id}: pink 0%, cyan 10%, yellow 20%', fill='white')
    preview.save(args.output / 'boxes_preview.png')
    result = {
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'source_manifest': str(args.manifest.resolve()),
        'source_manifest_sha256': sha256(args.manifest),
        'boxes_sha256': sha256(args.boxes), 'preparer_sha256': sha256(Path(__file__)),
        'primary_condition': 'crop_p10', 'sensitivity_conditions': ['crop_p00', 'crop_p20'],
        'crop_protocol': 'Manual full-vehicle xyxy box, exclusive maximum. '
                         'Padding fraction of box width/height on EACH side; '
                         'floor minimum, ceil maximum, clip to image. No resizing here.',
        'limitations': [
            'Manual ROI diagnostic, not an automatic vehicle detector test.',
            'Cropping changes object scale, background fraction and aspect distortion at 224x224.',
            'Four views of one vehicle cannot establish generalization or temporal stability.',
        ],
        'samples': result_samples,
    }
    (args.output / 'samples.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(f'Prepared {len(result_samples)} inputs: {args.output / "samples.json"}')


if __name__ == '__main__':
    main()
