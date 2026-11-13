#!/usr/bin/env python3
"""Export a local Ultralytics detector for C++ inference; never auto-install packages.

Offline preparation only. Runtime perception and quality evaluation are C++.
Install export dependencies separately; see docs/stage3_plate_observation.md.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'assets/models')
    args = parser.parse_args()
    source = args.weights.resolve(strict=True)
    destination = args.output_dir.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    local_weights = destination / source.name
    if source != local_weights:
        shutil.copyfile(source, local_weights)
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    os.environ['YOLO_OFFLINE'] = 'true'
    os.environ['CUDA_VISIBLE_DEVICES'] = ''
    with tempfile.TemporaryDirectory(prefix='amr_plate_export_') as temporary:
        os.environ['YOLO_CONFIG_DIR'] = temporary
        os.environ['MPLCONFIGDIR'] = temporary
        import torch
        import ultralytics
        from ultralytics import YOLO
        import onnx
        torch.set_num_threads(2)
        model = YOLO(str(local_weights), task='detect')
        labels = model.names
        plate_ids = [key for key, value in labels.items() if value == 'Plate']
        if len(plate_ids) != 1:
            raise ValueError(f'Expected exactly one Plate class, got {labels}')
        exported = Path(model.export(format='onnx', imgsz=640, batch=1, device='cpu',
                                     opset=12, dynamic=False, simplify=True, nms=False,
                                     half=False))
        onnx.checker.check_model(str(exported))
        manifest = {
            'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'onnx_sha256': hashlib.sha256(exported.read_bytes()).hexdigest(),
            'classes': labels, 'plate_class_id': plate_ids[0], 'num_classes': len(labels),
            'input_size': 640, 'opset': 12, 'dynamic': False, 'nms_in_graph': False,
            'ultralytics_version': ultralytics.__version__, 'torch_version': torch.__version__,
            'onnx_version': onnx.__version__,
            'preprocessing': 'RGB float32 /255, centered square letterbox fill=114',
            'output': 'batch=1; channels=4+num_classes; xywh pixels and class scores; no objectness',
        }
        exported.with_suffix('.json').write_text(json.dumps(manifest, indent=2) + '\n')
        print(f'Exported: {exported}')
        print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
