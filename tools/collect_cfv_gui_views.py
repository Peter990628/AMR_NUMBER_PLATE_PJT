#!/usr/bin/env python3
"""Collect one ROS image per pose of capture_cfv_views_in_gui.py.

This process reads the GUI job handshake and subscribes to images through the
one-shot capture tool. It does not start Isaac, publish ROS commands, or infer.
Start it before running the GUI script; only a newly announced job is accepted.
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
ARTIFACTS = ROOT / 'local_artifacts/cfv_test_2026-11-13'


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wait-start', type=float, default=1200)
    args = parser.parse_args()
    if not 0 < args.wait_start <= 3600:
        parser.error('--wait-start must be in (0, 3600] seconds')
    pointer = ARTIFACTS / 'gui_test_latest.json'
    previous = read_json(pointer)
    print('Waiting for a new GUI capture job; no simulator will be started.', flush=True)
    deadline = time.monotonic() + args.wait_start
    job = None
    while time.monotonic() < deadline:
        current = read_json(pointer)
        if current and current != previous:
            job = Path(current['job_dir']).resolve()
            if job.parent != ARTIFACTS.resolve() or not job.name.startswith('gui_views_'):
                raise RuntimeError('Unexpected GUI job directory')
            break
        time.sleep(0.5)
    if job is None:
        raise RuntimeError('No new GUI job was announced before timeout')
    print(f'Collecting {job}', flush=True)
    samples = []
    completed = set()
    deadline = time.monotonic() + 600
    ready_id = None
    try:
        while time.monotonic() < deadline:
            state = read_json(job / 'state.json') or {}
            phase = state.get('phase')
            if phase in ('complete', 'error'):
                write_json(job / 'samples.json', {
                    'samples': samples,
                    'fixture_state': state,
                    'complete': phase == 'complete' and state.get('restored', False),
                    'limitations': [
                        'Single sedan, one frame per pose; no temporal accuracy claim.',
                        'Car is repositioned in a temporary layer; camera and AMR stay fixed.',
                        'Geometric angles describe the fixture, not CFV human labels.',
                    ],
                })
                if phase == 'error':
                    raise RuntimeError(f'GUI job failed: {state}')
                if not state.get('restored'):
                    raise RuntimeError(f'GUI did not confirm scene restoration: {state}')
                if not samples:
                    raise RuntimeError('GUI completed without captured samples')
                print(f'CAPTURE_COMPLETE {len(samples)} samples; scene restored: {job}', flush=True)
                return
            ready_id = state.get('view_id')
            if phase != 'ready' or ready_id in completed:
                time.sleep(0.5)
                continue
            if not isinstance(ready_id, str) or not ready_id.replace('_', '').isalnum():
                raise RuntimeError('Invalid GUI view identifier')
            output = job / ('capture_' + ready_id)
            command = [sys.executable, str(ROOT / 'tools/capture_cfv_image.py'),
                       '--output', str(output), '--id', ready_id, '--timeout', '25']
            subprocess.run(command, check=True, timeout=35)
            capture = read_json(output / 'samples.json')
            sample = capture['samples'][0]
            sample.update({
                'group': ready_id,
                'note': ('Raw ROS image; controlled vehicle-center geometric reference '
                         'from the separate GUI fixture, not a CFV dataset label.'),
                'geometric_reference_deg': state['geometric_reference_deg'],
                'fixture_metadata': state,
                'display_name': state.get('display_name', ready_id),
            })
            samples.append(sample)
            completed.add(ready_id)
            write_json(job / 'samples_partial.json', {'samples': samples})
            write_json(job / 'ack.json', {
                'view_id': ready_id, 'status': 'captured',
                'capture_manifest': str(output / 'samples.json'),
            })
            print(f'CAPTURED {ready_id} reference={sample["geometric_reference_deg"]:.2f}', flush=True)
        raise RuntimeError('GUI capture session exceeded 600 seconds')
    except BaseException:
        if ready_id:
            write_json(job / 'ack.json', {'view_id': ready_id, 'status': 'abort'})
        raise


if __name__ == '__main__':
    main()
