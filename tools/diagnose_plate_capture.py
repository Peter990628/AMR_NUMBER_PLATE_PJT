#!/usr/bin/env python3
"""Read-only independent witness of a fixed-window plate capture.

Records source headers, not image pixels, while a separate capture process saves
PNGs. A missing result is never labelled a detector skip without evidence.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from plate_evaluation import source_key


def diagnose(capture, records):
    def keys(kind):
        return {source_key(r['header']) for r in records if r['kind'] == kind
                and r['header']['frame_id']}
    observations, debug = keys('observation'), keys('debug')
    missing = []
    for s in capture['samples']:
        if not s.get('image') or s.get('observation') is not None:
            continue
        key = source_key(s['image_header'])
        if key in observations:
            reason = 'result_seen_by_independent_witness'
        elif key in debug:
            reason = 'debug_seen_but_result_not_seen'
        else:
            reason = 'no_result_seen_by_either_subscriber'
        missing.append({'slot': s['slot'], 'source_header': s['image_header'], 'evidence': reason})
    return {'saved_images': sum(bool(s.get('image')) for s in capture['samples']),
            'matched_results': sum(s.get('observation') is not None for s in capture['samples']),
            'missing': missing, 'missing_evidence_counts': dict(Counter(m['evidence'] for m in missing)),
            'witness_message_counts': dict(Counter(r['kind'] for r in records)),
            'witness_unique_headers': {k: len(keys(k)) for k in ('raw', 'observation', 'debug')},
            'parameters_recorded': bool(capture.get('detector_parameters')),
            'capture_fault': capture.get('capture_fault'),
            'limitation': 'No result on both subscribers cannot distinguish pre-inference queue drops, sensor transport loss, or loss on both output streams.'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=10.)
    p.add_argument('--sampling-mode', choices=['first_raw', 'first_pair'], default='first_raw')
    args = p.parse_args()
    if args.output.exists() or not 1 <= args.seconds <= 60:
        p.error('Use a new output directory and duration 1..60 seconds')
    out = args.output.resolve(); out.mkdir(parents=True)
    os.environ.setdefault('ROS_LOG_DIR', str(out/'ros_log'))
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import Image
    from amr_reobservation.msg import PlateObservation
    rclpy.init(args=[])
    node = rclpy.create_node('plate_capture_independent_witness')
    start = time.monotonic(); records = []; child = None
    def callback(msg, kind):
        if len(records) >= 20000:
            raise RuntimeError('Diagnostic metadata bound exceeded')
        record = {'kind': kind, 'receipt_sec': time.monotonic()-start,
                  'header': message_to_ordereddict(msg.header)}
        if kind == 'observation':
            record.update(status=msg.status, input_valid=msg.input_valid, processing_ms=msg.processing_ms)
        records.append(record)
    subs = [node.create_subscription(typ, topic, lambda m, k=kind: callback(m, k), qos)
            for typ, topic, kind, qos in (
                (Image, '/front_stereo_camera/left/image_raw', 'raw', qos_profile_sensor_data),
                (PlateObservation, '/plate/observation', 'observation', 200),
                (Image, '/debug/plate_observation', 'debug', qos_profile_sensor_data))]
    try:
        end = time.monotonic()+2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        with (out/'capture.log').open('w') as log:
            child = subprocess.Popen([sys.executable, str(Path(__file__).with_name('capture_plate_evaluation.py')),
                '--trial-id', 'stationary_diagnostic', '--phase', 'before', '--seconds', str(args.seconds),
                '--sampling-mode', args.sampling_mode,
                '--output', str(out/'capture')], stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+args.seconds+18
            while child.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
            if child.poll() is None:
                raise RuntimeError('Capture deadline exceeded')
        end = time.monotonic()+2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
        capture = json.loads((out/'capture/capture.json').read_text())
        result = diagnose(capture, records)
        result.update(created_at_utc=datetime.now(timezone.utc).isoformat(),
                      capture_exit_code=child.returncode,
                      cmd_vel_publishers=[p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')])
        (out/'witness.json').write_text(json.dumps(records, indent=2)+'\n')
        (out/'diagnosis.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
        return child.returncode
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.terminate(); child.wait(timeout=3)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
