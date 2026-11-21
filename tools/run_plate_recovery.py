#!/usr/bin/env python3
"""One-command, one-candidate Isaac recovery ending on a truth-free OCR decision.

No --execute: check local dependencies and print configuration; no ROS or motion.
--execute: own detector/preview/controller lifetimes, hold final pose, return JSON.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from recovery_ocr_session import ROOT, stop_process
from recovery_trial_math import MOTION_SPEEDS


def defaults():
    base = ROOT/'local_artifacts'
    return dict(ros_python='/usr/bin/python3',
        ocr_python=str(base/'paddle_ocr_validation_2026-11-19/venv/bin/python'),
        ocr_baseline=str(base/'paddle_ocr_validation_2026-11-19/whole_box_01/report.json'),
        cfv_upstream=str(base/'cfv_test_2026-11-13/upstream'),
        cfv_checkpoint=str(base/'cfv_test_2026-11-13/convnext-multi-weak.pth'),
        cfv_deps=str(base/'cfv_test_2026-11-13/python_deps'))


def graph_preflight(mock):
    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node('single_recovery_preflight')
    try:
        until = time.monotonic()+2
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        topics = ['/cmd_vel', '/plate/observation', '/reobservation/preview']
        if mock: topics.append('/reobservation/request')
        occupied = {t:[p.node_name for p in node.get_publishers_info_by_topic(t)] for t in topics}
        if any(occupied.values()):
            raise RuntimeError('existing_publishers:'+json.dumps(occupied))
    finally:
        node.destroy_node(); rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        help='New run directory (default: dated folder in local_artifacts/recovery_runs)')
    parser.add_argument('--record-sensors', action='store_true',
                        help='Record LiDAR, odometry, clock and TF to sensors/ during the trial')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--speed', choices=MOTION_SPEEDS, default='normal',
                        help='Driving speed for Isaac Sim recovery')
    probes = parser.add_mutually_exclusive_group()
    probes.add_argument('--corner-probe-once', action='store_true',
                        help='Try one observed corner viewpoint if ordinary end-face selection times out')
    probes.add_argument('--corner-probe-plus-30cm', action='store_true',
                        help='Compare a side-only viewpoint 0.65 m beyond the observed corner')
    parser.add_argument('--mock-params', type=Path,
                        help='Explicit manual vehicle-hint fixture; otherwise require upstream RecoveryRequest')
    parser.add_argument('--preview-params', type=Path,
                        default=ROOT/'ros2_ws/src/amr_reobservation/config/recovery_preview.yaml')
    parser.add_argument('--model-path', type=Path, default=ROOT/'assets/models/semi_allimages_v5n.onnx')
    parser.add_argument('--config', type=Path, help='JSON overriding local model/interpreter paths')
    args = parser.parse_args()
    config = defaults()
    if args.config:
        overrides = json.loads(args.config.read_text())
        if set(overrides)-set(config): parser.error('Unknown dependency config keys')
        config.update(overrides)
    # Preserve venv interpreter symlinks: resolving bin/python to /usr/bin/python
    # would silently discard the environment containing PaddleOCR.
    config = {k:str(Path(v).absolute()) for k,v in config.items()}
    for path in [*config.values(), args.model_path, args.preview_params, *([args.mock_params] if args.mock_params else [])]:
        if not Path(path).exists(): parser.error('Missing local dependency: '+str(path))
    if args.output and args.output.exists(): parser.error('Use a new output directory')
    if not args.execute:
        print(json.dumps({'status':'configuration_checked', 'motion_enabled':False,
                          'context':'explicit_mock' if args.mock_params else 'upstream_request',
                          'mock_params':str(args.mock_params) if args.mock_params else None,
                          'record_sensors':args.record_sensors, 'corner_probe_once':args.corner_probe_once,
                          'corner_probe_plus_30cm':args.corner_probe_plus_30cm,
                          'speed':args.speed, 'motion_limits':MOTION_SPEEDS[args.speed]._asdict(),
                          'dependencies':config}, indent=2))
        return 0
    if args.output:
        output = args.output.resolve()
        output.mkdir(parents=True)
    else:
        runs = ROOT/'local_artifacts/recovery_runs'
        runs.mkdir(parents=True, exist_ok=True)
        output = Path(tempfile.mkdtemp(prefix=datetime.now().strftime('%Y%m%d_%H%M%S_'), dir=runs))
    print(f'실험 기록 폴더: {output}', file=sys.stderr, flush=True)
    os.environ.setdefault('ROS_LOG_DIR', str(output/'ros_logs'))
    (output/'config.json').write_text(json.dumps(config, indent=2)+'\n')
    owned = []; logs = []
    final_report = {}
    result = {'status':'aborted', 'success':False, 'text':None, 'candidate_attempts':0,
              'context':'explicit_mock' if args.mock_params else 'upstream_request', 'speed':args.speed,
              'corner_probe_once':args.corner_probe_once, 'corner_probe_plus_30cm':args.corner_probe_plus_30cm}
    def spawn(argv, name):
        stream = (output/(name+'.log')).open('w'); logs.append(stream)
        process = subprocess.Popen([str(a) for a in argv], cwd=ROOT, stdout=stream,
                                   stdin=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
        owned.append((name, process)); return process
    def interrupt(signum, frame):
        raise KeyboardInterrupt(signal.Signals(signum).name)
    previous = {s:signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        graph_preflight(bool(args.mock_params))
        import yaml
        preview = yaml.safe_load(args.preview_params.read_text())
        preview['recovery_preview']['ros__parameters']['request_timeout_sec'] = 600.
        (output/'preview.yaml').write_text(yaml.safe_dump(preview))
        mock_args = ['mock:=false']
        if args.mock_params:
            fixture = yaml.safe_load(args.mock_params.read_text())
            fixture['mock_recovery_request']['ros__parameters'].update(valid_for_sec=600., max_wall_age_sec=600.)
            (output/'mock.yaml').write_text(yaml.safe_dump(fixture))
            mock_args = ['mock:=true', 'mock_params_file:='+str(output/'mock.yaml')]
        if args.record_sensors:
            # The motion guard requires Isaac to be the only /cmd_vel subscriber.
            # The controller already writes its command history to events.jsonl.
            spawn(['ros2', 'bag', 'record', '-o', output/'sensors',
                   '/observation_lidar/scan', '/front_2d_lidar/scan', '/back_2d_lidar/scan',
                   '/chassis/odom', '/clock', '/tf', '/tf_static'], 'sensor_recorder')
        spawn(['ros2', 'launch', 'amr_reobservation', 'plate_observation.launch.py',
               'model_path:='+str(args.model_path.resolve())], 'detector')
        spawn(['ros2', 'launch', 'amr_reobservation', 'recovery_preview.launch.py', 'rviz:=false',
               'params_file:='+str(output/'preview.yaml'), *mock_args], 'preview')
        runner = spawn([config['ros_python'], ROOT/'tools/run_recovery_trial.py',
                        '--single-ocr-config', output/'config.json', '--execute', '--speed', args.speed,
                        *(['--corner-probe-once'] if args.corner_probe_once else []),
                        *(['--corner-probe-plus-30cm'] if args.corner_probe_plus_30cm else []),
                        '--output', output/'controller'], 'controller')
        deadline = time.monotonic()+560
        last_stage = None
        while runner.poll() is None:
            for name, process in owned:
                if name != 'controller' and process.poll() is not None:
                    raise RuntimeError(f'companion_node_exited:{name}')
            if time.monotonic() >= deadline: raise RuntimeError('session_timeout')
            report_path = output/'controller/report.json'
            if report_path.exists():
                try:
                    report = json.loads(report_path.read_text())
                    stage = report.get('stage', report.get('status'))
                    if stage != last_stage:
                        print(json.dumps({'stage':stage}), flush=True); last_stage = stage
                except json.JSONDecodeError:
                    pass  # controller may be replacing its progress report
            time.sleep(.1)
        final_report = json.loads((output/'controller/report.json').read_text())
        result.update({k:final_report.get(k) for k in ('status','success','text','error')})
        if runner.returncode not in (0,2) or final_report.get('guard_exit_code') != 0:
            result.update(status='aborted', success=False, text=None,
                          error=final_report.get('error') or 'controller_or_guard_failed')
    except (Exception, KeyboardInterrupt) as error:
        result.update(status='aborted', success=False, text=None, error=str(error) or type(error).__name__)
    finally:
        cleanup = {}
        for name, process in reversed(owned):
            try:
                stop_process(process)
                cleanup[name] = process.poll()
            except Exception as error:
                cleanup[name] = str(error)
                result.update(status='aborted', success=False, text=None, error='cleanup_failed')
        result['owned_process_exit_codes'] = cleanup
        # On Ctrl+C the controller writes its last pose and completed OCR phases
        # while it stops. Keep those measurements with the launcher's stop reason.
        report_path = output/'controller/report.json'
        if not final_report and report_path.is_file():
            try:
                final_report = json.loads(report_path.read_text())
            except (OSError, ValueError):
                pass  # The owner may have stopped before the controller finished its report.
        for key in ('candidate_attempts', 'request', 'initial_pose', 'final_pose',
                    'final_measured_twist', 'guard_exit_code', 'before_ocr', 'after_ocr',
                    'elapsed_wall_sec'):
            result[key] = final_report.get(key, result.get(key))
        if args.record_sensors:
            result['sensor_bag_finalized'] = (output/'sensors/metadata.yaml').is_file()
            if 'sensor_recorder' in cleanup and not result['sensor_bag_finalized']:
                print(f"센서 기록 마무리 실패. 로그: {output/'sensor_recorder.log'}",
                      file=sys.stderr, flush=True)
        for stream in logs: stream.close()
        for number, handler in previous.items(): signal.signal(number, handler)
        (output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(result, ensure_ascii=False), flush=True)
        print(f"결과: {result['status']} | 번호판: {result['text'] or '—'} | {output/'result.json'}",
              file=sys.stderr, flush=True)
    if result['status']=='aborted' or (args.record_sensors and not result['sensor_bag_finalized']):
        return 1
    return 0 if result['success'] else 2


if __name__ == '__main__':
    sys.exit(main())
