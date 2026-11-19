#!/usr/bin/env python3
"""One-command, one-candidate Isaac recovery ending on a truth-free OCR decision.

No --execute: check local dependencies and print configuration; no ROS or motion.
--execute: own detector/preview/controller lifetimes, hold final pose, return JSON.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from recovery_ocr_session import ROOT, stop_process


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
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--execute', action='store_true')
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
    if args.output.exists(): parser.error('Use a new output directory')
    if not args.execute:
        print(json.dumps({'status':'configuration_checked', 'motion_enabled':False,
                          'context':'explicit_mock' if args.mock_params else 'upstream_request',
                          'dependencies':config}, indent=2))
        return 0
    output = args.output.resolve(); output.mkdir(parents=True)
    (output/'config.json').write_text(json.dumps(config, indent=2)+'\n')
    owned = []; logs = []
    result = {'status':'aborted', 'success':False, 'text':None, 'candidate_attempts':0,
              'context':'explicit_mock' if args.mock_params else 'upstream_request'}
    def spawn(argv, name):
        stream = (output/(name+'.log')).open('w'); logs.append(stream)
        process = subprocess.Popen([str(a) for a in argv], cwd=ROOT, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
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
        spawn(['ros2', 'launch', 'amr_reobservation', 'plate_observation.launch.py',
               'model_path:='+str(args.model_path.resolve())], 'detector')
        spawn(['ros2', 'launch', 'amr_reobservation', 'recovery_preview.launch.py', 'rviz:=false',
               'params_file:='+str(output/'preview.yaml'), *mock_args], 'preview')
        runner = spawn([config['ros_python'], ROOT/'tools/run_recovery_trial.py',
                        '--single-ocr-config', output/'config.json', '--execute',
                        '--output', output/'controller'], 'controller')
        deadline = time.monotonic()+560
        last_stage = None
        while runner.poll() is None:
            if any(p.poll() is not None for name,p in owned if name!='controller'):
                raise RuntimeError('companion_node_exited')
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
        report = json.loads((output/'controller/report.json').read_text())
        result.update({k:report.get(k) for k in ('status','success','text','candidate_attempts','error',
                      'request','initial_pose','final_pose','final_measured_twist','guard_exit_code',
                      'before_ocr','after_ocr','elapsed_wall_sec')})
        if runner.returncode not in (0,2) or report.get('guard_exit_code') != 0:
            result.update(status='aborted', success=False, text=None,
                          error=report.get('error') or 'controller_or_guard_failed')
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
        for stream in logs: stream.close()
        for number, handler in previous.items(): signal.signal(number, handler)
        (output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result['success'] else 1 if result['status']=='aborted' else 2


if __name__ == '__main__':
    sys.exit(main())
