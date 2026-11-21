"""Bounded child jobs used while the controller holds zero and checks health."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def stop_process(process):
    """Terminate only this job's process group, including helper descendants."""
    if process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass
    raise RuntimeError('child_cleanup_failed')


def child_job(argv, log, timeout, pump):
    with Path(log).open('w') as stream:
        process = subprocess.Popen([str(a) for a in argv], cwd=ROOT,
                                   stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+timeout
        try:
            while process.poll() is None:
                pump()  # zero command, spin, request/clock/sensor/stationarity checks
                if time.monotonic() >= deadline:
                    raise RuntimeError('child_timeout:'+Path(log).name)
            pump()
            if process.returncode:
                raise RuntimeError('child_failed:'+Path(log).name+':'+str(process.returncode))
        finally:
            stop_process(process)


class RecoveryOCRSession:
    def __init__(self, config_path, output, *, robot='iw_hub'):
        self.robot = robot
        self.config = json.loads(Path(config_path).read_text())
        self.output = Path(output)
        self.parameters = None
        for key in ('ros_python', 'ocr_python', 'ocr_baseline', 'cfv_upstream', 'cfv_checkpoint', 'cfv_deps'):
            if not Path(self.config[key]).exists():
                raise ValueError('missing_dependency:'+key)

    def observe(self, phase, pump):
        c = self.config
        capture_dir = self.output/(phase+'_capture')
        child_job([c['ros_python'], ROOT/'tools/capture_plate_evaluation.py',
                   '--trial-id', 'single_recovery', '--phase', phase, '--sampling-mode', 'numbered',
                   '--seconds', '4', '--hz', '2', '--output', capture_dir],
                  self.output/(phase+'_capture.log'), 20, pump)
        capture_path = capture_dir/'capture.json'
        capture = json.loads(capture_path.read_text())
        if capture['capture_fault'] or not capture['clock_advanced'] or not capture['detector_parameters']:
            raise RuntimeError('invalid_capture:'+phase)
        if self.parameters is not None and capture['detector_parameters'] != self.parameters:
            raise RuntimeError('detector_parameters_changed')
        self.parameters = capture['detector_parameters']
        destination = self.output/(phase+'_ocr')
        child_job([c['ros_python'], ROOT/'tools/inspect_plate_window.py', '--capture', capture_path,
                   '--output', destination],
                  self.output/(phase+'_ocr.log'), 45, pump)
        result = json.loads((destination/'report.json').read_text())
        return {**result['decision'], 'report': str(destination/'report.json')}

    def prepare(self, pump):
        c = self.config
        start = time.monotonic()
        capture = self.output/'cfv_capture'
        # Low real-time factor and intermittent confirmed contours can delay the
        # third pair. Keep all association gates, but stop as soon as it arrives.
        child_job([c['ros_python'], ROOT/'tools/capture_recovery_snapshot.py', '--seconds', '30',
                   '--robot', self.robot, '--samples', '3', '--stop-when-complete', '--output', capture],
                  self.output/'cfv_capture.log', 35, pump)
        inference = self.output/'cfv_inference'
        child_job(['systemd-run', '--user', '--scope', '-p', 'MemoryMax=2G', '-p', 'MemorySwapMax=0',
                   c['ros_python'], ROOT/'tools/probe_cfv_orientation.py', '--upstream', c['cfv_upstream'],
                   '--checkpoint', c['cfv_checkpoint'], '--deps', c['cfv_deps'],
                   '--manifest', capture/'samples.json', '--threads', '2', '--output', inference],
                   self.output/'cfv_inference.log', max(1, 60-(time.monotonic()-start)), pump)
        if time.monotonic()-start > 60:
            raise RuntimeError('cfv_preparation_timeout')
        return inference/'report.json'
