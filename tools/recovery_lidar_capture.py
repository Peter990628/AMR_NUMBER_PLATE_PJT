"""Rendezvous between a local recovery run and its in-process Isaac LiDAR reader."""
import fcntl
import json
from pathlib import Path
import time
from uuid import uuid4


REQUESTS = Path(__file__).resolve().parents[1] / 'local_artifacts/.lidar_requests'
SENSORS = {'create3': '/create3/base_link/observation_lidar/Scanner',
           'iw_hub': '/iw_hub_ROS/chassis/ObservationLidar/observation_lidar'}


def active_request(ticket):
    """The launcher owns an exclusive lock until the run ends, also released on crash."""
    try:
        with ticket.open() as stream:
            try:
                fcntl.flock(stream, fcntl.LOCK_SH | fcntl.LOCK_NB)
            except BlockingIOError:
                return True
    except FileNotFoundError:
        pass
    return False


class RunLidarCapture:
    def __init__(self, run, robot, requests=REQUESTS):
        self.directory = run.resolve() / 'lidar_internal'
        self.directory.mkdir()
        requests.mkdir(parents=True, exist_ok=True)
        self.ticket = requests / (uuid4().hex + '.json')
        manifest = self.directory / 'request.json'
        self.owner = manifest.open('x')
        try:
            fcntl.flock(self.owner, fcntl.LOCK_EX)
            json.dump({'robot': robot}, self.owner)
            self.owner.flush()
            self.ticket.symlink_to(manifest)
        except BaseException:
            self.owner.close()
            raise

    def report(self):
        try:
            return json.loads((self.directory / 'report.json').read_text())
        except FileNotFoundError:
            return {}

    def finish(self):
        self.owner.close()
        self.ticket.unlink(missing_ok=True)
        report = self.report()
        if (self.directory / 'claimed').exists():
            # Keep the ROS bag running during an already triggered post-event window.
            deadline = time.monotonic() + 7
            while 'finished_utc' not in report and time.monotonic() < deadline:
                time.sleep(.1)
                report = self.report()
        if 'finished_utc' not in report:
            status = 'reader_unavailable' if not (self.directory / 'claimed').exists() else 'reader_unfinished'
        else:
            status = report['status']
        return {'status': status, 'directory': str(self.directory),
                'trigger': report.get('trigger'), 'samples': report.get('saved_sample_count', 0),
                'error': report.get('error')}
