"""Read USD transforms across ONE user-driven Stop -> Play, without GPU copies.

Run in Isaac Sim Script Editor during Play. Wait for READY (3 wall seconds),
then Stop -> Play once and leave the scene unchanged for 20 wall seconds.
Sampling is at most 10 Hz. Only the last 3 seconds before Stop are saved;
waiting for the restart is limited to 120 seconds and all output to 8 MiB.

These are USD default-time transforms observed after Kit updates, NOT certified
PhysX/Fabric/GPU sensor poses. No timeline, scene, graph or robot commands are
issued. Parent and local transforms help locate a change within the USD chain.
"""

import asyncio
import builtins
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time


PROJECT = Path(__file__).resolve().parents[1]
OUTPUT_ROOT = PROJECT / 'local_artifacts/lidar_input_diagnosis_2026-11-20'
SENSOR = '/iw_hub_ROS/chassis/ObservationLidar/observation_lidar'
PATHS = ('/iw_hub_ROS', '/iw_hub_ROS/chassis',
         '/iw_hub_ROS/chassis/ObservationLidar', SENSOR, '/World/sedan_car')
LOCK = '_peter_sensor_pose_restart_task'
SAMPLE_INTERVAL = .1
PRE_SECONDS = 3.
POST_SECONDS = 20.
WAIT_SECONDS = 120.
MAX_BYTES = 8 * 1024 * 1024
REPORT_RESERVE = 128 * 1024


def encode(value):
    return (json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n').encode()


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def rotation_difference_deg(a, b):
    # trace(A * B^T) is the sum of corresponding elements' products.
    cosine = (sum(a[i][j]*b[i][j] for i in range(3) for j in range(3))-1.)/2.
    return math.degrees(math.acos(max(-1., min(1., cosine))))


def pose_metrics(rows):
    """Drift within ONE epoch/phase only; no odometry or cross-reset subtraction."""
    result = {}
    for path in PATHS:
        valid = [r for r in rows if 'world' in r['poses'].get(path, {})]
        if not valid:
            result[path] = {'sample_count': 0}
            continue
        first, last = valid[0], valid[-1]
        item = {'sample_count': len(valid), 'first_wall_sec': first['wall_sec'],
                'last_wall_sec': last['wall_sec'], 'first_pose': first['poses'][path],
                'last_pose': last['poses'][path]}
        for kind in ('world', 'local'):
            start = first['poses'][path][kind]
            item[kind] = {
                'max_translation_from_first_m': max(distance(start['translation_m'], r['poses'][path][kind]['translation_m']) for r in valid),
                'max_rotation_from_first_deg': max(rotation_difference_deg(start['rotation_matrix_3x3'], r['poses'][path][kind]['rotation_matrix_3x3']) for r in valid),
                'last_translation_from_first_m': distance(start['translation_m'], last['poses'][path][kind]['translation_m']),
                'last_rotation_from_first_deg': rotation_difference_deg(start['rotation_matrix_3x3'], last['poses'][path][kind]['rotation_matrix_3x3']),
            }
        result[path] = item
    return result


class PoseRecorder:
    """Small bounded state machine; deliberately independent of Isaac imports."""

    def __init__(self, output, *, max_bytes=MAX_BYTES):
        self.output = output
        self.max_bytes = max_bytes
        self.saved_bytes = 0
        self.ring = deque()
        self.retained = []
        self.events = []
        self.ready = False
        self.stop_wall = None
        self.play_wall = None
        self.epoch = 0
        self.first_baseline_wall = None
        self.reason = None
        self.observed_count = 0
        self.skipped_by_rate = 0
        self.last_sample_wall = -math.inf

    def write(self, filename, row):
        data = encode(row)
        if self.saved_bytes + len(data) > self.max_bytes - REPORT_RESERVE:
            self.reason = 'size_limit'
            return False
        with (self.output / filename).open('ab') as stream:
            stream.write(data)
        self.saved_bytes += len(data)
        return True

    def flush_ring(self):
        while self.ring and not self.reason:
            row = self.ring.popleft()
            if self.write('poses.jsonl', row):
                self.retained.append(row)

    def note(self, kind, wall, utc_ns, sim):
        if len(self.events) >= 64:
            self.reason = 'event_limit'
            return
        event = dict(kind=kind, wall_sec=wall, wall_time_ns=utc_ns,
                     timeline_seconds=sim, timeline_epoch=self.epoch)
        self.events.append(event)
        self.write('events.jsonl', event)
        if kind == 'stop':
            if self.stop_wall is not None:
                self.reason = 'additional_stop'
            else:
                while self.ring and wall-self.ring[0]['wall_sec'] > PRE_SECONDS+1e-9:
                    self.ring.popleft()
                self.flush_ring()
                self.stop_wall = wall
                self.epoch += 1
                if not self.ready:
                    self.reason = 'stop_before_ready'
        elif kind == 'play' and self.stop_wall is not None and self.play_wall is None:
            self.play_wall = wall
        elif kind == 'pause':
            if self.play_wall is not None:
                self.reason = 'paused_after_restart'
            elif self.stop_wall is None:
                self.first_baseline_wall = None
                self.ready = False

    def tick(self, wall):
        if self.reason:
            return True
        if self.play_wall is not None and wall-self.play_wall >= POST_SECONDS:
            self.reason = 'restart_window_complete'
        elif self.play_wall is None and wall >= WAIT_SECONDS:
            self.reason = 'no_stop_play_within_wait'
        return self.reason is not None

    def due(self, wall):
        # Epsilon prevents float round-off from turning exact 10 Hz into 5 Hz.
        return wall-self.last_sample_wall >= SAMPLE_INTERVAL-1e-9

    def observe(self, row):
        wall = row['wall_sec']
        if not self.due(wall):
            self.skipped_by_rate += 1
            return
        self.last_sample_wall = wall
        self.observed_count += 1
        row['timeline_epoch'] = self.epoch
        row['phase'] = ('after_restart' if self.play_wall is not None else
                        'stopped' if self.stop_wall is not None else 'before_stop')
        if self.stop_wall is None:
            self.ring.append(row)
            while self.ring and wall-self.ring[0]['wall_sec'] > PRE_SECONDS+1e-9:
                self.ring.popleft()
            if row['playing'] and 'world' in row['poses'].get(SENSOR, {}):
                if self.first_baseline_wall is None:
                    self.first_baseline_wall = wall
                self.ready = wall-self.first_baseline_wall >= PRE_SECONDS
            else:
                self.first_baseline_wall = None
                self.ready = False
        elif self.write('poses.jsonl', row):
            self.retained.append(row)

    def summary(self):
        phases = {}
        for phase in ('before_stop', 'stopped', 'after_restart'):
            rows = [r for r in self.retained if r['phase'] == phase]
            if not rows:
                continue
            last = rows[-1]['wall_sec']
            settled = [r for r in rows if last-r['wall_sec'] <= 5.]
            phases[phase] = dict(sample_count=len(rows), drift_from_first=pose_metrics(rows),
                last_5_wall_seconds=pose_metrics(settled),
                last_5_window_complete=last-rows[0]['wall_sec'] >= 5.)
        return dict(finished_reason=self.reason, ready=self.ready, stop_wall_sec=self.stop_wall,
                    play_wall_sec=self.play_wall, observed_sample_count=self.observed_count,
                    saved_sample_count=len(self.retained), stream_bytes=self.saved_bytes,
                    phases=phases,
                    statistics_scope='Only saved phase rows; before_stop is the retained <=3s ring. Last-5s means window duration, not proof of settling.')


def capture_poses(stage, UsdGeom):
    # Each sample re-resolves all prims and creates a new cache: no stale cache
    # survives a STOP/PLAY event. USD default-time values may lag physics/Fabric.
    cache = UsdGeom.XformCache()
    result = {}
    meters = float(UsdGeom.GetStageMetersPerUnit(stage))

    def unpack(matrix):
        rotation = matrix.GetOrthonormalized().ExtractRotationMatrix()
        return dict(translation_m=[float(v)*meters for v in matrix.ExtractTranslation()],
                    rotation_matrix_3x3=[[float(rotation[i][j]) for j in range(3)] for i in range(3)],
                    matrix_4x4_stage_units=[[float(matrix[i][j]) for j in range(4)] for i in range(4)])

    for path in PATHS:
        prim = stage.GetPrimAtPath(path)
        if not prim:
            result[path] = {'error': 'prim_missing'}
            continue
        try:
            xform = UsdGeom.Xformable(prim)
            local = xform.GetLocalTransformation()
            result[path] = dict(parent_path=str(prim.GetParent().GetPath()),
                                resets_xform_stack=bool(xform.GetResetXformStack()),
                                world=unpack(cache.GetLocalToWorldTransform(prim)), local=unpack(local))
        except Exception as exc:
            result[path] = {'error': type(exc).__name__ + ': ' + str(exc)}
    return result


async def capture_restart():
    import omni.kit.app
    import omni.timeline
    import omni.usd
    from pxr import UsdGeom

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage is None or not timeline.is_playing() or not stage.GetPrimAtPath(SENSOR):
        raise RuntimeError('Keep Play active in the failure scene with its observation LiDAR.')
    output = OUTPUT_ROOT / ('gui_pose_restart_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    output.mkdir(parents=True, exist_ok=False)
    rec = PoseRecorder(output)
    report = dict(status='preparing', scope=__doc__, output=str(output),
                  stage_identifier=stage.GetRootLayer().identifier,
                  started_utc=datetime.now(timezone.utc).isoformat(), paths=list(PATHS),
                  stage_meters_per_unit=float(UsdGeom.GetStageMetersPerUnit(stage)),
                  transform_time_code='USD default', matrix_convention='USD/Gf row-vector, translation in row 3',
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  scene_changes=False, motion_commands=False, timeline_commands=False,
                  sample_rate_limit_hz=10., pre_stop_seconds=PRE_SECONDS, post_play_seconds=POST_SECONDS,
                  wait_limit_seconds=WAIT_SECONDS, max_saved_bytes=MAX_BYTES)

    def save_report():
        data = encode(report)
        if len(data) > REPORT_RESERVE:
            raise RuntimeError('Report exceeds reserved storage budget')
        (output / 'report.json').write_bytes(data)

    save_report()
    start = time.monotonic()
    events = deque(maxlen=64)
    overflow = False
    names = {int(getattr(omni.timeline.TimelineEventType, n)): n.lower() for n in ('STOP', 'PLAY', 'PAUSE')}

    def on_event(event):
        nonlocal overflow
        if event.type in names:
            if len(events) == events.maxlen:
                overflow = True
            else:
                events.append((names[event.type], time.monotonic()-start,
                               time.time_ns(), timeline.get_current_time()))

    subscription = timeline.get_timeline_event_stream().create_subscription_to_pop(on_event, name='PeterSensorPoseRestartReader')
    announced = False
    next_report = 5.
    try:
        print('Pose recorder: preparing 3-second baseline. Wait for READY. ' + str(output))
        while True:
            try:
                await asyncio.wait_for(app.post_update_async(), timeout=1.)
                updated = True
            except asyncio.TimeoutError:
                updated = False
            wall = time.monotonic()-start
            if context.get_stage() != stage:
                raise RuntimeError('Stage changed; only Stop -> Play in the same stage is supported.')
            if overflow:
                raise RuntimeError('Timeline event queue overflow')
            while events:
                kind, event_wall, utc_ns, sim = events.popleft()
                rec.note(kind, event_wall, utc_ns, sim)
                if kind == 'stop':
                    print('Pose recorder: STOP recorded.')
                elif kind == 'play' and rec.play_wall is not None:
                    print('Pose recorder: PLAY recorded; recording for 20 wall seconds.')
            if rec.tick(wall):
                break
            if updated and rec.due(wall):
                rec.observe(dict(wall_sec=wall, wall_time_ns=time.time_ns(),
                    timeline_seconds=float(timeline.get_current_time()), playing=bool(timeline.is_playing()),
                    poses=capture_poses(stage, UsdGeom)))
            if rec.ready and not announced:
                announced = True
                print('READY: press Stop, then Play ONCE. Keep the AMR and vehicle unchanged.')
            if wall >= next_report:
                report.update(status='after_restart' if rec.play_wall is not None else
                              'waiting_for_stop_play' if rec.ready else 'collecting_baseline',
                              capture_wall_sec=wall, observed_sample_count=rec.observed_count)
                save_report()
                next_report = wall+5.
        report['status'] = rec.reason
    except BaseException as exc:
        report.update(status='error', error=type(exc).__name__ + ': ' + str(exc))
    finally:
        subscription.unsubscribe()
        # Preserve the final pre-restart ring even if the user never restarted.
        if rec.stop_wall is None:
            reason, rec.reason = rec.reason, None
            rec.flush_ring()
            rec.reason = rec.reason or reason
        report.update(rec.summary(), capture_wall_sec=time.monotonic()-start,
                      finished_utc=datetime.now(timezone.utc).isoformat())
        save_report()
        print('Pose recorder finished: ' + str(report['status']) + '. ' + str(output))


if __name__ in ('__main__', '<run_path>'):
    existing = getattr(builtins, LOCK, None)
    if existing is not None and not existing.done():
        raise RuntimeError('A sensor pose restart recorder is already running.')
    setattr(builtins, LOCK, asyncio.ensure_future(capture_restart()))
