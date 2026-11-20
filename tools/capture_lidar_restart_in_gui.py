"""Record the existing observation LiDAR across one USER-driven Stop -> Play.

Run in Isaac Sim Script Editor with Play active. Wait for READY, then press
Stop and Play once. Retains up to 3 seconds before Stop and 60 wall seconds
after Play. Waits at most 120 seconds for the restart; stream storage <=96 MiB
and the in-memory ring <=16 MiB. Does not edit USD/graphs/settings, attach
annotators, control the timeline, or issue robot commands.

GPU copies are post-update observations, not atomic snapshots of the pipeline.
Node paths, graph generations, timestamps and raw metadata permit alignment.
"""

import asyncio
import builtins
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import runpy
import time


BASE = Path(__file__).resolve().parent
UTILS = runpy.run_path(str(BASE / 'inspect_lidar_pipeline_in_gui.py'), run_name='lidar_reader_utils')
serializable, write_json = UTILS['serializable'], UTILS['write_json']
RestartRecorder = runpy.run_path(str(BASE / 'lidar_restart_recorder.py'))['RestartRecorder']
LOCK = UTILS['LOCK']  # Prevent overlap with the earlier pipeline/dropout reader.
SENSOR, FRAME, TOPIC = UTILS['SENSOR'], UTILS['FRAME'], UTILS['TOPIC']


def discover_pipeline(og):
    def read(attr):
        try:
            if str(attr.get_resolved_type()) == 'execution':
                return {'omitted': 'execution_attribute'}
            return serializable(og.Controller.get(attr))
        except Exception as exc:
            return {'read_error': type(exc).__name__ + ': ' + str(exc)}

    def named(node, name):
        attr = node.get_attribute(name)
        return read(attr) if attr.is_valid() else None

    nodes = [n for g in og.get_all_graphs() for n in g.get_nodes()]
    publishers = [n for n in nodes if n.get_type_name().endswith('.ROS2PublishLaserScan')
                  and (named(n, 'inputs:frameId') == FRAME or
                       named(n, 'inputs:topicName') in (TOPIC, TOPIC.lstrip('/')))]
    if len(publishers) != 1:
        raise RuntimeError(f'Expected one observation publisher, found {len(publishers)}')
    pending, selected, connections = list(publishers), {}, []
    while pending:
        node = pending.pop()
        path = node.get_prim_path()
        if path in selected:
            continue
        if len(selected) >= 80:
            raise RuntimeError('Upstream graph exceeded 80 nodes')
        selected[path] = node
        for attr in node.get_attributes():
            for upstream in attr.get_upstream_connections():
                parent = upstream.get_node()
                connections.append(dict(source_node=parent.get_prim_path(), source_attribute=upstream.get_name(),
                                        target_node=path, target_attribute=attr.get_name()))
                pending.append(parent)
    dynamic, inventory, specs = [], [], []
    for path, node in selected.items():
        typ = node.get_type_name()
        attrs = list(node.get_attributes())
        watched = []
        for attr in attrs:
            name = attr.get_name()
            lower = name.lower()
            if any(x in lower for x in ('intensitiesdata', 'exec', 'context')):
                continue
            if name.startswith('outputs:') or any(x in lower for x in
                    ('lineardepthdata', 'timestamp', 'renderproduct', 'cuda', 'ptr',
                     'buffersize', 'rotationrate', 'numcols', 'numrows')):
                watched.append(attr)
        dynamic.append((path, typ, watched))
        inventory.append(dict(path=path, type=typ, attributes={a.get_name(): read(a) for a in attrs}))
        if typ.endswith('.SdRenderVarPtr') and named(node, 'inputs:renderVar') == 'GenericModelOutput':
            specs.append(('raw_gmo', node, 'outputs:dataPtr', 'outputs:bufferSize', 'uint8'))
        if typ.endswith('.IsaacCreateRTXLidarScanBuffer'):
            for field in ('azimuth', 'distance'):
                specs.append(('accumulator_' + field, node, 'outputs:' + field + 'Ptr',
                              'outputs:' + field + 'BufferSize', 'float32'))
    if sorted(s[0] for s in specs) != ['accumulator_azimuth', 'accumulator_distance', 'raw_gmo']:
        raise RuntimeError('GMO/accumulator chain not ready or ambiguous')
    if sum(t.endswith('.IsaacComputeRTXLidarFlatScan') for _, t, _ in dynamic) != 1:
        raise RuntimeError('Expected one observation FlatScan node')
    return dict(read=read, named=named, dynamic=dynamic, specs=specs,
                inventory=dict(nodes=inventory, connections=connections))


def sample_pipeline(binding, wp, copy_budget):
    values = [dict(path=path, type=typ, values={a.get_name(): binding['read'](a) for a in attrs})
              for path, typ, attrs in binding['dynamic']]
    metadata, blobs = {}, {}
    for label, node, ptr_name, size_name, dtype in binding['specs']:
        named = binding['named']
        entry = dict(node=node.get_prim_path(), dtype=dtype)
        metadata[label] = entry
        try:
            ptr, size = int(named(node, ptr_name)), int(named(node, size_name))
            device_id = int(named(node, 'outputs:cudaDeviceIndex'))
            entry.update(pointer=ptr, byte_count=size, device=device_id)
        except (TypeError, ValueError) as exc:
            entry.update(status='not_ready', error=str(exc))
            continue
        if ptr == 0 or size == 0:
            entry['status'] = 'empty'
        elif device_id < 0:
            entry['status'] = 'cpu_pointer_not_copied'
        elif not 0 < size <= 2 * 1024 * 1024:
            raise RuntimeError('Unexpected GPU buffer size: ' + str(entry))
        elif size > copy_budget:
            entry['status'] = 'storage_budget_not_copied'
        else:
            device = f'cuda:{device_id}'
            wp.synchronize_device(device)
            # Recheck after synchronization; never use addresses cached across a restart.
            if ptr != int(named(node, ptr_name)) or size != int(named(node, size_name)):
                entry['status'] = 'changed_before_copy'
                continue
            view = wp.array(ptr=ptr, dtype=wp.uint8, shape=(size,), device=device, requires_grad=False)
            data = view.numpy().copy().tobytes()
            entry.update(status='copied', sha256=hashlib.sha256(data).hexdigest(),
                         source_pointer_unchanged=ptr == int(named(node, ptr_name)),
                         source_size_unchanged=size == int(named(node, size_name)))
            blobs[label] = data
            copy_budget -= size
    return values, metadata, blobs


async def capture_restart():
    import omni.graph.core as og
    import omni.kit.app
    import omni.timeline
    import omni.usd
    import warp as wp
    from pxr import UsdGeom

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage is None or not timeline.is_playing() or not stage.GetPrimAtPath(SENSOR):
        raise RuntimeError('Open the failure scene with its observation LiDAR and keep Play active.')
    output = UTILS['OUTPUT_ROOT'] / ('gui_restart_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    output.mkdir(parents=True, exist_ok=False)
    rec = RestartRecorder(output)
    report = dict(status='preparing', output=str(output), scene_changes=False, motion_commands=False,
                  annotators_attached=False, stage_identifier=stage.GetRootLayer().identifier,
                  sensor_path=SENSOR, topic=TOPIC, frame_id=FRAME, pre_stop_sec=3., post_play_sec=60.,
                  restart_wait_limit_sec=120., total_limit_sec=180., ring_limit_bytes=16 * 1024 * 1024,
                  sample_count=0, binding_generation=0, binding_failures=0, scope=__doc__,
                  started_utc=datetime.now(timezone.utc).isoformat())
    for name in (Path(__file__).name, 'lidar_restart_recorder.py', 'inspect_lidar_pipeline_in_gui.py'):
        data = (BASE / name).read_bytes()
        (output / name).write_bytes(data)
        report[name + '_sha256'] = hashlib.sha256(data).hexdigest()
    log_root = Path('/home/peter-msi/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.1')
    logs = list(log_root.glob('kit_*.log'))
    logfile = max(logs, key=lambda p: p.stat().st_mtime) if logs else None
    if logfile:
        report.update(isaac_log_path=str(logfile), isaac_log_start_offset=logfile.stat().st_size)
    write_json(output / 'report.json', report)
    started = time.monotonic()
    events = deque()
    event_overflow = False
    names = {int(getattr(omni.timeline.TimelineEventType, n)): n.lower() for n in ('STOP', 'PLAY', 'PAUSE')}

    def on_timeline(event):
        nonlocal event_overflow
        if event.type in names:
            if len(events) >= 64:
                event_overflow = True
                return
            events.append((names[event.type], time.monotonic()-started, time.time_ns()))

    subscription = timeline.get_timeline_event_stream().create_subscription_to_pop(
        on_timeline, name='PeterObservationLidarRestartReader')
    binding = None
    previous_playing = True
    announced_ready = False
    next_report = 0.
    last_stopped_sample = -1.
    try:
        print('LiDAR restart recorder: preparing. Keep Play active; wait for READY. ' + str(output))
        while True:
            update_received = True
            try:
                await asyncio.wait_for(app.post_update_async(), timeout=2.)
            except asyncio.TimeoutError:
                update_received = False
            wall = time.monotonic() - started
            if context.get_stage() != stage:
                raise RuntimeError('Stage changed; this capture only supports Stop -> Play in the same stage.')
            if event_overflow:
                raise RuntimeError('Timeline event queue overflow')
            while events:
                kind, event_wall, event_utc_ns = events.popleft()
                rec.note(kind, event_wall, event_utc_ns)
                binding = None  # Discard every old node/attribute handle on timeline transitions.
                if kind == 'stop':
                    print('LiDAR restart recorder: STOP recorded. Press Play once.')
                elif kind == 'play' and rec.restart_wall is not None:
                    print('LiDAR restart recorder: PLAY recorded. Keep this scene running for 60 seconds.')
            if rec.tick(wall) or wall >= 180.:
                break
            playing = bool(timeline.is_playing())
            if playing != previous_playing:
                binding = None
            previous_playing = playing
            event = dict(sample_index=report['sample_count'], wall_sec=wall, wall_time_ns=time.time_ns(),
                         timeline_seconds=timeline.get_current_time(), playing=playing,
                         update_received=update_received, nodes=[], gpu_payloads={})
            blobs = {}
            if playing and update_received:
                if binding is None:
                    try:
                        binding = discover_pipeline(og)
                    except Exception as exc:
                        report['binding_failures'] += 1
                        event['binding_error'] = type(exc).__name__ + ': ' + str(exc)
                    else:
                        report['binding_generation'] += 1
                        cache = UsdGeom.XformCache()
                        transforms = {p: serializable(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(p)))
                                      for p in (SENSOR, '/iw_hub_ROS', '/World/sedan_car') if stage.GetPrimAtPath(p)}
                        write_json(output / f"pipeline_{report['binding_generation']:03d}.json",
                                   dict(binding['inventory'], wall_sec=wall, wall_time_ns=event['wall_time_ns'],
                                        timeline_epoch=rec.timeline_epoch, usd_world_transforms=transforms))
                if binding is not None:
                    budget = (rec.max_ring_bytes if rec.stop_wall is None
                              else max(0, rec.max_saved_bytes - rec.saved_bytes))
                    event['nodes'], event['gpu_payloads'], blobs = sample_pipeline(binding, wp, budget)
            elif not playing and wall - last_stopped_sample < .25:
                continue
            if not playing:
                last_stopped_sample = wall
                event['buffer_read_skipped'] = 'timeline_not_playing'
            event['binding_generation'] = report['binding_generation']
            done = rec.observe(event, blobs)
            report['sample_count'] += 1
            if rec.ready and not announced_ready:
                print('READY: press Stop, then Play ONCE. Keep the AMR and vehicle unchanged.')
                announced_ready = True
            if wall >= next_report or done:
                report.update(rec.summary(), status='after_restart' if rec.restart_wall is not None else
                              'waiting_for_stop_play' if rec.ready else 'collecting_baseline', capture_wall_sec=wall)
                write_json(output / 'report.json', report)
                next_report = wall + 5.
            if done:
                break
        report['status'] = rec.finished_reason or 'total_time_limit'
    except BaseException as exc:
        report.update(status='error', error=type(exc).__name__ + ': ' + str(exc))
        print('LiDAR restart recorder: ' + report['error'])
    finally:
        subscription.unsubscribe()
        report.update(rec.close(), capture_wall_sec=time.monotonic()-started,
                      finished_utc=datetime.now(timezone.utc).isoformat())
        if rec.finished_reason == 'size_limit' and report['status'] != 'error':
            report['status'] = 'size_limit'
        if logfile and logfile.exists():
            report['isaac_log_end_offset'] = logfile.stat().st_size
        write_json(output / 'report.json', report)
        print('LiDAR restart recorder finished: ' + report['status'] + '. ' + str(output))


if __name__ in ('__main__', '<run_path>'):
    existing = getattr(builtins, LOCK, None)
    if existing is not None and not existing.done():
        raise RuntimeError('A LiDAR diagnostic recorder is already running.')
    setattr(builtins, LOCK, asyncio.ensure_future(capture_restart()))
