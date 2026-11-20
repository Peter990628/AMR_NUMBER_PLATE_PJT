"""Read the existing observation-LiDAR graph in an Isaac Sim 5.1 GUI.

Run with runpy.run_path() in Script Editor while Play is active. Samples for
30 wall seconds without attaching annotators, creating render products, editing
USD/graph/settings, controlling the timeline, or sending robot commands.

By default only OmniGraph-managed values are copied. The optional
CAPTURE_GPU_PAYLOADS mode copies bounded, existing GPU buffers through Warp;
it does not create new sensor/annotator buffers. CPU pointers are not accessed.
A post-update snapshot is not a guarantee
that every node last executed on the same render frame; frame/time fields and
connections are preserved for subsequent alignment.
"""

import asyncio
import builtins
import hashlib
import json
import math
import runpy
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


PROJECT = Path(__file__).resolve().parents[1]
OUTPUT_ROOT = PROJECT / 'local_artifacts/lidar_input_diagnosis_2026-11-20'
SENSOR = '/iw_hub_ROS/chassis/ObservationLidar/observation_lidar'
FRAME = 'observation_lidar'
TOPIC = '/observation_lidar/scan'
LOCK = '_peter_lidar_pipeline_inspection_task'
DURATION = 30.0
MAX_ARRAY = 16384
MAX_BYTES = 100 * 1024 * 1024
CAPTURE_GPU_PAYLOADS = bool(globals().get('CAPTURE_GPU_PAYLOADS', False))
CAPTURE_DROPOUT_WINDOW = bool(globals().get('CAPTURE_DROPOUT_WINDOW', False))
if CAPTURE_DROPOUT_WINDOW:
    CAPTURE_GPU_PAYLOADS = True
    DURATION = 125.0


def serializable(value):
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, np.generic):
        return serializable(value.item())
    if isinstance(value, np.ndarray):
        if value.size > MAX_ARRAY:
            return {'omitted': 'array_limit', 'shape': list(value.shape), 'dtype': str(value.dtype)}
        return serializable(value.tolist())
    if isinstance(value, dict):
        return {str(k): serializable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)) or hasattr(value, '__len__'):
        try:
            if len(value) > MAX_ARRAY:
                return {'omitted': 'array_limit', 'length': len(value)}
            return [serializable(v) for v in value]
        except TypeError:
            pass
    return str(value)


def write_json(path, value):
    path.write_text(json.dumps(serializable(value), ensure_ascii=False, indent=2, allow_nan=False) + '\n')


async def inspect_pipeline():
    import carb.settings
    import omni.graph.core as og
    import omni.kit.app
    import omni.timeline
    import omni.usd
    from pxr import UsdGeom

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage is None or not timeline.is_playing():
        raise RuntimeError('Open the existing failure scene and keep Play active before running this reader.')
    sensor = stage.GetPrimAtPath(SENSOR)
    if not sensor:
        raise RuntimeError('Expected observation LiDAR not found: ' + SENSOR)
    prefix = 'gui_dropout_' if CAPTURE_DROPOUT_WINDOW else 'gui_pipeline_'
    output = OUTPUT_ROOT / (prefix + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'preparing', 'output': str(output), 'scene_changes': False,
              'motion_commands': False, 'annotators_attached': False,
              'pointer_payloads_captured': False, 'gpu_payload_mode': CAPTURE_GPU_PAYLOADS,
              'dropout_window_mode': CAPTURE_DROPOUT_WINDOW,
              'duration_limit_sec': DURATION,
              'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'scope': __doc__, 'sample_count': 0}
    write_json(output / 'report.json', report)
    start = time.monotonic()
    stream = None
    payload_stream = None
    recorder = None
    try:
        def read(attr):
            try:
                if str(attr.get_resolved_type()) == 'execution':
                    return {'omitted': 'execution_attribute'}
                return serializable(og.Controller.get(attr))
            except Exception as exc:
                return {'read_error': type(exc).__name__ + ': ' + str(exc)}

        def read_named(node, name):
            attr = node.get_attribute(name)
            return read(attr) if attr.is_valid() else None

        all_nodes = {}
        graph_paths = []
        for graph in og.get_all_graphs():
            graph_paths.append(graph.get_path_to_graph())
            for node in graph.get_nodes():
                all_nodes[node.get_prim_path()] = node
        publishers = [n for n in all_nodes.values()
                      if n.get_type_name().endswith('.ROS2PublishLaserScan') and
                      (read_named(n, 'inputs:frameId') == FRAME or
                       read_named(n, 'inputs:topicName') in (TOPIC, TOPIC.lstrip('/')))]
        metadata = {
            'stage_identifier': stage.GetRootLayer().identifier,
            'timeline_seconds': timeline.get_current_time(),
            'sensor_path': SENSOR, 'sensor_type': sensor.GetTypeName(),
            'sensor_attributes': {a.GetName(): serializable(a.Get()) for a in sensor.GetAttributes()},
            'sensor_world_transform': serializable(UsdGeom.XformCache().GetLocalToWorldTransform(sensor)),
            'settings': {key: serializable(carb.settings.get_settings().get(key)) for key in [
                '/app/sensors/nv/lidar/outputBufferOnGPU', '/rtx/sensors/coordinates',
                '/app/runLoops/main/rateLimitFrequency', '/app/runLoops/main/rateLimitEnabled']},
            'graph_paths': graph_paths,
            'publisher_paths': [n.get_prim_path() for n in publishers],
            'lidar_related_nodes': [{'path': p, 'type': n.get_type_name()}
                                    for p, n in all_nodes.items()
                                    if any(s in n.get_type_name() for s in ('Lidar', 'LaserScan'))],
        }
        write_json(output / 'scene_and_graphs.json', metadata)
        if len(publishers) != 1:
            raise RuntimeError(f'Expected exactly one observation LaserScan publisher, found {len(publishers)}. See scene_and_graphs.json.')

        # Follow the actual publisher inputs, including the runtime render graph.
        selected, connections = {}, []
        pending = list(publishers)
        while pending:
            node = pending.pop()
            path = node.get_prim_path()
            if path in selected:
                continue
            if len(selected) >= 80:
                raise RuntimeError('Unexpectedly large upstream graph; stopped without changing it.')
            selected[path] = node
            for attr in node.get_attributes():
                for upstream in attr.get_upstream_connections():
                    parent = upstream.get_node()
                    connections.append({'source_node': parent.get_prim_path(), 'source_attribute': upstream.get_name(),
                                        'target_node': path, 'target_attribute': attr.get_name()})
                    if parent.get_prim_path() not in selected:
                        pending.append(parent)
        inventory = []
        dynamic = []
        for path, node in selected.items():
            attrs = list(node.get_attributes())
            inventory.append({'path': path, 'type': node.get_type_name(),
                              'attributes': {a.get_name(): read(a) for a in attrs}})
            # Keep arrays only for scan depths; avoid image buffers or intensities.
            watched = []
            for attr in attrs:
                name = attr.get_name()
                lower = name.lower()
                if any(s in lower for s in ('intensitiesdata', 'exec', 'context')):
                    continue
                if name.startswith('outputs:') or any(s in lower for s in (
                        'lineardepthdata', 'timestamp', 'renderproduct', 'cuda', 'ptr',
                        'buffersize', 'rotationrate', 'numcols', 'numrows')):
                    watched.append(attr)
            dynamic.append((path, node.get_type_name(), watched))
        write_json(output / 'pipeline.json', {'nodes': inventory, 'connections': connections})
        payload_specs = []
        if CAPTURE_GPU_PAYLOADS:
            import warp as wp
            for node in selected.values():
                typ = node.get_type_name()
                if typ.endswith('.SdRenderVarPtr') and read_named(node, 'inputs:renderVar') == 'GenericModelOutput':
                    payload_specs.append(('raw_gmo', node, 'outputs:dataPtr', 'outputs:bufferSize', 'uint8'))
                if typ.endswith('.IsaacCreateRTXLidarScanBuffer'):
                    for field in ['azimuth', 'distance']:
                        payload_specs.append(('accumulator_' + field, node, 'outputs:' + field + 'Ptr',
                                              'outputs:' + field + 'BufferSize', 'float32'))
            if sorted(x[0] for x in payload_specs) != ['accumulator_azimuth', 'accumulator_distance', 'raw_gmo']:
                raise RuntimeError('Expected one GMO source and one accumulator; refusing ambiguous buffer capture.')
            if CAPTURE_DROPOUT_WINDOW:
                helper = Path(__file__).with_name('lidar_dropout_recorder.py')
                recorder_class = runpy.run_path(str(helper))['DropoutRecorder']
                recorder = recorder_class(output)
                (output / 'recorder_source.py').write_bytes(helper.read_bytes())
                report.update(trigger_wait_limit_sec=120., pre_event_sec=3., post_event_sec=5.,
                              ring_limit_bytes=16 * 1024 * 1024,
                              recorder_sha256=hashlib.sha256(helper.read_bytes()).hexdigest())
            else:
                payload_stream = (output / 'gpu_payloads.bin').open('wb')

        def copy_payloads():
            result = {}
            blobs = {}
            for label, node, ptr_name, size_name, dtype in payload_specs:
                ptr = int(read_named(node, ptr_name)); size = int(read_named(node, size_name))
                device_id = int(read_named(node, 'outputs:cudaDeviceIndex'))
                entry = {'node': node.get_prim_path(), 'pointer': ptr, 'byte_count': size,
                         'device': device_id, 'dtype': dtype}
                if ptr == 0 or size == 0:
                    entry['status'] = 'empty'
                elif device_id < 0:
                    entry['status'] = 'cpu_pointer_not_copied'
                elif not (0 < size <= 2 * 1024 * 1024):
                    raise RuntimeError('Unexpected buffer size: ' + str(entry))
                else:
                    device = f'cuda:{device_id}'
                    wp.synchronize_device(device)
                    # Non-owning view, as used by the installed Replicator pointer reader.
                    view = wp.array(ptr=ptr, dtype=wp.uint8, shape=(size,), device=device, requires_grad=False)
                    data = view.numpy().copy().tobytes()
                    entry.update(status='copied',
                                 sha256=hashlib.sha256(data).hexdigest(),
                                 source_pointer_unchanged=ptr == int(read_named(node, ptr_name)),
                                 source_size_unchanged=size == int(read_named(node, size_name)))
                    if recorder is not None:
                        blobs[label] = data
                    else:
                        entry['offset'] = payload_stream.tell()
                        payload_stream.write(data)
                    report['pointer_payloads_captured'] = True
                result[label] = entry
            return result, blobs
        report.update(status='recording', node_count=len(selected), started_utc=datetime.now(timezone.utc).isoformat())
        write_json(output / 'report.json', report)
        (output / 'capture_script.py').write_bytes(Path(__file__).read_bytes())
        print('LiDAR pipeline reader started (maximum ' + str(DURATION) + ' seconds): ' + str(output))
        if recorder is None:
            stream = (output / 'frames.jsonl').open('w', buffering=1)
        capture_start = time.monotonic()
        previous_signature = None
        next_heartbeat = 5.
        while time.monotonic() - capture_start < DURATION:
            # Time bound also applies if the GUI stops producing update events.
            await asyncio.wait_for(app.post_update_async(), timeout=5.0)
            if context.get_stage() != stage or not timeline.is_playing():
                raise RuntimeError('Stage or Play state changed; recording stopped.')
            values = [{'path': path, 'type': typ, 'values': {a.get_name(): read(a) for a in attrs}}
                      for path, typ, attrs in dynamic]
            signature = hashlib.sha256(json.dumps(values, sort_keys=True).encode()).hexdigest()
            event = {'wall_sec': time.monotonic()-capture_start, 'wall_time_ns': time.time_ns(),
                     'timeline_seconds': timeline.get_current_time(),
                     'sample_index': report['sample_count'], 'node_values_sha256': signature,
                     'same_values_as_previous_sample': signature == previous_signature,
                     'nodes': values}
            if CAPTURE_GPU_PAYLOADS:
                event['gpu_payloads'], blobs = copy_payloads()
            done = recorder.observe(event, blobs) if recorder is not None else False
            if stream is not None:
                stream.write(json.dumps(event, allow_nan=False) + '\n')
            previous_signature = signature
            report['sample_count'] += 1
            if recorder is not None and (event['wall_sec'] >= next_heartbeat or done):
                report.update(recorder.summary())
                write_json(output / 'report.json', report)
                next_heartbeat = event['wall_sec'] + 5.
            if done:
                report['status'] = recorder.finished_reason
                break
            total_bytes = ((recorder.frames.tell() + recorder.payload_file.tell()) if recorder is not None
                           else stream.tell() + (payload_stream.tell() if payload_stream is not None else 0))
            if total_bytes >= MAX_BYTES or report['sample_count'] >= (10000 if recorder is not None else 2000):
                report['status'] = 'bounded_limit'
                break
        else:
            report['status'] = 'complete'
        report['capture_wall_sec'] = time.monotonic()-capture_start
    except BaseException as exc:
        report.update(status='error', error=type(exc).__name__ + ': ' + str(exc))
        print('LiDAR pipeline reader: ' + report['error'])
    finally:
        if recorder is not None:
            report.update(recorder.close())
        if stream is not None:
            stream.close()
        if payload_stream is not None:
            report['gpu_payload_bytes'] = payload_stream.tell()
            payload_stream.close()
        report.update(elapsed_wall_sec=time.monotonic()-start, finished_utc=datetime.now(timezone.utc).isoformat())
        write_json(output / 'report.json', report)
        print('LiDAR pipeline reader ' + report['status'] + ': ' + str(output))


if __name__ in ('__main__', '<run_path>'):
    existing = getattr(builtins, LOCK, None)
    if existing is not None and not existing.done():
        raise RuntimeError('A LiDAR pipeline reader is already running.')
    setattr(builtins, LOCK, asyncio.ensure_future(inspect_pipeline()))
