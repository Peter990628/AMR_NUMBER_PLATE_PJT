#!/usr/bin/env python3
"""Offline support-extent experiment using production C++ fitting and gating.

No ROS imports, publishers, simulator access or runtime parameter changes.
Compare methods from an identical cold gate state; never substitute missing TF.
"""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

from candidate_frame_math import transform_pose, yaw
from endpoint_support import SupportConfig, supported_extent

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2_ws/src/amr_reobservation'


def load(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def stats(values):
    values = sorted(values)
    if not values:
        return dict(count=0, mean=None, p95=None, max=None)
    return dict(count=len(values), mean=statistics.fmean(values),
                p95=values[math.ceil(.95*len(values))-1], max=values[-1])


def xyz(value):
    return [value[k] for k in 'xyz']


def signature(row):
    return row['epoch'],row['frame_id'],row['stamp_ns']


def source_tf(row):
    for name in ('side','raw'):
        fixed = (row.get(name) or {}).get('fixed',{})
        if 'source_tf' in fixed:
            tf = fixed['source_tf']
            header = tf['header']
            ns = header['stamp']['sec']*10**9+header['stamp']['nanosec']
            if (ns != row['stamp_ns'] or header['frame_id'] != 'odom' or
                    tf['child_frame_id'] != row['frame_id']):
                raise ValueError('TF must be odom-from-sensor at exact source time')
            return tf
    return None


def fixed_candidates(estimate, extent, row):
    tf = source_tf(row)
    if tf is None or not extent['valid']:
        return None
    parameters = (row.get('raw') or row['stable_latest'])['message']
    extension, standoff = parameters['extension_m'],parameters['standoff_m']
    if not all(math.isfinite(v) and v > 0 for v in (extension,standoff)):
        raise ValueError('positive candidate offsets required')
    translation = xyz(tf['transform']['translation'])
    rotation = [tf['transform']['rotation'][k] for k in 'xyzw']
    tangent,normal = estimate['tangent'],estimate['normal_toward_sensor']
    candidates = []
    for endpoint,sign in ((extent['endpoint_a'],-1),(extent['endpoint_b'],1)):
        position = [endpoint[k]+sign*extension*tangent[k]+standoff*normal[k] for k in range(2)]
        heading = math.atan2(endpoint[1]-position[1],endpoint[0]-position[0])
        quaternion = [0,0,math.sin(heading/2),math.cos(heading/2)]
        p,q = transform_pose([*position,0],quaternion,translation,rotation)
        target,_ = transform_pose([*endpoint,0],[0,0,0,1],translation,rotation)
        candidates.append(dict(position_xyz=p,look_at_xyz=target,yaw_rad=yaw(q)))
    return candidates


def original_extent(record):
    e = record['estimate']
    if e is None:
        return dict(valid=False,reason='invalid_line')
    return dict(valid=True,reason='original',endpoint_a=e['endpoint_a'],endpoint_b=e['endpoint_b'],
                observed_length_m=e['observed_length_m'],trim_a_m=0.,trim_b_m=0.)


def replay_gate(binary, rows, candidates, start):
    lines = []
    previous_epoch = None
    for i in range(start,len(rows)):
        row, pair = rows[i], candidates[i]
        if previous_epoch is not None and previous_epoch != row['epoch']:
            lines.append(f'-1 0 0')
        previous_epoch = row['epoch']
        fields = [i,row['stamp_ns'],int(pair is not None)]
        if pair:
            fields.extend(v for c in pair for v in (*c['position_xyz'],*c['look_at_xyz'],c['yaw_rad']))
        lines.append(' '.join(map(str,fields)))
    output = subprocess.check_output([str(binary),'gate'],input='\n'.join(lines)+'\n',text=True)
    return [json.loads(line) for line in output.splitlines() if json.loads(line)['row'] >= 0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--gate-start-index',type=int,default=0)
    parser.add_argument('--score-start-index',type=int,default=2)
    parser.add_argument('--partial-indices',type=int,nargs='*',default=[])
    parser.add_argument('--half-window-m',type=float,default=.05)
    parser.add_argument('--max-half-window-m',type=float,default=.15)
    parser.add_argument('--support-fraction',type=float,default=.6)
    parser.add_argument('--benchmark-repeats',type=int,default=5)
    args = parser.parse_args()
    config = SupportConfig(half_window_m=args.half_window_m,
                           max_half_window_m=args.max_half_window_m,
                           support_fraction=args.support_fraction)
    output = args.output_dir.resolve()
    output.mkdir(parents=True,exist_ok=False)
    rows = load(args.capture/'paired.jsonl')
    if not 0 <= args.gate_start_index < args.score_start_index < len(rows):
        raise ValueError('require 0 <= gate-start < score-start < paired count')
    if args.benchmark_repeats < 1 or any(not 0 <= i < len(rows) for i in args.partial_indices):
        raise ValueError('invalid benchmark count or partial indices')
    if len({signature(r) for r in rows}) != len(rows):
        raise ValueError('duplicate paired source headers')
    for a,b in zip(rows,rows[1:]):
        if a['epoch'] == b['epoch'] and a['stamp_ns'] >= b['stamp_ns']:
            raise ValueError('paired sources must be strictly ordered within an epoch')
    streams = load(args.capture/'streams.jsonl')
    scans = {r['sequence']:r for r in streams if r['stream']=='scan'}
    inputs = []
    for i,row in enumerate(rows):
        scan = scans[row['scan_sequence']]
        if signature(scan) != signature(row):
            raise ValueError('scan must match epoch, frame and stamp')
        m = scan['message']
        fields = [i,len(m['ranges']),m['angle_min'],m['angle_increment'],m['range_min'],m['range_max']]
        fields.extend('nan' if r is None else r for r in m['ranges'])
        inputs.append(' '.join(map(str,fields)))
    binary = output/'replay'
    sources = [ROOT/'tools/endpoint_support_replay.cpp',
               *(PACKAGE/'src'/name for name in ('scan_preprocessor.cpp','vehicle_side_estimator.cpp','candidate_stability.cpp'))]
    compile_command = ['c++','-std=c++17','-O2','-Wall','-Wextra','-pedantic',
                       '-I'+str(PACKAGE/'include'),'-I/usr/include/eigen3',
                       *map(str,sources),'-o',str(binary)]
    subprocess.run(compile_command,check=True)
    replay = subprocess.check_output([str(binary),'estimate'],input='\n'.join(inputs)+'\n',text=True)
    (output/'fitted.jsonl').write_text(replay)
    fitted = [json.loads(line) for line in replay.splitlines()]
    assert len(fitted) == len(rows)
    old_matches = 0
    for i,(record,row) in enumerate(zip(fitted,rows)):
        assert record['scan_sequence'] == i
        old = row['side']['message']
        if old['valid']:
            e = record['estimate']
            assert e is not None
            for key in ('center','tangent','normal_toward_sensor','endpoint_a','endpoint_b'):
                assert e[key] == [old[key]['x'],old[key]['y']], (i,key)
            for key in ('observed_length_m','rmse_m','inlier_ratio'):
                assert e[key] == old[key],(i,key)
            assert len(e['inlier_indices']) == old['inlier_count']
            old_matches += 1
    definitions = {'baseline':None,'fixed_support':replace(config,adaptive=False),'adaptive_support':config}
    report = dict(capture=str(args.capture.resolve()),paired_count=len(rows),
                  cpp_refinement_limit=20,existing_valid_side_exact_matches=old_matches,
                  fitted_valid_count=sum(r['estimate'] is not None for r in fitted),
                  exact_tf_count=sum(source_tf(r) is not None for r in rows),
                  config=asdict(config),compile_command=compile_command,
                  gate_start_index=args.gate_start_index,score_start_index=args.score_start_index,
                  partial_indices=args.partial_indices,methods={},
                  methodology='Same-input controlled offline replay. Same cold gate start for all methods; '
                  'production C++ gate thresholds unchanged. Invalid extents or missing exact TF reset gate. '
                  'No live timing, no held poses, no claim of true vehicle endpoints.',
                  source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources+[
                      Path(__file__),ROOT/'tools/endpoint_support.py',ROOT/'tools/candidate_frame_math.py',
                      *(PACKAGE/'include/amr_reobservation'/name for name in ('scan_preprocessor.hpp','vehicle_side_estimator.hpp','candidate_stability.hpp')),
                      args.capture/'paired.jsonl',args.capture/'streams.jsonl']})
    records_by_method = {}
    for name,method_config in definitions.items():
        extents = [original_extent(r) if method_config is None else supported_extent(r,method_config) for r in fitted]
        pairs = [fixed_candidates(r['estimate'],e,row) if e['valid'] else None
                 for r,e,row in zip(fitted,extents,rows)]
        if name == 'baseline':
            for pair,row in zip(pairs,rows):
                old_pair = (row.get('raw') or {}).get('fixed',{}).get('candidates')
                if old_pair and pair:
                    for a,b in zip(pair,old_pair):
                        assert math.dist(a['position_xyz'],b['position_xyz']) < 1e-10
                        assert math.dist(a['look_at_xyz'],b['look_at_xyz']) < 1e-10
                        assert abs(math.remainder(a['yaw_rad']-b['yaw_rad'],2*math.pi)) < 1e-10
        gate = replay_gate(binary,rows,pairs,args.gate_start_index)
        by_row = {r['row']:r for r in gate}
        scoring = [r for r in gate if r['row'] >= args.score_start_index]
        transitions = []
        for i in range(1,len(rows)):
            if pairs[i] is None or pairs[i-1] is None or rows[i]['epoch'] != rows[i-1]['epoch']:
                continue
            if not 0 < (rows[i]['stamp_ns']-rows[i-1]['stamp_ns'])/1e9 <= .35:
                continue
            # Geometric matching mirrors the production gate's sum of XYZ distances.
            old,current = pairs[i-1],pairs[i]
            direct = sum(math.dist(a['position_xyz'],b['position_xyz']) for a,b in zip(old,current))
            reverse = sum(math.dist(a['position_xyz'],b['position_xyz']) for a,b in zip(old,reversed(current)))
            if reverse < direct:
                current = list(reversed(current))
            transitions.append(dict(index=i,
                partial_related=i in args.partial_indices or i-1 in args.partial_indices,
                candidate_delta_m=max(math.dist(a['position_xyz'],b['position_xyz']) for a,b in zip(old,current)),
                endpoint_delta_m=max(math.dist(a['look_at_xyz'],b['look_at_xyz']) for a,b in zip(old,current))))
        times = []
        if method_config is not None:
            for _ in range(2):
                for r in fitted:
                    supported_extent(r,method_config)
            for _ in range(args.benchmark_repeats):
                for r in fitted:
                    start = time.perf_counter_ns()
                    supported_extent(r,method_config)
                    times.append((time.perf_counter_ns()-start)/1e6)
        summary = dict(extent_status_counts=dict(Counter(e['reason'] for e in extents)),
                       supported_valid_count=sum(e['valid'] for e in extents),
                       scored_count=len(scoring),accepted=sum(r['valid'] for r in scoring),
                       acceptance_rate=sum(r['valid'] for r in scoring)/len(scoring),
                       gate_status_counts=dict(Counter(r['status'] for r in scoring)),
                       span_reduction_m=stats([r['estimate']['observed_length_m']-e['observed_length_m']
                                             for r,e in zip(fitted,extents) if e['valid']]),
                       trim_a_m=stats([e['trim_a_m'] for e in extents if e['valid']]),
                       trim_b_m=stats([e['trim_b_m'] for e in extents if e['valid']]),
                       normal_adjacent_candidate_delta_m=stats([t['candidate_delta_m'] for t in transitions if not t['partial_related']]),
                       normal_adjacent_endpoint_delta_m=stats([t['endpoint_delta_m'] for t in transitions if not t['partial_related']]),
                       normal_adjacent_candidate_over15_count=sum(t['candidate_delta_m']>.15 for t in transitions if not t['partial_related']),
                       partial_transitions=[t for t in transitions if t['partial_related']],
                       partial_gate_decisions=[by_row[i] for i in args.partial_indices if i in by_row],
                       support_python_ms=stats(times),timing_scope='Python endpoint support only, excluding C++ fit, TF, gate, I/O; offline host timing')
        if name == 'baseline':
            summary['recorded_gate_status_mismatches'] = [r['row'] for r in scoring
                if rows[r['row']].get('stable_latest') is not None and
                (r['status'] if r['valid'] else f"{r['status']}:{r['consecutive']}/3") != rows[r['row']]['stable_latest']['message']['status']]
        records = [dict(index=i,source_key=signature(row),extent=e,fixed_candidates=pair,gate=by_row.get(i))
                   for i,(row,e,pair) in enumerate(zip(rows,extents,pairs))]
        records_by_method[name] = records
        with (output/f'{name}.jsonl').open('w') as f:
            for r in records:
                f.write(json.dumps(r,allow_nan=False)+'\n')
        report['methods'][name] = summary
    # Check that support only selects points within the original inlier extent.
    for name in ('fixed_support','adaptive_support'):
        for r in records_by_method[name]:
            if r['extent']['valid']:
                assert r['extent']['trim_a_m'] >= 0 and r['extent']['trim_b_m'] >= 0
                assert r['extent']['observed_length_m'] <= fitted[r['index']]['estimate']['observed_length_m']+1e-12
    write_json(output/'report.json',report)
    print(json.dumps({name:{k:s[k] for k in ('scored_count','accepted','gate_status_counts','span_reduction_m',
                        'normal_adjacent_candidate_delta_m','support_python_ms')} for name,s in report['methods'].items()},indent=2))


if __name__ == '__main__':
    main()
