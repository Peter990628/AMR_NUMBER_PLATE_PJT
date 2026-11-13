#!/usr/bin/env python3
"""Controlled offline comparison of the production gate and retained EMA reference.

Uses original min/max endpoints, actual C++ cap20 fits, and exact-time odom TF.
EMA is an internal decision reference; accepted outputs retain current raw geometry.
No ROS, simulator changes, publications, or use of known failure labels in gating.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time

from candidate_ema_experiment import EmaConfig, EmaGate
from evaluate_endpoint_support import (
    ROOT, PACKAGE, fixed_candidates, load, original_extent, replay_gate,
    signature, source_tf, stats, write_json,
)

DATASETS = (
    ('main', 'refinement_limit_live_2026-11-12/after_20', 3, 4, (16, 115)),
    ('parallel', 'candidate_stability_2026-11-12/parallel_return', 0, 2, ()),
    ('plus10', 'candidate_stability_2026-11-12/plus10_confirmed', 0, 2, ()),
    ('minus10', 'candidate_stability_2026-11-12/after', 0, 2, ()),
)
METHODS = {'ema_02': .2, 'ema_01': .1, 'ema_04': .4, 'retained_last_10': 1.}


def align(reference, current):
    direct = sum(math.dist(a['position_xyz'], b['position_xyz'])
                 for a, b in zip(reference, current))
    reverse = sum(math.dist(a['position_xyz'], b['position_xyz'])
                  for a, b in zip(reference, reversed(current)))
    return list(reversed(current)) if reverse < direct else current


def delta(reference, current):
    current = align(reference, current)
    return {
        'position_m': max(math.dist(a['position_xyz'], b['position_xyz'])
                          for a, b in zip(reference, current)),
        'target_m': max(math.dist(a['look_at_xyz'], b['look_at_xyz'])
                        for a, b in zip(reference, current)),
        'yaw_deg': max(abs(math.remainder(a['yaw_rad']-b['yaw_rad'], 2*math.pi))
                       *180/math.pi for a, b in zip(reference, current)),
    }


def write_lines(path, rows):
    path.write_text(''.join(json.dumps(r, allow_nan=False)+'\n' for r in rows))


def source_context(row):
    params = (row.get('raw') or row['stable_latest'])['message']
    return json.dumps(['odom', row['epoch'], row['frame_id'],
                       params['extension_m'], params['standoff_m']])


def ema_replay(rows, pairs, start, alpha, benchmark=False):
    gate = EmaGate(EmaConfig(alpha=alpha))
    result, durations = [], []
    reference = None
    for i in range(start, len(rows)):
        row, pair = rows[i], pairs[i]
        before = delta(reference, pair) if reference and pair else None
        wall = (row.get('raw') or row['side'])['received_wall_sec']
        begin = time.perf_counter_ns()
        decision = gate.update(pair, row['stamp_ns'], wall, source_context(row))
        elapsed = (time.perf_counter_ns()-begin)/1e6
        if benchmark:
            durations.append(elapsed)
        assert decision['valid'] == (decision['output'] is not None)
        if decision['valid']:
            # An accepted output is exactly this observation, possibly A/B swapped.
            for emitted, raw in zip(decision['output'], align(decision['output'], pair)):
                assert emitted['position_xyz'] == list(raw['position_xyz'])
                assert emitted['look_at_xyz'] == list(raw['look_at_xyz'])
                assert emitted['yaw_rad'] == raw['yaw_rad']
        reference = decision['reference']
        result.append(dict(index=i, source_key=signature(row),
                           pre_reference_delta=before, **decision))
    return result, durations


def transitions(records, rows, field, partial, adjacent_only=True, max_gap_sec=.35):
    previous = None
    result = []
    for r in records:
        value = r.get(field) if r['valid'] else None
        if value is None:
            if adjacent_only:
                previous = None
            continue
        if previous is not None:
            i, j = previous['index'], r['index']
            gap = (rows[j]['stamp_ns']-rows[i]['stamp_ns'])/1e9
            if (source_context(rows[i]) == source_context(rows[j]) and gap > 0
                    and (max_gap_sec is None or gap <= max_gap_sec)):
                result.append(dict(index=j, previous_index=i, gap_sec=gap,
                                   partial_related=any(k in partial for k in range(i, j+1)),
                                   **delta(previous[field], value)))
        previous = r
    return result


def transition_summary(values):
    return dict(position_m=stats([v['position_m'] for v in values]),
                target_m=stats([v['target_m'] for v in values]),
                yaw_deg=stats([v['yaw_deg'] for v in values]),
                position_over_15cm=sum(v['position_m'] > .15 for v in values),
                target_over_15cm=sum(v['target_m'] > .15 for v in values))


def summary(records, rows, score, partial):
    scored = [r for r in records if r['index'] >= score]
    out_steps = transitions(scored, rows, 'output', partial)
    ref_steps = transitions(scored, rows, 'reference', partial)
    accepted_steps = transitions(scored, rows, 'output', partial, adjacent_only=False)
    all_accepted_steps = transitions(scored, rows, 'output', partial,
                                    adjacent_only=False, max_gap_sec=None)
    raw_ref = [delta(r['reference'], r['output'])['position_m']
               for r in scored if r['valid'] and r.get('reference')]
    run, maximum = 0, 0
    for r in scored:
        run = 0 if r['valid'] else run+1
        maximum = max(maximum, run)
    events = []
    lookup = {r['index']: r for r in records}
    for i in partial:
        recovery = next((lookup[j] for j in range(i+1, len(rows))
                         if j in lookup and lookup[j]['valid']), None)
        events.append(dict(index=i, rejected=not lookup[i]['valid'],
                           status=lookup[i]['status'],
                           first_return_accepted=lookup.get(i+1, {}).get('valid'),
                           recovery_index=recovery['index'] if recovery else None,
                           recovery_source_sec=(rows[recovery['index']]['stamp_ns']-
                                                rows[i]['stamp_ns'])/1e9 if recovery else None))
    return dict(scored_count=len(scored), accepted=sum(r['valid'] for r in scored),
                acceptance_rate=sum(r['valid'] for r in scored)/len(scored),
                statuses=dict(Counter(r['status'] for r in scored)),
                reacquired_count=sum(r.get('reacquired', False) for r in scored),
                max_consecutive_invalid=maximum,
                adjacent_accepted_raw=transition_summary(out_steps),
                adjacent_accepted_reference=transition_summary(ref_steps),
                successive_accepted_raw_within_gap=transition_summary(accepted_steps),
                successive_accepted_raw_all_gaps=transition_summary(all_accepted_steps),
                accepted_raw_to_reference_m=stats(raw_ref),
                known_single_partial_events=events,
                transition_scope='Scored interval only; actual adjacent accepted pairs for '
                'adjacent_*; successive accepted output pairs may span invalid frames '
                'but within_gap requires source gap <= .35s, all_gaps has no gap cutoff. '
                'All transitions require unchanged context. Reference is internal, not output. '
                'pre_reference_delta is measured before reset/expiry handling, not a rejection reason.')


def fit_capture(capture, binary, output):
    rows, streams = load(capture/'paired.jsonl'), load(capture/'streams.jsonl')
    if len({signature(r) for r in rows}) != len(rows):
        raise ValueError('duplicate paired source headers')
    for a, b in zip(rows, rows[1:]):
        if a['epoch'] == b['epoch'] and a['stamp_ns'] >= b['stamp_ns']:
            raise ValueError('unordered source timestamps')
    scans = {r['sequence']: r for r in streams if r['stream'] == 'scan'}
    inputs = []
    for i, row in enumerate(rows):
        scan = scans[row['scan_sequence']]
        if signature(scan) != signature(row):
            raise ValueError('scan does not match paired header')
        m = scan['message']
        fields = [i, len(m['ranges']), m['angle_min'], m['angle_increment'],
                  m['range_min'], m['range_max']]
        fields.extend('nan' if r is None else r for r in m['ranges'])
        inputs.append(' '.join(map(str, fields)))
    replay = subprocess.check_output([str(binary), 'estimate'],
                                     input='\n'.join(inputs)+'\n', text=True)
    (output/'fitted.jsonl').write_text(replay)
    fitted = [json.loads(line) for line in replay.splitlines()]
    assert len(fitted) == len(rows)
    matched = 0
    for i, (record, row) in enumerate(zip(fitted, rows)):
        assert record['scan_sequence'] == i
        old, e = row['side']['message'], record['estimate']
        if old['valid']:
            for key in ('center', 'tangent', 'normal_toward_sensor', 'endpoint_a', 'endpoint_b'):
                assert e[key] == [old[key]['x'], old[key]['y']], (i, key)
            for key in ('observed_length_m', 'rmse_m', 'inlier_ratio'):
                assert e[key] == old[key], (i, key)
            assert len(e['inlier_indices']) == old['inlier_count']
            matched += 1
    pairs = [fixed_candidates(r['estimate'], original_extent(r), row)
             if r['estimate'] is not None else None for r, row in zip(fitted, rows)]
    for pair, row in zip(pairs, rows):
        recorded = (row.get('raw') or {}).get('fixed', {}).get('candidates')
        if recorded and pair:
            assert max(delta(recorded, pair).values()) < 1e-10
    return rows, pairs, fitted, matched


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--artifact-root', type=Path, default=ROOT/'local_artifacts')
    parser.add_argument('--benchmark-repeats', type=int, default=5)
    args = parser.parse_args()
    if args.benchmark_repeats < 1:
        parser.error('benchmark repeats must be positive')
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    binary = output/'replay'
    cpp = [ROOT/'tools/endpoint_support_replay.cpp',
           *(PACKAGE/'src'/name for name in ('scan_preprocessor.cpp',
             'vehicle_side_estimator.cpp', 'candidate_stability.cpp'))]
    command = ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-pedantic',
               '-I'+str(PACKAGE/'include'), '-I/usr/include/eigen3',
               *map(str, cpp), '-o', str(binary)]
    subprocess.run(command, check=True)
    paths = cpp + [Path(__file__), ROOT/'tools/candidate_ema_experiment.py',
                   ROOT/'tools/evaluate_endpoint_support.py',
                   ROOT/'tools/candidate_frame_math.py',
                   *(PACKAGE/'include/amr_reobservation'/name for name in (
                       'scan_preprocessor.hpp', 'vehicle_side_estimator.hpp', 'candidate_stability.hpp'))]
    report = dict(primary_method='ema_02', config=asdict(EmaConfig()),
                  methods=METHODS, compile_command=command, datasets={},
                  methodology='Same cap20 min/max geometry and exact odom TF. Same cold starts '
                  'and scoring intervals. Primary alpha .2; .1/.4 sensitivity and alpha1 retained '
                  'last-reference ablation. Same numeric thresholds but changed consistency rule. '
                  'No actual endpoint truth; adoption rate is availability, not accuracy. '
                  'Known partial indices are labels for reporting only. '
                  'Accepted output is current raw geometry, not the diagnostic EMA reference. '
                  'Recorded raw arrival wall seconds drive timeout checks; no ROS I/O replay.')
    all_timing = []
    for name, relative, start, score, partial in DATASETS:
        capture, dest = args.artifact_root/relative, output/name
        dest.mkdir()
        paths += [capture/'paired.jsonl', capture/'streams.jsonl']
        rows, pairs, fitted, matched = fit_capture(capture, binary, dest)
        baseline = [dict(index=g['row'], source_key=signature(rows[g['row']]),
                         output=pairs[g['row']] if g['valid'] else None,
                         reference=None, reacquired=False, **g)
                    for g in replay_gate(binary, rows, pairs, start)]
        write_lines(dest/'baseline.jsonl', baseline)
        result = dict(paired_count=len(rows), fitted_valid_count=sum(r['estimate'] is not None for r in fitted),
                      existing_valid_fit_exact_matches=matched,
                      exact_tf_count=sum(source_tf(r) is not None for r in rows),
                      gate_start_index=start, score_start_index=score,
                      known_partial_indices=partial,
                      baseline=summary(baseline, rows, score, partial), methods={})
        if name == 'main':
            mismatches = [r['index'] for r in baseline if r['index'] >= score and
                (r['status'] if r['valid'] else f"{r['status']}:{r['consecutive']}/3") !=
                rows[r['index']]['stable_latest']['message']['status']]
            assert not mismatches, mismatches
            result['baseline_live_status_mismatches'] = mismatches
        for method, alpha in METHODS.items():
            records, _ = ema_replay(rows, pairs, start, alpha)
            write_lines(dest/(method+'.jsonl'), records)
            metrics = summary(records, rows, score, partial)
            common = [i for i in range(score, len(rows)) if
                      baseline[i-start]['valid'] and records[i-start]['valid']]
            metrics['common_accepted_count'] = len(common)
            metrics['common_accepted_geometry_difference_max_m'] = max(
                (delta(baseline[i-start]['output'], records[i-start]['output'])['position_m']
                 for i in common), default=None)
            result['methods'][method] = metrics
            if method == 'ema_02':
                for _ in range(args.benchmark_repeats):
                    _, times = ema_replay(rows, pairs, start, alpha, benchmark=True)
                    all_timing.extend(times)
        # Adjacent raw-input motion independent of gate acceptance, same scored indices.
        raw_records = [dict(index=i, valid=pairs[i] is not None, output=pairs[i])
                       for i in range(score, len(rows))]
        raw_steps = transitions(raw_records, rows, 'output', partial)
        result['raw_input_normal_adjacent'] = transition_summary(
            [r for r in raw_steps if not r['partial_related']])
        result['raw_input_partial_transitions'] = [r for r in raw_steps if r['partial_related']]
        write_json(dest/'report.json', result)
        report['datasets'][name] = result
        print(name, 'baseline', result['baseline']['accepted'], '/', result['baseline']['scored_count'],
              {k: v['accepted'] for k, v in result['methods'].items()}, flush=True)
    report['primary_python_update_ms'] = stats(all_timing)
    report['timing_scope'] = ('Offline Python state update including validation/copies only; excludes '
                             'fitting, TF, transport, timer scheduling and C++ runtime. Not node latency.')
    report['sha256'] = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    write_json(output/'report.json', report)


if __name__ == '__main__':
    main()
