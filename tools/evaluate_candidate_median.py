#!/usr/bin/env python3
"""Saved-scan comparison of production, retained-last and causal segment median.

All methods use identical cap20 fits with original min/max endpoints and exact
odom TF. Median changes emitted geometry; saved data supplies no endpoint truth.
No ROS runtime changes or publications.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time

from candidate_frame_math import quaternion_matrix, transform_pose
from candidate_median_experiment import MedianCandidateFilter, make_candidates
from evaluate_candidate_ema import (
    DATASETS, ROOT, PACKAGE, delta, ema_replay, fit_capture, replay_gate,
    signature, source_context, source_tf, stats, summary, write_json, write_lines,
    transitions, transition_summary,
)


def fixed_segment(record, row):
    """Transform the fitted segment, never median already-generated poses."""
    tf, e = source_tf(row), record['estimate']
    if tf is None or e is None:
        return None
    rotation = [tf['transform']['rotation'][k] for k in 'xyzw']
    translation = [tf['transform']['translation'][k] for k in 'xyz']
    matrix = quaternion_matrix(rotation)
    midpoint = [(a+b)/2 for a, b in zip(e['endpoint_a'], e['endpoint_b'])]+[0.]
    center, _ = transform_pose(midpoint, [0, 0, 0, 1], translation, rotation)

    def rotate(v):
        return [sum(matrix[i][j]*v[j] for j in range(3)) for i in range(3)]

    offsets = (row.get('raw') or row['stable_latest'])['message']
    return dict(center_xyz=list(center), tangent_xyz=rotate([*e['tangent'], 0.]),
                normal_xyz=rotate([*e['normal_toward_sensor'], 0.]),
                length_m=e['observed_length_m'],
                extension_m=offsets['extension_m'], standoff_m=offsets['standoff_m'])


def geometry_errors(segment, pair):
    """Absolute extension/standoff, perpendicularity and look-direction residuals."""
    t, n = segment['tangent_xyz'], segment['normal_xyz']
    position_residual, extension_residual, standoff_residual, yaw_residual = [], [], [], []
    for sign, c in zip((-1, 1), pair):
        expected_target = [x+sign*.5*segment['length_m']*u for x, u in zip(segment['center_xyz'], t)]
        position_residual.append(math.dist(expected_target, c['look_at_xyz']))
        offset = [a-b for a, b in zip(c['position_xyz'], c['look_at_xyz'])]
        extension_residual.append(abs(sum(a*b for a, b in zip(offset, t))-sign*segment['extension_m']))
        standoff_residual.append(abs(sum(a*b for a, b in zip(offset, n))-segment['standoff_m']))
        heading = math.atan2(-offset[1], -offset[0])
        yaw_residual.append(abs(math.remainder(heading-c['yaw_rad'], 2*math.pi)))
    return dict(target_m=max(position_residual), extension_m=max(extension_residual),
                standoff_m=max(standoff_residual), yaw_rad=max(yaw_residual),
                unit_tangent=abs(math.dist(t, (0, 0, 0))-1),
                unit_normal=abs(math.dist(n, (0, 0, 0))-1),
                perpendicular=abs(sum(a*b for a, b in zip(t, n))))


def median_replay(rows, segments, start):
    model = MedianCandidateFilter()
    records, durations = [], []
    for i in range(start, len(rows)):
        row = rows[i]
        wall = (row.get('raw') or row['side'])['received_wall_sec']
        begin = time.perf_counter_ns()
        d = model.update(segments[i], row['stamp_ns'], wall, source_context(row))
        durations.append((time.perf_counter_ns()-begin)/1e6)
        assert d['valid'] == (d['output'] is not None)
        residual = None
        if d['valid']:
            assert len(d['source_stamps_ns']) == 3
            assert d['source_stamps_ns'][-1] == row['stamp_ns']
            assert all(a < b for a, b in zip(d['source_stamps_ns'], d['source_stamps_ns'][1:]))
            assert 0 <= d['window_age_sec'] <= .35
            assert d['raw_decision']['valid']
            residual = geometry_errors(d['segment'], d['output'])
            assert max(residual.values()) < 1e-9
        records.append(dict(index=i, source_key=signature(row), reference=None,
                            reacquired=d['raw_decision'].get('reacquired', False),
                            geometry_residual=residual, **d))
    return records, durations


def metric_summary(records, rows, score, partial, raw_pairs):
    values = summary(records, rows, score, partial)
    # Output of the median method is reconstructed, not raw. Name metrics accordingly.
    for old, new in (
        ('adjacent_accepted_raw', 'adjacent_output'),
        ('successive_accepted_raw_within_gap', 'successive_output_within_gap'),
        ('successive_accepted_raw_all_gaps', 'successive_output_all_gaps'),
    ):
        values[new] = values.pop(old)
    values.pop('adjacent_accepted_reference')
    values.pop('accepted_raw_to_reference_m')
    values['transition_scope'] = ('Scored indices only. adjacent_output requires consecutive inputs '
                                 'both emitted; successive output may cross invalid input. '
                                 'Different methods accept different subsets; no ground-truth accuracy.')
    valid = [r for r in records if r['index'] >= score and r['valid']]
    values['output_minus_current_raw_position_m'] = stats([
        delta(raw_pairs[r['index']], r['output'])['position_m'] for r in valid])
    values['output_minus_current_raw_target_m'] = stats([
        delta(raw_pairs[r['index']], r['output'])['target_m'] for r in valid])
    values['first_valid_index'] = next((r['index'] for r in records if r['valid']), None)
    ages = [r['window_age_sec'] for r in valid if r.get('window_age_sec') is not None]
    values['median_window_source_age_sec'] = stats(ages)
    values['invalid_rows'] = [dict(index=r['index'], status=r['status']) for r in records
                              if r['index'] >= score and not r['valid']]
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--artifact-root', type=Path, default=ROOT/'local_artifacts')
    parser.add_argument('--benchmark-repeats', type=int, default=5)
    args = parser.parse_args()
    if args.benchmark_repeats < 1:
        parser.error('benchmark repeats must be positive')
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    binary = output/'replay'
    cpp = [ROOT/'tools/endpoint_support_replay.cpp', *(PACKAGE/'src'/name for name in (
        'scan_preprocessor.cpp', 'vehicle_side_estimator.cpp', 'candidate_stability.cpp'))]
    command = ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-pedantic',
               '-I'+str(PACKAGE/'include'), '-I/usr/include/eigen3', *map(str, cpp), '-o', str(binary)]
    subprocess.run(command, check=True)
    paths = cpp+[
        Path(__file__), ROOT/'tools/candidate_median_experiment.py',
        ROOT/'tools/candidate_ema_experiment.py', ROOT/'tools/candidate_frame_math.py',
        ROOT/'tools/evaluate_candidate_ema.py', ROOT/'tools/evaluate_endpoint_support.py',
        *(PACKAGE/'include/amr_reobservation'/name for name in (
            'scan_preprocessor.hpp', 'vehicle_side_estimator.hpp', 'candidate_stability.hpp'))]
    report = dict(compile_command=command, datasets={}, methodology=(
        'Same actual C++ cap20 min/max fits, exact source-stamp odom TF, cold starts/scoring intervals. '
        'baseline=production window gate; retained_last=experimental alpha1 raw gate; '
        'median3=retained-last admission then 3 consecutive accepted segments median/recomputed candidates. '
        'Median history clears on rejection, invalid input, reacquisition, context/offset change, timeout. '
        'Thus median startup may need 5 raw frames versus 3 for comparators. '
        'Known partial labels only label metrics. Actual captures have no endpoint ground truth. '
        'Median is causal but integrates old samples: provenance included, never current-only geometry.'))
    all_durations = []
    for name, relative, start, score, partial in DATASETS:
        capture, dest = args.artifact_root/relative, output/name
        dest.mkdir()
        paths += [capture/'paired.jsonl', capture/'streams.jsonl']
        rows, pairs, fitted, matches = fit_capture(capture, binary, dest)
        segments = [fixed_segment(r, row) for r, row in zip(fitted, rows)]
        for segment, pair in zip(segments, pairs):
            assert (segment is None) == (pair is None)
            if segment is not None:
                assert max(delta(make_candidates(segment), pair).values()) < 1e-9
        baseline = [dict(index=d['row'], source_key=signature(rows[d['row']]),
                         output=pairs[d['row']] if d['valid'] else None,
                         reference=None, reacquired=False, **d)
                    for d in replay_gate(binary, rows, pairs, start)]
        retained, _ = ema_replay(rows, pairs, start, 1.)
        median, _ = median_replay(rows, segments, start)
        # Isolate output filtering: raw admission must match the retained-last control.
        for control, filtered in zip(retained, median):
            assert control['index'] == filtered['index']
            for key in ('valid', 'status', 'consecutive', 'reacquired'):
                assert control[key] == filtered['raw_decision'][key], (name, control['index'], key)
        methods = {'baseline': baseline, 'retained_last': retained, 'median3': median}
        result = dict(paired_count=len(rows), fitted_valid_count=sum(r['estimate'] is not None for r in fitted),
                      exact_tf_count=sum(s is not None for s in segments),
                      recorded_valid_fit_exact_matches=matches,
                      median_raw_admission_matches_retained=True,
                      gate_start_index=start, score_start_index=score, known_partial_indices=partial,
                      methods={})
        for method, records in methods.items():
            write_lines(dest/(method+'.jsonl'), records)
            result['methods'][method] = metric_summary(records, rows, score, partial, pairs)
        if name == 'main':
            mismatch = [r['index'] for r in baseline if r['index'] >= score and
                        (r['status'] if r['valid'] else f"{r['status']}:{r['consecutive']}/3") !=
                        rows[r['index']]['stable_latest']['message']['status']]
            assert not mismatch
            result['baseline_live_status_mismatches'] = mismatch
        # Compare motion on exactly the same accepted consecutive pairs across all methods.
        common = [i for i in range(score, len(rows)) if all(v[i-start]['valid'] for v in methods.values())]
        result['all_methods_common_accepted_count'] = len(common)
        common_set = set(common)
        for method, records in methods.items():
            selected = [{**r, 'valid': r['index'] in common_set} for r in records if r['index'] >= score]
            result['methods'][method]['common_adjacent_output'] = transition_summary(
                transitions(selected, rows, 'output', partial))
        residuals = [r['geometry_residual'] for r in median if r['geometry_residual']]
        result['max_median_geometry_residuals'] = {
            k: max(r[k] for r in residuals) for k in residuals[0]} if residuals else {}
        raw_steps = transitions([dict(index=i, valid=pairs[i] is not None, output=pairs[i])
                                 for i in range(score, len(rows))], rows, 'output', partial)
        result['raw_input_normal_adjacent'] = transition_summary([r for r in raw_steps if not r['partial_related']])
        for _ in range(args.benchmark_repeats):
            _, elapsed = median_replay(rows, segments, start)
            all_durations.extend(elapsed)
        write_json(dest/'report.json', result)
        report['datasets'][name] = result
        print(name, {method: f"{m['accepted']}/{m['scored_count']}" for method, m in result['methods'].items()}, flush=True)
    report['median_python_update_ms'] = stats(all_durations)
    report['timing_scope'] = 'Offline median pipeline Python update only; excludes fit, TF, ROS transport and real-time scheduling.'
    report['sha256'] = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    write_json(output/'report.json', report)


if __name__ == '__main__':
    main()
