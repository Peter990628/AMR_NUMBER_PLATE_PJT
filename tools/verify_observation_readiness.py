#!/usr/bin/env python3
"""Replay saved observations through production selection and the motion decision.

No ROS, inference, publishers, simulator connection, or actual motion. Original
source/receipt/ROS times and each recording's own CFV context are preserved.
Movement is replaced by a counting callback; OCR returns a declared test stub.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from cfv_view_trial import choose_view_snapshot
from evaluate_cfv_axis_recovery import stamp
from recovery_ocr_policy import run_once
from recovery_selection import CandidateSelector, SelectionConfig
from run_recovery_trial import observe_buffered_preview


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def file_info(path):
    return {'path': str(path.resolve()), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def load_recording(folder):
    capture = json.loads((folder / 'capture_report.json').read_text())
    if capture['status'] != 'recorded':
        raise ValueError('incomplete_recording')
    rows = [json.loads(line) for line in (folder / 'previews.jsonl').open()]
    if len(rows) != capture['preview_records'] or not rows:
        raise ValueError('record_count_mismatch')
    if any(b['wall_sec'] <= a['wall_sec'] for a, b in zip(rows, rows[1:])):
        raise ValueError('receipt_order_changed')
    return rows, json.loads((folder / 'context.json').read_text())


def replay(rows, context, *, stop_at_first=False):
    selector = CandidateSelector(SelectionConfig(max_trials=18))
    trace = []
    first = None
    for i, row in enumerate(rows):
        preview = row['preview']
        ns = stamp(preview['header'])
        if ns < context['image_stamp_ns']:
            continue
        selected = observe_buffered_preview(selector, preview, row['wall_sec'], row['now_ros_sec'])
        chosen = choose_view_snapshot(selected, preview, context, row['now_ros_sec'], fresh_close=True)
        candidate = chosen['queue'][0] if chosen['queue'] else None
        # Same final source-age acceptance as the integrated runner.
        if candidate and not -.1 <= row['now_ros_sec'] - stamp(candidate['header']) / 1e9 <= .5:
            raise AssertionError('candidate_not_fresh')
        trace.append({'row_index': i, 'wall_sec': row['wall_sec'], 'source_stamp_ns': ns,
                      'now_ros_sec': row['now_ros_sec'], 'preview_valid': preview['valid'],
                      'preview_status': preview['status'], 'selector_status': selected['status'],
                      'final_status': chosen['status'], 'confirmed_goal': candidate is not None,
                      'tracks': selected['diagnostics']['tracks'],
                      'rejections': selected['diagnostics']['rejections'],
                      'resets': selected['diagnostics']['resets'],
                      'confirmation': candidate['confirmation'] if candidate else None})
        if candidate and first is None:
            first = candidate
            if stop_at_first:
                break
    return trace, first


def control_flow(rows, context):
    """Exercise actual run_once, replacing external work with explicit stubs."""
    moves, states, observed = [], [], []
    selection_trace = []

    def select():
        trace, candidate = replay(rows, context, stop_at_first=True)
        selection_trace.extend(trace)
        return candidate

    def observe(phase):
        observed.append(phase)
        return {'success': False, 'text': None, 'reason': 'not_detected'}

    outcome = run_once(observe, lambda: None, select, moves.append, states.append)
    return {'status': outcome['status'], 'candidate_attempts': outcome['candidate_attempts'],
            'movement_callback_count': len(moves), 'states': states, 'observation_stubs': observed,
            'selected_candidate': moves[0] if moves else None, 'selection_trace': selection_trace}


def verify(missing_folder, recovery_folder, output):
    missing, missing_context = load_recording(missing_folder)
    recovered, recovered_context = load_recording(recovery_folder)
    missing_trace, missing_candidate = replay(missing, missing_context)
    recovered_trace, _ = replay(recovered, recovered_context)
    checks = []

    def check(name, passed, evidence):
        checks.append({'name': name, 'passed': bool(passed), 'evidence': evidence})

    missing_states = Counter(r['final_status'] for r in missing_trace)
    check('all_actual_missing_face_observations_block_goal',
          missing_candidate is None and all(r['preview_valid'] for r in missing_trace)
          and missing_states['no_unique_confirmed_end_face'] > 0
          and set(missing_states) <= {'collecting', 'no_unique_confirmed_end_face'},
          {'observations': len(missing_trace),
           'final_status_counts': dict(missing_states),
           'rough_selector_ready_count': sum(r['selector_status'] == 'ready' for r in missing_trace)})
    # Bound the replayed selection attempt to the runner's 12 wall-second budget.
    attempt = control_flow([r for r in missing if r['wall_sec'] < 12.], missing_context)
    check('missing_face_never_reaches_movement_callback',
          attempt['status'] == 'no_candidate' and attempt['movement_callback_count'] == 0
          and attempt['candidate_attempts'] == 0 and attempt['observation_stubs'] == ['before'],
          {k: v for k, v in attempt.items() if k not in ('selection_trace', 'selected_candidate')})
    save(output / 'missing_attempt.json', attempt)

    episodes = []
    for index, row in enumerate(recovered):
        if row['preview']['valid']:
            continue
        after = next((r for r in recovered_trace if r['row_index'] > index and r['confirmed_goal']), None)
        if after is None:
            episodes.append({'invalid_row': index, 'complete': False})
            continue
        end = after['row_index']
        segment = recovered[index:end + 1]
        evidence = after['confirmation']['source_stamps_ns']
        lost_stamp = stamp(row['preview']['header'])
        # Prefixes include the invalid observation: old confirmation cannot survive.
        prefixes = [control_flow(segment[:n], recovered_context) for n in range(1, len(segment) + 1)]
        counts = [p['movement_callback_count'] for p in prefixes]
        expected = [0] * (len(segment) - 1) + [1]
        episode = {'invalid_row': index, 'ready_row': end, 'complete': True,
                   'valid_observations_until_ready': sum(r['preview']['valid'] for r in segment),
                   'wall_delay_from_invalid_sec': after['wall_sec'] - row['wall_sec'],
                   'wall_delay_from_first_restored_sec': after['wall_sec'] - segment[1]['wall_sec'],
                   'supporting_source_stamps_ns': evidence,
                   'all_evidence_after_invalid': all(s > lost_stamp for s in evidence),
                   'movement_callback_counts_by_prefix': counts,
                   'stages': [r['final_status'] for r in recovered_trace[index:end + 1]]}
        episodes.append(episode)
        check('reconfirm_after_invalid_row_' + str(index),
              counts == expected and len(set(evidence)) >= 3 and all(s > lost_stamp for s in evidence)
              and evidence[-1] == after['source_stamp_ns']
              and after['wall_sec'] - row['wall_sec'] < 12., episode)
    check('actual_recovery_episodes_available', len(episodes) >= 2 and all(e['complete'] for e in episodes),
          {'episode_count': len(episodes)})
    save(output / 'missing_trace.json', missing_trace)
    save(output / 'recovery_trace.json', recovered_trace)
    source_files = [p / name for p in (missing_folder, recovery_folder)
                    for name in ('previews.jsonl', 'context.json', 'capture_report.json')]
    code_files = [Path(__file__), *(Path(__file__).with_name(name) for name in
                  ('recovery_selection.py', 'cfv_view_trial.py', 'cfv_trial_selection.py',
                   'run_recovery_trial.py', 'recovery_ocr_policy.py'))]
    report = {'status': 'passed' if all(c['passed'] for c in checks) else 'failed',
              'checks': checks, 'recovery_episodes': episodes,
              'missing_observations': len(missing), 'recovery_observations': len(recovered),
              'source_files': [file_info(p) for p in source_files],
              'code_files': [file_info(p) for p in code_files],
              'scope': 'Original recorded preview/CFV selection replay and production run_once with intercepted movement.',
              'actual_motion_commands': 0, 'ros_connection_created': False,
              'limitations': ['Recordings are separate Play/request epochs and are never concatenated.',
                             'Saved CFV predictions are reused at their original recorded times; no new inference.',
                             'Recovery episodes here follow invalid previews, not all natural partial-sector failures.',
                             'Movement callback only: no path/controller/DDS/actuator or OCR accuracy validation.',
                             'Confirmation tracking here is before goal freeze; motion-time perception policy is unchanged.',
                             'Replay uses recorded receipt and ROS times, not live executor backlog scheduling.']}
    save(output / 'report.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--missing-recording', type=Path, required=True)
    parser.add_argument('--recovery-recording', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = verify(args.missing_recording, args.recovery_recording, args.output)
    print(json.dumps({'status': report['status'], 'checks': report['checks']}, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
