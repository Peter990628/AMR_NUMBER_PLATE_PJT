"""Truth-free stopping policy for one stationary, exactly paired OCR window."""
import math
import re

from plate_evaluation import source_key
from probe_plate_ocr import normalize_text

DEFAULT_POLICY = {'consecutive': 3, 'min_ocr_confidence': .8,
                  'max_source_gap_sec': 1.5, 'min_box_iou': .3}
FAULT_REASONS = ('capture_invalid', 'no_valid_observations', 'incomplete_slot_sequence',
                 'duplicate_or_reordered_source', 'stream_changed')


def box_iou(a, b):
    left, top = max(a['x'], b['x']), max(a['y'], b['y'])
    right = min(a['x']+a['width'], b['x']+b['width'])
    bottom = min(a['y']+a['height'], b['y']+b['height'])
    overlap = max(0., right-left)*max(0., bottom-top)
    union = a['width']*a['height']+b['width']*b['height']-overlap
    return overlap/union if union > 0 else 0.


def decide(capture, rows, policy=None):
    """Only a valid trailing streak can succeed; earlier success is not latched.

    This accepts simple 2/3-digit + Hangul + 4-digit plates, not all Korean
    registration formats. Agreement is evidence, never proof of correct text.
    """
    p = dict(DEFAULT_POLICY if policy is None else policy)
    if (type(p['consecutive']) is not int or not 2 <= p['consecutive'] <= 10
            or not all(math.isfinite(p[k]) for k in
                       ('min_ocr_confidence', 'max_source_gap_sec', 'min_box_iou'))
            or not 0 <= p['min_ocr_confidence'] <= 1
            or not 0 < p['max_source_gap_sec'] <= 5
            or not 0 <= p['min_box_iou'] <= 1):
        raise ValueError('invalid_ocr_policy')
    result = dict(success=False, text=None, reason='no_consistent_read',
                  policy=p, trailing_count=0, supporting_slots=[], rows_considered=len(rows))
    if capture.get('capture_fault') or capture.get('clock_advanced') is not True:
        return {**result, 'reason': 'capture_invalid'}
    count = capture['sampling']['count']
    if [r['slot'] for r in rows] != list(range(count)):
        return {**result, 'reason': 'incomplete_slot_sequence'}
    text = None
    previous = None
    support = []
    seen = set()
    last_stamp = None
    stream = None
    ready = 0
    for row in rows:
        key = None
        if row.get('image_header'):
            key = source_key(row['image_header'])
            # Duplicate/reordered frames or a changed stream invalidate the window.
            if key in seen or (last_stamp is not None and key[1] <= last_stamp):
                return {**result, 'reason': 'duplicate_or_reordered_source'}
            current_stream = (key[0], tuple(row['image_size_wh']))
            if stream is not None and current_stream != stream:
                return {**result, 'reason': 'stream_changed'}
            seen.add(key); last_stamp = key[1]; stream = current_stream
        confidence = row.get('recognition_confidence')
        value = normalize_text(row.get('raw_text') or '')
        good = (key is not None and row.get('status') == 'recognized'
                and isinstance(confidence, (float, int)) and math.isfinite(confidence)
                and p['min_ocr_confidence'] <= confidence <= 1
                and re.fullmatch(r'[0-9]{2,3}[가-힣][0-9]{4}', value) is not None)
        if row.get('status') in ('recognized', 'not_detected'):
            ready += 1
        if not good:
            text = previous = None; support = []
            continue
        continuous = (previous is not None and not row.get('barrier_since_previous')
                      and value == text
                      and 0 < (key[1]-source_key(previous['image_header'])[1])/1e9 <= p['max_source_gap_sec']
                      and box_iou(row['observation'], previous['observation']) >= p['min_box_iou'])
        support = support+[row['slot']] if continuous else [row['slot']]
        text = value; previous = row
    result.update(trailing_count=len(support), supporting_slots=support)
    if len(support) >= p['consecutive']:
        result.update(success=True, text=text, reason='consistent_ocr')
    elif not ready:
        result['reason'] = 'no_valid_observations'
    elif not any(r.get('status') == 'recognized' for r in rows):
        result['reason'] = 'not_detected'
    return result


def run_once(observe, prepare, select, move, state, *, execute=True):
    """Shared one-candidate control flow. Exceptions abort; never move on faults."""
    state('observe_initial')
    before = observe('before')
    if before['success']:
        state('succeeded')
        return {'status': 'succeeded_without_motion', 'success': True, 'text': before['text'],
                'before': before, 'after': None, 'candidate_attempts': 0}
    if before['reason'] in FAULT_REASONS:
        raise RuntimeError('initial_ocr_'+before['reason'])
    state('prepare_fresh_direction')
    prepare()
    state('select_candidate')
    candidate = select()
    if candidate is None:
        return {'status': 'no_candidate', 'success': False, 'text': None,
                'before': before, 'after': None, 'candidate_attempts': 0}
    if not execute:
        state('selected_only')
        return {'status': 'selected_only', 'success': False, 'text': None,
                'before': before, 'after': None, 'candidate_attempts': 0}
    state('moving')
    move(candidate)
    state('observe_candidate')
    after = observe('after')
    if after['reason'] in FAULT_REASONS:
        raise RuntimeError('candidate_ocr_'+after['reason'])
    state('succeeded' if after['success'] else 'failed')
    return {'status': 'succeeded_after_motion' if after['success'] else 'candidate_exhausted',
            'success': after['success'], 'text': after['text'], 'before': before, 'after': after,
            'candidate_attempts': 1}
