"""Source-frame measurements for one stationary simulated recovery trial.

This measures a detector's observation-quality proxy, not OCR or identity.
Create a new window after each arrival and settling period. Only source images
strictly newer than ``cutoff_ros_ns`` can count. Count samples by source header,
not callbacks or PlateObservation.consecutive_count (which saturates at three).

``add`` returning False does NOT mean a message is irrelevant to control health:
the runner must inspect every latest input_valid/status and keep its own receive
wall watchdog. In particular, a watchdog invalidation with an already counted
image stamp revokes that sample, even when its source image has since aged out.
There is no timer, ROS I/O, velocity publisher or autonomous control here.
"""

from collections import Counter
from copy import deepcopy
import math
from numbers import Integral, Real
from statistics import median


def _integer(value):
    return isinstance(value, Integral) and not isinstance(value, bool)


def _finite(value):
    return (isinstance(value, Real) and not isinstance(value, bool)
            and math.isfinite(value))


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered)-1)*fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    return ordered[lower]+(ordered[upper]-ordered[lower])*(index-lower)


def match_measurement_image(image_header, image_width, image_height, window,
                            observations):
    """Return owned metadata only for an exact, valid measured source image.

    ``observations`` maps (frame_id, stamp_ns) to original PlateObservation
    dictionaries. An image is evidence only when that exact source is retained
    as valid in ``window`` with the same resolution and result. No nearby stamp,
    latest-value fallback, or previously invalidated source is accepted. Window
    insertion already checks source freshness; this matcher is also suitable for
    saving historical evidence after a measurement has finished.
    """
    if (not isinstance(window, ObservationWindow) or window.stream_error is not None
            or not isinstance(observations, dict) or not isinstance(image_header, dict)
            or not _integer(image_width) or not _integer(image_height)
            or image_width <= 0 or image_height <= 0):
        return None
    stamp = image_header.get('stamp')
    frame = image_header.get('frame_id')
    if not isinstance(stamp, dict) or not isinstance(frame, str) or not frame:
        return None
    sec, nsec = stamp.get('sec'), stamp.get('nanosec')
    if (not _integer(sec) or sec < 0 or not _integer(nsec)
            or not 0 <= nsec < 1_000_000_000):
        return None
    stamp_ns = int(sec)*1_000_000_000+int(nsec)
    if stamp_ns <= window.cutoff_ros_ns:
        return None
    key = (frame, stamp_ns)
    sample, original = window.samples.get(key), observations.get(key)
    if (sample is None or sample['input_valid'] is not True
            or not isinstance(original, dict) or original.get('input_valid') is not True):
        return None
    original_header = original.get('header')
    if (not isinstance(original_header, dict)
            or original_header.get('frame_id') != frame
            or original_header.get('stamp') != stamp):
        return None
    resolution = (image_width, image_height)
    if (window.resolution != resolution
            or (sample['image_width'], sample['image_height']) != resolution
            or (original.get('image_width'), original.get('image_height')) != resolution):
        return None
    # The keyed dictionary must describe the result actually measured, rather
    # than another callback/retry carrying the same source header.
    for field in ('status', 'detected', 'usable', 'confidence', 'width', 'height',
                  'border_margin_ratio'):
        if original.get(field) != sample[field]:
            return None
    return deepcopy(original)


class ObservationWindow:
    """Accumulate bounded-trial evidence without accepting stale successes.

    Frame or resolution changes, backwards wall/ROS time, and conflicting valid
    results for one source frame latch ``stream_error``. Start a new window after
    diagnosing those changes; they must not silently splice different streams.
    An invalidated source frame cannot become valid again through a retry.

    ``complete`` requires min_samples *valid* distinct images and min_span_sec
    between their source stamps. The usable fraction denominator includes every
    retained source frame, including invalid images, so faults cannot improve it.
    """

    def __init__(self, cutoff_ros_ns, min_samples=12, min_span_sec=1.0,
                 max_age_sec=.5):
        if not _integer(cutoff_ros_ns) or cutoff_ros_ns < 0:
            raise ValueError("cutoff_ros_ns must be a nonnegative integer")
        if not _integer(min_samples) or min_samples < 1:
            raise ValueError("min_samples must be a positive integer")
        if not _finite(min_span_sec) or min_span_sec < 0:
            raise ValueError("min_span_sec must be finite and nonnegative")
        if not _finite(max_age_sec) or max_age_sec <= 0:
            raise ValueError("max_age_sec must be finite and positive")
        self.cutoff_ros_ns = int(cutoff_ros_ns)
        self.min_samples = int(min_samples)
        self.min_span_sec = float(min_span_sec)
        self.max_age_ns = int(max_age_sec*1_000_000_000)
        self.samples = {}
        self.errors = Counter()
        self.status_events = Counter()
        self.event_count = 0
        self.duplicate_count = 0
        self.invalidation_count = 0
        self.stream_error = None
        self.frame = None
        self.resolution = None
        self._last_now_ns = None
        self._last_receipt_wall = None
        self._latest_input_valid = False

    def _reject(self, reason, *, fatal=False):
        self.errors[reason] += 1
        if fatal and self.stream_error is None:
            self.stream_error = reason
        return False

    def add(self, message_dict, now_ros_ns, receipt_wall_sec):
        """Return True only when one previously unseen source frame was added.

        Fresh invalid images may count as invalid source samples. Duplicate
        invalidations replace the prior record but return False. Rejected input
        is exposed in errors, with stream changes additionally latching a fault.
        The runner must not use this return value as a motion-health predicate.
        """
        self.event_count += 1
        if (not _integer(now_ros_ns) or now_ros_ns < 0
                or not _finite(receipt_wall_sec) or receipt_wall_sec < 0):
            return self._reject('invalid_receive_time', fatal=True)
        if self._last_now_ns is not None and now_ros_ns < self._last_now_ns:
            return self._reject('ros_time_regression', fatal=True)
        if (self._last_receipt_wall is not None
                and receipt_wall_sec < self._last_receipt_wall):
            return self._reject('receipt_time_regression', fatal=True)
        self._last_now_ns = int(now_ros_ns)
        self._last_receipt_wall = float(receipt_wall_sec)
        if not isinstance(message_dict, dict):
            return self._reject('invalid_message')
        status = message_dict.get('status')
        if not isinstance(status, str) or not status:
            return self._reject('invalid_status')
        self.status_events[status] += 1
        input_valid = message_dict.get('input_valid')
        detected = message_dict.get('detected')
        usable = message_dict.get('usable')
        if not all(isinstance(flag, bool) for flag in (input_valid, detected, usable)):
            return self._reject('invalid_flags')
        if usable and (not input_valid or not detected or status != 'usable'):
            return self._reject('inconsistent_usable')
        header = message_dict.get('header', {})
        if not isinstance(header, dict):
            return self._reject('invalid_header')
        stamp = header.get('stamp', {})
        if not isinstance(stamp, dict):
            return self._reject('invalid_stamp')
        sec, nsec = stamp.get('sec'), stamp.get('nanosec')
        if (not _integer(sec) or sec < 0 or not _integer(nsec)
                or not 0 <= nsec < 1_000_000_000):
            return self._reject('invalid_stamp')
        stamp_ns = int(sec)*1_000_000_000+int(nsec)
        if stamp_ns <= self.cutoff_ros_ns:
            return self._reject('pre_cutoff')
        frame = header.get('frame_id')
        if not isinstance(frame, str) or not frame:
            return self._reject('empty_frame')
        if self.frame is not None and frame != self.frame:
            return self._reject('frame_changed', fatal=True)
        width, height = (message_dict.get(name)
                         for name in ('image_width', 'image_height'))
        if (not _integer(width) or not _integer(height)
                or width < 0 or height < 0 or (input_valid and (not width or not height))):
            return self._reject('invalid_resolution')
        resolution = (int(width), int(height))
        if (width and height and self.resolution is not None
                and resolution != self.resolution):
            return self._reject('resolution_changed', fatal=True)
        key = (frame, stamp_ns)
        existing = self.samples.get(key)
        # A timeout notification deliberately carries the old image stamp. It
        # must still revoke an already-counted sample after max_age has passed.
        duplicate_invalidation = existing is not None and not input_valid
        if stamp_ns > now_ros_ns:
            return self._reject('future_image')
        if now_ros_ns-stamp_ns > self.max_age_ns and not duplicate_invalidation:
            return self._reject('stale_image')
        metrics = {}
        for name in ('confidence', 'width', 'height', 'border_margin_ratio'):
            value = message_dict.get(name, 0.0)
            if not _finite(value):
                return self._reject('invalid_metrics')
            metrics[name] = float(value)
        if (not 0 <= metrics['confidence'] <= 1 or metrics['width'] < 0
                or metrics['height'] < 0
                or (detected and (metrics['width'] == 0 or metrics['height'] == 0))):
            return self._reject('invalid_metrics')
        record = dict(frame_id=frame, stamp_ns=stamp_ns, input_valid=input_valid,
                      detected=detected, usable=usable, status=status,
                      image_width=int(width), image_height=int(height), **metrics)
        self._latest_input_valid = input_valid
        if existing is not None:
            self.duplicate_count += 1
            if not input_valid:
                self.invalidation_count += 1
                record['receipt_wall_sec'] = existing['receipt_wall_sec']
                record['latest_receipt_wall_sec'] = float(receipt_wall_sec)
                self.samples[key] = record
            elif not existing['input_valid']:
                self._latest_input_valid = False
                self._reject('retry_after_invalidation')
            else:
                comparable = {k: v for k, v in existing.items()
                              if k not in ('receipt_wall_sec', 'latest_receipt_wall_sec')}
                if comparable != record:
                    return self._reject('conflicting_duplicate', fatal=True)
            return False
        self.frame = frame
        if width and height:
            self.resolution = resolution
        record['receipt_wall_sec'] = float(receipt_wall_sec)
        record['latest_receipt_wall_sec'] = float(receipt_wall_sec)
        self.samples[key] = record
        return True

    def summary(self):
        """Return JSON-serializable evidence and the observation-quality verdict.

        This is a historical window summary. It cannot detect silence since the
        last add call; the runner must AND recovered with its current watchdogs.
        """
        records = sorted(self.samples.values(), key=lambda item: item['stamp_ns'])
        valid = [item for item in records if item['input_valid']]
        detections = [item for item in valid if item['detected']]
        usable_count = sum(item['input_valid'] and item['usable'] for item in records)
        source_span = ((valid[-1]['stamp_ns']-valid[0]['stamp_ns'])/1e9
                       if len(valid) > 1 else 0.0)
        receipt_times = [item['receipt_wall_sec'] for item in records]
        receipt_span = max(receipt_times)-min(receipt_times) if records else 0.0
        longest = current_run = 0
        for item in records:
            current_run = current_run+1 if item['input_valid'] and item['usable'] else 0
            longest = max(longest, current_run)
        count = len(records)
        usable_fraction = usable_count/count if count else 0.0
        complete = (self.stream_error is None and len(valid) >= self.min_samples
                    and source_span+1e-12 >= self.min_span_sec)
        recovered = (complete and usable_fraction >= .8 and longest >= 3
                     and self._latest_input_valid)
        confidence = [item['confidence'] for item in detections]
        def middle(field):
            values = [item[field] for item in detections]
            return median(values) if values else None
        return {
            'complete': complete, 'recovered': recovered,
            'sample_count': count, 'valid_count': len(valid),
            'invalid_count': count-len(valid), 'detected_count': len(detections),
            'usable_count': usable_count, 'usable_fraction': usable_fraction,
            'valid_fraction': len(valid)/count if count else 0.0,
            'detected_fraction': len(detections)/count if count else 0.0,
            'longest_usable_run': longest,
            'source_span_sec': source_span, 'receipt_span_sec': receipt_span,
            'first_source_stamp_ns': records[0]['stamp_ns'] if records else None,
            'last_source_stamp_ns': records[-1]['stamp_ns'] if records else None,
            'confidence_median': middle('confidence'),
            'confidence_p10': _percentile(confidence, .1),
            'width_median_px': middle('width'), 'height_median_px': middle('height'),
            'border_margin_median_ratio': middle('border_margin_ratio'),
            'status_counts': dict(Counter(item['status'] for item in records)),
            'status_event_counts': dict(self.status_events),
            'event_count': self.event_count, 'duplicate_count': self.duplicate_count,
            'invalidation_count': self.invalidation_count,
            'latest_input_valid': self._latest_input_valid,
            'errors': dict(self.errors), 'stream_error': self.stream_error,
            'frame_id': self.frame,
            'image_resolution': list(self.resolution) if self.resolution else None,
            'cutoff_ros_ns': self.cutoff_ros_ns,
            'requirements': {'min_valid_samples': self.min_samples,
                             'min_source_span_sec': self.min_span_sec,
                             'max_image_age_sec': self.max_age_ns/1e9,
                             'min_usable_fraction': .8,
                             'min_longest_usable_run': 3},
            'meaning': 'Plate observation quality proxy; not OCR or identity confirmation.',
        }
