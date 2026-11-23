"""Bounded, offline-testable storage for the existing-graph GUI reader."""
import collections
import json
import math
from statistics import median


class ScanCountDrop:
    """Detect an early fall in the Create 3 failure trace, on distinct complete scans."""

    def __init__(self):
        self.history = collections.deque(maxlen=10)
        self.last_stamp = None
        self.first_low = None
        self.low_streak = 0

    def observe(self, stamp_ns, count):
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            return None
        if self.last_stamp is not None and stamp_ns - self.last_stamp > 250_000_000:
            self.first_low = None
            self.low_streak = 0
        self.last_stamp = stamp_ns
        baseline = self.first_low['previous_median'] if self.first_low else (
            median(self.history) if len(self.history) >= 3 else None)
        self.history.append(count)
        if baseline is not None and count < 220 and baseline - count >= max(30, .25 * baseline):
            if self.first_low is None:
                self.first_low = {'reason': 'point_count_drop', 'stamp_ns_rounded': stamp_ns,
                                  'valid_points': count, 'previous_median': baseline}
            self.low_streak += 1
            if self.low_streak == 3:
                return {**self.first_low, 'confirmed_stamp_ns': stamp_ns, 'confirmed_points': count}
        else:
            self.first_low = None
            self.low_streak = 0
        return None


class DropoutRecorder:
    def __init__(self, output, pre_seconds=3., post_seconds=5., wait_seconds=120.,
                 max_ring_bytes=16 * 1024 * 1024, count_drop=False):
        self.pre_seconds = pre_seconds
        self.post_seconds = post_seconds
        self.wait_seconds = wait_seconds
        self.max_ring_bytes = max_ring_bytes
        self.ring = collections.deque()
        self.ring_bytes = 0
        self.peak_ring_bytes = 0
        self.evicted_for_size = 0
        self.last_stamp = None
        self.good_streak = 0
        self.armed = False
        self.count_drop = ScanCountDrop() if count_drop else None
        self.last_complete_timeline = None
        self.trigger = None
        self.saved_samples = 0
        self.saved_first_wall = None
        self.saved_last_wall = None
        self.complete_scan_count = 0
        self.sectors = collections.Counter()
        self.finished_reason = None
        self.frames = (output / 'frames.jsonl').open('w', buffering=1)
        self.payload_file = (output / 'gpu_payloads.bin').open('wb')
        self.scans = (output / 'scan_summary.jsonl').open('w', buffering=1)

    def scan_row(self, event):
        flat = next(n['values'] for n in event['nodes'] if n['type'].endswith('.IsaacComputeRTXLidarFlatScan'))
        pub = next(n['values'] for n in event['nodes'] if n['type'].endswith('.ROS2PublishLaserScan'))
        if not flat['outputs:numCols']:
            return None  # Old arrays retained between complete scans are not observations.
        stamp = float(pub['inputs:timeStamp'])
        if not math.isfinite(stamp):
            raise ValueError('Non-finite publisher timestamp')
        stamp_ns = round(stamp * 1e9)
        if self.last_stamp is not None:
            if stamp_ns < self.last_stamp:
                raise ValueError('Simulation timestamp rolled back')
            if stamp_ns == self.last_stamp:
                return None
        self.last_stamp = stamp_ns
        ranges = flat['outputs:linearDepthData']
        if len(ranges) != flat['outputs:numCols']:
            raise ValueError('Complete scan array size mismatch')
        negative = positive = valid_points = 0
        start = flat['outputs:azimuthRange'][0]
        step = flat['outputs:horizontalResolution']
        near, far = flat['outputs:depthRange'] if self.count_drop else (.1, 6.)
        for i, distance in enumerate(ranges):
            d = float(distance)
            if math.isfinite(d) and near <= d <= far:
                valid_points += 1
                angle = math.radians(start + i * step)
                negative += int(angle < -.01)
                positive += int(angle > .01)
        sector = ('both' if negative > 10 and positive > 10 else
                  'negative_only' if negative > 10 else 'positive_only' if positive > 10 else 'neither')
        return {'sample_index': event['sample_index'], 'wall_sec': event['wall_sec'],
                'wall_time_ns': event['wall_time_ns'], 'stamp_ns_rounded': stamp_ns,
                'negative_count': negative, 'positive_count': positive, 'sector': sector,
                'valid_points': valid_points,
                'flat_equals_publisher_input': ranges == pub['inputs:linearDepthData']}

    def write_event(self, encoded, payloads):
        event = json.loads(encoded)
        for name, data in payloads.items():
            event['gpu_payloads'][name]['offset'] = self.payload_file.tell()
            self.payload_file.write(data)
        self.frames.write(json.dumps(event, allow_nan=False) + '\n')
        self.saved_samples += 1
        if self.saved_first_wall is None:
            self.saved_first_wall = event['wall_sec']
        self.saved_last_wall = event['wall_sec']

    def flush_ring(self):
        while self.ring:
            _, encoded, payloads, size = self.ring.popleft()
            self.write_event(encoded, payloads)
            self.ring_bytes -= size

    def observe(self, event, payloads, ros_trigger=None):
        wall = event['wall_sec']
        row = self.scan_row(event)
        if row:
            self.complete_scan_count += 1
            self.sectors[row['sector']] += 1
            if self.count_drop:
                drop = self.count_drop.observe(row['stamp_ns_rounded'], row['valid_points'])
                self.armed = self.complete_scan_count >= 3
                self.last_complete_timeline = event['timeline_seconds']
                if drop and self.trigger is None:
                    self.trigger = {**drop, 'source': 'isaac_flat_scan', 'wall_sec': wall,
                                    'confirmation_sample_index': event['sample_index']}
            elif row['sector'] == 'both':
                self.good_streak += 1
                self.armed |= self.good_streak >= 3
            else:
                self.good_streak = 0
                if self.armed and self.trigger is None:
                    self.trigger = dict(row)
            row['armed'] = self.armed
            self.scans.write(json.dumps(row, allow_nan=False) + '\n')
        if self.count_drop and self.trigger is None:
            if ros_trigger:
                self.trigger = {**ros_trigger, 'source': 'ros_scan', 'wall_sec': wall}
            elif self.armed and event['timeline_seconds'] - self.last_complete_timeline >= .5:
                self.trigger = {'source': 'isaac_flat_scan', 'reason': 'no_new_complete_scan',
                                'wall_sec': wall, 'last_scan_stamp_ns': self.last_stamp,
                                'timeline_seconds': event['timeline_seconds']}
        encoded = json.dumps(event, allow_nan=False).encode()
        if self.trigger is None:
            size = len(encoded) + sum(map(len, payloads.values()))
            self.ring.append((wall, encoded, payloads, size))
            self.ring_bytes += size
            while self.ring and (wall - self.ring[0][0] > self.pre_seconds or self.ring_bytes > self.max_ring_bytes):
                self.evicted_for_size += int(self.ring_bytes > self.max_ring_bytes)
                self.ring_bytes -= self.ring.popleft()[3]
            self.peak_ring_bytes = max(self.peak_ring_bytes, self.ring_bytes)
            if wall >= self.wait_seconds:
                self.flush_ring()  # Keep a bounded normal tail if no event occurred.
                self.finished_reason = 'no_dropout_within_wait'
        else:
            while self.ring and wall - self.ring[0][0] > self.pre_seconds:
                self.ring_bytes -= self.ring.popleft()[3]
            self.flush_ring()
            self.write_event(encoded, payloads)
            if wall - self.trigger['wall_sec'] >= self.post_seconds:
                self.finished_reason = 'event_window_complete'
        return self.finished_reason is not None

    def summary(self):
        armed_key = 'armed_after_three_scans' if self.count_drop else 'armed_after_three_normal_scans'
        return {armed_key: self.armed, 'trigger': self.trigger,
                'trigger_detected': self.trigger is not None, 'complete_scan_count': self.complete_scan_count,
                'sector_counts': dict(self.sectors), 'saved_sample_count': self.saved_samples,
                'saved_wall_range_sec': [self.saved_first_wall, self.saved_last_wall],
                'ring_bytes': self.ring_bytes, 'peak_ring_bytes': self.peak_ring_bytes,
                'ring_size_evictions': self.evicted_for_size,
                'gpu_payload_bytes': self.payload_file.tell(), 'finished_reason': self.finished_reason}

    def close(self):
        self.flush_ring()
        result = self.summary()
        self.frames.close()
        self.payload_file.close()
        self.scans.close()
        return result
