"""Bounded, offline-testable storage for the existing-graph GUI reader."""
import collections
import json
import math


class DropoutRecorder:
    def __init__(self, output, pre_seconds=3., post_seconds=5., wait_seconds=120.,
                 max_ring_bytes=16 * 1024 * 1024):
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
        negative = positive = 0
        start = flat['outputs:azimuthRange'][0]
        step = flat['outputs:horizontalResolution']
        for i, distance in enumerate(ranges):
            d = float(distance)
            if math.isfinite(d) and .1 <= d <= 6.:
                angle = math.radians(start + i * step)
                negative += int(angle < -.01)
                positive += int(angle > .01)
        sector = ('both' if negative > 10 and positive > 10 else
                  'negative_only' if negative > 10 else 'positive_only' if positive > 10 else 'neither')
        return {'sample_index': event['sample_index'], 'wall_sec': event['wall_sec'],
                'wall_time_ns': event['wall_time_ns'], 'stamp_ns_rounded': stamp_ns,
                'negative_count': negative, 'positive_count': positive, 'sector': sector,
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

    def observe(self, event, payloads):
        wall = event['wall_sec']
        row = self.scan_row(event)
        if row:
            self.complete_scan_count += 1
            self.sectors[row['sector']] += 1
            if row['sector'] == 'both':
                self.good_streak += 1
                self.armed |= self.good_streak >= 3
            else:
                self.good_streak = 0
                if self.armed and self.trigger is None:
                    self.trigger = dict(row)
            row['armed'] = self.armed
            self.scans.write(json.dumps(row, allow_nan=False) + '\n')
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
            self.flush_ring()
            self.write_event(encoded, payloads)
            if wall - self.trigger['wall_sec'] >= self.post_seconds:
                self.finished_reason = 'event_window_complete'
        return self.finished_reason is not None

    def summary(self):
        return {'armed_after_three_normal_scans': self.armed, 'trigger': self.trigger,
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
