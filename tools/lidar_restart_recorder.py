"""Bounded storage for a user-driven Stop -> Play diagnostic (no Isaac imports)."""

from collections import Counter, deque
import json
import math


class RestartRecorder:
    def __init__(self, output, pre_seconds=3., post_seconds=60., wait_seconds=120.,
                 max_ring_bytes=16 * 1024 * 1024, max_saved_bytes=96 * 1024 * 1024):
        self.pre_seconds, self.post_seconds, self.wait_seconds = pre_seconds, post_seconds, wait_seconds
        self.max_ring_bytes, self.max_saved_bytes = max_ring_bytes, max_saved_bytes
        self.ring = deque()
        self.ring_bytes = self.peak_ring_bytes = self.size_evictions = 0
        self.saved_bytes = self.saved_samples = 0
        self.stop_wall = self.restart_wall = None
        self.timeline_epoch = self.scan_clock_epoch = 0
        self.last_stamp = None
        self.finished_reason = None
        self.scan_counts = Counter()
        self.sectors = Counter()
        self.ready = False
        self.frames = (output / 'frames.jsonl').open('wb')
        self.payloads = (output / 'gpu_payloads.bin').open('wb')
        self.scans = (output / 'scan_summary.jsonl').open('wb')
        self.events = (output / 'events.jsonl').open('wb')

    @staticmethod
    def encode(value):
        return (json.dumps(value, allow_nan=False) + '\n').encode()

    def write_jsonl(self, stream, value):
        data = self.encode(value)
        if self.saved_bytes + len(data) > self.max_saved_bytes:
            self.finished_reason = 'size_limit'
            return False
        stream.write(data)
        stream.flush()
        self.saved_bytes += len(data)
        return True

    def note(self, kind, wall, wall_time_ns, **details):
        event = dict(kind=kind, wall_sec=wall, wall_time_ns=wall_time_ns, **details)
        if kind == 'stop':
            self.timeline_epoch += 1
            self.last_stamp = None
            if self.stop_wall is None:
                self.stop_wall = wall
                while self.ring and wall - self.ring[0][0] > self.pre_seconds:
                    self.ring_bytes -= self.ring.popleft()[3]
                self.flush_ring()
            elif self.restart_wall is not None:
                self.finished_reason = 'additional_stop'
        elif kind == 'play' and self.stop_wall is not None and self.restart_wall is None:
            self.restart_wall = wall
        event['timeline_epoch'] = self.timeline_epoch
        self.write_jsonl(self.events, event)

    def write_sample(self, encoded, blobs):
        event = json.loads(encoded)
        offset = self.payloads.tell()
        for name, data in blobs.items():
            event['gpu_payloads'][name]['offset'] = offset
            offset += len(data)
        encoded = self.encode(event)
        size = len(encoded) + sum(map(len, blobs.values()))
        if self.saved_bytes + size > self.max_saved_bytes:
            self.finished_reason = 'size_limit'
            return False
        for data in blobs.values():
            self.payloads.write(data)
        self.frames.write(encoded)
        self.saved_bytes += size
        self.saved_samples += 1
        return True

    def flush_ring(self):
        while self.ring:
            _, encoded, blobs, size = self.ring.popleft()
            self.ring_bytes -= size
            if self.finished_reason != 'size_limit':
                self.write_sample(encoded, blobs)

    def scan_row(self, event):
        if not event.get('playing') or not event.get('nodes'):
            return None
        try:
            flat = next(n['values'] for n in event['nodes']
                        if n['type'].endswith('.IsaacComputeRTXLidarFlatScan'))
            pub = next(n['values'] for n in event['nodes']
                       if n['type'].endswith('.ROS2PublishLaserScan'))
            cols = int(flat['outputs:numCols'])
            if cols == 0:
                return None
            ranges = flat['outputs:linearDepthData']
            if cols < 0 or len(ranges) != cols:
                raise ValueError('complete scan array size mismatch')
            stamp = float(pub['inputs:timeStamp'])
            if not math.isfinite(stamp):
                raise ValueError('non-finite publisher timestamp')
            stamp_ns = round(stamp * 1e9)
            if self.last_stamp is not None and stamp_ns < self.last_stamp:
                self.scan_clock_epoch += 1
                self.note('publisher_clock_rollback', event['wall_sec'], event['wall_time_ns'],
                          previous_stamp_ns=self.last_stamp, stamp_ns=stamp_ns)
            elif stamp_ns == self.last_stamp:
                return None
            self.last_stamp = stamp_ns
            start = float(flat['outputs:azimuthRange'][0])
            step = float(flat['outputs:horizontalResolution'])
            counts = Counter()
            for i, distance in enumerate(ranges):
                d = float(distance)
                if not math.isfinite(d) or not .1 <= d <= 6.:
                    continue
                angle = start + i * step
                counts['negative'] += angle < -math.degrees(.01)
                counts['positive'] += angle > math.degrees(.01)
                # Fixed audit sectors from the previous normal/restart comparison.
                counts['minus20_to_minus2'] += -20. <= angle <= -2.
                counts['plus2_to_plus30'] += 2. <= angle <= 30.
            sector = ('both' if counts['negative'] > 10 and counts['positive'] > 10 else
                      'negative_only' if counts['negative'] > 10 else
                      'positive_only' if counts['positive'] > 10 else 'neither')
            phase = 'before_stop' if self.stop_wall is None else 'after_restart'
            self.scan_counts[phase] += 1
            self.sectors[phase + ':' + sector] += 1
            return dict(sample_index=event['sample_index'], wall_sec=event['wall_sec'],
                        wall_time_ns=event['wall_time_ns'], timeline_epoch=self.timeline_epoch,
                        scan_clock_epoch=self.scan_clock_epoch, stamp_ns=stamp_ns, phase=phase,
                        sector=sector, counts=dict(counts),
                        flat_equals_publisher_input=ranges == pub['inputs:linearDepthData'])
        except (KeyError, TypeError, ValueError, StopIteration, IndexError) as exc:
            # Preserve incomplete initialization metadata; do not mislabel it as a scan.
            return dict(sample_index=event['sample_index'], wall_sec=event['wall_sec'],
                        timeline_epoch=self.timeline_epoch, scan_read_error=str(exc))

    def observe(self, event, blobs):
        if self.finished_reason:
            return True
        event = dict(event, timeline_epoch=self.timeline_epoch)
        row = self.scan_row(event)
        if row is not None:
            self.write_jsonl(self.scans, row)
        self.ready |= (self.stop_wall is None and event['wall_sec'] >= self.pre_seconds
                       and self.scan_counts['before_stop'] > 0)
        encoded = self.encode(event)
        if self.stop_wall is None:
            size = len(encoded) + sum(map(len, blobs.values()))
            self.ring.append((event['wall_sec'], encoded, blobs, size))
            self.ring_bytes += size
            while self.ring and (event['wall_sec'] - self.ring[0][0] > self.pre_seconds
                                 or self.ring_bytes > self.max_ring_bytes):
                self.size_evictions += self.ring_bytes > self.max_ring_bytes
                self.ring_bytes -= self.ring.popleft()[3]
            self.peak_ring_bytes = max(self.peak_ring_bytes, self.ring_bytes)
        elif self.finished_reason != 'size_limit':
            self.write_sample(encoded, blobs)
        return self.tick(event['wall_sec'])

    def tick(self, wall):
        if self.finished_reason:
            return True
        if self.restart_wall is not None and wall - self.restart_wall >= self.post_seconds:
            self.finished_reason = 'restart_window_complete'
        elif self.restart_wall is None and wall >= self.wait_seconds:
            self.finished_reason = 'no_stop_play_within_wait'
        return self.finished_reason is not None

    def summary(self):
        return dict(finished_reason=self.finished_reason, ready_for_stop_play=self.ready,
                    stop_wall_sec=self.stop_wall, restart_wall_sec=self.restart_wall,
                    timeline_epoch=self.timeline_epoch, publisher_clock_rollbacks=self.scan_clock_epoch,
                    complete_scan_counts=dict(self.scan_counts), sector_counts=dict(self.sectors),
                    saved_samples=self.saved_samples, saved_bytes=self.saved_bytes,
                    peak_ring_bytes=self.peak_ring_bytes, ring_size_evictions=self.size_evictions,
                    max_saved_bytes=self.max_saved_bytes)

    def close(self):
        self.flush_ring()
        for stream in (self.frames, self.payloads, self.scans, self.events):
            stream.close()
        return self.summary()
