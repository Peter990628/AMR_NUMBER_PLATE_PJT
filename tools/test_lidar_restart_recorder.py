import asyncio
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from lidar_restart_recorder import RestartRecorder
from test_lidar_dropout_recorder import event as scan_event


def sample(index, wall, stamp, missing=False):
    return dict(scan_event(index, wall, missing=missing, stamp=stamp), playing=True)


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


class RestartRecorderTests(unittest.TestCase):
    def test_stop_play_preserves_pre_window_and_resets_scan_clock(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            rec = RestartRecorder(out, post_seconds=2.)
            # A missing sector BEFORE Stop must not prevent the recorder from arming.
            for i in range(5):
                rec.observe(sample(i, i, 100.+i, missing=True), {'raw_gmo': b'abc'})
            self.assertTrue(rec.ready)
            rec.note('stop', 4.5, 4500000000)
            rec.observe(dict(sample_index=5, wall_sec=5., wall_time_ns=5000000000,
                             playing=False, nodes=[], gpu_payloads={}), {})
            rec.note('play', 6., 6000000000)
            rec.observe(sample(6, 6., 0.), {'raw_gmo': b'abc'})
            rec.observe(sample(7, 7., 1.), {'raw_gmo': b'abc'})
            self.assertTrue(rec.observe(sample(8, 8., 2.), {'raw_gmo': b'abc'}))
            result = rec.close()
            self.assertEqual(result['finished_reason'], 'restart_window_complete')
            self.assertEqual(result['complete_scan_counts'], {'before_stop': 5, 'after_restart': 3})
            saved = rows(out / 'frames.jsonl')
            self.assertEqual([r['sample_index'] for r in saved], list(range(2, 9)))
            payload = (out / 'gpu_payloads.bin').read_bytes()
            for row in saved:
                for meta in row['gpu_payloads'].values():
                    self.assertEqual(payload[meta['offset']:meta['offset']+meta['byte_count']], b'abc')
            scans = rows(out / 'scan_summary.jsonl')
            self.assertEqual(scans[-3]['stamp_ns'], 0)
            self.assertEqual(scans[-3]['timeline_epoch'], 1)

    def test_pause_resume_does_not_count_as_restart_and_wait_is_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            rec = RestartRecorder(Path(directory), wait_seconds=10.)
            rec.note('pause', 1., 1)
            rec.note('play', 2., 2)
            self.assertIsNone(rec.restart_wall)
            self.assertTrue(rec.tick(10.))
            self.assertEqual(rec.close()['finished_reason'], 'no_stop_play_within_wait')

    def test_duplicate_and_clock_rollback_are_recorded_without_crashing(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            rec = RestartRecorder(out)
            rec.observe(sample(0, 0., 10.), {})
            rec.observe(sample(1, 1., 10.), {})
            rec.observe(sample(2, 2., 1.), {})
            result = rec.close()
            self.assertEqual(result['complete_scan_counts'], {'before_stop': 2})
            self.assertEqual(result['publisher_clock_rollbacks'], 1)
            self.assertEqual(rows(out / 'events.jsonl')[0]['kind'], 'publisher_clock_rollback')

    def test_ring_and_disk_are_bounded_including_final_flush(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            rec = RestartRecorder(out, max_ring_bytes=4000, max_saved_bytes=7000)
            for i in range(10):
                if rec.observe(sample(i, i*.01, float(i)), {'raw_gmo': b'abc'}):
                    break
                self.assertLessEqual(rec.ring_bytes, 4000)
            rec.note('stop', .2, 2)
            rec.note('play', .3, 3)
            for i in range(10, 30):
                if rec.observe(sample(i, i*.1, float(i)), {'raw_gmo': b'abc'}):
                    break
            result = rec.close()
            self.assertEqual(result['finished_reason'], 'size_limit')
            self.assertLessEqual(sum(p.stat().st_size for p in out.iterdir()), 7000)
            self.assertLessEqual(result['peak_ring_bytes'], 4000)

    def test_initial_empty_scan_and_second_stop_are_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            rec = RestartRecorder(Path(directory))
            e = sample(0, 0., 0.)
            e['nodes'][0]['values']['outputs:numCols'] = 0
            rec.observe(e, {})
            self.assertFalse(rec.ready)
            self.assertEqual(dict(rec.scan_counts), {})
            rec.note('stop', 1., 1)
            rec.note('play', 2., 2)
            rec.note('stop', 3., 3)
            self.assertTrue(rec.tick(3.))
            self.assertEqual(rec.close()['finished_reason'], 'additional_stop')


class GuiLifecycleTests(unittest.TestCase):
    def test_gui_stop_play_rebinds_nodes_and_never_copies_while_stopped(self):
        spec = importlib.util.spec_from_file_location('restart_gui_test',
                    Path(__file__).with_name('capture_lidar_restart_in_gui.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        clock = SimpleNamespace(wall=0.)
        callbacks = []
        unsubscribed = []
        subscription = SimpleNamespace(unsubscribe=lambda: unsubscribed.append(True))
        stream = SimpleNamespace(create_subscription_to_pop=lambda cb, **kw:
                                 (callbacks.append(cb) or subscription))
        timeline = SimpleNamespace(playing=True, sim=100.,
                                   get_timeline_event_stream=lambda: stream)
        timeline.is_playing = lambda: timeline.playing
        timeline.get_current_time = lambda: timeline.sim
        # Simulate a rapid restart; the new scan time is lower than the old one.
        sequence = iter([(0., True, 100., None), (1., True, 101., None),
                         (2., True, 102., None), (3., True, 103., None),
                         (4., False, 0., 2), (5., False, 0., None),
                         (6., True, 0., 1), (7., True, 1., None), (66., True, 60., None)])

        async def post_update():
            wall, playing, sim, kind = next(sequence)
            clock.wall, timeline.playing, timeline.sim = wall, playing, sim
            if kind is not None:
                for callback in callbacks:
                    callback(SimpleNamespace(type=kind))
            await asyncio.sleep(0)

        stage = SimpleNamespace(GetPrimAtPath=lambda p: p,
                                GetRootLayer=lambda: SimpleNamespace(identifier='fake.usd'))
        context = SimpleNamespace(get_stage=lambda: stage)
        graph = SimpleNamespace(core=SimpleNamespace())
        kit = SimpleNamespace(app=SimpleNamespace(get_app=lambda: SimpleNamespace(post_update_async=post_update)))
        tl = SimpleNamespace(get_timeline_interface=lambda: timeline,
                             TimelineEventType=SimpleNamespace(PLAY=1, STOP=2, PAUSE=3))
        usd = SimpleNamespace(get_context=lambda: context)
        omni = SimpleNamespace(graph=graph, kit=kit, timeline=tl, usd=usd)
        pxr = SimpleNamespace(UsdGeom=SimpleNamespace(XformCache=lambda:
                              SimpleNamespace(GetLocalToWorldTransform=lambda p: [[1., 0.], [0., 1.]])))
        fake_modules = {'omni': omni, 'omni.graph': graph, 'omni.graph.core': graph.core,
                        'omni.kit': kit, 'omni.kit.app': kit.app, 'omni.timeline': tl,
                        'omni.usd': usd, 'warp': SimpleNamespace(), 'pxr': pxr}
        bindings, copied_generations = [], []

        def discover(og):
            binding = dict(generation=len(bindings)+1, inventory=dict(nodes=[], connections=[]))
            bindings.append(binding)
            return binding

        def copy(binding, wp, budget):
            self.assertTrue(timeline.playing)
            copied_generations.append(binding['generation'])
            e = sample(0, clock.wall, timeline.sim)
            return e['nodes'], e['gpu_payloads'], {'raw_gmo': b'abc'}

        with tempfile.TemporaryDirectory() as directory, patch.dict(sys.modules, fake_modules):
            module.UTILS['OUTPUT_ROOT'] = Path(directory)
            module.time = SimpleNamespace(monotonic=lambda: clock.wall,
                                          time_ns=lambda: int(clock.wall*1e9))
            module.discover_pipeline, module.sample_pipeline = discover, copy
            asyncio.run(module.capture_restart())
            out = next(Path(directory).iterdir())
            report = json.loads((out / 'report.json').read_text())
            self.assertEqual(report['status'], 'restart_window_complete')
            self.assertEqual(report['binding_generation'], 2)
            self.assertEqual(copied_generations, [1, 1, 1, 1, 2, 2])
            self.assertEqual(report['complete_scan_counts'], {'before_stop': 4, 'after_restart': 2})
            self.assertTrue(unsubscribed)


if __name__ == '__main__':
    unittest.main()
