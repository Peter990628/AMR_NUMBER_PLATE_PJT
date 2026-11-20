import asyncio
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import capture_sensor_pose_restart_in_gui as recorder


def sample(wall, sim=100., x=0., playing=True):
    pose = dict(translation_m=[x, 0., 0.], rotation_matrix_3x3=[[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    return dict(wall_sec=wall, wall_time_ns=int(wall*1e9), timeline_seconds=sim,
                playing=playing, poses={p: {'world': pose, 'local': pose} for p in recorder.PATHS})


class RecorderTests(unittest.TestCase):
    def test_rate_ring_epochs_and_local_world_drift(self):
        with tempfile.TemporaryDirectory() as folder:
            rec = recorder.PoseRecorder(Path(folder))
            for i in range(501):
                rec.observe(sample(i*.01, sim=100.+i*.01))
            self.assertTrue(rec.ready)
            self.assertLessEqual(rec.observed_count, 51)
            rec.note('stop', 5.1, 0, 0.)
            rec.observe(sample(5.2, sim=0., playing=False))
            rec.note('play', 5.3, 0, 0.)
            for i in range(200):
                rec.observe(sample(5.3+i*.1, sim=i*.1, x=min(i, 20)*.001))
            self.assertTrue(rec.tick(25.3))
            result = rec.summary()
            self.assertEqual(result['finished_reason'], 'restart_window_complete')
            before = [r for r in rec.retained if r['phase'] == 'before_stop']
            self.assertGreaterEqual(before[0]['wall_sec'], 2.)
            after = [r for r in rec.retained if r['phase'] == 'after_restart']
            self.assertEqual(after[0]['timeline_epoch'], 1)
            self.assertEqual(after[0]['timeline_seconds'], 0.)
            stats = result['phases']['after_restart']
            self.assertAlmostEqual(stats['drift_from_first'][recorder.SENSOR]['world']['max_translation_from_first_m'], .02)
            self.assertEqual(stats['last_5_wall_seconds'][recorder.SENSOR]['local']['max_translation_from_first_m'], 0.)
            self.assertTrue(stats['last_5_window_complete'])

    def test_pause_resume_is_not_restart_and_wait_is_bounded(self):
        with tempfile.TemporaryDirectory() as folder:
            rec = recorder.PoseRecorder(Path(folder))
            rec.observe(sample(0.))
            rec.note('pause', 1., 0, 1.)
            rec.note('play', 2., 0, 1.)
            self.assertIsNone(rec.play_wall)
            self.assertFalse(rec.ready)
            self.assertTrue(rec.tick(120.))
            self.assertEqual(rec.reason, 'no_stop_play_within_wait')

    def test_early_or_second_stop_is_explicit_and_disk_budget_reserved(self):
        with tempfile.TemporaryDirectory() as folder:
            rec = recorder.PoseRecorder(Path(folder), max_bytes=recorder.REPORT_RESERVE+3000)
            for i in range(4):
                rec.observe(sample(float(i)))
            rec.note('stop', 3.1, 0, 0.)
            self.assertEqual(rec.reason, 'size_limit')
            self.assertLessEqual(sum(p.stat().st_size for p in Path(folder).iterdir()), 3000)
        with tempfile.TemporaryDirectory() as folder:
            rec = recorder.PoseRecorder(Path(folder))
            rec.note('stop', .1, 0, 0.)
            self.assertEqual(rec.reason, 'stop_before_ready')
            rec.note('stop', .2, 0, 0.)
            self.assertEqual(rec.reason, 'additional_stop')

    def test_full_rotation_difference_detects_non_yaw_motion(self):
        identity = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
        angle = math.radians(12.)
        roll = [[1., 0., 0.], [0., math.cos(angle), -math.sin(angle)], [0., math.sin(angle), math.cos(angle)]]
        self.assertAlmostEqual(recorder.rotation_difference_deg(identity, roll), 12.)


class GuiTests(unittest.TestCase):
    def test_actual_async_lifecycle_fresh_cache_and_no_commands(self):
        spec = importlib.util.spec_from_file_location('pose_gui_test', Path(recorder.__file__))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        clock = SimpleNamespace(wall=0.)
        callbacks, unsubscribed, caches = [], [], []
        subscription = SimpleNamespace(unsubscribe=lambda: unsubscribed.append(True))
        stream = SimpleNamespace(create_subscription_to_pop=lambda cb, **kw: (callbacks.append(cb) or subscription))
        timeline = SimpleNamespace(playing=True, sim=100., get_timeline_event_stream=lambda: stream)
        timeline.is_playing = lambda: timeline.playing
        timeline.get_current_time = lambda: timeline.sim
        sequence = iter([(0., True, 100., None), (.02, True, 100.02, None),
                         (.1, True, 100.1, None), (1., True, 101., None),
                         (2., True, 102., None), (3.01, True, 103.01, None),
                         (3.2, False, 0., 2), (3.4, True, 0., 1),
                         (4.4, True, 1., None), (9.4, True, 6., None), (23.5, True, 20.1, None)])

        async def update():
            wall, playing, sim, kind = next(sequence)
            clock.wall, timeline.playing, timeline.sim = wall, playing, sim
            if kind is not None:
                for callback in callbacks:
                    callback(SimpleNamespace(type=kind))
            await asyncio.sleep(0)

        class Matrix:
            def __init__(self):
                self.rows = [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [.027 if timeline.sim < 100. else 0., 0., 0., 1.]]

            def GetOrthonormalized(self):
                return self

            def ExtractRotationMatrix(self):
                return [r[:3] for r in self.rows[:3]]

            def ExtractTranslation(self):
                return self.rows[3][:3]

            def __getitem__(self, item):
                return self.rows[item]

        def cache():
            caches.append(clock.wall)
            return SimpleNamespace(GetLocalToWorldTransform=lambda p: Matrix())

        stage = SimpleNamespace(GetPrimAtPath=lambda p: SimpleNamespace(GetParent=lambda: SimpleNamespace(GetPath=lambda: str(Path(p).parent))),
                                GetRootLayer=lambda: SimpleNamespace(identifier='fake.usd'))
        geom = SimpleNamespace(XformCache=cache, GetStageMetersPerUnit=lambda s: 1.,
            Xformable=lambda p: SimpleNamespace(GetLocalTransformation=Matrix, GetResetXformStack=lambda: False))
        kit = SimpleNamespace(app=SimpleNamespace(get_app=lambda: SimpleNamespace(post_update_async=update)))
        tl = SimpleNamespace(get_timeline_interface=lambda: timeline, TimelineEventType=SimpleNamespace(PLAY=1, STOP=2, PAUSE=3))
        usd = SimpleNamespace(get_context=lambda: SimpleNamespace(get_stage=lambda: stage))
        omni = SimpleNamespace(kit=kit, timeline=tl, usd=usd)
        fake = {'omni': omni, 'omni.kit': kit, 'omni.kit.app': kit.app,
                'omni.timeline': tl, 'omni.usd': usd, 'pxr': SimpleNamespace(UsdGeom=geom)}
        with tempfile.TemporaryDirectory() as folder, patch.dict(sys.modules, fake):
            module.OUTPUT_ROOT = Path(folder)
            module.time = SimpleNamespace(monotonic=lambda: clock.wall, time_ns=lambda: int(clock.wall*1e9))
            asyncio.run(module.capture_restart())
            output = next(Path(folder).iterdir())
            report = json.loads((output / 'report.json').read_text())
            self.assertEqual(report['status'], 'restart_window_complete')
            self.assertEqual(len(caches), report['observed_sample_count'])
            self.assertNotIn(.02, caches)
            self.assertTrue(unsubscribed)
            self.assertLessEqual(sum(p.stat().st_size for p in output.iterdir()), module.MAX_BYTES)
            rows = [json.loads(line) for line in (output / 'poses.jsonl').read_text().splitlines()]
            post = [r for r in rows if r['phase'] == 'after_restart']
            self.assertEqual(post[0]['timeline_epoch'], 1)
            self.assertEqual(post[0]['poses'][module.SENSOR]['world']['translation_m'], [.027, 0., 0.])


if __name__ == '__main__':
    unittest.main()
