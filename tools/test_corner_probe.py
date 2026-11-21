import copy
import math
import unittest

from corner_probe import confirmed_corner_probe


def point(x, y):
    return {'x': x, 'y': y, 'z': 0.0}


class CornerProbeTest(unittest.TestCase):
    def setUp(self):
        pose = {'position': point(-2, .5), 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}
        self.preview = {
            'valid': True, 'header': {'frame_id': 'odom', 'stamp': {'sec': 10, 'nanosec': 0}},
            'request_header': {}, 'request_id': 'one', 'target_id': 'car',
            'vehicle_hint': point(.5, 2), 'approach_pose': pose, 'current_pose': pose,
            'contour_points': [point(0, i/10) for i in range(41)],
            'aim_endpoint_a': [point(0, 0)], 'aim_endpoint_b': [point(0, 4)]}
        self.ctx = {'image_stamp_ns': 10**10, 'max_source_age_sec': 30.,
                    'image_pose': pose, 'reference_preview': copy.deepcopy(self.preview),
                    'axis_xy_odom': (0, 1), 'camera_offset_base_xy': (.32, .08),
                    'camera_forward_yaw_rad': .03}

    def frames(self):
        items = []
        for i in range(3):
            p = copy.deepcopy(self.preview)
            p['header']['stamp']['nanosec'] = i*100_000_000
            items.append((20.+i*.3, p))
        return items

    def select(self, items):
        return confirmed_corner_probe(items, self.ctx, 10.24, 20.64)

    def test_keeps_side_offset_and_points_camera_at_near_end(self):
        self.preview['contour_points'].reverse()
        g = self.select(self.frames())['evidence']
        self.assertAlmostEqual(g['position'][0], -2.)
        self.assertAlmostEqual(g['position'][1], -.35)
        h = g['yaw']
        x = -2.+.32*math.cos(h)-.08*math.sin(h)
        y = -.35+.32*math.sin(h)+.08*math.cos(h)
        self.assertAlmostEqual(math.atan2(-y, -x), h+.03)

    def test_rejects_missing_stale_and_duplicate_observations(self):
        frames = self.frames()
        self.assertIsNone(self.select(frames[:2] + frames[1:2]))
        self.assertIsNone(confirmed_corner_probe(frames, self.ctx, 15., 30.))
        frames[1][1]['valid'] = False
        self.assertIsNone(self.select(frames))

    def test_shorter_overrun_uses_the_same_travel_limit(self):
        self.preview['current_pose'] = copy.deepcopy(self.preview['current_pose'])
        self.ctx['image_pose'] = self.preview['current_pose']
        self.preview['current_pose']['position']['y'] = 1.3
        self.assertIsNone(self.select(self.frames()))
        goal = confirmed_corner_probe(self.frames(), self.ctx, 10.24, 20.64, beyond_corner_m=.15)
        self.assertAlmostEqual(goal['evidence']['step_m'], 1.45)
        self.preview['current_pose']['position']['y'] = 1.4
        self.assertIsNone(confirmed_corner_probe(self.frames(), self.ctx, 10.24, 20.64, beyond_corner_m=.15))

    def test_comparison_adds_30cm_and_obeys_existing_close_view_radius(self):
        normal = self.select(self.frames())['evidence']
        def comparison():
            return confirmed_corner_probe(self.frames(), self.ctx, 10.24, 20.64, plus_30cm=True)
        extra = comparison()['evidence']
        self.assertAlmostEqual(extra['step_m'] - normal['step_m'], .30)
        self.assertEqual(extra['near_corner'], normal['near_corner'])
        self.assertAlmostEqual(extra['position'][1], -.65)
        self.preview['current_pose'] = copy.deepcopy(self.preview['current_pose'])
        self.preview['current_pose']['position']['y'] = 1.2
        self.ctx['image_pose'] = self.preview['current_pose']
        self.assertIsNone(comparison())

    def test_end_face_or_unstable_corner_does_not_support_probe(self):
        frames = self.frames()
        frames[-1][1]['contour_points'][0] = point(0, .4)
        self.assertIsNone(self.select(frames))
        self.preview['aim_endpoint_a'].append(point(0, 0))
        self.preview['aim_endpoint_b'].append(point(1, 0))
        self.assertIsNone(self.select(self.frames()))
