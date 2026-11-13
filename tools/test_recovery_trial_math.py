import math
import unittest
from recovery_trial_math import PoseController, RotationController, segment_clearance, preview_allows_frozen_target

class TrialMathTests(unittest.TestCase):
    def test_rotation_never_translates_to_correct_xy_drift(self):
        c=RotationController((.1,.2,.4))
        for pose in [(0.,0.,0.),(.09,.21,.2),(1.,3.,.4)]:
            v,w=c.command(pose)
            self.assertEqual(v,0.)
            self.assertLessEqual(abs(w),.25)
        self.assertEqual(c.phase,'done')

    def test_rotation_wrap_and_invalid_input(self):
        c=RotationController((0,0,-math.pi+.02))
        self.assertGreater(c.command((0,0,math.pi-.03))[1],0.)
        self.assertEqual(c.command((0,0,-math.pi+.02)),(0.,0.))
        with self.assertRaises(ValueError): c.command((float('nan'),0,0))
        with self.assertRaises(ValueError): RotationController((0,0,0),max_angular=.4)
    def test_frozen_target_does_not_require_reselection_but_keeps_faults(self):
        self.assertFalse(preview_allows_frozen_target(False,'contour_ambiguous_target',False))
        self.assertTrue(preview_allows_frozen_target(False,'contour_ambiguous_target',True))
        for status in ['contour_invalid_input','contour_empty','request_expired', 'time_reset', 'missing_tf', 'no_observed_points']:
            self.assertFalse(preview_allows_frozen_target(False,status,True))
    def test_rotation_wrap_and_stop(self):
        c=PoseController((0,0,-math.pi+.05))
        v,w=c.command((0,0,math.pi-.05))
        self.assertEqual(v,0); self.assertGreater(w,0); self.assertLessEqual(w,.25)
        self.assertEqual(c.command((0,0,-math.pi+.05)),(0.,0.))
    def test_lateral_candidate_turn_drive_align(self):
        c=PoseController((0,.35,0))
        v,w=c.command((0,0,0)); self.assertEqual(v,0); self.assertGreater(w,0)
        v,w=c.command((0,0,math.pi/2)); self.assertGreater(v,0); self.assertLessEqual(v,.07)
        v,w=c.command((0,.34,math.pi/2)); self.assertEqual(v,0); self.assertLess(w,0)
        self.assertEqual(c.command((0,.34,0)),(0.,0.))
    def test_mid_path_obstacle_not_just_endpoint(self):
        self.assertAlmostEqual(segment_clearance([(1,.2)],(0,0),(2,0)),.2)
        self.assertAlmostEqual(segment_clearance([(1,.2)],(0,0),(0,0)),math.hypot(1,.2))
    def test_no_nonfinite_motion(self):
        for pose in [(0,float('nan'),0),(0,0,float('inf'))]:
            with self.assertRaises(ValueError): PoseController(pose)
        with self.assertRaises(ValueError): PoseController((0,0,0)).command((0,0,float('nan')))
        with self.assertRaises(ValueError): segment_clearance([], (0,0),(1,0))
    def test_simple_closed_loop_reaches_target(self):
        target=(.35,-.35,.4); c=PoseController(target); x=y=yaw=0.
        for _ in range(4000):
            v,w=c.command((x,y,yaw)); yaw+=w*.02
            x+=v*math.cos(yaw)*.02; y+=v*math.sin(yaw)*.02
            if c.phase=='done': break
        self.assertEqual(c.phase,'done')
        self.assertLess(math.hypot(x-target[0],y-target[1]),.021)
        self.assertLess(abs(yaw-target[2]),.022)
if __name__=='__main__': unittest.main()
