import math
import unittest
from recovery_trial_math import PoseController, RotationController, segment_clearance, preview_allows_frozen_target

class TrialMathTests(unittest.TestCase):
    def test_stationary_preparation_never_permits_nonzero_commands(self):
        from recovery_trial_math import validate_trial_motion
        for fixed in (False,True):
            validate_trial_motion(fixed,True,0.,0.)
            for v,w in ((.07,0.),(0.,.2),(.07,.2)):
                with self.assertRaises(ValueError):validate_trial_motion(fixed,True,v,w)

    def test_motion_needs_fixed_target_even_after_preparation(self):
        from recovery_trial_math import validate_trial_motion
        with self.assertRaises(ValueError):validate_trial_motion(False,False,.07,0.)
        validate_trial_motion(True,False,.07,.2)

    def test_guard_discovery_accepts_either_endpoint_arrival_order(self):
        from recovery_trial_math import guard_graph_ready
        for inputs,outputs in [([],[]),(['guard'],[]),([],['guard']),
                               (['_NODE_NAME_UNKNOWN_'],['guard']),
                               (['guard'],['_NODE_NAME_UNKNOWN_'])]:
            self.assertFalse(guard_graph_ready(inputs,outputs,'guard'))
        self.assertTrue(guard_graph_ready(['guard'],['guard'],'guard'))

    def test_guard_discovery_rejects_competitor_before_full_discovery(self):
        from recovery_trial_math import guard_graph_ready
        for inputs,outputs in [([],['teleop']),(['other'],[]),(['guard'],['guard','teleop'])]:
            with self.subTest(inputs=inputs,outputs=outputs),self.assertRaises(ValueError):
                guard_graph_ready(inputs,outputs,'guard')

    def test_guard_discovery_rejects_duplicate_node_names(self):
        from recovery_trial_math import guard_graph_ready
        for inputs,outputs in [(['guard','guard'],['guard']),(['guard'],['guard','guard'])]:
            with self.assertRaises(ValueError):guard_graph_ready(inputs,outputs,'guard')

    def test_manual_setup_offset_is_base_relative_and_bounded(self):
        from recovery_trial_math import bounded_setup_target
        p=bounded_setup_target((1,2,math.pi/2),(.2,-.2,10))
        self.assertAlmostEqual(p[0],1.2)
        self.assertAlmostEqual(p[1],2.2)
        self.assertAlmostEqual(p[2],math.radians(100))
        for v in ((.501,0,0),(.4,.4,0),(.2,0,21),(float('nan'),0,0),(0,0,10)):
            with self.subTest(v=v),self.assertRaises(ValueError):
                bounded_setup_target((0,0,0),v)

    def test_half_metre_setup_preserves_heading_and_controller_reaches_it(self):
        from recovery_trial_math import bounded_setup_target, angle
        initial=(.28,-.23,.23)
        target=bounded_setup_target(initial,(-.3,.4,0))
        self.assertAlmostEqual(math.dist(initial[:2],target[:2]),.5)
        self.assertEqual(target[2],initial[2])
        controller=PoseController(target);x,y,yaw=initial
        for _ in range(6000):
            v,w=controller.command((x,y,yaw))
            x+=v*math.cos(yaw)*.02;y+=v*math.sin(yaw)*.02;yaw=angle(yaw+w*.02)
            self.assertLessEqual(math.dist(initial[:2],(x,y)),.55)
            if controller.phase=='done':break
        self.assertEqual(controller.phase,'done')
        self.assertLess(math.dist(target[:2],(x,y)),.021)
        self.assertLess(abs(angle(yaw-initial[2])),math.radians(1.2))

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
class ReturnValidationTests(unittest.TestCase):
    def test_only_near_start_unchanged_stopped_pose_can_finish_yaw(self):
        from recovery_trial_math import validate_rotation_return
        validate_rotation_return((0,0,0),(.016,0,.75),(.016,0,.75))
        for initial,saved,current in [
                ((0,0,0),(.04,0,.75),(.04,0,.75)),
                ((0,0,0),(0,0,.75),(.02,0,.85)),
                ((0,0,0),(0,0,1.1),(0,0,1.1)),
                ((0,0,0),(0,0,.75),(float('nan'),0,.75))]:
            with self.assertRaises(ValueError): validate_rotation_return(initial,saved,current)

if __name__=='__main__': unittest.main()
