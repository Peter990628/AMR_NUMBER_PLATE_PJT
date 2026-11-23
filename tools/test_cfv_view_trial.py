import copy
import math
import unittest

from cfv_view_trial import CREATE3_EXTENDED_LEG_LIMITS, choose_view_snapshot, validate_view_leases
from recovery_selection import CandidateSelector, SelectionConfig
from test_recovery_selection import snapshot, pose


class ViewCSelectionTests(unittest.TestCase):
    def test_invalid_contour_default_pose_cannot_be_ranked_or_confirmed(self):
        from cfv_trial_selection import choose_cfv_snapshot
        selected,p,c=self.fixtures()
        c['image_pose']['position']['x']=1.
        p['valid']=False
        p['status']='contour_ambiguous_target'
        for choose in (choose_view_snapshot,choose_cfv_snapshot):
            result=choose(selected,p,c,10.3)
            self.assertEqual(result['status'],'invalid_preview')
            self.assertEqual(result['queue'],[])
            p['valid']=True
            with self.assertRaisesRegex(ValueError,'cfv_pose_changed_before_selection'):
                choose(selected,p,c,10.3)
            p['valid']=False

    def test_companion_lease_mismatch_rejected_before_motion(self):
        validate_view_leases(600.,600.,600.)
        for values in ((300.,600.,600.),(600.,300.,600.),(600.,600.,300.),(float('nan'),600.,600.)):
            with self.assertRaises(ValueError):validate_view_leases(*values)

    def fixtures(self, aim_y=3, *, current=None, hint_x=3.5, half_span=1.):
        selector=CandidateSelector(SelectionConfig(max_trials=18))
        for t in (10,10.1,10.2):
            preview=snapshot(t,patches=[(2,aim_y-half_span,aim_y+half_span,aim_y)],current=current)
            preview['vehicle_hint']['x']=hint_x
            confirmed=selector.observe(preview,t,t)
        context={'axis_xy_odom':(1,0),'image_stamp_ns':10_150_000_000,
                 'image_pose':copy.deepcopy(preview['current_pose']),'reference_preview':copy.deepcopy(preview),
                 'max_source_age_sec':30.,'camera_offset_base_xy':(0,0),
                 'camera_forward_yaw_rad':0.}
        return confirmed,preview,context

    def test_generates_reviewed_angle_range_with_fresh_source_provenance(self):
        selected,p,c=self.fixtures()
        original=copy.deepcopy(p)
        row=choose_view_snapshot(selected,p,c,10.3)['queue'][0]
        self.assertEqual(p,original)
        self.assertTrue(row['generated_pose'])
        self.assertEqual(row['source_anchor']['base_pose'],selected['queue'][0]['base_pose'])
        self.assertEqual(row['header'],p['header'])
        self.assertAlmostEqual(row['cfv_evaluation']['incidence_deg'],45)
        self.assertAlmostEqual(row['cfv_evaluation']['camera_to_aim_m'],2.5)
        self.assertLess(row['translation_m'],1.5)
        self.assertNotEqual(row['base_pose'],row['source_anchor']['base_pose'])

    def test_axis_reversal_preserves_same_physical_candidate(self):
        selected,p,c=self.fixtures()
        a=choose_view_snapshot(selected,p,c,10.3)['queue'][0]
        c['axis_xy_odom']=(-1,0)
        b=choose_view_snapshot(selected,p,c,10.3)['queue'][0]
        self.assertEqual(a['base_pose'],b['base_pose'])

    def test_fresh_close_uses_new_axis_and_surface_without_reference(self):
        selected,p,c=self.fixtures()
        a=choose_view_snapshot(selected,p,c,10.3,fresh_close=True)
        self.assertNotIn('distance_comparison',a['diagnostics'])
        self.assertAlmostEqual(a['queue'][0]['cfv_evaluation']['camera_to_aim_m'],2.)
        self.assertAlmostEqual(a['queue'][0]['cfv_evaluation']['incidence_deg'],45.)
        c['axis_xy_odom']=(math.cos(.02),math.sin(.02))
        b=choose_view_snapshot(selected,p,c,10.3,fresh_close=True)
        self.assertNotEqual(a['queue'][0]['base_pose'],b['queue'][0]['base_pose'])
        selected,p,c=self.fixtures(3.02)
        d=choose_view_snapshot(selected,p,c,10.3,fresh_close=True)
        self.assertNotEqual(a['queue'][0]['look_at'],d['queue'][0]['look_at'])
        self.assertNotEqual(a['queue'][0]['base_pose'],d['queue'][0]['base_pose'])

    def test_fresh_close_rejects_reference_and_excess_movement(self):
        selected,p,c=self.fixtures()
        r=self.reference(selected,p,c)
        with self.assertRaisesRegex(ValueError,'cannot_reuse_reference'):
            choose_view_snapshot(selected,p,c,10.3,r,fresh_close=True)
        selected,p,c=self.fixtures(4)
        result=choose_view_snapshot(selected,p,c,10.3,fresh_close=True)
        self.assertEqual(result['status'],'view_target_outside_bound')
        self.assertFalse(result['queue'])

    def test_camera_beside_end_selects_exterior_for_either_car_end(self):
        for outward in (-1, 1):
            for camera_inset in (0., .05):
                start = pose(2 - outward * camera_inset, 1., math.pi/2)
                selected, p, c = self.fixtures(current=start, hint_x=2 - outward*1.5, half_span=.35)
                for axis_sign in (-1, 1):
                    with self.subTest(outward=outward, inset=camera_inset, axis=axis_sign):
                        c['axis_xy_odom'] = (axis_sign, 0)
                        result = choose_view_snapshot(selected, p, c, 10.3, fresh_close=True)
                        self.assertEqual(result['status'], 'ready')
                        candidate = result['queue'][0]
                        self.assertAlmostEqual(candidate['camera_position']['x'], 2 + outward*math.sqrt(2))
                        self.assertAlmostEqual(candidate['camera_position']['y'], 3 - math.sqrt(2))
                        self.assertGreaterEqual(candidate['cfv_evaluation']['baseline_incidence_deg'], 90)

    def test_create3_wider_second_leg_is_bounded_and_first_leg_stays_local(self):
        for aim_y, expected in ((3.45, 'ready'), (4., 'view_target_outside_bound')):
            selected, preview, context = self.fixtures(aim_y)
            args = (selected, preview, context, 10.3)
            initial = choose_view_snapshot(*args, fresh_close=True)
            self.assertEqual(initial['status'], 'view_target_outside_bound')
            second = choose_view_snapshot(*args, fresh_close=True, limits=CREATE3_EXTENDED_LEG_LIMITS)
            self.assertEqual(second['status'], expected)
            if expected == 'ready':
                distance = second['queue'][0]['translation_m']
                self.assertGreater(distance, 1.75)
                self.assertLessEqual(distance, 2.30)
                self.assertEqual(second['diagnostics']['outward_view']['target_radius_m'], 2.30)

    def test_center_without_longitudinal_separation_defers_motion(self):
        selected, p, c = self.fixtures(hint_x=2.05)
        result = choose_view_snapshot(selected, p, c, 10.3, fresh_close=True)
        self.assertEqual(result['status'], 'view_ambiguous_end_direction')
        self.assertEqual(result['queue'], [])
        self.assertEqual(result['diagnostics']['outward_view']['branches'], [])

    def test_unreachable_outward_views_do_not_fall_back_to_camera_side(self):
        selected, p, c = self.fixtures(hint_x=.5)
        result = choose_view_snapshot(selected, p, c, 10.3, fresh_close=True)
        self.assertEqual(result['status'], 'view_target_outside_bound')
        self.assertEqual(result['queue'], [])
        branches = result['diagnostics']['outward_view']['branches']
        self.assertEqual(len(branches), 2)
        for branch in branches:
            self.assertGreater(branch['camera_xy_odom'][0], 2)
            self.assertEqual(branch['rejection'], 'view_target_outside_bound')

    def test_large_translation_rejected_instead_of_expanding_bound(self):
        selected,p,c=self.fixtures(4)
        result=choose_view_snapshot(selected,p,c,10.3)
        self.assertFalse(result['queue'])
        self.assertEqual(result['status'],'view_target_outside_bound')

    def test_no_confirmation_or_side_hypothesis_cannot_generate(self):
        selected,p,c=self.fixtures()
        for variant in ('side','unconfirmed'):
            a,b,d=copy.deepcopy((selected,p,c))
            if variant=='side': d['axis_xy_odom']=(0,1)
            else: a['status']='collecting'; a['queue']=[]
            self.assertFalse(choose_view_snapshot(a,b,d,10.3)['queue'])

    def test_stale_or_changed_mount_rejected(self):
        selected,p,c=self.fixtures()
        with self.assertRaises(ValueError): choose_view_snapshot(selected,p,c,41)
        c['camera_offset_base_xy']=(.1,0)
        with self.assertRaisesRegex(ValueError,'extrinsic_changed'):
            choose_view_snapshot(selected,p,c,10.3)

    def reference(self, selected, preview, context):
        candidate=choose_view_snapshot(selected,preview,context,10.3)['queue'][0]
        return {'selection':{'queue':[candidate]},'cfv_context':copy.deepcopy(context),
            'comparison':{'cfv_view_c':True},'initial_pose':[0,0,0],
            'trials':[{'observation':{'complete':True,'last_source_stamp_ns':9_900_000_000}}]}

    def test_distance_changes_without_changing_reference_ray_or_aim(self):
        selected,p,c=self.fixtures()
        reference=self.reference(selected,p,c)
        # Fresh estimates may move a little; that must not alter the comparison ray.
        c['axis_xy_odom']=(math.cos(.02),math.sin(.02))
        candidate=choose_view_snapshot(selected,p,c,10.3,reference)['queue'][0]
        old=reference['selection']['queue'][0]
        self.assertEqual(candidate['look_at'],old['look_at'])
        self.assertAlmostEqual(candidate['cfv_evaluation']['camera_to_aim_m'],2.)
        self.assertAlmostEqual(candidate['cfv_evaluation']['incidence_deg'],45.)
        self.assertAlmostEqual(math.dist(tuple(candidate['camera_position'][k] for k in 'xy'),
                                        tuple(old['camera_position'][k] for k in 'xy')),.5)
        self.assertAlmostEqual(candidate['base_pose']['orientation']['z'],old['base_pose']['orientation']['z'])

    def test_close_rejects_changed_scene_reference_or_epoch(self):
        selected,p,c=self.fixtures()
        reference=self.reference(selected,p,c)
        for variant in ('epoch','target','axis','aim','start','unmeasured','range'):
            a,b,d,r=copy.deepcopy((selected,p,c,reference))
            if variant=='epoch':r['trials'][0]['observation']['last_source_stamp_ns']=11_000_000_000
            elif variant=='target':r['selection']['queue'][0]['target_id']='other'
            elif variant=='axis':r['cfv_context']['axis_xy_odom']=(0,1)
            elif variant=='aim':r['selection']['queue'][0]['look_at']['y']+=.2
            elif variant=='start':r['initial_pose'][0]=.06
            elif variant=='unmeasured':r['trials'][0]['observation']['complete']=False
            else:r['selection']['queue'][0]['camera_position']['x']+=.1
            with self.subTest(variant=variant),self.assertRaises(ValueError):
                choose_view_snapshot(a,b,d,10.3,r)

    def repeat_reference(self, selected, preview, context):
        original=self.reference(selected,preview,context)
        chosen=choose_view_snapshot(selected,preview,context,10.3,original)
        return {'selection':chosen,'cfv_context':copy.deepcopy(context),
            'comparison':{'cfv_view_c':True},'initial_pose':[0,0,0],
            'status':'translation_comparison_complete',
            'trials':[{'observation':{'complete':True,'recovered':True,
                                     'last_source_stamp_ns':9_900_000_000}}]}

    def test_repeat_keeps_saved_target_despite_new_axis_noise(self):
        selected,p,c=self.fixtures()
        reference=self.repeat_reference(selected,p,c)
        c['axis_xy_odom']=(math.cos(.02),math.sin(.02))
        chosen=choose_view_snapshot(selected,p,c,10.3,reference,repeat=True)
        original=reference['selection']['queue'][0]
        for key in ('base_pose','camera_position','look_at'):
            self.assertEqual(chosen['queue'][0][key],original[key])
        self.assertAlmostEqual(chosen['diagnostics']['distance_comparison']['camera_shift_m'],0.)

    def test_repeat_rejects_unfinished_or_failed_reference(self):
        selected,p,c=self.fixtures()
        for variant in ('failed','aborted','different_target'):
            r=self.repeat_reference(selected,p,c)
            if variant=='failed':r['trials'][0]['observation']['recovered']=False
            elif variant=='aborted':r['status']='aborted'
            else:r['selection']['queue'][0]['base_pose']['position']['x']+=.01
            with self.subTest(variant=variant),self.assertRaises(ValueError):
                choose_view_snapshot(selected,p,c,10.3,r,repeat=True)


if __name__=='__main__': unittest.main()
