"""Offline tests for snapshot association and axis-based candidate ranking."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from evaluate_cfv_axis_recovery import pair_recording, rank_snapshot, stamp
from recovery_selection import CandidateSelector, SelectionConfig
from test_recovery_selection import snapshot, header, pose


class AxisRecoveryTests(unittest.TestCase):
    def confirmed(self):
        selector = CandidateSelector(SelectionConfig(max_trials=48))
        for t in (10, 10.1, 10.2):
            msg = snapshot(t, patches=[(3, -2, 0, -1)])
            result = selector.observe(msg, t, t)
        self.assertEqual(result['status'], 'ready')
        return msg, result

    def test_same_candidate_under_front_rear_swap(self):
        msg, result = self.confirmed()
        a, b = [rank_snapshot(msg, result, axis) for axis in ((1,0),(-1,0))]
        self.assertEqual(a['selected']['label'], 'A1/right')
        self.assertEqual(a['selected']['candidate_index'], b['selected']['candidate_index'])
        self.assertAlmostEqual(a['selected']['improvement_deg'], b['selected']['improvement_deg'])
        self.assertEqual(a['selected']['base_pose'], msg['base_poses'][3])

    def test_side_and_ambiguous_patches_do_not_produce_goal(self):
        msg, result = self.confirmed()
        for axis in ((0,1),(1,1)):
            self.assertIsNone(rank_snapshot(msg,result,axis)['selected'])

    def test_mixed_candidate_snapshot_is_rejected(self):
        msg, result = self.confirmed()
        bad = copy.deepcopy(result)
        bad['queue'][0]['camera_position']['x'] += .1
        with self.assertRaisesRegex(ValueError,'Mixed'):
            rank_snapshot(msg,bad,(1,0))

    def recording(self, root, *, future=False, child='base_link', image_time=10.25):
        raw = b'fixture image bytes for hash association, not inference'
        (root/'baseline.png').write_bytes(raw)
        ns = round(image_time*1e9)
        (root/'report.json').write_text(json.dumps({'baseline':{'image_evidence':[
            {'image':'baseline.png','source_stamp_ns':ns}]}}))
        events = [{'kind':'preview','wall_sec':t,'message':snapshot(t,patches=[(3,-2,0,-1)])}
                  for t in ((10,10.1,10.2,10.3) if future else (10,10.1,10.2))]
        events.append({'kind':'pose','message':{'header':header(image_time),
            'child_frame_id':child,'pose':{'pose':pose()}}})
        (root/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
        return {'samples':[{'image_absolute':str(root/'baseline.png'),
                'image_sha256':hashlib.sha256(raw).hexdigest(),'angle_deg':180}]}

    def test_pair_uses_preceding_snapshot_never_future(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d); cfv=self.recording(p,future=True)
            paired=pair_recording(cfv,p)
            self.assertEqual(paired['preview_stamp_ns'],round(10.2*1e9))
            self.assertAlmostEqual(paired['image_preview_gap_sec'],.05)

    def test_pair_rejects_stale_frame_wrong_transform_or_hash(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            for options in ({'image_time':10.5},{'child':'camera'}):
                cfv=self.recording(p,**options)
                with self.assertRaises(ValueError): pair_recording(cfv,p)
            cfv=self.recording(p)
            cfv['samples'][0]['image_sha256']='wrong'
            with self.assertRaisesRegex(ValueError,'hash-matching'):
                pair_recording(cfv,p)


if __name__ == '__main__':
    unittest.main()
