"""Check paired-window fairness and replay freshness without ROS or motion."""
import unittest
from unittest.mock import patch
import json
from pathlib import Path
import tempfile

from compare_observation_wait import main, paired_windows, replay
from test_recovery_selection import snapshot


class ObservationWaitTests(unittest.TestCase):
    def test_complete_paired_starts_and_strict_deadline(self):
        rows = [{'wall_sec': 1.}, {'wall_sec': 3.}, {'wall_sec': 5.}]
        result = paired_windows(rows, 5., [1.,3.], 1.,
                                lambda r: {'first_wall_sec': r[0]['wall_sec'] if r else None})
        self.assertEqual([w['start_wall_sec'] for w in result], [0.,1.,2.])
        self.assertEqual(result[0]['available'], {'1.0': False, '3.0': True})
        self.assertEqual(result[2]['first_wall_sec'], 3.)

    def test_no_evidence_from_before_start(self):
        groups = []
        paired_windows([{'wall_sec': t} for t in [0.,.5,1.,1.5,2.]], 3., [2.], 1.,
                       lambda r: groups.append([v['wall_sec'] for v in r]) or {'first_wall_sec': None})
        self.assertEqual(groups, [[0.,.5,1.,1.5], [1.,1.5,2.]])

    def test_replay_preserves_recorded_ros_age_and_wall_gaps(self):
        rows = [{'wall_sec': w, 'now_ros_sec': ros, 'preview': snapshot(source)}
                for w,ros,source in [(0.,10.,10.),(.1,10.1,10.1),(.2,11.,10.2),
                                     (.3,11.1,11.1),(.4,11.2,11.2),(2.,11.3,11.3)]]
        with patch('compare_observation_wait.choose_view_snapshot',
                   side_effect=lambda s,*a,**k: {'status':s['status'], 'queue':[]}):
            result = replay(rows, {'image_stamp_ns':0}, stop_at_first=False)
        self.assertEqual(result['selector_status_counts'].get('stale_input'), 1)
        self.assertNotIn('ready', result['selector_status_counts'])
        self.assertGreaterEqual(result['resets'].get('observation_gap',0), 1)

    def test_duplicate_source_does_not_count_as_new_evidence(self):
        rows = [{'wall_sec': t, 'now_ros_sec':10., 'preview':snapshot(10.)} for t in [0.,.1,.2]]
        with patch('compare_observation_wait.choose_view_snapshot',
                   side_effect=lambda s,*a,**k: {'status':s['status'], 'queue':[]}):
            result = replay(rows, {'image_stamp_ns':0}, stop_at_first=False)
        self.assertEqual(result['selector_status_counts'], {'collecting':1})

    def test_offline_cli_accepts_json_roundtrip_but_rejects_changed_context(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            context = {'axis_xy_odom': (1., 0.), 'image_stamp_ns': 42}
            (output/'context.json').write_text(json.dumps(context))
            (output/'previews.jsonl').write_text('')
            (output/'capture_report.json').write_text('{}')
            argv = ['compare_observation_wait.py','--replay','--output',directory,'--context','cfv.json']
            with patch('sys.argv', argv), patch('compare_observation_wait.load_context', return_value=context), \
                    patch('compare_observation_wait.analyze') as analyze:
                main()
                analyze.assert_called_once()
                context['image_stamp_ns'] = 43
                with self.assertRaisesRegex(ValueError,'replay_context_changed'):
                    main()


if __name__ == '__main__':
    unittest.main()
