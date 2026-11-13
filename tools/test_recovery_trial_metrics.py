"""Independent measurements for stopped-pose recovery comparisons (no ROS)."""

import math
import unittest

from recovery_trial_metrics import ObservationWindow, match_measurement_image


def observation(stamp_ns, status='usable', **kwargs):
    detected = status in ('usable', 'warming_up', 'low_confidence')
    result = {
        'header': {'stamp': {'sec': stamp_ns//1_000_000_000,
                             'nanosec': stamp_ns % 1_000_000_000},
                   'frame_id': 'camera_optical'},
        'input_valid': status not in ('input_timeout', 'invalid_image'),
        'detected': detected, 'usable': status == 'usable', 'status': status,
        'image_width': 1280, 'image_height': 720,
        'confidence': .7 if detected else 0.,
        'width': 160. if detected else 0., 'height': 32. if detected else 0.,
        'border_margin_ratio': .1 if detected else 0.,
        # Production counter saturates. It must not replace our source count.
        'consecutive_count': 3 if status == 'usable' else 0,
    }
    result.update(kwargs)
    return result


def fill(window, statuses, *, start_ns=1_100_000_000, step_ns=100_000_000):
    for index, status in enumerate(statuses):
        stamp_ns = start_ns+index*step_ns
        window.add(observation(stamp_ns, status), stamp_ns+10_000_000,
                   10.+index*.2)


class ObservationWindowTests(unittest.TestCase):
    def test_empty_and_not_detected_windows_are_distinct(self):
        window = ObservationWindow(1_000_000_000)
        self.assertFalse(window.summary()['complete'])
        fill(window, ['not_detected']*12)
        result = window.summary()
        self.assertTrue(result['complete'])
        self.assertFalse(result['recovered'])
        self.assertEqual(result['valid_count'], 12)
        self.assertEqual(result['status_counts'], {'not_detected': 12})
        self.assertEqual(result['usable_fraction'], 0)
        self.assertIsNone(result['confidence_median'])

    def test_complete_quality_recovery_needs_samples_and_source_span(self):
        window = ObservationWindow(1_000_000_000)
        fill(window, ['usable']*11)
        self.assertFalse(window.summary()['complete'])
        window.add(observation(2_200_000_000), 2_210_000_000, 13.)
        result = window.summary()
        self.assertTrue(result['complete'])
        self.assertTrue(result['recovered'])
        self.assertEqual(result['sample_count'], 12)
        self.assertEqual(result['longest_usable_run'], 12)
        self.assertAlmostEqual(result['source_span_sec'], 1.1)
        self.assertAlmostEqual(result['confidence_median'], .7)
        self.assertAlmostEqual(result['width_median_px'], 160.)

    def test_fast_burst_is_incomplete_despite_twelve_frames(self):
        window = ObservationWindow(1_000_000_000)
        fill(window, ['usable']*12, step_ns=1_000_000)
        self.assertFalse(window.summary()['complete'])
        self.assertEqual(window.summary()['valid_count'], 12)

    def test_pre_arrival_delayed_and_future_results_do_not_count(self):
        window = ObservationWindow(1_000_000_000)
        self.assertFalse(window.add(observation(900_000_000), 1_100_000_000, 1.))
        self.assertFalse(window.add(observation(1_000_000_000), 1_100_000_000, 2.))
        self.assertFalse(window.add(observation(1_100_000_000), 1_700_000_000, 3.))
        self.assertFalse(window.add(observation(2_000_000_000), 1_800_000_000, 4.))
        result = window.summary()
        self.assertEqual(result['sample_count'], 0)
        self.assertEqual(result['errors'], {'pre_cutoff': 2, 'stale_image': 1,
                                           'future_image': 1})

    def test_age_boundary_and_strict_cutoff(self):
        window = ObservationWindow(1_000_000_000)
        self.assertTrue(window.add(observation(1_000_000_001), 1_500_000_001, 1.))
        self.assertFalse(window.add(observation(1_000_000_002), 1_500_000_003, 2.))
        self.assertEqual(window.summary()['sample_count'], 1)

    def test_identical_duplicate_does_not_inflate_counter_or_wall_span(self):
        window = ObservationWindow(1_000_000_000)
        message = observation(1_100_000_000)
        self.assertTrue(window.add(message, 1_110_000_000, 1.))
        for index in range(20):
            self.assertFalse(window.add(message, 1_110_000_000, 2.+index))
        result = window.summary()
        self.assertEqual(result['sample_count'], 1)
        self.assertEqual(result['duplicate_count'], 20)
        self.assertEqual(result['longest_usable_run'], 1)
        self.assertEqual(result['receipt_span_sec'], 0)
        self.assertFalse(result['recovered'])

    def test_same_stamp_watchdog_revokes_even_after_source_becomes_stale(self):
        window = ObservationWindow(1_000_000_000)
        fill(window, ['usable']*12)
        self.assertTrue(window.summary()['recovered'])
        timeout = observation(2_200_000_000, 'input_timeout')
        self.assertFalse(window.add(timeout, 4_000_000_000, 20.))
        self.assertFalse(window.add(timeout, 4_100_000_000, 21.))
        result = window.summary()
        self.assertEqual(result['sample_count'], 12)
        self.assertEqual(result['valid_count'], 11)
        self.assertEqual(result['invalid_count'], 1)
        self.assertEqual(result['invalidation_count'], 2)
        self.assertEqual(result['status_counts']['input_timeout'], 1)
        self.assertEqual(result['status_event_counts']['input_timeout'], 2)
        self.assertFalse(result['complete'])
        self.assertFalse(result['recovered'])

    def test_same_stamp_success_cannot_repair_revoked_source(self):
        window = ObservationWindow(1_000_000_000, min_samples=1, min_span_sec=0)
        window.add(observation(1_100_000_000), 1_110_000_000, 1.)
        window.add(observation(1_100_000_000, 'input_timeout'), 1_120_000_000, 2.)
        window.add(observation(1_100_000_000), 1_130_000_000, 3.)
        result = window.summary()
        self.assertEqual(result['valid_count'], 0)
        self.assertFalse(result['latest_input_valid'])
        self.assertEqual(result['errors']['retry_after_invalidation'], 1)

    def test_invalid_new_frames_reduce_fraction_and_do_not_meet_valid_minimum(self):
        window = ObservationWindow(1_000_000_000)
        fill(window, ['usable']*10+['invalid_image']*2)
        result = window.summary()
        self.assertEqual(result['sample_count'], 12)
        self.assertEqual(result['valid_count'], 10)
        self.assertAlmostEqual(result['usable_fraction'], 10/12)
        self.assertFalse(result['complete'])

    def test_threshold_fraction_and_post_arrival_run(self):
        accepted = ObservationWindow(1_000_000_000)
        fill(accepted, ['warming_up']*2+['usable']*10)
        self.assertTrue(accepted.summary()['recovered'])
        rejected = ObservationWindow(1_000_000_000)
        fill(rejected, ['not_detected']*3+['usable']*9)
        self.assertTrue(rejected.summary()['complete'])
        self.assertFalse(rejected.summary()['recovered'])
        one_frame = ObservationWindow(1_000_000_000, min_samples=1, min_span_sec=0)
        one_frame.add(observation(1_100_000_000), 1_110_000_000, 1.)
        self.assertTrue(one_frame.summary()['complete'])
        self.assertFalse(one_frame.summary()['recovered'])

    def test_confidence_and_size_use_valid_detections_not_missing_frames(self):
        window = ObservationWindow(1_000_000_000)
        for index, confidence in enumerate((.3, .6, .9)):
            stamp = 1_100_000_000+index*100_000_000
            message = observation(stamp, 'low_confidence', confidence=confidence,
                                  width=80.+80*index, height=16.+16*index,
                                  border_margin_ratio=.01+.01*index)
            window.add(message, stamp, 1.+index)
        window.add(observation(1_400_000_000, 'not_detected'), 1_400_000_000, 4.)
        result = window.summary()
        self.assertAlmostEqual(result['confidence_median'], .6)
        self.assertAlmostEqual(result['confidence_p10'], .36)
        self.assertEqual(result['width_median_px'], 160.)
        self.assertEqual(result['height_median_px'], 32.)
        self.assertAlmostEqual(result['border_margin_median_ratio'], .02)
        self.assertEqual(result['detected_count'], 3)

    def test_frame_or_resolution_change_latches_fault(self):
        for mutation, reason in ((lambda m: m['header'].update(frame_id='other'),
                                  'frame_changed'),
                                 (lambda m: m.update(image_width=640),
                                  'resolution_changed')):
            window = ObservationWindow(1_000_000_000, min_samples=1, min_span_sec=0)
            fill(window, ['usable']*3)
            altered = observation(1_400_000_000)
            mutation(altered)
            self.assertFalse(window.add(altered, 1_400_000_000, 20.))
            result = window.summary()
            self.assertEqual(result['stream_error'], reason)
            self.assertFalse(result['complete'])
            self.assertFalse(result['recovered'])

    def test_clock_or_receipt_regression_latches_fault(self):
        for now, wall, reason in ((1_200_000_000, 2., 'ros_time_regression'),
                                  (1_400_000_000, .5, 'receipt_time_regression')):
            window = ObservationWindow(1_000_000_000)
            window.add(observation(1_100_000_000), 1_300_000_000, 1.)
            self.assertFalse(window.add(observation(1_200_000_000), now, wall))
            self.assertEqual(window.summary()['stream_error'], reason)

    def test_conflicting_duplicate_latches_fault_without_extra_sample(self):
        window = ObservationWindow(1_000_000_000)
        first = observation(1_100_000_000)
        window.add(first, 1_100_000_000, 1.)
        window.add(observation(1_100_000_000, confidence=.8), 1_200_000_000, 2.)
        result = window.summary()
        self.assertEqual(result['sample_count'], 1)
        self.assertEqual(result['stream_error'], 'conflicting_duplicate')

    def test_late_distinct_frame_counts_in_source_order(self):
        window = ObservationWindow(1_000_000_000, min_samples=3, min_span_sec=.2)
        for index, stamp in enumerate((1_100_000_000, 1_300_000_000, 1_200_000_000)):
            window.add(observation(stamp), 1_400_000_000, float(index))
        result = window.summary()
        self.assertTrue(result['recovered'])
        self.assertEqual(result['longest_usable_run'], 3)
        self.assertAlmostEqual(result['source_span_sec'], .2)

    def test_owned_records_do_not_change_when_caller_mutates_message(self):
        window = ObservationWindow(1_000_000_000)
        message = observation(1_100_000_000)
        window.add(message, 1_100_000_000, 1.)
        message['confidence'] = .1
        message['header']['stamp']['sec'] = 40
        self.assertEqual(window.summary()['confidence_median'], .7)
        self.assertEqual(window.summary()['last_source_stamp_ns'], 1_100_000_000)

    def test_malformed_message_does_not_create_evidence(self):
        mutations = [lambda m: m.update(confidence=math.nan),
                     lambda m: m.update(confidence=2),
                     lambda m: m.update(width=0),
                     lambda m: m.update(input_valid='yes'),
                     lambda m: m.update(input_valid=False),
                     lambda m: m.update(image_width=0),
                     lambda m: m['header'].update(frame_id=''),
                     lambda m: m['header']['stamp'].update(nanosec=1_000_000_000)]
        for mutation in mutations:
            window = ObservationWindow(1_000_000_000)
            message = observation(1_100_000_000)
            mutation(message)
            self.assertFalse(window.add(message, 1_100_000_000, 1.))
            self.assertEqual(window.summary()['sample_count'], 0)

    def test_bad_constructor_settings_raise(self):
        for kwargs in ({'cutoff_ros_ns': -1}, {'cutoff_ros_ns': True},
                       {'cutoff_ros_ns': 0, 'min_samples': 0},
                       {'cutoff_ros_ns': 0, 'min_span_sec': math.nan},
                       {'cutoff_ros_ns': 0, 'max_age_sec': 0}):
            with self.assertRaises(ValueError):
                ObservationWindow(**kwargs)


class ExactImageEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.window = ObservationWindow(1_000_000_000)
        self.message = observation(1_100_000_000)
        self.window.add(self.message, 1_110_000_000, 1.)
        self.observations = {('camera_optical', 1_100_000_000): self.message}

    def match(self, header=None, width=1280, height=720):
        return match_measurement_image(
            self.message['header'] if header is None else header,
            width, height, self.window, self.observations)

    def test_exact_measured_image_matches_and_metadata_is_owned(self):
        result = self.match()
        self.assertEqual(result, self.message)
        result['confidence'] = .1
        result['header']['stamp']['sec'] = 99
        self.assertEqual(self.message['confidence'], .7)
        self.assertEqual(self.message['header']['stamp']['sec'], 1)

    def test_one_nanosecond_offset_and_unmeasured_source_do_not_match(self):
        for stamp in (1_100_000_001, 1_200_000_000):
            message = observation(stamp)
            self.observations[('camera_optical', stamp)] = message
            self.assertIsNone(self.match(message['header']))

    def test_pre_cutoff_and_stale_rejected_images_do_not_match(self):
        for stamp in (1_000_000_000, 1_100_000_001):
            message = observation(stamp)
            self.assertFalse(self.window.add(message, 2_000_000_000, 2.))
            self.observations[('camera_optical', stamp)] = message
            self.assertIsNone(self.match(message['header']))

    def test_wrong_frame_or_image_resolution_does_not_match(self):
        self.assertIsNone(self.match({'frame_id': 'other',
                                     'stamp': self.message['header']['stamp']}))
        self.assertIsNone(self.match(width=640))
        self.assertIsNone(self.match(height=480))

    def test_same_source_watchdog_invalidation_revokes_image_evidence(self):
        self.window.add(observation(1_100_000_000, 'input_timeout'),
                        2_000_000_000, 2.)
        # Cached original metadata remains usable; the current window owns
        # the invalidation and must prevent an old success screenshot pairing.
        self.assertTrue(self.message['usable'])
        self.assertIsNone(self.match())

    def test_faulted_window_rejects_previously_matching_image(self):
        changed = observation(1_200_000_000)
        changed['header']['frame_id'] = 'other'
        self.window.add(changed, 1_210_000_000, 2.)
        self.assertEqual(self.window.stream_error, 'frame_changed')
        self.assertIsNone(self.match())

    def test_invalid_or_inconsistent_original_metadata_is_not_paired(self):
        for field, bad in (('input_valid', False), ('image_width', 640),
                           ('confidence', .9), ('status', 'not_detected')):
            original = self.message[field]
            self.message[field] = bad
            self.assertIsNone(self.match())
            self.message[field] = original
        original_header = self.message['header']
        self.message['header'] = observation(1_100_000_001)['header']
        self.assertIsNone(self.match(original_header))
        self.message['header'] = original_header

    def test_invalid_and_missing_header_dimensions_or_metadata_are_rejected(self):
        for header in ({}, {'frame_id': 'camera_optical', 'stamp': {}},
                       {'frame_id': 'camera_optical',
                        'stamp': {'sec': 1, 'nanosec': 1_000_000_000}}):
            self.assertIsNone(self.match(header))
        self.assertIsNone(self.match(width=0))
        self.assertIsNone(self.match(width=True))
        self.observations.clear()
        self.assertIsNone(self.match())


if __name__ == '__main__':
    unittest.main()
