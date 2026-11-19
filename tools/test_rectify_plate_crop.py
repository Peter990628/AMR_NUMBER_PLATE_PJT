import itertools
import unittest
import cv2
import numpy as np
from rectify_plate_crop import order_quad, prepare_variants, warp_quad, find_plate_quad


class RectificationTests(unittest.TestCase):
    def test_corner_mapping_and_order_independent_of_input(self):
        quad = np.float32([[12, 15], [185, 9], [175, 70], [18, 75]])
        image = np.zeros((90, 200, 3), np.uint8)
        expected = np.float32([[0, 0], [519, 0], [519, 109], [0, 109]])
        for permutation in itertools.permutations(quad.tolist()):
            ordered = order_quad(permutation)
            np.testing.assert_allclose(ordered, quad)
            warped, matrix = warp_quad(image, permutation, (520, 110))
            actual = cv2.perspectiveTransform(quad[None], matrix)[0]
            np.testing.assert_allclose(actual, expected, atol=1e-4)
            self.assertEqual(warped.shape, (110, 520, 3))

    def test_invalid_corners(self):
        for points in ([[0, 0]]*4, [[0, 0], [1, 0], [2, 0], [3, 0]],
                       [[0, 0], [10, 0], [10, 10], [float('nan'), 10]]):
            with self.assertRaises(ValueError):
                order_quad(points)

    def test_missing_boundary_retains_original(self):
        image = np.full((80, 240, 3), 126, np.uint8)
        variants, diagnostic = prepare_variants(image)
        self.assertEqual(diagnostic['status'], 'no_quad_original_retained')
        self.assertIsNone(diagnostic['transform'])
        for name in ('original', 'recrop_resize', 'perspective'):
            np.testing.assert_array_equal(variants[name], image)
        self.assertEqual(variants['resize_only'].shape, (110, 520, 3))

    def test_colored_detector_border_is_not_plate(self):
        image = np.zeros((80, 240, 3), np.uint8)
        cv2.rectangle(image, (10, 10), (230, 70), (0, 255, 0), 3)
        quad, _ = find_plate_quad(image)
        self.assertIsNone(quad)


if __name__ == '__main__':
    unittest.main()
