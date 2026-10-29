import math
import unittest

import numpy as np

from raspbot_chassis.vision import detect


class VisionTests(unittest.TestCase):
    def test_centered_tape_and_live_images(self):
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        frame[:, 300:340] = 0
        result = detect(frame)
        self.assertTrue(result.visible)
        self.assertAlmostEqual(result.offset_px, -0.5, delta=2)
        self.assertEqual(result.annotated.shape, frame.shape)
        self.assertEqual(result.edges.shape, frame.shape[:2])
        self.assertEqual(result.mask.shape, frame.shape[:2])

    def test_tape_right_and_left(self):
        for left, expected_sign in [(400, 1), (150, -1)]:
            frame = np.full((480, 640, 3), 255, dtype=np.uint8)
            frame[:, left:left + 40] = 0
            result = detect(frame)
            self.assertTrue(result.visible)
            self.assertEqual(math.copysign(1, result.offset_px), expected_sign)
            self.assertAlmostEqual(result.offset_normalized, result.offset_px / 320)

    def test_lost_tape_clears_offset(self):
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        result = detect(frame, previous_center=400)
        self.assertFalse(result.visible)
        self.assertIsNone(result.center_px)
        self.assertTrue(math.isnan(result.offset_px))
        self.assertTrue(math.isnan(result.offset_normalized))

    def test_previous_center_selects_nearest_stripe(self):
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        frame[:, 100:140] = 0
        frame[:, 400:440] = 0
        self.assertAlmostEqual(detect(frame, previous_center=420).center_px, 419.5, delta=2)
        self.assertAlmostEqual(detect(frame, previous_center=120).center_px, 119.5, delta=2)


if __name__ == '__main__':
    unittest.main()
