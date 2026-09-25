"""Offline command-limit regression checks; never creates ROS publishers."""
import math
import unittest

from speed_profile import limited_velocity


class SpeedProfileTests(unittest.TestCase):
    def test_diagonal_limit_preserves_direction(self):
        vx, vy = limited_velocity(3., 4., 1., 1., .05, 10.)
        self.assertAlmostEqual(math.hypot(vx, vy), 1.)
        self.assertAlmostEqual(vx / vy, .75)

    def test_start_and_delayed_iteration_cannot_jump(self):
        for dt, expected in [(.05, .015), (10., .03)]:
            vx, _ = limited_velocity(1., 0., 1., 0., dt, 10.)
            self.assertAlmostEqual(vx, expected)

    def test_reductions_and_stop_are_immediate(self):
        self.assertEqual(limited_velocity(0., 0., 1., 1., .05, 10.), (0., 0.))
        vx, _ = limited_velocity(.1, 0., 1., 1., .05, 10.)
        self.assertAlmostEqual(vx, .1)
        vx, _ = limited_velocity(1., 0., 1., 1., .05, .12)
        self.assertAlmostEqual(vx, .15)
        vx, _ = limited_velocity(1., 0., .1, 1., .05, .12)
        self.assertAlmostEqual(vx, .1)

    def test_invalid_inputs(self):
        baseline = [1., 0., 1., 0., .05, 10.]
        for index in range(len(baseline)):
            for invalid in [math.nan, math.inf, -math.inf]:
                values = baseline.copy()
                values[index] = invalid
                with self.assertRaises(ValueError):
                    limited_velocity(*values)
        for index, invalid in [(2, 0.), (2, 1.01), (3, -.1), (4, -.1), (5, -.1)]:
            values = baseline.copy()
            values[index] = invalid
            with self.assertRaises(ValueError):
                limited_velocity(*values)


if __name__ == '__main__':
    unittest.main()
