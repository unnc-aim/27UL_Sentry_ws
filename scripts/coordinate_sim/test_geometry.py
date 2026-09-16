"""Frame invariants used by the independent simulation odometry/actuator adapters."""
import math
import unittest

from scenario import GOALS, clearance, rotate, wrap


class FrameInvariants(unittest.TestCase):
    def test_map_motion_survives_body_and_gimbal_rotation(self):
        # Map vector converted by Nav2 to body, bridge to gimbal, simulator back
        # to body, physics back to world: the physical vector must be unchanged.
        for body in (0., .7, math.pi/2, math.pi, -2.3, 4*math.pi+.4):
            for gimbal in (0., .9, -math.pi/2, math.pi, 6*math.pi+.2):
                for desired in ((1., 0.), (0., 1.), (-.3, .8)):
                    local = rotate(*desired, -body)
                    output = rotate(*local, -gimbal)
                    physical = rotate(*rotate(*output, gimbal), body)
                    for expected, actual in zip(desired, physical):
                        self.assertAlmostEqual(expected, actual, places=12)

    def test_relative_odometry_does_not_reveal_world_origin(self):
        for x, y, a in ((-2.5, 0., 1.2), (2., -1., -2.)):
            local = rotate(x-x, y-y, -a)
            self.assertEqual(local, (0., 0.))
            self.assertEqual(wrap(a-a), 0.)
            dx, dy = rotate(.5, -.2, a)
            recovered = rotate(dx, dy, -a)
            self.assertAlmostEqual(recovered[0], .5)
            self.assertAlmostEqual(recovered[1], -.2)

    def test_landmarks_have_robot_clearance(self):
        for x, y, _ in GOALS.values():
            self.assertGreater(clearance(x, y), .6)


if __name__ == '__main__':
    unittest.main()
