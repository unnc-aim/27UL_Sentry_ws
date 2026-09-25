"""No ROS or actuator access."""
import unittest
from obstacle_hold import ObstacleHold


class HoldTests(unittest.TestCase):
    def test_single_return_stops_before_clear_recovery(self):
        h = ObstacleHold()
        self.assertFalse(h.update(0., 1., True))
        self.assertTrue(h.update(.1, .35, True))
        self.assertTrue(h.update(.2, 1., True))
        self.assertTrue(h.update(1., 1., True))
        self.assertFalse(h.update(1.3, 1., True))

    def test_hysteresis_and_localization_reset_recovery(self):
        h = ObstacleHold()
        for now, distance, ready in [(0., .35, True), (.2, 1., True),
                                     (.9, .44, True), (1., 1., True),
                                     (1.9, 1., False), (2., 1., True)]:
            self.assertTrue(h.update(now, distance, ready))
        self.assertFalse(h.update(3.1, 1., True))

    def test_persistent_obstacle_times_out(self):
        h = ObstacleHold()
        self.assertTrue(h.update(0., .35, True))
        with self.assertRaises(RuntimeError):
            h.update(10., .35, True)

    def test_invalid_data_cannot_resume(self):
        for distance in [float('nan'), float('inf'), -1.]:
            with self.assertRaises(ValueError):
                ObstacleHold().update(0., distance, True)


if __name__ == '__main__':
    unittest.main()
