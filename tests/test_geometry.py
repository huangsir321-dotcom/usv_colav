import math
import unittest
from usv_colav.geometry import cpa_tcpa, velocity_from_course, wrap180

class GeometryTests(unittest.TestCase):
    def test_heading_convention(self):
        vx, vy = velocity_from_course(10, 0)
        self.assertAlmostEqual(vx, 0, places=6)
        self.assertGreater(vy, 0)
        vx, vy = velocity_from_course(10, 90)
        self.assertGreater(vx, 0)
        self.assertAlmostEqual(vy, 0, places=6)

    def test_head_on_cpa(self):
        # Ownship is 1000 m south of the target; both move toward each other at 10 kn.
        result = cpa_tcpa((0, 0), 10, 0, (0, 1000), 10, 180)
        self.assertAlmostEqual(result.cpa_m, 0, delta=1e-5)
        self.assertAlmostEqual(result.tcpa_min, 1000 / (2 * 10 * 0.514444) / 60, places=3)

    def test_parallel_same_velocity(self):
        result = cpa_tcpa((0, 0), 10, 0, (100, 0), 10, 0)
        self.assertAlmostEqual(result.cpa_m, 100, places=6)
        self.assertTrue(math.isinf(result.tcpa_min))

    def test_wrap(self):
        self.assertAlmostEqual(wrap180(359), -1)
        self.assertAlmostEqual(wrap180(-181), 179)

if __name__ == '__main__':
    unittest.main()
