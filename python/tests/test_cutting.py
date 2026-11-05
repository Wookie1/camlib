import math
import unittest

from camlib.cutting import cutting_data
from camlib.tools import BallEndMill, EndMill


class CuttingDataTest(unittest.TestCase):
    def test_standard_formulas(self):
        mill = EndMill("em10", diameter=10, flute_length=22, overall_length=72, flutes=4)
        # Handbook example: Vc 200 m/min, D 10 -> n = 6366 rpm; fz 0.05, z 4 -> 1273 mm/min.
        data = cutting_data(mill, surface_speed_m_min=200, feed_per_tooth_mm=0.05)
        self.assertAlmostEqual(data.spindle_rpm, 200000 / (math.pi * 10))
        self.assertAlmostEqual(data.feed_mm_min, data.spindle_rpm * 4 * 0.05)
        self.assertFalse(data.rpm_limited)

    def test_chip_thinning_keeps_chip_thickness(self):
        mill = EndMill("em10", diameter=10, flute_length=22, overall_length=72, flutes=4)
        data = cutting_data(mill, surface_speed_m_min=200, feed_per_tooth_mm=0.05, radial_engagement_mm=1)
        # Maximum chip thickness at the programmed fz equals the requested fz.
        self.assertAlmostEqual(data.feed_per_tooth_mm * 2 * math.sqrt(1 * 9) / 10, 0.05)
        half = cutting_data(mill, surface_speed_m_min=200, feed_per_tooth_mm=0.05, radial_engagement_mm=5)
        self.assertEqual(half.feed_per_tooth_mm, 0.05)
        plain = cutting_data(
            mill, surface_speed_m_min=200, feed_per_tooth_mm=0.05, radial_engagement_mm=1, chip_thinning=False
        )
        self.assertEqual(plain.feed_per_tooth_mm, 0.05)

    def test_ball_effective_diameter_and_rpm_cap(self):
        ball = BallEndMill("b6", diameter=6, flute_length=12, overall_length=50, flutes=2)
        data = cutting_data(ball, surface_speed_m_min=150, feed_per_tooth_mm=0.04, axial_depth_mm=0.5)
        self.assertAlmostEqual(data.effective_diameter_mm, 2 * math.sqrt(0.5 * 5.5))
        self.assertAlmostEqual(data.spindle_rpm, 150000 / (math.pi * data.effective_diameter_mm))
        capped = cutting_data(ball, surface_speed_m_min=150, feed_per_tooth_mm=0.04, axial_depth_mm=0.5, max_rpm=12000)
        self.assertTrue(capped.rpm_limited)
        self.assertEqual(capped.spindle_rpm, 12000)
        self.assertAlmostEqual(capped.feed_mm_min, 12000 * 2 * 0.04)

    def test_invalid_input_is_refused(self):
        mill = EndMill("em10", diameter=10, flute_length=22, overall_length=72)
        for kwargs in (
            dict(surface_speed_m_min=0, feed_per_tooth_mm=0.05),
            dict(surface_speed_m_min=200, feed_per_tooth_mm=float("nan")),
            dict(surface_speed_m_min=200, feed_per_tooth_mm=0.05, radial_engagement_mm=11),
        ):
            with self.assertRaises(ValueError):
                cutting_data(mill, **kwargs)


if __name__ == "__main__":
    unittest.main()
