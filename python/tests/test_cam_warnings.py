"""Valid-but-suspicious plans are reported, not silent."""

import unittest
import warnings

from camlib.jobs import Job, Setup
from camlib.operations import Pocket
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import CamWarning, StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def run(diameter, square):
    tool = ToolAssembly(
        "m",
        EndMill("e", diameter=diameter, flute_length=20, overall_length=60),
        30,
        70,
        (HolderSection(30, 70, 10, 10),),
    )
    (x0, y0), (x1, y1) = square
    region = PlanarRegion("p", (Component(Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))),))
    pocket = Pocket(
        "p",
        tool,
        region,
        top_z_mm=10,
        floor_z_mm=7,
        stepdown_mm=1.5,
        stepover_mm=2,
        clearance_z_mm=12,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    setup = Setup("top", StockSpec((0, 0, 0), (20, 20, 10), 0.5), (tool,), (pocket,), (0, 0, 12))
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with Job("w", "1", (setup,)).regenerate() as job:
            for _ in job:
                pass
    return [str(w.message) for w in caught if issubclass(w.category, CamWarning)]


class CamWarningTest(unittest.TestCase):
    def test_empty_and_air_only_operations_warn_and_normal_ones_do_not(self):
        self.assertEqual(run(6, ((5, 5), (15, 15))), [])
        self.assertIn("no cutting motion", " ".join(run(12, ((5, 5), (15, 15)))))
        self.assertIn("removed no stock", " ".join(run(6, ((50, 50), (60, 60)))))


if __name__ == "__main__":
    unittest.main()
