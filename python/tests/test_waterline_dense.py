"""Waterline on a dense triangle mesh (as from tessellated CAD or scans).

Regression: a 12,800-triangle dome used to fail with fixed work caps
("surface contact/clearance work limit exceeded"). Contours must complete and
keep the documented normal clearance to the surface (0..0.015 mm to the mesh).
"""

import math
import unittest

from camlib.jobs import Job, Setup
from camlib.operations import Waterline
from camlib.simulation import StockSpec
from camlib.surface import SurfaceMesh
from camlib.tools import BallEndMill, HolderSection, ToolAssembly

R, ZC, C = 20.0, -15.0, 20.0  # sphere radius, centre height (cap 5 mm), centre x/y


def dome_mesh(h=0.5):
    def z(x, y):
        d2 = (x - C) ** 2 + (y - C) ** 2
        return max(0.0, ZC + math.sqrt(R * R - d2)) if d2 < R * R else 0.0

    n = int(40 / h)
    triangles = []
    for i in range(n):
        for j in range(n):
            p = [
                (x, y, z(x, y))
                for x, y in ((i * h, j * h), ((i + 1) * h, j * h), ((i + 1) * h, (j + 1) * h), (i * h, (j + 1) * h))
            ]
            triangles += [(p[0], p[1], p[2]), (p[0], p[2], p[3])]
    return SurfaceMesh(triangles)


class DenseWaterlineTest(unittest.TestCase):
    def test_dense_mesh_waterline_completes_within_normal_clearance(self):
        ball = ToolAssembly(
            "ball4",
            BallEndMill("b4", diameter=4, flute_length=12, overall_length=60),
            30,
            70,
            (HolderSection(30, 70, 4, 4),),
        )
        op = Waterline(
            "w",
            ball,
            dome_mesh(),
            floor_z_mm=0,
            top_z_mm=3,
            bottom_z_mm=2,
            stepdown_mm=0.5,
            clearance_z_mm=12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        setup = Setup("top", StockSpec((0, 0, 0), (40, 40, 7), 0.5), (ball,), (op,), (0, 0, 15))
        clearances = []
        with Job("dense", "1", (setup,)).regenerate() as run:
            for update in run:
                motion = getattr(update.update, "motion", None)
                if isinstance(motion, dict) and motion["kind"] == "cut":
                    x, y, tip = motion["curve"]["end"]
                    centre = (x, y, tip + 2.0)
                    clearances.append(math.dist(centre, (C, C, ZC)) - R - 2.0)
        self.assertGreater(len(clearances), 100)
        # The mesh lies up to h^2/(8R) = 0.0016 mm below the true sphere.
        self.assertGreaterEqual(min(clearances), -0.002)
        self.assertLessEqual(max(clearances), 0.016)


if __name__ == "__main__":
    unittest.main()
