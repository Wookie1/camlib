"""V-carve: the outline is never overcut and the depth follows the width."""

import math
import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import Simulation, SimulationError, VerificationUpdate


def vbit():
    return camlib.ToolAssembly(
        "v90",
        camlib.ChamferMill("v90", angle=90, tip_diameter=0, diameter=12, flute_length=8, overall_length=50),
        30,
        60,
        (camlib.HolderSection(30, 60, 6, 6),),
    )


# An "L": a 4 mm wide stroke, 20 mm tall and 14 mm wide.
L = [(5, 5), (19, 5), (19, 9), (9, 9), (9, 25), (5, 25)]


def config(op):
    return dict(
        stock=camlib.StockSpec((0, 0, 0), (30, 30, 10), 0.1),
        tools=(op.tool,),
        operations=(op,),
        revision="1",
        initial_position_mm=(0, 0, 20),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )


def region():
    return PlanarRegion("L", (Component(Boundary.polygon(L)),), 1e-4)


def inside(p):
    x, y = p
    return (5 <= x <= 19 and 5 <= y <= 9) or (5 <= x <= 9 and 5 <= y <= 25)


def distance_out(p):
    # Distance from p to the L when outside it.
    x, y = p
    a = math.hypot(max(5 - x, 0, x - 19), max(5 - y, 0, y - 9))
    b = math.hypot(max(5 - x, 0, x - 9), max(5 - y, 0, y - 25))
    return min(a, b)


class VCarveTest(unittest.TestCase):
    def test_carves_the_l_without_crossing_its_outline(self):
        op = camlib.VCarve(
            "carve",
            vbit(),
            region(),
            10,
            20,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        statuses = set()
        with Simulation(**config(op)) as sim:
            for update in sim.run_operations():
                if isinstance(update, VerificationUpdate):
                    statuses.add(update.status)
            s = sim.state
            self.assertTrue(s.finished)
            self.assertNotIn("violation", statuses)
            deepest = 10.0
            for j in range(s.ny):
                for i in range(s.nx):
                    c = ((i + 0.5) * s.dx, (j + 0.5) * s.dy)
                    upper = s.heights[2 * (j * s.nx + i) + 1]
                    if not inside(c) and distance_out(c) > s.dx:
                        # Outside the outline (beyond a cell): untouched.
                        self.assertGreaterEqual(upper, 10 - 1e-6, c)
                    deepest = min(deepest, upper)
            # Deepest where the strokes meet: the disk touching both outer
            # walls and the inner corner (9, 9) has its center at c with
            # c - 5 = sqrt(2) (9 - c); a 90 degree bit goes that deep.
            c = (9 * math.sqrt(2) + 5) / (1 + math.sqrt(2))
            self.assertAlmostEqual(deepest, 10 - (c - 5), delta=0.15)
            # Along the 4 mm stroke's middle: 2 mm deep.
            mid = (7, 17)
            i, j = int(mid[0] / s.dx), int(mid[1] / s.dy)
            self.assertAlmostEqual(s.heights[2 * (j * s.nx + i) + 1], 8, delta=0.15)

    def test_avoid_area_below_stock_top_is_respected(self):
        op = camlib.VCarve(
            "carve",
            vbit(),
            region(),
            10,
            20,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        cfg = config(op)
        # The obstacle is below the stock top but inside the carving depth.
        definition = op.to_dict()
        definition["limits"] = camlib.Limits(avoid=(camlib.AvoidArea(region(), 9.5),)).to_dict()
        cfg["operations"] = (definition,)
        with self.assertRaisesRegex(SimulationError, "limits leave nothing"):
            with Simulation(**cfg) as sim:
                list(sim.run_operations())

    def test_too_shallow_is_refused(self):
        op = camlib.VCarve(
            "carve",
            vbit(),
            region(),
            10,
            20,
            max_depth_mm=1,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        with Simulation(**config(op)) as sim:
            with self.assertRaisesRegex(SimulationError, "needs a flat clearing tool"):
                list(sim.run_operations())


if __name__ == "__main__":
    unittest.main()
