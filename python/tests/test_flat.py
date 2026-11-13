import unittest

import camlib
from camlib.cadquery import export_surface_mesh
import math

from camlib.simulation import OperationUpdate, Simulation, Update
from camlib.tools import HolderSection, ToolAssembly


def part():
    import cadquery as cq

    base = cq.Workplane("XY").box(24, 16, 4, centered=False)
    boss = cq.Workplane("XY").box(8, 8, 3, centered=False).translate((8, 4, 4))
    return base.union(boss)


def target(x, y):
    return 7 if 8 <= x <= 16 and 4 <= y <= 12 else 4


class FlatTest(unittest.TestCase):
    def test_finishes_each_flat_at_its_height_without_touching_walls(self):
        surface = export_surface_mesh(part(), source=dict(model="stepped", revision="1"))
        tool = ToolAssembly(
            "mill4",
            camlib.EndMill("em4", diameter=4, flute_length=20, overall_length=50),
            30,
            60,
            (HolderSection(30, 60, 4, 4),),
        )
        op = camlib.Flat(
            "flats",
            tool,
            surface,
            0,
            1.5,
            12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        report = None
        with Simulation(
            stock=camlib.StockSpec((0, 0, 0), (24, 16, 8), 0.25),
            tools=(tool,),
            operations=(op,),
            revision="1",
            initial_position_mm=(0, 0, 12),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
            self.assertTrue(sim.state.finished)
            self.assertEqual(report["levels_z_mm"], [7, 4])
            self.assertEqual(report["runs_completed"], report["runs"])
            s = sim.state
            finished = 0
            for j in range(s.ny):
                for i in range(s.nx):
                    x0, x1, y0, y1 = i * s.dx, (i + 1) * s.dx, j * s.dy, (j + 1) * s.dy
                    upper = s.heights[2 * (j * s.nx + i) + 1]
                    corners = [target(x, y) for x in (x0, x1) for y in (y0, y1)]
                    # Never below the part (conservative cell interval).
                    self.assertGreaterEqual(upper, min(corners) - 1e-6, (i, j))
                    # Open flat areas 2.5 mm from the boss walls are finished.
                    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                    away = min(abs(cx - 8), abs(cx - 16), abs(cy - 4), abs(cy - 12)) > 2.5
                    if away and len(set(corners)) == 1:
                        self.assertLessEqual(upper, corners[0] + 1e-4, (i, j, upper))
                        finished += 1
            self.assertGreater(finished, 1000)

    def test_runs_under_stock_are_entered_by_a_shallow_ramp_not_a_plunge(self):
        # Fresh 8 mm stock stands over both flats (7 and 4): no straight
        # plunge may take the flat end mill down through it; each such run
        # is entered by zigzagging along the run at 3 degrees.
        surface = export_surface_mesh(part(), source=dict(model="stepped", revision="1"))
        tool = ToolAssembly(
            "mill4",
            camlib.EndMill("em4", diameter=4, flute_length=20, overall_length=50),
            30,
            60,
            (HolderSection(30, 60, 4, 4),),
        )
        op = camlib.Flat(
            "flats",
            tool,
            surface,
            0,
            1.5,
            12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        report, motions = None, []
        with Simulation(
            stock=camlib.StockSpec((0, 0, 0), (24, 16, 8), 0.25),
            tools=(tool,),
            operations=(op,),
            revision="1",
            initial_position_mm=(0, 0, 12),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, Update):
                    motions.append(update.motion)
                elif isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
        self.assertGreater(report["ramped_entries"], 0)
        slope = math.tan(math.radians(3)) + 1e-9
        plunges = [m for m in motions if m["kind"] == "plunge"]
        # The first run starts under untouched stock: its plunge stops above
        # it (later runs may start where earlier rows already cut).
        self.assertGreaterEqual(plunges[0]["curve"]["end"][2], 8 - 1e-9)
        for m in motions:
            a, b = m["curve"]["start"], m["curve"]["end"]
            run = math.hypot(b[0] - a[0], b[1] - a[1])
            if m["kind"] != "rapid" and b[2] < a[2] and run > 0:
                self.assertLessEqual((a[2] - b[2]) / run, slope, m)


if __name__ == "__main__":
    unittest.main()
