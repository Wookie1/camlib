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


def roughed(cell=0.25):
    """The part with 0.1 mm left on the flats and the walls to size: the
    blank a flat finishing pass meets after roughing."""
    import cadquery as cq
    from camlib.cadquery import export_stock_mesh

    blank = (
        cq.Workplane("XY")
        .box(24, 16, 4.1, centered=False)
        .union(cq.Workplane("XY").box(8, 8, 3, centered=False).translate((8, 4, 4.1)))
    )
    return camlib.StockSpec.from_mesh(export_stock_mesh(blank), cell)


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
            entry=camlib.RampEntry(angle_deg=3, min_length_mm=1),
        )
        report = None
        with Simulation(
            stock=roughed(),
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

    def flat(self, entry=None, **changes):
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
            entry=entry,
            **changes,
        )
        return tool, op

    def simulation(self, tool, op, stock=None):
        return Simulation(
            stock=stock or camlib.StockSpec((0, 0, 0), (24, 16, 8), 0.25),
            tools=(tool,),
            operations=(op,),
            revision="1",
            initial_position_mm=(0, 0, 12),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        )

    def test_no_plunge_enters_live_stock_and_ramps_follow_the_declared_entry(self):
        # The blank stands 0.1 mm over both flats (7 and 4). With a declared
        # entry every run is entered by a ramp; independently of the plan,
        # every vertical plunge of the whole operation (runs after the first
        # too) must end at or above the stock under the tool at that moment.
        tool, op = self.flat(camlib.RampEntry(angle_deg=3, min_length_mm=1, feed_mm_min=111))
        report, snapshot = None, None
        plunges, ramp_feeds, slopes = 0, set(), []
        radius = 2.0
        with self.simulation(tool, op, roughed()) as sim:
            s = sim.state
            snapshot = list(s.heights)
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
                if isinstance(update, Update):
                    m = update.motion
                    a, b = m["curve"]["start"], m["curve"]["end"]
                    run = math.hypot(b[0] - a[0], b[1] - a[1])
                    if m["kind"] == "plunge" and run < 1e-9:
                        plunges += 1
                        top = 0.0
                        for j in range(s.ny):
                            for i in range(s.nx):
                                cx, cy = (i + 0.5) * s.dx, (j + 0.5) * s.dy
                                # Whole cells the tool's disk can touch.
                                if math.hypot(cx - b[0], cy - b[1]) <= radius + 0.5 * math.hypot(s.dx, s.dy):
                                    top = max(top, snapshot[2 * (j * s.nx + i) + 1])
                        # Within the operation's tolerance: cut cells keep their cut height
                        # (the flats finished earlier) to that tolerance.
                        self.assertGreaterEqual(b[2], min(top, a[2]) - op.tolerance_mm - 1e-9, (m, top))
                    if m["kind"] == "plunge" and run > 0:
                        ramp_feeds.add(m["feed_mm_min"])
                        slopes.append((a[2] - b[2]) / run)
                    snapshot = list(s.heights)
            self.assertTrue(sim.state.finished)
        # One vertical plunge per run (to the stock's top, or to depth where
        # no stock stands above the run); the others are ramps.
        self.assertEqual(plunges, report["runs"])
        self.assertGreater(report["ramped_entries"], 0)
        self.assertEqual(ramp_feeds, {111})
        self.assertLessEqual(max(slopes), math.tan(math.radians(3)) + 1e-9)

    def test_a_run_under_stock_with_no_entry_is_refused_by_name(self):
        tool, op = self.flat()
        with self.assertRaisesRegex(Exception, r"declares no entry.*entry=RampEntry"):
            with self.simulation(tool, op) as sim:
                for _ in sim.run_operations():
                    pass

    def test_a_ramp_that_does_not_fit_is_refused_not_plunged(self):
        # 4 mm of stock at 3 degrees needs about 76 mm of travel: the ramp
        # must reverse on a 24 mm run, which needs a declared min_length_mm.
        tool, op = self.flat(camlib.RampEntry(angle_deg=3))
        with self.assertRaisesRegex(Exception, "min_length_mm is not set"):
            with self.simulation(tool, op) as sim:
                for _ in sim.run_operations():
                    pass
        tool, op = self.flat(camlib.RampEntry(angle_deg=3, min_length_mm=100))
        with self.assertRaisesRegex(Exception, "shorter than min_length_mm"):
            with self.simulation(tool, op) as sim:
                for _ in sim.run_operations():
                    pass


if __name__ == "__main__":
    unittest.main()
