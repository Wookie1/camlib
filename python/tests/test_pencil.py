from dataclasses import replace
import math
import unittest

import camlib
from camlib.cadquery import export_surface_mesh
from camlib.simulation import OperationUpdate, Simulation
from camlib.tools import BallEndMill, HolderSection, ToolAssembly


def part():
    import cadquery as cq

    base = cq.Workplane("XY").box(24, 16, 4, centered=False)
    boss = cq.Workplane("XY").box(8, 8, 3, centered=False).translate((8, 4, 4))
    return base.union(boss)


def boss_distance(x, y):
    """XY distance from the 8 x 8 boss footprint (8..16, 4..12)."""
    dx = max(8 - x, 0, x - 16)
    dy = max(4 - y, 0, y - 12)
    return math.hypot(dx, dy)


class PencilTest(unittest.TestCase):
    def test_traces_the_inside_corner_around_the_boss(self):
        surface = export_surface_mesh(part(), source=dict(model="stepped", revision="1"))
        tool = ToolAssembly(
            "ball4",
            BallEndMill("be4", diameter=4, flute_length=20, overall_length=50),
            30,
            60,
            (HolderSection(30, 60, 4, 4),),
        )
        op = camlib.Pencil(
            "pencil",
            tool,
            surface,
            0,
            12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        # Pencil straight out of fresh 8 mm stock: its entries plunge ~4 mm
        # into that stock with no ramp proven clear (audit #22), refused by
        # default.
        with self.assertRaisesRegex(Exception, "no ramp along the row is proven clear"):
            with Simulation(
                stock=camlib.StockSpec((0, 0, 0), (24, 16, 8), 0.25),
                tools=(tool,),
                operations=(op,),
                revision="1",
                initial_position_mm=(0, 0, 12),
            ) as sim:
                for _ in sim.run_operations():
                    pass
        # This test checks the corner geometry: opt in, and check below that
        # every such plunge is listed for independent verification.
        op = replace(op, plunge_unproven_entries=True)
        cuts, plunges, report = [], [], None
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
                motion = getattr(update, "motion", None)
                if motion is not None and motion["kind"] == "cut":
                    cuts.append(motion["curve"])
                if motion is not None and motion["kind"] == "plunge":
                    plunges.append(motion["curve"]["end"])
            self.assertTrue(sim.state.finished)
        self.assertEqual(report["strategy"], "pencil")
        listed = report["unproven_entries"]
        self.assertTrue(listed)
        for e in listed:
            self.assertIn([e["x_mm"], e["y_mm"], e["z_mm"]], [list(p) for p in plunges])
        # The first entry goes into untouched stock (the box to Z8): its
        # real depth there is the independent check that it was listed.
        first = plunges[0]
        self.assertGreater(8 - first[2], 0.25)
        self.assertIn([first[0], first[1], first[2]], [[e["x_mm"], e["y_mm"], e["z_mm"]] for e in listed])
        # Cuts along the boss corner: tip on the floor (Z 4), ball center
        # 2.005 from the walls (radius plus safety margin).
        corner = [
            c
            for c in cuts
            if abs(c["start"][2] - 4) < 0.02
            and abs(c["end"][2] - 4) < 0.02
            and abs(boss_distance(*c["start"][:2]) - 2.005) < 0.05
        ]
        length = sum(math.dist(c["start"][:2], c["end"][:2]) for c in corner)
        self.assertGreater(length, 40, length)  # 4 x 8 + 2 pi 2 = 44.6 around the boss
        for c in cuts:
            for p in (c["start"], c["end"]):
                # Creases with the machining base (outside the part) are not pencil paths.
                self.assertTrue(-0.01 <= p[0] <= 24.01 and -0.01 <= p[1] <= 16.01, p)
                # Never inside the boss or below the floor.
                self.assertGreaterEqual(p[2], -1e-6)
                if boss_distance(*p[:2]) < 2:
                    self.assertGreaterEqual(p[2], 7 - 1e-6, p)


if __name__ == "__main__":
    unittest.main()
