"""engagement_front rapids lowered by travel_clearance_mm.

With travel_clearance_mm, a rapid between passes travels just above the
whole-cell stock bounds the tool can meet over the rectangle its two ends
span, instead of climbing to retract_z_mm. The stream's own stock-contact
check (independent of the planner) must still find every move clear.
"""

import unittest

from camlib.operations import AdaptiveClearing, Roughing
from camlib.simulation import OperationUpdate, Simulation, StockSpec, Update, VerificationUpdate
from camlib.surface import SurfaceMesh
from camlib.verification import StockContactOptions, StreamChecks

from test_engagement_front_rapid_descent import CUT, TOOL, box

RETRACT = 20


class TravelClearanceTest(unittest.TestCase):
    def run_rough(self, clearance):
        mesh = SurfaceMesh(box(12, 28, 12, 28, 0, 9), 0.0)
        rough = Roughing(
            "rough",
            TOOL,
            mesh,
            top_z_mm=10,
            floor_z_mm=1,
            stepdown_mm=4,
            stepup_mm=1,
            clearance_z_mm=RETRACT,
            retract_z_mm=RETRACT,
            feed_z_mm=11,
            travel_clearance_mm=clearance,
            clearing=AdaptiveClearing(1.0, path_strategy="engagement_front"),
            radial_allowance_mm=0.2,
            bottom_allowance_mm=0.2,
            **CUT,
        )
        statuses, low, rapid_mm, lowered = [], 0, 0.0, 0
        with Simulation(
            stock=StockSpec((0, 0, 0), (40, 40, 10), 0.5),
            tools=(TOOL,),
            operations=(rough,),
            revision="1",
            initial_position_mm=(0, 0, RETRACT),
            verification=StreamChecks(stock_contact=StockContactOptions()),
        ) as sim:
            for u in sim.run_operations():
                if isinstance(u, OperationUpdate) and u.stage == "end":
                    lowered = sum(layer["clearing"].get("lowered_travels", 0) for layer in u.report["layers"])
                elif isinstance(u, Update):
                    c, kind = u.motion["curve"], u.motion["kind"]
                    s, e = c["start"], c["end"]
                    if kind in ("rapid", "retract"):
                        rapid_mm += sum((a - b) ** 2 for a, b in zip(s, e)) ** 0.5
                    across = abs(s[0] - e[0]) + abs(s[1] - e[1]) > 1e-9
                    if kind == "rapid" and across and s[2] < RETRACT - 1e-9:
                        low += 1
                elif isinstance(u, VerificationUpdate) and u.status not in (None, "clear"):
                    statuses.append((u.completed, u.status))
            self.assertTrue(sim.state.finished)
        return statuses, low, rapid_mm, lowered

    def test_lowered_rapids_are_certified_by_the_stream_check(self):
        statuses, low, rapid_mm, lowered = self.run_rough(1.0)
        self.assertEqual(statuses, [])
        self.assertGreater(low, 0)
        self.assertGreater(lowered, 0)
        plain, plain_low, plain_mm, plain_lowered = self.run_rough(None)
        self.assertEqual(plain, [])
        self.assertEqual((plain_low, plain_lowered), (0, 0))
        self.assertLess(rapid_mm, plain_mm)


if __name__ == "__main__":
    unittest.main()
