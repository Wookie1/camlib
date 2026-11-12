"""engagement_front fine_steering: an opt-in steering grid twice as fine.

The option is off by default and leaves the request unchanged then. When on,
the stream still proves every stroke and its own stock checks stay clear.
"""

import unittest

from camlib.operations import AdaptiveClearing, Roughing
from camlib.simulation import Simulation, StockSpec, VerificationUpdate
from camlib.surface import SurfaceMesh
from camlib.verification import StockContactOptions, StreamChecks

from test_engagement_front_rapid_descent import CUT, TOOL, box


def roughing(**extra):
    return Roughing(
        "rough",
        TOOL,
        SurfaceMesh(box(12, 28, 12, 28, 0, 9), 0.0),
        top_z_mm=10,
        floor_z_mm=1,
        stepdown_mm=4,
        clearance_z_mm=20,
        retract_z_mm=11,
        feed_z_mm=11,
        clearing=AdaptiveClearing(1.0, path_strategy="engagement_front"),
        radial_allowance_mm=0.2,
        bottom_allowance_mm=0.2,
        **CUT,
        **extra,
    )


def run(op):
    statuses = []
    with Simulation(
        stock=StockSpec((0, 0, 0), (40, 40, 10), 0.5),
        tools=(TOOL,),
        operations=(op,),
        revision="1",
        initial_position_mm=(0, 0, 20),
        verification=StreamChecks(stock_contact=StockContactOptions()),
    ) as sim:
        for u in sim.run_operations():
            if isinstance(u, VerificationUpdate) and u.status not in (None, "clear"):
                statuses.append((u.completed, u.status))
        assert sim.state.finished
    return statuses


class FineSteeringTest(unittest.TestCase):
    def test_off_by_default_and_left_out_of_the_request(self):
        self.assertNotIn("fine_steering", roughing().to_dict())
        self.assertIs(roughing(fine_steering=True).to_dict()["fine_steering"], True)

    def test_must_be_a_boolean(self):
        with self.assertRaises(ValueError):
            roughing(fine_steering=1).to_dict()

    def test_fine_run_keeps_its_stock_checks_clear(self):
        self.assertEqual(run(roughing(fine_steering=True)), [])


if __name__ == "__main__":
    unittest.main()
