"""Ramp finishing: waterline levels cut as continuous descents."""

import unittest

import camlib
from camlib.simulation import Simulation, Update, VerificationUpdate
from test_flat_finishing import flat_job
from test_surface_patterns import roof


class RampTest(unittest.TestCase):
    def test_ramp_descends_along_the_contours_without_gouging(self):
        config = roof().parallel_job()
        ball, surface = config["tools"][0], config["operations"][0].surface
        flat = flat_job()["tools"][0]
        for tool in (ball, flat):
            with self.subTest(tool.id):
                op = camlib.Waterline(
                    "ramp",
                    tool,
                    surface,
                    0,
                    7.5,
                    4.5,
                    0.5,
                    12,
                    ramp=True,
                    feed_mm_min=600,
                    plunge_mm_min=150,
                    spindle_rpm=12000,
                    tool_number=1,
                    length_register=1,
                    coolant="flood",
                )
                values = dict(
                    config,
                    tools=(tool,),
                    operations=(op,),
                    verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
                )
                moves, statuses = [], set()
                with Simulation(**values) as sim:
                    for update in sim.run_operations():
                        if isinstance(update, Update):
                            moves.append(update.motion)
                        elif isinstance(update, VerificationUpdate):
                            statuses.add(update.status)
                    self.assertTrue(sim.state.finished)

                    def target(x):
                        return 4 + min(x, 24 - x) / 3

                    for y in range(sim.state.ny):
                        for x in range(sim.state.nx):
                            x0, x1 = x * sim.state.dx, (x + 1) * sim.state.dx
                            i = y * sim.state.nx + x
                            self.assertGreaterEqual(sim.state.heights[2 * i + 1], min(target(x0), target(x1)))
                self.assertNotIn("violation", statuses)
                cuts = [m for m in moves if m["kind"] == "cut"]
                ramps = [m for m in cuts if m["curve"]["start"][2] > m["curve"]["end"][2]]
                self.assertTrue(ramps)
                # Ramps never climb, and the lowest level is finished flat.
                self.assertFalse(any(m["curve"]["end"][2] > m["curve"]["start"][2] for m in cuts))
                self.assertTrue(any(m["curve"]["start"][2] == m["curve"]["end"][2] == 4.5 for m in cuts))


if __name__ == "__main__":
    unittest.main()
