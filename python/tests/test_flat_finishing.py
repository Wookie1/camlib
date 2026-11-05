"""Flat end mills on 3D surfaces: exact cylinder contact, proven no-gouge
moves, sampled finishing error."""

from dataclasses import replace
import unittest

import camlib
from camlib.simulation import OperationUpdate, Simulation, SimulationError, VerificationUpdate
from test_surface_patterns import roof


def flat_job(**op_values):
    config = roof().parallel_job()
    flat = camlib.ToolAssembly(
        "flat4",
        camlib.EndMill("fe4", diameter=4, flute_length=20, overall_length=50),
        30,
        60,
        (camlib.HolderSection(30, 60, 4, 4),),
    )
    op = replace(config["operations"][0], tool=flat, **op_values)
    return dict(
        config,
        tools=(flat,),
        operations=(op,),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )


class FlatFinishingTest(unittest.TestCase):
    def test_flat_end_mill_never_cuts_below_the_roof(self):
        base = flat_job()
        flat, surface = base["tools"][0], base["operations"][0].surface
        ops = {
            "raster": base["operations"][0],
            "spiral": camlib.Spiral(
                "spiral",
                flat,
                surface,
                0,
                0.5,
                12,
                (12, 8),
                feed_mm_min=600,
                plunge_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
            ),
            "waterline": camlib.Waterline(
                "waterline",
                flat,
                surface,
                0,
                7.5,
                4.5,
                0.5,
                12,
                feed_mm_min=600,
                plunge_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
            ),
        }
        for name, op in ops.items():
            with self.subTest(name):
                report, statuses = None, set()
                with Simulation(**dict(base, operations=(op,))) as sim:
                    for update in sim.run_operations():
                        if isinstance(update, OperationUpdate) and update.stage == "end":
                            report = update.report
                        elif isinstance(update, VerificationUpdate):
                            statuses.add(update.status)
                    self.assertTrue(sim.state.finished)
                    if name != "waterline":
                        self.assertIsNone(report["max_certified_path_excess_mm"])
                        self.assertLessEqual(report["max_sampled_path_excess_mm"], 0.005)
                    self.assertGreater(sim.state.volume()["removed_lower_mm3"], 1)

                    def target(x):
                        return 4 + min(x, 24 - x) / 3

                    for y in range(sim.state.ny):
                        for x in range(sim.state.nx):
                            x0, x1 = x * sim.state.dx, (x + 1) * sim.state.dx
                            i = y * sim.state.nx + x
                            self.assertGreaterEqual(sim.state.heights[2 * i + 1], min(target(x0), target(x1)))
                self.assertNotIn("violation", statuses)

    def test_bull_nose_never_cuts_below_the_roof(self):
        base = flat_job()
        bull = camlib.ToolAssembly(
            "bull4",
            camlib.BullEndMill("bn4", corner_radius=1, diameter=4, flute_length=20, overall_length=50),
            30,
            60,
            (camlib.HolderSection(30, 60, 4, 4),),
        )
        surface = base["operations"][0].surface
        ops = {
            "raster": replace(base["operations"][0], tool=bull),
            "spiral": camlib.Spiral(
                "spiral",
                bull,
                surface,
                0,
                0.5,
                12,
                (12, 8),
                feed_mm_min=600,
                plunge_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
            ),
            "waterline": camlib.Waterline(
                "waterline",
                bull,
                surface,
                0,
                7.5,
                4.5,
                0.5,
                12,
                feed_mm_min=600,
                plunge_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
            ),
        }
        for name, op in ops.items():
            with self.subTest(name):
                report, statuses = None, set()
                with Simulation(**dict(base, tools=(bull,), operations=(op,))) as sim:
                    for update in sim.run_operations():
                        if isinstance(update, OperationUpdate) and update.stage == "end":
                            report = update.report
                        elif isinstance(update, VerificationUpdate):
                            statuses.add(update.status)
                    self.assertTrue(sim.state.finished)
                    if name != "waterline":
                        self.assertLessEqual(report["max_sampled_path_excess_mm"], 0.005)
                    self.assertGreater(sim.state.volume()["removed_lower_mm3"], 1)

                    def target(x):
                        return 4 + min(x, 24 - x) / 3

                    for y in range(sim.state.ny):
                        for x in range(sim.state.nx):
                            x0, x1 = x * sim.state.dx, (x + 1) * sim.state.dx
                            i = y * sim.state.nx + x
                            self.assertGreaterEqual(sim.state.heights[2 * i + 1], min(target(x0), target(x1)))
                self.assertNotIn("violation", statuses)

    def test_ball_only_options_are_refused(self):
        config = flat_job(slope_from_deg=30)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "need a ball end mill"):
                list(sim.run_operations())


if __name__ == "__main__":
    unittest.main()
