from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import unittest

from camlib import Engine
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError


def example():
    path = Path(__file__).resolve().parents[2] / "python/tests/support/chamfer.py"
    spec = importlib.util.spec_from_file_location("chamfer_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def edge_cells(state):
    """Interior of the bevel, outside the prior roughing tool's swept band."""
    result = []
    for y in range(state.ny):
        for x in range(state.nx):
            r = math.hypot(state.min_mm[0] + (x + 0.5) * state.dx - 20, state.min_mm[1] + (y + 0.5) * state.dy - 20)
            if 7.45 < r < 7.55 or 4.45 < r < 4.55:
                result.append(y * state.nx + x)
    return result


class ChamferTest(unittest.TestCase):
    def test_real_cad_bevel_removes_new_material_and_validates_two_tool_program(self):
        module = example()
        config = module.chamfer_job()
        self.assertEqual(config["operations"][1].region.source["adapter"], "analytic_horizontal_lines_circles_v1")
        events, controls = [], []
        rough_heights = None
        definite_streamed_cut = False
        report = None
        with Simulation(**config) as sim:
            cells = edge_cells(sim.state)
            self.assertTrue(cells)
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    events.extend(update.events)
                    controls.append((update.operation, update.stage))
                    if update.stage == "end" and update.operation == "rough-profile":
                        rough_heights = [(sim.state.heights[2 * i], sim.state.heights[2 * i + 1]) for i in cells]
                        self.assertTrue(all(lo == hi == 10 for lo, hi in rough_heights))
                    if update.stage == "end" and update.operation == "top-bevel":
                        report = update.report
                else:
                    events.append(dict(kind="motion", motion=update.motion))
                    if rough_heights and any(
                        sim.state.heights[2 * i + 1] < lo - 0.01 for i, (lo, _) in zip(cells, rough_heights)
                    ):
                        definite_streamed_cut = True
                        # Streamed: seen before the run's end (not process timing).
                        self.assertFalse(sim.state.finished)
            self.assertTrue(definite_streamed_cut and sim.state.finished)
            self.assertTrue(all(9.2 < sim.state.heights[2 * i] <= sim.state.heights[2 * i + 1] < 9.8 for i in cells))
            self.assertEqual(
                controls,
                [("rough-profile", "begin"), ("rough-profile", "end"), ("top-bevel", "begin"), ("top-bevel", "end")],
            )
            self.assertEqual(report["geometric_residual_status"], "unknown")
            self.assertIn("not_certified", report["edge_completion"])
            self.assertAlmostEqual(report["top_target_area_mm2"], 24 * math.pi)
            self.assertEqual(report["requested_width_mm"], 1)
            self.assertAlmostEqual(report["axial_bevel_depth_mm"], 1)
            self.assertAlmostEqual(report["tool_tip_target_z_mm"], 8.5)
            self.assertEqual(report["path_safety_margin_mm"], 0.005)
            self.assertTrue(report["entry_requires_cleared_tip"])
            target = module.cadquery_target().val()
            self.assertTrue(target.isValid())
            ideal = 16000 - target.Volume()
            self.assertAlmostEqual(ideal, 780 * math.pi, places=6)
            volume = sim.state.volume()
            self.assertLessEqual(volume["removed_lower_mm3"], ideal)
            self.assertGreaterEqual(volume["removed_upper_mm3"], ideal)
            count = sim.state.completed
        program = dict(
            version=1,
            id="chamfer-program",
            revision="1",
            units="mm",
            tolerance_mm=1e-5,
            frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
            initial_position_mm=(0, 0, 12),
            tools=[t.to_dict() for t in config["tools"]],
            events=events,
        )
        with Engine() as engine:
            self.assertEqual(
                engine._request("validate_program", program=program, capabilities=None)["summary"]["motions"], count
            )

    def test_partial_new_chamfer_arc_seek_replays_exact_stock(self):
        with RecordedRun(**{'rapid_mm_min': 6000, **example().chamfer_job()}, checkpoint_every=8) as run:
            player = Player(run)
            player.pause()
            run.wait()
            controls = list(run.operation_controls())
            start = controls[2]["completed"]
            player.seek(Cursor(start))
            before = player.state.heights.tobytes()
            player.seek(Cursor(start + 3, 0.5))
            partial = player.state.heights.tobytes()
            self.assertNotEqual(before, partial)
            player.seek(Cursor(run.status()["available"]))
            self.assertTrue(player.state.finished)
            player.seek(Cursor(start + 3, 0.5))
            self.assertEqual(player.state.heights.tobytes(), partial)

    def test_noncenter_cutting_tip_requires_cleared_entry_before_first_motion(self):
        config = example().chamfer_job()
        config["operations"] = (config["operations"][1],)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "cleared tip"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)
        config["operations"] = (replace(config["operations"][0], width_mm=4),)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "cone|conical"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)
