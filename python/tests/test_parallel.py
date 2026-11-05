from dataclasses import replace
import importlib.util
import itertools
import os
from pathlib import Path
import unittest
from unittest import mock

from camlib import Engine
from camlib.cadquery import export_surface_mesh
from camlib.operations import Facing
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError
from camlib.surface import SurfaceMesh
from camlib.tools import EndMill, ToolAssembly, HolderSection


def example():
    path = Path(__file__).resolve().parents[2] / "python/tests/support/parallel.py"
    spec = importlib.util.spec_from_file_location("parallel_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ParallelTest(unittest.TestCase):
    def test_actual_cad_surface_streams_three_axis_removal_and_preserves_roof(self):
        module = example()
        config = module.parallel_job()
        op = config["operations"][0]
        self.assertEqual(op.surface.source["adapter"], "planar_cad_solid_facets_v1")
        self.assertAlmostEqual(module.target().val().Volume(), 2304)
        first = False
        z_moves = 0
        report = None
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    if update.stage == "end":
                        report = update.report
                else:
                    curve = update.motion["curve"]
                    if update.motion["kind"] == "cut":
                        z_moves += abs(curve["start"][2] - curve["end"][2]) > 0.001
                    if not first and sim.state.volume()["removed_lower_mm3"] > 0.001:
                        first = True
                        self.assertFalse(sim.state.finished)  # streamed: before the run's end
                        self.assertFalse(sim.state.finished)
            self.assertTrue(first and sim.state.finished)
            self.assertGreater(z_moves, 10)
            self.assertEqual(report["rows_completed"], report["rows"])
            self.assertLessEqual(report["max_certified_path_excess_mm"], 0.005)
            self.assertGreater(report["work"]["distance_tests"], 0)
            self.assertEqual(report["geometric_residual_status"], "unknown")
            self.assertIn("not_certified", report["surface_completion"])
            for y in range(sim.state.ny):
                for x in range(sim.state.nx):
                    x0 = x * sim.state.dx
                    x1 = (x + 1) * sim.state.dx

                    def target(x):
                        return 4 + min(x, 24 - x) / 3

                    low = min(target(x0), target(x1))
                    i = y * sim.state.nx + x
                    # A conservative stock interval can straddle the target.
                    # Native tests separately prove whole-segment roof safety
                    # and compare every cell with an exact swept-ball oracle.
                    self.assertGreaterEqual(sim.state.heights[2 * i + 1], low)
                    high = 8 if x0 <= 12 <= x1 else max(target(x0), target(x1))
                    self.assertLessEqual(sim.state.heights[2 * i + 1], high + 0.15)
            volume = sim.state.volume()
            self.assertGreater(volume["removed_lower_mm3"], 1400)
            self.assertLessEqual(volume["removed_lower_mm3"], 1536)

    def test_two_tool_job_and_partial_sloped_segment_seek_replay(self):
        config = example().parallel_job()
        face_tool = ToolAssembly(
            "face8",
            EndMill("em8", diameter=8, flute_length=20, overall_length=50),
            30,
            60,
            (HolderSection(30, 60, 5, 5),),
        )
        face = Facing(
            "face",
            face_tool,
            9,
            1,
            6,
            12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        op = replace(config["operations"][0], tool_number=2, length_register=2)
        config.update(tools=(face_tool, *config["tools"]), operations=(face, op))
        events = []
        partial_motion = None
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    events.extend(update.events)
                else:
                    events.append(dict(kind="motion", motion=update.motion))
                    c = update.motion["curve"]
                    if (
                        partial_motion is None
                        and update.motion["kind"] == "cut"
                        and abs(c["end"][2] - c["start"][2]) > 0.001
                        and all(4 <= p[0] <= 8 and 0 <= p[1] <= 16 for p in (c["start"], c["end"]))
                        and update.changed_count
                    ):
                        partial_motion = update.completed
            count = sim.state.completed
        program = dict(
            version=1,
            id="surface-program",
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
        self.assertIsNotNone(partial_motion)
        with RecordedRun(**{'rapid_mm_min': 6000, **config}, checkpoint_every=16) as run:
            player = Player(run)
            player.pause()
            run.wait()
            player.seek(Cursor(partial_motion - 1))
            before = player.state.heights.tobytes()
            player.seek(Cursor(partial_motion - 1, 0.5))
            partial = player.state.heights.tobytes()
            self.assertNotEqual(before, partial)
            volume = player.state.volume()["removed_lower_mm3"]
            self.assertGreater(volume, 0.001)
            player.seek(Cursor(run.status()["available"]))
            self.assertGreater(player.state.volume()["removed_lower_mm3"], volume)
            player.seek(Cursor(partial_motion - 1, 0.5))
            self.assertEqual(player.state.heights.tobytes(), partial)

    def test_cad_approximation_bounded_input_and_invalid_before_motion(self):
        import cadquery as cq

        with self.assertRaisesRegex(ValueError, "max_facets"):
            export_surface_mesh(cq.Workplane("XY").sphere(4), max_facets=4)
        # No count cap: an endless mesh is refused by the memory budget.
        with mock.patch.dict(os.environ, {"CAMLIB_MEMORY_BUDGET_BYTES": str(64 << 20)}):
            with self.assertRaisesRegex(ValueError, "surface mesh triangles needs"):
                SurfaceMesh(itertools.repeat(((0, 0, 0), (1, 0, 0), (0, 1, 0))))
        with self.assertRaisesRegex(ValueError, "three coordinates"):
            SurfaceMesh(((itertools.repeat(1), (1, 0, 0), (0, 1, 0)),))
        config = example().parallel_job()
        config["operations"] = (replace(config["operations"][0], safety_margin_mm=0),)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "parallel finishing"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)
