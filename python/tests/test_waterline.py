from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import unittest

from camlib import Engine
from camlib.operations import Facing
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError
from camlib.tools import EndMill, ToolAssembly, HolderSection


def example():
    path = Path(__file__).resolve().parents[2] / "python/tests/support/waterline.py"
    spec = importlib.util.spec_from_file_location("waterline_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class WaterlineTest(unittest.TestCase):
    def test_actual_cad_levels_stream_new_material_with_explicit_normal_error(self):
        module = example()
        config = module.waterline_job()
        self.assertEqual(config["operations"][0].surface.source["adapter"], "planar_cad_solid_facets_v1")
        shape = module.target().val()
        self.assertTrue(shape.isValid())
        t = math.tan(math.radians(20))
        self.assertAlmostEqual(shape.Volume(), 768 + 960 - 936 * t + 288 * t * t, places=6)
        report = None
        levels = set()
        first = False
        # Exercise the exact support-polygon method explicitly (default is quad_tree).
        config["operations"] = (replace(config["operations"][0], contour_method="support"),)
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    if update.stage == "end":
                        report = update.report
                else:
                    if update.motion["kind"] == "cut":
                        c = update.motion["curve"]
                        self.assertEqual(c["start"][2], c["end"][2])
                        levels.add(c["end"][2])
                    if not first and sim.state.volume()["removed_lower_mm3"] > 0.001:
                        first = True
                        self.assertIsNone(sim._process.poll())
                        self.assertFalse(sim.state.finished)
            self.assertTrue(first and sim.state.finished)
            self.assertEqual(levels, {8, 6, 4, 2.5})
            self.assertEqual(report["levels_completed"], 4)
            self.assertEqual(report["last_level_z_mm"], 2.5)
            self.assertGreater(report["cut_segments"], 0)
            self.assertEqual(report["contour_method"], "support")
            self.assertTrue(all(r["grid_cells"] == 0 for r in report["level_reports"]))
            self.assertGreater(sum(r["support_queries"] for r in report["level_reports"]), 0)
            self.assertTrue(all(r["max_normal_excess_mm"] <= 0.01 for r in report["level_reports"]))
            self.assertTrue(any(r["removed_vertices"] > 0 for r in report["level_reports"]))
            self.assertEqual(report["geometric_residual_status"], "unknown")
            self.assertIn("not_certified", report["surface_completion"])
            self.assertGreater(sim.state.volume()["removed_lower_mm3"], 100)
            self.assertLessEqual(sim.state.volume()["removed_lower_mm3"], 3840 - shape.Volume())

    def test_two_tool_program_and_partial_waterline_seek_replay(self):
        config = example().waterline_job()
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
                        and c["end"][2] < 8
                        and math.dist(c["start"], c["end"]) > 0.2
                        and update.changed_count
                    ):
                        partial_motion = update.completed
            count = sim.state.completed
        self.assertIsNotNone(partial_motion)
        program = dict(
            version=1,
            id="waterline-program",
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
            player.seek(Cursor(run.status()["available"]))
            self.assertGreater(player.state.volume()["removed_lower_mm3"], volume)
            player.seek(Cursor(partial_motion - 1, 0.5))
            self.assertEqual(player.state.heights.tobytes(), partial)

    def test_a_loop_entered_on_the_wall_is_warned_about_and_a_roughed_blank_is_not(self):
        import warnings

        import cadquery as cq

        from camlib import CamWarning, StockSpec
        from camlib.cadquery import export_stock_mesh

        def run(config):
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                with Simulation(**config) as sim:
                    reports = [
                        u.report for u in sim.run_operations() if isinstance(u, OperationUpdate) and u.stage == "end"
                    ]
                    self.assertTrue(sim.state.finished)
            on_wall = [w for w in caught if issubclass(w.category, CamWarning) and "on the wall" in str(w.message)]
            return reports[0], on_wall

        # Raw stock stands above a level on every side of a loop: its entry
        # stays on the wall, the report counts it and the run warns.
        config = example().waterline_job()
        report, on_wall = run(config)
        self.assertGreater(report["wall_entries"], 0)
        self.assertEqual(len(on_wall), 1)
        self.assertIn(f"entered {report['wall_entries']} loop(s) on the wall", str(on_wall[0].message))
        # A blank roughed to 0.3 mm over the island and 0.1 mm over the base
        # (the benchmark's): every loop has a stock-free point beside it.
        base = cq.Workplane("XY").box(24, 16, 2.1, centered=(False, False, False))
        island = cq.Workplane("XY").workplane(offset=2).center(12, 8).rect(16.7, 10.7).extrude(6.3, taper=20)
        roughed = StockSpec.from_mesh(export_stock_mesh(base.union(island)), 0.4)
        report, on_wall = run(dict(config, stock=roughed))
        self.assertNotIn("wall_entries", report)
        self.assertEqual(on_wall, [])

    def test_empty_above_part_level_and_invalid_offset_base(self):
        config = example().waterline_job()
        op = config["operations"][0]
        config["operations"] = (replace(op, top_z_mm=9, bottom_z_mm=9),)
        with Simulation(**config) as sim:
            reports = [u.report for u in sim.run_operations() if isinstance(u, OperationUpdate) and u.stage == "end"]
            self.assertEqual(sim.state.completed, 0)
            self.assertTrue(sim.state.finished)
            self.assertEqual(reports[0]["empty_levels"], 1)
        config["operations"] = (replace(op, bottom_z_mm=0),)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "waterline parameters"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)

    def test_explicit_reference_method_and_unknown_method_validation(self):
        config = example().waterline_job()
        op = config["operations"][0]
        positional = type(op)(
            op.id,
            op.tool,
            op.surface,
            op.floor_z_mm,
            op.top_z_mm,
            op.bottom_z_mm,
            op.stepdown_mm,
            op.clearance_z_mm,
            0.02,
            # Cutting data is keyword-only and required.
            **{
                k: getattr(op, k)
                for k in ("feed_mm_min", "plunge_mm_min", "spindle_rpm", "tool_number", "length_register", "coolant")
            },
        )
        self.assertEqual(positional.contour_error_mm, 0.02)
        self.assertEqual(positional.contour_method, "quad_tree")
        config["operations"] = (replace(op, contour_method="quad_tree", top_z_mm=9, bottom_z_mm=9),)
        with Simulation(**config) as sim:
            reports = [u.report for u in sim.run_operations() if isinstance(u, OperationUpdate) and u.stage == "end"]
            self.assertEqual(reports[0]["contour_method"], "quad_tree")
            self.assertTrue(sim.state.finished)
            self.assertEqual(sim.state.completed, 0)
        config["operations"] = (replace(op, contour_method="unknown"),)
        with self.assertRaisesRegex((ValueError, SimulationError), "unknown|variant"):
            with Simulation(**config) as sim:
                list(sim.run_operations())
