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
            # #46: the 70 degree flanks get depth-1 in-between levels on the
            # full gaps (7, 5); the short last gap (4, 2.5) would gain less
            # than contour_error_mm and gets none; levels_completed and
            # last_level_z below are unchanged.
            self.assertEqual(levels, {8, 7, 6, 5, 4, 2.5})
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

    def test_a_loop_without_a_clear_start_entry_moves_and_nothing_warns_a_roughed_blank_does_not(self):
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
            entries = [w for w in caught if issubclass(w.category, CamWarning) and "loop(s)" in str(w.message)]
            return reports[0], entries

        # Raw stock stands above the level on every side of the loop's given
        # start, but a clear entry exists along the loop: the start moves to
        # it (the report's moved_entries), nothing plunges into the stock and
        # nothing warns (#46).
        config = example().waterline_job()
        report, entries = run(config)
        self.assertGreater(report["moved_entries"], 0)
        self.assertNotIn("wall_entries", report)
        self.assertNotIn("helical_entries", report)
        self.assertEqual(entries, [])
        # A blank roughed to 0.3 mm over the island and 0.1 mm over the base
        # (the benchmark's): every loop enters beside its given start.
        base = cq.Workplane("XY").box(24, 16, 2.1, centered=(False, False, False))
        island = cq.Workplane("XY").workplane(offset=2).center(12, 8).rect(16.7, 10.7).extrude(6.3, taper=20)
        roughed = StockSpec.from_mesh(export_stock_mesh(base.union(island)), 0.4)
        report, entries = run(dict(config, stock=roughed))
        self.assertNotIn("moved_entries", report)
        self.assertNotIn("helical_entries", report)
        self.assertNotIn("wall_entries", report)
        self.assertEqual(entries, [])

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

    def test_ball_job_reports_the_scallop_bound_and_a_flat_cutter_has_neither_key(self):
        # #46: a ball waterline reports the in-between-level scallop bound (the
        # same scallop a vertical wall gets at the stepdown, plus the path
        # tolerance) and the contact angle below which the bound is not
        # claimed; a flat cutter's report has neither key.
        config = example().waterline_job()
        stepdown = config["operations"][0].stepdown_mm
        with Simulation(**config) as sim:
            reports = [u.report for u in sim.run_operations() if isinstance(u, OperationUpdate) and u.stage == "end"]
            self.assertTrue(sim.state.finished)
        # R = 2, stepdown 2: scallop_mm = 2 - sqrt(4 - 1) + contour_error_mm,
        # K = 2; the bound is not claimed below asin(min(1, stepdown/(2^K D)))
        # degrees, D = 2 sqrt(R^2 - (R - scallop_mm)^2) (the formulas of #46).
        e = config["operations"][0].contour_error_mm
        scallop_mm = min(2.0 - math.sqrt(4.0 - stepdown**2 / 4.0) + e, 2.0)
        d = 2.0 * math.sqrt(4.0 - (2.0 - scallop_mm) ** 2)
        k = 0
        while stepdown / 2.0 ** (k + 1) >= scallop_mm:
            k += 1
        self.assertEqual(k, 2)
        self.assertAlmostEqual(reports[0]["scallop_mm"], scallop_mm, places=12)
        self.assertAlmostEqual(
            reports[0]["scallop_from_deg"],
            math.degrees(math.asin(min(1.0, stepdown / (2.0**k * d)))),
            places=12,
        )
        # The island's 70 degree flanks are not vertical, so they get
        # in-between levels (#46).
        self.assertGreater(reports[0]["scallop_levels"], 0)
        # The same job with a flat end mill claims no scallop bound.
        flat_tool = ToolAssembly(
            "flat4",
            EndMill("fe4", diameter=4, flute_length=20, overall_length=50),
            30,
            60,
            (HolderSection(30, 60, 4, 4),),
        )
        flat_config = dict(
            config,
            tools=(flat_tool,),
            operations=(replace(config["operations"][0], tool=flat_tool),),
        )
        with Simulation(**flat_config) as sim:
            reports = [u.report for u in sim.run_operations() if isinstance(u, OperationUpdate) and u.stage == "end"]
            self.assertTrue(sim.state.finished)
        self.assertNotIn("scallop_mm", reports[0])
        self.assertNotIn("scallop_from_deg", reports[0])
