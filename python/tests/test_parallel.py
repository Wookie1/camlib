from dataclasses import replace
import importlib.util
import itertools
import math
import os
from pathlib import Path
import unittest
from unittest import mock

import camlib
from camlib import Engine
from camlib.cadquery import export_surface_mesh
from camlib.operations import Facing
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, ReachRefused, Simulation, SimulationError
from camlib.surface import SurfaceMesh
from camlib.tools import BallEndMill, EndMill, ToolAssembly, HolderSection
from camlib.simulation import Update


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


class ActualDepthTest(unittest.TestCase):
    """A part taller than the flutes (issue #88): parallel passes are refused
    only where they really cut deeper than the cutting length."""

    def setUp(self):
        import cadquery as cq

        from camlib.cadquery import export_surface_mesh

        # A 40 x 40 x 4 plate with a 10 x 10 mesa 16 mm tall (top Z 20).
        part = cq.Workplane("XY").box(40, 40, 4, centered=False)
        part = part.union(cq.Workplane("XY").box(10, 10, 20, centered=False).translate((15, 15, 0)))
        self.mesh = export_surface_mesh(part.val())
        holder = (camlib.HolderSection(40, 80, 10, 10),)
        self.mill = camlib.ToolAssembly(
            "em8", camlib.EndMill("em8", diameter=8, flute_length=25, overall_length=60), 40, 80, holder
        )
        self.ball = camlib.ToolAssembly(
            "b6", camlib.BallEndMill("b6", diameter=6, flute_length=18, overall_length=60), 40, 80, holder
        )
        self.cut = dict(feed_mm_min=1200, plunge_mm_min=300, spindle_rpm=9000, coolant="flood")

    def run_ops(self, operations):
        config = dict(
            stock=camlib.StockSpec((-1, -1, 0), (41, 41, 20.5), 0.5),
            tools=(self.mill, self.ball),
            operations=operations,
            revision="1",
            initial_position_mm=(0, 0, 40),
        )
        moves = 0
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                moves += hasattr(update, "motion")
        return moves

    def finish(self):
        return camlib.Parallel(
            "finish",
            self.ball,
            self.mesh,
            0,
            3,
            30,
            tool_number=2,
            length_register=2,
            **self.cut,
        )

    def test_passes_after_roughing_reach_only_what_the_flutes_cut(self):
        rough = camlib.Roughing(
            "rough",
            self.mill,
            self.mesh,
            top_z_mm=20.5,
            floor_z_mm=4,
            stepdown_mm=4,
            clearance_z_mm=30,
            clearing=camlib.OffsetClearing(3),
            radial_allowance_mm=0.3,
            bottom_allowance_mm=0.2,
            tool_number=1,
            length_register=1,
            **self.cut,
        )
        # The stock top stays 20.5 over the mesa (20.5 mm above the floor,
        # past the 18 mm flutes), but no pass cuts more than the flutes.
        self.assertGreater(self.run_ops((rough, self.finish())), 100)

    def test_a_pass_really_deeper_than_the_flutes_is_refused(self):
        with self.assertRaisesRegex(ReachRefused, "cuts deeper than the cutting length 18") as caught:
            self.run_ops((self.finish(),))
        # Typed (#161): the shank, 18 mm above the tip, meets the unroughed
        # stock; the obstruction is the stock top, the tip the pass's.
        r = caught.exception.refusal
        self.assertEqual((r.operation, r.tool, r.limit, r.holder_section), ("finish", "b6", "shank", None))
        self.assertEqual((r.top_z_mm, r.depth_available_mm), (20.5, 18))
        self.assertAlmostEqual(r.depth_needed_mm, r.top_z_mm - r.z_mm)
        self.assertLess(r.z_mm, 20.5 - 18)


class RowEntryTest(unittest.TestCase):
    def run_rows(self, cell):
        import cadquery as cq
        import camlib
        from camlib.cadquery import export_surface_mesh

        part = (
            cq.Workplane("XY")
            .box(24, 16, 4, centered=False)
            .union(cq.Workplane("XY").box(8, 8, 3, centered=False).translate((8, 4, 4)))
        )
        surface = export_surface_mesh(part, source=dict(model="stepped", revision="1"))
        tool = ToolAssembly(
            "ball4",
            BallEndMill("b4", diameter=4, flute_length=20, overall_length=50),
            30,
            60,
            (HolderSection(30, 60, 4, 4),),
        )
        op = camlib.Parallel(
            "rows",
            tool,
            surface,
            0,
            2,
            12,
            slope_to_deg=45,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        motions = []
        with Simulation(
            stock=camlib.StockSpec((0, 0, 0), (24, 16, 8), cell),
            tools=(tool,),
            operations=(op,),
            revision="1",
            initial_position_mm=(0, 0, 12),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, Update):
                    motions.append(update.motion)
        return motions

    def wedge_rows(self, plunge_unproven_entries):
        # A 40 deg slope rising from the foot of a vertical wall, under fresh
        # stock to Z12: rows start at the wall foot, the wall blocks a ramp
        # backward and the slope a ramp forward, so no entry ramp is clear.
        import cadquery as cq
        import camlib
        from camlib.cadquery import export_surface_mesh

        profile = [(0, 0), (30, 0), (30, 10.39), (16, 10.39), (6, 2), (6, 12), (0, 12)]
        part = cq.Workplane("XZ").polyline(profile).close().extrude(-16)
        surface = export_surface_mesh(part, source=dict(model="wedge", revision="1"))
        tool = ToolAssembly(
            "ball4",
            BallEndMill("b4", diameter=4, flute_length=20, overall_length=50),
            30,
            60,
            (HolderSection(30, 60, 4, 4),),
        )
        op = camlib.Parallel(
            "rows",
            tool,
            surface,
            0,
            2,
            16,
            slope_to_deg=45,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
            plunge_unproven_entries=plunge_unproven_entries,
        )
        report, motions = None, []
        with Simulation(
            stock=camlib.StockSpec((0, 0, 0), (30, 16, 12), 0.5),
            tools=(tool,),
            operations=(op,),
            revision="1",
            initial_position_mm=(0, 0, 16),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, Update):
                    motions.append(update.motion)
                elif isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
        return report, motions

    def test_an_entry_with_no_proven_ramp_is_refused_by_default(self):
        with self.assertRaisesRegex(Exception, "no ramp along the row is proven clear"):
            self.wedge_rows(False)

    def test_opted_in_unproven_entries_plunge_and_are_all_listed(self):
        report, motions = self.wedge_rows(True)
        listed = report["unproven_entries"]
        self.assertTrue(listed)
        # Independent check: before the first cut the stock is the box to
        # Z12, so every plunge whose ball tip ends below Z12 - 0.25 goes
        # into fresh stock; each one at an entry must be listed, with a
        # bound no smaller than the plunge's real depth into that stock.
        keyed = {(round(e["x_mm"], 6), round(e["y_mm"], 6), round(e["z_mm"], 6)): e for e in listed}
        deep = []
        for m in motions:
            if m["kind"] != "plunge":
                continue
            x, y, z = m["curve"]["end"]
            if 12 - z > 0.25 and (round(x, 6), round(y, 6), round(z, 6)) in keyed:
                deep.append(keyed[(round(x, 6), round(y, 6), round(z, 6))])
                self.assertGreaterEqual(keyed[(round(x, 6), round(y, 6), round(z, 6))]["bound_mm"], 12 - z - 1e-9)
        self.assertEqual(len(deep), len(listed))

    def test_coarse_stock_cells_never_hide_stock_from_a_row_entry(self):
        # Audit #22: on 2-8 mm stock cells the entry test used to read only
        # cell centres, and the first entry plunged 4 mm into fresh stock at
        # the stock's edge. Independent bound: before any cut the stock is
        # the 0..24 x 0..16 box up to Z8, so a ball plunge to tip z at (x, y)
        # cuts 8 - (z + 2 - sqrt(4 - d^2)) at the box point nearest the axis
        # (d its distance). The first entry may cut at most 0.25 mm of it, or
        # the operation refuses.
        for cell in (2, 4, 8):
            try:
                motions = self.run_rows(cell)
            except Exception as e:  # refused: no ramp proven clear
                self.assertIn("no ramp along the row is proven clear", str(e))
                continue
            first = next(m for m in motions if m["kind"] == "plunge")
            x, y, z = first["curve"]["end"]
            d = math.hypot(max(0, x - 24, -x), max(0, y - 16, -y))
            if d < 2:
                cut = 8 - (z + 2 - math.sqrt(4 - d * d))
                self.assertLessEqual(cut, 0.25 + 1e-9, (cell, first))

    def test_rows_under_stock_are_entered_by_a_ramp_not_a_plunge(self):
        # Fresh 8 mm stock stands over a stepped part (flats at 4 and 7).
        # Rows limited to slopes up to 45 deg skip the boss walls and
        # re-enter beside and on the boss under that stock: a straight
        # plunge would take the ball down through it each time.
        import cadquery as cq
        import camlib
        from camlib.cadquery import export_surface_mesh

        part = (
            cq.Workplane("XY")
            .box(24, 16, 4, centered=False)
            .union(cq.Workplane("XY").box(8, 8, 3, centered=False).translate((8, 4, 4)))
        )
        surface = export_surface_mesh(part, source=dict(model="stepped", revision="1"))
        tool = ToolAssembly(
            "ball4",
            BallEndMill("b4", diameter=4, flute_length=20, overall_length=50),
            30,
            60,
            (HolderSection(30, 60, 4, 4),),
        )
        op = camlib.Parallel(
            "rows",
            tool,
            surface,
            0,
            2,
            12,
            slope_to_deg=45,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        report, motions = None, []
        with Simulation(
            stock=camlib.StockSpec((0, 0, 0), (24, 16, 8), 0.25),
            tools=(tool,),
            operations=(op,),
            revision="1",
            initial_position_mm=(0, 0, 12),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, Update):
                    motions.append(update.motion)
                elif isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
        self.assertGreater(report["ramped_entries"], 0)
        # Every plunge stops above the stock it would otherwise go through
        # (the boss top at 7 and the floor at 4 lie under 8 mm stock).
        for m in motions:
            if m["kind"] == "plunge":
                self.assertGreater(m["curve"]["end"][2], 7.5, m)
        # Entry ramps: feed moves that descend while going back and forth
        # along the row, never steeper than 10 deg.
        slope = math.tan(math.radians(10)) + 1e-9
        ramps = 0
        for prev, m in zip(motions, motions[1:]):
            a, b = m["curve"]["start"], m["curve"]["end"]
            run = math.hypot(b[0] - a[0], b[1] - a[1])
            if prev["kind"] == "plunge" and m["kind"] == "lead" and b[2] < a[2]:
                ramps += 1
                self.assertLessEqual((a[2] - b[2]) / run, slope)
        self.assertEqual(ramps, report["ramped_entries"])
