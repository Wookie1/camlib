import importlib.util
import math
from pathlib import Path
import unittest

from camlib import Engine
from camlib.operations import Pocket, TrochoidalSlot
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError, StockSpec
from test_simulation import tool

ROOT = Path(__file__).resolve().parents[2]


def slot():
    return TrochoidalSlot(
        "slot",
        tool(),
        (8, 12),
        (14, 12),
        6,
        10,
        8,
        1,
        12,
        feed_mm_min=600,
        entry_feed_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )


class TrochoidalSlotTest(unittest.TestCase):
    def test_python_native_slot_streams_actual_stock_and_a_valid_complete_program(self):
        op = slot()
        config = dict(
            stock=StockSpec((0, 0, 0), (24, 24, 10), 0.25),
            tools=(tool(),),
            operations=(op,),
            revision="python-slot",
            initial_position_mm=(0, 0, 12),
        )
        events, reports, first_stock = [], [], False
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    reports.append(update.report)
                    events.extend(update.events)
                else:
                    events.append(dict(kind="motion", motion=update.motion))
                    if update.changed_count and not first_stock:
                        first_stock = True
                        self.assertIsNone(sim._process.poll())
                        self.assertFalse(sim.state.finished)
            self.assertTrue(first_stock and sim.state.finished)
            self.assertEqual(reports[-1]["seed_orbits"], 2)
            self.assertIn("NOT limited", reports[-1]["seed_entry_policy"])
            self.assertLessEqual(reports[-1]["max_bounded_engagement_deg"], 90)
            ideal = (6 * 6 + 9 * math.pi) * 2
            volume = sim.state.volume()
            self.assertLessEqual(volume["removed_lower_mm3"], ideal)
            self.assertGreaterEqual(volume["removed_upper_mm3"], ideal - 0.5)
            self.assertGreater(volume["removed_lower_mm3"], ideal * 0.8)
            count = sim.state.completed
        program = dict(
            version=1,
            id="slot-program",
            revision="1",
            units="mm",
            tolerance_mm=1e-5,
            frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
            initial_position_mm=(0, 0, 12),
            tools=[tool().to_dict()],
            events=events,
        )
        with Engine() as engine:
            self.assertEqual(
                engine._request("validate_program", program=program, capabilities=None)["summary"]["motions"], count
            )

    def test_mixed_pocket_and_slot_retained_run_replays_actual_material(self):
        region = PlanarRegion("pocket", (Component(Boundary.polygon(((3, 3), (11, 3), (11, 11), (3, 11)))),))
        rough = Pocket(
            "rough",
            tool(),
            region,
            10,
            9,
            1,
            1.5,
            12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        with RecordedRun(
            StockSpec((0, 0, 0), (24, 24, 10), 0.25),
            (tool(),),
            operations=(rough, slot()),
            revision="mixed-slot",
            initial_position_mm=(0, 0, 12),
            checkpoint_every=32,
            rapid_mm_min=6000,
        ) as run:
            player = Player(run)
            player.pause()
            run.wait()
            controls = list(run.operation_controls())
            self.assertEqual(
                [(c["operation"], c["stage"]) for c in controls],
                [("rough", "begin"), ("rough", "end"), ("slot", "begin"), ("slot", "end")],
            )
            first_slot = controls[2]["completed"] + 1
            player.seek(Cursor(first_slot, 0.5))
            partial = player.state.heights.tobytes()
            player.seek(Cursor(run.status()["available"]))
            self.assertTrue(player.state.finished)
            self.assertGreater(player.state.volume()["removed_lower_mm3"], 100)
            player.seek(Cursor(first_slot, 0.5))
            self.assertEqual(player.state.heights.tobytes(), partial)

    def test_matching_cadquery_target_has_independent_capsule_volume(self):
        path = ROOT / "python/tests/support/trochoidal_slot.py"
        spec = importlib.util.spec_from_file_location("slot_example", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        target = module.cadquery_target().val()
        self.assertTrue(target.isValid())
        self.assertAlmostEqual(target.Volume(), 40 * 24 * 10 - (20 * 6 + 9 * math.pi) * 2, places=6)
        self.assertEqual(module.slot_job()["operations"][0].boundary_tolerance_mm, 0.01)

    def test_invalid_slot_fails_before_any_motion(self):
        from dataclasses import replace

        with Simulation(
            StockSpec((0, 0, 0), (24, 24, 10), 0.25),
            (tool(),),
            operations=(replace(slot(), width_mm=4),),
            revision="bad-slot",
            initial_position_mm=(0, 0, 12),
        ) as sim:
            with self.assertRaisesRegex(SimulationError, "trochoidal slot"):
                list(sim.run_operations())
            self.assertFalse(sim.state.finished)
            self.assertEqual(sim.state.completed, 0)


S_PATH = ((6, 12, 1), (20, 12, -1), (34, 12, 0))


def s_slot(**overrides):
    values = dict(
        feed_mm_min=600,
        entry_feed_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
        path_mm=S_PATH,
    )
    values.update(overrides)
    return TrochoidalSlot("s", tool(), None, None, 6, 10, 9, 1, 12, **values)


class CurvedSlotTest(unittest.TestCase):
    """#9: slots along a centreline of lines and arcs."""

    def test_s_slot_plans_within_the_engagement_limit_and_posts(self):
        import camlib
        from camlib.post import LoweringOptions, Machine, ToolSetting
        from camlib.postprocessor import GCodeDialect

        op = s_slot(max_engagement_deg=75)
        self.assertEqual(op.to_dict()["path_mm"], [list(map(float, v)) for v in S_PATH])
        self.assertNotIn("start_mm", op.to_dict())
        setup = camlib.Setup("top", StockSpec((0, 0, 0), (40, 24, 10), 0.25), (tool(),), (op,), (0, 0, 12))
        job = camlib.Job("s-slot", "1", (setup,))
        report = None
        with job.regenerate() as run:
            for event in run:
                if isinstance(event.update, OperationUpdate) and event.update.stage == "end":
                    report = event.update.report
        self.assertEqual(report["path_pieces"], 2)
        self.assertEqual(report["corner_links"], 0)
        self.assertLessEqual(report["max_bounded_engagement_deg"], 75)
        machine = Machine(
            "mill",
            (ToolSetting.for_tool(tool(), number=1, length_register=1, position_after_mm=(0, 0, 12)),),
            dialect=GCodeDialect.linuxcnc(),
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=("xy", "xz", "yz"),
            arcs=True,
            helices=True,
            max_sweep_rad=math.tau,
            max_tool_number=99,
            max_length_register=99,
            tool_changes=True,
            flood=True,
            mist=False,
            tool_change_position_mode="preserve_tip",
            rapid_mm_min=6000,
        )
        text = job.post_nc(
            "top", machine=machine, lowering=LoweringOptions(native_arcs=True), accept={"unchecked_machine"}
        ).text
        # Orbits and the links along the arcs are circular blocks.
        self.assertGreater(sum(line.startswith(("G2", "G3")) for line in text.splitlines()), 10)

    def test_path_or_points_not_both(self):
        with self.assertRaisesRegex(ValueError, "either start_mm and end_mm"):
            TrochoidalSlot(
                "s",
                tool(),
                (8, 12),
                (14, 12),
                6,
                10,
                9,
                1,
                12,
                feed_mm_min=600,
                entry_feed_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
                path_mm=S_PATH,
            )
        with self.assertRaisesRegex(ValueError, "x, y, bulge_to_next"):
            s_slot(path_mm=((6, 12), (20, 12)))

    def test_too_tight_bend_is_refused_before_any_motion(self):
        op = s_slot(path_mm=((10, 12, 1), (11.8, 12, 0)))
        config = dict(
            stock=StockSpec((0, 0, 0), (40, 24, 10), 0.25),
            tools=(tool(),),
            operations=(op,),
            revision="tight",
            initial_position_mm=(0, 0, 12),
        )
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "bends toward the orbit side"):
                list(sim.run_operations())


class TrochoidalSlotAirFeedTest(unittest.TestCase):
    """#34 part 1: air feed on the proven-air part of each controlled orbit."""

    def posted_air_job(self, decimals):
        from dataclasses import replace

        from camlib import Job, OperationNode, Setup, StockContactOptions, StreamChecks
        from camlib.material import Options, Target
        from camlib.post import ToolSetting
        from camlib.verification import MaterialCheck
        from test_post import fixture

        ux, uy = math.cos(math.pi / 6), math.sin(math.pi / 6)
        op = replace(
            slot(),
            air_feed_mm_min=2400.0,
            floor_z_mm=9.0,
            width_mm=4.5,
            start_mm=(8.003, 12.004),
            end_mm=(8.003 + 6 * ux, 12.004 + 6 * uy),
        )
        radius = op.width_mm / 2
        vertices = tuple(
            (p[0] - side * radius * uy, p[1] + side * radius * ux, bulge)
            for p, side, bulge in ((op.start_mm, -1, 0), (op.end_mm, -1, 1), (op.end_mm, 1, 0), (op.start_mm, 1, 1))
        )
        region = PlanarRegion('slot', (Component(Boundary(vertices)),))
        checks = StreamChecks(
            stock_contact=StockContactOptions(),
            material=(MaterialCheck(Target('slot', '1', region, 9), options=Options(gouge_tolerance_mm=0.05)),),
        )
        job = Job(
            'air-slot',
            '1',
            (
                Setup(
                    'top',
                    stock=StockSpec((0, 0, 0), (24, 24, 10), 0.25),
                    tools=(tool(),),
                    operations=(OperationNode(op, sources=()),),
                    initial_position_mm=(0, 0, 12),
                    verification=checks,
                ),
            ),
        )
        _, base = fixture()
        machine = replace(
            base,
            decimals=decimals,
            tolerance_mm=0.05,
            tools=(ToolSetting.for_tool(tool(), number=1, length_register=1, position_after_mm=(0, 0, 12)),),
        )
        return job, machine

    def test_rounded_air_sweep_cannot_be_exported(self):
        from tempfile import TemporaryDirectory
        from camlib.post import LoweringOptions

        job, machine = self.posted_air_job(2)
        # With ordinary Cut intent the two-decimal fast arc passes all gates,
        # despite reaching about .01 mm beyond the preceding cutter sweeps.
        with TemporaryDirectory() as directory:
            output = Path(directory) / 'slot.nc'
            with job.post(
                'top', machine=machine, lowering=LoweringOptions(native_arcs=True, chord_tolerance_mm=0.0005)
            ) as posted:
                with self.assertRaisesRegex(SimulationError, 'air motion .*not proven'):
                    posted.export(output, accept={'unchecked_machine'})
            self.assertFalse(output.exists())

    def test_precise_air_sweep_keeps_intent_through_checked_export(self):
        from camlib.post import LoweringOptions

        job, machine = self.posted_air_job(6)
        with job.post(
            'top', machine=machine, lowering=LoweringOptions(native_arcs=True, chord_tolerance_mm=0.0005)
        ) as posted:
            result = posted.export(accept={'unchecked_machine'})
            self.assertFalse(result['findings'])
            self.assertIn('F2400', result['text'])
            motions = [posted.posted_motion(i)['motion'] for i in range(result['motions'])]
            air = [m for m in motions if m['kind'] == 'air']
            self.assertTrue(air)
            self.assertTrue(all(m['feed_mm_min'] == 2400.0 for m in air))

    def test_air_feed_mm_min_round_trips_through_to_dict(self):
        from dataclasses import replace

        op = slot()
        self.assertNotIn("air_feed_mm_min", op.to_dict())
        fed = replace(op, air_feed_mm_min=2400.0)
        self.assertEqual(fed.to_dict()["air_feed_mm_min"], 2400.0)

    def test_air_feed_runs_and_report_counts_air_arcs(self):
        from dataclasses import replace

        op = replace(slot(), air_feed_mm_min=2400.0)
        reports = []
        air_arcs = 0
        with Simulation(
            StockSpec((0, 0, 0), (24, 24, 10), 0.25),
            (tool(),),
            operations=(op,),
            revision="air-slot",
            initial_position_mm=(0, 0, 12),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    reports.append(update.report)
                elif update.motion["kind"] == "air" and update.motion["curve"]["kind"] == "arc":
                    air_arcs += 1
        self.assertTrue(sim.state.finished)
        self.assertGreater(air_arcs, 0)
        self.assertEqual(reports[-1]["air_fed_arcs"], air_arcs)
        self.assertGreater(reports[-1]["air_fed_mm"], 0)

    def test_d_shaped_loops_report_return_chords_without_air_feed(self):
        reports = []
        with Simulation(
            StockSpec((0, 0, 0), (24, 24, 10), 0.25),
            (tool(),),
            operations=(slot(),),
            revision="d-slot",
            initial_position_mm=(0, 0, 12),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    reports.append(update.report)
        self.assertTrue(sim.state.finished)
        report = reports[-1]
        self.assertGreater(report["return_chords"], 0)
        self.assertGreater(report["return_chord_saved_mm"], 0)
        self.assertNotIn("air_fed_arcs", report)


class SlotWallFinishTest(unittest.TestCase):
    """#34 part 4: the slot's walls are finished by a separate Contour, as the
    docs recommend: the slot is cut narrower by an allowance, then a Contour
    around the slot's outline, entered on a ramp, cuts that allowance."""

    WIDTH, ALLOWANCE, CELL = 6.0, 0.2, 0.05

    def run_recipe(self, finish):
        from dataclasses import replace

        import numpy as np
        from camlib.operations import Contour, RampEntry

        rough = replace(slot(), width_mm=self.WIDTH - 2 * self.ALLOWANCE)
        (x0, y0), (x1, y1) = rough.start_mm, rough.end_mm
        h = self.WIDTH / 2
        outline = Boundary(((x0, y0 - h, 0), (x1, y1 - h, 1), (x1, y1 + h, 0), (x0, y0 + h, 1)))
        square = Boundary.polygon(((0, 0), (24, 0), (24, 24), (0, 24)))
        part = PlanarRegion("part", (Component(square, (outline,)),))
        walls = Contour(
            "walls",
            tool(),
            part,
            rough.floor_z_mm,
            2,
            12,
            boundaries="holes",
            entry=RampEntry(3),
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        operations = (rough, walls) if finish else (rough,)
        motions, reports, current = {}, {}, None
        with Simulation(
            StockSpec((0, 0, 0), (24, 24, 10), self.CELL),
            (tool(),),
            operations=operations,
            revision="walls",
            initial_position_mm=(0, 0, 12),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    current = update.operation
                    if update.stage == "end":
                        reports[current] = update.report
                else:
                    motions.setdefault(current, []).append(update.motion)
            self.assertTrue(sim.state.finished)
            nx, ny = sim.state.nx, sim.state.ny
            cells = np.asarray(sim.state.heights).reshape(ny, nx, 2)
        # Distance of each cell centre from the slot's centreline.
        xs, ys = np.meshgrid((np.arange(nx) + 0.5) * self.CELL, (np.arange(ny) + 0.5) * self.CELL)
        ax, ay = x1 - x0, y1 - y0
        t = np.clip(((xs - x0) * ax + (ys - y0) * ay) / (ax * ax + ay * ay), 0, 1)
        d = np.hypot(xs - (x0 + t * ax), ys - (y0 + t * ay))
        return d, cells[..., 0], cells[..., 1], motions, reports, rough, walls

    def test_contour_after_a_narrower_slot_finishes_both_walls_without_plunging(self):
        d, low, high, motions, reports, rough, walls = self.run_recipe(True)
        h = self.WIDTH / 2
        # A cell wholly inside the slot, short of the wall by the Contour's
        # boundary tolerance, holds no material above the floor; a cell
        # wholly outside it was not cut at all.
        half = self.CELL * math.sqrt(2) / 2
        inside = d <= h - half - walls.boundary_tolerance_mm
        outside = d >= h + half
        self.assertGreater(inside.sum(), 20000)
        self.assertLessEqual(high[inside].max(), rough.floor_z_mm + 1e-4)
        self.assertGreaterEqual(low[inside].min(), rough.floor_z_mm - 1e-4)
        self.assertGreaterEqual(low[outside].min(), 10 - 1e-6)
        self.assertEqual(reports["walls"]["loops_completed"], 1)
        # The Contour never moves straight down: every descent is a ramp no
        # steeper than its 3 degrees.
        descents = 0
        for motion in motions["walls"]:
            curve = motion["curve"]
            drop = curve["start"][2] - curve["end"][2]
            if drop <= 1e-9 or motion["kind"] == "rapid":
                continue
            a, b = curve["start"], curve["end"]
            if curve["kind"] == "line":
                run = math.hypot(b[0] - a[0], b[1] - a[1])
            else:
                c = curve["center"]
                run = math.hypot(a[0] - c[0], a[1] - c[1]) * abs(curve["sweep_rad"])
            if b[2] < 10:
                descents += 1
                self.assertLessEqual(drop, run * math.tan(math.radians(3)) * (1 + 1e-9), motion)
        self.assertGreater(descents, 0)

    def test_the_narrower_slot_alone_leaves_the_wall_allowance(self):
        d, _, high, _, _, rough, _ = self.run_recipe(False)
        h = self.WIDTH / 2
        half = self.CELL * math.sqrt(2) / 2
        band = (d > h - self.ALLOWANCE + half) & (d < h - half)
        self.assertGreater(band.sum(), 1000)
        self.assertTrue((high[band] > rough.floor_z_mm + 1).all())
