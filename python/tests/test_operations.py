from dataclasses import replace
import json
import math
from pathlib import Path
import unittest
import warnings

from camlib import Engine
from camlib.operations import Pocket, RestPocket
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, ProtocolError, Simulation, SimulationError, StockSpec, curve_length
from test_simulation import tool

ROOT = Path(__file__).resolve().parents[2]


def pocket():
    region = PlanarRegion.from_dict(json.loads((ROOT / "fixtures/planar/cadquery-round-island.json").read_text()))
    return Pocket(
        "round-pocket",
        tool(),
        region,
        20,
        region.source["floor_z_mm"],
        2,
        1.5,
        25,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )


def small():
    boundary = Boundary.polygon(((2, 2), (18, 2), (18, 18), (2, 18)))
    region = PlanarRegion("small", (Component(boundary),))
    return Pocket(
        "small-pocket",
        "t1",
        region,
        10,
        4,
        2,
        1.5,
        12,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )


class NativeOperationsTest(unittest.TestCase):
    def test_smaller_tool_rest_operation_uses_actual_previous_stock(self):
        large = tool()
        fine = replace(large, id='fine', cutter=replace(large.cutter, diameter_mm=2, shank_diameter_mm=2))
        rough = small()
        base = replace(rough, id='small-full', tool=fine, stepover_mm=0.75, tool_number=2, length_register=2)
        rest = RestPocket(**{k: getattr(base, k) for k in base.__dataclass_fields__ if k != 'rest_tolerance_mm'})
        results = []
        for final in (base, rest):
            lengths = 0
            report = None
            volume_before = None
            with Simulation(
                StockSpec((0, 0, 0), (20, 20, 10), 0.25),
                (large, fine),
                operations=(rough, final),
                revision='actual-rest',
                initial_position_mm=(0, 0, 12),
            ) as sim:
                for update in sim.run_operations():
                    if isinstance(update, OperationUpdate):
                        if update.operation == rough.id and update.stage == 'end':
                            volume_before = sim.state.volume()['removed_lower_mm3']
                        if update.operation == final.id and update.stage == 'end':
                            report = update.report
                    elif update.motion['operation'] == final.id and update.motion['kind'] == 'cut':
                        lengths += curve_length(update.motion['curve'])
                self.assertTrue(sim.state.finished)
                self.assertGreater(sim.state.volume()['removed_lower_mm3'], volume_before)
                results.append((lengths, report))
        self.assertGreater(results[1][1]['rest_skipped_strokes'], 0)
        self.assertGreater(results[1][1]['rest_merged_pieces'], 0)
        self.assertEqual(results[1][1]['rest_tolerance_mm'], 0.01)
        self.assertLess(results[1][0], results[0][0] * 0.75)
        self.assertGreater(results[1][1]['rest_from_generation'], 0)

    def test_cadquery_fixture_generates_complete_typed_program_and_actual_stock_before_finish(self):
        op = pocket()
        with Simulation(
            StockSpec((0, 0, 0), (80, 60, 20), 0.5),
            (tool(),),
            revision="cadquery-pocket-1",
            initial_position_mm=(0, 0, 25),
            operations=(op,),
        ) as sim:
            self.assertEqual(sim.state.manifest["operations"][0]["region"]["source"], op.region.source)
            stream = sim.run_operations()
            begin = next(stream)
            self.assertIsInstance(begin, OperationUpdate)
            self.assertEqual(begin.report["depth_passes"], 4)
            self.assertEqual(begin.report["geometric_residual_status"], "pending")
            self.assertGreater(begin.report["boundary_strokes"], 4)
            events = list(begin.events)
            first_cut, arc = False, False
            for update in stream:
                if isinstance(update, OperationUpdate):
                    self.assertEqual(update.stage, "end")
                    self.assertIn(update.report["geometric_residual_status"], ("estimated", "unknown"))
                    events.extend(update.events)
                else:
                    events.append({"kind": "motion", "motion": update.motion})
                    arc |= update.motion["curve"]["kind"] == "arc"
                    if update.changed_count and not first_cut:
                        first_cut = True
                        self.assertFalse(sim.state.finished)  # streamed: before the run's end
                        self.assertFalse(sim.state.finished)
                        self.assertGreater(sim.state.volume()["removed_lower_mm3"], 0)
            self.assertTrue(first_cut and arc)
            self.assertTrue(sim.state.finished)
            self.assertEqual(sim.state.position[2], 25)
            ideal = (40 * 30 - 25 * math.pi - 4 * (4 - math.pi)) * 8
            volume = sim.state.volume()
            self.assertLessEqual(volume["removed_lower_mm3"], ideal)
            self.assertGreaterEqual(volume["removed_upper_mm3"], ideal)
            self.assertGreater(volume["removed_lower_mm3"], ideal * 0.85)
            program = dict(
                version=1,
                id="native-pocket",
                revision="1",
                units="mm",
                tolerance_mm=1e-5,
                frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
                initial_position_mm=(0, 0, 25),
                tools=[tool().to_dict()],
                events=events,
            )
            with Engine() as engine:
                checked = engine._request("validate_program", program=program, capabilities=None)
                self.assertEqual(checked["summary"]["motions"], sim.state.completed)

    def test_native_generation_uses_retained_player_seek_and_control_records(self):
        with RecordedRun(
            StockSpec((0, 0, 0), (20, 20, 10), 0.25),
            (tool(),),
            operations=(small(),),
            revision="native-playback",
            initial_position_mm=(0, 0, 12),
            checkpoint_every=16,
            rapid_mm_min=6000,
        ) as run:
            player = Player(run)
            player.pause()
            run.wait()
            self.assertEqual(player.cursor, Cursor())
            controls = list(run.operation_controls())
            self.assertEqual([c["stage"] for c in controls], ["begin", "end"])
            self.assertEqual(controls[-1]["completed"], run.status()["available"])
            player.seek(Cursor(1, 0.8))
            partial = player.state.heights.tobytes()
            player.seek(Cursor(run.status()["available"]))
            self.assertTrue(player.state.finished)
            self.assertGreater(player.state.volume()["removed_lower_mm3"], 1000)
            player.seek(Cursor(1, 0.8))
            self.assertEqual(player.state.heights.tobytes(), partial)
            self.assertEqual(player.motion["operation"], "small-pocket")

    def test_operation_definitions_are_bound_into_every_stock_frame(self):
        config = dict(
            stock=StockSpec((0, 0, 0), (20, 20, 10), 0.25),
            tools=(tool(),),
            revision="same-cad",
            run_id="same-run",
            initial_position_mm=(0, 0, 12),
        )
        with (
            Simulation(**config, operations=(small(),)) as first,
            Simulation(**config, operations=(replace(small(), stepover_mm=1),)) as second,
        ):
            stream = first.run_operations()
            next(stream)
            motion = next(stream)
            initial = second.state.heights.tobytes()
            with self.assertRaisesRegex(ProtocolError, "manifest"):
                second.state.accept(motion.frame)
            self.assertEqual(second.state.heights.tobytes(), initial)
            self.assertNotEqual(first.state.canonical_manifest, second.state.canonical_manifest)
            first.cancel()

    def test_invalid_operation_remains_partial_and_cannot_skip_generation(self):
        config = dict(
            stock=StockSpec((0, 0, 0), (20, 20, 10), 0.25),
            tools=(tool(),),
            revision="invalid",
            initial_position_mm=(0, 0, 12),
        )
        with Simulation(**config, operations=(replace(small(), stepover_mm=3),)) as sim:
            with self.assertRaisesRegex(SimulationError, "pocket parameters"):
                list(sim.run_operations())
            self.assertFalse(sim.state.finished)
            self.assertEqual(sim.state.completed, 0)
        with Simulation(**config, operations=(small(),)) as sim:
            with self.assertRaisesRegex(SimulationError, "operation"):
                sim.finish()
            self.assertFalse(sim.state.finished)

    def test_unbounded_operation_iterables_are_rejected_before_spawning(self):
        import itertools

        config = dict(
            stock=StockSpec((0, 0, 0), (20, 20, 10), 0.25),
            tools=(tool(),),
            revision="too-many",
            initial_position_mm=(0, 0, 12),
            operations=itertools.repeat(small()),
        )
        # Any count of operations, but a finite collection: an endless
        # iterator is refused before anything starts.
        with self.assertRaisesRegex(TypeError, "finite collection"):
            Simulation(**config)
        config["operations"] = itertools.repeat(small())
        with self.assertRaisesRegex(TypeError, "finite collection"):
            RecordedRun(**{'rapid_mm_min': 6000, **config})

    def test_large_jobs_have_no_count_caps(self):
        # 200 tools, 200 operations and 200 scene bodies: counts are the
        # caller's business; only transport sizes are bounded.
        from camlib.verification import Prism, Scene, StockContactOptions, StreamChecks

        base = tool()
        tools = tuple(replace(base, id=f"t{i}") for i in range(200))
        operations = tuple(
            replace(small(), id=f"op{i}", tool=f"t{i}", floor_z_mm=9.5, stepdown_mm=0.5, tool_number=i + 1)
            for i in range(200)
        )
        bodies = tuple(
            Prism(
                f"clamp{i}",
                PlanarRegion(
                    f"f{i}", (Component(Boundary.polygon(((40 + i, 0), (40.5 + i, 0), (40.5 + i, 1), (40 + i, 1)))),)
                ),
                0,
                10,
            )
            for i in range(200)
        )
        checks = StreamChecks(scene=Scene("clamps", "1", bodies=bodies), stock_contact=StockContactOptions())
        with Simulation(
            StockSpec((0, 0, 0), (20, 20, 10), 0.5),
            tools,
            operations=operations,
            verification=checks,
            revision="large",
            initial_position_mm=(0, 0, 12),
        ) as sim:
            seen = set()
            with warnings.catch_warnings():
                # Every operation after the first cuts air (same pocket).
                warnings.simplefilter("ignore")
                for update in sim.run_operations():
                    if isinstance(update, OperationUpdate):
                        seen.add(update.operation)
            self.assertTrue(sim.state.finished)
        self.assertEqual(seen, {f"op{i}" for i in range(200)})

    def test_multiple_operations_have_one_ordered_program_and_distinct_attribution(self):
        first = replace(small(), floor_z_mm=8)
        second = replace(small(), id="deeper", top_z_mm=10, floor_z_mm=4)
        with Simulation(
            StockSpec((0, 0, 0), (20, 20, 10), 0.5),
            (tool(),),
            operations=(first, second),
            revision="ordered",
            initial_position_mm=(0, 0, 12),
        ) as sim:
            controls, operations = [], set()
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    controls.append((update.operation, update.stage))
                else:
                    operations.add(update.motion["operation"])
            self.assertEqual(
                controls, [(first.id, "begin"), (first.id, "end"), (second.id, "begin"), (second.id, "end")]
            )
            self.assertEqual(operations, {first.id, second.id})
            self.assertTrue(sim.state.finished)


if __name__ == "__main__":
    unittest.main()
