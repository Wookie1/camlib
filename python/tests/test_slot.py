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
