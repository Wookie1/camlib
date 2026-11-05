from dataclasses import replace
from functools import lru_cache
import importlib.util
from pathlib import Path
import unittest
from camlib import Engine
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError


@lru_cache(None)
def example():
    path = Path(__file__).resolve().parents[2] / "python/tests/support/adaptive_connected.py"
    spec = importlib.util.spec_from_file_location("connected_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@lru_cache(None)
def config():
    return example().connected_job()


class ConnectedAdaptiveTest(unittest.TestCase):
    def test_real_cad_one_connected_entry_material_and_typed_program(self):
        c = config()
        shape = example().target().val()
        self.assertTrue(shape.isValid())
        self.assertAlmostEqual(shape.Volume(), 6480.0)
        events = []
        first = False
        report = None
        plunges = 0
        with Simulation(**c) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    events.extend(update.events)
                    if update.stage == "end":
                        report = update.report
                else:
                    events.append(dict(kind="motion", motion=update.motion))
                    plunges += update.motion["kind"] == "plunge"
                    if not first and sim.state.volume()["removed_lower_mm3"] > 0.001:
                        first = True
                        self.assertFalse(sim.state.finished)
                        self.assertFalse(sim.state.finished)  # streamed: before the run's end
            self.assertTrue(first and sim.state.finished)
            self.assertEqual(plunges, 1)
            self.assertEqual(report["navigation_components"], 1)
            self.assertEqual(report["seed_orbits"], 1)
            self.assertAlmostEqual(report["min_cutting_orbit_radius_mm"], 0.25)
            self.assertLessEqual(report["max_bounded_engagement_deg"], 90.0)
            self.assertGreater(report["committed_tube_advances"], 0)
            self.assertGreater(report["omitted_endpoint_orbits"], 0)
            self.assertGreater(sim.state.volume()["removed_lower_mm3"], 250.0)
            self.assertLessEqual(sim.state.volume()["removed_lower_mm3"], 6800.0 - shape.Volume() + 1e-7)
            for y in range(sim.state.ny):
                for x in range(sim.state.nx):
                    px = (x + 0.5) * sim.state.dx
                    py = (y + 0.5) * sim.state.dy
                    if 14 <= px <= 20 and 9.7 <= py <= 10.3:
                        self.assertLess(sim.state.heights[2 * (y * sim.state.nx + x) + 1], 9.01)
            count = sim.state.completed
        program = dict(
            version=1,
            id="connected-program",
            revision="1",
            units="mm",
            tolerance_mm=1e-5,
            frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
            initial_position_mm=(0, 0, 12),
            tools=[t.to_dict() for t in c["tools"]],
            events=events,
        )
        with Engine() as engine:
            self.assertEqual(
                engine._request("validate_program", program=program, capabilities=None)["summary"]["motions"], count
            )

    def test_partial_radius_change_replay_and_bad_minimum_before_motion(self):
        c = config()
        with RecordedRun(**{'rapid_mm_min': 6000, **c}, checkpoint_every=32) as run:
            player = Player(run)
            player.pause()
            run.wait(timeout=60)
            count = run.status()["available"]
            partial = None
            for i in range(min(count, 400)):
                player.seek(Cursor(i, 0.5))
                motion = player.motion
                curve = motion["curve"]
                if (
                    motion["kind"] == "link"
                    and motion["feed_mm_min"] == 150
                    and curve["start"][2] == curve["end"][2] == 9
                ):
                    partial = Cursor(i, 0.5)
                    break
            self.assertIsNotNone(partial)
            saved = player.state.heights.tobytes()
            position = player.state.position
            player.seek(Cursor(count))
            self.assertTrue(player.state.finished)
            player.seek(Cursor())
            player.seek(partial)
            self.assertEqual(player.state.position, position)
            self.assertEqual(player.state.heights.tobytes(), saved)
            self.assertLessEqual(run.status()["largest_frame_bytes"], 8 * 1024 * 1024)
            self.assertEqual(player.state.manifest["run"]["revision"], c["revision"])
        for minimum in [0, -1, 2]:
            bad = dict(c, operations=(replace(c["operations"][0], min_orbit_radius_mm=minimum),))
            with Simulation(**bad) as sim:
                with self.assertRaises(SimulationError):
                    list(sim.run_operations())
                self.assertEqual(sim.state.completed, 0)
