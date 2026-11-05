"""Exercise local orbit selection through the public Python/stream interface."""

from dataclasses import replace
from functools import lru_cache
import importlib.util
import math
from pathlib import Path
import unittest

from camlib import Engine
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError


@lru_cache(None)
def example():
    path = Path(__file__).resolve().parents[2] / 'python/tests/support/adaptive_tapered.py'
    spec = importlib.util.spec_from_file_location('tapered_example', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@lru_cache(None)
def config():
    return example().tapered_job()


class TaperedAdaptiveTest(unittest.TestCase):
    def test_real_cad_local_targets_cut_material_and_validate_as_typed_program(self):
        c = config()
        shape = example().target().val()
        self.assertTrue(shape.isValid())
        self.assertAlmostEqual(shape.Volume(), 6472.0)
        events, radii = [], []
        first = False
        report = None
        with Simulation(**c) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    events.extend(update.events)
                    if update.stage == 'end':
                        report = update.report
                    continue
                motion = update.motion
                events.append(dict(kind='motion', motion=motion))
                curve = motion['curve']
                if motion['kind'] == 'cut' and curve['kind'] == 'arc':
                    radii.append(
                        math.hypot(curve['start'][0] - curve['center'][0], curve['start'][1] - curve['center'][1])
                    )
                if not first and sim.state.volume()['removed_lower_mm3'] > 0.001:
                    first = True
                    self.assertFalse(sim.state.finished)
                    self.assertFalse(sim.state.finished)  # streamed: before the run's end
            self.assertTrue(first and sim.state.finished)
            self.assertEqual(report['navigation_components'], 1)
            self.assertEqual(report['seed_orbits'], 1)
            self.assertGreater(report['local_radius_splits'], 0)
            self.assertGreater(report['locally_sized_guides'], 0)
            self.assertGreater(report['distinct_guide_radii'], 2)
            self.assertLessEqual(report['max_bounded_engagement_deg'], 90.0)
            self.assertTrue(
                any(
                    0.30 < target < 0.95 and any(abs(actual - target) < 1e-8 for actual in radii)
                    for target in report['guide_orbit_radii_mm']
                )
            )
            self.assertGreater(sim.state.volume()['removed_lower_mm3'], 250.0)
            self.assertLessEqual(sim.state.volume()['removed_lower_mm3'], 328.0 + 1e-7)
            count = sim.state.completed
        program = dict(
            version=1,
            id='tapered-program',
            revision='1',
            units='mm',
            tolerance_mm=1e-5,
            frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
            initial_position_mm=(0, 0, 12),
            tools=[t.to_dict() for t in c['tools']],
            events=events,
        )
        with Engine() as engine:
            self.assertEqual(
                engine._request('validate_program', program=program, capabilities=None)['summary']['motions'], count
            )

    def test_partial_local_radius_replay_and_resolution_admission(self):
        c = config()
        with RecordedRun(**{'rapid_mm_min': 6000, **c}, checkpoint_every=32) as run:
            player = Player(run)
            player.pause()
            run.wait(timeout=60)
            count = run.status()['available']
            partial = None
            for i in range(min(count, 600)):
                player.seek(Cursor(i, 0.5))
                motion = player.motion
                curve = motion['curve']
                if (
                    motion['kind'] == 'link'
                    and motion['feed_mm_min'] == 150
                    and curve['start'][2] == curve['end'][2] == 9
                ):
                    partial = Cursor(i, 0.5)
                    break
            self.assertIsNotNone(partial)
            saved, position = player.state.heights.tobytes(), player.state.position
            player.seek(Cursor(count))
            self.assertTrue(player.state.finished)
            player.seek(Cursor())
            player.seek(partial)
            self.assertEqual(player.state.position, position)
            self.assertEqual(player.state.heights.tobytes(), saved)
            self.assertLessEqual(run.status()['largest_frame_bytes'], 8 * 1024 * 1024)
            self.assertEqual(player.state.manifest['run']['revision'], c['revision'])
        operation = c['operations'][0]
        self.assertNotIn('local_orbit_tolerance_mm', replace(operation, local_orbit_tolerance_mm=None).to_dict())
        for tolerance in (0, -0.05, 0.0001, 2):
            bad = dict(c, operations=(replace(operation, local_orbit_tolerance_mm=tolerance),))
            with Simulation(**bad) as sim:
                with self.assertRaises(SimulationError):
                    list(sim.run_operations())
                self.assertEqual(sim.state.completed, 0)
