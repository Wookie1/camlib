"""Curved CAD through three native cutters, committed stock and checked output."""

from dataclasses import replace
from functools import lru_cache
from tempfile import TemporaryDirectory
import unittest

from camlib.jobs import RegenerationCache
from camlib.operations import RestRoughing, OffsetClearing, AdaptiveClearing
from camlib.playback import Cursor, Player
from camlib.post import LoweringOptions
from camlib.simulation import Simulation, SimulationError
from test_cad_surfaces import example
from test_rough_finish import collect, engine


@lru_cache(None)
def job():
    return example().build_machining_job()


@lru_cache(None)
def offset_job():
    base = job()
    setup = base.setups[0]
    rest = replace(setup.operations[1], clearing=OffsetClearing(0.4))
    return replace(base, setups=(replace(setup, operations=(setup.operations[0], rest, *setup.operations[2:])),))


class CurvedRoughingTest(unittest.TestCase):
    machining_job = staticmethod(job)

    def test_curved_three_tool_stock_material_and_cached_replay(self):
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            cold = collect(self.machining_job(), cache)
            self.assertEqual(set(cold['ends']), {'rough', 'rest', 'finish'})
            self.assertEqual(set(cold['statuses']), {'clear'})
            self.assertLess(cold['volumes']['rough'], cold['volumes']['rest'])
            self.assertLess(cold['volumes']['rest'], cold['volumes']['finish'])
            rest = cold['ends']['rest']
            self.assertEqual(rest['rest_tolerance_mm'], 0.01)
            self.assertEqual(rest['completed_layers'], 4)
            layers = [layer['clearing'] for layer in rest['layers']]
            self.assertGreater(sum(layer['rest_skipped_strokes'] for layer in layers), 0)
            self.assertFalse(any(layer['rest_query_work_exhausted'] for layer in layers))
            self.assertEqual(len(cold['materials']), 3)
            for check in cold['materials']:
                self.assertTrue(check['report']['complete'])
                self.assertEqual(check['report']['gouge']['definite_cells'], 0)
                self.assertEqual(check['report']['context']['generation'], cold['counts'][check['after_operation']])
            warm = collect(replace(self.machining_job(), revision='2'), cache)
            self.assertTrue(all(stat['cache_hit'] for stat in warm['stats'].values()))
            for op in cold['states']:
                self.assertEqual(cold['states'][op].heights.tobytes(), warm['states'][op].heights.tobytes())

    def test_three_tool_post_stock_and_fractional_seek(self):
        with self.machining_job().post(
            'top',
            machine=example().machine_for(self.machining_job()),
            executable=engine(),
            checkpoint_every=64,
            lowering=LoweringOptions(),
        ) as run:
            run.wait(timeout=180)
            self.assertTrue(run.posting_status()['finished'])
            self.assertGreater(run.posting_status()['checked_motions'], 100)
            changes, start = 0, 1
            while page := run.nc_lines(start, limit=256):
                changes += sum(row['text'] == 'M6' for row in page)
                start = page[-1]['number'] + 1
            self.assertEqual(changes, 3)
            player = Player(run)
            end = Cursor(run.status()['available'])
            player.seek(end)
            self.assertEqual(player.verification['progress']['violation'], 0)
            self.assertEqual(player.verification['progress']['uncertain'], 0)
            final = player.state.heights.tobytes()
            cursor = Cursor(end.completed // 2, 0.5)
            player.seek(cursor)
            partial = player.state.heights.tobytes()
            player.seek(end)
            self.assertEqual(player.state.heights.tobytes(), final)
            player.seek(cursor)
            self.assertEqual(player.state.heights.tobytes(), partial)

    def test_rest_policy_refuses_unsupported_strategy_before_motion(self):
        setup = job().setups[0]
        rest = setup.operations[1]
        self.assertIsInstance(rest, RestRoughing)
        bad = replace(rest, clearing=AdaptiveClearing(0.2))
        refused = replace(job(), setups=(replace(setup, operations=(bad,), verification=None),))
        config = refused.compile().setup('top').simulation_options()
        with Simulation(**config, executable=engine()) as sim:
            with self.assertRaisesRegex(SimulationError, 'Adaptive rest filtering is not implemented'):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)


class OffsetRestRoughingTest(CurvedRoughingTest):
    machining_job = staticmethod(offset_job)

    def test_strategy_change_invalidates_rest_and_downstream_stock_only(self):
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            collect(job(), cache)
            changed = collect(replace(offset_job(), revision='offset-2'), cache)
            self.assertTrue(changed['stats'][('top', 'rough')]['cache_hit'])
            self.assertFalse(changed['stats'][('top', 'rest')]['cache_hit'])
            self.assertFalse(changed['stats'][('top', 'finish')]['cache_hit'])
            self.assertGreater(changed['volumes']['rest'], changed['volumes']['rough'])
            for layer in changed['ends']['rest']['layers']:
                self.assertIn('before rest filtering', layer['clearing']['offset']['coverage_scope'])
                self.assertEqual(layer['clearing']['routing']['reversed_rasters'], 0)
