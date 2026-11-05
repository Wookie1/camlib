"""Economical native routing keeps actual stock, source identity and Python posts."""

from camlib.postprocessor import GCodeDialect

from dataclasses import replace
from functools import lru_cache
from tempfile import TemporaryDirectory
import unittest

from camlib import Engine
from camlib.jobs import Job, Setup, RegenerationCache
from camlib.motion import Program
from camlib.operations import RasterClearing
from camlib.playback import Cursor, Player, RecordedRun
from camlib.post import LoweringOptions, Machine, ToolSetting, post_events
from camlib.simulation import OperationUpdate, Simulation, SimulationError, StockState, ProtocolError
from test_roughing import config, stream


def economical():
    c = config()
    return dict(
        c,
        operations=(replace(c['operations'][0], clearing=replace(c['operations'][0].clearing, routing='economical')),),
    )


@lru_cache(None)
def routed():
    events, report, helix = [], None, None
    first = False
    motions = 0
    with Simulation(**economical()) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate):
                events.extend(update.events)
                if update.stage == 'end':
                    report = update.report
            else:
                events.append(dict(kind='motion', motion=update.motion))
                # The motion's own index (the stock state may already hold
                # the rest of its decoded batch).
                if update.motion['curve']['kind'] == 'helix' and helix is None:
                    helix = motions
                motions += 1
                if sim.state.volume()['removed_lower_mm3'] > 0.001:
                    first = True
                    assert not sim.state.finished
        assert first and sim.state.finished
        return dict(
            events=events, report=report, helix=helix, heights=sim.state.heights.tobytes(), count=sim.state.completed
        )


class RasterRoutingTest(unittest.TestCase):
    def test_authoring_omission_policy_preflight_and_complete_typed_stream(self):
        self.assertEqual(RasterClearing(0.8).to_dict(), dict(kind='raster', stepover_mm=0.8))
        self.assertEqual(RasterClearing(0.8, routing='economical').to_dict()['routing'], 'economical')
        result = routed()
        old = stream()

        def nav(r):
            return sum(item['clearing']['entry_navigation_moves'] for item in r['layers'])

        self.assertLessEqual(2 * nav(result['report']), nav(old['report']))
        self.assertLess(result['count'], old['count'])
        self.assertEqual(result['report']['stock_edge_entries'], 4)
        self.assertEqual(result['report']['helical_entries'], 4)
        for layer in result['report']['layers']:
            self.assertEqual(layer['clearing']['routing']['policy'], 'economical')
            self.assertLessEqual(layer['clearing']['routing']['work'], 20_000_000)
        c = economical()
        program = dict(
            version=1,
            id='routed-program',
            revision='1',
            units='mm',
            tolerance_mm=1e-5,
            frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
            initial_position_mm=c['initial_position_mm'],
            tools=[t.to_dict() for t in c['tools']],
            events=result['events'],
        )
        with Engine() as engine:
            self.assertEqual(
                engine._request('validate_program', program=program, capabilities=None)['summary']['motions'],
                result['count'],
            )
        bad = dict(c, operations=(replace(c['operations'][0], clearing=RasterClearing(0.8, routing='unknown')),))
        with self.assertRaises(SimulationError):
            with Simulation(**bad) as sim:
                list(sim.run_operations())

    def test_partial_helix_replay_and_changed_policy_chunk_isolation(self):
        c = economical()
        with RecordedRun(**{'rapid_mm_min': 6000, **c}, checkpoint_every=128) as run:
            run.wait(timeout=60)
            player = Player(run)
            index = routed()['helix']
            self.assertIsNotNone(index)
            player.seek(Cursor(index))
            before = player.state.volume()['removed_lower_mm3']
            partial = Cursor(index, 0.371)
            player.seek(partial)
            self.assertEqual(player.motion['curve']['kind'], 'helix')
            self.assertGreater(player.state.volume()['removed_lower_mm3'], before + 0.001)
            saved, position = player.state.heights.tobytes(), player.state.position
            player.seek(Cursor(run.status()['available']))
            self.assertEqual(player.state.heights.tobytes(), routed()['heights'])
            player.seek(Cursor())
            player.seek(partial)
            self.assertEqual(player.state.heights.tobytes(), saved)
            self.assertEqual(player.state.position, position)
        with Simulation(**config(), run_id='same-routing-run') as sim:
            for update in sim.run_operations():
                if not isinstance(update, OperationUpdate):
                    foreign = update.frame
                    break
        with Simulation(**c, run_id='same-routing-run') as sim:
            fresh = StockState(sim.manifest_frame)
            with self.assertRaises(ProtocolError):
                fresh.accept(foreign)
            self.assertEqual(fresh.completed, 0)

    def test_checked_post_and_policy_cache_invalidation_preserve_independent_setup(self):
        c = config()
        header = Program('posted-routing', '1', c['tools'], c['initial_position_mm'], (), tolerance_mm=1e-5)
        machine = Machine(
            'Python routed mill',
            (
                ToolSetting.for_tool(
                    c['tools'][0], number=1, length_register=1, position_after_mm=c['initial_position_mm']
                ),
            ),
            dialect=GCodeDialect.linuxcnc(),
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=('xy', 'xz', 'yz'),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            tool_change_position_mode='table',
            rapid_mm_min=6000,
        )
        chunks = list(post_events(header, routed()['events'], machine=machine, lowering=LoweringOptions()))
        self.assertTrue(chunks[-1].text.endswith('M2\n'))
        maps = [m for chunk in chunks for m in chunk.data.get('mappings', ())]
        for event in routed()['events']:
            if event['kind'] == 'motion' and event['motion']['curve']['kind'] == 'helix':
                parts = [m['source_fraction_interval'] for m in maps if m['source_motion_id'] == event['motion']['id']]
                self.assertEqual(parts[0][0], 0)
                self.assertEqual(parts[-1][1], 1)
                self.assertTrue(all(a[1] == b[0] for a, b in zip(parts, parts[1:])))

        def job(optimized):
            op = economical()['operations'][0] if optimized else c['operations'][0]
            other = replace(c['operations'][0], id='other-rough')
            setups = tuple(
                Setup(id, c['stock'], c['tools'], (operation,), c['initial_position_mm'])
                for id, operation in [('main', op), ('other', other)]
            )
            return Job('routing-job', str(optimized), setups)

        def collect(j, cache):
            ends = {}
            with j.regenerate(cache) as run:
                for event in run:
                    if isinstance(event.update, OperationUpdate) and event.update.stage == 'end':
                        ends[event.setup] = event.update.report
                self.assertTrue(run.finished)
                return dict(run.statistics), ends

        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            cold, old = collect(job(False), cache)
            self.assertTrue(all(not v['cache_hit'] for v in cold.values()))
            changed, new = collect(job(True), cache)
            self.assertFalse(changed[('main', 'rough')]['cache_hit'])
            self.assertTrue(changed[('other', 'other-rough')]['cache_hit'])
            self.assertLess(new['main']['motions'], old['main']['motions'])
            warm, _ = collect(job(True), cache)
            self.assertTrue(all(v['cache_hit'] for v in warm.values()))
