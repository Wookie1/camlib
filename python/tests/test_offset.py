"""Ordinary Python offset clearing through CAD, stock, replay, cache and posts."""

from camlib.postprocessor import GCodeDialect

from dataclasses import replace
from functools import lru_cache
from tempfile import TemporaryDirectory
import unittest

from camlib import Engine
from camlib.jobs import Job, Setup, RegenerationCache
from camlib.motion import Program
from camlib.operations import OffsetClearing
from camlib.playback import Cursor, Player, RecordedRun
from camlib.post import LoweringOptions, Machine, ToolSetting, post_events
from camlib.simulation import OperationUpdate, Simulation, SimulationError, StockState, ProtocolError
from test_roughing import config, example


@lru_cache(None)
def offset_config():
    return example().roughing_job(offset=True)


@lru_cache(None)
def offset_run():
    events, report, partials = [], None, {}
    first = False
    motions = 0
    with Simulation(**offset_config()) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate):
                events.extend(update.events)
                if update.stage == 'end':
                    report = update.report
            else:
                motion = update.motion
                events.append(dict(kind='motion', motion=motion))
                kind = motion['curve']['kind']
                # The motion's own index (the stock state may already hold
                # the rest of its decoded batch).
                if kind == 'helix' or kind == 'arc' and motion['kind'] == 'cut':
                    partials.setdefault(kind, motions)
                motions += 1
                if sim.state.volume()['removed_lower_mm3'] > 0.001:
                    first = True
                    assert not sim.state.finished
        assert first and sim.state.finished
        return dict(
            events=events,
            report=report,
            partials=partials,
            heights=sim.state.heights.tobytes(),
            count=sim.state.completed,
        )


class OffsetClearingTest(unittest.TestCase):
    def test_authoring_actual_cad_typed_stream_coverage_and_invalid_preflight(self):
        self.assertEqual(OffsetClearing(0.8).to_dict(), dict(kind='offset', stepover_mm=0.8, coverage_margin_mm=0.01))
        result = offset_run()
        self.assertEqual(result['report']['clearing'], 'offset')
        self.assertEqual(result['report']['completed_layers'], 4)
        for layer in result['report']['layers']:
            r = layer['clearing']
            self.assertEqual(r['raster_strokes'], 0)
            self.assertEqual(r['boundary_strokes'], r['xy_strokes'])
            self.assertGreater(r['offset']['levels'], 1)
            self.assertGreater(r['offset']['coverage_tiles'], 0)
            self.assertLessEqual(r['offset']['coverage_max_bound_mm'], 0.99)
            self.assertLessEqual(r['offset']['work'], 20_000_000)
        c = offset_config()
        header = Program('offset-program', '1', c['tools'], c['initial_position_mm'], (), tolerance_mm=1e-5)
        program = dict(header.to_dict(), events=result['events'])
        with Engine() as engine:
            self.assertEqual(
                engine._request('validate_program', program=program, capabilities=None)['summary']['motions'],
                result['count'],
            )
        for clearing in [OffsetClearing(0.8, 0), OffsetClearing(0.8, 0.3), OffsetClearing(-1), OffsetClearing(2)]:
            bad = dict(c, operations=(replace(c['operations'][0], clearing=clearing),))
            with self.assertRaises(SimulationError):
                with Simulation(**bad) as sim:
                    list(sim.run_operations())

    def test_partial_helix_and_arc_stock_seek_and_changed_strategy_revision(self):
        c = offset_config()
        with RecordedRun(**{'rapid_mm_min': 6000, **c}, checkpoint_every=128) as run:
            run.wait(timeout=60)
            player = Player(run)
            for kind in ['helix', 'arc']:
                index = offset_run()['partials'][kind]
                player.seek(Cursor(index))
                before = player.state.volume()['removed_lower_mm3']
                cursor = Cursor(index, 0.371)
                player.seek(cursor)
                self.assertEqual(player.motion['curve']['kind'], kind)
                if kind == 'helix':
                    self.assertGreater(player.state.volume()['removed_lower_mm3'], before + 0.001)
                saved, position = player.state.heights.tobytes(), player.state.position
                player.seek(Cursor(run.status()['available']))
                self.assertEqual(player.state.heights.tobytes(), offset_run()['heights'])
                player.seek(Cursor())
                player.seek(cursor)
                self.assertEqual(player.state.heights.tobytes(), saved)
                self.assertEqual(player.state.position, position)
        with Simulation(**config(), run_id='same-offset-run') as sim:
            for update in sim.run_operations():
                if not isinstance(update, OperationUpdate):
                    foreign = update.frame
                    break
        with Simulation(**c, run_id='same-offset-run') as sim:
            fresh = StockState(sim.manifest_frame)
            with self.assertRaises(ProtocolError):
                fresh.accept(foreign)
            self.assertEqual(fresh.completed, 0)

    def test_python_checked_post_and_strategy_cache_invalidation(self):
        c = offset_config()
        header = Program('posted-offset', '1', c['tools'], c['initial_position_mm'], (), tolerance_mm=1e-5)
        machine = Machine(
            'Python offset mill',
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
        chunks = list(post_events(header, offset_run()['events'], machine=machine, lowering=LoweringOptions()))
        self.assertTrue(chunks[-1].text.endswith('M2\n'))
        maps = [mapping for chunk in chunks for mapping in chunk.data.get('mappings', ())]
        for event in offset_run()['events']:
            if event['kind'] == 'motion' and event['motion']['curve']['kind'] in ('arc', 'helix'):
                parts = [m['source_fraction_interval'] for m in maps if m['source_motion_id'] == event['motion']['id']]
                self.assertEqual(parts[0][0], 0)
                self.assertEqual(parts[-1][1], 1)
                self.assertTrue(all(a[1] == b[0] for a, b in zip(parts, parts[1:])))

        def job(offset):
            main = c['operations'][0] if offset else config()['operations'][0]
            other = replace(config()['operations'][0], id='other-rough')
            return Job(
                'offset-job',
                str(offset),
                tuple(
                    Setup(name, c['stock'], c['tools'], (op,), c['initial_position_mm'])
                    for name, op in [('main', main), ('other', other)]
                ),
            )

        def collect(job, cache):
            with job.regenerate(cache) as run:
                list(run)
                self.assertTrue(run.finished)
                return dict(run.statistics)

        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            cold = collect(job(False), cache)
            self.assertTrue(all(not v['cache_hit'] for v in cold.values()))
            changed = collect(job(True), cache)
            self.assertFalse(changed[('main', 'rough')]['cache_hit'])
            self.assertTrue(changed[('other', 'other-rough')]['cache_hit'])
            warm = collect(job(True), cache)
            self.assertTrue(all(v['cache_hit'] for v in warm.values()))
