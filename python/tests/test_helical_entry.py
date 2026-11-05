"""Real CAD entry authoring, authoritative helix playback and checked Python posts."""

from camlib.postprocessor import GCodeDialect

from dataclasses import replace
from functools import lru_cache
import importlib.util
import math
from pathlib import Path
import unittest

from camlib import Engine
from camlib.motion import Program
from camlib.operations import HelicalEntry
from camlib.playback import Cursor, Player, RecordedRun
from camlib.post import LoweringOptions, Machine, ToolSetting, post_events
from camlib.simulation import OperationUpdate, Simulation, SimulationError

ROOT = Path(__file__).resolve().parents[2]


@lru_cache(None)
def example(name='helical_pocket'):
    spec = importlib.util.spec_from_file_location(
        name + '_example', ROOT / 'python' / 'tests' / 'support' / (name + '.py')
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@lru_cache(None)
def config(adaptive=False):
    return example().helical_job(adaptive)


def stream(adaptive=False):
    c = config(adaptive)
    events = []
    report = None
    helices = []
    first = False
    with Simulation(**c) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate):
                events.extend(update.events)
                if update.stage == 'end':
                    report = update.report
            else:
                events.append(dict(kind='motion', motion=update.motion))
                if update.motion['curve']['kind'] == 'helix':
                    helices.append(update.motion)
                if not first and sim.state.volume()['removed_lower_mm3'] > 0.001:
                    first = True
                    assert not sim.state.finished and sim._process.poll() is None
        assert first and sim.state.finished
        return events, report, helices, sim.state.completed, sim.state.volume()


class HelicalEntryTest(unittest.TestCase):
    def test_real_cad_both_native_strategies_stream_entries_and_validate_typed_motion(self):
        target = example().target().val()
        self.assertTrue(target.isValid())
        self.assertAlmostEqual(target.Volume(), 32 * 24 * 10 - 4 * (26 * 18 - 9 * math.pi))
        for adaptive in (False, True):
            c = config(adaptive)
            events, report, helices, count, volume = stream(adaptive)
            # Each descent is emitted as consecutive one-turn helix motions.
            descents = []
            for m in helices:
                if descents and tuple(descents[-1][-1]['curve']['end']) == tuple(m['curve']['start']):
                    descents[-1].append(m)
                else:
                    descents.append([m])
            self.assertEqual(len(descents), 2)
            self.assertTrue(all(abs(abs(m['curve']['sweep_rad']) - math.tau) < 1e-9 for m in helices))
            self.assertEqual(report['helical_entries'], 2)
            self.assertEqual(report['entry_turns'], 20 if adaptive else 14)
            self.assertEqual(
                report['entry_turns'], sum(round(abs(m['curve']['sweep_rad']) / math.tau) for m in helices)
            )
            self.assertEqual(report.get('cleared_entry_descents', 0), 0 if adaptive else 1)
            self.assertEqual(report['entry_radii_mm'], [1.0])
            self.assertGreater(report['entry_geometry_work'], 0)
            for i, descent in enumerate(descents):
                curve = descent[0]['curve']
                if adaptive or i == 0:
                    self.assertEqual(curve['start'][2], 10)
                else:
                    previous_floor = descents[i - 1][-1]['curve']['end'][2]
                    self.assertGreater(curve['start'][2], previous_floor)
                    self.assertLessEqual(curve['start'][2], previous_floor + 0.011)
            for motion in helices:
                curve = motion['curve']
                self.assertEqual(motion['feed_mm_min'], 150)
                radius = math.hypot(curve['start'][0] - curve['center'][0], curve['start'][1] - curve['center'][1])
                angle = math.atan((curve['start'][2] - curve['end'][2]) / (radius * abs(curve['sweep_rad'])))
                self.assertLessEqual(angle, math.radians(3) + 1e-12)
            self.assertGreater(volume['removed_lower_mm3'], 1500)
            self.assertLessEqual(volume['removed_lower_mm3'], 4 * (468 - 9 * math.pi))
            program = dict(
                version=1,
                id='helical-program',
                revision='1',
                units='mm',
                tolerance_mm=1e-5,
                frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
                initial_position_mm=c['initial_position_mm'],
                tools=[t.to_dict() for t in c['tools']],
                events=events,
            )
            with Engine() as engine:
                self.assertEqual(
                    engine._request('validate_program', program=program, capabilities=None)['summary']['motions'], count
                )
            self.assertNotIn('entry', replace(c['operations'][0], entry=None).to_dict())

    def test_partial_helix_seek_is_actual_material_and_invalid_input_has_no_motion(self):
        for adaptive in (False, True):
            c = config(adaptive)
            with RecordedRun(**{'rapid_mm_min': 6000, **c}, checkpoint_every=8) as run:
                player = Player(run)
                player.pause()
                run.wait(timeout=60)
                partial = None
                for i in range(min(run.status()['available'], 12)):
                    player.seek(Cursor(i, 0.371))
                    if player.motion['curve']['kind'] == 'helix':
                        partial = Cursor(i, 0.371)
                        break
                self.assertIsNotNone(partial)
                saved = player.state.heights.tobytes()
                position = player.state.position
                self.assertGreater(player.state.volume()['removed_lower_mm3'], 0.001)
                player.seek(Cursor(run.status()['available']))
                self.assertTrue(player.state.finished)
                self.assertGreater(player.state.volume()['removed_lower_mm3'], 1500)
                player.seek(Cursor())
                player.seek(partial)
                self.assertEqual(player.state.heights.tobytes(), saved)
                self.assertEqual(player.state.position, position)
                self.assertLessEqual(run.status()['largest_frame_bytes'], 8 * 1024 * 1024)
            for entry in (
                HelicalEntry(angle_deg=0),
                HelicalEntry(radius_mm=0),
                HelicalEntry(angle_deg=0.00001),
                HelicalEntry(radius_mm=3),
            ):
                bad = dict(c, operations=(replace(c['operations'][0], entry=entry),))
                with Simulation(**bad) as sim:
                    with self.assertRaises(SimulationError):
                        list(sim.run_operations())
                    self.assertEqual(sim.state.completed, 0)

    def test_multiturn_entry_posts_natively_or_lowered_and_is_independently_decoded(self):
        c = config()
        events, _, helices, _, _ = stream()
        header = Program('posted-helical-entry', '1', c['tools'], c['initial_position_mm'], (), tolerance_mm=1e-5)
        setting = ToolSetting.for_tool(
            c['tools'][0], number=1, length_register=1, position_after_mm=c['initial_position_mm']
        )
        machine = Machine(
            'User Python machine with one-turn circles',
            (setting,),
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
        # Entries are planned one turn per motion, so a machine limited to
        # one-turn circles posts them natively as G2/G3 blocks.
        native = list(post_events(header, events, machine=machine))
        decoded = [m for chunk in native for m in chunk.motions if m.curve.to_dict()['kind'] == 'helix']
        self.assertEqual(len(decoded), len(helices))
        self.assertTrue(all(abs(m.curve.sweep_rad) <= math.tau + 1e-9 for m in decoded))
        for helices_supported in (True, False):
            chunks = list(
                post_events(
                    header, events, machine=replace(machine, helices=helices_supported), lowering=LoweringOptions()
                )
            )
            lowered = [m for chunk in chunks for m in chunk.motions]
            # Existing explicit LoweringOptions selects certified LINE output,
            # including when the declared machine also supports circular motion.
            self.assertTrue(lowered)
            self.assertTrue(all(m.curve.to_dict()['kind'] == 'line' for m in lowered))
            self.assertTrue(chunks[-1].text.endswith('M2\n'))
            mappings = [mapping for chunk in chunks for mapping in chunk.data.get('mappings', ())]
            for helix in helices:
                fractions = [
                    mapping['source_fraction_interval']
                    for mapping in mappings
                    if mapping['source_motion_id'] == helix['id']
                ]
                self.assertGreater(len(fractions), abs(helix['curve']['sweep_rad']) / math.tau)
                self.assertEqual(fractions[0][0], 0)
                self.assertEqual(fractions[-1][1], 1)
                for left, right in zip(fractions, fractions[1:]):
                    self.assertEqual(left[1], right[0])
            lines = [line for chunk in chunks for line in chunk.data.get('lines', ())]
            self.assertTrue(any(line['motion_id'] is not None for line in lines))
            self.assertTrue(all(line['operation'] == 'helical' for line in lines if line['motion_id'] is not None))
