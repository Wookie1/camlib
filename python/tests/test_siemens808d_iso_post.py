from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest

from camlib.motion import Arc, Helix, Line, Motion, Coolant, Spindle, Stop, ToolChange, WorkFrame, WorkOffset
from camlib.post import Cancellation, LoweringOptions, PostEngine, ToolSetting
from test_post import fixture
from siemens808d_iso_oracle import Oracle

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'examples'))
try:
    from siemens808d_iso_post import (
        define_machine,
        Siemens808DISOPost,
        Siemens808DISODecoder,
        post_iso,
        post_events_iso,
    )
finally:
    sys.path.pop(0)

OEM = dict(off='M9', flood='M8', mist='M7')


class SiemensISO(unittest.TestCase):
    def machine(self, **settings):
        p, s = fixture()
        p = replace(p, events=(*p.events[:5], Spindle(0), Coolant()))
        defaults = dict(
            startup_units=settings.get('output_units', 'mm'),
            extern_toolprog_mode=0,
            cutting_edge_default=0,
            tool_corr_move_mode=0,
            tool_length_const=17,
            tool_change='m6',
            coolant_codes=OEM,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(origin_mm=(100, 200, 300)),
        )
        defaults.update(settings)
        return p, define_machine(
            'ISO milling example',
            s.tools,
            **{
                'max_feed_mm_min': 10000,
                'max_spindle_rpm': 30000,
                'max_sweep_rad': 6.283185307179586,
                'max_tool_number': 99999,
                'max_length_register': 99999,
                'tool_changes': True,
                'tool_change_position_mode': 'table',
                'rapid_mm_min': 6000,
                'output_units': (defaults).get('output_units', 'mm'),
                'decimals': 3 if (defaults).get('output_units', 'mm') == 'mm' else 4,
                **(defaults),
            },
        )

    def oracle(self, p, m, chunks):
        expected = [x for c in chunks for x in c.motions]
        for calculator in (False, True):
            for increment in ('B', 'C'):
                oracle = Oracle(
                    tools=m.tools,
                    initial_position=p.initial_position_mm,
                    coolant_codes=OEM,
                    tool_change=m.dialect.tool_change,
                    units=m.output_units,
                    calculator=calculator,
                    increment=increment,
                    work_offset=m.work_offset,
                )
                actual = [x for c in chunks for line in c.text.splitlines() for x in oracle.read(line)]
                self.assertTrue(oracle.ended)
                self.assertEqual(
                    [(x.tool, x.curve, x.feed_mm_min) for x in actual],
                    [(x.tool, x.curve, x.feed_mm_min) for x in expected],
                )
        return actual

    def test_milling_H_mapping_both_tool_change_modes_and_explicit_startup(self):
        expected = [
            'G290',
            'COMPOF',
            'RNDM=0',
            'CFTCP',
            'G601',
            'G291',
            'G21',
            'G90',
            'G17',
            'G94',
            'G40',
            'G49',
            'G80',
            'G50',
            'G69',
            'G50.1 X0. Y0. Z0.',
            'G92.1 X0. Y0. Z0.',
            'G61',
            'M5',
            'M9',
        ]
        for mode in ('m6', 'direct_t'):
            for md in (0, 4, 64, 68):
                p, m = self.machine(tool_change=mode, extern_toolprog_mode=md)
                chunks = list(post_iso(p, m))
                self.assertEqual(chunks[0].text.splitlines(), expected)
                text = ''.join(c.text for c in chunks)
                self.assertIn('T2\nM6\nG43 H7\n' if mode == 'm6' else 'T2\nG43 H7\n', text)
                self.assertTrue(text.endswith('M5\nM9\nG49\nM30\n'))
                self.assertNotIn('D7', text)
                self.oracle(p, m, chunks)

    def test_units_curves_helices_full_circle_and_nonzero_work_offset(self):
        for units in ('mm', 'inch'):
            p, m = self.machine(output_units=units, curve_policy='exact_ijk', work_offset=2)
            start = (5.08, 0, 2.54) if units == 'inch' else (5, 0, 3)
            m = replace(m, tools=(replace(m.tools[0], position_after_mm=start),))
            control = (WorkOffset(2), replace(p.events[1], position_after_mm=start), Spindle(1200, 'ccw'))
            for sense in (-1, 1):
                end = (0, sense * start[0], start[2])
                center = (0, 0, start[2])
                lower = (end[0], end[1], start[2] - (0.254 if units == 'inch' else 0.5))
                curves = (
                    Arc(start, end, center, sense * math.pi / 2),
                    Arc(end, end, center, -sense * math.tau),
                    Helix(end, lower, center, sense * math.tau),
                )
                job = replace(
                    p,
                    initial_position_mm=start,
                    events=(*control, *(Motion(i + 1, 'curves', 't1', c, 254) for i, c in enumerate(curves))),
                )
                chunks = list(post_iso(job, m))
                self.assertEqual(sum(len(c.motions) for c in chunks), 3)
                self.oracle(job, m, chunks)
                text = ''.join(c.text for c in chunks)
                self.assertIn('G55\n', text)
                self.assertNotIn('TURN', text)

    def test_complete_literal_golden_file(self):
        p, m = self.machine(curve_policy='exact_ijk')
        curves = (
            Arc((5, 0, 3), (0, 5, 3), (0, 0, 3), math.pi / 2),
            Arc((0, 5, 3), (5, 0, 3), (0, 0, 3), -math.pi / 2),
            Arc((5, 0, 3), (5, 0, 3), (0, 0, 3), math.tau),
            Helix((5, 0, 3), (5, 0, 2), (0, 0, 3), -math.tau),
        )
        job = replace(
            p,
            events=(
                *p.events[:2],
                Spindle(1200),
                *(
                    Motion(i + 1, 'golden', 't1', c, 300 if isinstance(c, Helix) else 1200)
                    for i, c in enumerate(curves)
                ),
            ),
        )
        chunks = list(post_iso(job, m))
        golden = (Path(__file__).resolve().parents[2] / 'fixtures/posts/ISO2016.MPF').read_text()
        self.assertEqual(''.join(c.text for c in chunks), golden)
        self.oracle(job, m, chunks)

    def test_oracle_exposes_undotted_geometric_and_feed_words(self):
        p, m = self.machine(output_units='inch')
        chunks = list(post_iso(p, m))
        text = ''.join(c.text for c in chunks)
        changed = text.replace('X0.3937', 'X3937').replace('F47.2441', 'F472441')
        self.assertNotEqual(text, changed)
        observations = []
        for calculator in (False, True):
            oracle = Oracle(
                tools=m.tools,
                initial_position=p.initial_position_mm,
                coolant_codes=OEM,
                units='inch',
                calculator=calculator,
            )
            observations.append([x for line in changed.splitlines() for x in oracle.read(line)])
        self.assertNotEqual(observations[0][0].curve, observations[1][0].curve)
        self.assertNotEqual(observations[0][0].feed_mm_min, observations[1][0].feed_mm_min)

    def test_two_tools_stops_and_offset_corruption(self):
        p, m = self.machine(optional_stop_enabled=True)
        other = replace(p.tools[0], id='second')
        settings = (*m.tools, ToolSetting.for_tool(other, number=3, length_register=11, position_after_mm=(10, 0, 4)))
        m = replace(m, tools=settings)
        events = (
            *p.events,
            Stop(True),
            ToolChange('second', 3, 11, (10, 0, 4)),
            Spindle(1500),
            Motion(2, 'second', 'second', Line((10, 0, 4), (10, 0, 3)), 300),
        )
        job = replace(p, tools=(*p.tools, other), events=events)
        chunks = list(post_iso(job, m))
        self.oracle(job, m, chunks)
        with self.assertRaises(ValueError):
            list(post_iso(job, replace(m, optional_stop_enabled=False)))

        class WrongOffset(Siemens808DISOPost):
            def on_tool_change(self, event):
                super().on_tool_change(event)
                self._lines[-1] = 'G43 H1'

        with self.assertRaises(ValueError):
            engine = PostEngine(m, WrongOffset, decoder=Siemens808DISODecoder)
            engine.begin(job)
            engine.consume(job.events[:2])

    def test_settings_and_mutated_machine_definitions_reject(self):
        for setting in (
            dict(startup_units='inch'),
            dict(extern_toolprog_mode=1),
            dict(cutting_edge_default=1),
            dict(tool_corr_move_mode=1),
            dict(tool_length_const=18),
            dict(traverse_feed_mm_min=None),
            dict(coolant_codes={'off': 'M30'}),
        ):
            with self.subTest(setting=setting), self.assertRaises(ValueError):
                self.machine(**setting)
        _, m = self.machine()
        with self.assertRaises(ValueError):
            replace(m, dialect=m.dialect.with_commands(rapid='G0'))
        with self.assertRaises(ValueError):
            replace(m, dialect=replace(m.dialect, length_word='D'))

    def test_missing_initialization_and_hostile_modes_reject(self):
        p, m = self.machine()
        begin = Siemens808DISOPost(m)._call('on_begin', p, limit=32)
        for omitted in (
            'COMPOF',
            'RNDM=0',
            'CFTCP',
            'G601',
            'G291',
            'G50',
            'G69',
            'G50.1 X0. Y0. Z0.',
            'G92.1 X0. Y0. Z0.',
            'G61',
            'G49',
            'G80',
        ):

            class Missing(Siemens808DISOPost):
                def on_begin(self, program):
                    super().on_begin(program)
                    self._lines.remove(omitted)

            with self.subTest(omitted=omitted), self.assertRaises(ValueError):
                PostEngine(m, Missing, decoder=Siemens808DISODecoder).begin(p)
        for body in (
            'G20',
            'G21 G90',
            'G290',
            'G291 G90',
            'G64',
            'G91',
            'G18',
            'G51 X1.',
            'G92.1',
            'G92.1 X0. Y0.',
            'G50.1 X1. Y0. Z0.',
            'G43 D7',
            'G0 X10. Y0. Z3.',
            'G1 X10 Y0. Z3. F1200.',
            'G1 X10.0001 Y0. Z3. F1200.',
            'G2 X5. Y0. I-5. J0. TURN=0',
        ):
            decoder = Siemens808DISODecoder(m, p)
            decoder.read(begin)
            with self.subTest(body=body), self.assertRaises(ValueError):
                decoder.read_line(body)

    def test_language_reselection_preserves_state_and_corruption_is_atomic(self):
        p, m = self.machine()
        decoder = Siemens808DISODecoder(m, p)
        post = Siemens808DISOPost(m)
        decoder.read(post._call('on_begin', p, limit=32))
        for event in p.events[:3]:
            decoder.read(post._call('on_' + event.to_dict()['kind'], event))
        state = (decoder.active, decoder.length, decoder.offset, decoder.position)
        decoder.read_line('G291')
        self.assertEqual((decoder.active, decoder.length, decoder.offset, decoder.position), state)

        class Corrupt(Siemens808DISOPost):
            bad = True

            def on_motion(self, motion):
                super().on_motion(motion)
                if self.bad:
                    self._lines[-1] = self._lines[-1].replace('X10.', 'X11.')

        engine = PostEngine(m, Corrupt, decoder=Siemens808DISODecoder)
        engine.begin(p)
        engine.consume(p.events[:4])
        with self.assertRaises(ValueError):
            engine.consume(p.events[4:5])
        engine.post.bad = False
        chunk = engine.consume(p.events[4:5])
        self.assertEqual(chunk.motions[0].curve, p.events[4].curve)

    def test_multiturn_allplane_lowering_traverse_and_cancellation(self):
        p, m = self.machine()
        for plane, (u, v, a) in {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}.items():
            start = p.initial_position_mm
            end = list(start)
            end[a] -= 2
            center = list(start)
            center[u] -= 5
            motion = Motion(42, 'helix', 't1', Helix(start, tuple(end), tuple(center), -4 * math.tau, plane), 1200)
            job = replace(p, events=(*p.events[:3], motion))
            seen = []

            def events():
                for e in job.events:
                    seen.append(e)
                    yield e
                raise AssertionError('source read after cancellation')

            token = Cancellation()
            stream = post_events_iso(job, events(), m, cancel=token)
            next(stream)
            self.assertFalse(seen)
            for _ in range(4):
                chunk = next(stream)
            self.assertEqual(len(seen), 4)
            self.assertEqual(len(chunk.motions), 64)
            self.assertTrue(all(x['source_motion_id'] == 42 for x in chunk.data['mappings']))
            token.cancel()
            with self.assertRaisesRegex(ValueError, 'cancelled'):
                next(stream)
            self.oracle(job, m, list(post_iso(job, m)))
            with self.assertRaisesRegex(ValueError, 'budget'):
                list(post_iso(job, m, lowering=LoweringOptions(max_segments_per_source=4)))
        rapid = replace(p.events[4], kind='rapid', feed_mm_min=None)
        job = replace(p, events=(*p.events[:3], rapid))
        chunks = list(post_iso(job, m))
        self.assertIn('G1 X10. Y0. Z3. F3000.', ''.join(c.text for c in chunks))
        self.oracle(job, m, chunks)

    def test_complete_pocket_actual_stock_and_independent_literal_file(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'examples'))
        try:
            from support.post_pocket import run
        finally:
            sys.path.pop(0)
        captured = {}

        def builder(header, tools):
            _, machine = self.machine()
            machine = replace(machine, tools=tools, frame=header.frame)
            captured.update(machine=machine, header=header)
            return machine

        with tempfile.TemporaryDirectory() as directory:
            nc = Path(directory) / 'ISOPOCKET.MPF'
            result = run(
                nc,
                profile='python',
                lowering=True,
                radial_allowance_mm=0.01,
                post_type=Siemens808DISOPost,
                decoder_type=Siemens808DISODecoder,
                machine_builder=builder,
            )
            machine, header = captured['machine'], captured['header']
            oracle = Oracle(tools=machine.tools, initial_position=header.initial_position_mm, coolant_codes=OEM)
            decoder = Siemens808DISODecoder(machine, header)
            count = 0
            for line in nc.read_text().splitlines():
                observed = oracle.read(line)
                actual = [
                    e
                    for e in decoder.read_line(line, dict(id=count + 1, operation='file', kind='cut'))
                    if isinstance(e, Motion)
                ]
                self.assertEqual(
                    [(e.tool, e.curve, e.feed_mm_min) for e in observed],
                    [(e.tool, e.curve, e.feed_mm_min) for e in actual],
                )
                count += len(observed)
            self.assertTrue(oracle.ended and decoder.ended)
        self.assertEqual(count, result['motions'])
        self.assertEqual(result['source_motions'], 352)
        self.assertEqual(result['posted_scene_stock_counts'], dict(clear=count, violation=0, uncertain=0))
        self.assertTrue(result['posted_material']['complete'])
        self.assertFalse(result['first_posted_motion']['source_finished'])
        self.assertFalse(result['first_posted_motion']['source_native_exited'])


if __name__ == '__main__':
    unittest.main()
