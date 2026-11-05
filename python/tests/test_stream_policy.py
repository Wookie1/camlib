from camlib.postprocessor import GCodeDialect
from copy import deepcopy
from dataclasses import replace
import math
import os
from pathlib import Path
import json
from tempfile import TemporaryDirectory
import unittest

from camlib.dynamics import MotionPolicy
from camlib.motion import Motion, Line, Arc, Helix
from camlib.post import Machine, PostEngine, GCodePost, post_events, post_program, Cancellation, LoweringOptions
from test_post import fixture


class AuditPost(GCodePost):
    def __init__(self, machine):
        super().__init__(machine)
        self.seen = []
        self.bad = False
        self.cancel_hook = None

    def on_motion(self, motion):
        self.seen.append((self.source_event.feed_mm_min, motion.feed_mm_min))
        if self.cancel_hook is not None:
            self.cancel_hook()
        super().on_motion(replace(motion, feed_mm_min=900) if self.bad else motion)


class StreamPolicyTest(unittest.TestCase):
    def machine(self, setup, **options):
        return Machine(
            'Example mill',
            setup.tools,
            motion_policy=MotionPolicy(100, 6000),
            schedule_post_feeds=True,
            **{
                'dialect': GCodeDialect.linuxcnc(),
                'max_feed_mm_min': 10000,
                'max_spindle_rpm': 30000,
                'planes': ('xy', 'xz', 'yz'),
                'arcs': True,
                'helices': True,
                'max_sweep_rad': 6.283185307179586,
                'max_tool_number': 99999,
                'max_length_register': 99999,
                'tool_changes': True,
                'flood': True,
                'mist': True,
                'tool_change_position_mode': 'table',
                'rapid_mm_min': 6000,
                'output_units': (options).get('output_units', 'mm'),
                'decimals': 3 if (options).get('output_units', 'mm') == 'mm' else 4,
                **(options),
            },
        )

    def test_direct_stream_emits_bounded_prefix_and_keeps_authored_feed_hashes(self):
        p, s = fixture()
        machine = self.machine(s, max_feed_mm_min=300)
        produced = 0

        def events():
            nonlocal produced
            for e in p.events[:4]:
                produced += 1
                yield e
            a = p.initial_position_mm
            for i in range(200):
                b = (a[0] + 1, a[1], a[2])
                produced += 1
                yield Motion(i + 1, 'cut', 't1', Line(a, b), 600)
                a = b

        stream = post_events(replace(p, events=()), events(), machine)
        begin = next(stream)
        self.assertEqual(produced, 0)
        self.assertEqual(begin.data['identity']['abi'], 7)
        first = next(stream)
        self.assertEqual(produced, 64)
        self.assertLess(produced, 204)
        self.assertEqual(first.data['motion_scheduling']['summary']['completed_source_events'], 64)
        summaries = [first.data['motion_scheduling']['summary']]
        max_bytes = len(first.raw)
        for chunk in stream:
            summaries.append(chunk.data['motion_scheduling']['summary'])
            max_bytes = max(max_bytes, len(chunk.raw))
        self.assertEqual(summaries[-1]['completed_source_motions'], 200)
        self.assertEqual(summaries[-1]['limited_source_motions'], 200)
        self.assertLess(max_bytes, 512 * 1024)
        self.assertAlmostEqual(
            summaries[-1]['source_exact_stop_seconds'], summaries[-1]['decoded_exact_stop_seconds'], places=8
        )
        original = replace(
            p, events=(*p.events[:4], Motion(1, 'cut', 't1', Line(p.initial_position_mm, (6, 0, 3)), 600))
        )
        baseline = list(
            post_program(
                original,
                replace(machine, schedule_post_feeds=False, max_feed_mm_min=10000),
            )
        )
        authored = next(m for c in baseline for m in c.data['mappings'] if m['source_motion_id'] == 1)
        record = first.data['motion_scheduling']['records'][0]
        self.assertEqual(
            first.data['mappings'][record['check_index']]['source_event_sha256'], authored['source_event_sha256']
        )
        self.assertEqual(record['authored_feed_mm_min'], 600)
        self.assertLess(record['scheduled_feed_mm_min'], 300)
        self.assertNotEqual(record['scheduled_source_event_sha256'], authored['source_event_sha256'])
        from camlib.posted import PostedRun
        from camlib.simulation import ProtocolError

        data = deepcopy(first.data)
        i = record['check_index']
        PostedRun._validate_link(data, i, data['mappings'][i])
        data['motion_scheduling']['records'][0]['decoded_feed_mm_min'] += 1
        with self.assertRaisesRegex(ProtocolError, 'scheduling source/decoded'):
            PostedRun._validate_link(data, i, data['mappings'][i])

    def test_custom_hook_failure_and_cancellation_do_not_commit_policy_or_decoder(self):
        p, s = fixture()
        machine = self.machine(s, max_feed_mm_min=1000, max_axis_feed_mm_min=(300, 300, 300))
        engine = PostEngine(machine, AuditPost)
        engine.begin(p)
        engine.consume(p.events[:4])
        event = p.events[4]
        before = engine.position_mm
        engine.post.bad = True
        with self.assertRaises(ValueError):
            engine.consume((event,))
        self.assertEqual(engine.position_mm, before)
        self.assertEqual(engine.post.seen, [])
        engine.post.bad = False
        token = Cancellation()
        engine.post.cancel_hook = lambda: token.cancel()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            engine.consume((event,), cancel=token)
        self.assertEqual(engine.position_mm, before)
        self.assertEqual(engine.post.seen, [])
        engine.post.cancel_hook = None
        chunk = engine.consume((event,))
        report = chunk.data['motion_scheduling']
        self.assertEqual(report['summary']['completed_source_motions'], 1)
        self.assertEqual(len(engine.post.seen), 1)
        self.assertEqual(engine.post.seen[0][0], 1200)
        self.assertLess(engine.post.seen[0][1], 300)
        self.assertEqual(report['records'][0]['authored_feed_mm_min'], 1200)
        engine.finish()

    def test_planes_units_pitch_lowering_original_caps_and_traverse_are_attributed(self):
        p, s = fixture()
        for units in ('mm', 'inch'):
            for lower in (None, LoweringOptions()):
                for plane, (u, v, w) in {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}.items():
                    for sign in (-1, 1):
                        for pitch in (0, 2.54):
                            start = (25.4, 0, 0)
                            center = list(start)
                            center[u] -= 2.54
                            end = list(start)
                            end[w] += pitch
                            curve = (Helix if pitch else Arc)(start, tuple(end), tuple(center), sign * math.tau, plane)
                            job = replace(
                                p,
                                initial_position_mm=start,
                                events=(
                                    p.events[0],
                                    replace(p.events[1], position_after_mm=start),
                                    p.events[2],
                                    Motion(1, 'curve', 't1', curve, 6000),
                                ),
                            )
                            machine = self.machine(
                                replace(s, tools=(replace(s.tools[0], position_after_mm=start),)),
                                output_units=units,
                                max_normal_accel_mm_s2=20,
                            )
                            chunks = list(post_program(job, machine, lowering=lower))
                            rows = [r for c in chunks for r in c.data['motion_scheduling']['records']]
                            self.assertTrue(rows)
                            self.assertTrue(all(r['authored_feed_mm_min'] == 6000 for r in rows))
                            feeds = {r['scheduled_feed_mm_min'] for r in rows}
                            self.assertEqual(len(feeds), 1)
                            feed = feeds.pop()
                            radius = 2.54
                            curvature = radius / (radius * radius + (pitch / math.tau) ** 2)
                            self.assertLess((feed / 60) ** 2 * curvature, 20)
                            self.assertEqual(sum(r['completed_source_exact_stop_seconds'] is not None for r in rows), 1)
                            report = chunks[-1].data['motion_scheduling']['summary']
                            self.assertEqual(report['completed_source_motions'], 1)
                            self.assertEqual(report['limited_source_motions'], 1)
                            if lower:
                                self.assertGreater(report['decoded_motions'], 1)
        machine = self.machine(s, max_axis_feed_mm_min=(300, 300, 300), traverse_feed_mm_min=600)
        job = replace(p, events=(*p.events[:4], replace(p.events[4], kind='rapid', feed_mm_min=None)))
        row = next(r for c in post_program(job, machine) for r in c.data['motion_scheduling']['records'])
        self.assertIsNone(row['authored_feed_mm_min'])
        self.assertEqual(row['configured_feed_mm_min'], 600)
        self.assertLess(row['scheduled_feed_mm_min'], 300)
        self.assertIsNotNone(row['decoded_feed_mm_min'])

    def test_partial_lowered_cancel_retry_keeps_pending_fraction_and_summary(self):
        p, s = fixture()
        start = p.initial_position_mm
        end = (start[0] - 4, start[1], start[2])
        curve = Arc(start, end, (start[0] - 2, start[1], start[2]), math.pi)
        machine = self.machine(s, max_normal_accel_mm_s2=10)
        engine = PostEngine(machine, lowering=LoweringOptions())
        engine.begin(p)
        list(engine.consume_events(p.events[:4]))
        engine.start(Motion(1, 'curve', 't1', curve, 6000))
        chunk = engine.drain(2)
        summary = chunk.data['motion_scheduling']['summary']
        self.assertEqual(summary['completed_source_motions'], 0)
        self.assertEqual(summary['decoded_motions'], 2)
        token = Cancellation()
        token.cancel()
        position = engine.position_mm
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            engine.drain(2, cancel=token)
        self.assertEqual(engine.position_mm, position)
        self.assertTrue(engine.pending)
        while engine.pending:
            chunk = engine.drain(2)
        summary = chunk.data['motion_scheduling']['summary']
        self.assertEqual(summary['completed_source_motions'], 1)
        self.assertEqual(summary['limited_source_motions'], 1)
        engine.finish()

    def test_actual_cad_rest_stream_cache_policy_revision_posted_stock_and_seek(self):
        from test_rest_job import example

        job = example().build_rest_job()
        machine = example().rest_machine_for(job)
        self.check_job(job, machine)

    def test_complete_connected_passage_streamed_policy_preserves_posted_stock(self):
        from camlib.jobs import Job, Setup
        from camlib.operations import AdaptivePocket
        from camlib.simulation import StockSpec, SweepOptions
        from camlib.tools import ToolAssembly
        from camlib.post import ToolSetting
        from camlib.verification import StreamChecks, StockContactOptions

        m = json.loads(
            (Path(__file__).resolve().parents[2] / 'fixtures/operations/adaptive-connected.json').read_text()
        )
        tool = ToolAssembly.from_dict(m['tools'][0])
        definition = dict(m['operations'][0])
        definition.pop('kind')
        definition.update(tool=tool, path_strategy='stock_front')
        stock = StockSpec(**{k: v for k, v in m['stock'].items() if k not in ('version', 'units')})
        setup = Setup(
            'top',
            stock,
            (tool,),
            (
                AdaptivePocket(
                    **{
                        'feed_mm_min': 600,
                        'plunge_mm_min': 150,
                        'spindle_rpm': 12000,
                        'tool_number': 1,
                        'length_register': 1,
                        'coolant': "flood",
                        **definition,
                    }
                ),
            ),
            tuple(m['initial_position_mm']),
            sweep=SweepOptions(**m['sweep']),
            verification=StreamChecks(stock_contact=StockContactOptions()),
        )
        job = Job('stream-policy', 'r1', (setup,))
        machine = Machine(
            'Example mill',
            (ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=setup.initial_position_mm),),
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
        self.check_job(job, machine, repeat_policy=False)

    def check_job(self, job, machine, repeat_policy=True):
        from camlib.jobs import RegenerationCache
        from camlib.playback import Player, Cursor
        from camlib.simulation import OperationUpdate

        policies = [
            machine,
            replace(machine, motion_policy=MotionPolicy(200, 6000), schedule_post_feeds=True, max_feed_mm_min=300),
        ]
        if repeat_policy:
            policies.append(replace(policies[-1], motion_policy=MotionPolicy(300, 6000)))
        saved = None
        policy_ids = []
        prefixes = []
        times = []
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            for index, configured in enumerate(policies):
                reports = []

                def observe(update):
                    if isinstance(update, OperationUpdate) and update.stage == 'end':
                        reports.append(update.report)

                with job.post(
                    'top',
                    machine=configured,
                    cache=cache,
                    executable=os.environ.get('CAMLIB_TEST_ENGINE'),
                    checkpoint_every=16,
                    on_operation=observe,
                ) as run:
                    run.wait(timeout=180)
                    status = run.posting_status()
                    self.assertTrue(status['finished'])
                    self.assertLess(status['max_chunk_bytes'], 512 * 1024)
                    player = Player(run)
                    last = Cursor(run.status()['available'])
                    player.seek(last)
                    self.assertEqual(player.verification['progress']['violation'], 0)
                    self.assertEqual(player.verification['progress']['uncertain'], 0)
                    stock = player.state.heights.tobytes()
                    if saved is None:
                        saved = stock
                    else:
                        self.assertEqual(stock, saved)
                    self.assertTrue(reports)
                    self.assertTrue(all(r['regeneration']['cache_hit'] == (index > 0) for r in reports))
                    if index:
                        summary = status['motion_scheduling']
                        policy_ids.append(summary['policy_sha256'])
                        prefixes.append(summary['authored_prefix_sha256'])
                        times.append(summary['source_exact_stop_seconds'])
                        self.assertGreater(summary['limited_source_motions'], 0)
                        self.assertEqual(summary['decoded_motions'], last.completed)
                        record = next(
                            run.posted_motion(i)['motion_scheduling']
                            for i in range(last.completed)
                            if run.posted_motion(i)['motion_scheduling']['scheduled_feed_mm_min'] is not None
                        )
                        self.assertIsNotNone(record['authored_feed_mm_min'])
                        at = Cursor(last.completed // 2, 0.5)
                        player.seek(at)
                        partial = player.state.heights.tobytes()
                        player.seek(last)
                        player.seek(at)
                        self.assertEqual(player.state.heights.tobytes(), partial)
            if repeat_policy:
                self.assertNotEqual(*policy_ids)
                self.assertEqual(*prefixes)
                self.assertGreater(times[0], times[1])

    def test_stationary_posted_interval_keeps_source_time_without_fake_decoded_motion(self):
        p, s = fixture()
        a = p.initial_position_mm
        b = (a[0] + 0.0001, a[1], a[2])
        p = replace(p, events=(*p.events[:4], Motion(1, 'short', 't1', Line(a, b), 600)))
        machine = self.machine(s, max_feed_mm_min=300)
        chunks = list(post_program(p, machine))
        summary = chunks[-1].data['motion_scheduling']['summary']
        self.assertEqual(summary['completed_source_motions'], 1)
        self.assertEqual(summary['decoded_motions'], 0)
        self.assertGreater(summary['source_exact_stop_seconds'], 0)
        self.assertEqual(summary['decoded_exact_stop_seconds'], 0)
        row = next(r for c in chunks for r in c.data['motion_scheduling']['records'])
        self.assertEqual(row['authored_feed_mm_min'], 600)
        self.assertIsNone(row['decoded_feed_mm_min'])
