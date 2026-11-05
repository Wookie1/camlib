from camlib.postprocessor import GCodeDialect
from dataclasses import replace
from functools import lru_cache
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib.jobs import OperationNode, RegenerationCache
from camlib.simulation import OperationUpdate, VerificationUpdate, StockState, ProtocolError
from camlib.playback import Player, Cursor
from camlib.verification import StreamChecks, Scene, MachineLimits
from camlib.verification import MaterialCheck
from camlib.material import Target
from camlib.post import Machine, ToolSetting


@lru_cache(None)
def example():
    path = Path(__file__).resolve().parents[2] / "python/tests/support/incremental_job.py"
    spec = importlib.util.spec_from_file_location("incremental_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def collect(job, cache):
    ends = {}
    frames = {}
    manifests = {}
    checks = []
    streamed = False
    with job.regenerate(cache) as run:
        for event in run:
            update = event.update
            sim = run._simulation
            manifests.setdefault(event.setup, sim.manifest_frame)
            if isinstance(update, OperationUpdate):
                if update.stage == 'end':
                    ends[(event.setup, update.operation)] = update.report
            elif isinstance(update, VerificationUpdate):
                checks.append(update.check)
            else:
                frames.setdefault(event.setup, update.frame)
                if sim.state.volume()['removed_lower_mm3'] > 0.001:
                    streamed = True
                    # A fast worker may have exited while its bounded pipe still
                    # contains frames. Material must reach the consumer before
                    # the job reports completion; worker liveness is unrelated.
                    assert not run.finished
        assert run.finished
        return dict(
            states=run.states,
            statistics=run.statistics,
            ends=ends,
            frames=frames,
            manifests=manifests,
            checks=checks,
            streamed=streamed,
        )


class JobTest(unittest.TestCase):
    def test_real_cadquery_revision_cad_change_independence_and_fresh_stock_identity(self):
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            base = example().build_job()
            cold = collect(base, cache)
            self.assertTrue(cold['streamed'])
            self.assertTrue(all(not s['cache_hit'] for s in cold['statistics'].values()))
            new = example().build_job(document_revision=2)
            warm = collect(new, cache)
            self.assertTrue(
                all(
                    s['cache_hit'] and s['computed_sweeps'] == s['planned_motions'] == 0
                    for s in warm['statistics'].values()
                )
            )
            for id in cold['states']:
                self.assertEqual(cold['states'][id].heights.tobytes(), warm['states'][id].heights.tobytes())
            fresh = StockState(warm['manifests']['main'])
            with self.assertRaises(ProtocolError):
                fresh.accept(cold['frames']['main'])
            self.assertEqual(fresh.completed, 0)
            residual = warm['ends'][('main', 'finish')]['geometric_residual']
            self.assertEqual(residual['source']['job_sources']['main-part']['revision'], '2')
            changed = example().build_job(12, document_revision=3)
            result = collect(changed, cache)
            self.assertTrue(result['statistics'][('main', 'face')]['cache_hit'])
            self.assertTrue(result['statistics'][('secondary', 'face')]['cache_hit'])
            self.assertFalse(result['statistics'][('main', 'rough')]['cache_hit'])
            self.assertFalse(result['statistics'][('main', 'finish')]['cache_hit'])
            reference = collect(changed, None)
            for id in reference['states']:
                self.assertEqual(reference['states'][id].heights.tobytes(), result['states'][id].heights.tobytes())
            compiled = base.compile()
            view = compiled.setups[0].config
            view['operations'][0]['feed_mm_min'] = 12
            self.assertEqual(compiled.setups[0].config['operations'][0]['feed_mm_min'], 600)
            header = compiled.setups[0].program_header()
            self.assertEqual(header.to_dict()['tools'][0]['id'], 't1')

    def test_feed_tool_upstream_stock_and_explicit_dependency_invalidation(self):
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            base = example().build_job()
            collect(base, cache)
            feed = collect(example().build_job(document_revision=2, feed=700), cache)['statistics']
            self.assertTrue(feed[('main', 'face')]['cache_hit'])
            self.assertFalse(feed[('main', 'rough')]['cache_hit'])
            self.assertTrue(feed[('main', 'finish')]['cache_hit'])
            self.assertTrue(feed[('secondary', 'face')]['cache_hit'])
            tool = collect(example().build_job(document_revision=3, diameter=5), cache)['statistics']
            self.assertTrue(all(not tool[('main', op)]['cache_hit'] for op in ['face', 'rough', 'finish']))
            self.assertTrue(tool[('secondary', 'face')]['cache_hit'])
            upstream = example().build_job(document_revision=4, rough_floor=4.5)
            result = collect(upstream, cache)
            self.assertFalse(result['statistics'][('main', 'rough')]['cache_hit'])
            self.assertFalse(result['statistics'][('main', 'finish')]['cache_hit'])
            ref = collect(upstream, None)
            self.assertEqual(result['states']['main'].heights.tobytes(), ref['states']['main'].heights.tobytes())

            def explicit(job):
                setup = job.setups[0]
                nodes = (*setup.operations[:2], OperationNode(setup.operations[2], depends_on=('rough',)))
                return replace(job, setups=(replace(setup, operations=nodes), *job.setups[1:]))

            collect(explicit(base), cache)
            dependent = collect(explicit(example().build_job(document_revision=5, feed=750)), cache)
            self.assertFalse(dependent['statistics'][('main', 'finish')]['cache_hit'])
            setup = base.setups[0]
            bad = replace(
                setup,
                operations=(
                    OperationNode(setup.operations[0].operation, depends_on=('finish',)),
                    *setup.operations[1:],
                ),
            )
            with self.assertRaisesRegex(ValueError, 'preceding'):
                replace(base, setups=(bad,)).compile()

    def test_fresh_verification_and_retained_partial_seek_on_warm_run(self):
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            base = example().build_job()

            def checked(job, revision, feed):
                region = job.setups[0].operations[1].region
                checks = StreamChecks(
                    Scene('machine-limits', revision, machine=MachineLimits((-100, -100, -100), (100, 100, 100), feed)),
                    material=(MaterialCheck(Target('pocket', revision, region, 3)),),
                )
                return replace(job, setups=(replace(job.setups[0], verification=checks),))

            collect(checked(base, '1', 1000), cache)  # cold run fills the cache
            new = checked(example().build_job(document_revision=2), '2', 100)
            warm = collect(new, cache)
            self.assertTrue(all(s['cache_hit'] for s in warm['statistics'].values()))
            statuses = [r['report']['status'] for r in warm['checks'] if r['kind'] == 'motion']
            self.assertIn('violation', statuses)
            self.assertEqual(warm['states']['main'].manifest['verification']['scene']['scene']['revision'], '2')
            material = [r['report'] for r in warm['checks'] if r['kind'] == 'material']
            self.assertEqual(len(material), 1)
            self.assertEqual(material[0]['context']['run']['run']['revision'], new.compile().revision)
            self.assertEqual(material[0]['context']['generation'], warm['states']['main'].completed)
            with new.record('main', cache, checkpoint_every=8, rapid_mm_min=6000) as run:
                run.wait(timeout=60)
                player = Player(run)
                partial = None
                for i in range(run.status()['available']):
                    player.seek(Cursor(i))
                    before = player.state.heights.tobytes()
                    player.seek(Cursor(i, 0.5))
                    if player.state.heights.tobytes() != before:
                        partial = Cursor(i, 0.5)
                        break
                self.assertIsNotNone(partial)
                saved = player.state.heights.tobytes()
                player.seek(Cursor(run.status()['available']))
                self.assertTrue(player.state.finished)
                player.seek(Cursor())
                player.seek(partial)
                self.assertEqual(player.state.heights.tobytes(), saved)
                self.assertEqual(player.state.manifest['run']['revision'], new.compile().revision)

    def test_normal_python_machine_posts_regenerated_operations_and_checks_actual_stock(self):
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            base = example().build_job()
            collect(base, cache)
            job = example().build_job(document_revision=2)
            setup = job.compile().setup('main')
            header = setup.program_header()
            machine = Machine(
                'Job test mill',
                (
                    ToolSetting.for_tool(
                        header.tools[0], number=1, length_register=1, position_after_mm=header.initial_position_mm
                    ),
                ),
                frame=header.frame,
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
            with job.post('main', machine=machine, cache=cache, checkpoint_every=8) as run:
                run.wait(timeout=60)
                self.assertTrue(run.posting_status()['finished'])
                self.assertGreater(run.status()['available'], 0)
                self.assertEqual(run.posting_manifest()['identity']['revision'], header.revision)
                lines = run.nc_lines(1, limit=256)
                self.assertTrue(any('G1' in row['text'] for row in lines))
                self.assertEqual(sum(row['text'] == 'M6' for row in lines), 1)
                record = run.posted_motion(0)
                self.assertTrue(record['stock_committed'])
                self.assertIsNotNone(record['mapping']['source_motion_id'])
                player = Player(run)
                player.seek(Cursor(run.status()['available']))
                self.assertGreater(player.state.volume()['removed_lower_mm3'], 0)
