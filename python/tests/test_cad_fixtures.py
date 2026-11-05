from dataclasses import replace
import io
import math
import unittest

import cadquery as cq
from OCP.BRepTools import BRepTools
from OCP.TopTools import TopTools_FormatVersion

from camlib.cadquery import export_fixture, FixtureLimits
from camlib.motion import WorkFrame, Motion, Line
from camlib.verification import Scene, Verifier, StreamChecks
from camlib.simulation import Simulation, StockSpec, VerificationStream, ProtocolError
from camlib.playback import RecordedRun, Player, Cursor
from test_simulation import tool
from test_stream_verification import modified


def source_bytes(shape):
    data = io.BytesIO()
    BRepTools.Write_s(shape.wrapped, data, False, False, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)
    return data.getvalue()


def clamp():
    return cq.Workplane('XY').box(12, 10, 8).faces('>Z').workplane().hole(6).val()


class CadFixtureTest(unittest.TestCase):
    def test_through_hole_prism_retains_analytic_boundary_and_source_without_mutation(self):
        shape = clamp()
        before = source_bytes(shape)
        body = export_fixture(shape, id='clamp', revision='doc-1', source={'body_reference': 'clamp-body'})
        self.assertEqual(body.representation, 'solid_prism')
        self.assertEqual((body.z_min_mm, body.z_max_mm), (-4, 4))
        self.assertEqual(len(body.footprint.components[0].holes), 1)
        self.assertEqual(len(body.footprint.components[0].holes[0].vertices), 2)
        self.assertLess(body.z_error_mm, 1e-5)
        metadata = body.footprint.source['cad_body']
        self.assertEqual(metadata['source_reference']['body_reference'], 'clamp-body')
        self.assertEqual(len(metadata['brep_sha256']), 64)
        self.assertLess(metadata['hash_max_chunk_bytes'], 1024 * 1024)
        self.assertNotIn('extraction_ms', body.footprint.source)
        self.assertEqual(before, source_bytes(shape))
        with Verifier(Scene('fixtures', '1', (body,))) as verifier:
            # A stationary XY axis through the radius3 hole accommodates a
            # radius2 cutter continuously while moving in Z.
            clear = verifier.check(tool(), Motion(1, 'test', 't1', Line((0, 0, -6), (0, 0, -5)), 100))
            self.assertEqual(clear['status'], 'clear')
            hit = verifier.check(tool(), Motion(2, 'test', 't1', Line((-12, 4, 0), (12, 4, 0)), 100))
            self.assertEqual(hit['status'], 'violation')
            self.assertEqual(hit['findings'][0]['object_id'], 'clamp')
            self.assertGreater(hit['findings'][0]['witness']['fraction'], 0)
            self.assertLess(hit['findings'][0]['witness']['fraction'], 1)

    def test_fingerprint_and_scene_identity_ignore_mesh_caches_but_detect_changed_geometry(self):
        shape = clamp()
        body = export_fixture(shape, id='clamp', revision='same')
        shape.tessellate(0.1)
        after = export_fixture(shape, id='clamp', revision='same')
        self.assertEqual(body.to_dict(), after.to_dict())
        with Verifier(Scene('scene', 'same', (body,))) as verifier:
            old = verifier._identity['scene_sha256']
            verifier.prepare(Scene('scene', 'same', (after,)))
            self.assertEqual(old, verifier._identity['scene_sha256'])
            changed = export_fixture(clamp().translate((1, 0, 0)), id='clamp', revision='same')
            self.assertNotEqual(
                body.footprint.source['cad_body']['brep_sha256'], changed.footprint.source['cad_body']['brep_sha256']
            )
            verifier.prepare(Scene('scene', 'same', (changed,)))
            self.assertNotEqual(old, verifier._identity['scene_sha256'])

    def test_rigid_work_frame_accounts_for_placed_cad_geometry_and_rejects_scale_reflection(self):
        shape = cq.Workplane('XY').box(4, 6, 8).val()
        angle = math.pi / 6
        frame = WorkFrame(
            (100, 200, 300), (1, 0, 0), (0, math.cos(angle), math.sin(angle)), (0, -math.sin(angle), math.cos(angle))
        )
        placed = shape.rotate((0, 0, 0), (1, 0, 0), 30).translate(frame.origin_mm)
        original = source_bytes(placed)
        body = export_fixture(placed, id='placed', revision='1', work_frame=frame)
        self.assertAlmostEqual(body.z_min_mm, -4)
        self.assertAlmostEqual(body.z_max_mm, 4)
        self.assertEqual(source_bytes(placed), original)
        with Verifier(Scene('frame', '1', (body,))) as verifier:
            self.assertEqual(
                verifier.check(tool(), Motion(1, 'test', 't1', Line((-10, 0, 0), (10, 0, 0)), 100))['status'],
                'violation',
            )
            self.assertEqual(
                verifier.check(tool(), Motion(2, 'test', 't1', Line((-10, 10, 0), (10, 10, 0)), 100))['status'], 'clear'
            )
        for bad in [
            replace(frame, x=(2, 0, 0)),
            replace(frame, z=tuple(-x for x in frame.z)),
            replace(frame, origin_mm=(math.nan, 0, 0)),
        ]:
            with self.assertRaises(ValueError):
                export_fixture(placed, id='bad', revision='1', work_frame=bad)

    def test_unsupported_surfaces_steps_blind_holes_and_mixed_containers_reject(self):
        shapes = [
            cq.Workplane('XY').sphere(5).val(),
            cq.Solid.makeCone(5, 3, 8),
            cq.Workplane('XY').box(10, 12, 8).faces('>Z').workplane().hole(2, 3).val(),
            cq.Workplane('XY').ellipse(5, 3).extrude(8).val(),
            cq.Workplane('XY').box(4, 6, 8).val().rotate((0, 0, 0), (1, 0, 0), 15),
        ]
        for shape in shapes:
            with self.subTest(shape=shape.ShapeType()):
                with self.assertRaisesRegex(ValueError, 'supported solid prism'):
                    export_fixture(shape, id='unsupported', revision='1')
        s = cq.Workplane('XY').box(1, 2, 3).val()
        for invalid in [
            s.Faces()[0],
            cq.Compound.makeCompound([s, s.translate((5, 0, 0))]),
            cq.Compound.makeCompound([s, s.Edges()[0]]),
        ]:
            with self.assertRaisesRegex(ValueError, 'one CAD solid'):
                export_fixture(invalid, id='bad', revision='1')

    def test_explicit_general_enclosures_never_claim_source_collision_and_coverage_is_visible(self):
        sphere = cq.Workplane('XY').sphere(5).val()
        body = export_fixture(sphere, id='sphere', revision='1', unsupported='enclose')
        self.assertEqual(body.representation, 'outer_envelope')
        self.assertIn('unsupported side surface', body.footprint.source['cad_body']['unsupported_reason'])
        with Verifier(Scene('enclosure', '1', (body,))) as verifier:
            r = verifier.check(tool(), Motion(1, 'test', 't1', Line((-10, 0, 0), (10, 0, 0)), 100))
            self.assertEqual(r['status'], 'uncertain')
            self.assertEqual(r['coverage']['enclosure_only_bodies'], 1)
            self.assertTrue(all(f['witness'] is None and f['status'] == 'uncertain' for f in r['findings']))
            self.assertEqual(
                verifier.check(tool(), Motion(2, 'test', 't1', Line((-10, 10, 0), (10, 10, 0)), 100))['status'], 'clear'
            )
        motions = [Motion(1, 'test', 't1', Line((-10, 0, 0), (10, 0, 0)), 100)]
        config = dict(
            stock=StockSpec((-12, -2, -10), (12, 2, -9), 0.5),
            tools=(tool(),),
            revision='enclosure-1',
            initial_position_mm=(-10, 0, 0),
            verification=StreamChecks(Scene('scene', '1', (body,))),
        )
        with Simulation(**config) as sim:
            motion, update = list(sim.motions(motions))
            reader = VerificationStream(sim.manifest_frame)
            self.assertEqual(update.status, 'uncertain')
            bad = modified(update.frame, lambda d: d['check']['report']['coverage'].update(enclosure_only_bodies=0))
            with self.assertRaises(ProtocolError):
                reader.read(bad, 1, motion.motion)

            def forge(d):
                r = d['check']['report']
                r['status'] = 'violation'
                r['findings'][0]['status'] = 'violation'

            with self.assertRaises(ProtocolError):
                reader.read(modified(update.frame, forge), 1, motion.motion)
            reader.read(update.frame, 1, motion.motion)
            sim.finish()
        with RecordedRun(**{'rapid_mm_min': 6000, **config}, motions=motions) as run:
            run.wait()
            player = Player(run)
            self.assertEqual(player.verification['coverage']['enclosure_only_bodies'], 1)
            player.seek(Cursor(1))
            self.assertEqual(player.verification['progress']['uncertain'], 1)
            self.assertTrue(all(s.get('status') == 'uncertain' for s in player.verification['samples']))

    def test_limits_cancellation_and_invalid_identity_fail_without_source_changes(self):
        shape = clamp()
        before = source_bytes(shape)
        for limits in [
            FixtureLimits(max_faces=1),
            FixtureLimits(max_brep_bytes=32),
            FixtureLimits(max_body_bytes=32),
            FixtureLimits(max_cad_tolerance_mm=1e-9),
        ]:
            with self.assertRaises(ValueError):
                export_fixture(shape, id='clamp', revision='1', limits=limits)
            self.assertEqual(before, source_bytes(shape))
        calls = 0

        def cancel():
            nonlocal calls
            calls += 1
            return calls > 5

        with self.assertRaisesRegex(ValueError, 'cancelled'):
            export_fixture(shape, id='clamp', revision='1', cancel=cancel)
        for kwargs in [
            dict(id='bad id', revision='1'),
            dict(id='ok', revision=''),
            dict(id='ok', revision='1', unsupported='guess'),
        ]:
            with self.assertRaises(ValueError):
                export_fixture(shape, **kwargs)
        self.assertEqual(before, source_bytes(shape))


if __name__ == '__main__':
    unittest.main()
