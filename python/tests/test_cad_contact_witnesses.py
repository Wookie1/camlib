"""CAD-backed exclusion points, whole-cell bounds and streamed report binding."""

from array import array
from dataclasses import replace
import math
import os
import unittest

from camlib.cadquery import export_contact_witnesses, export_surface_mesh
from camlib.material import ContactWitnesses, FinishingScope, SurfaceTarget, Verifier
from camlib.simulation import StockSpec, Simulation, VerificationUpdate
from camlib.playback import Player, Cursor


class CADContactWitnesses(unittest.TestCase):
    def test_trimmed_hole_sphere_and_thin_solid_points_are_on_original_surfaces(self):
        import cadquery as cq

        block = cq.Workplane('XY').box(6, 4, 2, centered=(True, True, False)).faces('>Z').workplane().hole(2).val()
        witnesses = export_contact_witnesses(block, spacing_mm=0.4, min_z_mm=1.8)
        self.assertGreater(len(witnesses.points_mm), 100)
        self.assertLess(witnesses.error_mm, 1e-5)
        for x, y, z in witnesses.points_mm:
            eps = witnesses.error_mm
            self.assertGreaterEqual(math.hypot(x, y), 1 - eps)
            self.assertTrue(
                abs(z - 2) <= eps
                or abs(abs(x) - 3) <= eps
                or abs(abs(y) - 2) <= eps
                or abs(math.hypot(x, y) - 1) <= eps
            )
        sphere = cq.Workplane('XY').sphere(3).val()
        points = export_contact_witnesses(sphere, spacing_mm=0.5)
        for p in points.points_mm:
            self.assertLessEqual(abs(math.hypot(*p) - 3), points.error_mm)
        thin = cq.Workplane('XY').box(0.02, 2, 1).val()
        points = export_contact_witnesses(thin, spacing_mm=1)
        for x, y, z in points.points_mm:
            self.assertLessEqual(abs(x), 0.01 + points.error_mm)
            self.assertTrue(min(abs(abs(x) - 0.01), abs(abs(y) - 1), abs(abs(z) - 0.5)) <= points.error_mm)

    def test_points_reduce_access_uncertainty_without_changing_raw_target(self):
        import cadquery as cq

        shape = cq.Workplane('XY').box(6, 6, 2, centered=(True, True, False)).val()
        surface = export_surface_mesh(shape)
        target = SurfaceTarget('plate', '1', surface, 0, finishing=FinishingScope(0, 1, max_refinement_depth=4))
        stock = StockSpec((-0.1, -0.1, 0), (0.1, 0.1, 3), 0.1)
        heights = array('d', [3.0]) * 8
        before = Verifier(target).check_bounds(heights, stock)
        points = export_contact_witnesses(shape, spacing_mm=0.25, min_z_mm=1.9)
        target = replace(target, contact_witnesses=points)
        verifier = Verifier(target)
        after = verifier.check_bounds(heights, stock)
        self.assertTrue(after['complete'] and after['finishing']['complete'])
        self.assertTrue(after['finishing']['inner_contact_available'])
        for field in ('gouge', 'residual', 'stats', 'stock_sha256'):
            self.assertEqual(before[field], after[field])
        self.assertLess(
            after['finishing']['reachable_residual']['volume_mm3'][1],
            before['finishing']['reachable_residual']['volume_mm3'][1] * 0.6,
        )
        for finding in after['finishing']['findings']:
            lo, hi = finding['attainable_height_mm']
            self.assertLessEqual(lo, 2)
            self.assertGreaterEqual(hi, 2)
        old = verifier.digest
        verifier.prepare(replace(target, contact_witnesses=replace(points, error_mm=0.05)))
        self.assertNotEqual(old, verifier.digest)

    def test_point_error_budget_cancellation_and_invalid_source(self):
        import cadquery as cq

        shape = cq.Workplane('XY').box(4, 4, 2).val()
        for settings in (
            dict(spacing_mm=0),
            dict(max_points=1),
            dict(max_candidates=1),
            dict(min_z_mm=100),
            dict(cancel=lambda: True),
        ):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                export_contact_witnesses(shape, **settings)
        with self.assertRaises(ValueError):
            export_contact_witnesses(shape.Faces()[0])
        mixed = cq.Compound.makeCompound([shape, shape.Faces()[0].translate((0, 0, 10))])
        with self.assertRaisesRegex(ValueError, 'outside its solids'):
            export_contact_witnesses(mixed)
        for points, error in (
            ((), 0),
            (((0, 0),), 0),
            (((0, 0, 0),), float('nan')),
            (((0, 0, float('inf')),), 0),
            (((0, 0, 0),), -0.1),
        ):
            with self.assertRaises(ValueError):
                ContactWitnesses(points, error)
        source = [[0, 0, 1]]
        frozen = ContactWitnesses(source)
        source[0][2] = 2
        self.assertEqual(frozen.points_mm, ((0, 0, 1),))

    def test_actual_finishing_stream_and_posted_seek_bind_witness_definition(self):
        from test_cad_surfaces import example

        # Reuse the existing actual curved CAD finishing job. Derive witnesses
        # from its original analytic CAD source, without asserting mesh inclusion.
        module = example()
        job = module.build_job(cad_contacts=True)
        setup = job.setups[0]
        verification = setup.verification
        target = verification.material[0].target
        verifier = Verifier(target)
        with Simulation(
            setup.stock,
            setup.tools,
            revision='witness-stream',
            initial_position_mm=setup.initial_position_mm,
            operations=setup.operations,
            verification=verification,
            executable=os.environ.get('CAMLIB_TEST_ENGINE'),
        ) as sim:
            reports = [
                u.check['report']
                for u in sim.run_operations()
                if isinstance(u, VerificationUpdate) and u.check['kind'] == 'material'
            ]
            self.assertEqual(reports[-1], verifier.check(sim.state))
            self.assertTrue(reports[-1]['finishing']['inner_contact_available'])
            self.assertTrue(reports[-1]['finishing']['complete'])
        with job.post('top', machine=module.machine_for(job), executable=os.environ.get('CAMLIB_TEST_ENGINE')) as run:
            run.wait()
            count = run.status()['available']
            a, b = Player(run, compact=True), Player(run)
            for cursor in (Cursor(count), Cursor(count // 2, 0.37), Cursor(count)):
                a.seek(cursor)
                b.seek(cursor)
                self.assertEqual(verifier.check(a.state), verifier.check(b.state))
            self.assertTrue(verifier.check(a.state)['finishing']['inner_contact_available'])


if __name__ == '__main__':
    unittest.main()
