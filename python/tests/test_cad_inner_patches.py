"""Whole-footprint CAD source bounds, independent of tessellation inclusion."""

from array import array
from dataclasses import replace
import math
import os
import unittest

from camlib.cadquery import export_inner_patches, export_surface_mesh
from camlib.material import SurfaceTarget, Verifier, Options, FinishingScope
from camlib.motion import WorkFrame
from camlib.simulation import StockSpec, Simulation, VerificationUpdate
from camlib.playback import Player, Cursor


class CADInnerPatches(unittest.TestCase):
    @staticmethod
    def report(shape, stock, height, *, base=0, finishing=None):
        target = SurfaceTarget(
            'part',
            '1',
            export_surface_mesh(shape),
            base,
            finishing=finishing,
            inner_patches=export_inner_patches(shape),
        )
        nx = math.ceil((stock.max_mm[0] - stock.min_mm[0]) / stock.cell_size_mm)
        ny = math.ceil((stock.max_mm[1] - stock.min_mm[1]) / stock.cell_size_mm)
        heights = array('d', [height, height]) * (nx * ny)
        return target, heights, Verifier(target).check_bounds(heights, stock)

    def test_hole_full_footprints_reduce_raw_uncertainty_without_filling_hole(self):
        import cadquery as cq

        shape = cq.Workplane('XY').box(6, 6, 2, centered=(True, True, False)).faces('>Z').workplane().hole(1.5).val()
        stock = StockSpec((-2, -2, 0), (2, 2, 3), 0.25)
        target, heights, after = self.report(shape, stock, 3)
        before = Verifier(replace(target, inner_patches=())).check_bounds(heights, stock)
        self.assertTrue(after['complete'])
        self.assertEqual(after['stock_sha256'], before['stock_sha256'])
        self.assertLess(after['residual']['volume_mm3'][1], before['residual']['volume_mm3'][1])
        self.assertEqual(len(target.inner_patches), 2)
        self.assertTrue(all(len(p.region.components[0].holes) == 1 for p in target.inner_patches))
        proven, empty = 0, 0
        for f in after['findings']:
            x0, y0, x1, y1 = f['xy_mm']
            lo, hi = f['target_height_mm']
            near = math.hypot(max(x0, 0, -x1), max(y0, 0, -y1))
            far = max(math.hypot(x, y) for x in (x0, x1) for y in (y0, y1))
            self.assertLessEqual(lo, 2)
            if near > 0.75 + 0.2:
                self.assertGreater(lo, 1.999)
                self.assertGreaterEqual(hi, 2)
                proven += 1
            if far < 0.75 - 0.01:
                self.assertEqual(lo, 0)
                self.assertLess(hi, 0.01)
                empty += 1
            if near < 0.75:
                self.assertEqual(lo, 0, 'mixed/hole footprint must not acquire a solid roof')
        self.assertGreater(proven, 100)
        self.assertGreater(empty, 4)

    def test_tilted_translated_solid_bounds_every_cell_corner(self):
        import cadquery as cq

        angle = math.radians(25)
        for shift in (0, 100000):
            shape = (
                cq.Workplane('XY')
                .box(8, 8, 2, centered=(True, True, False))
                .rotate((0, 0, 0), (0, 1, 0), 25)
                .translate((shift, -shift, 0))
                .val()
            )
            stock = StockSpec((shift - 1, -shift - 1, 0), (shift + 1, -shift + 1, 4), 0.25)
            target, heights, report = self.report(shape, stock, 4, base=-4)
            self.assertTrue(report['complete'])
            self.assertEqual(len(target.inner_patches), 4)
            for f in report['findings']:
                x0, _, x1, _ = f['xy_mm']
                exact = [2 / math.cos(angle) - math.tan(angle) * (x - shift) for x in (x0, x1)]
                lo, hi = f['target_height_mm']
                self.assertLessEqual(lo, min(exact))
                self.assertGreater(lo, min(exact) - 0.001)
                self.assertGreaterEqual(hi, max(exact))
            old = Verifier(target).digest
            patch = target.inner_patches[0]
            changed = replace(patch, normal_error_mm=patch.normal_error_mm + 0.001)
            self.assertNotEqual(
                old, Verifier(replace(target, inner_patches=(changed, *target.inner_patches[1:]))).digest
            )

    def test_thin_concave_and_curved_trims_remain_conservative(self):
        import cadquery as cq

        shapes = [
            cq.Workplane('XY').box(0.02, 2, 1, centered=(True, True, False)).val(),
            cq.Workplane('XY').polyline([(-2, -2), (2, -2), (2, 0), (0, 0), (0, 2), (-2, 2)]).close().extrude(1).val(),
            cq.Workplane('XY').ellipse(2, 1).extrude(1).val(),
        ]
        from OCP.BRepClass import BRepClass_FaceClassifier
        from OCP.gp import gp_Pnt
        from OCP.TopAbs import TopAbs_IN, TopAbs_ON

        for kind, shape in enumerate(shapes):
            patches = export_inner_patches(shape)
            self.assertTrue(patches)
            target = SurfaceTarget('shape', '1', export_surface_mesh(shape), 0, inner_patches=patches)
            stock = StockSpec((-2.5, -2.5, 0), (2.5, 2.5, 2), 0.25)
            report = Verifier(target).check_bounds(array('d', [2, 2]) * 400, stock)
            top = max((f for f in shape.Faces() if f.geomType() == 'PLANE'), key=lambda f: f.Center().z)
            for f in report['findings']:
                if f['target_height_mm'][0] > 0.5:
                    x0, y0, x1, y1 = f['xy_mm']
                    # Independent complete-box conditions: these extrema prove
                    # inclusion over the continuum, not just sampled points.
                    if kind == 0:
                        self.assertTrue(-0.01 <= x0 <= x1 <= 0.01 and -1 <= y0 <= y1 <= 1)
                    elif kind == 1:
                        self.assertTrue(-2 <= x0 <= x1 <= 2 and -2 <= y0 <= y1 <= 2)
                        self.assertTrue(x1 <= 0 or y1 <= 0)
                    else:
                        self.assertLessEqual(max(x * x / 4 + y * y for x in (x0, x1) for y in (y0, y1)), 1)
                    for x in (x0, (x0 + x1) / 2, x1):
                        for y in (y0, (y0 + y1) / 2, y1):
                            self.assertIn(
                                BRepClass_FaceClassifier(top.wrapped, gp_Pnt(x, y, 1), 1e-8).State(),
                                (TopAbs_IN, TopAbs_ON),
                            )
            if shape.BoundingBox().xlen < 0.1:
                self.assertTrue(all(f['target_height_mm'][0] == 0 for f in report['findings']))

    def test_limits_cancellation_foreign_faces_and_contradictory_source(self):
        import cadquery as cq

        shape = cq.Workplane('XY').box(6, 6, 2, centered=(True, True, False)).val()
        for settings in (
            dict(max_patches=1),
            dict(max_segments=4),
            dict(cancel=lambda: True),
            dict(max_error_mm=0),
            dict(max_patches=0),
        ):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                export_inner_patches(shape, **settings)
        with self.assertRaisesRegex(ValueError, 'outside its solids'):
            export_inner_patches(cq.Compound.makeCompound([shape, shape.Faces()[0].translate((0, 0, 10))]))
        self.assertEqual(export_inner_patches(cq.Workplane('XY').sphere(1)), ())
        stock = StockSpec((-1, -1, 0), (1, 1, 3), 0.5)
        target, heights, _ = self.report(shape, stock, 3)
        patch = max(target.inner_patches, key=lambda p: p.frame.origin_mm[2])
        raised = replace(patch, frame=replace(patch.frame, origin_mm=(0, 0, 10)))
        with self.assertRaisesRegex(ValueError, 'contradicts'):
            Verifier(replace(target, inner_patches=(raised,))).check_bounds(heights, stock)
        invalid = replace(patch, frame=WorkFrame(x=(0, 0, -1), z=(1, 0, 0)))
        with self.assertRaisesRegex(ValueError, 'nonvertical'):
            Verifier(replace(target, inner_patches=(invalid,)))
        limited = Verifier(target, Options(max_boundary_tests=1)).check_bounds(heights, stock)
        self.assertFalse(limited['complete'])

    def test_reflected_cad_plane_axes_from_chamfers_keep_a_proper_frame(self):
        import cadquery as cq
        from OCP.BRepAdaptor import BRepAdaptor_Surface

        shape = cq.Workplane('XY').box(6, 4, 2, centered=(True, True, False)).edges('<Z').chamfer(0.3).val()
        source_planes = [BRepAdaptor_Surface(f.wrapped).Plane() for f in shape.Faces() if f.geomType() == 'PLANE']
        self.assertTrue(any(not p.Position().Direct() for p in source_planes))
        patches = export_inner_patches(shape)
        self.assertEqual(len(patches), 6)
        surface = export_surface_mesh(shape)
        base = min(p[2] for tri in surface.triangles for p in tri)
        target = SurfaceTarget('bevel', '1', surface, base, inner_patches=patches)
        stock = StockSpec((-1, -1, 0), (1, 1, 3), 0.5)
        report = Verifier(target).check_bounds(array('d', [3, 3]) * 16, stock)
        self.assertTrue(report['complete'])
        self.assertTrue(all(1.999 < f['target_height_mm'][0] <= 2 for f in report['findings']))

    def test_allowance_on_source_below_stock_does_not_invent_protection(self):
        import cadquery as cq

        shape = cq.Workplane('XY').box(6, 6, 1, centered=(True, True, False)).translate((0, 0, -2)).val()
        stock = StockSpec((-0.1, -0.1, 0), (0.1, 0.1, 2), 0.1)
        target, heights, report = self.report(
            shape, stock, 2, base=-2, finishing=FinishingScope(0, 1, stock_to_leave_mm=0.2)
        )
        self.assertTrue(report['finishing']['complete'])
        self.assertLess(report['finishing']['within_allowance']['volume_mm3'][1], 1e-10)

    def test_native_posted_stream_and_fractional_seek_bind_patches(self):
        from test_cad_surfaces import example

        module = example()
        job = module.build_job(cad_inner_patches=True)
        setup = job.setups[0]
        target = setup.verification.material[0].target
        verifier = Verifier(target)
        self.assertTrue(target.inner_patches)
        with Simulation(
            setup.stock,
            setup.tools,
            revision='patch-stream',
            initial_position_mm=setup.initial_position_mm,
            operations=setup.operations,
            verification=setup.verification,
            executable=os.environ.get('CAMLIB_TEST_ENGINE'),
        ) as sim:
            reports = [
                u.check['report']
                for u in sim.run_operations()
                if isinstance(u, VerificationUpdate) and u.check['kind'] == 'material'
            ]
            self.assertEqual(reports[-1], verifier.check(sim.state))
            self.assertTrue(reports[-1]['finishing']['complete'])
        with job.post('top', machine=module.machine_for(job), executable=os.environ.get('CAMLIB_TEST_ENGINE')) as run:
            run.wait()
            count = run.status()['available']
            a, b = Player(run, compact=True), Player(run)
            for cursor in (Cursor(count), Cursor(count // 2, 0.37), Cursor(count)):
                a.seek(cursor)
                b.seek(cursor)
                self.assertEqual(verifier.check(a.state), verifier.check(b.state))


if __name__ == '__main__':
    unittest.main()
