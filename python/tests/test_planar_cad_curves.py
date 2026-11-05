"""Bounded sketch curves in rigid setups, and their ordinary pocket workflow."""

import math
import unittest
from dataclasses import replace
import cadquery as cq
from camlib.cadquery import export_floor_region
from camlib.motion import WorkFrame
from test_cad_fixtures import source_bytes


def curved_floor():
    edge = cq.Edge.makeBezier([cq.Vector(-4, 0, 0), cq.Vector(-2, 6, 0), cq.Vector(3, 5, 0), cq.Vector(5, 0, 0)])
    wire = cq.Wire.assembleEdges([edge, cq.Edge.makeLine((5, 0, 0), (-4, 0, 0))])
    return cq.Face.makeFromWires(wire), edge


def segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


class PlanarCadCurvesTest(unittest.TestCase):
    def test_bezier_enclosure_convergence_native_offsets_and_source_immutability(self):
        from camlib import Engine

        face, edge = curved_floor()
        original = source_bytes(face)
        coarse = export_floor_region(face, name='curve', source={})
        fine = export_floor_region(face, name='curve', source={}, max_error_mm=0.001)
        self.assertLessEqual(coarse.boundary_error_mm, 0.005)
        self.assertLess(fine.boundary_error_mm, coarse.boundary_error_mm)
        self.assertGreater(len(fine.components[0].outer.vertices), len(coarse.components[0].outer.vertices))
        self.assertEqual(source_bytes(face), original)
        # Independent CAD evaluation challenges the certificate; these samples
        # are tests, not the production whole-interval bound.
        from OCP.BRepAdaptor import BRepAdaptor_Curve

        curve = BRepAdaptor_Curve(edge.wrapped)
        for r in (coarse, fine):
            ring = r.components[0].outer.vertices
            for i in range(501):
                p = curve.Value(i / 500).Coord()
                d = min(segment_distance(p, a, b) for a, b in zip(ring, ring[1:] + ring[:1]))
                self.assertLessEqual(d, r.boundary_error_mm)
            with Engine() as engine:
                summary = engine.prepare_region(r)['summary']
                self.assertLess(abs(summary['area_mm2'] - face.Area()), face.outerWire().Length() * r.boundary_error_mm)
                inset = engine.offset_region(0.4)['region']
                self.assertGreater(engine.prepare_region(inset)['summary']['area_mm2'], 1)

    def test_rigid_side_setup_and_holes_retain_model_frame_without_mutation(self):
        face = cq.Face.makeFromWires(cq.Workplane('XY').ellipse(6, 4).val(), [cq.Workplane('XY').circle(1).val()])
        frame = WorkFrame((20, -30, 12), (1, 0, 0), (0, 0, 1), (0, -1, 0))
        placed = face.rotate((0, 0, 0), (1, 0, 0), 90).translate(frame.origin_mm)
        original = source_bytes(placed)
        r = export_floor_region(placed, name='side', source={}, work_frame=frame)
        self.assertLessEqual(r.boundary_error_mm, 0.005)
        self.assertAlmostEqual(r.source['floor_z_mm'], 0)
        self.assertEqual(r.source['curve_counts'], {'ellipse': 1, 'circle': 1})
        self.assertEqual(len(r.components[0].holes[0].vertices), 2)
        self.assertEqual(source_bytes(placed), original)
        for bad in (replace(frame, x=(2, 0, 0)), replace(frame, z=(0, 1, 0))):
            with self.assertRaisesRegex(ValueError, 'rigid frame'):
                export_floor_region(placed, name='bad', source={}, work_frame=bad)
        with self.assertRaisesRegex(ValueError, 'parallel to work XY'):
            export_floor_region(placed, name='bad', source={})

    def test_rational_bezier_and_c0_spline_bounds_survive_trim_reversal(self):
        from OCP.Geom import Geom_BezierCurve, Geom_BSplineCurve
        from OCP.gp import gp_Pnt
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge

        try:
            from OCP.TColgp import TColgp_Array1OfPnt as Points
            from OCP.TColStd import TColStd_Array1OfReal as Reals, TColStd_Array1OfInteger as Ints
        except ImportError:
            from OCP.collections import Array1_gp_Pnt as Points, Array1_double as Reals, Array1_int as Ints
        points = Points(1, 3)
        weights = Reals(1, 3)
        for i, (x, y, w) in enumerate(((4, 0, 1), (4, 4, math.sqrt(0.5)), (0, 4, 1)), 1):
            points.SetValue(i, gp_Pnt(x, y, 0))
            weights.SetValue(i, w)
        rational = Geom_BezierCurve(points, weights)
        for i, (x, y) in enumerate(((0, 0), (2, 4), (4, 0)), 1):
            points.SetValue(i, gp_Pnt(x, y, 0))
        knots = Reals(1, 3)
        mult = Ints(1, 3)
        for i, (k, m) in enumerate(((0, 2), (0.37, 1), (1, 2)), 1):
            knots.SetValue(i, k)
            mult.SetValue(i, m)
        spline = Geom_BSplineCurve(points, knots, mult, 1)
        for curve in (rational, spline):
            edge = cq.Edge(BRepBuilderAPI_MakeEdge(curve, 0.1, 0.9).Edge())
            a = edge.startPoint()
            b = edge.endPoint()
            face = cq.Face.makeFromWires(cq.Wire.assembleEdges([edge, cq.Edge.makeLine(b, a)]))
            for f in (face, face.rotate((0, 0, 0), (1, 0, 0), 180)):
                r = export_floor_region(f, name='trim', source={})
                ring = r.components[0].outer.vertices
                self.assertLessEqual(r.boundary_error_mm, 0.005)
                self.assertGreater(len(ring), 4)
                for i in range(201):
                    p = curve.Value(0.1 + 0.8 * i / 200).Coord()
                    p = (p[0], p[1] if f is face else -p[1])
                    self.assertLessEqual(
                        min(segment_distance(p, a, b) for a, b in zip(ring, ring[1:] + ring[:1])), r.boundary_error_mm
                    )

    def test_budgets_cancellation_and_requested_error_fail_without_mutation(self):
        face, _ = curved_floor()
        original = source_bytes(face)
        for options, why in (
            (dict(max_segments=4), 'max_segments'),
            (dict(max_error_mm=0), 'max_error_mm'),
            (dict(cancel=lambda: True), 'cancelled'),
        ):
            with self.assertRaisesRegex(ValueError, why):
                export_floor_region(face, name='fail', source={}, **options)
        calls = 0

        def cancel():
            nonlocal calls
            calls += 1
            return calls > 4

        with self.assertRaisesRegex(ValueError, 'cancelled'):
            export_floor_region(face, name='fail', source={}, cancel=cancel)
        self.assertEqual(source_bytes(face), original)

    def test_side_pocket_job_cache_checked_post_and_fractional_stock_seek(self):
        import importlib.util
        import os
        from pathlib import Path
        from tempfile import TemporaryDirectory
        from camlib.jobs import RegenerationCache
        from camlib.playback import Player, Cursor
        from camlib.simulation import Update

        path = Path(__file__).resolve().parents[2] / 'python/tests/support/side_pockets.py'
        spec = importlib.util.spec_from_file_location('side_pockets_example', path)
        example = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(example)
        executable = os.environ.get('CAMLIB_TEST_ENGINE')
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            previous = None
            for revision in ('1', '2'):
                job = example.build_job(revision)
                with job.regenerate(cache, executable=executable) as run:
                    motions = [event.update.motion for event in run if isinstance(event.update, Update)]
                    self.assertTrue(run.finished)
                    state = run.states['side']
                    raw = state.heights.tobytes()
                    self.assertGreater(len(motions), 50)
                    self.assertGreater(state.volume()['removed_lower_mm3'], 1)
                    self.assertGreaterEqual(min(state.heights), -0.60001)
                    if previous is not None:
                        self.assertEqual(previous, raw)
                        self.assertTrue(run.statistics[('side', 'pocket')]['cache_hit'])
                    previous = raw
            with job.post('side', machine=example.machine_for(job), cache=cache, executable=executable) as run:
                run.wait(timeout=60)
                self.assertTrue(run.posting_status()['finished'])
                player = Player(run)
                last = run.status()['available']
                player.seek(Cursor(last))
                self.assertGreater(player.state.volume()['removed_lower_mm3'], 1)
                self.assertGreaterEqual(min(player.state.heights), -0.60001)
                full = player.state.heights.tobytes()
                cursor = Cursor(last // 2, 0.37)
                player.seek(cursor)
                partial = player.state.heights.tobytes()
                player.seek(Cursor(last))
                self.assertEqual(full, player.state.heights.tobytes())
                player.seek(cursor)
                self.assertEqual(partial, player.state.heights.tobytes())
                self.assertTrue(all(run.posted_motion(i)['stock_committed'] for i in range(last)))


if __name__ == '__main__':
    unittest.main()
