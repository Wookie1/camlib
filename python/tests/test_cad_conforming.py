"""CAD import refines each face conformingly: no T-junctions, so the solid stays closed."""

from collections import Counter
import unittest
from unittest import mock

from camlib import _cad_surface


def junctions(triangles):
    """Vertices that lie strictly inside another triangle's edge."""
    vertices = {p for t in triangles for p in t}
    found = []
    for t in triangles:
        for i in range(3):
            a, b = t[i], t[(i + 1) % 3]
            for p in vertices:
                if p in (a, b):
                    continue
                cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
                dot = (p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])
                length2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
                if abs(cross) < 1e-12 and 0 < dot < length2:
                    found.append((a, b, p))
    return found


class ConformingRefinementTest(unittest.TestCase):
    def refine(self, triangles, hot):
        # Only triangles touching `hot` are too coarse, and only until they are small.
        def error(triangle, bounds):
            area = abs(
                (triangle[1][0] - triangle[0][0]) * (triangle[2][1] - triangle[0][1])
                - (triangle[2][0] - triangle[0][0]) * (triangle[1][1] - triangle[0][1])
            )
            return 1.0 if hot in triangle and area > 0.02 else 0.0

        with (
            mock.patch.object(_cad_surface, '_interpolation_error', error),
            mock.patch.object(_cad_surface, '_local_bounds', lambda bounds, triangle: (0, 0, 1, 0, 1)),
        ):
            return _cad_surface._refine_conforming(triangles, None, 0.5, lambda live: None, lambda: None)

    def test_a_split_edge_is_split_on_both_sides(self):
        a, b, c, d = (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)
        # Two triangles sharing the diagonal a-c; only the first is too coarse.
        refined = self.refine([(a, b, c), (a, c, d)], hot=b)
        triangles = [t for t, _ in refined]
        self.assertGreater(len(triangles), 4)
        self.assertEqual(junctions(triangles), [])
        # Closed fan: every interior edge has two triangles, boundary edges one.
        uses = Counter(frozenset((t[i], t[(i + 1) % 3])) for t in triangles for i in range(3))
        self.assertTrue(all(n in (1, 2) for n in uses.values()))
        self.assertAlmostEqual(
            sum(
                abs((t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[2][0] - t[0][0]) * (t[1][1] - t[0][1])) / 2
                for t in triangles
            ),
            1.0,
        )

    def test_unrefined_neighbours_are_untouched_away_from_the_split(self):
        a, b, c, d = (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)
        refined = self.refine([(a, b, c), (a, c, d)], hot=d)
        self.assertEqual(junctions([t for t, _ in refined]), [])

    def test_both_sides_of_a_periodic_seam_split_together(self):
        # u = 0 and u = 1 are the same line in space (a periodic surface's seam).
        left = ((0.0, 0.0), (0.0, 1.0), (0.5, 0.5))
        right = ((1.0, 1.0), (1.0, 0.0), (0.5, 0.5))

        def error(triangle, bounds):
            area = abs(
                (triangle[1][0] - triangle[0][0]) * (triangle[2][1] - triangle[0][1])
                - (triangle[2][0] - triangle[0][0]) * (triangle[1][1] - triangle[0][1])
            )
            return 1.0 if any(p[0] == 0.0 for p in triangle) and area > 0.05 else 0.0

        with (
            mock.patch.object(_cad_surface, '_interpolation_error', error),
            mock.patch.object(_cad_surface, '_local_bounds', lambda bounds, triangle: (0, 0, 1, 0, 1)),
        ):
            refined = _cad_surface._refine_conforming(
                [left, right], None, 0.5, lambda live: None, lambda: None, lambda p: (p[0] % 1.0, p[1])
            )
        triangles = [t for t, _ in refined]

        def on(u):
            return sorted({p[1] for t in triangles for p in t if p[0] == u})

        self.assertEqual(on(0.0), on(1.0))
        self.assertGreater(len(on(0.0)), 2)


class EllipsoidImportTest(unittest.TestCase):
    def test_the_imported_ellipsoid_has_no_unmatched_edges(self):
        try:
            import cadquery as cq
        except ImportError:
            self.skipTest('CadQuery is not installed')
        from camlib.cadquery import export_surface_mesh

        ellipsoid = (
            cq.Workplane('XY')
            .sphere(3)
            .val()
            .transformGeometry(cq.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1.5, 0], [0, 0, 0, 1]]))
        )
        mesh = export_surface_mesh(ellipsoid, source={'revision': '1'})
        # The only gaps left are the facet regularization of numerically flat
        # slivers at the poles, which moves a vertex by at most the reported
        # `regularization_mm`: weld vertices within twice that, and every edge
        # is then shared by an even number of facets (it was thousands of
        # single-sided edges before the T-junctions were closed).
        weld = 2 * max(f['regularization_mm'] for f in mesh.source['face_bounds'])
        triangles = [tuple(tuple(round(v / weld) for v in p) for p in t) for t in mesh.triangles]
        triangles = [t for t in triangles if len(set(t)) == 3]
        uses = Counter(frozenset((t[i], t[(i + 1) % 3])) for t in triangles for i in range(3))
        odd = [e for e, n in uses.items() if n % 2]
        self.assertEqual(odd, [], f'{len(odd)} edges are not shared by an even number of facets')


if __name__ == '__main__':
    unittest.main()
