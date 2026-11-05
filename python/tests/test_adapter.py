import math
import unittest

try:
    import cadquery as cq
except ImportError:
    cq = None

from camlib.cadquery import export_floor, export_floor_region


@unittest.skipUnless(cq is not None, "requires cadquery's CadQuery environment")
class AdapterTest(unittest.TestCase):
    def test_line_circle_error_and_wire_order(self):
        face = cq.Face.makeFromWires(
            cq.Workplane("XY").rect(40, 30).val(),
            [cq.Workplane("XY").circle(5).val()],
        )
        fixture = export_floor(face, name="test", source={})
        self.assertEqual(len(fixture["outer"]), 4)
        self.assertEqual(len(fixture["holes"]), 1)
        ring = fixture["holes"][0]
        for i, a in enumerate(ring):
            b = ring[(i + 1) % len(ring)]
            mid_radius = math.hypot((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            self.assertLessEqual(5 - mid_radius, fixture["boundary_error_mm"])
        self.assertEqual(fixture["source"]["curve_counts"], {"line": 4, "circle": 1})

    def test_unsupported_curves_and_tilted_faces_fail(self):
        ellipse = cq.Face.makeFromWires(cq.Workplane("XY").ellipse(20, 10).val())
        with self.assertRaisesRegex(ValueError, "unsupported boundary curve"):
            export_floor(ellipse, name="ellipse", source={})
        tilted = cq.Face.makeFromWires(cq.Workplane("YZ").rect(40, 30).val())
        with self.assertRaisesRegex(ValueError, "parallel to work XY"):
            export_floor(tilted, name="tilted", source={})

    def test_analytic_export_preserves_circles_in_both_wire_directions(self):
        from camlib import Engine

        face = cq.Face.makeFromWires(cq.Workplane("XY").rect(40, 30).val(), [cq.Workplane("XY").circle(5).val()])
        region = export_floor_region(face, name="analytic", source={"revision": "test"})
        self.assertEqual(len(region.components[0].outer.vertices), 4)
        self.assertEqual(len(region.components[0].holes[0].vertices), 2)
        self.assertEqual(region.source["error_components_mm"]["chord"], 0)
        self.assertLessEqual(region.boundary_error_mm, 1e-6)
        with Engine() as engine:
            summary = engine.prepare_region(region)["summary"]
            self.assertEqual(summary["arc_segments"], 2)
            self.assertAlmostEqual(summary["area_mm2"], 1200 - 25 * math.pi, places=8)
            offset = engine.offset_region(3)
            result = engine.prepare_region(offset["region"])
            self.assertEqual(result["summary"]["arc_segments"], 2)
            self.assertAlmostEqual(result["summary"]["area_mm2"], 34 * 24 - 64 * math.pi, places=7)
            self.assertEqual([q["verdict"] for q in engine.query_region([[0, 0], [10, 0]])], ["blocked", "allowed"])

    def test_partial_arc_side_is_preserved_under_reversal_and_plane_flip(self):
        from camlib import Engine

        wires = [
            cq.Workplane("XY").moveTo(-10, 0).threePointArc((0, 10), (10, 0)).lineTo(-10, 0).close().val(),
            cq.Workplane("XY").moveTo(10, 0).threePointArc((0, 10), (-10, 0)).lineTo(10, 0).close().val(),
        ]
        faces = [cq.Face.makeFromWires(wire) for wire in wires]
        faces.append(faces[0].rotate((0, 0, 0), (1, 0, 0), 180))
        with Engine() as engine:
            for i, face in enumerate(faces):
                region = export_floor_region(face, name="half-circle", source={})
                summary = engine.prepare_region(region)["summary"]
                self.assertAlmostEqual(summary["area_mm2"], 50 * math.pi, places=7)
                expected = ["allowed", "blocked"] if i < 2 else ["blocked", "allowed"]
                self.assertEqual([r["verdict"] for r in engine.query_region([[0, 5], [0, -5]])], expected)


if __name__ == "__main__":
    unittest.main()
