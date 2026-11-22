"""Leaning seam pcurves are straightened within edge tolerance and bounded (#361).

Open CASCADE's BRepMesh_IncrementalMesh can leave a valid conical face
untriangulated when its seam pcurves lean from vertical by 1e-15 in UV. The
adapter straightens vertical/horizontal seams within edge tolerance on its
private copy, bounds the 3D move over the expanded UV region, charges it into
the face's budget, and reports skipped leaning seams on mesher refusal.
"""

import math
import unittest
from unittest import mock

try:
    import cadquery as cq
    import numpy as np
except ImportError:  # pragma: no cover - CI installs CadQuery
    cq = None

from camlib import _cad_surface as cad
from camlib.cadquery import export_surface_mesh

if cq is not None:
    from OCP.BRep import BRep_Builder, BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve2d, BRepAdaptor_Surface
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire
    from OCP.BRepLib import BRepLib
    from OCP.Geom import Geom_BSplineSurface, Geom_Circle, Geom_ConicalSurface, Geom_CylindricalSurface, Geom_Line
    from OCP.Geom2d import Geom2d_BezierCurve, Geom2d_BSplineCurve, Geom2d_Line, Geom2d_OffsetCurve
    from OCP.Geom2dAdaptor import Geom2dAdaptor_Curve
    from OCP.GeomAPI import GeomAPI_Interpolate
    from OCP.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Dir2d, gp_Lin, gp_Pnt, gp_Pnt2d, gp_Trsf, gp_Vec
    from OCP.TopAbs import TopAbs_FORWARD, TopAbs_REVERSED
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS, TopoDS_Edge, TopoDS_Face, TopoDS_Shell, TopoDS_Solid, TopoDS_Vertex, TopoDS_Wire

    try:
        from OCP.TColgp import TColgp_Array1OfPnt2d as P2d, TColgp_Array2OfPnt as P3d2, TColgp_HArray1OfPnt as HP3d
        from OCP.TColStd import TColStd_Array1OfInteger as Ints, TColStd_Array1OfReal as Reals
    except ImportError:  # OCP 8 moved collection aliases.
        from OCP.collections import (
            Array1_double as Reals,
            Array1_gp_Pnt2d as P2d,
            Array1_int as Ints,
            Array2_gp_Pnt as P3d2,
            HArray1_gp_Pnt as HP3d,
        )
    from test_cad_watertight import distance_to_mesh, distance_to_solid, edge_points, surface_points, unmatched_edges


def _solid_from_faces(builder, fixed, flippable, min_volume=0.0):
    for flips in range(1 << len(flippable)):
        shell = TopoDS_Shell()
        solid = TopoDS_Solid()
        builder.MakeShell(shell)
        for face in fixed:
            builder.Add(shell, face)
        for k, face in enumerate(flippable):
            builder.Add(shell, face.Reversed() if flips >> k & 1 else face)
        builder.MakeSolid(solid)
        builder.Add(solid, shell)
        result = cq.Solid(solid)
        if result.isValid() and result.Volume() > min_volume:
            return result
    raise RuntimeError("no valid solid orientation")


def _cap_solid(face, cap_edges):
    caps = [BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(e).Wire(), True).Face() for e in cap_edges]
    return _solid_from_faces(BRep_Builder(), (face,), caps)


def _vertex(builder, surface, u, v, tol):
    out = TopoDS_Vertex()
    builder.MakeVertex(out, surface.Value(u, v), tol)
    return out


def _make_edge(builder, face, curve3d, v0, v1, pcurves, prange, tol, len3d=None):
    edge = TopoDS_Edge()
    builder.MakeEdge(edge, curve3d, tol)
    builder.Add(edge, v0.Oriented(TopAbs_FORWARD))
    builder.Add(edge, v1.Oriented(TopAbs_REVERSED))
    if isinstance(pcurves, tuple):
        builder.UpdateEdge(edge, pcurves[0], pcurves[1], face, tol)
        builder.Range(edge, face, *prange)
        if len3d is not None:
            builder.Range(edge, 0.0, len3d, True)
        builder.SameParameter(edge, False)
        builder.SameRange(edge, False)
    else:
        builder.Range(edge, *prange)
        builder.UpdateEdge(edge, pcurves, face, tol)
    return edge


def _add_wire(builder, face, oriented_edges):
    wire = TopoDS_Wire()
    builder.MakeWire(wire)
    for edge, ori in oriented_edges:
        builder.Add(wire, edge.Oriented(ori))
    builder.Add(face, wire)


def _bspline2d(coords, t1, weights=None):
    pts = P2d(1, len(coords))
    knots = Reals(1, 2)
    mults = Ints(1, 2)
    for i, (u, v) in enumerate(coords, 1):
        pts.SetValue(i, gp_Pnt2d(u, v))
    knots.SetValue(1, 0.0)
    knots.SetValue(2, t1)
    mults.SetValue(1, len(coords))
    mults.SetValue(2, len(coords))
    if weights is None:
        return Geom2d_BSplineCurve(pts, knots, mults, len(coords) - 1)
    wts = Reals(1, len(weights))
    for i, w in enumerate(weights, 1):
        wts.SetValue(i, w)
    return Geom2d_BSplineCurve(pts, wts, knots, mults, len(coords) - 1), pts, wts


def _cone_face(lean, *, tol=1e-7, match_top=False, bottom_lean=False, bspline_seam=False):
    """Build the 1-degree tapered cone face from issue #361 edge by edge."""
    radius = 1.1
    height = 3.2
    u0 = math.pi
    builder = BRep_Builder()
    face = TopoDS_Face()
    surface = Geom_ConicalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, -1), gp_Dir(-1, 0, 0)), math.radians(1), radius)
    builder.MakeFace(face, surface, tol)
    u_top = u0 + (lean * height if match_top else 0.0)
    top = _vertex(builder, surface, u_top, 0.0, tol)
    bottom = _vertex(builder, surface, u0, -height, tol)

    def circle(u_start, v, end):
        c = surface.VIso(v).Circ()
        geom = Geom_Circle(gp_Ax2(c.Location(), gp_Dir(0, 0, 1), gp_Dir(-1, 0, 0)), c.Radius())
        t_end = 2 * u0 + 2 * math.pi - u_start
        pc = Geom2d_Line(gp_Pnt2d(2 * u0 + 2 * math.pi, v), gp_Dir2d(-1, 0))
        return _make_edge(builder, face, geom, end, end, pc, (t_end - 2 * math.pi, t_end), tol)

    c_top = circle(u_top, 0.0, top)
    c_bottom = circle(u0, -height, bottom)
    a = surface.Value(u0, -height)
    b = surface.Value(u_top, 0.0)
    u_shift = -lean * height if bottom_lean else 0.0
    if bspline_seam:
        spans = ((0.0, -height), (0.5 * lean * height, -0.5 * height), (lean * height, 0.0))
        pcurves = tuple(
            _bspline2d([(ub + u_shift + du, v) for du, v in spans], height) for ub in (u0 + 2 * math.pi, u0)
        )
    else:
        pcurves = (
            Geom2d_Line(gp_Pnt2d(u0 + 2 * math.pi + u_shift, -height), gp_Dir2d(lean, 1.0)),
            Geom2d_Line(gp_Pnt2d(u0 + u_shift, -height), gp_Dir2d(lean, 1.0)),
        )
    line3d = Geom_Line(gp_Lin(a, gp_Dir(b.X() - a.X(), b.Y() - a.Y(), b.Z() - a.Z())))
    seam = _make_edge(builder, face, line3d, bottom, top, pcurves, (0.0, height), tol, a.Distance(b))
    _add_wire(
        builder,
        face,
        ((c_bottom, TopAbs_REVERSED), (seam, TopAbs_FORWARD), (c_top, TopAbs_FORWARD), (seam, TopAbs_REVERSED)),
    )
    BRepLib.SameParameter_s(face, tol, False)
    return face, (c_top, c_bottom)


def _frustum(lean, **kwargs):
    return _cap_solid(*_cone_face(lean, **kwargs))


def _tapered_hole_bushing(lean):
    """A valid hollow cylinder whose inner wall is the REVERSED tapered cone."""
    cone_face, (c_top, c_bottom) = _cone_face(lean)
    z_top = cq.Edge(c_bottom).Center().z
    outer = cq.Solid.makeCylinder(4.0, z_top)
    cyl = next(f for f in outer.Faces() if f.geomType() == "CYLINDER")
    top_d = next(f for f in outer.Faces() if f.geomType() == "PLANE" and abs(f.Center().z - z_top) < 1e-6)
    bot_d = next(f for f in outer.Faces() if f.geomType() == "PLANE" and abs(f.Center().z) < 1e-6)

    def ring(disk, inner):
        for flip in (False, True):
            hole = BRepBuilderAPI_MakeWire(TopoDS.Edge_s(inner.Reversed() if flip else inner)).Wire()
            f = BRepBuilderAPI_MakeFace(TopoDS.Face_s(disk.wrapped.Oriented(TopAbs_FORWARD)), hole).Face()
            if cq.Face(f).isValid() and cq.Face(f).Area() < disk.Area():
                return f
        raise RuntimeError("no valid ring face")

    return _solid_from_faces(
        BRep_Builder(), (cyl.wrapped,), (ring(top_d, c_bottom), ring(bot_d, c_top), cone_face), 100
    )


def _helical_seam_cylinder():
    """A valid cylinder whose seam genuinely twists by 0.25 rad (0.5 mm >> tol)."""
    tol = 1e-7
    radius = 2.0
    height = 4.0
    twist = 0.25
    builder = BRep_Builder()
    face = TopoDS_Face()
    surface = Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)), radius)
    builder.MakeFace(face, surface, tol)
    v0 = _vertex(builder, surface, 0.0, 0.0, tol)
    v1 = _vertex(builder, surface, twist, height, tol)

    def circle(u_start, v, end):
        c = surface.VIso(v).Circ()
        pc = Geom2d_Line(gp_Pnt2d(0.0, v), gp_Dir2d(1, 0))
        return _make_edge(
            builder, face, Geom_Circle(c.Position(), c.Radius()), end, end, pc, (u_start, u_start + 2 * math.pi), tol
        )

    c_bot = circle(0.0, 0.0, v0)
    c_top = circle(twist, height, v1)
    samples = HP3d(1, 33)
    for i in range(1, 34):
        samples.SetValue(i, surface.Value(twist * (i - 1) / 32, height * (i - 1) / 32))
    interp = GeomAPI_Interpolate(samples, False, 1e-9)
    interp.Perform()
    pcurves = (
        Geom2d_Line(gp_Pnt2d(2 * math.pi, 0.0), gp_Dir2d(twist, height)),
        Geom2d_Line(gp_Pnt2d(0.0, 0.0), gp_Dir2d(twist, height)),
    )
    seam = _make_edge(builder, face, interp.Curve(), v0, v1, pcurves, (0.0, math.hypot(twist, height)), tol)
    _add_wire(
        builder,
        face,
        ((c_bot, TopAbs_FORWARD), (seam, TopAbs_FORWARD), (c_top, TopAbs_REVERSED), (seam, TopAbs_REVERSED)),
    )
    BRepLib.SameParameter_s(face, 1e-5, False)
    return _cap_solid(face, (c_bot, c_top))


def _lean_torus_seams(lean):
    """Copy a torus and lean both its vertical (U) and horizontal (V) seams."""
    copy = cq.Shape.cast(BRepBuilderAPI_Copy(cq.Solid.makeTorus(5, 1).wrapped, True, False).Shape())
    builder = BRep_Builder()
    for face in copy.Faces():
        face_fwd = TopoDS.Face_s(face.wrapped.Oriented(TopAbs_FORWARD))
        seen = []
        for edge in face.Edges():
            ew = edge.wrapped
            if not BRep_Tool.IsClosed_s(ew, face_fwd) or any(ew.IsSame(x) for x in seen):
                continue
            seen.append(ew)
            c1 = BRepAdaptor_Curve2d(TopoDS.Edge_s(ew.Oriented(TopAbs_FORWARD)), face_fwd)
            c2 = BRepAdaptor_Curve2d(TopoDS.Edge_s(ew.Oriented(TopAbs_REVERSED)), face_fwd)
            f1 = c1.FirstParameter()
            l1 = c1.LastParameter()
            f2 = c2.FirstParameter()
            p1a = c1.Value(f1).Coord()
            p1b = c1.Value(l1).Coord()
            p2a = c2.Value(f2).Coord()
            if abs(p1b[0] - p1a[0]) < abs(p1b[1] - p1a[1]):
                d = gp_Dir2d(lean, 1.0 if p1b[1] > p1a[1] else -1.0)
            else:
                d = gp_Dir2d(1.0 if p1b[0] > p1a[0] else -1.0, lean)
            o1 = gp_Pnt2d(p1a[0] - f1 * d.X(), p1a[1] - f1 * d.Y())
            o2 = gp_Pnt2d(p2a[0] - f2 * d.X(), p2a[1] - f2 * d.Y())
            builder.UpdateEdge(ew, Geom2d_Line(o1, d), Geom2d_Line(o2, d), face_fwd, BRep_Tool.Tolerance_s(ew))
    return copy


def _run_straighten(solid, budget=0.04):
    copy = cq.Shape.cast(BRepBuilderAPI_Copy(solid.wrapped, True, False).Shape())
    faces = copy.Faces()
    adaptors = [BRepAdaptor_Surface(f.wrapped) for f in faces]
    splines = [cad._spline_bounds(ad) for ad in adaptors]
    sbounds = [cad._surface_bounds(ad, sp) for ad, sp in zip(adaptors, splines)]
    return copy, adaptors, *cad._straighten_seams(faces, splines, sbounds, budget, None)


@unittest.skipIf(cq is None, "needs CadQuery")
class SeamStraighteningTest(unittest.TestCase):
    def assert_refused_without_straightening(self, solid):
        with (
            mock.patch.object(cad, "_straighten_seams", lambda faces, *_: ([0.0] * len(faces), 0, [[] for _ in faces])),
            self.assertRaisesRegex(ValueError, "CAD mesher omitted face UV nodes"),
        ):
            export_surface_mesh(solid, max_error_mm=0.04)

    def assert_watertight_and_bounded(self, solid, mesh, max_error_mm, *, mesh_to_surface=None):
        self.assertEqual(unmatched_edges(mesh.triangles), [])
        self.assertLessEqual(mesh.surface_error_mm, max_error_mm)
        arr = np.array(mesh.triangles)
        normals = np.cross(arr[:, 1] - arr[:, 0], arr[:, 2] - arr[:, 0])
        self.assertGreater(np.linalg.norm(normals, axis=1).min(), 2e-10)
        volume = np.einsum("ij,ij->", arr[:, 0], normals) / 6
        area = np.linalg.norm(normals, axis=1).sum() / 2
        self.assertGreater(volume, 0)
        self.assertLessEqual(abs(volume - solid.Volume()), area * mesh.surface_error_mm)
        reach = 4 * max_error_mm
        measured = dict(
            surface_to_mesh=distance_to_mesh(surface_points(solid), mesh.triangles, reach),
            edges_to_mesh=distance_to_mesh(edge_points(solid), mesh.triangles, reach),
            mesh_to_surface=mesh_to_surface(arr) if mesh_to_surface else distance_to_solid(solid, mesh.triangles),
        )
        for direction, value in measured.items():
            self.assertLessEqual(value, mesh.surface_error_mm, (direction, measured))
        src = mesh.source
        fixed = src["cad_topology_error_mm"] + src["numerical_guard_mm"]
        for f in src["face_bounds"]:
            bound = f["interpolation_error_mm"] + f["trim_error_mm"] + f["edge_weld_mm"] + fixed
            self.assertLessEqual(bound, mesh.surface_error_mm)
        self.assertEqual(sum(f["facets"] for f in src["face_bounds"]), len(mesh.triangles))

    def assert_straightened_mesh(self, solid, max_error_mm, expected_seams, max_seam_err):
        mesh = export_surface_mesh(solid, max_error_mm=max_error_mm)
        self.assertEqual(mesh.source["seams_straightened"], expected_seams)
        self.assertTrue(0.0 < mesh.source["seam_straighten_error_mm"] <= max_seam_err)
        self.assert_watertight_and_bounded(solid, mesh, max_error_mm)
        return mesh

    def test_repro_fails_without_fix_and_passes_with_independent_checks(self):
        self.assert_refused_without_straightening(_frustum(-1e-15))
        for lean in (0.0, -1e-15, 1e-15, -9.69e-16, -1e-12, -1e-9):
            with self.subTest(lean=lean):
                solid = _frustum(lean)
                mesh = export_surface_mesh(solid, max_error_mm=0.04)
                self.assert_watertight_and_bounded(solid, mesh, 0.04)
                self.assertFalse(mesh.source["tolerances_tightened_for_meshing"])
                self.assertEqual(mesh.source["seams_straightened"], 0 if lean == 0.0 else 1)
                if lean == 0.0:
                    self.assertEqual(mesh.source["seam_straighten_error_mm"], 0.0)
                else:
                    err = mesh.source["seam_straighten_error_mm"]
                    self.assertTrue(0.0 < err <= 1e-7)
                    self.assertGreaterEqual(mesh.source["face_bounds"][0]["trim_error_mm"], err)
                    cone = solid.Faces()[0].wrapped
                    seam = next(e.wrapped for e in solid.Faces()[0].Edges() if BRep_Tool.IsClosed_s(e.wrapped, cone))
                    curve = BRepAdaptor_Curve2d(
                        TopoDS.Edge_s(seam.Oriented(TopAbs_FORWARD)), TopoDS.Face_s(cone.Oriented(TopAbs_FORWARD))
                    )
                    self.assertNotEqual(curve.Line().Direction().X(), 0.0)
        fine = _frustum(-1e-15)
        self.assert_watertight_and_bounded(fine, export_surface_mesh(fine, max_error_mm=0.002), 0.002)

    def test_reversed_tapered_hole_in_bushing(self):
        bushing = _tapered_hole_bushing(-9.69e-16)
        cone = next(f for f in bushing.Faces() if f.geomType() == "CONE")
        self.assertEqual(cone.wrapped.Orientation(), TopAbs_REVERSED)
        self.assert_refused_without_straightening(bushing)
        self.assert_straightened_mesh(bushing, 0.04, 1, 1e-7)

    def test_moved_location_frustum(self):
        trsf = gp_Trsf()
        trsf.SetTranslation(gp_Vec(50.0, -30.0, 12.0))
        moved = cq.Solid(TopoDS.Solid_s(_frustum(-1e-15).wrapped.Moved(TopLoc_Location(trsf))))
        self.assertFalse(moved.Faces()[0].wrapped.Location().IsIdentity())
        self.assert_straightened_mesh(moved, 0.04, 1, 1e-7)

    def test_bottom_lean_and_loose_tolerance_frusta(self):
        cases = (
            ("bottom lean 1e-15", _frustum(-1e-15, bottom_lean=True), 1e-7),
            ("bottom lean 1e-7", _frustum(-1e-7, tol=1e-6, bottom_lean=True), 1e-6),
            ("loose matched slant", _frustum(-2e-4, tol=2e-3, match_top=True), 2e-3),
        )
        for label, solid, edge_tol in cases:
            with self.subTest(case=label):
                self.assert_straightened_mesh(solid, 0.04, 1, edge_tol)

    def test_torus_u_and_v_seams(self):
        self.assert_straightened_mesh(_lean_torus_seams(-1e-12), 0.02, 2, 1e-7)

    def test_bspline_seam_pcurves(self):
        self.assert_straightened_mesh(_frustum(-1e-12, bspline_seam=True), 0.04, 1, 1e-7)

    def test_genuinely_slanted_helical_seam_left_untouched(self):
        cylinder = _helical_seam_cylinder()
        mesh = export_surface_mesh(cylinder, max_error_mm=0.04)
        self.assertEqual((mesh.source["seams_straightened"], mesh.source["seam_straighten_error_mm"]), (0, 0.0))

        def cyl_distance(arr):
            pts = np.concatenate([arr[:, 0], arr.mean(1), (arr[:, 0] + arr[:, 1]) / 2])
            r = np.hypot(pts[:, 0], pts[:, 1])
            z = pts[:, 2]
            wall = np.hypot(r - 2.0, np.maximum(0.0, np.maximum(-z, z - 4.0)))
            caps = np.hypot(np.maximum(0.0, r - 2.0), np.minimum(np.abs(z), np.abs(z - 4.0)))
            return float(np.minimum(wall, caps).max())

        self.assert_watertight_and_bounded(cylinder, mesh, 0.04, mesh_to_surface=cyl_distance)

    def test_clean_analytic_solids_left_untouched(self):
        cases = (
            ("cone frustum", cq.Solid.makeCone(4, 2, 5), 0.01),
            ("cone apex", cq.Solid.makeCone(4, 0, 5), 0.01),
            ("cylinder", cq.Solid.makeCylinder(3, 5), 0.01),
            ("sphere", cq.Workplane("XY").sphere(2).val(), 0.02),
            ("torus", cq.Solid.makeTorus(5, 1), 0.02),
        )
        for label, solid, max_err in cases:
            with self.subTest(shape=label):
                clean = export_surface_mesh(solid, max_error_mm=max_err)
                self.assertEqual(
                    (clean.source["seams_straightened"], clean.source["seam_straighten_error_mm"]), (0, 0.0)
                )

    def test_pcurve_straighten_bounds_lines_and_splines(self):
        vline = Geom2dAdaptor_Curve(Geom2d_Line(gp_Pnt2d(3.0, -1.0), gp_Dir2d(-1e-4, 1.0)), 2.0, 7.0)
        hline = Geom2dAdaptor_Curve(Geom2d_Line(gp_Pnt2d(1.0, 2.5), gp_Dir2d(-1.0, 2e-4)), -3.0, 4.0)
        bsp_geom, pts, wts = _bspline2d(((1.0, 0.0), (1.0 + 3e-4, 1.5), (1.0 - 2e-4, 3.0)), 3.0, (1.0, 0.7, 1.0))
        bsp = Geom2dAdaptor_Curve(bsp_geom, 0.0, 3.0)
        bez = Geom2dAdaptor_Curve(Geom2d_BezierCurve(pts, wts), 0.0, 1.0)
        for label, curve, axis, target in (
            ("vertical line", vline, 0, 3.0),
            ("horizontal line", hline, 1, 2.5),
            ("rational bspline", bsp, 0, 1.0),
            ("rational bezier", bez, 0, 1.0),
        ):
            with self.subTest(curve=label):
                f0 = curve.FirstParameter()
                f1 = curve.LastParameter()
                new_curve, du, dv, (u0, u1, v0, v1) = cad._pcurve_straighten(curve, f0, f1, axis, target)
                for i in range(101):
                    t = f0 + (f1 - f0) * i / 100
                    orig = curve.Value(t).Coord()
                    fixed = new_curve.Value(t).Coord()
                    self.assertEqual(fixed[axis], target)
                    self.assertLessEqual(abs(fixed[0] - orig[0]), du)
                    self.assertLessEqual(abs(fixed[1] - orig[1]), dv)
                    for p in (orig, fixed):
                        self.assertTrue(u0 - 1e-13 <= p[0] <= u1 + 1e-13 and v0 - 1e-13 <= p[1] <= v1 + 1e-13)

    def test_3d_move_bound_covers_independent_sampling(self):
        cases = (
            ("repro -1e-15", _frustum(-1e-15)),
            ("loose -2e-4", _frustum(-2e-4, tol=2e-3, match_top=True)),
            ("bspline seam", _frustum(-1e-12, bspline_seam=True)),
            ("torus seams", _lean_torus_seams(-1e-12)),
        )
        for label, solid in cases:
            with self.subTest(sampled_3d=label):
                orig_copy = cq.Shape.cast(BRepBuilderAPI_Copy(solid.wrapped, True, False).Shape())
                work_copy, adaptors, seam_errors, count, _ = _run_straighten(solid)
                self.assertGreater(count, 0)
                for fi, (of, nf, ad) in enumerate(zip(orig_copy.Faces(), work_copy.Faces(), adaptors)):
                    o_fwd = TopoDS.Face_s(of.wrapped.Oriented(TopAbs_FORWARD))
                    n_fwd = TopoDS.Face_s(nf.wrapped.Oriented(TopAbs_FORWARD))
                    o_seams = [e.wrapped for e in of.Edges() if BRep_Tool.IsClosed_s(e.wrapped, o_fwd)]
                    n_seams = [e.wrapped for e in nf.Edges() if BRep_Tool.IsClosed_s(e.wrapped, n_fwd)]
                    if not o_seams:
                        continue
                    max_3d = 0.0
                    for oe, ne in zip(o_seams, n_seams):
                        for ori in (TopAbs_FORWARD, TopAbs_REVERSED):
                            oc = BRepAdaptor_Curve2d(TopoDS.Edge_s(oe.Oriented(ori)), o_fwd)
                            nc = BRepAdaptor_Curve2d(TopoDS.Edge_s(ne.Oriented(ori)), n_fwd)
                            f0 = oc.FirstParameter()
                            f1 = oc.LastParameter()
                            for i in range(101):
                                t = f0 + (f1 - f0) * i / 100
                                d = ad.Value(*oc.Value(t).Coord()).Distance(ad.Value(*nc.Value(t).Coord()))
                                max_3d = max(max_3d, d)
                    self.assertTrue(0.0 < max_3d <= seam_errors[fi], (max_3d, seam_errors[fi]))
                mesh = export_surface_mesh(solid, max_error_mm=0.04)
                self.assertEqual(mesh.source["seam_straighten_error_mm"], max(seam_errors))

    def test_seam_move_bound_refuses_outside_non_periodic_spline_domain(self):
        poles = P3d2(1, 2, 1, 2)
        uk = Reals(1, 2)
        vk = Reals(1, 2)
        um = Ints(1, 2)
        vm = Ints(1, 2)
        for iu, u in enumerate((0.0, 1.0), 1):
            for iv, v in enumerate((0.0, 1.0), 1):
                poles.SetValue(iu, iv, gp_Pnt(u, v, u * v))
        for arr, a, b in ((uk, 0.0, 1.0), (vk, 0.0, 1.0), (um, 2, 2), (vm, 2, 2)):
            arr.SetValue(1, a)
            arr.SetValue(2, b)
        surface = Geom_BSplineSurface(poles, uk, vk, um, vm, 1, 1)
        ad = BRepAdaptor_Surface(BRepBuilderAPI_MakeFace(surface, 1e-7).Face())
        sp = cad._spline_bounds(ad)
        sb = cad._surface_bounds(ad, sp)
        self.assertIsNotNone(cad._seam_move_bound(ad, sp, sb, 1e-4, 0.0, (0.0, 1.0, 0.0, 1.0)))
        self.assertIsNone(cad._seam_move_bound(ad, sp, sb, 1e-4, 0.0, (-1e-5, 1.0, 0.0, 1.0)))
        self.assertIsNone(cad._seam_move_bound(ad, sp, sb, 0.0, 1e-4, (0.0, 1.0, 0.0, 1.0 + 1e-5)))

    def test_declined_leaning_seam_reported_on_mesher_refusal(self):
        solid = _frustum(-1e-15)
        f0 = TopoDS.Face_s(solid.Faces()[0].wrapped.Oriented(TopAbs_FORWARD))
        seam = next(e.wrapped for e in solid.Faces()[0].Edges() if BRep_Tool.IsClosed_s(e.wrapped, f0))
        c1 = BRepAdaptor_Curve2d(TopoDS.Edge_s(seam.Oriented(TopAbs_FORWARD)), f0)
        c2 = BRepAdaptor_Curve2d(TopoDS.Edge_s(seam.Oriented(TopAbs_REVERSED)), f0)
        BRep_Builder().UpdateEdge(
            seam,
            Geom2d_OffsetCurve(Geom2d_Line(c1.Line()), 0.0),
            Geom2d_OffsetCurve(Geom2d_Line(c2.Line()), 0.0),
            f0,
            BRep_Tool.Tolerance_s(seam),
        )
        self.assertTrue(solid.isValid())
        with self.assertRaisesRegex(
            ValueError,
            r"CAD mesher omitted face UV nodes; face 0 \(CONE, [0-9.]+ mm2\) has a seam whose pcurves lean "
            r"from vertical by [0-9.e+-]+ in UV, which could not be straightened because "
            r"pcurve types \(GeomAbs_OffsetCurve, GeomAbs_OffsetCurve\) are not supported",
        ):
            export_surface_mesh(solid, max_error_mm=0.04)

        for label, src, budget, fragment in (
            ("budget exceeded", _frustum(-1e-9), 1e-12, "exceeds half the error budget"),
            ("edge tolerance exceeded", _helical_seam_cylinder(), 1.0, "exceeds the edge tolerance"),
        ):
            with self.subTest(reason=label):
                _, _, _, count, notes = _run_straighten(src, budget)
                self.assertEqual(count, 0)
                self.assertTrue(any(fragment in msg for face_notes in notes for msg in face_notes), notes)


if __name__ == "__main__":
    unittest.main()
