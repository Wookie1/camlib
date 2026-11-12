"""CAD-kernel triangulation with independently bounded parametric interpolation.

The CAD kernel supplies valid trimmed-face topology. Requested mesher deflection
is only a seed: surface Hessians and trimming-curve derivative bounds determine
acceptance. No sampled distance is used as an error certificate.

The interpolation bound is the classical one from bounds on derivatives: a
triangle's linear interpolant of a surface differs from it by at most the
second-derivative bound times the squared UV edge length, and trimming curves
are bounded the same way. See R. Filip, D. Magedson and R. Markot, "Surface
algorithms using bounds on derivatives", Computer Aided Geometric Design 3
(1986) 295-311.
"""

from collections import Counter
import math
import os
import sys
import time


def _norm(values):
    return math.sqrt(sum(x * x for x in values))


def _bezier_bounds(curve, span=1.0):
    """Convex-hull bounds for rational Bezier first/second derivatives."""
    points = [tuple(curve.Pole(i).Coord()) for i in range(1, curve.NbPoles() + 1)]
    weights = [curve.Weight(i) for i in range(1, curve.NbPoles() + 1)]
    if not points or min(weights) <= 0 or not math.isfinite(span) or span <= 0:
        raise ValueError('invalid bounded Bezier curve')
    anchor = points[0]
    radius = max(_norm(tuple(a - b for a, b in zip(p, anchor))) for p in points)
    numerator = [tuple(w * (a - b) for a, b in zip(p, anchor)) for p, w in zip(points, weights)]

    def derivative(poles):
        degree = len(poles) - 1
        return [tuple(degree * (b - a) / span for a, b in zip(p, q)) for p, q in zip(poles, poles[1:])]

    n1 = derivative(numerator)
    n2 = derivative(n1)
    w1 = derivative([(w,) for w in weights])
    w2 = derivative(w1)
    n1max = max(map(_norm, n1), default=0.0)
    n2max = max(map(_norm, n2), default=0.0)
    w1max = max(map(_norm, w1), default=0.0)
    w2max = max(map(_norm, w2), default=0.0)
    speed = (n1max + w1max * radius) / min(weights)
    accel = (n2max + w2max * radius + 2 * w1max * speed) / min(weights)
    return speed, accel


def _curve_bounds(adaptor, dimension):
    """Global derivative bounds over the represented analytic/Bezier spans."""
    from OCP.GeomAbs import GeomAbs_Line, GeomAbs_Circle, GeomAbs_Ellipse, GeomAbs_BezierCurve, GeomAbs_BSplineCurve

    kind = adaptor.GetType()
    if kind == GeomAbs_Line:
        return 1.0, 0.0
    if kind == GeomAbs_Circle:
        r = adaptor.Circle().Radius()
        return r, r
    if kind == GeomAbs_Ellipse:
        r = adaptor.Ellipse().MajorRadius()
        return r, r
    if kind == GeomAbs_BezierCurve:
        return _bezier_bounds(adaptor.Bezier())
    if kind == GeomAbs_BSplineCurve:
        try:
            from OCP.TColStd import TColStd_Array1OfReal
        except ImportError:  # OCP 8 moved collection aliases.
            from OCP.collections import Array1_double as TColStd_Array1OfReal
        if dimension == 2:
            from OCP.Geom2dConvert import Geom2dConvert_BSplineCurveToBezierCurve as Converter
        else:
            from OCP.GeomConvert import GeomConvert_BSplineCurveToBezierCurve as Converter
        converter = Converter(adaptor.BSpline(), adaptor.FirstParameter(), adaptor.LastParameter(), 1e-12)
        count = converter.NbArcs()
        if count < 1:
            raise ValueError('CAD curve has no Bezier spans')
        knots = TColStd_Array1OfReal(1, count + 1)
        converter.Knots(knots)
        bounds = [_bezier_bounds(converter.Arc(i), knots.Value(i + 1) - knots.Value(i)) for i in range(1, count + 1)]
        spline = adaptor.BSpline()
        c1 = all(spline.Multiplicity(i) <= spline.Degree() - 1 for i in range(2, spline.NbKnots()))
        return max(b[0] for b in bounds), max(b[1] for b in bounds) if c1 else None
    raise ValueError(f'no derivative bound for CAD trimming/basis curve {kind}')


def _trim_deviation(adaptor):
    """speed, deviation(t0, t1): bound on a 2D trim's distance from its chord.

    B-spline trims use each Bezier arc's own second-derivative bound. A chord
    across a C0 knot k is split at C(k): |C - L| <= |C - M| + |M - L| for the
    polyline M through the knots, and |M - L| peaks at a knot, where it is
    evaluated directly."""
    from bisect import bisect_left, bisect_right
    from OCP.GeomAbs import GeomAbs_BSplineCurve

    speed, accel = _curve_bounds(adaptor, 2)
    if adaptor.GetType() != GeomAbs_BSplineCurve:
        if accel is None:
            return speed, lambda t0, t1: speed * (t1 - t0) / 2
        return speed, lambda t0, t1: accel * (t1 - t0) ** 2 / 8
    try:
        from OCP.TColStd import TColStd_Array1OfReal
    except ImportError:  # OCP 8 moved collection aliases.
        from OCP.collections import Array1_double as TColStd_Array1OfReal
    from OCP.Geom2dConvert import Geom2dConvert_BSplineCurveToBezierCurve

    spline = adaptor.BSpline()
    converter = Geom2dConvert_BSplineCurveToBezierCurve(
        spline, adaptor.FirstParameter(), adaptor.LastParameter(), 1e-12
    )
    count = converter.NbArcs()
    knots = TColStd_Array1OfReal(1, count + 1)
    converter.Knots(knots)
    knots = [knots.Value(i) for i in range(1, count + 2)]
    accels = [_bezier_bounds(converter.Arc(i), knots[i] - knots[i - 1])[1] for i in range(1, count + 1)]
    corners = sorted(
        spline.Knot(i)
        for i in range(2, spline.NbKnots())
        if spline.Multiplicity(i) >= spline.Degree() and knots[0] < spline.Knot(i) < knots[-1]
    )

    def value(t):
        return tuple(adaptor.Value(t).Coord())

    def deviation(t0, t1):
        if t1 <= t0:
            return 0.0
        i0 = min(max(bisect_right(knots, t0) - 1, 0), count - 1)
        i1 = min(max(bisect_left(knots, t1) - 1, i0), count - 1)
        a = max(accels[i0 : i1 + 1])
        inside = corners[bisect_right(corners, t0) : bisect_left(corners, t1)]
        if not inside:
            return a * (t1 - t0) ** 2 / 8
        nodes = [t0, *inside, t1]
        h = max(y - x for x, y in zip(nodes, nodes[1:]))
        c0, c1 = value(t0), value(t1)
        kink = max(
            math.dist(value(k), tuple(p + (k - t0) / (t1 - t0) * (q - p) for p, q in zip(c0, c1))) for k in inside
        )
        return a * h * h / 8 + kink

    return speed, deviation


def _bezier_patch_bounds(patch, su=1.0, sv=1.0):
    """Convex-hull bounds for a rational Bezier patch's first/second partials.

    With G = F - A for an anchor pole A, G*W = N where N has poles w*(P-A)
    and W is the weight polynomial, so |G| <= R (the pole radius about A) and
    G_u = (N_u - G W_u)/W, G_uu = (N_uu - 2 G_u W_u - G W_uu)/W,
    G_uv = (N_uv - G_u W_v - G_v W_u - G W_uv)/W. Each hodograph is bounded by
    its control net; W >= the smallest (positive) weight."""
    nu, nv = patch.NbUPoles(), patch.NbVPoles()
    points = [[tuple(patch.Pole(i, j).Coord()) for j in range(1, nv + 1)] for i in range(1, nu + 1)]
    weights = [[patch.Weight(i, j) for j in range(1, nv + 1)] for i in range(1, nu + 1)]
    wmin = min(min(row) for row in weights)
    if wmin <= 0 or not (math.isfinite(su) and math.isfinite(sv) and su > 0 and sv > 0):
        raise ValueError('invalid bounded Bezier surface')
    anchor = points[0][0]
    radius = max(_norm(tuple(a - b for a, b in zip(p, anchor))) for row in points for p in row)
    numer = [
        [tuple(w * (a - b) for a, b in zip(p, anchor)) for p, w in zip(prow, wrow)]
        for prow, wrow in zip(points, weights)
    ]
    weight = [[(w,) for w in row] for row in weights]

    def diff(net, du, dv):
        # Control net of d^(du+dv)/du^du dv^dv of a tensor Bezier, over [0,su]x[0,sv].
        for _ in range(du):
            k = len(net) - 1
            net = [
                [tuple(k * (b - a) / su for a, b in zip(x, y)) for x, y in zip(r0, r1)] for r0, r1 in zip(net, net[1:])
            ]
        for _ in range(dv):
            net = [
                [tuple((len(row) - 1) * (b - a) / sv for a, b in zip(x, y)) for x, y in zip(row, row[1:])]
                for row in net
            ]
        return max((_norm(x) for row in net for x in row), default=0.0)

    nu1, nv1 = diff(numer, 1, 0), diff(numer, 0, 1)
    wu1, wv1 = diff(weight, 1, 0), diff(weight, 0, 1)
    fu = (nu1 + wu1 * radius) / wmin
    fv = (nv1 + wv1 * radius) / wmin
    fuu = (diff(numer, 2, 0) + 2 * fu * wu1 + radius * diff(weight, 2, 0)) / wmin
    fvv = (diff(numer, 0, 2) + 2 * fv * wv1 + radius * diff(weight, 0, 2)) / wmin
    fuv = (diff(numer, 1, 1) + fu * wv1 + fv * wu1 + radius * diff(weight, 1, 1)) / wmin
    return fu, fv, fuu, fuv, fvv


class _PatchBounds(tuple):
    """Whole-face bounds (|Fu|, |Fv|, |Fuu|, |Fuv|, |Fvv|, False) plus a local
    lookup over the Bezier patches a UV box overlaps, and the interior knot
    lines where the surface is only C0 (triangles are cut along them first, so
    the second-derivative bound holds on each piece)."""

    def __new__(cls, uknots, vknots, patches, c0_u, c0_v):
        whole = tuple(max(b[k] for row in patches for b in row) for k in range(5))
        self = super().__new__(cls, (*whole, False))
        self.uknots, self.vknots, self.patches = uknots, vknots, patches
        self.c0 = ((0, tuple(c0_u)), (1, tuple(c0_v)))
        return self

    def local(self, uv):
        from bisect import bisect_left, bisect_right

        def span(knots, lo, hi):
            # Control-net bounds hold only inside each patch's own domain.
            guard = 1e-9 * max(1.0, knots[-1] - knots[0])
            if lo < knots[0] - guard or hi > knots[-1] + guard:
                raise ValueError('CAD UV mesh leaves the B-spline surface domain')
            last = len(knots) - 2
            i0 = min(max(bisect_right(knots, lo) - 1, 0), last)
            i1 = min(max(bisect_left(knots, hi) - 1, i0), last)
            return range(i0, i1 + 1)

        us = span(self.uknots, min(p[0] for p in uv), max(p[0] for p in uv))
        vs = span(self.vknots, min(p[1] for p in uv), max(p[1] for p in uv))
        rows = [self.patches[i][j] for i in us for j in vs]
        return (*(max(b[k] for b in rows) for k in range(5)), False)


SPLIT_PATCHES = 4


def _spline_surface_bounds(spline):
    try:
        from OCP.TColStd import TColStd_Array1OfReal
    except ImportError:  # OCP 8 moved collection aliases.
        from OCP.collections import Array1_double as TColStd_Array1OfReal
    from OCP.GeomConvert import GeomConvert_BSplineSurfaceToBezierSurface

    if spline.IsUPeriodic() or spline.IsVPeriodic():
        # Same geometry and parameters, written as one non-periodic period.
        from OCP.Geom import Geom_BSplineSurface

        spline = spline.Copy()
        if not isinstance(spline, Geom_BSplineSurface):
            raise ValueError("CAD B-spline surface copy failed")
        if spline.IsUPeriodic():
            spline.SetUNotPeriodic()
        if spline.IsVPeriodic():
            spline.SetVNotPeriodic()
    converter = GeomConvert_BSplineSurfaceToBezierSurface(spline)
    nu, nv = converter.NbUPatches(), converter.NbVPatches()
    if nu < 1 or nv < 1:
        raise ValueError('CAD surface has no Bezier patches')
    uk, vk = TColStd_Array1OfReal(1, nu + 1), TColStd_Array1OfReal(1, nv + 1)
    converter.UKnots(uk)
    converter.VKnots(vk)
    uknots = [uk.Value(i) for i in range(1, nu + 2)]
    vknots = [vk.Value(i) for i in range(1, nv + 2)]
    # Control-net bounds tighten as a patch is split (its net converges to
    # the surface): bound each patch as SPLIT x SPLIT sub-patches.
    k = SPLIT_PATCHES
    fine_u = [a + (b - a) * s / k for a, b in zip(uknots, uknots[1:]) for s in range(k)] + [uknots[-1]]
    fine_v = [a + (b - a) * t / k for a, b in zip(vknots, vknots[1:]) for t in range(k)] + [vknots[-1]]
    patches = [[None] * (nv * k) for _ in range(nu * k)]
    for i in range(1, nu + 1):
        su = (uknots[i] - uknots[i - 1]) / k
        for j in range(1, nv + 1):
            sv = (vknots[j] - vknots[j - 1]) / k
            patch = converter.Patch(i, j)
            for s in range(k):
                for t in range(k):
                    piece = patch.Copy()
                    piece.Segment(s / k, (s + 1) / k, t / k, (t + 1) / k)
                    patches[(i - 1) * k + s][(j - 1) * k + t] = _bezier_patch_bounds(piece, su, sv)
    uknots, vknots = tuple(fine_u), tuple(fine_v)
    c0_u = [spline.UKnot(i) for i in range(2, spline.NbUKnots()) if spline.UMultiplicity(i) >= spline.UDegree()]
    c0_v = [spline.VKnot(i) for i in range(2, spline.NbVKnots()) if spline.VMultiplicity(i) >= spline.VDegree()]
    return _PatchBounds(uknots, vknots, patches, c0_u, c0_v)


def _local_bounds(bounds, uv):
    return bounds.local(uv) if isinstance(bounds, _PatchBounds) else bounds


def _uv_box(bounds, a, b, reach):
    """Corners of the UV box around a and b grown by `reach`, clamped to a
    spline's domain (the trim and its chord lie inside the face)."""
    lo = [min(a[k], b[k]) - reach for k in (0, 1)]
    hi = [max(a[k], b[k]) + reach for k in (0, 1)]
    if isinstance(bounds, _PatchBounds):
        for k, knots in ((0, bounds.uknots), (1, bounds.vknots)):
            lo[k] = min(max(lo[k], knots[0]), knots[-1])
            hi[k] = min(max(hi[k], knots[0]), knots[-1])
    return ((lo[0], lo[1]), (hi[0], hi[1]))


def _cut_at_c0(triangle, bounds):
    """Split a CCW UV triangle along the surface's C0 knot lines."""
    pieces = [triangle]
    if not isinstance(bounds, _PatchBounds):
        return pieces
    for axis, lines in bounds.c0:
        for k in lines:
            out = []
            for tri in pieces:
                s = [p[axis] - k for p in tri]
                if min(s) >= 0 or max(s) <= 0:
                    out.append(tri)
                    continue
                for side in (1, -1):
                    poly = []
                    for n in range(3):
                        a, b = tri[n], tri[(n + 1) % 3]
                        sa, sb = side * s[n], side * s[(n + 1) % 3]
                        if sa >= 0:
                            poly.append(a)
                        if sa * sb < 0:
                            t = sa / (sa - sb)
                            x = tuple(pa + t * (pb - pa) for pa, pb in zip(a, b))
                            x = tuple(k if i == axis else v for i, v in enumerate(x))
                            poly.append(x)
                    for m in range(1, len(poly) - 1):
                        c = (poly[0], poly[m], poly[m + 1])
                        area = (c[1][0] - c[0][0]) * (c[2][1] - c[0][1]) - (c[1][1] - c[0][1]) * (c[2][0] - c[0][0])
                        if area > 0:
                            out.append(c)
            pieces = out
    return pieces


def _surface_bounds(adaptor):
    """Bounds for |Fu|, |Fv|, |Fuu|, |Fuv|, |Fvv| in world coordinates."""
    from OCP.GeomAbs import (
        GeomAbs_Plane,
        GeomAbs_Cylinder,
        GeomAbs_Cone,
        GeomAbs_Sphere,
        GeomAbs_Torus,
        GeomAbs_SurfaceOfExtrusion,
        GeomAbs_BSplineSurface,
        GeomAbs_BezierSurface,
    )

    kind = adaptor.GetType()
    if kind == GeomAbs_Plane:
        return 1.0, 1.0, 0.0, 0.0, 0.0
    if kind == GeomAbs_Cylinder:
        r = adaptor.Cylinder().Radius()
        return r, 1.0, r, 0.0, 0.0
    if kind == GeomAbs_Cone:
        cone = adaptor.Cone()
        sine = abs(math.sin(cone.SemiAngle()))
        r = max(
            abs(cone.RefRadius() + v * math.sin(cone.SemiAngle()))
            for v in (adaptor.FirstVParameter(), adaptor.LastVParameter())
        )
        return r, 1.0, r, sine, 0.0
    if kind == GeomAbs_Sphere:
        r = adaptor.Sphere().Radius()
        return r, r, r, r, r
    if kind == GeomAbs_Torus:
        torus = adaptor.Torus()
        r = torus.MinorRadius()
        total = torus.MajorRadius() + r
        return total, r, total, r, r
    if kind == GeomAbs_SurfaceOfExtrusion:
        speed, accel = _curve_bounds(adaptor.BasisCurve(), 3)
        return speed, 1.0, accel or 0.0, 0.0, 0.0, accel is None
    if kind == GeomAbs_BSplineSurface:
        return _spline_surface_bounds(adaptor.BSpline())
    if kind == GeomAbs_BezierSurface:
        patch = adaptor.Bezier()
        u0, u1, v0, v1 = patch.Bounds()
        b = _bezier_patch_bounds(patch, u1 - u0, v1 - v0)
        return _PatchBounds((u0, u1), (v0, v1), [[b]], (), ())
    raise ValueError(f'no parametric interpolation bound for CAD surface {kind}')


def _interpolation_error(uv, bounds):
    bounds = _local_bounds(bounds, uv)
    # Taylor's integral remainder and barycentric interpolation cancel the
    # first derivative. Variance <= span^2/4; |covariance| <= du*dv/4.
    du = max(p[0] for p in uv) - min(p[0] for p in uv)
    dv = max(p[1] for p in uv) - min(p[1] for p in uv)
    if len(bounds) > 5 and bounds[5]:
        return bounds[0] * du / 2  # C0 spline: first-derivative Lipschitz bound.
    return (bounds[2] * du * du + 2 * bounds[3] * du * dv + bounds[4] * dv * dv) / 8


def _regularize_facet(points):
    """Represent collapsed parameter facets with a tiny, explicitly bounded face.

    Pole UV edges can map to one point. The replacement displaces each vertex
    by at most the returned distance; convex interpolation inherits that bound.
    This changes no source surface or machining tolerance.
    """
    a, b, c = points
    ab = tuple(y - x for x, y in zip(a, b))
    ac = tuple(y - x for x, y in zip(a, c))
    cross = (ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0])
    if _norm(cross) > 2e-10:
        return points, 0.0
    pairs = [(math.dist(points[i], points[j]), i, j) for i, j in ((0, 1), (1, 2), (2, 0))]
    length, i, j = max(pairs)
    k = 3 - i - j
    result = list(points)
    if length < 1e-5:
        # All original vertices are within 10 nm of a; a 30 nm triangle has
        # area above the native numeric-resolution floor.
        result = [a, (a[0] + 3e-5, a[1], a[2]), (a[0], a[1] + 3e-5, a[2])]
    else:
        direction = tuple((q - p) / length for p, q in zip(points[i], points[j]))
        axis = min(range(3), key=lambda n: abs(direction[n]))
        normal = tuple((1.0 if n == axis else 0.0) - direction[axis] * direction[n] for n in range(3))
        norm = _norm(normal)
        displacement = max(1e-6, 4e-10 / length)
        result[k] = tuple(p + displacement * n / norm for p, n in zip(points[k], normal))
    error = max(math.dist(p, q) for p, q in zip(points, result))
    if error > 1e-4:
        raise ValueError('CAD facet regularization exceeds numeric error reserve')
    return tuple(result), error


def _check_cancel(cancel):
    if cancel is not None and cancel():
        raise ValueError('CAD surface export cancelled')


def _face_data(face, cancel):
    """Check UV mesh orientation/incidence and every trim's parameter chain."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve2d
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.TopLoc import TopLoc_Location

    adaptor = BRepAdaptor_Surface(face.wrapped)
    bounds = _surface_bounds(adaptor)
    if not all(math.isfinite(x) and x >= 0 for x in bounds[:5]):
        raise ValueError('nonfinite CAD derivative bound')
    location = TopLoc_Location()
    mesh = BRep_Tool.Triangulation_s(face.wrapped, location)
    if mesh is None or not mesh.HasUVNodes():
        raise ValueError('CAD mesher omitted face UV nodes')
    uv = [None] + [tuple(mesh.UVNode(i).Coord()) for i in range(1, mesh.NbNodes() + 1)]
    if any(not all(math.isfinite(v) for v in p) for p in uv[1:]):
        raise ValueError('nonfinite CAD UV mesh')
    indices = []
    edges = Counter()
    directions = Counter()
    for i in range(1, mesh.NbTriangles() + 1):
        if i % 1024 == 0:
            _check_cancel(cancel)
        triangle = tuple(mesh.Triangle(i).Get())
        a, b, c = (uv[j] for j in triangle)
        area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if area <= 0 or not math.isfinite(area):
            raise ValueError('CAD UV mesh has reversed/degenerate triangles')
        indices.append(triangle)
        for p, q in zip(triangle, (*triangle[1:], triangle[0])):
            edges[tuple(sorted((p, q)))] += 1
            directions[(p, q)] += 1
    if any(n not in (1, 2) for n in edges.values()) or any(
        n == 2 and (directions[(p, q)] != 1 or directions[(q, p)] != 1) for (p, q), n in edges.items()
    ):
        raise ValueError('CAD UV mesh has inconsistent edge incidence')
    boundary = {edge for edge, n in edges.items() if n == 1}
    clamp_error = 0.0
    if isinstance(bounds, _PatchBounds):
        # The mesher may place trim nodes a hair outside a spline's domain
        # (loose CAD tolerances); patch bounds hold only inside it. Clamp them
        # onto the domain: the trim check below measures each node against
        # its pcurve, so the move is charged to the trim error.
        # Boundary nodes may stray 1e-4 of the span, interior ones 1e-6;
        # every clamp is charged |F'| x the move.
        on_boundary = {i for edge in boundary for i in edge}
        lo = (bounds.uknots[0], bounds.vknots[0])
        hi = (bounds.uknots[-1], bounds.vknots[-1])
        span = max(hi[0] - lo[0], hi[1] - lo[1])
        for i in range(1, len(uv)):
            p = uv[i]
            q = tuple(min(max(p[k], lo[k]), hi[k]) for k in (0, 1))
            if q != p:
                moved = math.dist(p, q)
                if moved > (1e-4 if i in on_boundary else 1e-6) * span:
                    raise ValueError(
                        f'CAD UV mesh node {p} leaves the B-spline surface domain '
                        f'[{lo[0]}, {hi[0]}] x [{lo[1]}, {hi[1]}]'
                    )
                fu, fv = _local_bounds(bounds, _uv_box(bounds, q, q, moved))[:2]
                clamp_error = max(clamp_error, math.hypot(fu, fv) * moved)
                uv[i] = q
    represented = Counter()
    trim_error = 0.0
    trim_segments = 0
    # The input BRep is valid; matching every topological trim once to the mesh
    # boundary retains its holes/seams. Interior edges cancel in opposite pairs.
    for wire in face.Wires():
        explorer = BRepTools_WireExplorer(wire.wrapped, face.wrapped)
        while explorer.More():
            _check_cancel(cancel)
            edge = explorer.Current()
            curve = BRepAdaptor_Curve2d(edge, face.wrapped)
            polygon = BRep_Tool.PolygonOnTriangulation_s(edge, mesh, location)
            if polygon is None or not polygon.HasParameters():
                raise ValueError('CAD trim has no mesh parameter correspondence')
            count = polygon.NbNodes()
            if count < 2:
                raise ValueError('invalid CAD trim node count')
            first, last = curve.FirstParameter(), curve.LastParameter()
            params = [polygon.Parameter(i) for i in range(1, count + 1)]
            guard = 1e-10 * max(1.0, abs(first), abs(last))
            if (
                abs(params[0] - first) > guard
                or abs(params[-1] - last) > guard
                or any(b < a for a, b in zip(params, params[1:]))
            ):
                raise ValueError('CAD trim parameter chain is incomplete')
            speed, deviation_of = _trim_deviation(curve)
            ids = [polygon.Node(i) for i in range(1, count + 1)]
            first_uv = tuple(curve.Value(params[0]).Coord())
            shift = [0.0, 0.0]
            for axis, periodic, period in (
                (0, adaptor.IsUPeriodic(), adaptor.UPeriod),
                (1, adaptor.IsVPeriodic(), adaptor.VPeriod),
            ):
                if periodic:
                    value = period()
                    shift[axis] = round((uv[ids[0]][axis] - first_uv[axis]) / value) * value
            gaps = [
                math.dist(uv[j], tuple(v + delta for v, delta in zip(curve.Value(t).Coord(), shift)))
                for j, t in zip(ids, params)
            ]
            ends = speed * (abs(params[0] - first) + abs(params[-1] - last))
            for p, q, t0, t1, g0, g1 in zip(ids, ids[1:], params, params[1:], gaps, gaps[1:]):
                represented[tuple(sorted((p, q)))] += 1
                reach = max(g0, g1) + ends + deviation_of(t0, t1)
                # |F'| over the UV box holding this chord and the trim span
                # it stands for (B-splines: only the patches it overlaps).
                fu, fv = _local_bounds(bounds, _uv_box(bounds, uv[p], uv[q], reach))[:2]
                trim_error = max(trim_error, math.hypot(fu, fv) * reach)
                trim_segments += 1
            explorer.Next()
    if set(represented) != boundary or any(n != 1 for n in represented.values()):
        raise ValueError('CAD trim chains do not cover mesh boundary exactly once')
    return adaptor, bounds, uv, indices, trim_error + clamp_error, trim_segments


def _remeshable(error):
    """Mesher/topology mismatches (not missing bounds or cancellation)."""
    text = str(error)
    return not any(
        k in text for k in ('cancelled', 'interpolation bound', 'derivative bound', 'invalid bounded', 'surface domain')
    )


def _mesh_face_alone(face, deflection, trim_budget, cancel):
    """Triangulate one face on its own when the whole-solid mesh of it does
    not match its trims (imported parts with loose tolerances: the mesher
    merges trim nodes closer than the tolerance, or skips the face).

    Each face's facets are bounded against that face's own surface and
    trims, so faces need not share boundary nodes. Tightening tolerances on
    the private copy changes no geometry: the caller still charges twice the
    original topology tolerance. Returns _face_data's row, or None."""
    import cadquery as cq
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools
    from OCP.ShapeFix import ShapeFix_ShapeTolerance
    from OCP.TopAbs import TopAbs_SHAPE

    def attempt(tight, scale):
        _check_cancel(cancel)
        alone = cq.Face(BRepBuilderAPI_Copy(face.wrapped, True, False).Shape())
        if alone.wrapped.Orientation() != face.wrapped.Orientation():
            return None
        if tight:
            ShapeFix_ShapeTolerance().LimitTolerance(alone.wrapped, 1e-7, 1e-7, TopAbs_SHAPE)
        BRepTools.Clean_s(alone.wrapped)
        BRepMesh_IncrementalMesh(alone.wrapped, deflection * scale, False, 0.5, True)
        try:
            row = _face_data(alone, cancel)
        except ValueError as e:
            if not _remeshable(e):
                raise
            return None
        return row if row[4] <= trim_budget else None

    # Nearby deflections first, then an exactly spanned parameter rectangle
    # (cheap, zero trim error), then ever finer trims.
    for tight, scale in ((False, 1.0), (False, 0.5), (False, 2.0), (True, 1.0), (True, 0.5)):
        row = attempt(tight, scale)
        if row is not None:
            return row
    row = _rectangle_face_data(face, cancel)
    if row is not None:
        return row
    for scale in (0.25, 1 / 16):
        for tight in (False, True):
            row = attempt(tight, scale)
            if row is not None:
                return row
    return None


def _rectangle_face_data(face, cancel):
    """Last resort when the CAD mesher yields nothing usable for a face whose
    single trim loop is exactly its UV parameter rectangle (iso-line pcurves,
    e.g. a full-turn cone or cylinder band). Two UV triangles span the
    rectangle; the caller's bounded refinement subdivides them. The trims
    are the rectangle's sides themselves, so the trim error is zero (the
    edges' 3D curves stay within the topology tolerance the caller charges).
    Returns _face_data's row, or None when the trims are not that rectangle."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve2d
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.GeomAbs import GeomAbs_Line

    _check_cancel(cancel)
    if len(face.Wires()) != 1:
        return None
    adaptor = BRepAdaptor_Surface(face.wrapped)
    bounds = _surface_bounds(adaptor)
    u0, u1 = adaptor.FirstUParameter(), adaptor.LastUParameter()
    v0, v1 = adaptor.FirstVParameter(), adaptor.LastVParameter()
    if not all(math.isfinite(x) for x in (u0, u1, v0, v1)) or u1 <= u0 or v1 <= v0:
        return None
    guard = 1e-9 * max(1.0, abs(u0), abs(u1), abs(v0), abs(v1))
    covered = {('u', u0): 0.0, ('u', u1): 0.0, ('v', v0): 0.0, ('v', v1): 0.0}
    segments = 0
    explorer = BRepTools_WireExplorer(face.Wires()[0].wrapped, face.wrapped)
    while explorer.More():
        curve = BRepAdaptor_Curve2d(explorer.Current(), face.wrapped)
        if curve.GetType() != GeomAbs_Line:
            return None
        a = tuple(curve.Value(curve.FirstParameter()).Coord())
        b = tuple(curve.Value(curve.LastParameter()).Coord())
        for axis, other, side in (('u', 1, 0), ('v', 0, 1)):
            match = next(
                (
                    k
                    for (name, k) in covered
                    if name == axis and abs(a[side] - k) <= guard and abs(b[side] - k) <= guard
                ),
                None,
            )
            if match is not None:
                covered[(axis, match)] += abs(b[other] - a[other])
                break
        else:
            return None
        segments += 1
        explorer.Next()
    span = {'u': v1 - v0, 'v': u1 - u0}
    if any(abs(length - span[axis]) > 1e3 * guard for (axis, _), length in covered.items()):
        return None
    uv = [None, (u0, v0), (u1, v0), (u1, v1), (u0, v1)]
    return adaptor, bounds, uv, [(1, 2, 3), (1, 3, 4)], 0.0, segments


def _refine_conforming(triangles, bounds, remaining, check_budget, check_cancel, locate=lambda p: p):
    """Refine a face's UV triangles until each interpolates within `remaining`.

    Splitting a triangle's edge splits every triangle on that edge too, so the
    refined mesh has no T-junctions: a vertex lies on a neighbour's edge only
    when that edge is split there as well. (A lone split leaves the vertex
    inside the neighbour's edge, a gap as large as the interpolation error;
    the verification then sees a leaky solid.) Edges are matched by where their
    ends are in space (`locate` maps a UV point to coordinates), so the two
    sides of a periodic seam, which are far apart in UV, split together. An
    edge collapsed to a point (a pole) has no neighbour. Returns
    (triangle, error) pairs.
    """
    tris, depth, edges = [], [], {}

    def place(p):
        return tuple(round(v * 1e7) for v in locate(p))

    def key(p, q):
        p, q = place(p), place(q)
        return None if p == q else (p, q) if p <= q else (q, p)

    def add(triangle, d):
        tris.append(triangle)
        depth.append(d)
        for i in range(3):
            k = key(triangle[i], triangle[(i + 1) % 3])
            if k is not None:
                edges.setdefault(k, []).append(len(tris) - 1)
        return len(tris) - 1

    def remove(t):
        triangle = tris[t]
        tris[t] = None
        for i in range(3):
            k = key(triangle[i], triangle[(i + 1) % 3])
            if k is not None:
                edges[k].remove(t)

    def split(t, i, stack):
        # Bisect triangle `t` across its edge i (vertex i to i + 1).
        triangle, d = tris[t], depth[t]
        if d >= 24:
            raise ValueError('CAD surface interpolation exceeds refinement depth')
        p, q, c = triangle[i], triangle[(i + 1) % 3], triangle[(i + 2) % 3]
        middle = tuple((x + y) / 2 for x, y in zip(p, q))
        remove(t)
        accepted.pop(t, None)
        stack.extend((add((p, middle, c), d + 1), add((middle, q, c), d + 1)))

    stack = [add(triangle, 0) for triangle in triangles]
    accepted = {}
    steps = 0
    while stack:
        steps += 1
        if steps % 1024 == 0:
            check_cancel()
            check_budget(sum(1 for x in tris if x is not None))
        t = stack.pop()
        triangle = tris[t]
        if triangle is None or t in accepted:
            continue
        error = _interpolation_error(triangle, bounds)
        if error <= remaining:
            accepted[t] = error
            continue
        local = _local_bounds(bounds, triangle)

        def edge_cost(pair):
            a, b = (triangle[i] for i in pair)
            du, dv = abs(a[0] - b[0]), abs(a[1] - b[1])
            if len(local) > 5 and local[5]:
                return local[0] * du
            return local[2] * du * du + 2 * local[3] * du * dv + local[4] * dv * dv

        i, j = max(((0, 1), (1, 2), (2, 0)), key=edge_cost)
        k = key(triangle[i], triangle[j])
        if k is None:
            split(t, i, stack)
            continue
        for user in list(edges[k]):
            corners = tris[user]
            side = next(m for m in range(3) if key(corners[m], corners[(m + 1) % 3]) == k)
            split(user, side, stack)
    check_budget(sum(1 for x in tris if x is not None))
    return [(tris[t], error) for t, error in accepted.items()]


def export_curved_solid(shape, *, max_error_mm, max_facets, source, cancel):
    import cadquery as cq
    import OCP
    from OCP.BRep import BRep_Tool
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools
    from OCP.TopAbs import TopAbs_REVERSED
    from .surface import SurfaceMesh

    started = time.perf_counter()
    _check_cancel(cancel)
    if not math.isfinite(max_error_mm) or not 1e-6 <= max_error_mm <= 0.1:
        raise ValueError('max_error_mm must be finite in [1e-6, .1]')
    if max_facets is not None and (type(max_facets) is not int or max_facets < 4):
        raise ValueError('max_facets must be None (no cap) or an integer >= 4')
    # Meshing mutates kernel caches: use an owned copy, never the caller's BRep.
    shape = cq.Shape.cast(BRepBuilderAPI_Copy(shape.wrapped, True, False).Shape())
    faces = shape.Faces()
    if not faces:
        raise ValueError('CAD surface has no faces')
    topology = max(
        [BRep_Tool.Tolerance_s(v.wrapped) for v in shape.Vertices()]
        + [BRep_Tool.Tolerance_s(e.wrapped) for e in shape.Edges()]
        + [BRep_Tool.Tolerance_s(f.wrapped) for f in faces]
    )
    box = shape.BoundingBox()
    magnitude = max(1.0, *(abs(v) for v in (box.xmin, box.xmax, box.ymin, box.ymax, box.zmin, box.zmax)))
    numerical = 8192 * math.ulp(magnitude) + 1e-10
    fixed_error = 2 * topology + numerical
    budget = max_error_mm - fixed_error
    if budget <= 0:
        raise ValueError('CAD topology/numerical error exceeds requested surface error')
    # Check supported surface definitions before invoking a potentially costly mesh.
    from OCP.BRepAdaptor import BRepAdaptor_Surface

    for face in faces:
        _surface_bounds(BRepAdaptor_Surface(face.wrapped))
    seed_deflection = budget / 4
    remeshed = set()
    progress = os.environ.get('CAMLIB_CAD_PROGRESS') == '1'
    _check_cancel(cancel)
    BRepTools.Clean_s(shape.wrapped)
    BRepMesh_IncrementalMesh(shape.wrapped, seed_deflection, False, 0.5, True)
    if progress:
        print(
            f'cad: solid mesh at {seed_deflection:.6f} mm in {time.perf_counter() - started:.1f} s',
            file=sys.stderr,
            flush=True,
        )
    data = []
    for n, f in enumerate(faces):
        problem = None
        checked = time.perf_counter()
        try:
            row = _face_data(f, cancel)
        except ValueError as e:
            if not _remeshable(e):
                raise
            row, problem = None, e
        if row is None or row[4] > budget / 2:
            # Only this face is re-triangulated (finer, alone, or spanned).
            alone = _mesh_face_alone(f, seed_deflection, budget / 2, cancel)
            if alone is None and row is not None and row[4] < budget * 0.75:
                alone = row  # over half the budget: interior refines into the rest
            if alone is None:
                if problem is None:
                    raise ValueError(
                        f'CAD trim approximation exceeds requested bound on face {n} ({f.geomType()}, '
                        f'{f.Area():.3f} mm2): trim error {row[4]:.5f} mm, budget {budget:.5f} mm after '
                        f'2 x CAD tolerance {2 * topology:.5f} mm'
                    )
                raise ValueError(
                    f'CAD mesher cannot triangulate face {n} ({f.geomType()}, {f.Area():.3f} mm2) '
                    f'consistently with its trims: {problem}'
                )
            if alone is not row:
                remeshed.add(n)
            row = alone
        if progress:
            print(
                f'cad: trims of face {n} {f.geomType()} {time.perf_counter() - checked:.1f} s'
                + (f' (remeshed: {problem or "trim error"})' if n in remeshed else ''),
                file=sys.stderr,
                flush=True,
            )
        data.append(row)
    result = []
    face_reports = []
    max_error = 0.0
    evaluations = 0
    if progress:
        print(
            f'cad: meshed {len(faces)} faces in {time.perf_counter() - started:.1f} s, remeshed {sorted(remeshed)}',
            file=sys.stderr,
            flush=True,
        )
    for n, (face, (adaptor, bounds, uv, indices, trim_error, trim_segments)) in enumerate(zip(faces, data)):
        face_started = time.perf_counter()
        if progress:
            print(f'cad: face {n} {face.geomType()} start, {len(result)} facets so far', file=sys.stderr, flush=True)
        _check_cancel(cancel)
        reserve = min(1e-4, (budget - trim_error) / 4)
        remaining = budget - trim_error - reserve
        count = 0
        max_interior = 0.0
        regularization = 0.0
        point_cache = {}

        def point(p):
            nonlocal evaluations
            if p not in point_cache:
                point_cache[p] = tuple(adaptor.Value(*p).Coord())
                evaluations += 1
            return point_cache[p]

        def check_budget(live):
            if max_facets is not None and len(result) + live > max_facets:
                raise ValueError(
                    f'CAD surface exceeds max_facets {max_facets} at requested error, on face {n} '
                    f'({face.geomType()}, {face.Area():.3f} mm2, {count} facets of it so far)'
                )

        refined = _refine_conforming(
            [piece for tri in indices for piece in _cut_at_c0(tuple(uv[i] for i in tri), bounds)],
            bounds,
            remaining,
            check_budget,
            lambda: _check_cancel(cancel),
            point,
        )
        for triangle, error in refined:
            if count % 1024 == 0:
                _check_cancel(cancel)
            xyz = tuple(point(p) for p in triangle)
            if any(not all(math.isfinite(v) and abs(v) <= 1e6 for v in p) for p in xyz):
                raise ValueError('CAD surface exceeds bounded coordinates')
            xyz, displacement = _regularize_facet(xyz)
            if displacement > reserve:
                raise ValueError('requested error is below CAD facet numeric resolution')
            regularization = max(regularization, displacement)
            if face.wrapped.Orientation() == TopAbs_REVERSED:
                xyz = (xyz[0], xyz[2], xyz[1])
            result.append(xyz)
            count += 1
            max_interior = max(max_interior, error)
        achieved = max_interior + trim_error + regularization + fixed_error
        max_error = max(max_error, achieved)
        if progress:
            print(
                f'cad: face {n} {face.geomType()} {count} facets {time.perf_counter() - face_started:.1f} s',
                file=sys.stderr,
                flush=True,
            )
        face_reports.append(
            dict(
                kind=face.geomType(),
                remeshed_alone=n in remeshed,
                facets=count,
                trim_segments=trim_segments,
                interpolation_error_mm=max_interior,
                trim_error_mm=trim_error,
                regularization_mm=regularization,
            )
        )
    _check_cancel(cancel)
    return SurfaceMesh(
        result,
        max_error,
        dict(
            source or {},
            adapter='bounded_parametric_cad_solid_v1',
            cadquery_version=cq.__version__,
            ocp_version=OCP.__version__,
            requested_error_mm=max_error_mm,
            error_basis=(
                'CAD-kernel valid trim topology; matched UV boundary chains; '
                'analytic/Bezier derivative bounds (B-spline surfaces per Bezier patch, cut at C0 knots) '
                'for trim and barycentric surface interpolation'
            ),
            cad_topology_error_mm=2 * topology,
            numerical_guard_mm=numerical,
            face_bounds=face_reports,
            triangle_count=len(result),
            surface_evaluations=evaluations,
            seed_deflection_mm=seed_deflection,
            extraction_ms=(time.perf_counter() - started) * 1000,
        ),
    )
