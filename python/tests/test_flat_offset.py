"""Flat-area finishing in contour-parallel passes, outside in (#38).

Every check here is measured on the motion stream and the simulated stock
against analytic parts, independently of the planner.
"""

import math
import unittest

import numpy as np

import camlib
from camlib.cadquery import export_stock_mesh, export_surface_mesh
from camlib.simulation import OperationUpdate, Simulation, Update
from camlib.tools import HolderSection, ToolAssembly

R = 2.0  # cutter radius
STEPOVER = 1.5
CELL = 0.25
INITIAL_Z = 14


def tool():
    return ToolAssembly(
        "mill4",
        camlib.EndMill("em4", diameter=4, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )


# Parts as unions of boxes (x0, y0, z0, x1, y1, z1), with pockets cut out.
# `boxes` are added, `pockets` removed, `bosses` added after the pockets.
STEPPED = dict(
    boxes=[(0, 0, 0, 24, 16, 4)],
    pockets=[],
    bosses=[(8, 4, 4, 16, 12, 7)],
    extent=(0, 0, 24, 16),
)
# A block with a pocket; in the pocket a short island and, 3 mm beside it
# (narrower than the 4 mm tool), a boss standing well above the block.
ISLAND = dict(
    boxes=[(0, 0, 0, 30, 20, 6)],
    pockets=[(3, 3, 3, 27, 17, 6)],
    bosses=[(7, 8, 3, 13, 12, 5), (16, 8, 3, 20, 12, 9)],
    extent=(0, 0, 30, 20),
)


def solid(spec, skin=0.0):
    """The part, or with `skin` mm left on every flat (walls to size): the
    blank a flat finishing pass meets after roughing."""
    import cadquery as cq

    def box(b, top_skin):
        x0, y0, z0, x1, y1, z1 = b
        return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0 + top_skin, centered=False).translate((x0, y0, z0))

    shape = None
    for b in spec["boxes"]:
        s = box(b, skin)
        shape = s if shape is None else shape.union(s)
    for p in spec["pockets"]:
        x0, y0, z0, x1, y1, z1 = p
        shape = shape.cut(box((x0, y0, z0 + skin, x1, y1, z1 + skin + 1), 0))
    for b in spec["bosses"]:
        shape = shape.union(box(b, skin))
    return shape


def height(spec, x, y):
    """Analytic top of the part at (x, y) (numpy arrays)."""
    h = np.zeros(np.broadcast(x, y).shape)
    inside = lambda r: (x >= r[0]) & (x <= r[3]) & (y >= r[1]) & (y <= r[4])  # noqa: E731
    for b in spec["boxes"]:
        h = np.where(inside(b), np.maximum(h, b[5]), h)
    for p in spec["pockets"]:
        strict = (x > p[0]) & (x < p[3]) & (y > p[1]) & (y < p[4])
        h = np.where(strict, p[2], h)
    for b in spec["bosses"]:
        h = np.where(inside(b), np.maximum(h, b[5]), h)
    return h


class Field:
    """The part sampled on a fine grid: distance from a point to the part
    standing above a height."""

    def __init__(self, spec, step=0.05):
        x0, y0, x1, y1 = spec["extent"]
        xs = np.arange(x0, x1 + step / 2, step)
        ys = np.arange(y0, y1 + step / 2, step)
        self.X, self.Y = np.meshgrid(xs, ys)
        self.H = height(spec, self.X, self.Y)
        self.step = step
        self._above = {}

    def above(self, z):
        key = round(z, 6)
        if key not in self._above:
            m = self.H > z + 1e-6
            self._above[key] = np.stack([self.X[m], self.Y[m]], axis=1)
        return self._above[key]

    def clearance(self, p, z):
        """XY distance from p to the part above z (inf if none); exact to
        the sampling step."""
        pts = self.above(z)
        if len(pts) == 0:
            return math.inf
        return float(np.min(np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1])))


def flat_op(spec, entry=camlib.RampEntry(angle_deg=3, min_length_mm=1), **changes):
    surface = export_surface_mesh(solid(spec), source=dict(model="flat-offset", revision="1"))
    t = tool()
    op = camlib.Flat(
        "flats",
        t,
        surface,
        0,
        STEPOVER,
        14,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
        entry=entry,
        **changes,
    )
    return t, op


def run(spec, skin=0.1):
    """Run Flat on the roughed blank; return (report, motions, stock
    snapshots before each motion as (nx, ny, dx, dy, heights), final state)."""
    t, op = flat_op(spec)
    stock = camlib.StockSpec.from_mesh(export_stock_mesh(solid(spec, skin)), CELL)
    motions, before = [], []
    report = None
    with Simulation(
        stock=stock,
        tools=(t,),
        operations=(op,),
        revision="1",
        initial_position_mm=(0, 0, INITIAL_Z),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    ) as sim:
        s = sim.state
        snapshot = list(s.heights)
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate) and update.stage == "end":
                report = update.report
            if isinstance(update, Update):
                motions.append(update.motion)
                before.append(snapshot)
                snapshot = list(s.heights)
        assert sim.state.finished
        grid = (s.nx, s.ny, s.dx, s.dy, tuple(s.min_mm))
        final = list(s.heights)
    return report, motions, before, grid, final


def stock_top_near(grid, heights, p, radius):
    """Highest stock cell upper bound whose cell rectangle comes within
    `radius` of p (what the tool's disk at p can touch)."""
    nx, ny, dx, dy, lo = grid
    top = -math.inf
    for j in range(ny):
        for i in range(nx):
            x0, y0 = lo[0] + i * dx, lo[1] + j * dy
            ex = max(x0 - p[0], 0, p[0] - (x0 + dx))
            ey = max(y0 - p[1], 0, p[1] - (y0 + dy))
            if math.hypot(ex, ey) < radius:
                top = max(top, heights[2 * (j * nx + i) + 1])
    return top


def points(m, n=None):
    c = m["curve"]
    a, b = c["start"], c["end"]
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = n or max(2, int(length / 0.05) + 1)
    return [[a[k] + (b[k] - a[k]) * i / (n - 1) for k in range(3)] for i in range(n)]


def arc_points(c, step=0.05):
    a, ce = c["start"], c["center"]
    rho = math.hypot(a[0] - ce[0], a[1] - ce[1])
    a0 = math.atan2(a[1] - ce[1], a[0] - ce[0])
    sweep = c["sweep_rad"]
    n = max(2, int(rho * abs(sweep) / step) + 1)
    return [
        [ce[0] + rho * math.cos(a0 + sweep * i / (n - 1)), ce[1] + rho * math.sin(a0 + sweep * i / (n - 1)), a[2]]
        for i in range(n)
    ]


def vertical_plunges(motions):
    out = []
    for m in motions:
        a, b = m["curve"]["start"], m["curve"]["end"]
        if m["kind"] == "plunge" and math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-9:
            out.append(m)
    return out


def level_cuts(motions, z):
    return [
        m
        for m in motions
        if m["kind"] in ("cut", "link", "lead")
        and all(abs(p[2] - z) < 1e-6 for p in (m["curve"]["start"], m["curve"]["end"]))
    ]


def seg_distance(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    vx, vy = bx - ax, by - ay
    l2 = vx * vx + vy * vy
    t = 0 if l2 == 0 else max(0, min(1, ((p[0] - ax) * vx + (p[1] - ay) * vy) / l2))
    return math.hypot(p[0] - ax - t * vx, p[1] - ay - t * vy)


def rect_outline_distance(p, r):
    """Distance from p to the outline of rectangle r = (x0, y0, x1, y1)."""
    x0, y0, x1, y1 = r
    inside = x0 <= p[0] <= x1 and y0 <= p[1] <= y1
    if inside:
        return min(p[0] - x0, x1 - p[0], p[1] - y0, y1 - p[1])
    return math.hypot(max(x0 - p[0], 0, p[0] - x1), max(y0 - p[1], 0, p[1] - y1))


_cache = {}


def cached(name):
    if name not in _cache:
        _cache[name] = run({"stepped": STEPPED, "island": ISLAND}[name])
    return _cache[name]


class FlatOffsetTest(unittest.TestCase):
    def test_few_entries_and_links_at_depth(self):
        # Entries are the vertical plunges. The stepped part needs one per
        # ring along an edge or wall that cannot be linked to (boss top,
        # base outer edge, ring round the boss) plus one per piece the
        # offsets split off out of link reach (the base's two side strips
        # at d = 3): at most 6. Today: one per raster run (39).
        report, motions, *_ = cached("stepped")
        plunges = len(vertical_plunges(motions))
        self.assertLessEqual(plunges, 6, f"{plunges} entries; runs={report['runs']}")
        self.assertEqual(plunges, report["runs"])
        self.assertGreater(report.get("links_at_depth", 0), 0)

    def test_no_vertical_plunge_onto_a_finished_face(self):
        # No plunge marks: a vertical plunge that ends at a flat's cutting
        # height never has the tool's disk over that flat's face; the tool
        # comes down beside the face (side entry) or stops above the stock
        # and ramps.
        for name, spec in (("stepped", STEPPED), ("island", ISLAND)):
            _, motions, *_ = cached(name)
            field = Field(spec)
            worst = (math.inf, None)
            for m in vertical_plunges(motions):
                b = m["curve"]["end"]
                # Within the level tolerance: the mesh puts a level a few
                # 1e-5 off its analytic height (3.00001 for Z3).
                face = np.abs(field.H - b[2]) <= 1e-3
                if not face.any():
                    continue
                d = float(np.min(np.hypot(field.X[face] - b[0], field.Y[face] - b[1])))
                if d < worst[0]:
                    worst = (d, [round(v, 4) for v in b])
            self.assertGreaterEqual(
                worst[0],
                R - 1e-6 - field.step,
                f"{name}: plunge onto a finished face at {worst[1]} ({worst[0]:.4f} mm)",
            )

    def test_open_edges_are_entered_from_the_side_in_air(self):
        # The boss top and the base's outer edge are open: their first ring
        # is entered by a vertical plunge where the tool's disk touches no
        # stock above the plunge's end (measured on the stock before the
        # move), then a horizontal lead into the ring. Only the ring round
        # the boss (walled, 0.1 mm skin left on the flat) may need a ramp.
        report, motions, before, grid, _ = cached("stepped")
        in_air = 0
        for m in vertical_plunges(motions):
            b = m["curve"]["end"]
            top = stock_top_near(grid, before[motions.index(m)], b, R)
            if top <= b[2] + 1e-6:
                in_air += 1
        self.assertGreaterEqual(in_air, 2, f"in-air entries {in_air}")
        self.assertLessEqual(report.get("ramped_entries", 0), 1)

    def test_first_ring_of_each_level_follows_its_edge(self):
        # Outside in: the first loop cut at each level runs round the
        # level's outer edge (boss top: the 8 x 8 boss; base: the 24 x 16
        # block), all the way round, its centre the radius less one stepover
        # outside the edge: its disk takes one stepover of the face, like
        # every later pass, rather than rolling a radius outside it in air.
        _, motions, *_ = cached("stepped")
        for z, rect in ((7.0, (8, 4, 16, 12)), (4.0, (0, 0, 24, 16))):
            cuts = [m for m in level_cuts(motions, z) if m["kind"] == "cut"]
            self.assertTrue(cuts, z)
            start = cuts[0]["curve"]["start"]
            ring, travelled = [], 0.0
            for m in cuts:
                ring.append(m)
                a, b = m["curve"]["start"], m["curve"]["end"]
                travelled += math.hypot(b[0] - a[0], b[1] - a[1])
                if travelled > 1 and math.hypot(b[0] - start[0], b[1] - start[1]) < 1e-6:
                    break
            worst = max(abs(rect_outline_distance(p, rect) - (R - STEPOVER)) for m in ring for p in points(m))
            self.assertLessEqual(worst, 0.01, f"Z{z}: first loop strays {worst:.4f} mm from the edge ring")
            perimeter = 2 * (rect[2] - rect[0] + rect[3] - rect[1]) + 2 * math.pi * (R - STEPOVER)
            self.assertGreater(travelled, 0.98 * perimeter, f"Z{z}: first loop {travelled:.3f} of {perimeter:.3f} mm")

    def test_coverage_of_every_reachable_flat_point(self):
        # Every flat point at least a radius (+0.01) from the part standing
        # above it lies under the tool's disk on some cut at its level.
        for name, spec in (("stepped", STEPPED), ("island", ISLAND)):
            _, motions, *_ = cached(name)
            field = Field(spec)
            x0, y0, x1, y1 = spec["extent"]
            worst = (0.0, None)
            for x in np.arange(x0 + 0.125, x1, 0.25):
                for y in np.arange(y0 + 0.125, y1, 0.25):
                    z = float(height(spec, np.array(x), np.array(y)))
                    if z <= 0 or field.clearance((x, y), z) < R + 0.01:
                        continue
                    segs = [(m["curve"]["start"], m["curve"]["end"]) for m in level_cuts(motions, z)]
                    d = min((seg_distance((x, y), a, b) for a, b in segs), default=math.inf)
                    if d > worst[0]:
                        worst = (d, (x, y, z))
            self.assertLessEqual(
                worst[0], R - 1e-6, f"{name}: uncovered flat point {worst[1]} {worst[0]:.4f} mm from any cut"
            )

    def test_tool_never_reaches_into_the_part_above_its_tip(self):
        # On every feed move (cut, link, lead, plunge, ramp) the tool's
        # axis stays at least a radius (less 1e-4) from every part point
        # above its tip: measured against the analytic part, including the
        # tall boss 3 mm beside the island.
        for name, spec in (("stepped", STEPPED), ("island", ISLAND)):
            _, motions, *_ = cached(name)
            field = Field(spec)
            worst = (math.inf, None)
            for m in motions:
                if m["kind"] == "rapid":
                    continue
                for p in points(m, 9):
                    c = field.clearance(p, p[2])
                    if c < worst[0]:
                        worst = (c, (m["id"], m["kind"], [round(v, 4) for v in p]))
            self.assertGreaterEqual(
                worst[0],
                R - 1e-4 - field.step,
                f"{name}: tool reaches {R - worst[0]:.4f} mm into the part at {worst[1]}",
            )

    def test_never_cuts_below_the_part(self):
        for name, spec in (("stepped", STEPPED), ("island", ISLAND)):
            _, _, _, grid, final = cached(name)
            nx, ny, dx, dy, lo = grid
            worst = 0.0
            for j in range(ny):
                for i in range(nx):
                    xs = np.array([lo[0] + i * dx, lo[0] + (i + 1) * dx])
                    ys = np.array([lo[1] + j * dy, lo[1] + (j + 1) * dy])
                    X, Y = np.meshgrid(xs, ys)
                    lowest = float(np.min(height(spec, X, Y)))
                    worst = max(worst, lowest - final[2 * (j * nx + i) + 1])
            self.assertLessEqual(worst, 1e-6, f"{name}: stock cut {worst:.4f} mm below the part")

    def test_no_moves_of_kernel_artefacts(self):
        # A kernel's flattened circle (chords mixed with micro-arcs) and its
        # snapped corner stubs are not cut as moves of their own: no cut,
        # link or lead shorter than 5 microns (the arcs' own move sagitta).
        for name in ("stepped", "island"):
            _, motions, *_ = cached(name)
            tiny = [
                m["id"]
                for m in motions
                if m["kind"] in ("cut", "link", "lead")
                and math.hypot(*(m["curve"]["end"][k] - m["curve"]["start"][k] for k in (0, 1))) < 0.005
            ]
            self.assertEqual(tiny, [], f"{name}: moves under 5 microns: {tiny[:10]}")

    def test_passes_climb_mill(self):
        # With a clockwise spindle a pass climbs when the stock it cuts lies on
        # the right of travel. Measured on the live stock: just outside the
        # disk, beside the middle of each straight cut, which side still has
        # stock above the cut height.
        for name in ("stepped", "island"):
            _, motions, before, grid, _ = cached(name)
            nx, ny, dx, dy, lo = grid
            side = {"climb": 0.0, "conventional": 0.0}
            for m, snap in zip(motions, before):
                c = m["curve"]
                a, b = c["start"], c["end"]
                length = math.hypot(b[0] - a[0], b[1] - a[1])
                if m["kind"] != "cut" or c.get("kind", "line") != "line" or length < 1e-6:
                    continue
                tx, ty = (b[0] - a[0]) / length, (b[1] - a[1]) / length
                mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)

                def stock(sign):
                    for d in (R * 0.8, R * 0.95):
                        x, y = mid[0] + sign * ty * d, mid[1] - sign * tx * d
                        i, j = int((x - lo[0]) / dx), int((y - lo[1]) / dy)
                        if 0 <= i < nx and 0 <= j < ny and snap[2 * (j * nx + i) + 1] > a[2] + 0.02:
                            return True
                    return False

                right, left = stock(1), stock(-1)
                if right != left:
                    side["climb" if right else "conventional"] += length
            self.assertGreater(side["climb"], 3 * side["conventional"], f"{name}: {side}")

    def test_passes_do_not_run_over_finished_floor(self):
        # Measured on the live stock before each move: the length of the
        # level passes (cuts, links and leads at a flat's height) whose disk
        # (less 0.1 mm) has no stock standing 0.02 mm above the cut — air
        # beside an open edge, or floor an earlier pass already finished.
        # A first ring rolling a radius outside every open edge, and inner
        # rings run all the way round a rim or top that earlier rings had
        # already covered, made this 47 % (stepped) and 45 % (island).
        for name in ("stepped", "island"):
            _, motions, before, grid, _ = cached(name)
            nx, ny, dx, dy, lo = grid
            xs = lo[0] + (np.arange(nx) + 0.5) * dx
            ys = lo[1] + (np.arange(ny) + 0.5) * dy
            X, Y = np.meshgrid(xs, ys)
            total = idle = 0.0
            for m, snap in zip(motions, before):
                c = m["curve"]
                z = c["start"][2]
                if m["kind"] not in ("cut", "link", "lead") or abs(c["end"][2] - z) > 1e-6:
                    continue
                H = np.array(snap[1::2]).reshape(ny, nx)
                ps = points(m) if c.get("kind", "line") == "line" else arc_points(c)
                length = sum(math.dist(p[:2], q[:2]) for p, q in zip(ps, ps[1:]))
                live = [bool((H[(X - p[0]) ** 2 + (Y - p[1]) ** 2 < (R - 0.1) ** 2] > z + 0.02).any()) for p in ps]
                total += length
                idle += length * (1 - sum(live) / len(live))
            self.assertLess(idle, 0.1 * total, f"{name}: {idle:.1f} of {total:.1f} mm over no stock")


if __name__ == "__main__":
    unittest.main()
