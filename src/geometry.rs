//! Adaptive orientation predicates decide topology; distance estimates carry a
//! conservative engineering guard. The guard is not a formal interval proof.
use crate::{Clearance, Fixture, Point, Verdict};
use robust::{Coord, orient2d};
use std::ops::Range;

const MAX_COORD: f64 = 1_000_000.0;
const MIN_EDGE: f64 = 0.000_001;

fn orient(a: Point, b: Point, c: Point) -> f64 {
    orient2d(
        Coord { x: a[0], y: a[1] },
        Coord { x: b[0], y: b[1] },
        Coord { x: c[0], y: c[1] },
    )
}

fn valid_point(p: Point) -> bool {
    // Exclude exponent extremes where expansion products can underflow. This is
    // far below physical CAM scales, but must be an explicit input restriction.
    p.iter()
        .all(|v| v.is_finite() && v.abs() <= MAX_COORD && (*v == 0.0 || v.abs() >= 1e-100))
}

#[derive(Clone, Copy, Debug)]
pub struct Bounds {
    pub min: Point,
    pub max: Point,
}

impl Bounds {
    fn edge(a: Point, b: Point) -> Self {
        Self {
            min: [a[0].min(b[0]), a[1].min(b[1])],
            max: [a[0].max(b[0]), a[1].max(b[1])],
        }
    }
    pub(crate) fn union(self, other: Self) -> Self {
        Self {
            min: [self.min[0].min(other.min[0]), self.min[1].min(other.min[1])],
            max: [self.max[0].max(other.max[0]), self.max[1].max(other.max[1])],
        }
    }
    fn overlaps(self, other: Self) -> bool {
        (0..2).all(|k| self.min[k] <= other.max[k] && other.min[k] <= self.max[k])
    }
    pub(crate) fn distance_sq(self, p: Point) -> f64 {
        (0..2)
            .map(|k| {
                (self.min[k] - p[k])
                    .max(p[k] - self.max[k])
                    .max(0.0)
                    .powi(2)
            })
            .sum()
    }
    fn ray_hits(self, p: Point) -> bool {
        self.min[1] <= p[1] && p[1] <= self.max[1] && p[0] <= self.max[0]
    }
}

pub(crate) fn point_segment_distance_sq(p: Point, a: Point, b: Point) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let v = [p[0] - a[0], p[1] - a[1]];
    let length_sq = d[0] * d[0] + d[1] * d[1];
    let t = if length_sq == 0. {
        0.
    } else {
        ((v[0] * d[0] + v[1] * d[1]) / length_sq).clamp(0., 1.)
    };
    (v[0] - t * d[0]).powi(2) + (v[1] - t * d[1]).powi(2)
}

#[derive(Clone, Copy)]
struct Edge {
    a: Point,
    b: Point,
    ring: usize,
    slot: usize,
    count: usize,
}

impl Edge {
    fn bounds(self) -> Bounds {
        Bounds::edge(self.a, self.b)
    }
    fn on(self, p: Point) -> bool {
        self.bounds().overlaps(Bounds::edge(p, p)) && orient(self.a, self.b, p) == 0.0
    }
    fn distance_sq(self, p: Point) -> f64 {
        point_segment_distance_sq(p, self.a, self.b)
    }
    fn intersects(self, other: Self) -> bool {
        if !self.bounds().overlaps(other.bounds()) {
            return false;
        }
        let a = orient(self.a, self.b, other.a);
        let b = orient(self.a, self.b, other.b);
        let c = orient(other.a, other.b, self.a);
        let d = orient(other.a, other.b, self.b);
        (a > 0.0 && b < 0.0 || a < 0.0 && b > 0.0) && (c > 0.0 && d < 0.0 || c < 0.0 && d > 0.0)
            || a == 0.0 && self.on(other.a)
            || b == 0.0 && self.on(other.b)
            || c == 0.0 && other.on(self.a)
            || d == 0.0 && other.on(self.b)
    }
}

struct Node {
    bounds: Bounds,
    range: Range<usize>,
    children: Option<[usize; 2]>,
}

pub struct Region {
    edges: Vec<Edge>,
    order: Vec<usize>,
    nodes: Vec<Node>,
    boundary_error: f64,
    guard: f64,
}

impl Region {
    pub fn prepare(f: &Fixture) -> Result<Self, String> {
        if f.version != 1 || f.units != "mm" {
            return Err("expected fixture version 1, units mm".into());
        }
        if !f.boundary_error_mm.is_finite()
            || f.boundary_error_mm < 0.0
            || f.boundary_error_mm > 1.0
        {
            return Err("boundary_error_mm must be finite and in [0, 1]".into());
        }
        let mut edges = Vec::new();
        let mut magnitude: f64 = 1.0;
        for (ring, points) in std::iter::once(&f.outer).chain(&f.holes).enumerate() {
            if points.len() < 3 {
                return Err(format!("ring {ring} needs at least three points"));
            }
            for (slot, &a) in points.iter().enumerate() {
                let b = points[(slot + 1) % points.len()];
                if !valid_point(a) {
                    return Err(format!(
                        "ring {ring}: coordinates must be finite, within ±{MAX_COORD} mm, nonzero magnitude >=1e-100 mm"
                    ));
                }
                if (a[0] - b[0]).hypot(a[1] - b[1]) < MIN_EDGE {
                    return Err(format!(
                        "ring {ring}, edge {slot}: length below {MIN_EDGE} mm (omit closing duplicate)"
                    ));
                }
                magnitude = magnitude.max(a[0].abs()).max(a[1].abs());
                edges.push(Edge {
                    a,
                    b,
                    ring,
                    slot,
                    count: points.len(),
                });
            }
        }
        let mut region = Self {
            order: (0..edges.len()).collect(),
            edges,
            nodes: Vec::new(),
            boundary_error: f.boundary_error_mm,
            guard: 1e-9_f64.max(128.0 * f64::EPSILON * magnitude),
        };
        region.build(0..region.order.len());
        region.validate_intersections()?;
        // No touching, crossings, nested islands, or islands outside the outer ring.
        // A shared ring-parity index suffices for queries once these invariants hold.
        for (i, hole) in f.holes.iter().enumerate() {
            if !ring_contains(&f.outer, hole[0]) {
                return Err(format!("hole {i} lies outside the outer ring"));
            }
            for (j, other) in f.holes.iter().enumerate() {
                if i != j && ring_contains(other, hole[0]) {
                    return Err(format!("holes {i} and {j} overlap or nest"));
                }
            }
        }
        Ok(region)
    }

    pub fn bounds(&self) -> Bounds {
        self.nodes[0].bounds
    }
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }
    pub fn numerical_guard_mm(&self) -> f64 {
        self.guard
    }
    pub fn boundary_error_mm(&self) -> f64 {
        self.boundary_error
    }

    fn build(&mut self, range: Range<usize>) -> usize {
        let bounds = self.order[range.clone()]
            .iter()
            .map(|&i| self.edges[i].bounds())
            .reduce(Bounds::union)
            .unwrap();
        let index = self.nodes.len();
        self.nodes.push(Node {
            bounds,
            range: range.clone(),
            children: None,
        });
        if range.len() > 8 {
            let axis = usize::from(bounds.max[1] - bounds.min[1] > bounds.max[0] - bounds.min[0]);
            let half = range.len() / 2;
            self.order[range.clone()].select_nth_unstable_by(half, |&i, &j| {
                (self.edges[i].a[axis] + self.edges[i].b[axis])
                    .total_cmp(&(self.edges[j].a[axis] + self.edges[j].b[axis]))
            });
            let mid = range.start + half;
            let left = self.build(range.start..mid);
            let right = self.build(mid..range.end);
            self.nodes[index].children = Some([left, right]);
        }
        index
    }

    fn validate_intersections(&self) -> Result<(), String> {
        for (i, edge) in self.edges.iter().enumerate() {
            self.validate_node(0, i, *edge)?;
        }
        Ok(())
    }

    fn validate_node(&self, index: usize, i: usize, edge: Edge) -> Result<(), String> {
        let node = &self.nodes[index];
        if !node.bounds.overlaps(edge.bounds()) {
            return Ok(());
        }
        if let Some([a, b]) = node.children {
            self.validate_node(a, i, edge)?;
            self.validate_node(b, i, edge)?;
        } else {
            for &j in &self.order[node.range.clone()] {
                if j <= i {
                    continue;
                }
                let other = self.edges[j];
                if !edge.intersects(other) {
                    continue;
                }
                let adjacent = edge.ring == other.ring
                    && ((edge.slot + 1) % edge.count == other.slot
                        || (other.slot + 1) % other.count == edge.slot);
                if adjacent {
                    let (shared, a, b) = if edge.b == other.a {
                        (edge.b, edge.a, other.b)
                    } else {
                        (edge.a, edge.b, other.a)
                    };
                    let dot = (a[0] - shared[0]) * (b[0] - shared[0])
                        + (a[1] - shared[1]) * (b[1] - shared[1]);
                    if orient(a, shared, b) != 0.0 || dot < 0.0 {
                        continue;
                    }
                }
                return Err(format!(
                    "boundaries intersect or touch: ring {} edge {}, ring {} edge {}",
                    edge.ring, edge.slot, other.ring, other.slot
                ));
            }
        }
        Ok(())
    }

    fn ray(&self, index: usize, p: Point, parity: &mut bool, boundary: &mut bool) {
        let node = &self.nodes[index];
        if !node.bounds.ray_hits(p) {
            return;
        }
        if let Some([a, b]) = node.children {
            self.ray(a, p, parity, boundary);
            self.ray(b, p, parity, boundary);
        } else {
            for &i in &self.order[node.range.clone()] {
                let e = self.edges[i];
                if e.on(p) {
                    *boundary = true;
                }
                // Half-open ray crossing avoids counting vertices twice; orientation
                // avoids division and the unstable computed x of an intersection.
                if (e.a[1] > p[1]) != (e.b[1] > p[1]) {
                    let side = orient(e.a, e.b, p);
                    if (e.b[1] > e.a[1] && side > 0.0) || (e.b[1] < e.a[1] && side < 0.0) {
                        *parity = !*parity;
                    }
                }
            }
        }
    }

    fn nearest(&self, index: usize, p: Point, best: &mut f64) {
        let node = &self.nodes[index];
        // Small slack prevents a rounded bbox lower bound pruning a nearer edge.
        if node.bounds.distance_sq(p) > *best * (1.0 + 64.0 * f64::EPSILON) + self.guard.powi(2) {
            return;
        }
        if let Some([a, b]) = node.children {
            let (first, second) =
                if self.nodes[a].bounds.distance_sq(p) <= self.nodes[b].bounds.distance_sq(p) {
                    (a, b)
                } else {
                    (b, a)
                };
            self.nearest(first, p, best);
            self.nearest(second, p, best);
        } else {
            for &i in &self.order[node.range.clone()] {
                *best = best.min(self.edges[i].distance_sq(p));
            }
        }
    }

    /// Distances are prepared once and can be reused for cutter-radius changes.
    pub fn clearance(&self, p: Point) -> Result<f64, String> {
        if !valid_point(p) {
            return Err(format!(
                "query coordinates must be finite, within ±{MAX_COORD} mm"
            ));
        }
        let (mut parity, mut boundary) = (false, false);
        self.ray(0, p, &mut parity, &mut boundary);
        if boundary {
            return Ok(0.0);
        }
        let mut distance = f64::INFINITY;
        self.nearest(0, p, &mut distance);
        Ok(if parity {
            distance.sqrt()
        } else {
            -distance.sqrt()
        })
    }

    pub fn classify(&self, signed_clearance_mm: f64, radius_mm: f64) -> Result<Clearance, String> {
        if !signed_clearance_mm.is_finite()
            || !radius_mm.is_finite()
            || !(0.0..=MAX_COORD).contains(&radius_mm)
        {
            return Err(
                "clearance must be finite; radius_mm must be finite and in [0, 1000000]".into(),
            );
        }
        let guard_mm = self.guard + self.boundary_error;
        let margin = signed_clearance_mm - radius_mm;
        let verdict = if margin > guard_mm {
            Verdict::Allowed
        } else if margin < -guard_mm {
            Verdict::Blocked
        } else {
            Verdict::Uncertain
        };
        Ok(Clearance {
            verdict,
            signed_clearance_mm,
            guard_mm,
        })
    }

    pub fn query(&self, p: Point, radius_mm: f64) -> Result<Clearance, String> {
        self.classify(self.clearance(p)?, radius_mm)
    }

    /// Deliberately unindexed reference for differential testing and benchmarks.
    pub fn clearance_linear(&self, p: Point) -> Result<f64, String> {
        if !valid_point(p) {
            return Err("invalid query coordinates".into());
        }
        let mut parity = false;
        let mut best = f64::INFINITY;
        for &e in &self.edges {
            if e.on(p) {
                return Ok(0.0);
            }
            if (e.a[1] > p[1]) != (e.b[1] > p[1]) {
                let side = orient(e.a, e.b, p);
                if (e.b[1] > e.a[1] && side > 0.0) || (e.b[1] < e.a[1] && side < 0.0) {
                    parity = !parity;
                }
            }
            best = best.min(e.distance_sq(p));
        }
        Ok(if parity { best.sqrt() } else { -best.sqrt() })
    }
}

fn ring_contains(ring: &[Point], p: Point) -> bool {
    let mut inside = false;
    for (i, &a) in ring.iter().enumerate() {
        let b = ring[(i + 1) % ring.len()];
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let side = orient(a, b, p);
            if b[1] > a[1] && side > 0.0 || b[1] < a[1] && side < 0.0 {
                inside = !inside;
            }
        }
    }
    inside
}
