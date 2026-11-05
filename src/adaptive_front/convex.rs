//! Coherent convex parallel fronts. Only the owner of matching whole native
//! recipes may advance the material envelope. See docs/convex-front.md.
use super::{candidate, tangent};
use crate::{
    Point,
    motion::{Curve, Motion, MoveKind, Plane, Vec3},
    planar::{Component, ConstructionOptions, PlanarRegion, PreparedRegion},
    pocket::{Stroke, stroke_distance},
    stock::CancelSignal,
};
use robust::{Coord, orient2d};

fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}
fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn norm(a: Point) -> f64 {
    a[0].hypot(a[1])
}
fn orient(a: Point, b: Point, c: Point) -> f64 {
    orient2d(
        Coord { x: a[0], y: a[1] },
        Coord { x: b[0], y: b[1] },
        Coord { x: c[0], y: c[1] },
    )
}
pub(super) fn hull(mut points: Vec<Point>) -> Vec<Point> {
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points.dedup();
    fn half(iter: impl Iterator<Item = Point>) -> Vec<Point> {
        let mut h = Vec::new();
        for p in iter {
            while h.len() > 1 && orient(h[h.len() - 2], h[h.len() - 1], p) <= 0. {
                h.pop();
            }
            h.push(p);
        }
        h
    }
    let mut a = half(points.iter().copied());
    let mut b = half(points.iter().rev().copied());
    a.pop();
    b.pop();
    a.extend(b);
    a
}

pub(super) struct ConvexFront {
    vertices: Vec<Point>,
    normals: Vec<Point>,
    edge: usize,
    distance: f64,
    floor: f64,
    minimum_axis: f64,
    previous: Option<(Vec3, f64, f64)>, // real closed-contour seam, axis radius, face parameter
}
pub(super) struct ConvexContour {
    pub start: Vec3,
    pub heading: f64,
    pub leads: Vec<Curve>,
    pub contour: Vec<Curve>,
    pub reset: bool,
    pub angle: f64,
    radius: f64,
    tolerance: f64,
    cursor: usize,
    distance: f64,
    axis: f64,
    seam: f64,
    bounds: [Point; 2],
}
/// All remaining material is inside this convex body offset by `distance`.
/// Constructed only from a fully consumed contour, never from a proposed hull.
pub(super) struct ConvexMaterial {
    vertices: Vec<Point>,
    distance: f64,
    floor: f64,
}

impl ConvexMaterial {
    /// Coverage-domain certificate only: all remaining requested material is
    /// inside C offset distance. A separating supporting plane excludes this
    /// ENTIRE rectangle. This must never be used as a stock/motion clearance.
    pub(super) fn excludes_box(
        &self,
        bounds: [Point; 2],
        floor: f64,
        tolerance: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        if cancel.is_cancelled() {
            return Err("convex coverage cancelled".into());
        }
        if floor < self.floor {
            return Ok(false);
        }
        let charge = self.vertices.len() as u64 * 2 + 1;
        if *work < charge {
            return Err("stock-front coverage work limit; coverage unproven".into());
        }
        *work -= charge;
        let scale = bounds
            .into_iter()
            .flatten()
            .chain(self.vertices.iter().flatten().copied())
            .map(f64::abs)
            .fold(1., f64::max);
        let guard = 64. * tolerance + 8192. * f64::EPSILON * (1. + scale + self.distance);
        for i in 0..self.vertices.len() {
            if i.is_multiple_of(64) && cancel.is_cancelled() {
                return Err("convex coverage cancelled".into());
            }
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % self.vertices.len()];
            let edge = sub(b, a);
            let length = norm(edge);
            let normal = [edge[1] / length, -edge[0] / length];
            let nearest = std::array::from_fn(|k| bounds[usize::from(normal[k] < 0.)][k]);
            if dot(sub(nearest, a), normal) > self.distance + guard {
                return Ok(true);
            }
        }
        Ok(false)
    }
    /// A connected material body is wholly inside a hole if one of its points
    /// is inside and the ENTIRE hole boundary stays separated from the body.
    /// This also handles concave holes without treating endpoint tests as proof.
    pub fn excludes_domain(
        &self,
        domain: &PreparedRegion,
        floor: f64,
        tolerance: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("coverage.material");
        if cancel.is_cancelled() {
            return Err("convex coverage cancelled".into());
        }
        // Another component could request an island nested in this hole.
        if floor < self.floor || domain.definition().components.len() != 1 {
            return Ok(false);
        }
        'holes: for hole in &domain.definition().components[0].holes {
            let n = hole.vertices.len() as u64;
            let charge = n
                .saturating_add(1)
                .saturating_pow(2)
                .saturating_add(n.saturating_mul(self.vertices.len() as u64 + 1));
            if charge > *work {
                return Ok(false);
            }
            *work -= charge;
            if cancel.is_cancelled() {
                return Err("convex coverage cancelled".into());
            }
            let definition = PlanarRegion {
                version: 1,
                units: "mm".into(),
                name: "coverage exclusion".into(),
                boundary_error_mm: domain.definition().boundary_error_mm,
                source: serde_json::Value::Null,
                components: vec![Component {
                    outer: hole.clone(),
                    holes: vec![],
                }],
            };
            let Ok(exclusion) = definition.prepare(ConstructionOptions {
                tolerance_mm: tolerance,
            }) else {
                continue;
            };
            let strokes = Stroke::from_boundary(&exclusion.definition().components[0].outer);
            let magnitude = self
                .vertices
                .iter()
                .flatten()
                .copied()
                .chain(strokes.iter().flat_map(|s| {
                    s.a.into_iter()
                        .chain(s.b)
                        .chain(s.arc.into_iter().flat_map(|(c, _)| c))
                }))
                .map(f64::abs)
                .fold(1., f64::max);
            let guard = 64. * tolerance + 8192. * f64::EPSILON * (1. + magnitude + self.distance);
            let inside = exclusion.query(self.vertices[0])?;
            if !inside
                .signed_clearance_mm
                .is_some_and(|d| d > self.distance + inside.uncertainty_mm + guard)
            {
                continue;
            }
            for stroke in strokes {
                if cancel.is_cancelled() {
                    return Err("convex coverage cancelled".into());
                }
                // Boundary wholly inside C has positive separation from its
                // edges, but zero separation from the filled material body.
                if (0..self.vertices.len()).all(|i| {
                    orient(
                        self.vertices[i],
                        self.vertices[(i + 1) % self.vertices.len()],
                        stroke.a,
                    ) >= 0.
                }) {
                    continue 'holes;
                }
                let curve = stroke.curve(self.floor);
                if curve.prepare(tolerance).is_err() {
                    continue 'holes;
                }
                let path_error = crate::verification::path_error(
                    &Motion {
                        id: 0,
                        operation: String::new(),
                        tool: String::new(),
                        kind: MoveKind::Cut,
                        feed_mm_min: None,
                        curve,
                    },
                    tolerance,
                );
                let required = self.distance + inside.uncertainty_mm + guard + path_error;
                if !required.is_finite() {
                    continue 'holes;
                }
                for i in 0..self.vertices.len() {
                    if cancel.is_cancelled() {
                        return Err("convex coverage cancelled".into());
                    }
                    let edge = Stroke::line(
                        self.vertices[i],
                        self.vertices[(i + 1) % self.vertices.len()],
                    );
                    let separation = stroke_distance(edge, stroke);
                    if !separation.is_finite() || separation <= required {
                        continue 'holes;
                    }
                }
            }
            return Ok(true);
        }
        Ok(false)
    }
}
impl ConvexFront {
    pub fn new(
        points: Vec<Point>,
        material: [Point; 2],
        floor: f64,
        minimum_axis: f64,
        tolerance: f64,
    ) -> Option<Self> {
        if points.len() < 3
            || points.len() > 4096
            || points.iter().flatten().any(|x| !x.is_finite())
        {
            return None;
        }
        let vertices = hull(points);
        if vertices.len() < 3 {
            return None;
        }
        let mut normals = Vec::new();
        let mut longest = 0.;
        let mut edge = 0;
        for i in 0..vertices.len() {
            let d = sub(vertices[(i + 1) % vertices.len()], vertices[i]);
            let length = norm(d);
            if length <= 128. * tolerance {
                return None;
            }
            normals.push([d[1] / length, -d[0] / length]);
            if length > longest {
                longest = length;
                edge = i;
            }
        }
        let mut result = Self {
            vertices,
            normals,
            edge,
            distance: 0.,
            floor,
            minimum_axis,
            previous: None,
        };
        // Distance to a convex set is convex: rectangle vertices bound its
        // ENTIRE original material domain, not merely four sampled positions.
        result.distance = result.material_distance(material) + 64. * tolerance;
        Some(result)
    }
    fn material_distance(&self, material: [Point; 2]) -> f64 {
        let mut distance: f64 = 0.;
        for x in [material[0][0], material[1][0]] {
            for y in [material[0][1], material[1][1]] {
                distance = distance.max(self.point_distance([x, y]));
            }
        }
        distance
    }
    /// Before the first convex recipe, intersect its envelope with the current
    /// authoritative material rectangle left by completed native peeling.
    pub fn restrict_material(
        &mut self,
        material: [Point; 2],
        tolerance: f64,
    ) -> Result<(), String> {
        if self.has_commit() {
            return Err("cannot restart a committed convex front".into());
        }
        self.distance = self
            .distance
            .min(self.material_distance(material) + 64. * tolerance);
        Ok(())
    }
    fn point_distance(&self, p: Point) -> f64 {
        let mut inside = true;
        let mut distance = f64::INFINITY;
        for i in 0..self.vertices.len() {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % self.vertices.len()];
            inside &= orient(a, b, p) >= 0.;
            let d = sub(b, a);
            let t = (dot(sub(p, a), d) / dot(d, d)).clamp(0., 1.);
            distance = distance.min(norm(sub(p, [a[0] + t * d[0], a[1] + t * d[1]])));
        }
        if inside { 0. } else { distance }
    }
    #[allow(clippy::too_many_arguments)]
    pub fn prepare(
        &self,
        position: Vec3,
        radius: f64,
        minimum_radius: f64,
        max_angle: f64,
        boundary_budget: f64,
        tolerance: f64,
        right: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Option<ConvexContour>, String> {
        if cancel.is_cancelled() {
            return Err("convex front cancelled".into());
        }
        if position[2] != self.floor {
            return Err("convex front floor changed".into());
        }
        let scale = self
            .vertices
            .iter()
            .flatten()
            .map(|x| x.abs())
            .fold(radius.max(1.), f64::max);
        let guard = 64. * tolerance + 8192. * f64::EPSILON * (1. + scale + radius + self.distance);
        let width = radius * (1. - (max_angle.to_radians() / 2.).cos());
        let step = width - boundary_budget / 4. - 4. * guard;
        if step <= 4. * guard || step >= radius {
            return Ok(None);
        }
        let axis = (self.distance + radius - step).max(self.minimum_axis);
        let distance = axis - radius + guard;
        if distance >= self.distance - 32. * tolerance || axis < minimum_radius {
            return Ok(None);
        }
        let n = self.normals[self.edge];
        let a = self.vertices[self.edge];
        let b = self.vertices[(self.edge + 1) % self.vertices.len()];
        let edge_length = norm(sub(b, a));
        let t = if right { sub(a, b) } else { sub(b, a) };
        let t = [t[0] / edge_length, t[1] / edge_length];
        let origin = if right { b } else { a };
        let pad = 64. * tolerance;
        let old = self.previous.filter(|(p, _, _)| *p == position);
        let shifted = |start_axis: f64| -> Option<(f64, f64, f64)> {
            let shift = start_axis - axis;
            if shift <= 0. || shift >= radius {
                return None;
            }
            let side_radius = shift * (2. * radius - shift) / (2. * (radius - shift));
            let lead_radius = minimum_radius
                .max(radius / 2.)
                .max(shift / 2.)
                .max(1.05 * side_radius);
            let theta = (1. - shift / (2. * lead_radius)).clamp(-1., 1.).acos();
            Some((lead_radius, theta, 2. * lead_radius * theta.sin()))
        };
        let mut reset = true;
        let mut start_axis = self.distance + radius + 2. * pad;
        let mut u = 0.;
        if let Some((_, old_axis, old_u)) = old
            && let Some((_, _, length)) = shifted(old_axis)
            && old_u + length + 128. * tolerance < edge_length
        {
            reset = false;
            start_axis = old_axis;
            u = old_u;
        }
        let Some((lead_radius, theta, length)) = shifted(start_axis) else {
            return Ok(None);
        };
        if reset {
            u = (edge_length - length) / 2.;
        }
        if u <= 128. * tolerance || u + length >= edge_length - 128. * tolerance {
            return Ok(None);
        }
        let point = |axis: f64, u: f64| {
            [
                origin[0] + u * t[0] + axis * n[0],
                origin[1] + u * t[1] + axis * n[1],
                self.floor,
            ]
        };
        // A reset disk of radius r+pad is outside the AUTHORITATIVE old
        // convex envelope: its inward support is D+pad. Native history is not
        // inferred from height cells. A continuing lead uses the actual pose.
        let start = if reset {
            point(start_axis, u)
        } else {
            position
        };
        let heading = t[1].atan2(t[0]);
        let sign = if right { -1. } else { 1. };
        let first = candidate(start, heading, sign / lead_radius, lead_radius * theta);
        let second = candidate(
            first.end(),
            tangent(&first, 1.),
            -sign / lead_radius,
            lead_radius * theta,
        );
        let q = second.end();
        let seam = u + length;
        let expected = point(axis, seam);
        let deviation = (q[0] - expected[0]).hypot(q[1] - expected[1]);
        if deviation > guard / 8. {
            return Ok(None);
        }
        let mut contour = Vec::with_capacity(2 * self.vertices.len() + 1);
        let side_fits = |start: Vec3, end: Vec3, n: Point| {
            let d = [end[0] - start[0], end[1] - start[1]];
            let length = norm(d);
            if length <= 128. * tolerance {
                return false;
            }
            let direction = [d[0] / length, d[1] / length];
            let outward = if right {
                [-direction[1], direction[0]]
            } else {
                [direction[1], -direction[0]]
            };
            dot(outward, n) > 0.
                && dot(direction, n).abs() < (axis - self.distance - guard / 4.) / radius
        };
        let lifted = |v: Point, n: Point| [v[0] + axis * n[0], v[1] + axis * n[1], self.floor];
        let first_start = lifted(origin, n);
        let mut cursor = q;
        for k in 0..self.vertices.len() {
            if cancel.is_cancelled() {
                return Err("convex contour cancelled".into());
            }
            let i = if right {
                (self.edge + self.vertices.len() - k) % self.vertices.len()
            } else {
                (self.edge + k) % self.vertices.len()
            };
            let j = if right {
                (i + self.vertices.len() - 1) % self.vertices.len()
            } else {
                (i + 1) % self.vertices.len()
            };
            let vertex = if right {
                self.vertices[i]
            } else {
                self.vertices[(i + 1) % self.vertices.len()]
            };
            let end = lifted(vertex, self.normals[i]);
            if !side_fits(cursor, end, self.normals[i]) {
                return Ok(None);
            }
            contour.push(Curve::Line { start: cursor, end });
            let next = lifted(vertex, self.normals[j]);
            let ni = self.normals[i];
            let nj = self.normals[j];
            let sweep = (ni[0] * nj[1] - ni[1] * nj[0]).atan2(dot(ni, nj));
            if sweep.signum() != sign || sweep.abs() >= std::f64::consts::PI {
                return Ok(None);
            }
            contour.push(Curve::Arc {
                start: end,
                end: next,
                center: [vertex[0], vertex[1], self.floor],
                plane: Plane::Xy,
                sweep_rad: sweep,
            });
            cursor = next;
        }
        if norm(sub(
            [cursor[0], cursor[1]],
            [first_start[0], first_start[1]],
        )) > guard / 8.
        {
            return Ok(None);
        }
        if !side_fits(cursor, q, n) {
            return Ok(None);
        }
        contour.push(Curve::Line {
            start: cursor,
            end: q,
        });
        let leads = vec![first, second];
        let mut error: f64 = deviation;
        for curve in leads.iter().chain(&contour) {
            curve.prepare(tolerance)?;
            error = error.max(crate::verification::path_error(
                &Motion {
                    id: 1,
                    operation: "convex-front".into(),
                    tool: "convex-front".into(),
                    kind: MoveKind::Cut,
                    curve: curve.clone(),
                    feed_mm_min: Some(1.),
                },
                tolerance,
            ));
        }
        if error > guard / 8. {
            return Ok(None);
        }
        // Full support-plane cap, at EVERY line/vertex-arc position. The
        // collar encloses axis arithmetic/path errors. S leads are NOT covered
        // here: FrontDriver requires its original continuous joint checker.
        let cosine = (axis - self.distance - guard / 4.) / radius;
        if cosine <= 0. || cosine > 1. {
            return Ok(None);
        }
        let angle = 2. * cosine.acos().to_degrees();
        if angle > max_angle {
            return Ok(None);
        }
        let bounds = [
            std::array::from_fn(|k| {
                self.vertices
                    .iter()
                    .map(|v| v[k])
                    .fold(f64::INFINITY, f64::min)
                    - distance
            }),
            std::array::from_fn(|k| {
                self.vertices
                    .iter()
                    .map(|v| v[k])
                    .fold(f64::NEG_INFINITY, f64::max)
                    + distance
            }),
        ];
        Ok(Some(ConvexContour {
            start,
            heading,
            leads,
            contour,
            reset,
            angle,
            radius,
            tolerance,
            cursor: 0,
            distance,
            axis,
            seam,
            bounds,
        }))
    }
    pub fn committed(&mut self, recipe: &ConvexContour) -> Result<(), String> {
        if recipe.cursor != recipe.leads.len() + recipe.contour.len()
            || recipe.distance >= self.distance
            || recipe.start[2] != self.floor
        {
            return Err("convex contour incomplete or mismatched recipe".into());
        }
        self.distance = recipe.distance;
        self.previous = Some((
            recipe.contour.last().unwrap().end(),
            recipe.axis,
            recipe.seam,
        ));
        Ok(())
    }
    pub fn has_commit(&self) -> bool {
        self.previous.is_some()
    }
    pub fn material_envelope(&self) -> Option<ConvexMaterial> {
        (self.has_commit() && self.distance.is_finite() && self.distance >= 0.).then(|| {
            ConvexMaterial {
                vertices: self.vertices.clone(),
                distance: self.distance,
                floor: self.floor,
            }
        })
    }
}
impl ConvexContour {
    pub fn expects(&self, motion: &Motion, radius: f64, tolerance: f64) -> Result<bool, String> {
        if motion.kind == MoveKind::Rapid || motion.curve.start()[2] != motion.curve.end()[2] {
            return Ok(false);
        }
        if radius != self.radius || tolerance > self.tolerance {
            return Err("convex contour cutter/accuracy changed".into());
        }
        let expected = self
            .leads
            .iter()
            .chain(&self.contour)
            .nth(self.cursor)
            .ok_or("convex contour receipt exhausted")?;
        if &motion.curve != expected || !matches!(motion.kind, MoveKind::Lead | MoveKind::Cut) {
            return Err("convex contour whole native commit mismatch".into());
        }
        Ok(true)
    }
    pub fn advance(&mut self) -> bool {
        self.cursor += 1;
        self.cursor == self.leads.len() + self.contour.len()
    }
    pub fn certified_angle(&self, motion: &Motion) -> Option<f64> {
        (self.cursor >= self.leads.len()
            && self.contour.get(self.cursor - self.leads.len()) == Some(&motion.curve)
            && motion.kind == MoveKind::Cut)
            .then_some(self.angle)
    }
    pub fn envelope(&self) -> ([Point; 2], f64) {
        (self.bounds, self.start[2])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{PI, TAU};

    fn motion(curve: Curve, kind: MoveKind) -> Motion {
        Motion {
            id: 1,
            operation: "contour".into(),
            tool: "cutter".into(),
            kind,
            curve,
            feed_mm_min: Some(600.),
        }
    }

    fn coverage_domain(holes: Vec<crate::planar::Boundary>, error: f64) -> PreparedRegion {
        PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "coverage geometry".into(),
            boundary_error_mm: error,
            source: serde_json::Value::Null,
            components: vec![Component {
                outer: crate::planar::Boundary {
                    vertices: vec![
                        [-20., -20., 0.],
                        [20., -20., 0.],
                        [20., 20., 0.],
                        [-20., 20., 0.],
                    ],
                },
                holes,
            }],
        }
        .prepare(ConstructionOptions::default())
        .unwrap()
    }

    #[test]
    fn material_containment_checks_whole_lines_arcs_floor_and_uncertainty() {
        let square = |r: f64| crate::planar::Boundary {
            vertices: vec![[-r, -r, 0.], [r, -r, 0.], [r, r, 0.], [-r, r, 0.]],
        };
        let body = ConvexMaterial {
            vertices: hull(vec![[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]),
            distance: 0.2,
            floor: 5.,
        };
        let circle = PlanarRegion::circle("circular exclusion", [0., 0.], 2.).unwrap();
        for hole in [square(1.4), circle.components[0].outer.clone()] {
            let domain = coverage_domain(vec![hole], 0.);
            assert!(
                body.excludes_domain(&domain, 5., 1e-5, &mut 10000, &|| false)
                    .unwrap()
            );
            assert!(
                body.excludes_domain(&domain, 6., 1e-5, &mut 10000, &|| false)
                    .unwrap()
            );
            assert!(
                !body
                    .excludes_domain(&domain, 4.99999, 1e-5, &mut 10000, &|| false)
                    .unwrap()
            );
            assert!(
                !body
                    .excludes_domain(&domain, 5., 1e-5, &mut 0, &|| false)
                    .unwrap()
            );
            assert!(
                body.excludes_domain(&domain, 5., 1e-5, &mut 10000, &|| true)
                    .is_err()
            );
        }
        // All C vertices lie inside this circle, but their offset disks do not.
        let small = PlanarRegion::circle("tight circle", [0., 0.], 1.5).unwrap();
        let domain = coverage_domain(vec![small.components[0].outer.clone()], 0.);
        assert!(
            !body
                .excludes_domain(&domain, 5., 1e-5, &mut 10000, &|| false)
                .unwrap()
        );
        for (hole, error) in [(square(1.2), 0.), (square(1.2001), 0.), (square(1.3), 0.15)] {
            assert!(
                !body
                    .excludes_domain(
                        &coverage_domain(vec![hole], error),
                        5.,
                        1e-5,
                        &mut 10000,
                        &|| false
                    )
                    .unwrap()
            );
        }
    }

    #[test]
    fn material_containment_is_rigid_transform_invariant() {
        for (angle, shift) in [(0.41_f64, [0., 0.]), (-1.17, [10000., -20000.])] {
            let transform = |p: Point| {
                [
                    p[0] * angle.cos() - p[1] * angle.sin() + shift[0],
                    p[0] * angle.sin() + p[1] * angle.cos() + shift[1],
                ]
            };
            let body = ConvexMaterial {
                vertices: hull(
                    [[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]
                        .into_iter()
                        .map(transform)
                        .collect(),
                ),
                distance: 0.2,
                floor: 5.,
            };
            for (curved, radius, expected) in [
                (false, 1.4, true),
                (false, 1.2, false),
                (true, 2., true),
                (true, 1.5, false),
            ] {
                let hole = if curved {
                    PlanarRegion::circle("rotated circle", [0., 0.], radius)
                        .unwrap()
                        .components[0]
                        .outer
                        .clone()
                } else {
                    crate::planar::Boundary {
                        vertices: vec![
                            [-radius, -radius, 0.],
                            [radius, -radius, 0.],
                            [radius, radius, 0.],
                            [-radius, radius, 0.],
                        ],
                    }
                };
                let mut definition = coverage_domain(vec![hole], 0.).definition().clone();
                for component in &mut definition.components {
                    for boundary in
                        std::iter::once(&mut component.outer).chain(component.holes.iter_mut())
                    {
                        for vertex in &mut boundary.vertices {
                            let p = transform([vertex[0], vertex[1]]);
                            vertex[0] = p[0];
                            vertex[1] = p[1];
                        }
                    }
                }
                let domain = definition.prepare(ConstructionOptions::default()).unwrap();
                assert_eq!(
                    body.excludes_domain(&domain, 5., 1e-5, &mut 10000, &|| false)
                        .unwrap(),
                    expected,
                    "angle={angle} shift={shift:?} curved={curved} radius={radius}"
                );
            }
        }
    }

    #[test]
    fn material_containment_rejects_concave_gaps_and_nested_requested_components() {
        let body = ConvexMaterial {
            vertices: hull(vec![[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]),
            distance: 0.2,
            floor: 5.,
        };
        // Every hull vertex lies well inside the excluded U, and all hole
        // vertices lie outside C. Only whole-edge separation finds the gap.
        let notch = crate::planar::Boundary {
            vertices: vec![
                [-2., -2., 0.],
                [2., -2., 0.],
                [2., 2., 0.],
                [0.4, 2., 0.],
                [0.4, -1.5, 0.],
                [-0.4, -1.5, 0.],
                [-0.4, 2., 0.],
                [-2., 2., 0.],
            ],
        };
        let mut exclusion = coverage_domain(vec![], 0.).definition().clone();
        exclusion.components[0].outer = notch.clone();
        let exclusion = exclusion.prepare(ConstructionOptions::default()).unwrap();
        for p in &body.vertices {
            assert!(exclusion.query(*p).unwrap().signed_clearance_mm.unwrap() > 0.2);
        }
        let domain = coverage_domain(vec![notch], 0.);
        assert!(domain.query([0., 0.]).unwrap().signed_clearance_mm.unwrap() > 0.);
        assert!(
            !body
                .excludes_domain(&domain, 5., 1e-5, &mut 10000, &|| false)
                .unwrap()
        );

        let hole = PlanarRegion::circle("hole", [0., 0.], 3.).unwrap();
        let mut nested = coverage_domain(vec![hole.components[0].outer.clone()], 0.)
            .definition()
            .clone();
        nested.components.extend(
            PlanarRegion::circle("requested island", [0., 0.], 0.5)
                .unwrap()
                .components,
        );
        let nested = nested.prepare(ConstructionOptions::default()).unwrap();
        assert!(nested.query([0., 0.]).unwrap().signed_clearance_mm.unwrap() > 0.);
        assert!(
            !body
                .excludes_domain(&nested, 5., 1e-5, &mut 10000, &|| false)
                .unwrap()
        );
    }

    #[test]
    fn material_envelope_is_unavailable_until_the_entire_recipe_commits() {
        let mut front = ConvexFront::new(
            vec![[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]],
            [[-4., -4.], [4., 4.]],
            5.,
            1.1,
            1e-5,
        )
        .unwrap();
        assert!(front.material_envelope().is_none());
        let mut recipe = front
            .prepare([-6., 0., 5.], 1., 0.2, 82., 0.01, 1e-5, true, &|| false)
            .unwrap()
            .unwrap();
        assert!(front.committed(&recipe).is_err());
        assert!(front.material_envelope().is_none());
        finish(&mut front, &mut recipe);
        let material = front.material_envelope().unwrap();
        assert_eq!(material.floor, 5.);
        assert_eq!(material.distance, recipe.distance);
        assert!(
            front.committed(&recipe).is_err(),
            "replaying a completed recipe is not progress"
        );
    }
    fn angle_on(angle: f64, start: f64, sweep: f64) -> bool {
        let travel = if sweep > 0. {
            angle - start
        } else {
            start - angle
        };
        travel.rem_euclid(TAU) <= sweep.abs() + 1e-10
    }
    // Independent analytic extrema of v.n throughout an arc's rotating normal.
    fn maximum_dot(v: Point, start: f64, sweep: f64) -> f64 {
        let at = |a: f64| v[0] * a.cos() + v[1] * a.sin();
        let mut result = at(start).max(at(start + sweep));
        if angle_on(v[1].atan2(v[0]), start, sweep) {
            result = result.max(v[0].hypot(v[1]));
        }
        result
    }
    fn finish(front: &mut ConvexFront, recipe: &mut ConvexContour) {
        let old = front.distance;
        let curves: Vec<_> = recipe
            .leads
            .iter()
            .chain(&recipe.contour)
            .cloned()
            .collect();
        for (i, curve) in curves.into_iter().enumerate() {
            let kind = if i < recipe.leads.len() {
                MoveKind::Lead
            } else {
                MoveKind::Cut
            };
            let m = motion(curve, kind);
            assert!(recipe.expects(&m, 1., 1e-5).unwrap());
            assert_eq!(recipe.certified_angle(&m).is_some(), kind == MoveKind::Cut);
            let complete = recipe.advance();
            assert_eq!(complete, i + 1 == recipe.leads.len() + recipe.contour.len());
            // A partly consumed recipe cannot update its owner's material bound.
            assert_eq!(front.distance, old);
            if complete {
                front.committed(recipe).unwrap();
            }
        }
    }

    #[test]
    fn rotated_convex_contours_have_whole_curve_support_and_correct_material_side() {
        let shapes = [
            vec![[-6., -4.], [6., -4.], [6., 4.], [-6., 4.]],
            vec![[-6., -3.], [1., -5.], [6., -1.], [4., 5.], [-5., 4.]],
        ];
        for shape in shapes {
            for angle in [0_f64, 0.37, 1.21] {
                for shift in [[0., 0.], [12000., -8000.]] {
                    let points: Vec<Point> = shape
                        .iter()
                        .map(|p| {
                            [
                                shift[0] + p[0] * angle.cos() - p[1] * angle.sin(),
                                shift[1] + p[0] * angle.sin() + p[1] * angle.cos(),
                            ]
                        })
                        .collect();
                    let material = [
                        std::array::from_fn(|k| {
                            points.iter().map(|p| p[k]).fold(f64::INFINITY, f64::min) - 2.
                        }),
                        std::array::from_fn(|k| {
                            points
                                .iter()
                                .map(|p| p[k])
                                .fold(f64::NEG_INFINITY, f64::max)
                                + 2.
                        }),
                    ];
                    for right in [false, true] {
                        let mut front =
                            ConvexFront::new(points.clone(), material, 2., 1.101, 1e-5).unwrap();
                        let mut position = [material[0][0] - 1., material[0][1], 2.];
                        let mut count = 0;
                        let mut resets = 0;
                        while let Some(mut recipe) = front
                            .prepare(position, 1., 0.2, 82., 0.01, 1e-5, right, &|| false)
                            .unwrap()
                        {
                            count += 1;
                            assert!(count < 100);
                            resets += usize::from(recipe.reset);
                            assert!(recipe.distance < front.distance);
                            assert!(recipe.angle <= 82.);
                            // Normal-distance support bounds contact continuously,
                            // including every intermediate normal of every vertex arc.
                            let mut maximum_path_error: f64 = 0.;
                            let mut cursor = recipe.start;
                            for curve in recipe.leads.iter().chain(&recipe.contour) {
                                assert_eq!(curve.start(), cursor);
                                let error = crate::verification::path_error(
                                    &motion(curve.clone(), MoveKind::Cut),
                                    1e-5,
                                );
                                assert!(error < recipe.distance - (recipe.axis - 1.));
                                maximum_path_error = maximum_path_error.max(error);
                                cursor = curve.end();
                            }
                            assert_eq!(cursor, recipe.contour[0].start());
                            assert!(
                                2. * (recipe.axis - front.distance - maximum_path_error)
                                    .acos()
                                    .to_degrees()
                                    <= recipe.angle
                            );
                            let mut total_turn = 0.;
                            for curve in &recipe.contour {
                                match *curve {
                                    Curve::Line { start, end } => {
                                        let d = [end[0] - start[0], end[1] - start[1]];
                                        let length = d[0].hypot(d[1]);
                                        let n = if right {
                                            [-d[1] / length, d[0] / length]
                                        } else {
                                            [d[1] / length, -d[0] / length]
                                        };
                                        let support = points
                                            .iter()
                                            .map(|p| p[0] * n[0] + p[1] * n[1])
                                            .fold(f64::NEG_INFINITY, f64::max);
                                        for p in [start, end] {
                                            assert!(
                                                (p[0] * n[0] + p[1] * n[1] - support - recipe.axis)
                                                    .abs()
                                                    < 1e-7
                                            );
                                        }
                                    }
                                    Curve::Arc {
                                        start,
                                        end,
                                        center,
                                        sweep_rad,
                                        plane: Plane::Xy,
                                    } => {
                                        assert_eq!(sweep_rad.is_sign_negative(), right);
                                        assert!(sweep_rad.abs() < PI);
                                        total_turn += sweep_rad;
                                        for p in [start, end] {
                                            assert!(
                                                ((p[0] - center[0]).hypot(p[1] - center[1])
                                                    - recipe.axis)
                                                    .abs()
                                                    < 1e-7
                                            );
                                        }
                                        let start_angle =
                                            (start[1] - center[1]).atan2(start[0] - center[0]);
                                        for p in &points {
                                            assert!(
                                                maximum_dot(
                                                    [p[0] - center[0], p[1] - center[1]],
                                                    start_angle,
                                                    sweep_rad
                                                ) < 1e-7
                                            );
                                        }
                                    }
                                    _ => panic!("unexpected convex contour curve"),
                                }
                            }
                            assert!((total_turn.abs() - TAU).abs() < 1e-10);
                            position = cursor;
                            finish(&mut front, &mut recipe);
                        }
                        assert!(count > 5);
                        assert!(resets > 1, "finite face must eventually exhaust");
                        assert!(front.distance < 0.102);
                    }
                }
            }
        }
    }

    #[test]
    fn transition_uses_remaining_material_and_receipts_match_exact_curves() {
        let points = vec![[-6., -4.], [6., -4.], [6., 4.], [-6., 4.]];
        let mut front =
            ConvexFront::new(points, [[-20., -20.], [20., 20.]], 2., 1.101, 1e-5).unwrap();
        let old = front.distance;
        front
            .restrict_material([[-6.2, -4.2], [6.2, 4.2]], 1e-5)
            .unwrap();
        assert!(front.distance < 0.284 && front.distance < old);
        let restricted = front.distance;
        front
            .restrict_material([[-30., -30.], [30., 30.]], 1e-5)
            .unwrap();
        assert_eq!(front.distance, restricted);
        let mut recipe = front
            .prepare([0., -5., 2.], 1., 0.2, 82., 0.01, 1e-5, true, &|| false)
            .unwrap()
            .unwrap();
        let mut m = motion(recipe.leads[0].clone(), MoveKind::Lead);
        assert!(recipe.certified_angle(&m).is_none());
        assert!(recipe.expects(&m, 1.1, 1e-5).is_err());
        assert!(recipe.expects(&m, 1., 2e-5).is_err());
        m.curve = recipe.contour[0].clone();
        assert!(recipe.expects(&m, 1., 1e-5).is_err());
        assert!(recipe.certified_angle(&m).is_none());
        m.kind = MoveKind::Rapid;
        assert!(!recipe.expects(&m, 1., 1e-5).unwrap());
        assert!(!front.has_commit());
        finish(&mut front, &mut recipe);
        assert!(
            front
                .restrict_material([[-6.1, -4.1], [6.1, 4.1]], 1e-5)
                .is_err()
        );
        assert!(front.distance < 0.102);
    }

    #[test]
    fn degenerate_short_faces_and_cancelled_proposals_do_not_create_envelopes() {
        assert!(
            ConvexFront::new(
                vec![[0., 0.], [1., 0.], [2., 0.]],
                [[-1., -1.], [3., 1.]],
                2.,
                1.1,
                1e-5
            )
            .is_none()
        );
        assert!(
            ConvexFront::new(
                vec![[0., 0.], [1e-4, 0.], [0., 1e-4]],
                [[-1., -1.], [1., 1.]],
                2.,
                1.1,
                1e-5
            )
            .is_none()
        );
        let front = ConvexFront::new(
            vec![[0., 0.], [0.1, 0.], [0.1, 0.1], [0., 0.1]],
            [[-1., -1.], [1., 1.]],
            2.,
            1.1,
            1e-5,
        )
        .unwrap();
        assert!(
            front
                .prepare([0., 0., 2.], 1., 0.2, 82., 0.01, 1e-5, true, &|| false)
                .unwrap()
                .is_none()
        );
        assert!(
            front
                .prepare([0., 0., 2.], 1., 0.2, 82., 0.01, 1e-5, true, &|| true)
                .is_err()
        );
        assert!(!front.has_commit());
    }
    #[test]
    fn opposite_curvature_leads_pass_continuous_checks_against_real_sweeps() {
        use crate::engagement::{
            ClearedSweep, EngagementOptions, EngagementScene, EngagementStatus,
        };
        use crate::stock::StockSpec;
        for right in [false, true] {
            for angle in [0_f64, 0.39] {
                let points = [[-6., -3.], [1., -5.], [6., -1.], [4., 5.], [-5., 4.]].map(|p| {
                    [
                        p[0] * angle.cos() - p[1] * angle.sin(),
                        p[0] * angle.sin() + p[1] * angle.cos(),
                    ]
                });
                let material = [[-9., -9.], [9., 9.]];
                let mut front =
                    ConvexFront::new(points.to_vec(), material, 2., 1.101, 1e-5).unwrap();
                let mut scene = EngagementScene::new(
                    StockSpec {
                        version: 1,
                        units: "mm".into(),
                        min_mm: [-9., -9., 0.],
                        max_mm: [9., 9., 3.],
                        cell_size_mm: 0.5,
                        max_cells: 10000,
                    },
                    vec![],
                )
                .unwrap();
                let mut position = [-10., 0., 2.];
                let mut resets = 0;
                let mut count = 0;
                while let Some(mut recipe) = front
                    .prepare(position, 1., 0.2, 82., 0.01, 1e-5, right, &|| false)
                    .unwrap()
                {
                    count += 1;
                    assert!(count < 100);
                    resets += usize::from(recipe.reset);
                    for (i, curve) in recipe.leads.iter().chain(&recipe.contour).enumerate() {
                        let m = motion(
                            curve.clone(),
                            if i < recipe.leads.len() {
                                MoveKind::Lead
                            } else {
                                MoveKind::Cut
                            },
                        );
                        if m.kind == MoveKind::Lead {
                            let checked = scene
                                .check_front(
                                    &m,
                                    1.,
                                    82.,
                                    right,
                                    EngagementOptions {
                                        tolerance_mm: 1e-5,
                                        resolution_mm: 0.001,
                                        max_nodes: 8192,
                                        max_work: 2_000_000,
                                        ..Default::default()
                                    },
                                    &|| false,
                                )
                                .unwrap();
                            assert_eq!(
                                checked.status,
                                EngagementStatus::Bounded,
                                "right={right}, rotation={angle}, contour={count}, lead={i}, {checked:?}"
                            );
                        }
                        // Only actual full curve supports, no synthetic hull or
                        // rectangular-shell hints in this independent lead check.
                        scene
                            .append(ClearedSweep::from_motion(&m, 1., 1e-5).unwrap())
                            .unwrap();
                    }
                    position = recipe.contour.last().unwrap().end();
                    finish(&mut front, &mut recipe);
                }
                assert!(resets > 1);
                assert!(front.distance < 0.102);
            }
        }
    }

    #[test]
    fn concavities_and_spaces_between_islands_remain_in_original_coverage_domain() {
        use crate::{
            planar::Boundary,
            pocket::{Operation, PocketPlan},
            stream::StreamManifest,
        };
        let m: StreamManifest = serde_json::from_str(include_str!(
            "../../fixtures/operations/adaptive-pocket.json"
        ))
        .unwrap();
        let Operation::AdaptivePocket(op) = m.operations[0].clone() else {
            panic!("adaptive fixture")
        };
        let boundary = |points: &[[f64; 2]]| Boundary {
            vertices: points.iter().map(|p| [p[0], p[1], 0.]).collect(),
        };
        let cases = [
            (
                vec![boundary(&[
                    [8., 8.],
                    [18., 8.],
                    [18., 11.],
                    [11., 11.],
                    [11., 18.],
                    [8., 18.],
                ])],
                [13.5, 13.5],
            ),
            (
                vec![
                    boundary(&[[6., 8.], [9., 8.], [9., 18.], [6., 18.]]),
                    boundary(&[[17., 8.], [20., 8.], [20., 18.], [17., 18.]]),
                ],
                [13., 13.],
            ),
        ];
        for (holes, gap) in cases {
            let mut pocket = op.pocket.clone();
            pocket.region.components[0].outer =
                boundary(&[[0., 0.], [26., 0.], [26., 26.], [0., 26.]]);
            pocket.region.components[0].holes = holes;
            let plan = PocketPlan::prepare_guides(
                pocket,
                &m.tools[0],
                m.stock.height().unwrap(),
                m.initial_position_mm,
                1,
                Some(2.2),
                &|| false,
            )
            .unwrap();
            let front = ConvexFront::new(
                plan.front_obstacles([3., 3.]).unwrap(),
                [[0., 0.], [26., 26.]],
                8.,
                plan.cutter_axis_clearance(2.),
                1e-5,
            )
            .unwrap();
            assert_eq!(
                front.point_distance(gap),
                0.,
                "proposal hull encloses the accessible gap"
            );
            let (_, domain) = plan.front_component_region([3., 3.], 2.19).unwrap();
            let query = domain.query(gap).unwrap();
            assert!(
                query.signed_clearance_mm.unwrap() > query.uncertainty_mm,
                "original coverage must still require material inside the convex hull"
            );
            let material = ConvexMaterial {
                vertices: front.vertices.clone(),
                distance: 0.1,
                floor: 8.,
            };
            assert!(
                !material
                    .excludes_domain(&domain, 8., 1e-5, &mut 100000, &|| false)
                    .unwrap()
            );
        }
    }
    #[test]
    fn committed_material_support_planes_exclude_whole_boxes_only_at_valid_floors() {
        for angle in [0f64, 0.37, 1.6] {
            for shift in [[0., 0.], [100_000., -200_000.]] {
                let transform = |p: Point| {
                    [
                        shift[0] + p[0] * angle.cos() - p[1] * angle.sin(),
                        shift[1] + p[0] * angle.sin() + p[1] * angle.cos(),
                    ]
                };
                let body = ConvexMaterial {
                    vertices: [[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]
                        .map(transform)
                        .to_vec(),
                    distance: 0.2,
                    floor: 5.,
                };
                for (x, expected) in [
                    (1.3, true),
                    (1.2, false),
                    (1.19, false),
                    (0., false),
                    (-1.3, true),
                ] {
                    let p = transform([x, 0.]);
                    let bounds = [p.map(|v| v - 0.01), p.map(|v| v + 0.01)];
                    assert_eq!(
                        body.excludes_box(bounds, 5., 1e-5, &mut 100, &|| false)
                            .unwrap(),
                        expected
                    );
                    assert!(
                        !body
                            .excludes_box(bounds, 4., 1e-5, &mut 100, &|| false)
                            .unwrap()
                    );
                    assert!(
                        body.excludes_box(bounds, 5., 1e-5, &mut 0, &|| false)
                            .is_err()
                    );
                    assert!(
                        body.excludes_box(bounds, 5., 1e-5, &mut 100, &|| true)
                            .is_err()
                    );
                }
                // Centre outside is insufficient when the rectangle crosses C.
                let p = transform([1.3, 0.]);
                let crossing = [p.map(|v| v - 1.), p.map(|v| v + 1.)];
                assert!(
                    !body
                        .excludes_box(crossing, 5., 1e-5, &mut 100, &|| false)
                        .unwrap()
                );
            }
        }
    }
}
