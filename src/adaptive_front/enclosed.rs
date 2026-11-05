//! Growing convex pocket pieces from a real full floor seed. The owner binds
//! every completed contour to exact native sweeps before advancing its scale.
use super::convex::hull;
use crate::{
    Point,
    motion::{Curve, Motion, MoveKind, Plane, Vec3},
    planar::PlanarRegion,
    pocket::Stroke,
    stock::CancelSignal,
};
use robust::{Coord, orient2d};
use std::f64::consts::{PI, TAU};

fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}
fn length(a: Point) -> f64 {
    a[0].hypot(a[1])
}
fn orientation(a: Point, b: Point, c: Point) -> f64 {
    orient2d(
        Coord { x: a[0], y: a[1] },
        Coord { x: b[0], y: b[1] },
        Coord { x: c[0], y: c[1] },
    )
}
fn turn(a: Point, b: Point, ccw: bool) -> f64 {
    let raw = (a[0] * b[1] - a[1] * b[0]).atan2(a[0] * b[0] + a[1] * b[1]);
    if ccw {
        raw.rem_euclid(TAU)
    } else {
        -(-raw).rem_euclid(TAU)
    }
}

pub(super) fn growth_step(
    r: f64,
    s: f64,
    guard: f64,
    max_angle: f64,
    tolerance: f64,
) -> Option<(f64, f64)> {
    let clear = r + s - guard;
    let half = max_angle.to_radians() / 2.;
    let mut delta = ((clear * clear - r * r * half.sin().powi(2)).max(0.).sqrt()
        - r * half.cos()
        - s
        - 4. * guard)
        .min(s / 4.);
    let mut angle = 0.;
    for _ in 0..16 {
        if delta <= 32. * tolerance {
            return None;
        }
        let a = s + delta + guard / 8.;
        angle = 2.
            * ((clear * clear - a * a - r * r) / (2. * a * r))
                .clamp(-1., 1.)
                .acos();
        let deviation = ((delta + guard / 8.) / s).asin();
        if angle.to_degrees() <= max_angle && angle / 2. + deviation < PI / 2. - guard / s {
            break;
        }
        delta /= 2.;
    }
    if angle.to_degrees() > max_angle
        || angle / 2. + ((delta + guard / 8.) / s).asin() >= PI / 2. - guard / s
    {
        return None;
    }
    Some((delta, angle))
}

struct DiskStage {
    centre: Point,
    radius: f64,
    direction: Point,
    distance: f64,
}

pub(super) struct EnclosedFront {
    disks: Vec<DiskStage>,
    pieces: Vec<[Point; 3]>,
    seed: Point,
    floor: f64,
    radius: f64,
    cutter: f64,
    guard: f64,
    piece: usize,
    scale: f64,
    coverage_radius: Option<f64>,
    tolerance: f64,
}
pub(super) struct EnclosedContour {
    pub start: Vec3,
    pub heading: f64,
    pub reset: bool,
    pub leads: Vec<Curve>,
    pub contour: Vec<Curve>,
    angle: f64,
    piece: usize,
    scale: f64,
    cutter: f64,
    tolerance: f64,
    cursor: usize,
}
impl EnclosedFront {
    fn new_circular(
        region: &PlanarRegion,
        seed_motion: &Motion,
        cutter: f64,
        minimum: f64,
        tolerance: f64,
    ) -> Option<Self> {
        let Curve::Arc {
            start,
            end,
            center,
            plane: Plane::Xy,
            sweep_rad,
        } = seed_motion.curve
        else {
            return None;
        };
        if seed_motion.kind != MoveKind::Cut || start != end || sweep_rad.abs() != TAU {
            return None;
        }
        let seed = [center[0], center[1]];
        let radius = length(sub([start[0], start[1]], seed));
        let guard = 64. * tolerance
            + 8192. * f64::EPSILON * (1. + seed[0].abs() + seed[1].abs() + radius + cutter);
        if radius > cutter || radius + guard / 8. < minimum || radius <= 4. * guard {
            return None;
        }
        let strokes = Stroke::from_boundary(&region.components[0].outer);
        let (centre, _) = strokes.first()?.arc?;
        let boundary_radius = length(sub(strokes[0].a, centre));
        let mut sweep = 0.;
        if !strokes.iter().all(|s| {
            s.arc.is_some_and(|(c, a)| {
                sweep += a;
                length(sub(c, centre)) <= guard / 8.
                    && (length(sub(s.a, centre)) - boundary_radius).abs() <= guard / 8.
            })
        }) || (sweep.abs() - TAU).abs() > 1e-10
        {
            return None;
        }
        // The centre region has already reserved the minimum orbit. Recover
        // the final axis radius; original-source tube checks still admit it.
        let target = boundary_radius + minimum;
        let d = length(sub(centre, seed));
        if target + guard / 8. < radius + d {
            return None;
        }
        let direction = if d > 0. {
            [(centre[0] - seed[0]) / d, (centre[1] - seed[1]) / d]
        } else {
            [1., 0.]
        };
        let first = ((target - radius + d) / 2.).max(0.);
        let second = (first - d).max(0.);
        let mut disks = Vec::new();
        if first > 0. {
            disks.push(DiskStage {
                centre: seed,
                radius,
                direction,
                distance: first,
            });
        }
        if second > 0. {
            disks.push(DiskStage {
                centre: [
                    seed[0] + direction[0] * first,
                    seed[1] + direction[1] * first,
                ],
                radius: radius + first,
                direction: [-direction[0], -direction[1]],
                distance: second,
            });
        }
        if disks.is_empty() {
            return None;
        }
        Some(Self {
            disks,
            pieces: Vec::new(),
            seed,
            floor: start[2],
            radius,
            cutter,
            guard,
            piece: 0,
            scale: 0.,
            coverage_radius: None,
            tolerance,
        })
    }

    fn prepare_circular(
        &self,
        position: Vec3,
        max_angle: f64,
        tolerance: f64,
        right: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Option<EnclosedContour>, String> {
        let Some(stage) = self.disks.get(self.piece) else {
            return Ok(None);
        };
        if position[2] != self.floor {
            return Err("circular front floor changed".into());
        }
        let old_radius = stage.radius + stage.distance * self.scale;
        let Some((delta, angle)) =
            growth_step(self.cutter, old_radius, self.guard, max_angle, tolerance)
        else {
            return Ok(None);
        };
        // The inner unswept core must remain inside the preceding clear disk,
        // including when the growing axis radius is larger than the cutter.
        let scale = (self.scale + delta.min(self.cutter) / (2. * stage.distance)).min(1.);
        let centre_at = |scale: f64| {
            [
                stage.centre[0] + stage.direction[0] * stage.distance * scale,
                stage.centre[1] + stage.direction[1] * stage.distance * scale,
                self.floor,
            ]
        };
        let old_centre = centre_at(self.scale);
        let centre = centre_at(scale);
        let start = [
            stage.centre[0] - stage.direction[0] * stage.radius,
            stage.centre[1] - stage.direction[1] * stage.radius,
            self.floor,
        ];
        let old_offset = [position[0] - old_centre[0], position[1] - old_centre[1]];
        if (length(old_offset) - old_radius).abs() > self.guard / 8. {
            return Ok(None);
        }
        let seam = [-stage.direction[0], -stage.direction[1]];
        let mut leads = Vec::new();
        if position != start {
            let mut sweep = turn(old_offset, seam, right);
            if sweep.abs() < 1e-10 {
                sweep = if right { TAU } else { -TAU };
            }
            leads.push(Curve::Arc {
                start: position,
                end: start,
                center: old_centre,
                plane: Plane::Xy,
                sweep_rad: sweep,
            });
        }
        let contour = vec![Curve::Arc {
            start,
            end: start,
            center: centre,
            plane: Plane::Xy,
            sweep_rad: if right { TAU } else { -TAU },
        }];
        for curve in leads.iter().chain(&contour) {
            if cancel.is_cancelled() {
                return Err("circular front cancelled".into());
            }
            curve.prepare(tolerance)?;
            let m = Motion {
                id: 0,
                operation: String::new(),
                tool: String::new(),
                kind: MoveKind::Cut,
                curve: curve.clone(),
                feed_mm_min: None,
            };
            if crate::verification::path_error(&m, tolerance) > self.guard / 8. {
                return Ok(None);
            }
        }
        Ok(Some(EnclosedContour {
            start,
            heading: seam[1].atan2(seam[0]) + if right { PI / 2. } else { -PI / 2. },
            reset: false,
            leads,
            contour,
            angle: angle.to_degrees(),
            piece: self.piece,
            scale,
            cutter: self.cutter,
            tolerance,
            cursor: 0,
        }))
    }

    pub fn new(
        region: &PlanarRegion,
        seed_motion: &Motion,
        cutter: f64,
        minimum_radius: f64,
        tolerance: f64,
    ) -> Option<Self> {
        if region.components.len() != 1 || !region.components[0].holes.is_empty() {
            return None;
        }
        let boundary = &region.components[0].outer;
        if boundary.vertices.iter().any(|p| p[2] != 0.) {
            return Self::new_circular(region, seed_motion, cutter, minimum_radius, tolerance);
        }
        if boundary.vertices.len() > 4096 || boundary.vertices.iter().any(|p| p[2] != 0.) {
            return None;
        }
        let vertices = hull(boundary.vertices.iter().map(|p| [p[0], p[1]]).collect());
        // Collinear/concave input declines to the original full coverage planner.
        if vertices.len() < 3 || vertices.len() != boundary.vertices.len() {
            return None;
        }
        let Curve::Arc {
            start,
            end,
            center,
            plane: Plane::Xy,
            sweep_rad,
        } = seed_motion.curve
        else {
            return None;
        };
        if seed_motion.kind != MoveKind::Cut || start != end || sweep_rad.abs() != TAU {
            return None;
        }
        let seed = [center[0], center[1]];
        let seed_radius = (start[0] - seed[0]).hypot(start[1] - seed[1]);
        let roundoff = 256.
            * f64::EPSILON
            * (1. + seed[0].abs() + seed[1].abs() + seed_radius + minimum_radius);
        if seed_radius > cutter || seed_radius + roundoff < minimum_radius {
            return None;
        }
        let radius = minimum_radius.min(seed_radius);
        let magnitude = vertices
            .iter()
            .flatten()
            .map(|v| v.abs())
            .fold(cutter.max(1.), f64::max);
        let guard = 64. * tolerance + 8192. * f64::EPSILON * (1. + magnitude + cutter + radius);
        if radius <= 4. * guard {
            return None;
        }
        let mut pieces = Vec::new();
        for i in 0..vertices.len() {
            let a = vertices[i];
            let b = vertices[(i + 1) % vertices.len()];
            let area = orientation(a, b, seed);
            if area < 0. {
                return None;
            }
            if area == 0. {
                continue;
            }
            if area <= 128. * tolerance * length(sub(b, a)) {
                return None;
            }
            pieces.push([seed, a, b]);
        }
        if pieces.is_empty() {
            return None;
        }
        Some(Self {
            disks: Vec::new(),
            pieces,
            seed,
            floor: start[2],
            radius,
            cutter,
            guard,
            piece: 0,
            scale: 0.,
            // The actual seed must cover its central core even after erosion
            // by the sweep/path reserve. A nominal circle of cutter radius
            // alone does not prove a filled disk. Near equality keeps fallback.
            coverage_radius: (seed_radius + 2. * guard < cutter
                && crate::verification::path_error(seed_motion, tolerance) <= guard / 8.)
                .then_some(radius + cutter - guard),
            tolerance,
        })
    }
    pub fn prepare(
        &self,
        position: Vec3,
        max_angle: f64,
        tolerance: f64,
        right: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Option<EnclosedContour>, String> {
        if cancel.is_cancelled() {
            return Err("enclosed contour cancelled".into());
        }
        if !self.disks.is_empty() {
            return self.prepare_circular(position, max_angle, tolerance, right, cancel);
        }
        let Some(piece) = self.pieces.get(self.piece) else {
            return Ok(None);
        };
        if position[2] != self.floor {
            return Err("enclosed contour floor changed".into());
        }
        let r = self.cutter;
        let s = self.radius;
        let Some((delta, angle)) = growth_step(r, s, self.guard, max_angle, tolerance) else {
            return Ok(None);
        };
        let longest = piece
            .iter()
            .map(|p| length(sub(*p, self.seed)))
            .fold(0., f64::max);
        let scale = (self.scale + delta / longest).min(1.);
        let vertices = piece.map(|v| {
            [
                self.seed[0] + scale * (v[0] - self.seed[0]),
                self.seed[1] + scale * (v[1] - self.seed[1]),
            ]
        });
        let mut normals = [[0.; 2]; 3];
        for i in 0..3 {
            let d = sub(piece[(i + 1) % 3], piece[i]);
            let l = length(d);
            if l * scale <= 128. * tolerance {
                return Ok(None);
            }
            normals[i] = [d[1] / l, -d[0] / l];
        }
        // Anchor at the real seed. Its corner and seam stay fixed throughout
        // homothetic growth, so successive contours meet with the same tangent.
        let seam = [normals[2][0] + normals[0][0], normals[2][1] + normals[0][1]];
        let l = length(seam);
        if l < 1e-10 {
            return Ok(None);
        }
        let seam = [seam[0] / l, seam[1] / l];
        let lifted = |v: Point, n: Point| [v[0] + s * n[0], v[1] + s * n[1], self.floor];
        let start = lifted(self.seed, seam);
        let heading = seam[1].atan2(seam[0]) + if right { PI / 2. } else { -PI / 2. };
        let mut contour = Vec::with_capacity(8);
        let mut cursor = start;
        let first = if right { 0 } else { 2 };
        let first_end = lifted(self.seed, normals[first]);
        contour.push(Curve::Arc {
            start: cursor,
            end: first_end,
            center: [self.seed[0], self.seed[1], self.floor],
            plane: Plane::Xy,
            sweep_rad: turn(seam, normals[first], right),
        });
        cursor = first_end;
        for k in 0..3 {
            if cancel.is_cancelled() {
                return Err("enclosed contour cancelled".into());
            }
            let edge = if right { k } else { 2 - k };
            let vertex = if right { (edge + 1) % 3 } else { edge };
            let end = lifted(vertices[vertex], normals[edge]);
            let tangent = [end[0] - cursor[0], end[1] - cursor[1]];
            let tangent_length = length(tangent);
            let tilt = ((tangent[0] * normals[edge][0] + tangent[1] * normals[edge][1])
                / tangent_length)
                .abs()
                .clamp(0., 1.)
                .asin();
            if !tilt.is_finite()
                || angle / 2. + ((delta + self.guard / 8.) / s).asin() + tilt
                    >= PI / 2. - self.guard / s
            {
                return Ok(None);
            }
            contour.push(Curve::Line { start: cursor, end });
            cursor = end;
            let next_normal = if k == 2 {
                seam
            } else {
                normals[if right {
                    (edge + 1) % 3
                } else {
                    (edge + 2) % 3
                }]
            };
            let next = if k == 2 {
                start
            } else {
                lifted(vertices[vertex], next_normal)
            };
            contour.push(Curve::Arc {
                start: cursor,
                end: next,
                center: [vertices[vertex][0], vertices[vertex][1], self.floor],
                plane: Plane::Xy,
                sweep_rad: turn(normals[edge], next_normal, right),
            });
            cursor = next;
        }
        let mut leads = Vec::new();
        let reset = ((position[0] - self.seed[0]).hypot(position[1] - self.seed[1]) - s).abs()
            > self.guard / 8.;
        if !reset && position != start {
            let n = [
                (position[0] - self.seed[0]) / s,
                (position[1] - self.seed[1]) / s,
            ];
            let mut sweep = turn(n, seam, right);
            if sweep.abs() < 1e-10 {
                sweep = if right { TAU } else { -TAU };
            }
            leads.push(Curve::Arc {
                start: position,
                end: start,
                center: [self.seed[0], self.seed[1], self.floor],
                plane: Plane::Xy,
                sweep_rad: sweep,
            });
        }
        for curve in leads.iter().chain(&contour) {
            curve.prepare(tolerance)?;
            let m = Motion {
                id: 1,
                operation: "enclosed".into(),
                tool: "cutter".into(),
                kind: MoveKind::Cut,
                curve: curve.clone(),
                feed_mm_min: Some(1.),
            };
            if crate::verification::path_error(&m, tolerance) > self.guard / 8. {
                return Ok(None);
            }
        }
        Ok(Some(EnclosedContour {
            start,
            heading,
            reset,
            leads,
            contour,
            angle: angle.to_degrees(),
            piece: self.piece,
            scale,
            cutter: r,
            tolerance,
            cursor: 0,
        }))
    }
    /// Coverage only, after ALL native fan contours commit. The seed's filled
    /// disk starts the induction. For successive convex triangles A subset B,
    /// Hausdorff growth is <=delta<=orbit/4. Their rounded boundaries are swept
    /// with cutter reserve: the new exterior collar is swept, while B's interior
    /// is within delta of A and already clear. The possible inward unswept collar
    /// remains in A offset (orbit+cutter-guard), since delta<2*(cutter-guard).
    /// Thus every completed fan triangle offset coverage_radius is cleared.
    /// Dilation distributes over the complete fan union. Construction uncertainty
    /// and an additional guard must fit before this proves the requested domain.
    /// No partial fan, circle stage, proposed path or sampled pose supplies proof.
    pub fn covers_offset(
        &self,
        inflation: f64,
        error: f64,
        floor: f64,
        cutter: f64,
        tolerance: f64,
    ) -> bool {
        !self.pieces.is_empty()
            && self.disks.is_empty()
            && self.piece == self.pieces.len()
            && self.scale == 0.
            && floor == self.floor
            && cutter == self.cutter
            && tolerance == self.tolerance
            && inflation.is_finite()
            && inflation >= 0.
            && error.is_finite()
            && error >= 0.
            && self
                .coverage_radius
                .is_some_and(|radius| inflation + error + self.guard < radius)
    }
    pub fn committed(&mut self, recipe: &EnclosedContour) -> Result<(), String> {
        if recipe.piece != self.piece
            || recipe.cursor != recipe.leads.len() + recipe.contour.len()
            || recipe.scale <= self.scale
        {
            return Err("enclosed contour incomplete or mismatched recipe".into());
        }
        self.scale = recipe.scale;
        if self.scale == 1. {
            self.piece += 1;
            self.scale = 0.;
        }
        Ok(())
    }
}
impl EnclosedContour {
    pub fn expects(&self, motion: &Motion, radius: f64, tolerance: f64) -> Result<bool, String> {
        if motion.kind == MoveKind::Rapid || motion.curve.start()[2] != motion.curve.end()[2] {
            return Ok(false);
        }
        if radius != self.cutter || tolerance > self.tolerance {
            return Err("enclosed cutter/accuracy changed".into());
        }
        let expected = self
            .leads
            .iter()
            .chain(&self.contour)
            .nth(self.cursor)
            .ok_or("enclosed recipe exhausted")?;
        if &motion.curve != expected || !matches!(motion.kind, MoveKind::Lead | MoveKind::Cut) {
            return Err("enclosed whole native commit mismatch".into());
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        engagement::{ClearedSweep, EngagementOptions, EngagementScene, EngagementStatus},
        planar::{Boundary, Component},
        stock::StockSpec,
    };
    fn motion(curve: Curve) -> Motion {
        Motion {
            id: 1,
            operation: "pocket".into(),
            tool: "cutter".into(),
            kind: MoveKind::Cut,
            curve,
            feed_mm_min: Some(600.),
        }
    }
    fn region(points: Vec<Point>) -> PlanarRegion {
        PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "pocket".into(),
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
            components: vec![Component {
                outer: Boundary {
                    vertices: points.into_iter().map(|p| [p[0], p[1], 0.]).collect(),
                },
                holes: vec![],
            }],
        }
    }
    #[test]
    fn circular_growth_clears_offset_and_centred_seeds_with_continuous_contact() {
        use crate::{stock::Stock, stream::StreamManifest};
        for right in [false, true] {
            for (offset, target) in [(0.9, 2.), (0., 2.), (1., 5.)] {
                let mut m: StreamManifest = serde_json::from_str(include_str!(
                    "../../fixtures/operations/adaptive-pocket.json"
                ))
                .unwrap();
                let shift = if right { [0., 0.] } else { [100., -80.] };
                for (k, v) in shift.iter().enumerate() {
                    m.stock.height_mut().unwrap().min_mm[k] += v;
                    m.stock.height_mut().unwrap().max_mm[k] += v;
                }
                m.tools[0].id = "cutter".into();
                let centre = [13. + shift[0], 13. + shift[1]];
                let angle = 0.41_f64;
                let seed = [
                    centre[0] + offset * angle.cos(),
                    centre[1] + offset * angle.sin(),
                ];
                let start = [seed[0], seed[1] + 1., 9.];
                let mut prior = motion(Curve::Arc {
                    start,
                    end: start,
                    center: [seed[0], seed[1], 9.],
                    plane: Plane::Xy,
                    sweep_rad: if right { TAU } else { -TAU },
                });
                let minimum = if right { 1. } else { 0.25 };
                let region = PlanarRegion::circle("centre disk", centre, target - minimum).unwrap();
                let mut front = EnclosedFront::new(&region, &prior, 2., minimum, 1e-5).unwrap();
                assert_eq!(front.disks.len(), 2);
                let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
                let tool = m.tools[0].prepare().unwrap();
                let delta = stock
                    .preview_sweep(&tool, &prior, 1., m.sweep, &|| false)
                    .unwrap();
                stock.apply_delta(&delta).unwrap();
                let mut scene = EngagementScene::new(
                    m.stock.height().unwrap().clone(),
                    vec![ClearedSweep::from_motion(&prior, 2., 1e-5).unwrap()],
                )
                .unwrap();
                let mut count = 0;
                while let Some(mut recipe) = front
                    .prepare(prior.curve.end(), 90., 1e-5, right, &|| false)
                    .unwrap()
                {
                    assert!(!recipe.reset);
                    assert!(front.committed(&recipe).is_err());
                    for (i, curve) in recipe
                        .leads
                        .iter()
                        .chain(&recipe.contour)
                        .cloned()
                        .collect::<Vec<_>>()
                        .into_iter()
                        .enumerate()
                    {
                        let mut next = motion(curve);
                        if i < recipe.leads.len() {
                            next.kind = MoveKind::Lead;
                        }
                        assert_eq!(next.curve.start(), prior.curve.end());
                        let a = super::super::tangent(&prior.curve, 1.);
                        let b = super::super::tangent(&next.curve, 0.);
                        assert!((a - b).sin().abs() < 1e-8 && (a - b).cos() > 0.);
                        let Curve::Arc { start, center, .. } = next.curve else {
                            unreachable!()
                        };
                        let axis = length(sub([start[0], start[1]], [center[0], center[1]]));
                        assert!(
                            axis + length(sub([center[0], center[1]], centre)) <= target + 1e-7
                        );
                        assert!(axis + 1e-7 >= minimum);
                        let checked = scene
                            .check_front(
                                &next,
                                2.,
                                90.,
                                right,
                                EngagementOptions::default(),
                                &|| false,
                            )
                            .unwrap();
                        assert_eq!(
                            checked.status,
                            EngagementStatus::Bounded,
                            "right={right} offset={offset} target={target}: {next:?}; {checked:?}"
                        );
                        assert!(recipe.expects(&next, 2., 1e-5).unwrap());
                        assert!(recipe.expects(&next, 1.9, 1e-5).is_err());
                        assert!(recipe.expects(&next, 2., 1e-4).is_err());
                        let delta = stock
                            .preview_sweep(&tool, &next, 1., m.sweep, &|| false)
                            .unwrap();
                        stock.apply_delta(&delta).unwrap();
                        scene
                            .append(ClearedSweep::from_motion(&next, 2., 1e-5).unwrap())
                            .unwrap();
                        recipe.advance();
                        prior = next;
                    }
                    front.committed(&recipe).unwrap();
                    assert!(front.committed(&recipe).is_err());
                    count += 1;
                    assert!(count < 100);
                }
                assert_eq!(front.piece, front.disks.len());
                assert!(count > 2);
                let mut cleared = 0;
                for (i, h) in stock.cells().iter().enumerate() {
                    let d = length(sub(stock.cell_center(i).unwrap(), centre));
                    if d < target + 2. - 0.4 {
                        assert!(h.upper_mm <= 9. + 4. * m.sweep.tolerance_mm);
                        cleared += 1;
                    }
                    if d > target + 2. + 0.4 {
                        assert!(h.lower_mm >= 10. - 4. * m.sweep.tolerance_mm);
                    }
                }
                assert!(cleared > 100);
                assert!(
                    front
                        .prepare(prior.curve.end(), 90., 1e-5, right, &|| true)
                        .is_err()
                );
            }
        }
    }
    #[test]
    fn growing_fan_contours_obey_independent_continuous_engagement_and_side_checks() {
        for right in [false, true] {
            for rotation in [0_f64, 0.41] {
                let points = [[-0.3, -0.3], [1.1, -0.3], [1.1, 1.1], [-0.3, 1.1]].map(|p| {
                    [
                        p[0] * rotation.cos() - p[1] * rotation.sin(),
                        p[0] * rotation.sin() + p[1] * rotation.cos(),
                    ]
                });
                let seed = motion(Curve::Arc {
                    start: [0.5, 0., 2.],
                    end: [0.5, 0., 2.],
                    center: [0., 0., 2.],
                    plane: Plane::Xy,
                    sweep_rad: if right { TAU } else { -TAU },
                });
                let mut front =
                    EnclosedFront::new(&region(points.to_vec()), &seed, 1., 0.2, 1e-5).unwrap();
                let mut scene = EngagementScene::new(
                    StockSpec {
                        version: 1,
                        units: "mm".into(),
                        min_mm: [-3., -3., 0.],
                        max_mm: [3., 3., 3.],
                        cell_size_mm: 0.2,
                        max_cells: 10000,
                    },
                    vec![ClearedSweep::from_motion(&seed, 1., 1e-5).unwrap()],
                )
                .unwrap();
                let mut position = seed.curve.end();
                let mut count = 0;
                while let Some(mut recipe) = front
                    .prepare(position, 82., 1e-5, right, &|| false)
                    .unwrap()
                {
                    count += 1;
                    assert!(count < 300);
                    if recipe.reset {
                        assert!(
                            scene
                                .clear_disk(
                                    [recipe.start[0], recipe.start[1]],
                                    1.00064,
                                    2.,
                                    1e-5,
                                    &mut 2_000_000,
                                    &|| false
                                )
                                .unwrap()
                        );
                        position = recipe.start;
                    }
                    assert!(front.committed(&recipe).is_err());
                    let curves: Vec<_> = recipe
                        .leads
                        .iter()
                        .chain(&recipe.contour)
                        .cloned()
                        .collect();
                    for (i, c) in curves.into_iter().enumerate() {
                        assert_eq!(c.start(), position);
                        let mut m = motion(c);
                        if i < recipe.leads.len() {
                            m.kind = MoveKind::Lead;
                        }
                        assert!(recipe.expects(&m, 1., 1e-5).unwrap());
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
                            "sense={right},rotation={rotation},recipe={count},curve={i},{checked:?}"
                        );
                        scene
                            .append(ClearedSweep::from_motion(&m, 1., 1e-5).unwrap())
                            .unwrap();
                        position = m.curve.end();
                        if recipe.advance() {
                            front.committed(&recipe).unwrap();
                        }
                    }
                }
                assert_eq!(front.piece, 4);
                assert!(count > 20);
                // Entire boxes inside the final convex component must be clear
                // using only actual swept curves, without a synthetic envelope.
                assert!(
                    scene
                        .clear_box([[-0.1, -0.1], [0.1, 0.1]], 2., &mut 2_000_000, &|| false)
                        .unwrap()
                );
            }
        }
    }
    #[test]
    fn completed_fans_cover_the_whole_offset_by_independent_sweep_union() {
        use super::super::{CoverageResult, FrontCoverage};
        use crate::{planar::ConstructionOptions, stock::Stock};
        for right in [false, true] {
            for r in [0.25, 4.] {
                for shift in [[0., 0., 0.], [10_000., -20_000., 30_000.]] {
                    for points in [
                        vec![[-0.3, -0.3], [1.1, -0.3], [1.1, 1.1], [-0.3, 1.1]],
                        vec![
                            [-0.3, -0.3],
                            [1.1, -0.3],
                            [1.4, 0.6],
                            [0.6, 1.4],
                            [-0.3, 1.1],
                        ],
                    ] {
                        let tolerance = 1e-5 * r;
                        let floor = shift[2] + 2. * r;
                        let definition = region(
                            points
                                .into_iter()
                                .map(|p| [shift[0] + r * p[0], shift[1] + r * p[1]])
                                .collect(),
                        );
                        let seed = motion(Curve::Arc {
                            start: [shift[0] + 0.5 * r, shift[1], floor],
                            end: [shift[0] + 0.5 * r, shift[1], floor],
                            center: [shift[0], shift[1], floor],
                            plane: Plane::Xy,
                            sweep_rad: if right { TAU } else { -TAU },
                        });
                        let mut front =
                            EnclosedFront::new(&definition, &seed, r, 0.2 * r, tolerance).unwrap();
                        let inflation = 1.2 * r - 0.02 * r;
                        let options = ConstructionOptions {
                            tolerance_mm: tolerance,
                        };
                        let centres = definition.prepare(options).unwrap();
                        let domain = centres.offset_prepared(-inflation).unwrap().into_parts().1;
                        let error = centres.definition().boundary_error_mm
                            + domain.definition().boundary_error_mm;
                        assert!(!front.covers_offset(inflation, error, floor, r, tolerance));
                        let spec = StockSpec {
                            version: 1,
                            units: "mm".into(),
                            min_mm: [shift[0] - 3. * r, shift[1] - 3. * r, shift[2]],
                            max_mm: [shift[0] + 4. * r, shift[1] + 4. * r, shift[2] + 3. * r],
                            cell_size_mm: 0.2 * r,
                            max_cells: 10000,
                        };
                        let stock = Stock::new(spec.clone()).unwrap();
                        let mut scene = EngagementScene::new(
                            spec,
                            vec![ClearedSweep::from_motion(&seed, r, tolerance).unwrap()],
                        )
                        .unwrap();
                        let mut position = seed.curve.end();
                        let mut count = 0;
                        while let Some(mut recipe) = front
                            .prepare(position, 82., tolerance, right, &|| false)
                            .unwrap()
                        {
                            count += 1;
                            assert!(count < 1000);
                            assert!(!front.covers_offset(inflation, error, floor, r, tolerance));
                            assert!(front.committed(&recipe).is_err());
                            let curves: Vec<_> = recipe
                                .leads
                                .iter()
                                .chain(&recipe.contour)
                                .cloned()
                                .collect();
                            for (i, curve) in curves.into_iter().enumerate() {
                                let mut m = motion(curve);
                                if i < recipe.leads.len() {
                                    m.kind = MoveKind::Lead;
                                }
                                assert!(recipe.expects(&m, r, tolerance).unwrap());
                                // Missing even the final native curve cannot publish coverage.
                                assert!(
                                    !front.covers_offset(inflation, error, floor, r, tolerance)
                                );
                                scene
                                    .append(ClearedSweep::from_motion(&m, r, tolerance).unwrap())
                                    .unwrap();
                                position = m.curve.end();
                                recipe.advance();
                            }
                            front.committed(&recipe).unwrap();
                        }
                        assert!(front.covers_offset(inflation, error, floor, r, tolerance));
                        for (i, e, z, cutter, t) in [
                            (1.2 * r, error, floor, r, tolerance),
                            (inflation, 0.1 * r, floor, r, tolerance),
                            (inflation, error, floor - r, r, tolerance),
                            (inflation, error, floor, r * 0.9, tolerance),
                            (inflation, error, floor, r, tolerance * 2.),
                        ] {
                            assert!(!front.covers_offset(i, e, z, cutter, t));
                        }
                        // Original full-domain quadtree oracle sees only actual swept
                        // curves and untouched solid stock. No fan/filled-shape shortcut.
                        let mut oracle = FrontCoverage::new(
                            domain,
                            [shift[0], shift[1]],
                            (0, 0),
                            floor,
                            tolerance,
                            0.02 * r,
                            0.5 * r,
                            centres,
                            inflation,
                        )
                        .unwrap();
                        let (mut tiles, mut envelopes, mut hits, mut branches) = (0, 0, 0, 0);
                        assert!(matches!(
                            oracle
                                .check(
                                    &scene,
                                    &stock,
                                    position,
                                    r,
                                    tolerance,
                                    &mut tiles,
                                    &mut envelopes,
                                    &mut hits,
                                    &mut branches,
                                    &|| false
                                )
                                .unwrap(),
                            CoverageResult::Complete
                        ));
                        assert!(tiles > 1);
                        assert_eq!(envelopes, 0);
                    }
                }
            }
        }
    }

    #[test]
    fn completed_fan_requires_the_final_full_native_receipt_and_same_stock() {
        use super::super::{BulkContour, CoverageResult, FrontCoverage, FrontDriver};
        use crate::{planar::ConstructionOptions, stock::Stock, stream::StreamManifest};
        let mut m: StreamManifest = serde_json::from_str(include_str!(
            "../../fixtures/operations/adaptive-pocket.json"
        ))
        .unwrap();
        m.tools[0].id = "cutter".into();
        m.tools[0].cutter.diameter_mm = 2.;
        m.tools[0].cutter.shank_diameter_mm = 2.;
        let tool = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut driver = FrontDriver::new(&m.tools[0]);
        driver.synchronize(&stock, 1., 1e-5, &|| false).unwrap();
        let seed = motion(Curve::Arc {
            start: [8.5, 8., 9.],
            end: [8.5, 8., 9.],
            center: [8., 8., 9.],
            plane: Plane::Xy,
            sweep_rad: TAU,
        });
        driver.emitted(&stock, &seed);
        stock
            .apply_delta(
                &stock
                    .preview_sweep(&tool, &seed, 1., m.sweep, &|| false)
                    .unwrap(),
            )
            .unwrap();
        driver.synchronize(&stock, 1., 1e-5, &|| false).unwrap();
        let definition = region(vec![[7.7, 7.7], [9.1, 7.7], [9.1, 9.1], [7.7, 9.1]]);
        let centres = definition.prepare(ConstructionOptions::default()).unwrap();
        let domain = centres.offset_prepared(-1.18).unwrap().into_parts().1;
        driver.coverage = Some(
            FrontCoverage::new(domain, [8., 8.], (0, 0), 9., 1e-5, 0.02, 0.5, centres, 1.18)
                .unwrap(),
        );
        driver.enclosed_front =
            Some(EnclosedFront::new(&definition, &seed, 1., 0.2, 1e-5).unwrap());
        let mut position = seed.curve.end();
        let mut recipes = 0;
        while let Some(recipe) = driver
            .enclosed_front
            .as_ref()
            .unwrap()
            .prepare(position, 82., 1e-5, true, &|| false)
            .unwrap()
        {
            assert!(!driver.coverage.as_ref().unwrap().enclosed_complete);
            recipes += 1;
            assert!(recipes < 300);
            let curves: Vec<_> = recipe
                .leads
                .iter()
                .chain(&recipe.contour)
                .cloned()
                .collect();
            let lead_count = recipe.leads.len();
            let last = curves.len() - 1;
            driver.bulk = Some(BulkContour::Enclosed(recipe));
            for (i, curve) in curves.into_iter().enumerate() {
                let mut next = motion(curve);
                next.id = seed.id + recipes * 100 + i as u64;
                if i < lead_count {
                    next.kind = MoveKind::Lead;
                }
                driver.emitted(&stock, &next);
                if i == last {
                    // No native transaction and a partial last curve both leave
                    // the entire component unproved, including its final recipe.
                    assert!(driver.synchronize(&stock, 1., 1e-5, &|| false).is_err());
                    let mut partial = stock.clone();
                    partial
                        .apply_delta(
                            &stock
                                .preview_sweep(&tool, &next, 0.5, m.sweep, &|| false)
                                .unwrap(),
                        )
                        .unwrap();
                    assert!(driver.synchronize(&partial, 1., 1e-5, &|| false).is_err());
                    assert!(!driver.coverage.as_ref().unwrap().enclosed_complete);
                }
                let delta = stock
                    .preview_sweep(&tool, &next, 1., m.sweep, &|| false)
                    .unwrap();
                stock.apply_delta(&delta).unwrap();
                driver.synchronize(&stock, 1., 1e-5, &|| false).unwrap();
                position = next.curve.end();
            }
        }
        assert!(driver.coverage.as_ref().unwrap().enclosed_complete);
        let foreign = Stock::from_snapshot(stock.snapshot()).unwrap();
        assert!(driver.synchronize(&foreign, 1., 1e-5, &|| false).is_err());
        driver.synchronize(&stock, 1., 1e-5, &|| false).unwrap();
        let coverage = driver.coverage.as_mut().unwrap();
        let (mut tiles, mut envelopes, mut hits, mut branches) = (0, 0, 0, 0);
        assert!(
            coverage
                .check(
                    driver.scene.as_ref().unwrap(),
                    &stock,
                    position,
                    1.,
                    1e-5,
                    &mut tiles,
                    &mut envelopes,
                    &mut hits,
                    &mut branches,
                    &|| true
                )
                .is_err()
        );
        assert!(matches!(
            coverage
                .check(
                    driver.scene.as_ref().unwrap(),
                    &stock,
                    position,
                    1.,
                    1e-5,
                    &mut tiles,
                    &mut envelopes,
                    &mut hits,
                    &mut branches,
                    &|| false
                )
                .unwrap(),
            CoverageResult::Complete
        ));
        assert_eq!((tiles, envelopes), (0, 1));
    }

    #[test]
    fn seed_disk_error_reserve_and_unsupported_circles_keep_coverage_fallback() {
        let polygon = region(vec![[-0.3, -0.3], [1.1, -0.3], [1.1, 1.1], [-0.3, 1.1]]);
        for radius in [1., 1. - 1e-5, 0.5] {
            let seed = motion(Curve::Arc {
                start: [radius, 0., 2.],
                end: [radius, 0., 2.],
                center: [0., 0., 2.],
                plane: Plane::Xy,
                sweep_rad: TAU,
            });
            let front = EnclosedFront::new(&polygon, &seed, 1., 0.2, 1e-5).unwrap();
            assert_eq!(front.coverage_radius.is_some(), radius == 0.5);
            let circle = PlanarRegion::circle("circle", [0., 0.], 2.).unwrap();
            let circular = EnclosedFront::new(&circle, &seed, 1., 0.2, 1e-5).unwrap();
            assert!(circular.coverage_radius.is_none());
        }
    }

    #[test]
    fn unsupported_geometry_or_incomplete_seed_does_not_create_clearance() {
        let polygon = region(vec![[-0.3, -0.3], [1.1, -0.3], [1.1, 1.1], [-0.3, 1.1]]);
        let seed = motion(Curve::Arc {
            start: [0.5, 0., 2.],
            end: [0.5, 0., 2.],
            center: [0., 0., 2.],
            plane: Plane::Xy,
            sweep_rad: TAU,
        });
        let valid =
            |r: &PlanarRegion, m: &Motion| EnclosedFront::new(r, m, 1., 0.2, 1e-5).is_some();
        assert!(valid(&polygon, &seed));
        let mut partial = seed.clone();
        if let Curve::Arc { sweep_rad, .. } = &mut partial.curve {
            *sweep_rad = PI;
        }
        assert!(!valid(&polygon, &partial));
        partial = seed.clone();
        partial.kind = MoveKind::Rapid;
        assert!(!valid(&polygon, &partial));
        let concave = region(vec![[-1., -1.], [2., -1.], [2., 2.], [0.5, 0.5], [-1., 2.]]);
        assert!(!valid(&concave, &seed));
        let mut holes = polygon.clone();
        holes.components[0].holes.push(Boundary {
            vertices: vec![[0., 0., 0.], [0.1, 0., 0.], [0., 0.1, 0.]],
        });
        assert!(!valid(&holes, &seed));
        let mut curved = polygon.clone();
        curved.components[0].outer.vertices[0][2] = 0.1;
        assert!(!valid(&curved, &seed));
        let outside = region(vec![[3., 3.], [4., 3.], [4., 4.], [3., 4.]]);
        assert!(!valid(&outside, &seed));
    }

    #[test]
    fn cancellation_and_partial_recipe_leave_the_cleared_envelope_unchanged() {
        use std::cell::Cell;
        let polygon = region(vec![[-0.3, -0.3], [1.1, -0.3], [1.1, 1.1], [-0.3, 1.1]]);
        let seed = motion(Curve::Arc {
            start: [0.5, 0., 2.],
            end: [0.5, 0., 2.],
            center: [0., 0., 2.],
            plane: Plane::Xy,
            sweep_rad: TAU,
        });
        let mut front = EnclosedFront::new(&polygon, &seed, 1., 0.2, 1e-5).unwrap();
        let calls = Cell::new(0);
        assert!(
            front
                .prepare(seed.curve.end(), 82., 1e-5, true, &|| {
                    calls.set(calls.get() + 1);
                    calls.get() >= 3
                })
                .is_err()
        );
        assert_eq!((front.piece, front.scale), (0, 0.));
        assert!(
            front
                .prepare([0.5, 0., 1.], 82., 1e-5, true, &|| false)
                .is_err()
        );
        let mut recipe = front
            .prepare(seed.curve.end(), 82., 1e-5, true, &|| false)
            .unwrap()
            .unwrap();
        let m = motion(recipe.contour[0].clone());
        assert!(recipe.expects(&m, 1.1, 1e-5).is_err());
        assert!(recipe.expects(&m, 1., 2e-5).is_err());
        assert!(!recipe.advance());
        assert!(front.committed(&recipe).is_err());
        assert_eq!((front.piece, front.scale), (0, 0.));
    }
}
