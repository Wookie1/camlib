//! Continuous capsule advances along an exact circular rail from a native seed.
//! This bootstrap never marks the requested domain covered. See method contract.
use crate::{
    Point,
    motion::{Curve, Motion, MoveKind, Plane, Vec3},
    planar::PlanarRegion,
    pocket::Stroke,
    stock::CancelSignal,
};
use std::f64::consts::{PI, TAU};

pub(super) struct ChannelFront {
    centre: Point,
    rail: f64,
    seed: Point,
    tip: Point,
    angle: f64,
    progress: f64,
    floor: f64,
    orbit: f64,
    cutter: f64,
    minimum: f64,
    guard: f64,
    epoch: usize,
    inner_axis: Option<f64>,
}
pub(super) struct ChannelRecipe {
    pub curves: Vec<Curve>,
    pub reset: bool,
    inner_axis: Option<f64>,
    angle: f64,
    progress: f64,
    tip: Point,
    epoch: usize,
    cutter: f64,
    tolerance: f64,
    cursor: usize,
}
fn distance(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
impl ChannelFront {
    pub fn new(
        centres: &PlanarRegion,
        seed: &Motion,
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
        } = seed.curve
        else {
            return None;
        };
        if seed.kind != MoveKind::Cut || start != end || sweep_rad.abs() != TAU {
            return None;
        }
        let tip = [center[0], center[1]];
        let orbit = distance([start[0], start[1]], tip);
        let guard = 64. * tolerance
            + 8192. * f64::EPSILON * (1. + tip[0].abs() + tip[1].abs() + cutter + orbit);
        if orbit + guard / 8. < minimum || orbit > cutter || orbit <= 8. * guard {
            return None;
        }
        if centres.components.len() != 1 || centres.components[0].holes.is_empty() {
            return None;
        }
        // Circular topology supplies a rail proposal only. Original whole-tube
        // source admission is mandatory before the owner is enabled.
        for boundary in
            std::iter::once(&centres.components[0].outer).chain(&centres.components[0].holes)
        {
            let strokes = Stroke::from_boundary(boundary);
            let Some((centre, _)) = strokes.first().and_then(|s| s.arc) else {
                continue;
            };
            let mut sweep = 0.;
            let authored_radius = distance(strokes[0].a, centre);
            if !strokes.iter().all(|s| {
                s.arc.is_some_and(|(c, a)| {
                    sweep += a;
                    distance(c, centre) <= guard / 8.
                        && (distance(s.a, centre) - authored_radius).abs() <= guard / 8.
                })
            }) || (sweep.abs() - TAU).abs() > 1e-10
            {
                continue;
            }
            let rail = distance(tip, centre);
            if rail <= cutter + orbit + 4. * guard || rail - orbit + tolerance < minimum {
                continue;
            }
            return Some(Self {
                centre,
                rail,
                seed: tip,
                tip,
                angle: (tip[1] - centre[1]).atan2(tip[0] - centre[0]),
                progress: 0.,
                floor: start[2],
                orbit,
                cutter,
                minimum,
                guard,
                epoch: 0,
                inner_axis: None,
            });
        }
        None
    }
    pub fn rail_tube(&self) -> (Stroke, f64) {
        (
            Stroke {
                a: self.seed,
                b: self.seed,
                arc: Some((self.centre, TAU)),
            },
            self.cutter + self.orbit + self.guard / 8.,
        )
    }
    pub fn prepare(
        &self,
        position: Vec3,
        max_angle: f64,
        tolerance: f64,
        right: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Option<ChannelRecipe>, String> {
        if cancel.is_cancelled() {
            return Err("channel contour cancelled".into());
        }
        if self.progress >= TAU {
            return Ok(None);
        }
        if position[2] != self.floor {
            return Err("channel contour floor changed".into());
        }
        let (r, s, g) = (self.cutter, self.orbit, self.guard);
        let clear = r + s - g;
        let half = max_angle.to_radians() / 2.;
        let mut delta = ((clear * clear - r * r * half.sin().powi(2)).max(0.).sqrt()
            - r * half.cos()
            - s
            - 4. * g)
            .min(s / 4.);
        let mut cap = 0.;
        for _ in 0..16 {
            if delta <= 32. * tolerance {
                return Ok(None);
            }
            let a = s + delta + g / 8.;
            cap = 2.
                * ((clear * clear - a * a - r * r) / (2. * a * r))
                    .clamp(-1., 1.)
                    .acos();
            if cap.to_degrees() <= max_angle
                && cap / 2. + ((delta + g / 8.) / s).asin() < PI / 2. - g / s
            {
                break;
            }
            delta /= 2.;
        }
        if cap.to_degrees() > max_angle
            || cap / 2. + ((delta + g / 8.) / s).asin() >= PI / 2. - g / s
        {
            return Ok(None);
        }
        let progress = (self.progress + delta / self.rail).min(TAU);
        let theta = self.angle + progress;
        let tip = if progress == TAU {
            self.seed
        } else {
            [
                self.centre[0] + self.rail * theta.cos(),
                self.centre[1] + self.rail * theta.sin(),
            ]
        };
        let normal = |p: Point| {
            [
                (p[0] - self.centre[0]) / self.rail,
                (p[1] - self.centre[1]) / self.rail,
            ]
        };
        let old = normal(self.tip);
        let new = normal(tip);
        let sign = if right { 1. } else { -1. };
        let side =
            |p: Point, n: Point, k: f64| [p[0] + k * s * n[0], p[1] + k * s * n[1], self.floor];
        let a = side(self.tip, old, sign);
        let b = side(tip, new, sign);
        let c = side(tip, new, -sign);
        let d = side(self.tip, old, -sign);
        let centre = [self.centre[0], self.centre[1], self.floor];
        let advance = progress - self.progress;
        let mut curves = Vec::with_capacity(6);
        if self.epoch == 0 && position != a {
            if (distance([position[0], position[1]], self.tip) - s).abs() > g / 8. {
                return Ok(None);
            }
            let old_angle = (position[1] - self.tip[1]).atan2(position[0] - self.tip[0]);
            let next_angle = (a[1] - self.tip[1]).atan2(a[0] - self.tip[0]);
            let mut sweep = sign * ((sign * (next_angle - old_angle)).rem_euclid(TAU));
            if sweep.abs() < 1e-10 {
                sweep = sign * TAU;
            }
            curves.push(Curve::Arc {
                start: position,
                end: a,
                center: [self.tip[0], self.tip[1], self.floor],
                plane: Plane::Xy,
                sweep_rad: sweep,
            });
        } else if position != a {
            return Err("channel seam mismatch".into());
        }
        let side_curve = Curve::Arc {
            start: a,
            end: b,
            center: centre,
            plane: Plane::Xy,
            sweep_rad: advance,
        };
        curves.push(side_curve.clone());
        curves.push(Curve::Arc {
            start: b,
            end: c,
            center: [tip[0], tip[1], self.floor],
            plane: Plane::Xy,
            sweep_rad: sign * PI,
        });
        curves.push(Curve::Arc {
            start: c,
            end: d,
            center: centre,
            plane: Plane::Xy,
            sweep_rad: -advance,
        });
        curves.push(Curve::Arc {
            start: d,
            end: a,
            center: [self.tip[0], self.tip[1], self.floor],
            plane: Plane::Xy,
            sweep_rad: sign * PI,
        });
        curves.push(side_curve);
        for curve in &curves {
            if cancel.is_cancelled() {
                return Err("channel contour cancelled".into());
            }
            curve.prepare(tolerance)?;
            let Curve::Arc { start, center, .. } = curve else {
                unreachable!()
            };
            if distance([start[0], start[1]], [center[0], center[1]]) + tolerance < self.minimum {
                return Ok(None);
            }
            let motion = Motion {
                id: 0,
                operation: String::new(),
                tool: String::new(),
                kind: MoveKind::Cut,
                feed_mm_min: None,
                curve: curve.clone(),
            };
            if crate::verification::path_error(&motion, tolerance) > g / 8. {
                return Ok(None);
            }
        }
        Ok(Some(ChannelRecipe {
            curves,
            reset: false,
            inner_axis: None,
            angle: cap.to_degrees(),
            progress,
            tip,
            epoch: self.epoch,
            cutter: r,
            tolerance,
            cursor: 0,
        }))
    }
    /// Propose concentric cleanup only after the actual channel closes. Radius
    /// selection is a heuristic; these curves have NO analytic admission token.
    /// The parent checks source, descent and continuous engagement against stock.
    pub fn prepare_inner(
        &self,
        max_angle: f64,
        tolerance: f64,
        right: bool,
        fits: impl Fn(Stroke) -> bool,
        cancel: &impl CancelSignal,
    ) -> Result<Option<ChannelRecipe>, String> {
        if cancel.is_cancelled() {
            return Err("channel inner contour cancelled".into());
        }
        if self.progress < TAU {
            return Ok(None);
        }
        let r = self.cutter;
        let old = self
            .inner_axis
            .unwrap_or(self.rail - self.orbit + self.guard);
        let inner = old - r + self.guard;
        let half = (max_angle * 0.4).to_radians();
        let mut target = (r * half.cos()
            + (inner * inner - (r * half.sin()).powi(2)).max(0.).sqrt())
        .max(self.minimum);
        let circle = |radius: f64| Stroke {
            a: [self.centre[0] + radius, self.centre[1]],
            b: [self.centre[0] + radius, self.centre[1]],
            arc: Some((self.centre, TAU)),
        };
        if target >= old - 32. * tolerance || !fits(circle(old)) {
            return Ok(None);
        }
        if !fits(circle(target)) {
            let mut high = old;
            for _ in 0..48 {
                if cancel.is_cancelled() {
                    return Err("channel inner contour cancelled".into());
                }
                let mid = (target + high) / 2.;
                if fits(circle(mid)) {
                    high = mid;
                } else {
                    target = mid;
                }
            }
            target = high;
        }
        if old - target <= 32. * tolerance {
            return Ok(None);
        }
        let sign = if right { 1. } else { -1. };
        let s = self.orbit;
        let centre = [self.centre[0], self.centre[1], self.floor];
        let lead_centre = [centre[0] + target + s, centre[1], self.floor];
        let start = [lead_centre[0], lead_centre[1] + sign * s, self.floor];
        let end = [centre[0] + target, centre[1], self.floor];
        Ok(Some(ChannelRecipe {
            curves: vec![
                Curve::Arc {
                    start,
                    end,
                    center: lead_centre,
                    plane: Plane::Xy,
                    sweep_rad: sign * PI / 2.,
                },
                Curve::Arc {
                    start: end,
                    end,
                    center: centre,
                    plane: Plane::Xy,
                    sweep_rad: -sign * TAU,
                },
            ],
            reset: true,
            inner_axis: Some(target),
            angle: 0.,
            progress: self.progress,
            tip: self.tip,
            epoch: self.epoch,
            cutter: r,
            tolerance,
            cursor: 0,
        }))
    }
    pub fn committed(&mut self, recipe: &ChannelRecipe) -> Result<(), String> {
        if recipe.epoch != self.epoch
            || recipe.cursor != recipe.curves.len()
            || (recipe.inner_axis.is_none() && recipe.progress <= self.progress)
        {
            return Err("channel contour incomplete or mismatched recipe".into());
        }
        self.tip = recipe.tip;
        self.progress = recipe.progress;
        if let Some(axis) = recipe.inner_axis {
            self.inner_axis = Some(axis);
        }
        self.epoch += 1;
        Ok(())
    }
}
impl ChannelRecipe {
    pub fn expects(&self, motion: &Motion, radius: f64, tolerance: f64) -> Result<bool, String> {
        if motion.kind == MoveKind::Rapid || motion.curve.start()[2] != motion.curve.end()[2] {
            return Ok(false);
        }
        if radius != self.cutter
            || tolerance > self.tolerance
            || motion.kind != MoveKind::Cut
            || self.curves.get(self.cursor) != Some(&motion.curve)
        {
            return Err("channel whole native commit mismatch".into());
        }
        Ok(true)
    }
    pub fn advance(&mut self) -> bool {
        self.cursor += 1;
        self.cursor == self.curves.len()
    }
    pub fn certified_angle(&self, motion: &Motion) -> Option<f64> {
        (self.inner_axis.is_none()
            && motion.kind == MoveKind::Cut
            && self.curves.get(self.cursor) == Some(&motion.curve))
        .then_some(self.angle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engagement::{ClearedSweep, EngagementOptions, EngagementScene, EngagementStatus};
    use crate::stock::Stock;
    use crate::stream::StreamManifest;

    fn motion(curve: Curve) -> Motion {
        Motion {
            id: 1,
            operation: "channel".into(),
            tool: "t1".into(),
            kind: MoveKind::Cut,
            feed_mm_min: Some(600.),
            curve,
        }
    }
    #[test]
    fn capsules_obey_continuous_contact_side_tangency_and_actual_stock() {
        for right in [false, true] {
            let mut m: StreamManifest = serde_json::from_str(include_str!(
                "../../fixtures/operations/adaptive-pocket.json"
            ))
            .unwrap();
            let shift = if right { [0., 0.] } else { [100., -80.] };
            for (k, v) in shift.iter().enumerate() {
                m.stock.height_mut().unwrap().min_mm[k] += v;
                m.stock.height_mut().unwrap().max_mm[k] += v;
            }
            let centre = [13. + shift[0], 13. + shift[1]];
            let angle = 0.41_f64;
            let seed = [centre[0] + 6.5 * angle.cos(), centre[1] + 6.5 * angle.sin()];
            let p = [seed[0], seed[1] + 1., 9.];
            let mut prior = motion(Curve::Arc {
                start: p,
                end: p,
                center: [seed[0], seed[1], 9.],
                plane: Plane::Xy,
                sweep_rad: if right { TAU } else { -TAU },
            });
            let mut region = PlanarRegion::circle("centre ring", centre, 7.).unwrap();
            region.components[0].holes.push(
                PlanarRegion::circle("protected centre", centre, 6.)
                    .unwrap()
                    .components
                    .remove(0)
                    .outer,
            );
            let mut owner = ChannelFront::new(&region, &prior, 2., 1., 1e-5).unwrap();
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
            while let Some(mut recipe) = owner
                .prepare(prior.curve.end(), 90., 1e-5, right, &|| false)
                .unwrap()
            {
                assert!(owner.committed(&recipe).is_err());
                for curve in recipe.curves.clone() {
                    let next = motion(curve);
                    assert_eq!(next.curve.start(), prior.curve.end());
                    let a = super::super::tangent(&prior.curve, 1.);
                    let b = super::super::tangent(&next.curve, 0.);
                    assert!(
                        (a - b).sin().abs() < 1e-8 && (a - b).cos() > 0.,
                        "C1 join {a}/{b}"
                    );
                    let checked = scene
                        .check_front(&next, 2., 90., right, EngagementOptions::default(), &|| {
                            false
                        })
                        .unwrap();
                    assert_eq!(
                        checked.status,
                        EngagementStatus::Bounded,
                        "recipe{count}, {next:?}: {checked:?}"
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
                owner.committed(&recipe).unwrap();
                assert!(owner.committed(&recipe).is_err());
                count += 1;
                assert!(count < 1000);
            }
            assert!(count > 100);
            assert_eq!(owner.progress, TAU);
            let mut checked = 0;
            for (i, h) in stock.cells().iter().enumerate() {
                let radius = distance(stock.cell_center(i).unwrap(), centre);
                if (radius - 6.5).abs() < 2.6 {
                    assert!(
                        h.upper_mm <= 9. + 4. * m.sweep.tolerance_mm,
                        "band cell{radius}: {h:?}"
                    );
                    checked += 1;
                }
                if radius < 3.1 {
                    assert!(h.lower_mm >= 10. - 4. * m.sweep.tolerance_mm);
                }
            }
            assert!(checked > 100);
            // Finish toward a physical 3 mm island through independent scene
            // checks, not through the bootstrap's disk certificate.
            let mut inner_count = 0;
            while let Some(mut recipe) = owner
                .prepare_inner(
                    90.,
                    1e-5,
                    right,
                    |stroke| {
                        let axis = distance(stroke.a, centre);
                        (5.0001..=8.).contains(&axis)
                    },
                    &|| false,
                )
                .unwrap()
            {
                assert!(recipe.reset);
                let start = recipe.curves[0].start();
                assert!(
                    scene
                        .clear_disk(
                            [start[0], start[1]],
                            2.00064,
                            9.,
                            1e-5,
                            &mut 2_000_000,
                            &|| false
                        )
                        .unwrap()
                );
                assert!(owner.committed(&recipe).is_err());
                for curve in recipe.curves.clone() {
                    let next = motion(curve);
                    assert!(recipe.certified_angle(&next).is_none());
                    let checked = scene
                        .check_front(&next, 2., 90., right, EngagementOptions::default(), &|| {
                            false
                        })
                        .unwrap();
                    assert_eq!(
                        checked.status,
                        EngagementStatus::Bounded,
                        "inner {next:?}: {checked:?}"
                    );
                    let delta = stock
                        .preview_sweep(&tool, &next, 1., m.sweep, &|| false)
                        .unwrap();
                    stock.apply_delta(&delta).unwrap();
                    scene
                        .append(ClearedSweep::from_motion(&next, 2., 1e-5).unwrap())
                        .unwrap();
                    recipe.advance();
                }
                owner.committed(&recipe).unwrap();
                assert!(owner.committed(&recipe).is_err());
                inner_count += 1;
                assert!(inner_count < 10);
            }
            assert!(inner_count > 0);
            for (i, h) in stock.cells().iter().enumerate() {
                let radius = distance(stock.cell_center(i).unwrap(), centre);
                if radius > 3.4 && radius < 9.1 {
                    assert!(h.upper_mm <= 9. + 4. * m.sweep.tolerance_mm);
                }
                if radius < 2.7 {
                    assert!(h.lower_mm >= 10. - 4. * m.sweep.tolerance_mm);
                }
            }
            assert!(
                owner
                    .prepare(prior.curve.end(), 90., 1e-5, right, &|| true)
                    .is_err()
            );
        }
    }
}
