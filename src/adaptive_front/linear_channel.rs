//! Native-commit-owned straight channel and independently admitted side cleanup.
use super::enclosed::growth_step;
use crate::{
    Point,
    motion::{Curve, Motion, MoveKind, Plane, Vec3},
    pocket::Stroke,
    stock::CancelSignal,
};
use std::f64::consts::{PI, TAU};

pub(super) struct LinearChannel {
    start: Point,
    end: Point,
    direction: Point,
    length: f64,
    floor: f64,
    orbit: f64,
    cutter: f64,
    guard: f64,
    seeded: bool,
    progress: f64,
    offsets: [f64; 2],
    side: usize,
    epoch: usize,
}
enum Completion {
    Seed,
    Advance(f64),
    Side(usize, f64),
}
pub(super) struct LinearRecipe {
    pub curves: Vec<Curve>,
    pub reset: bool,
    completion: Completion,
    angle: Option<f64>,
    epoch: usize,
    cutter: f64,
    tolerance: f64,
    cursor: usize,
}
impl LinearChannel {
    pub fn new(
        rail: Stroke,
        floor: f64,
        orbit: f64,
        cutter: f64,
        minimum: f64,
        tolerance: f64,
    ) -> Option<Self> {
        let length = (rail.b[0] - rail.a[0]).hypot(rail.b[1] - rail.a[1]);
        let guard = 64. * tolerance
            + 8192.
                * f64::EPSILON
                * (1.
                    + rail.a[0].abs()
                    + rail.a[1].abs()
                    + rail.b[0].abs()
                    + rail.b[1].abs()
                    + cutter
                    + orbit);
        if rail.arc.is_some()
            || !length.is_finite()
            || length <= 32. * tolerance
            || orbit < minimum
            || orbit > cutter
            || orbit <= 4. * guard
        {
            return None;
        }
        Some(Self {
            start: rail.a,
            end: rail.b,
            direction: [
                (rail.b[0] - rail.a[0]) / length,
                (rail.b[1] - rail.a[1]) / length,
            ],
            length,
            floor,
            orbit,
            cutter,
            guard,
            seeded: false,
            progress: 0.,
            offsets: [orbit; 2],
            side: 0,
            epoch: 0,
        })
    }
    pub fn prepare(
        &self,
        position: Vec3,
        max_angle: f64,
        tolerance: f64,
        right: bool,
        fits: impl Fn(Stroke) -> bool,
        cancel: &impl CancelSignal,
    ) -> Result<Option<LinearRecipe>, String> {
        if cancel.is_cancelled() {
            return Err("linear channel cancelled".into());
        }
        if position[2] != self.floor {
            return Err("linear channel floor changed".into());
        }
        let n = [self.direction[1], -self.direction[0]];
        let sign = if right { 1. } else { -1. };
        let lift = |p: Point, k: f64| [p[0] + k * n[0], p[1] + k * n[1], self.floor];
        let arc = |start, end, centre: Point, sweep| Curve::Arc {
            start,
            end,
            center: [centre[0], centre[1], self.floor],
            plane: Plane::Xy,
            sweep_rad: sweep,
        };
        let (curves, reset, completion, angle) = if !self.seeded {
            let p = lift(self.start, sign * self.orbit);
            (
                vec![arc(p, p, self.start, sign * TAU)],
                true,
                Completion::Seed,
                None,
            )
        } else if self.progress < self.length {
            let Some((delta, angle)) =
                growth_step(self.cutter, self.orbit, self.guard, max_angle, tolerance)
            else {
                return Ok(None);
            };
            let progress = (self.progress + delta).min(self.length);
            let point = |d: f64| {
                if d == self.length {
                    self.end
                } else {
                    [
                        self.start[0] + d * self.direction[0],
                        self.start[1] + d * self.direction[1],
                    ]
                }
            };
            let old = point(self.progress);
            let new = point(progress);
            let a = lift(old, sign * self.orbit);
            let b = lift(new, sign * self.orbit);
            let c = lift(new, -sign * self.orbit);
            let d = lift(old, -sign * self.orbit);
            if position != a {
                return Err("linear channel seam mismatch".into());
            }
            let side = Curve::Line { start: a, end: b };
            (
                vec![
                    side.clone(),
                    arc(b, c, new, sign * PI),
                    Curve::Line { start: c, end: d },
                    arc(d, a, old, sign * PI),
                    side,
                ],
                false,
                Completion::Advance(progress),
                Some(angle.to_degrees()),
            )
        } else {
            let step = self.cutter * (1. - (max_angle.to_radians() / 2.).cos()) * 0.8;
            let mut found = None;
            for side in self.side..2 {
                if cancel.is_cancelled() {
                    return Err("linear cleanup cancelled".into());
                }
                let old = self.offsets[side];
                let mut target = old + step;
                let stroke = |offset: f64| {
                    let offset = if side == 0 { offset } else { -offset };
                    let a = lift(self.start, offset);
                    let b = lift(self.end, offset);
                    Stroke::line([a[0], a[1]], [b[0], b[1]])
                };
                if !fits(stroke(old)) {
                    continue;
                }
                if !fits(stroke(target)) {
                    let mut low = old;
                    for _ in 0..48 {
                        if cancel.is_cancelled() {
                            return Err("linear cleanup cancelled".into());
                        }
                        let mid = (low + target) / 2.;
                        if fits(stroke(mid)) {
                            low = mid;
                        } else {
                            target = mid;
                        }
                    }
                    target = low;
                }
                if target - old <= 32. * tolerance {
                    continue;
                }
                let line = stroke(target);
                let forward = (side == 0) == right;
                let (a, b) = if forward {
                    (line.a, line.b)
                } else {
                    (line.b, line.a)
                };
                found = Some((
                    vec![Curve::Line {
                        start: [a[0], a[1], self.floor],
                        end: [b[0], b[1], self.floor],
                    }],
                    true,
                    Completion::Side(side, target),
                    None,
                ));
                break;
            }
            let Some(found) = found else {
                return Ok(None);
            };
            found
        };
        for curve in &curves {
            if cancel.is_cancelled() {
                return Err("linear channel cancelled".into());
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
        Ok(Some(LinearRecipe {
            curves,
            reset,
            completion,
            angle,
            epoch: self.epoch,
            cutter: self.cutter,
            tolerance,
            cursor: 0,
        }))
    }
    pub fn committed(&mut self, recipe: &LinearRecipe) -> Result<(), String> {
        if recipe.epoch != self.epoch || recipe.cursor != recipe.curves.len() {
            return Err("linear channel incomplete or replayed recipe".into());
        }
        match recipe.completion {
            Completion::Seed if !self.seeded => self.seeded = true,
            Completion::Advance(p) if self.seeded && p > self.progress && p <= self.length => {
                self.progress = p
            }
            Completion::Side(side, offset)
                if self.progress == self.length
                    && side >= self.side
                    && offset > self.offsets[side] =>
            {
                self.side = side;
                self.offsets[side] = offset;
            }
            _ => return Err("linear channel recipe mismatch".into()),
        }
        self.epoch += 1;
        Ok(())
    }
}
impl LinearRecipe {
    pub fn expects(&self, motion: &Motion, radius: f64, tolerance: f64) -> Result<bool, String> {
        if motion.kind == MoveKind::Rapid || motion.curve.start()[2] != motion.curve.end()[2] {
            return Ok(false);
        }
        if radius != self.cutter
            || tolerance > self.tolerance
            || motion.kind != MoveKind::Cut
            || self.curves.get(self.cursor) != Some(&motion.curve)
        {
            return Err("linear channel native commit mismatch".into());
        }
        Ok(true)
    }
    pub fn advance(&mut self) -> bool {
        self.cursor += 1;
        self.cursor == self.curves.len()
    }
    pub fn certified_angle(&self, motion: &Motion) -> Option<f64> {
        if motion.kind == MoveKind::Cut && self.curves.get(self.cursor) == Some(&motion.curve) {
            self.angle
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        engagement::{ClearedSweep, EngagementOptions, EngagementScene, EngagementStatus},
        stock::Stock,
        stream::StreamManifest,
    };
    #[test]
    fn straight_capsules_and_side_passes_preserve_contact_stock_and_native_progress() {
        for right in [false, true] {
            let shift = if right { [0., 0.] } else { [100., -80.] };
            let angle = if right { 0_f64 } else { 0.41 };
            let t = [angle.cos(), angle.sin()];
            let n = [t[1], -t[0]];
            let centre = [5. + shift[0], 5. + shift[1]];
            let point = |u: f64| [centre[0] + u * t[0], centre[1] + u * t[1]];
            let rail = Stroke::line(point(-6.), point(6.));
            let mut owner = LinearChannel::new(rail, 9., 0.5, 1., 0.2, 1e-5).unwrap();
            let mut m: StreamManifest = serde_json::from_str(include_str!(
                "../../fixtures/operations/adaptive-pocket.json"
            ))
            .unwrap();
            m.stock.height_mut().unwrap().min_mm = [2. + shift[0], 2. + shift[1], 0.];
            m.stock.height_mut().unwrap().max_mm = [8. + shift[0], 8. + shift[1], 10.];
            m.stock.height_mut().unwrap().cell_size_mm = 0.2;
            m.tools[0].cutter.diameter_mm = 2.;
            m.tools[0].cutter.shank_diameter_mm = 2.;
            let tool = m.tools[0].prepare().unwrap();
            let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
            let mut scene =
                EngagementScene::new(m.stock.height().unwrap().clone(), vec![]).unwrap();
            let mut position = [rail.a[0], rail.a[1], 9.];
            let mut prior: Option<Curve> = None;
            let mut count = 0;
            let mut resets = 0;
            let lateral = |p: Point| (p[0] - centre[0]) * n[0] + (p[1] - centre[1]) * n[1];
            while let Some(mut recipe) = owner
                .prepare(
                    position,
                    82.,
                    1e-5,
                    right,
                    |stroke| lateral(stroke.a).abs() <= 0.8 && lateral(stroke.b).abs() <= 0.8,
                    &|| false,
                )
                .unwrap()
            {
                assert!(owner.committed(&recipe).is_err());
                if recipe.reset {
                    position = recipe.curves[0].start();
                    prior = None;
                    resets += 1;
                    assert!(
                        scene
                            .clear_disk(
                                [position[0], position[1]],
                                1.00064,
                                9.,
                                1e-5,
                                &mut 2_000_000,
                                &|| false
                            )
                            .unwrap()
                    );
                }
                for curve in recipe.curves.clone() {
                    assert_eq!(curve.start(), position);
                    if let Some(old) = &prior {
                        let a = super::super::tangent(old, 1.);
                        let b = super::super::tangent(&curve, 0.);
                        assert!((a - b).sin().abs() < 1e-8 && (a - b).cos() > 0.);
                    }
                    let source_extent = match &curve {
                        Curve::Line { start, end } => lateral([start[0], start[1]])
                            .abs()
                            .max(lateral([end[0], end[1]]).abs()),
                        Curve::Arc { start, center, .. } => {
                            lateral([center[0], center[1]]).abs()
                                + (start[0] - center[0]).hypot(start[1] - center[1])
                        }
                        _ => panic!("planar primitive required"),
                    };
                    assert!(source_extent <= 0.8 + 1e-8);
                    let motion = Motion {
                        id: 1,
                        operation: "channel".into(),
                        tool: "t1".into(),
                        kind: MoveKind::Cut,
                        feed_mm_min: Some(600.),
                        curve: curve.clone(),
                    };
                    let checked = scene
                        .check_front(
                            &motion,
                            1.,
                            82.,
                            right,
                            EngagementOptions::default(),
                            &|| false,
                        )
                        .unwrap();
                    assert_eq!(
                        checked.status,
                        EngagementStatus::Bounded,
                        "{motion:?}: {checked:?}"
                    );
                    assert!(recipe.expects(&motion, 1., 1e-5).unwrap());
                    assert!(recipe.expects(&motion, 0.9, 1e-5).is_err());
                    assert!(recipe.expects(&motion, 1., 1e-4).is_err());
                    let delta = stock
                        .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
                        .unwrap();
                    stock.apply_delta(&delta).unwrap();
                    scene
                        .append(ClearedSweep::from_motion(&motion, 1., 1e-5).unwrap())
                        .unwrap();
                    if !recipe.advance() {
                        assert!(owner.committed(&recipe).is_err());
                    }
                    position = curve.end();
                    prior = Some(curve);
                }
                owner.committed(&recipe).unwrap();
                assert!(owner.committed(&recipe).is_err());
                count += 1;
                assert!(count < 300);
            }
            assert!(owner.seeded && owner.progress == owner.length);
            assert!(resets >= 5);
            let mut cleared = 0;
            let mut protected = 0;
            for (i, h) in stock.cells().iter().enumerate() {
                let d = lateral(stock.cell_center(i).unwrap()).abs();
                if d < 1.5 {
                    assert!(h.upper_mm <= 9. + 4. * m.sweep.tolerance_mm);
                    cleared += 1;
                }
                if d > 2.1 {
                    assert!(h.lower_mm >= 10. - 4. * m.sweep.tolerance_mm);
                    protected += 1;
                }
            }
            assert!(cleared > 100 && protected > 100);
            assert!(
                owner
                    .prepare(position, 82., 1e-5, right, |_| true, &|| true)
                    .is_err()
            );
        }
    }
}
