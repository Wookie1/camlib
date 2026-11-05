//! Stock-derived bulk contours. Proposed geometry is never cleared history.
use super::{candidate, tangent};
use crate::{
    Point,
    motion::{Curve, Motion, MoveKind, Plane, Vec3},
};
use std::f64::consts::FRAC_PI_2;

pub(super) struct PeelContour {
    pub start: Vec3,
    pub heading: f64,
    pub leads: Vec<Curve>,
    pub contour: Vec<Curve>,
    remaining: usize,
    cursor: usize,
    bounds: [Point; 2],
    floor: f64,
    radius: f64,
    tolerance: f64,
    engagement_angle: Option<f64>,
}

impl PeelContour {
    fn limits(
        old: [Point; 2],
        radius: f64,
        max_angle: f64,
        boundary_budget: f64,
        tolerance: f64,
    ) -> (f64, f64) {
        let scale = old
            .into_iter()
            .flatten()
            .map(f64::abs)
            .fold(radius.max(1.), f64::max);
        let guard = 64. * tolerance + 8192. * f64::EPSILON * (1. + scale + radius);
        let width = radius * (1. - (max_angle.to_radians() / 2.).cos());
        (guard, width - boundary_budget / 4. - guard)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn prepare(
        old: [Point; 2],
        floor: f64,
        radius: f64,
        minimum_radius: f64,
        max_angle: f64,
        boundary_budget: f64,
        tolerance: f64,
        right: bool,
    ) -> Result<Option<Self>, String> {
        let (_, step) = Self::limits(old, radius, max_angle, boundary_budget, tolerance);
        Self::prepare_insets(
            old,
            floor,
            radius,
            minimum_radius,
            max_angle,
            boundary_budget,
            tolerance,
            right,
            [[step; 2]; 2],
        )
    }

    /// Source fitting proposes geometry only. A failed probe does not prove
    /// intervening positions infeasible; every retained recipe has its OWN
    /// whole-source admission. Actual contact/side and native commits remain
    /// mandatory in FrontDriver. Four bounded searches let a blocked face stop
    /// without forcing all the other stock faces into local recovery orbits.
    #[allow(clippy::too_many_arguments)]
    pub fn prepare_fitted(
        old: [Point; 2],
        floor: f64,
        radius: f64,
        minimum_radius: f64,
        max_angle: f64,
        boundary_budget: f64,
        tolerance: f64,
        right: bool,
        mut fits: impl FnMut(&Curve) -> Result<bool, String>,
    ) -> Result<Option<Self>, String> {
        let Some(uniform) = Self::prepare(
            old,
            floor,
            radius,
            minimum_radius,
            max_angle,
            boundary_budget,
            tolerance,
            right,
        )?
        else {
            return Ok(None);
        };
        if uniform.fits_all(&mut fits)? {
            return Ok(Some(uniform));
        }
        let (_, step) = Self::limits(old, radius, max_angle, boundary_budget, tolerance);
        let mut insets = [[0.; 2]; 2];
        let Some(mut best) = Self::prepare_insets(
            old,
            floor,
            radius,
            minimum_radius,
            max_angle,
            boundary_budget,
            tolerance,
            right,
            insets,
        )?
        else {
            return Ok(None);
        };
        if !best.fits_all(&mut fits)? {
            return Ok(None);
        }
        for side in 0..2 {
            for axis in 0..2 {
                let mut lo = 0.;
                let mut hi = step;
                // First try a full authored-width advance of this face; then
                // at most twelve smaller trials. No unchecked interpolation.
                for attempt in 0..=12 {
                    let delta = if attempt == 0 { step } else { (lo + hi) / 2. };
                    let mut trial = insets;
                    trial[side][axis] = delta;
                    if let Some(recipe) = Self::prepare_insets(
                        old,
                        floor,
                        radius,
                        minimum_radius,
                        max_angle,
                        boundary_budget,
                        tolerance,
                        right,
                        trial,
                    )? && recipe.fits_all(&mut fits)?
                    {
                        lo = delta;
                        best = recipe;
                        if attempt == 0 {
                            break;
                        }
                    } else {
                        hi = delta;
                    }
                    if hi - lo <= 32. * tolerance {
                        break;
                    }
                }
                insets[side][axis] = lo;
            }
        }
        // Do not repeat a contour that cannot shrink the conservatively
        // bounded material rectangle by a meaningful numerical reserve.
        Ok((0..2)
            .any(|k| {
                best.bounds[0][k] - old[0][k] > 32. * tolerance
                    || old[1][k] - best.bounds[1][k] > 32. * tolerance
            })
            .then_some(best))
    }

    fn fits_all(
        &self,
        fits: &mut impl FnMut(&Curve) -> Result<bool, String>,
    ) -> Result<bool, String> {
        for curve in self.leads.iter().chain(&self.contour) {
            if !fits(curve)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    #[allow(clippy::too_many_arguments)]
    fn prepare_insets(
        old: [Point; 2],
        floor: f64,
        radius: f64,
        minimum_radius: f64,
        max_angle: f64,
        boundary_budget: f64,
        tolerance: f64,
        right: bool,
        insets: [Point; 2],
    ) -> Result<Option<Self>, String> {
        if radius < minimum_radius {
            return Ok(None);
        }
        let scale = old
            .into_iter()
            .flatten()
            .map(f64::abs)
            .fold(radius.max(1.), f64::max);
        let (guard, step) = Self::limits(old, radius, max_angle, boundary_budget, tolerance);
        if step <= 4. * guard || step > radius {
            return Ok(None);
        }
        if insets
            .into_iter()
            .flatten()
            .any(|d| !d.is_finite() || d < 0. || d > step)
        {
            return Err("stock-front contour inset exceeds authored width".into());
        }
        let inner: [Point; 2] = [
            std::array::from_fn(|k| old[0][k] + insets[0][k]),
            std::array::from_fn(|k| old[1][k] - insets[1][k]),
        ];
        if (0..2).any(|k| inner[1][k] - inner[0][k] <= guard) {
            return Ok(None);
        }
        let pad = 64. * tolerance;
        let left_step = insets[0][0];
        let shift = left_step + 2. * pad;
        // For the restoring S arc, wrong-side clearance against a straight
        // front requires r-step+R(1-cos u)-r*sin u>0. Its minimum gives
        // R >= step*(2r-step)/(2*(r-step)); leave search headroom above it.
        let side_radius = left_step * (2. * radius - left_step) / (2. * (radius - left_step));
        let lead_radius = minimum_radius
            .max(radius / 2.)
            .max(shift / 2.)
            .max(1.05 * side_radius);
        let theta = (1. - shift / (2. * lead_radius)).clamp(-1., 1.).acos();
        let length = 2. * lead_radius * theta.sin();
        if length / 2. + guard >= inner[1][1] - inner[0][1] {
            return Ok(None);
        }
        let start = [
            old[0][0] - radius - 2. * pad,
            inner[0][1] - length / 2.,
            floor,
        ];
        let first = candidate(start, FRAC_PI_2, -lead_radius.recip(), lead_radius * theta);
        let second = candidate(
            first.end(),
            tangent(&first, 1.),
            lead_radius.recip(),
            lead_radius * theta,
        );
        let q = second.end();
        let point = |x, y| [x, y, floor];
        let [lo, hi] = inner;
        let left = lo[0] - radius;
        let top = hi[1] + radius;
        let east = hi[0] + radius;
        let bottom = lo[1] - radius;
        let line = |a, b| Curve::Line { start: a, end: b };
        let arc = |a, b, c| Curve::Arc {
            start: a,
            end: b,
            center: c,
            plane: Plane::Xy,
            sweep_rad: -FRAC_PI_2,
        };
        let mut leads = vec![first, second];
        let mut contour = vec![
            line(q, point(left, hi[1])),
            arc(point(left, hi[1]), point(lo[0], top), point(lo[0], hi[1])),
            line(point(lo[0], top), point(hi[0], top)),
            arc(point(hi[0], top), point(east, hi[1]), point(hi[0], hi[1])),
            line(point(east, hi[1]), point(east, lo[1])),
            arc(
                point(east, lo[1]),
                point(hi[0], bottom),
                point(hi[0], lo[1]),
            ),
            line(point(hi[0], bottom), point(lo[0], bottom)),
            arc(
                point(lo[0], bottom),
                point(left, lo[1]),
                point(lo[0], lo[1]),
            ),
            line(point(left, lo[1]), q),
        ];
        if !right {
            for curve in leads.iter_mut().chain(contour.iter_mut()) {
                match curve {
                    Curve::Line { start, end } => {
                        start[1] = inner[0][1] + inner[1][1] - start[1];
                        end[1] = inner[0][1] + inner[1][1] - end[1];
                    }
                    Curve::Arc {
                        start,
                        end,
                        center,
                        sweep_rad,
                        ..
                    } => {
                        for p in [start, end, center] {
                            p[1] = inner[0][1] + inner[1][1] - p[1];
                        }
                        *sweep_rad = -*sweep_rad;
                    }
                    _ => unreachable!(),
                }
            }
        }
        let mut maximum_error: f64 = 0.;
        for curve in leads.iter().chain(&contour) {
            let motion = Motion {
                id: 1,
                operation: "shell".into(),
                tool: "shell".into(),
                kind: MoveKind::Cut,
                curve: curve.clone(),
                feed_mm_min: Some(1.),
            };
            curve.prepare(tolerance)?;
            maximum_error = maximum_error.max(crate::verification::path_error(&motion, tolerance));
        }
        // Covers endpoint/trigonometric consistency errors as well as the
        // tube-radius reserve. Tangent collars remain unknown.
        let deviation = (q[0] - left).abs() + 64. * f64::EPSILON * (1. + scale + radius);
        if 4. * (maximum_error + deviation) >= guard {
            return Ok(None);
        }
        // Reserve the entire construction guard, not just the nominal inset:
        // it exceeds four times path/endpoint error plus the seam/coordinate
        // displacement. This also covers the rounded axis radius and reflected
        // corner coordinates. Retain the generic engagement model's angular
        // reserve too, at a scale enclosing every contour endpoint. Insufficient
        // authored margin keeps generic admission rather than raising its limit.
        let inset = insets.into_iter().flatten().fold(0f64, f64::max) + guard;
        let numeric = 2048. * f64::EPSILON * (1. + scale.max(floor.abs()) + 2. * radius);
        let angular = (8. * (numeric / radius).sqrt()).min(std::f64::consts::PI);
        let engagement_angle = (inset < radius)
            .then(|| {
                (2. * (1. - inset / radius).acos() + 2. * angular + 64. * f64::EPSILON.sqrt())
                    .to_degrees()
            })
            .filter(|angle| angle.is_finite() && *angle < 180. && *angle <= max_angle);
        // Whole-shell certificate, with no endpoint sampling: side-strip
        // distances to straight axes are convex, so box-corner maxima suffice.
        // The worst inner corner has distance hypot(r-guard,guard)<r-guard/2.
        // Each corner quadrant has rho in [sqrt(2)*guard,sqrt(2)*step] from
        // its quarter-arc centre, hence distance |rho-r| to that arc. step<=r
        // and guard<step/4 bound this by r-guard/2 too. Actual recipe/path
        // errors are <guard/4. The expanded inner collar remains unknown.
        let bounds = [
            std::array::from_fn(|k| (inner[0][k] - guard).max(old[0][k])),
            std::array::from_fn(|k| (inner[1][k] + guard).min(old[1][k])),
        ];
        let remaining = leads.len() + contour.len();
        let start = leads[0].start();
        let heading = tangent(&leads[0], 0.);
        Ok(Some(Self {
            start,
            heading,
            leads,
            contour,
            remaining,
            cursor: 0,
            bounds,
            floor,
            radius,
            tolerance,
            engagement_angle,
        }))
    }

    /// Validate BEFORE changing history. Only a matching native full commit
    /// may subsequently advance this private recipe cursor.
    pub fn expects(
        &self,
        motion: &Motion,
        radius: f64,
        sweep_tolerance: f64,
    ) -> Result<bool, String> {
        if motion.kind == MoveKind::Rapid || motion.curve.start()[2] != motion.curve.end()[2] {
            return Ok(false);
        }
        if radius != self.radius || sweep_tolerance > self.tolerance {
            return Err("stock-front contour cutter radius/sweep accuracy changed".into());
        }
        let expected = self
            .leads
            .iter()
            .chain(&self.contour)
            .nth(self.cursor)
            .ok_or("stock-front contour receipt cursor exhausted")?;
        if &motion.curve != expected || !matches!(motion.kind, MoveKind::Lead | MoveKind::Cut) {
            return Err("stock-front contour commit does not match expected whole curve".into());
        }
        Ok(true)
    }
    pub fn committed(&mut self) -> Option<([Point; 2], f64)> {
        self.cursor += 1;
        self.remaining -= 1;
        (self.remaining == 0).then_some((self.bounds, self.floor))
    }

    /// Joint contact and material-side bound for the WHOLE expected contour
    /// curve, relative to the preceding committed material rectangle.
    ///
    /// A straight face has contact cap <=2*acos(1-a), where a is the largest
    /// guarded inset divided by cutter radius. At a rounded corner, rotate
    /// axes so material is x,y>=0 and the cutter centre is (sx-r*cos(t),
    /// sy-r*sin(t)), t in[0,pi/2]. Enlarging both insets to a*r gives contact
    /// vx>=cos(t)-a, vy>=sin(t)-a. Its width is at most
    /// acos(cos(t)-a)+acos(sin(t)-a)-pi/2, whose maximum is at t=0 or pi/2:
    /// acos(1-a)+asin(a)<=2*acos(1-a). The derivative has sign sin(t)-cos(t)
    /// for 0<a<1. These same constraints keep contact within the instantaneous
    /// inward half, since a<1<=sin(t)+cos(t). Thus both metrics hold over the
    /// whole corner, without inferring clearance from sampled poses.
    ///
    /// Leads and non-axis-aligned seam lines retain generic admission. The
    /// owner still checks source, descent, stock identity and every full native
    /// receipt; proposed geometry alone never advances this private cursor.
    pub fn certified_angle(&self, motion: &Motion) -> Option<f64> {
        let index = self.cursor.checked_sub(self.leads.len())?;
        let expected = self.contour.get(index)?;
        if motion.kind != MoveKind::Cut || &motion.curve != expected {
            return None;
        }
        match expected {
            Curve::Line { start, end } if start[0] == end[0] || start[1] == end[1] => {
                self.engagement_angle
            }
            Curve::Arc {
                plane: Plane::Xy,
                sweep_rad,
                ..
            } if sweep_rad.abs() == FRAC_PI_2 => self.engagement_angle,
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        engagement::{EngagementOptions, EngagementScene, EngagementStatus},
        stock::StockSpec,
    };
    #[test]
    fn rectangular_certificates_match_independent_continuous_checks_across_scales() {
        let mut certified = 0;
        for radius in [0.25, 1., 10.] {
            for shift in [[0., 0., 0.], [10_000., -20_000., 0.], [0., 0., 20_000.]] {
                let old = [
                    [shift[0], shift[1]],
                    [shift[0] + 32. * radius, shift[1] + 28. * radius],
                ];
                let floor = shift[2] + 8. * radius;
                let scene = EngagementScene::new(
                    StockSpec {
                        version: 1,
                        units: "mm".into(),
                        min_mm: [old[0][0], old[0][1], shift[2]],
                        max_mm: [old[1][0], old[1][1], shift[2] + 10. * radius],
                        cell_size_mm: radius,
                        max_cells: 10000,
                    },
                    vec![],
                )
                .unwrap();
                for right in [true, false] {
                    for limit in [30., 90., 160.] {
                        let step = PeelContour::limits(old, radius, limit, 0.01 * radius, 1e-5).1;
                        for insets in [
                            [[step; 2]; 2],
                            [[0., step], [step / 3., 0.]],
                            [[step / 2., 0.], [0., step * 0.8]],
                        ] {
                            let mut recipe = PeelContour::prepare_insets(
                                old,
                                floor,
                                radius,
                                radius * 0.2,
                                limit,
                                0.01 * radius,
                                1e-5,
                                right,
                                insets,
                            )
                            .unwrap()
                            .unwrap();
                            let curves: Vec<_> = recipe
                                .leads
                                .iter()
                                .chain(&recipe.contour)
                                .cloned()
                                .collect();
                            let mut local = 0;
                            for (i, curve) in curves.into_iter().enumerate() {
                                let m = Motion {
                                    id: i as u64 + 1,
                                    operation: "rectangle".into(),
                                    tool: "mill".into(),
                                    kind: if i < recipe.leads.len() {
                                        MoveKind::Lead
                                    } else {
                                        MoveKind::Cut
                                    },
                                    curve,
                                    feed_mm_min: Some(600.),
                                };
                                if let Some(angle) = recipe.certified_angle(&m) {
                                    assert!(angle <= limit && angle < 180.);
                                    // This scene has the original solid rectangle and NO
                                    // cleared sweeps or recipe certificates. The generic
                                    // continuous algorithm independently proves both metrics.
                                    let check = scene
                                        .check_front(
                                            &m,
                                            radius,
                                            angle,
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
                                        check.status,
                                        EngagementStatus::Bounded,
                                        "r={radius},shift={shift:?},sense={right},limit={limit},curve={i},angle={angle},check={check:?}"
                                    );
                                    local += 1;
                                }
                                assert!(recipe.expects(&m, radius, 1e-5).unwrap());
                                let done = recipe.committed();
                                assert_eq!(done.is_some(), i == 10);
                            }
                            assert!(local >= 7, "expected four corners and three straight sides");
                            certified += local;
                        }
                    }
                }
            }
        }
        assert!(certified >= 1134);
    }

    #[test]
    fn rectangular_certificate_needs_expected_cursor_curve_kind_and_margin() {
        let make = |budget| {
            PeelContour::prepare([[0., 0.], [32., 28.]], 8., 1., 0.2, 90., budget, 1e-5, true)
                .unwrap()
                .unwrap()
        };
        let mut recipe = make(0.01);
        let mut m = Motion {
            id: 1,
            operation: "rectangle".into(),
            tool: "mill".into(),
            kind: MoveKind::Cut,
            curve: recipe.contour[1].clone(),
            feed_mm_min: Some(600.),
        };
        assert!(recipe.certified_angle(&m).is_none()); // leads have no certificate
        for i in 0..3 {
            let curve = recipe
                .leads
                .iter()
                .chain(&recipe.contour)
                .nth(i)
                .unwrap()
                .clone();
            let next = Motion { curve, ..m.clone() };
            assert!(recipe.expects(&next, 0.9, 1e-5).is_err());
            assert!(recipe.expects(&next, 1., 1e-4).is_err());
            assert!(recipe.expects(&next, 1., 1e-5).unwrap());
            assert!(recipe.committed().is_none());
        }
        assert!(recipe.certified_angle(&m).is_some());
        m.kind = MoveKind::Lead;
        assert!(recipe.certified_angle(&m).is_none());
        m.kind = MoveKind::Cut;
        if let Curve::Arc { sweep_rad, .. } = &mut m.curve {
            *sweep_rad = -*sweep_rad;
        }
        assert!(recipe.certified_angle(&m).is_none());
        m.curve = recipe.contour[1].clone();
        if let Curve::Arc {
            start, end, center, ..
        } = &mut m.curve
        {
            start[2] -= 1.;
            end[2] -= 1.;
            center[2] -= 1.;
        }
        assert!(recipe.certified_angle(&m).is_none());
        let mut no_margin = make(0.);
        no_margin.cursor = no_margin.leads.len() + 1;
        m.curve = no_margin.contour[1].clone();
        assert!(no_margin.certified_angle(&m).is_none());
        no_margin.cursor = no_margin.leads.len() + no_margin.contour.len();
        assert!(no_margin.certified_angle(&m).is_none());
    }
    #[test]
    fn a_source_blocked_face_does_not_stop_other_stock_faces() {
        for right in [true, false] {
            for part_top in [18.99, 19.] {
                let fits = |curve: &Curve| -> Result<bool, String> {
                    let b = curve.prepare(1e-5)?.bounds();
                    // Independent sufficient certificate: the WHOLE axis
                    // bounding box stays one cutter radius from this solid.
                    Ok(b.max[0] <= 9.5
                        || b.min[0] >= 22.5
                        || b.max[1] <= 9.5
                        || b.min[1] >= part_top + 1.)
                };
                let p = PeelContour::prepare_fitted(
                    [[9., 9.], [23., 19.]],
                    8.,
                    1.,
                    0.2,
                    2. * 0.75f64.acos().to_degrees(),
                    0.01,
                    1e-5,
                    right,
                    fits,
                )
                .unwrap()
                .expect("three faces can still advance");
                assert!(p.bounds[0][0] > 9.24 && p.bounds[0][1] > 9.24);
                assert!(p.bounds[1][0] < 22.76);
                assert!(p.bounds[1][1] >= part_top && p.bounds[1][1] < part_top + 0.002);
                assert!(p.fits_all(&mut { fits }).unwrap());
                // Mirror direction must preserve the actual asymmetric shell,
                // rather than reflecting its tight top face onto the bottom.
                let spec = StockSpec {
                    version: 1,
                    units: "mm".into(),
                    min_mm: [9., 9., 0.],
                    max_mm: [23., 19., 10.],
                    cell_size_mm: 0.4,
                    max_cells: 10000,
                };
                let scene = EngagementScene::new(spec, vec![]).unwrap();
                for curve in p.leads.iter().chain(&p.contour) {
                    let m = Motion {
                        id: 1,
                        operation: "asymmetric".into(),
                        tool: "t".into(),
                        kind: MoveKind::Cut,
                        curve: curve.clone(),
                        feed_mm_min: Some(600.),
                    };
                    let checked = scene
                        .check_front(
                            &m,
                            1.,
                            2. * 0.75f64.acos().to_degrees(),
                            right,
                            EngagementOptions::default(),
                            &|| false,
                        )
                        .unwrap();
                    assert_eq!(checked.status, EngagementStatus::Bounded, "{curve:?}");
                }
            }
        }
        assert!(
            PeelContour::prepare_fitted(
                [[0., 0.], [32., 28.]],
                8.,
                1.,
                0.2,
                90.,
                0.01,
                1e-5,
                true,
                |_| Ok(false)
            )
            .unwrap()
            .is_none()
        );
        assert_eq!(
            PeelContour::prepare_fitted(
                [[0., 0.], [32., 28.]],
                8.,
                1.,
                0.2,
                90.,
                0.01,
                1e-5,
                true,
                |_| Err("cancelled".into())
            )
            .err()
            .unwrap(),
            "cancelled"
        );
    }
    #[test]
    fn stock_contour_entry_and_curves_have_continuous_front_admission() {
        let spec = StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [32., 28., 10.],
            cell_size_mm: 0.4,
            max_cells: 10000,
        };
        for (right, angle) in [true, false].into_iter().flat_map(|right| {
            [30., 2. * 0.75f64.acos().to_degrees(), 160.]
                .into_iter()
                .map(move |a| (right, a))
        }) {
            let scene = EngagementScene::new(spec.clone(), vec![]).unwrap();
            let p = PeelContour::prepare(
                [[0., 0.], [32., 28.]],
                8.,
                1.,
                0.2,
                angle,
                0.01,
                1e-5,
                right,
            )
            .unwrap()
            .unwrap();
            for (i, curve) in p.leads.iter().chain(&p.contour).enumerate() {
                let m = Motion {
                    id: i as u64 + 1,
                    operation: "bulk".into(),
                    tool: "t".into(),
                    kind: MoveKind::Cut,
                    curve: curve.clone(),
                    feed_mm_min: Some(600.),
                };
                let r = scene
                    .check_front(
                        &m,
                        1.,
                        angle,
                        right,
                        EngagementOptions {
                            max_nodes: 8192,
                            max_work: 2_000_000,
                            ..Default::default()
                        },
                        &|| false,
                    )
                    .unwrap();
                assert_eq!(
                    r.status,
                    EngagementStatus::Bounded,
                    "sense={right},angle={angle},curve={i},{r:?}"
                );
            }
        }
    }

    #[test]
    fn actual_recipe_covers_the_whole_claimed_shell_by_convex_and_radial_bounds() {
        let corners = |b: [Point; 2]| [b[0], [b[0][0], b[1][1]], b[1], [b[1][0], b[0][1]]];
        let distance_to_segment = |p: Point, a: Vec3, b: Vec3| {
            let d = [b[0] - a[0], b[1] - a[1]];
            let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (d[0] * d[0] + d[1] * d[1]))
                .clamp(0., 1.);
            (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
        };
        for (shift, insets) in [[0., 0.], [10000., -20000.]].into_iter().flat_map(|shift| {
            [
                None,
                Some([[0., 0.23], [0.12, 0.]]),
                Some([[0.15, 0.], [0., 0.22]]),
                Some([[0.005, 0.01], [0.18, 0.07]]),
            ]
            .into_iter()
            .map(move |insets| (shift, insets))
        }) {
            for right in [true, false] {
                let old = [shift, [shift[0] + 32., shift[1] + 28.]];
                let step =
                    PeelContour::limits(old, 1., 2. * 0.75f64.acos().to_degrees(), 0.01, 1e-5).1;
                let recipe = PeelContour::prepare_insets(
                    old,
                    8.,
                    1.,
                    0.2,
                    2. * 0.75f64.acos().to_degrees(),
                    0.01,
                    1e-5,
                    right,
                    insets.unwrap_or([[step; 2]; 2]),
                )
                .unwrap()
                .unwrap();
                let b = recipe.bounds;
                let mut axes = [vec![], vec![], vec![], vec![]];
                let mut arcs = 0;
                for curve in &recipe.contour {
                    let motion = Motion {
                        id: 1,
                        operation: "proof".into(),
                        tool: "t".into(),
                        kind: MoveKind::Cut,
                        curve: curve.clone(),
                        feed_mm_min: Some(600.),
                    };
                    let error = crate::verification::path_error(&motion, 1e-5);
                    match *curve {
                        Curve::Line { start, end } => {
                            let vertical = (end[1] - start[1]).abs() > (end[0] - start[0]).abs();
                            let mid = [(start[0] + end[0]) / 2., (start[1] + end[1]) / 2.];
                            let (side, region) = if vertical {
                                let left = mid[0] < (b[0][0] + b[1][0]) / 2.;
                                let lo = start[1].min(end[1]);
                                let hi = start[1].max(end[1]);
                                // The two left segments share q. Extend only
                                // their corner ends to the guarded rectangle.
                                let yl = if lo < b[0][1] + 0.01 { b[0][1] } else { lo };
                                let yh = if hi > b[1][1] - 0.01 { b[1][1] } else { hi };
                                (
                                    if left { 0 } else { 1 },
                                    if left {
                                        [[old[0][0], yl], [b[0][0], yh]]
                                    } else {
                                        [[b[1][0], yl], [old[1][0], yh]]
                                    },
                                )
                            } else {
                                let bottom = mid[1] < (b[0][1] + b[1][1]) / 2.;
                                (
                                    if bottom { 2 } else { 3 },
                                    if bottom {
                                        [[b[0][0], old[0][1]], [b[1][0], b[0][1]]]
                                    } else {
                                        [[b[0][0], b[1][1]], [b[1][0], old[1][1]]]
                                    },
                                )
                            };
                            axes[side].push(if vertical {
                                [region[0][1], region[1][1]]
                            } else {
                                [region[0][0], region[1][0]]
                            });
                            if (0..2).any(|k| region[0][k] >= region[1][k]) {
                                // A stationary face has no shell strip.
                                continue;
                            }
                            // Distance to a segment is convex: corner maxima
                            // certify every interior point of this strip box.
                            let max = corners(region)
                                .into_iter()
                                .map(|p| distance_to_segment(p, start, end))
                                .fold(0f64, f64::max);
                            assert!(max + error < 1., "uncleared whole side strip: {max}");
                        }
                        Curve::Arc {
                            start,
                            end,
                            center,
                            sweep_rad,
                            ..
                        } => {
                            arcs += 1;
                            let low_x = center[0] < (b[0][0] + b[1][0]) / 2.;
                            let low_y = center[1] < (b[0][1] + b[1][1]) / 2.;
                            let region = [
                                [
                                    if low_x { old[0][0] } else { b[1][0] },
                                    if low_y { old[0][1] } else { b[1][1] },
                                ],
                                [
                                    if low_x { b[0][0] } else { old[1][0] },
                                    if low_y { b[0][1] } else { old[1][1] },
                                ],
                            ];
                            if (0..2).any(|k| region[0][k] >= region[1][k]) {
                                // Degenerate corner belongs to the adjoining
                                // straight strip or the retained rectangle.
                                continue;
                            }
                            let u = [start[0] - center[0], start[1] - center[1]];
                            let v = [end[0] - center[0], end[1] - center[1]];
                            let mut far: f64 = 0.;
                            for p in corners(region) {
                                let d = [p[0] - center[0], p[1] - center[1]];
                                // Two linear halfplanes enclose the WHOLE
                                // box in this actual quarter-arc sector.
                                assert!(sweep_rad.signum() * (u[0] * d[1] - u[1] * d[0]) >= 0.);
                                assert!(sweep_rad.signum() * (v[0] * d[1] - v[1] * d[0]) <= 0.);
                                far = far.max(d[0].hypot(d[1]));
                            }
                            let nearest: Point = std::array::from_fn(|k| {
                                center[k].clamp(region[0][k], region[1][k])
                            });
                            let near = (nearest[0] - center[0]).hypot(nearest[1] - center[1]);
                            let radius = u[0].hypot(u[1]);
                            assert!(
                                (near - radius).abs().max((far - radius).abs()) + error < 1.,
                                "uncleared whole corner quadrant"
                            );
                        }
                        _ => panic!("flat recipe expected"),
                    }
                }
                assert_eq!(arcs, 4);
                assert_eq!(
                    axes.iter().map(Vec::len).collect::<Vec<_>>(),
                    vec![2, 1, 1, 1]
                );
                axes[0].sort_by(|a, b| a[0].total_cmp(&b[0]));
                assert_eq!(axes[0][0][0], b[0][1]);
                assert_eq!(axes[0][0][1], axes[0][1][0]);
                assert_eq!(axes[0][1][1], b[1][1]);
            }
        }
    }
}
