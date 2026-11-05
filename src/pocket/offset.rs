//! Absolute contour erosions with an independent whole-tile sweep enclosure.
use super::*;
use crate::{
    geometry::Bounds,
    verification::{AxisSpan, path_error},
};

const MAX_DEPTH: u8 = 28;

#[derive(Debug, Serialize)]
pub struct OffsetReport {
    pub requested_spacing_mm: f64,
    pub applied_spacing_mm: f64,
    pub coverage_margin_mm: f64,
    pub source_numerical_reserve_mm: f64,
    pub construction_error_mm: f64,
    pub levels: usize,
    pub loops: usize,
    pub arc_strokes: usize,
    pub coverage_tiles: usize,
    pub covered_tiles: usize,
    pub exterior_tiles: usize,
    pub coverage_max_depth: u8,
    pub coverage_max_bound_mm: f64,
    pub coverage_index_nodes: usize,
    pub work: usize,
    pub coverage_scope: &'static str,
}

struct Work<'a, C> {
    cancel: &'a C,
    used: usize,
    limit: usize,
}
impl<C: CancelSignal> Work<'_, C> {
    fn charge(&mut self, n: usize) -> Result<(), String> {
        if self.cancel.is_cancelled() {
            return Err("offset clearing cancelled".into());
        }
        self.used = self.used.checked_add(n).ok_or("offset work overflow")?;
        if self.used > self.limit {
            return Err("offset clearing work limit exceeded".into());
        }
        Ok(())
    }
}

struct Witness {
    bounds: Bounds,
    axis: AxisSpan,
    error: f64,
}
struct Node {
    bounds: Bounds,
    range: std::ops::Range<usize>,
    children: Option<[usize; 2]>,
}
struct CoverageIndex {
    curves: Vec<Witness>,
    nodes: Vec<Node>,
}
impl CoverageIndex {
    fn new(
        strokes: &[Stroke],
        tolerance: f64,
        work: &mut Work<'_, impl CancelSignal>,
    ) -> Result<Self, String> {
        let mut curves = Vec::with_capacity(strokes.len());
        for &stroke in strokes {
            work.charge(1)?;
            let motion = Motion {
                id: 1,
                operation: "offset-coverage".into(),
                tool: "coverage".into(),
                kind: MoveKind::Cut,
                curve: stroke.curve(0.),
                feed_mm_min: Some(1.),
            };
            let path = motion.curve.prepare(tolerance)?;
            let b = path.bounds();
            curves.push(Witness {
                bounds: Bounds {
                    min: [b.min[0], b.min[1]],
                    max: [b.max[0], b.max[1]],
                },
                axis: AxisSpan::new(&motion, &path, [0., 1.])
                    .ok_or("unsupported offset coverage curve")?,
                error: path_error(&motion, tolerance),
            });
        }
        let mut result = Self {
            curves,
            nodes: vec![],
        };
        if !result.curves.is_empty() {
            result.build(0..result.curves.len(), work)?;
        }
        Ok(result)
    }
    fn build(
        &mut self,
        range: std::ops::Range<usize>,
        work: &mut Work<'_, impl CancelSignal>,
    ) -> Result<usize, String> {
        // Includes sorting cost conservatively rather than counting only nodes.
        work.charge(range.len() * (1 + range.len().ilog2() as usize))?;
        let bounds = self.curves[range.clone()]
            .iter()
            .map(|c| c.bounds)
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
            self.curves[range.clone()].sort_unstable_by(|a, b| {
                (a.bounds.min[axis] + a.bounds.max[axis])
                    .total_cmp(&(b.bounds.min[axis] + b.bounds.max[axis]))
            });
            let mid = range.start + range.len() / 2;
            let left = self.build(range.start..mid, work)?;
            let right = self.build(mid..range.end, work)?;
            self.nodes[index].children = Some([left, right]);
        }
        Ok(index)
    }
    fn witness(
        &self,
        index: usize,
        point: Point,
        limit: f64,
        work: &mut Work<'_, impl CancelSignal>,
    ) -> Result<Option<f64>, String> {
        work.charge(1)?;
        let node = &self.nodes[index];
        if node.bounds.distance_sq(point) > limit * limit {
            return Ok(None);
        }
        if let Some(mut children) = node.children {
            if self.nodes[children[0]].bounds.distance_sq(point)
                > self.nodes[children[1]].bounds.distance_sq(point)
            {
                children.swap(0, 1);
            }
            for child in children {
                if let Some(d) = self.witness(child, point, limit, work)? {
                    return Ok(Some(d));
                }
            }
        } else {
            for curve in &self.curves[node.range.clone()] {
                work.charge(1)?;
                let d = curve.axis.distance(point) + curve.error;
                if d <= limit {
                    return Ok(Some(d));
                }
            }
        }
        Ok(None)
    }
}

fn certify(
    domain: &PreparedRegion,
    strokes: &[Stroke],
    radius: f64,
    tolerance: f64,
    report: &mut OffsetReport,
    work: &mut Work<'_, impl CancelSignal>,
) -> Result<(), String> {
    if domain.definition().components.is_empty() {
        return Ok(());
    }
    let index = CoverageIndex::new(strokes, tolerance, work)?;
    if index.nodes.is_empty() {
        return Err("offset coverage has no emitted curves".into());
    }
    // Derive bounds independently from D, never from the candidate paths: a
    // missing outer contour must not shrink the area being checked.
    let mut domain_bounds: Option<Bounds> = None;
    for component in &domain.definition().components {
        for boundary in std::iter::once(&component.outer).chain(&component.holes) {
            for stroke in Stroke::from_boundary(boundary) {
                work.charge(1)?;
                let b = stroke.curve(0.).prepare(tolerance)?.bounds();
                let b = Bounds {
                    min: [b.min[0], b.min[1]],
                    max: [b.max[0], b.max[1]],
                };
                domain_bounds = Some(domain_bounds.map_or(b, |old| old.union(b)));
            }
        }
    }
    let mut stack = vec![(domain_bounds.ok_or("offset domain has no bounds")?, 0u8)];
    let segments = domain.summary().segments;
    let safe_radius = radius - report.coverage_margin_mm;
    while let Some((bounds, depth)) = stack.pop() {
        report.coverage_tiles += 1;
        work.charge(1 + segments)?;
        report.coverage_max_depth = report.coverage_max_depth.max(depth);
        let p = std::array::from_fn(|k| (bounds.min[k] + bounds.max[k]) / 2.);
        let half = norm(bounds.min, bounds.max) / 2.;
        let query = domain.query(p)?;
        let guard = 256. * f64::EPSILON * (1. + p[0].abs() + p[1].abs() + half + radius);
        if query
            .signed_clearance_mm
            .is_some_and(|d| d + half + query.uncertainty_mm + guard < 0.)
        {
            report.exterior_tiles += 1;
            continue;
        }
        let limit = safe_radius - half - guard;
        if limit > 0.
            && let Some(distance) = index.witness(0, p, limit, work)?
        {
            report.covered_tiles += 1;
            report.coverage_max_bound_mm =
                report.coverage_max_bound_mm.max(distance + half + guard);
            continue;
        }
        if depth >= MAX_DEPTH || half <= 16. * tolerance {
            return Err(format!(
                "offset coverage unresolved near {p:?}; no whole-tile sweep bound"
            ));
        }
        for quadrant in 0..4 {
            let min = std::array::from_fn(|k| {
                if quadrant & (1 << k) == 0 {
                    bounds.min[k]
                } else {
                    p[k]
                }
            });
            let max = std::array::from_fn(|k| {
                if quadrant & (1 << k) == 0 {
                    p[k]
                } else {
                    bounds.max[k]
                }
            });
            stack.push((Bounds { min, max }, depth + 1));
        }
    }
    report.coverage_index_nodes = index.nodes.len();
    Ok(())
}

pub(crate) fn validate_margin(radius: f64, margin: f64) -> Result<(), String> {
    if !margin.is_finite() || margin <= 0. || margin > radius / 4. {
        return Err(
            "offset coverage margin must be positive and at most one quarter of cutter radius"
                .into(),
        );
    }
    Ok(())
}

pub(super) fn generate(
    domain: &PreparedRegion,
    boundaries: Vec<Stroke>,
    operation: &PocketOperation,
    radius: f64,
    margin: f64,
    cancel: &impl CancelSignal,
) -> Result<(Vec<Stroke>, OffsetReport), String> {
    let tolerance = operation.tolerance_mm;
    let reserve = domain.definition().boundary_error_mm + 32. * tolerance;
    validate_margin(radius, margin)?;
    let spacing = operation.stepover_mm.min(radius - margin - reserve);
    if spacing < 16. * tolerance {
        return Err("offset spacing has no room for coverage/source/numerical reserves".into());
    }
    let mut report = OffsetReport {
        requested_spacing_mm: operation.stepover_mm,
        applied_spacing_mm: spacing,
        coverage_margin_mm: margin,
        source_numerical_reserve_mm: reserve,
        construction_error_mm: domain.definition().boundary_error_mm,
        levels: 0,
        loops: 0,
        arc_strokes: 0,
        coverage_tiles: 0,
        covered_tiles: 0,
        exterior_tiles: 0,
        coverage_max_depth: 0,
        coverage_max_bound_mm: 0.,
        coverage_index_nodes: 0,
        work: 0,
        coverage_scope: "whole centre-domain tiles enclosed by actual analytic line/arc cutter sweeps; original boundary sweep covers exterior radius band; engineering bounds",
    };
    let mut work = Work {
        cancel,
        used: 0,
        limit: usize::MAX,
    };
    work.charge(boundaries.len())?;
    let mut strokes = boundaries;
    if !strokes.is_empty() {
        report.levels = 1;
        report.loops = domain.summary().components + domain.summary().holes;
        let mut terminated = false;
        // Each level erodes by `spacing`; a region inside a box of diagonal D
        // is empty once eroded by D/2, so this bound is derived, not imposed.
        let mut low = [f64::INFINITY; 2];
        let mut high = [f64::NEG_INFINITY; 2];
        for component in &domain.definition().components {
            for boundary in std::iter::once(&component.outer).chain(&component.holes) {
                for v in &boundary.vertices {
                    for k in 0..2 {
                        low[k] = low[k].min(v[k]);
                        high[k] = high[k].max(v[k]);
                    }
                }
            }
        }
        let diagonal = (high[0] - low[0]).hypot(high[1] - low[1]);
        let max_levels = (diagonal / spacing).ceil() as usize + 2;
        for level in 1..=max_levels {
            work.charge(domain.summary().segments)?;
            let erosion = domain.offset(level as f64 * spacing).map_err(|e| {
                format!(
                    "offset level {level} at distance {}: {e}",
                    level as f64 * spacing
                )
            })?;
            report.construction_error_mm = report
                .construction_error_mm
                .max(erosion.region.boundary_error_mm);
            if erosion.region.components.is_empty() {
                terminated = true;
                break;
            }
            if level == max_levels {
                break;
            }
            report.levels += 1;
            for component in &erosion.region.components {
                for boundary in std::iter::once(&component.outer).chain(&component.holes) {
                    work.charge(boundary.vertices.len())?;
                    let mut ring = Stroke::from_boundary(boundary);
                    if operation.boundary_clockwise == Some(true) {
                        ring = ring.into_iter().rev().map(Stroke::reversed).collect();
                    }
                    let before = strokes.len();
                    strokes.extend(ring);
                    if strokes.len() >> 16 != before >> 16 {
                        // Actual size so far (x2 for Vec growth).
                        crate::resources::check_memory(
                            strokes.len() as u128 * 2 * std::mem::size_of::<Stroke>() as u128,
                            "offset clearing rings",
                        )?;
                    }
                    report.loops += 1;
                }
            }
        }
        if !terminated {
            return Err("offset clearing did not converge within its geometric level bound".into());
        }
    }
    report.arc_strokes = strokes.iter().filter(|s| s.arc.is_some()).count();
    certify(domain, &strokes, radius, tolerance, &mut report, &mut work)?;
    report.work = work.used;
    Ok((strokes, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planar::Component;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    fn operation() -> PocketOperation {
        let manifest: crate::stream::StreamManifest = serde_json::from_str(include_str!(
            "../../fixtures/operations/helical-pocket.json"
        ))
        .unwrap();
        let Operation::Pocket(mut op) = manifest.operations[0].clone() else {
            panic!("fixture")
        };
        op.stepover_mm = 0.8;
        op.boundary_clockwise = Some(false);
        op
    }
    fn prepared_rest(clockwise: bool, rest: bool) -> (PocketPlan, crate::stream::StreamManifest) {
        let m: crate::stream::StreamManifest = serde_json::from_str(include_str!(
            "../../fixtures/operations/helical-pocket.json"
        ))
        .unwrap();
        let mut op = operation();
        op.boundary_clockwise = Some(clockwise);
        op.rest_tolerance_mm = rest.then_some(0.01);
        let plan = PocketPlan::prepare_offset(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            0.01,
            &|| false,
        )
        .unwrap();
        (plan, m)
    }

    #[test]
    fn offset_rest_keeps_scheduled_whole_arcs_order_sense_and_coverage() {
        for clockwise in [false, true] {
            let (whole, _) = prepared_rest(clockwise, false);
            let (rest, _) = prepared_rest(clockwise, true);
            assert_eq!(
                whole.report.offset.as_ref().unwrap().coverage_tiles,
                rest.report.offset.as_ref().unwrap().coverage_tiles
            );
            assert_eq!(
                whole.report.routing.as_ref().unwrap().boundary_rotations,
                rest.report.routing.as_ref().unwrap().boundary_rotations
            );
            let mut start = 0;
            let mut arcs = 0;
            for guide in &whole.strokes {
                let end = rest.rest_group_ends[start];
                let pieces = &rest.strokes[start..end];
                assert_eq!(pieces.first().unwrap().a, guide.a);
                assert_eq!(pieces.last().unwrap().b, guide.b);
                assert!(rest.rest_group_ends[start..end].iter().all(|&i| i == end));
                let original = guide
                    .curve(0.)
                    .prepare(whole.operation.tolerance_mm)
                    .unwrap();
                for (i, piece) in pieces.iter().enumerate() {
                    let curve = piece
                        .curve(0.)
                        .prepare(rest.operation.tolerance_mm)
                        .unwrap();
                    for t in [0., 0.25, 0.5, 0.75, 1.] {
                        let expected = original
                            .position((i as f64 + t) / pieces.len() as f64)
                            .unwrap();
                        let actual = curve.position(t).unwrap();
                        assert!((actual[0] - expected[0]).hypot(actual[1] - expected[1]) < 1e-9);
                    }
                    if let Some((center, sweep)) = guide.arc {
                        arcs += 1;
                        let (c, s) = piece.arc.expect("arc replaced by chord");
                        assert_eq!(center, c);
                        assert_eq!(s.signum(), sweep.signum());
                    } else {
                        assert!(piece.arc.is_none());
                    }
                }
                start = end;
            }
            assert_eq!(start, rest.strokes.len());
            assert!(arcs > 0);
            // Reordering a subdivided rest plan would invalidate guide groups.
            assert!(
                rest.schedule_guides(RasterRouting::Economical, &|| false)
                    .is_err()
            );
        }
    }

    #[test]
    fn offset_rest_unknown_stock_retains_whole_guides_and_cancel_retries() {
        let (mut rest, m) = prepared_rest(false, true);
        let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let before = (rest.stroke, rest.pass, rest.position);
        let cancel = CancelAfter(AtomicUsize::new(12));
        assert!(
            rest.next_motion_with_stock(&stock, &cancel)
                .unwrap_err()
                .contains("cancel")
        );
        assert_eq!((rest.stroke, rest.pass, rest.position), before);
        assert!(rest.pending.is_empty());
        let (mut expected, _) = prepared_rest(false, true);
        assert_eq!(
            rest.next_motion_with_stock(&stock, &|| false).unwrap(),
            expected.next_motion_with_stock(&stock, &|| false).unwrap()
        );
    }

    #[test]
    fn rotated_retained_source_offset_preflight_regression() {
        let mut manifest: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../../fixtures/operations/roughing.json")).unwrap();
        let Operation::Roughing(op) = &mut manifest.operations[0] else {
            panic!("roughing")
        };
        op.clearing = crate::roughing::RoughingClearing::Offset {
            stepover_mm: 0.8,
            coverage_margin_mm: 0.01,
        };
        for triangle in std::sync::Arc::make_mut(&mut op.target.triangles) {
            for p in triangle {
                let (x, y) = (p[0] - 16., p[1] - 14.);
                p[0] = 16. + x * 0.61f64.cos() - y * 0.61f64.sin();
                p[1] = 14. + x * 0.61f64.sin() + y * 0.61f64.cos();
            }
        }
        manifest.stock.height_mut().unwrap().min_mm = [-4., -4., 0.];
        manifest.stock.height_mut().unwrap().max_mm = [36., 32., 10.];
        crate::operation::OperationPlan::prepare(
            manifest.operations[0].clone(),
            &manifest.tools[0],
            manifest.stock.height().unwrap(),
            manifest.initial_position_mm,
            1,
            &AtomicBool::new(false),
        )
        .unwrap();
    }
    fn rectangle(a: Point, b: Point) -> Component {
        Component {
            outer: Boundary {
                vertices: vec![
                    [a[0], a[1], 0.],
                    [b[0], a[1], 0.],
                    [b[0], b[1], 0.],
                    [a[0], b[1], 0.],
                ],
            },
            holes: vec![],
        }
    }
    fn region(components: Vec<Component>) -> PreparedRegion {
        PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "offset-test".into(),
            boundary_error_mm: 0.002,
            components,
            source: serde_json::Value::Null,
        }
        .prepare(ConstructionOptions::default())
        .unwrap()
    }
    fn boundaries(domain: &PreparedRegion, clockwise: bool) -> Vec<Stroke> {
        domain
            .definition()
            .components
            .iter()
            .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
            .flat_map(|b| {
                let mut ring = Stroke::from_boundary(b);
                if clockwise {
                    ring = ring.into_iter().rev().map(Stroke::reversed).collect();
                }
                ring
            })
            .collect()
    }
    // Independent point-to-primitive oracle for grid coverage, supplementing
    // (not replacing) the production whole-tile enclosure.
    fn distance(s: Stroke, p: Point) -> f64 {
        if let Some((c, sweep)) = s.arc {
            let phase = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
            let angle = (p[1] - c[1]).atan2(p[0] - c[0]);
            let travel = if sweep > 0. {
                angle - phase
            } else {
                phase - angle
            }
            .rem_euclid(TAU);
            if travel <= sweep.abs() {
                return (norm(c, p) - norm(c, s.a)).abs();
            }
            norm(p, s.a).min(norm(p, s.b))
        } else {
            let v = [s.b[0] - s.a[0], s.b[1] - s.a[1]];
            let t = (((p[0] - s.a[0]) * v[0] + (p[1] - s.a[1]) * v[1])
                / (v[0] * v[0] + v[1] * v[1]))
                .clamp(0., 1.);
            norm(p, [s.a[0] + v[0] * t, s.a[1] + v[1] * t])
        }
    }
    #[test]
    fn absolute_offsets_cover_round_island_disconnected_and_disappearing_regions_in_both_senses() {
        let mut annulus = PlanarRegion::circle("annulus", [0., 0.], 8.).unwrap();
        annulus.components[0].holes.push(
            PlanarRegion::circle("hole", [0., 0.], 2.)
                .unwrap()
                .components
                .remove(0)
                .outer,
        );
        let mut cases = vec![
            region(annulus.components),
            region(vec![
                rectangle([0., 0.], [12., 6.]),
                rectangle([15., 0.], [15.3, 5.]),
                rectangle([18., 0.], [18.4, 0.4]),
            ]),
        ];
        let mut rotated = rectangle([-8., -3.], [8., 3.]);
        for p in &mut rotated.outer.vertices {
            let (x, y) = (p[0], p[1]);
            p[0] = x * 0.61f64.cos() - y * 0.61f64.sin();
            p[1] = x * 0.61f64.sin() + y * 0.61f64.cos();
        }
        cases.push(region(vec![rotated]));
        let cancel = AtomicBool::new(false);
        for domain in cases {
            let mut prior: Option<Vec<Curve>> = None;
            for clockwise in [false, true] {
                let mut op = operation();
                op.boundary_clockwise = Some(clockwise);
                let (strokes, report) = generate(
                    &domain,
                    boundaries(&domain, clockwise),
                    &op,
                    1.,
                    0.01,
                    &cancel,
                )
                .unwrap();
                assert!(report.levels > 1 && report.loops >= report.levels);
                assert!(report.covered_tiles > 0 && report.coverage_max_bound_mm <= 0.99);
                assert_eq!(report.applied_spacing_mm, 0.8);
                assert!(report.source_numerical_reserve_mm > 0.002);
                assert!(report.work > 0);
                let mut i = 0;
                let mut loops = 0;
                while i < strokes.len() {
                    let start = strokes[i].a;
                    loop {
                        let old = strokes[i];
                        i += 1;
                        if old.b == start {
                            break;
                        }
                        assert_eq!(old.b, strokes[i].a, "open contour");
                    }
                    loops += 1;
                }
                assert_eq!(loops, report.loops);
                let mut actual: Vec<_> = strokes
                    .iter()
                    .map(|s| {
                        if clockwise {
                            s.reversed().curve(0.)
                        } else {
                            s.curve(0.)
                        }
                    })
                    .collect();
                actual.sort_by_key(|c| serde_json::to_string(c).unwrap());
                if let Some(old) = &prior {
                    assert_eq!(&actual, old);
                }
                prior = Some(actual);
                for ix in -40..100 {
                    for iy in -40..40 {
                        let p = [ix as f64 * 0.2, iy as f64 * 0.2];
                        if domain
                            .query(p)
                            .unwrap()
                            .signed_clearance_mm
                            .is_some_and(|d| d >= 0.)
                        {
                            let distance = strokes
                                .iter()
                                .map(|&s| distance(s, p))
                                .fold(f64::INFINITY, f64::min);
                            assert!(distance < 0.99, "gap {p:?}: {distance}");
                        }
                    }
                }
            }
        }
    }
    struct CancelAfter(AtomicUsize);
    impl CancelSignal for CancelAfter {
        fn is_cancelled(&self) -> bool {
            // Count down to zero; cancelled once nothing is left.
            let mut left = self.0.load(Ordering::Relaxed);
            loop {
                let Some(next) = left.checked_sub(1) else {
                    return true;
                };
                match self
                    .0
                    .compare_exchange_weak(left, next, Ordering::Relaxed, Ordering::Relaxed)
                {
                    Ok(_) => return false,
                    Err(actual) => left = actual,
                }
            }
        }
    }
    #[test]
    fn coverage_rejects_missing_inner_or_outer_paths_and_bounded_work_or_cancellation() {
        let domain = region(vec![rectangle([0., 0.], [12., 12.])]);
        let cancel = AtomicBool::new(false);
        let op = operation();
        let (strokes, mut report) =
            generate(&domain, boundaries(&domain, false), &op, 1., 0.01, &cancel).unwrap();
        for incomplete in [&strokes[..4], &strokes[4..]] {
            report.coverage_tiles = 0;
            let mut work = Work {
                cancel: &cancel,
                used: 0,
                limit: usize::MAX,
            };
            assert!(
                certify(
                    &domain,
                    incomplete,
                    1.,
                    op.tolerance_mm,
                    &mut report,
                    &mut work
                )
                .is_err(),
                "missing contours accepted"
            );
        }
        // An explicit caller budget is still enforced by `Work`.
        let mut work = Work {
            cancel: &cancel,
            used: 10,
            limit: 10,
        };
        assert!(
            CoverageIndex::new(&strokes, op.tolerance_mm, &mut work)
                .err()
                .unwrap()
                .contains("work limit")
        );
        let cancel = CancelAfter(AtomicUsize::new(20));
        assert!(
            generate(&domain, boundaries(&domain, false), &op, 1., 0.01, &cancel)
                .err()
                .unwrap()
                .contains("cancelled")
        );
        for margin in [f64::NAN, -0.1, 0., 0.251] {
            assert!(
                generate(
                    &domain,
                    boundaries(&domain, false),
                    &op,
                    1.,
                    margin,
                    &AtomicBool::new(false)
                )
                .is_err()
            );
        }
        let empty = region(vec![]);
        let (strokes, report) =
            generate(&empty, vec![], &op, 1., 0.01, &AtomicBool::new(false)).unwrap();
        assert!(strokes.is_empty());
        assert_eq!((report.levels, report.coverage_tiles), (0, 0));
        let mut op = op;
        op.stepover_mm = 1.;
        let (_, report) = generate(
            &domain,
            boundaries(&domain, false),
            &op,
            1.,
            0.01,
            &AtomicBool::new(false),
        )
        .unwrap();
        assert!(report.applied_spacing_mm < 0.99 && report.applied_spacing_mm > 0.98);
        assert_eq!(report.requested_spacing_mm, 1.);
    }
}
