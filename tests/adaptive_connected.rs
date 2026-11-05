use camlib::{adaptive::*, motion::*, pocket::Operation, stock::*, stream::StreamManifest};
use std::{
    f64::consts::{PI, TAU},
    sync::atomic::AtomicBool,
};
fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!(
        "../fixtures/operations/adaptive-connected.json"
    ))
    .unwrap()
}
fn tapered_fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/adaptive-tapered.json")).unwrap()
}
fn operation() -> AdaptivePocketOperation {
    let Operation::AdaptivePocket(p) = fixture().operations.remove(0) else {
        panic!("fixture")
    };
    p
}
fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn point(c: &Curve, f: f64) -> Vec3 {
    let a = c.start();
    let b = c.end();
    if f == 0. {
        return a;
    }
    if f == 1. {
        return b;
    }
    match *c {
        Curve::Line { .. } => std::array::from_fn(|k| a[k] + f * (b[k] - a[k])),
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let r = dist([a[0], a[1]], [center[0], center[1]]);
            let angle = (a[1] - center[1]).atan2(a[0] - center[0]) + f * sweep_rad;
            [
                center[0] + r * angle.cos(),
                center[1] + r * angle.sin(),
                a[2],
            ]
        }
        _ => panic!("unexpected helix"),
    }
}
fn angle_on(a: f64, start: f64, sweep: f64) -> bool {
    let d = if sweep > 0. {
        (a - start).rem_euclid(TAU)
    } else {
        (start - a).rem_euclid(TAU)
    };
    sweep.abs() >= TAU - 1e-10 || d <= sweep.abs() + 1e-10 || d >= TAU - 1e-10
}
fn slice(c: &Curve, lo: f64, hi: f64) -> Curve {
    match *c {
        Curve::Line { .. } => Curve::Line {
            start: point(c, lo),
            end: point(c, hi),
        },
        Curve::Arc {
            center,
            sweep_rad,
            plane,
            ..
        } => Curve::Arc {
            start: point(c, lo),
            end: point(c, hi),
            center,
            sweep_rad: sweep_rad * (hi - lo),
            plane,
        },
        _ => panic!("unexpected helix"),
    }
}
fn path_distance(c: &Curve, p: [f64; 2], max: bool) -> f64 {
    let a = c.start();
    let b = c.end();
    match *c {
        Curve::Line { .. } => {
            if max {
                return dist([a[0], a[1]], p).max(dist([b[0], b[1]], p));
            }
            let d = [b[0] - a[0], b[1] - a[1]];
            let l = d[0] * d[0] + d[1] * d[1];
            let t = if l > 0. {
                (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l).clamp(0., 1.)
            } else {
                0.
            };
            dist([a[0] + t * d[0], a[1] + t * d[1]], p)
        }
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let r = dist([a[0], a[1]], [center[0], center[1]]);
            let d = dist([center[0], center[1]], p);
            let start = (a[1] - center[1]).atan2(a[0] - center[0]);
            let angle = (p[1] - center[1]).atan2(p[0] - center[0]) + if max { PI } else { 0. };
            let endpoints = [dist([a[0], a[1]], p), dist([b[0], b[1]], p)];
            let bound = if angle_on(angle, start, sweep_rad) {
                if max { d + r } else { (d - r).abs() }
            } else if max {
                0.
            } else {
                f64::INFINITY
            };
            if max {
                endpoints[0].max(endpoints[1]).max(bound)
            } else {
                endpoints[0].min(endpoints[1]).min(bound)
            }
        }
        _ => panic!("unexpected helix"),
    }
}
fn point_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l = d[0] * d[0] + d[1] * d[1];
    let t = if l > 0. {
        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l).clamp(0., 1.)
    } else {
        0.
    };
    dist(p, [a[0] + t * d[0], a[1] + t * d[1]])
}
fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
// Complete line/arc-to-segment minima from endpoint, stationary and intersection
// candidates. No endpoint sampling is used for original polygon containment.
fn path_segment(c: &Curve, a: [f64; 2], b: [f64; 2]) -> f64 {
    let start = c.start();
    let end = c.end();
    let u = [b[0] - a[0], b[1] - a[1]];
    let l2 = u[0] * u[0] + u[1] * u[1];
    let mut best = point_segment([start[0], start[1]], a, b)
        .min(point_segment([end[0], end[1]], a, b))
        .min(path_distance(c, a, false))
        .min(path_distance(c, b, false));
    match *c {
        Curve::Line { .. } => {
            let v = [end[0] - start[0], end[1] - start[1]];
            let det = cross(v, u);
            if det != 0. {
                let delta = [a[0] - start[0], a[1] - start[1]];
                let t = cross(delta, u) / det;
                let s = cross(delta, v) / det;
                if (0. ..=1.).contains(&t) && (0. ..=1.).contains(&s) {
                    return 0.;
                }
            }
        }
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let r = dist([start[0], start[1]], [center[0], center[1]]);
            let angle0 = (start[1] - center[1]).atan2(start[0] - center[0]);
            let foot = ((center[0] - a[0]) * u[0] + (center[1] - a[1]) * u[1]) / l2;
            let perpendicular = [
                a[0] + foot * u[0] - center[0],
                a[1] + foot * u[1] - center[1],
            ];
            let theta = perpendicular[1].atan2(perpendicular[0]);
            for angle in [theta, theta + PI] {
                let p = [center[0] + r * angle.cos(), center[1] + r * angle.sin()];
                let t = ((p[0] - a[0]) * u[0] + (p[1] - a[1]) * u[1]) / l2;
                if (0. ..=1.).contains(&t) && angle_on(angle, angle0, sweep_rad) {
                    best = best.min(point_segment(p, a, b));
                }
            }
            let h2 = perpendicular[0] * perpendicular[0] + perpendicular[1] * perpendicular[1];
            if h2 <= r * r {
                let delta = ((r * r - h2) / l2).sqrt();
                for t in [foot - delta, foot + delta] {
                    if (0. ..=1.).contains(&t) {
                        let p = [a[0] + t * u[0], a[1] + t * u[1]];
                        if angle_on(
                            (p[1] - center[1]).atan2(p[0] - center[0]),
                            angle0,
                            sweep_rad,
                        ) {
                            return 0.;
                        }
                    }
                }
            }
        }
        _ => panic!("unexpected helix"),
    }
    best
}
fn source_clearance(p: [f64; 2], poly: &[[f64; 2]]) -> f64 {
    let mut inside = false;
    let mut nearest = f64::INFINITY;
    for (&a, &b) in poly
        .iter()
        .zip(poly.iter().cycle().skip(1))
        .take(poly.len())
    {
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
        nearest = nearest.min(point_segment(p, a, b));
    }
    if inside { nearest } else { -nearest }
}
fn exposure(q: f64, a: f64) -> f64 {
    if q + 2. <= a {
        0.
    } else if q >= a + 2. {
        360.
    } else {
        let x = (q * q + 4. - a * a) / (2. * q);
        2. * (-x / 2.).clamp(-1., 1.).acos().to_degrees()
    }
}
// Whole-parameter interval certificate using ALL independently reconstructed
// prior full-orbit disks. Different disks can cover different intervals. This
// verifies long cached-return moves without trusting planner cache labels.
fn certify_exposure(c: &Curve, disks: &[([f64; 2], f64)], limit: f64) {
    let mut stack = vec![(0., 1., 0)];
    let mut work = 0;
    while let Some((lo, hi, depth)) = stack.pop() {
        let part = slice(c, lo, hi);
        let mut pass = false;
        for &(center, a) in disks.iter().rev() {
            work += 1;
            if exposure(path_distance(&part, center, true), a) <= limit + 1e-6 {
                pass = true;
                break;
            }
        }
        if pass {
            continue;
        }
        assert!(
            depth < 24 && work < 50_000_000,
            "uncertified exposure on {c:?}, interval {lo}..{hi}"
        );
        let mid = (lo + hi) / 2.;
        stack.extend([(lo, mid, depth + 1), (mid, hi, depth + 1)]);
    }
}
#[test]
fn connected_cad_passage_uses_both_radii_one_seed_and_whole_move_certificates() {
    let m = fixture();
    check_connected(m, 320., false);
}
#[test]
fn tapered_cad_passage_uses_clearance_derived_targets_and_whole_move_certificates() {
    for clockwise in [true, false] {
        let mut m = tapered_fixture();
        let Operation::AdaptivePocket(op) = &mut m.operations[0] else {
            panic!("fixture");
        };
        op.clockwise = clockwise;
        check_connected(m, 328., true);
    }
}
fn check_connected(m: StreamManifest, area: f64, local: bool) {
    m.validate().unwrap();
    let cancel = AtomicBool::new(false);
    let Operation::AdaptivePocket(op) = m.operations[0].clone() else {
        panic!("fixture");
    };
    let poly: Vec<_> = op.pocket.region.components[0]
        .outer
        .vertices
        .iter()
        .map(|p| [p[0], p[1]])
        .collect();
    let mut fixed = op.clone();
    fixed.min_orbit_radius_mm = None;
    fixed.local_orbit_tolerance_mm = None;
    let fixed = AdaptivePlan::prepare(
        fixed,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    assert_eq!(
        fixed.report().navigation_components,
        2,
        "large swept disk must not fit the neck"
    );
    let mut plan = AdaptivePlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    assert_eq!(
        plan.report().navigation_components,
        1,
        "small guides must connect both rooms"
    );
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut disks = vec![];
    let mut curves = vec![];
    let mut plunges = 0;
    let mut radii = vec![];
    let mut old = m.initial_position_mm;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        assert_eq!(motion.curve.start(), old);
        old = motion.curve.end();
        if motion.kind == MoveKind::Plunge {
            plunges += 1;
            disks.clear();
        }
        if motion.kind != MoveKind::Rapid {
            let start = motion.curve.start();
            assert!(source_clearance([start[0], start[1]], &poly) >= 2. - 1e-7);
            for (&a, &b) in poly
                .iter()
                .zip(poly.iter().cycle().skip(1))
                .take(poly.len())
            {
                assert!(
                    path_segment(&motion.curve, a, b) >= 2. - 1e-7,
                    "original polygon crossed: {motion:?}"
                );
            }
            if motion.kind != MoveKind::Retract
                && motion.kind != MoveKind::Plunge
                && !disks.is_empty()
            {
                certify_exposure(&motion.curve, &disks, 90.);
            }
        }
        if let Curve::Arc {
            center,
            start,
            sweep_rad,
            ..
        } = motion.curve
            && motion.kind == MoveKind::Cut
        {
            assert!((sweep_rad.abs() - TAU).abs() < 1e-9);
            let radius = dist([start[0], start[1]], [center[0], center[1]]);
            if disks.is_empty() {
                assert_eq!(motion.feed_mm_min, Some(150.));
            }
            radii.push(radius);
            disks.push(([center[0], center[1]], 2. + radius));
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        curves.push(motion.curve);
    }
    plan.finish_report(&cancel).unwrap();
    let r = plan.report();
    assert_eq!(plunges, 1);
    assert_eq!(r.seed_orbits, 1);
    assert!(r.radius_growth_orbits > 0);
    assert!(radii.iter().all(|x| *x >= 0.25 - 1e-8));
    assert!(r.committed_tube_advances > 0);
    assert!(r.omitted_endpoint_orbits > 0);
    assert!(radii.iter().any(|x| (*x - 1.).abs() < 1e-8));
    if !local {
        assert!(radii.iter().any(|x| (*x - 0.25).abs() < 1e-8));
    }
    assert!(r.max_bounded_engagement_deg <= 90.);
    if local {
        assert!(r.local_radius_splits > 0 && r.locally_sized_guides > 0 && r.local_radius_work > 0);
        let mut separated = vec![];
        for &radius in &r.guide_orbit_radii_mm {
            if separated
                .last()
                .is_none_or(|old: &f64| radius - *old > 0.04)
            {
                separated.push(radius);
            }
        }
        assert!(
            separated.len() > 2,
            "local guide targets were only fixed scales: {:?}",
            r.guide_orbit_radii_mm
        );
        assert!(
            r.guide_orbit_radii_mm.iter().any(|target| *target > 0.30
                && *target < 0.95
                && radii.iter().any(|actual| (actual - target).abs() < 1e-8)),
            "no actual cutting orbit attained an intermediate guide target"
        );
    }
    for (i, h) in stock.cells().iter().enumerate() {
        let xy = stock.cell_center(i).unwrap();
        let actual = curves
            .iter()
            .filter(|c| path_distance(c, xy, false) <= 2. + 1e-10)
            .map(|c| c.start()[2].min(c.end()[2]))
            .fold(10f64, f64::min);
        assert!(
            h.lower_mm <= actual + 1e-6 && h.upper_mm >= actual - 1e-6,
            "exact stock outside interval {xy:?}: {actual},{h:?}"
        );
        if source_clearance(xy, &poly) > 2.27 {
            assert!(
                actual <= 9. + 1e-7,
                "small-radius reachable core uncut at{xy:?}"
            );
        }
        if source_clearance(xy, &poly) < -1e-7 {
            assert_eq!(actual, 10.);
        }
    }
    assert!(stock.volume().removed_lower_mm3 > 250.);
    assert!(stock.volume().removed_lower_mm3 <= area + 1e-8);
    eprintln!("{}", serde_json::to_string(r).unwrap());
}

#[test]
fn proven_air_has_no_cutting_orbits_while_a_thin_wafer_keeps_cutting() {
    for m in [fixture(), tapered_fixture()] {
        check_preclear(m);
    }
}
fn check_preclear(m: StreamManifest) {
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    let Operation::AdaptivePocket(op) = m.operations[0].clone() else {
        panic!("fixture");
    };
    for top in [9., 9.000001] {
        let mut snapshot = Stock::new(m.stock.height().unwrap().clone())
            .unwrap()
            .snapshot();
        snapshot.cells.fill(HeightBounds {
            lower_mm: top,
            upper_mm: top,
        });
        let mut stock = Stock::from_snapshot(snapshot).unwrap();
        let mut plan = AdaptivePlan::prepare(
            op.clone(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let mut cuts = 0;
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            cuts += usize::from(motion.kind == MoveKind::Cut);
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
        }
        plan.finish_report(&cancel).unwrap();
        let r = plan.report();
        if top == 9. {
            assert_eq!(cuts, 0, "{}", serde_json::to_string(r).unwrap());
            assert_eq!(r.seed_orbits, 0);
            assert_eq!(r.controlled_orbits, 0);
            assert_eq!(r.omitted_seed_orbits, 1);
            assert!(r.omitted_endpoint_orbits > 20);
            assert!(r.min_cutting_orbit_radius_mm.is_none());
            if op.local_orbit_tolerance_mm.is_none() {
                assert!(r.motions < 500);
            }
            assert!(r.motions < 8 * (r.guide_strokes + r.navigation_links) + 10);
        } else {
            assert_eq!(r.seed_orbits, 1);
            assert!(cuts > 100);
            assert!(r.bounded_advances > 100);
            assert!(
                r.min_cutting_orbit_radius_mm
                    .is_some_and(|r| r >= 0.25 - 1e-8)
            );
        }
    }
}
#[test]
fn committed_tube_proofs_expire_at_depth_and_cancellation_preserves_growth_cursor() {
    for m in [fixture(), tapered_fixture()] {
        check_depth_cancel(m);
    }
}
fn check_depth_cancel(m: StreamManifest) {
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let Operation::AdaptivePocket(mut op) = m.operations[0].clone() else {
        panic!("fixture");
    };
    op.pocket.floor_z_mm = 8.;
    op.clockwise = false;
    let mut plan = AdaptivePlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut disks = vec![];
    let mut depths = vec![];
    let mut checked_cancel = false;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        if motion.kind == MoveKind::Plunge {
            disks.clear();
            depths.push(motion.curve.end()[2]);
        }
        if matches!(motion.kind, MoveKind::Link | MoveKind::Cut) && !disks.is_empty() {
            certify_exposure(&motion.curve, &disks, 90.);
        }
        if let Curve::Arc { center, start, .. } = motion.curve
            && motion.kind == MoveKind::Cut
        {
            disks.push((
                [center[0], center[1]],
                2. + dist([start[0], start[1]], [center[0], center[1]]),
            ));
        }
        if !checked_cancel && plan.report().radius_changes > 0 {
            assert!(
                plan.next_motion_with_stock(&stock, &cancel)
                    .unwrap_err()
                    .contains("commit")
            );
            checked_cancel = true;
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        if checked_cancel && plan.report().radius_changes == 1 {
            let before = plan.report().motions;
            cancel.store(true, std::sync::atomic::Ordering::Relaxed);
            assert!(plan.next_motion_with_stock(&stock, &cancel).is_err());
            assert_eq!(plan.report().motions, before);
            cancel.store(false, std::sync::atomic::Ordering::Relaxed);
        }
    }
    plan.finish_report(&cancel).unwrap();
    assert!(checked_cancel);
    assert_eq!(depths, [9., 8.]);
    assert_eq!(plan.report().seed_orbits, 2);
    assert!(plan.report().max_bounded_engagement_deg <= 90.);
    for min in [0., -1., 2., f64::NAN] {
        let mut bad = operation();
        bad.min_orbit_radius_mm = Some(min);
        assert!(
            AdaptivePlan::prepare(
                bad,
                &m.tools[0],
                m.stock.height().unwrap(),
                m.initial_position_mm,
                1,
                &cancel
            )
            .is_err()
        );
    }
}

#[test]
fn stock_front_completes_connected_cad_passage_with_source_and_stock_checks() {
    check_front_passage(fixture());
}
#[test]
fn stock_front_completes_tapered_cad_passage_with_source_and_stock_checks() {
    check_front_passage(tapered_fixture());
}
fn check_front_passage(m: StreamManifest) {
    let Operation::AdaptivePocket(mut op) = m.operations[0].clone() else {
        panic!("fixture")
    };
    op.path_strategy = AdaptivePathStrategy::StockFront;
    let poly: Vec<_> = op.pocket.region.components[0]
        .outer
        .vertices
        .iter()
        .map(|v| [v[0], v[1]])
        .collect();
    let radius = m.tools[0].cutter.diameter_mm / 2.;
    let mut plan = AdaptivePlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut curves = vec![];
    let mut position = m.initial_position_mm;
    let mut checked_link_receipt = false;
    let mut link_count = 0;
    loop {
        let result = plan.next_motion_with_stock(&stock, &|| false);
        let motion = result.unwrap_or_else(|error| {
            panic!(
                "failed after {}motions at {position:?}: {error}; {:?}; last curves {:?}",
                curves.len(),
                plan.report().front,
                &curves[curves.len().saturating_sub(8)..]
            )
        });
        let Some(motion) = motion else { break };
        assert_eq!(motion.curve.start(), position);
        let links = plan
            .report()
            .front
            .as_ref()
            .map_or(0, |r| r.cleared_reentry_links);
        if links > link_count && !checked_link_receipt {
            assert_eq!(motion.kind, MoveKind::Link);
            let mut partial_stock = stock.clone();
            let partial = partial_stock
                .preview_sweep(&tool, &motion, 0.5, m.sweep, &|| false)
                .unwrap();
            partial_stock.apply_delta(&partial).unwrap();
            assert!(
                plan.next_motion_with_stock(&partial_stock, &|| false)
                    .is_err()
            );
            checked_link_receipt = true;
        }
        link_count = links;
        if motion.kind != MoveKind::Rapid {
            let start = motion.curve.start();
            assert!(source_clearance([start[0], start[1]], &poly) >= radius - 1e-7);
            for (&a, &b) in poly
                .iter()
                .zip(poly.iter().cycle().skip(1))
                .take(poly.len())
            {
                assert!(
                    path_segment(&motion.curve, a, b) >= radius - 1e-7,
                    "source crossed: {motion:?}"
                );
            }
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        position = motion.curve.end();
        curves.push(motion.curve);
    }
    plan.finish_report(&|| false).unwrap();
    let front = plan.report().front.as_ref().unwrap();
    assert!(front.coverage_complete && checked_link_receipt);
    assert!(front.max_bounded_engagement_deg <= 90.);
    assert!(front.cleared_reentry_links > 0);
    assert!(front.reentries - front.cleared_reentry_links < 400);
    // Unchanged inputs used 4,149 and 4,157 moves before goal steering and
    // segment re-entry/cleared links. Keep at least a 10% whole-job reduction.
    assert!(curves.len() < 3734);
    for (i, h) in stock.cells().iter().enumerate() {
        let xy = stock.cell_center(i).unwrap();
        let actual = curves
            .iter()
            .filter(|c| path_distance(c, xy, false) <= radius + 1e-10)
            .map(|c| c.start()[2].min(c.end()[2]))
            .fold(10f64, f64::min);
        assert!(
            h.lower_mm <= actual + 1e-6 && h.upper_mm >= actual - 1e-6,
            "cell{xy:?}: {h:?} vs {actual}"
        );
        if source_clearance(xy, &poly) > 2.27 {
            assert!(actual <= 9. + 1e-7, "uncut reachable core {xy:?}");
        }
        if source_clearance(xy, &poly) < -1e-7 {
            assert_eq!(actual, 10.);
        }
    }
    eprintln!(
        "passage complete: {}motions; {:?}",
        curves.len(),
        plan.report().front
    );
}
