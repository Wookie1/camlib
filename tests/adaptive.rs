use camlib::{
    adaptive::*, motion::*, planar::*, pocket::Operation, stock::*, stream::StreamManifest,
};
use crate::roughing::LeadingOracle;
use std::{
    f64::consts::{PI, TAU},
    sync::atomic::AtomicBool,
};
fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json")).unwrap()
}
fn operation() -> AdaptivePocketOperation {
    let Operation::AdaptivePocket(p) = fixture().operations.remove(0) else {
        panic!("fixture")
    };
    p
}
/// What the sampled check may read above the proven limit: the oracle's
/// discs leave slivers step^2 / (8 r) deep between samples r / 20 apart, and
/// it counts whole degrees of the leading half-circle. Scale-free, as in the
/// roughing tests.
pub(crate) const LEADING_SLACK_DEG: f64 = 3.;

/// #190: the independent, sampled leading-contact check from the roughing
/// tests (`LeadingOracle`), run over an adaptive pocket's motions. Per depth
/// level, flat feed moves clear their sweep; every cutting and stay-down link
/// stroke has its leading contact measured first. No planner, prover or stock
/// code: material is the stock footprint less what earlier centres swept.
/// Returns the worst contact, in degrees, over all levels.
pub(crate) fn leading_contact_deg(motions: &[Motion], r: f64, footprint: [[f64; 2]; 2]) -> f64 {
    let mut worst = 0f64;
    let mut leading = LeadingOracle::new(r, footprint);
    let mut depth = f64::INFINITY;
    for motion in motions {
        let c = &motion.curve;
        let flat = c.start()[2] == c.end()[2] && !matches!(c, Curve::Helix { .. });
        if !flat || !motion.kind.cuts() {
            continue;
        }
        if c.start()[2] < depth - 1e-7 {
            // A new level: nothing is cleared at this depth yet.
            worst = worst.max(leading.worst_deg);
            leading = LeadingOracle::new(r, footprint);
            depth = c.start()[2];
        }
        leading.feed(c, matches!(motion.kind, MoveKind::Cut | MoveKind::Link));
    }
    worst.max(leading.worst_deg)
}
/// #190: the sampled check measures what it claims, so a pocket test that
/// passes it says something: a slot through fresh stock reads the full
/// leading half-circle, and a pass beside it at width w reads acos(1 - w / r)
/// (within the sampling slack above, and the whole-degree count below).
#[test]
fn leading_contact_check_reads_a_slot_and_a_side_step() {
    let r = 2.;
    let line = |y: f64, from: f64, to: f64| Curve::Line {
        start: [from, y, 9.],
        end: [to, y, 9.],
    };
    let mut leading = LeadingOracle::new(r, [[0., 0.], [26., 26.]]);
    leading.feed(&line(5., 2., 24.), true);
    assert!(leading.worst_deg >= 175., "slot read {}", leading.worst_deg);
    for w in [0.5, 1., 1.5] {
        let mut leading = LeadingOracle::new(r, [[0., 0.], [26., 26.]]);
        // The first pass runs right through the stock, so the second meets
        // no end of it (where it would cut a wider arc, and the check says so).
        leading.feed(&line(5., -3., 29.), false);
        leading.feed(&line(5. + w, 2., 24.), true);
        let exact = (1f64 - w / r).acos().to_degrees();
        assert!(
            leading.worst_deg <= exact + LEADING_SLACK_DEG && leading.worst_deg >= exact - 2.,
            "w {w}: read {} against {exact}",
            leading.worst_deg
        );
    }
}

/// #190 review: the reading does not depend on how a cut is split into
/// moves. The same straight cut through fresh stock, fed as one move or as
/// many moves shorter than the sampling step, reads the full leading
/// half-circle; so does a side step split the same way read its width.
#[test]
fn leading_contact_check_does_not_depend_on_move_length() {
    let r = 2.;
    let footprint = [[0., 0.], [26., 26.]];
    let split = |leading: &mut LeadingOracle, y: f64, from: f64, to: f64, n: usize| {
        for i in 0..n {
            let a = from + (to - from) * i as f64 / n as f64;
            let b = from + (to - from) * (i + 1) as f64 / n as f64;
            leading.feed(
                &Curve::Line {
                    start: [a, y, 9.],
                    end: [b, y, 9.],
                },
                true,
            );
        }
    };
    for n in [1, 200] {
        let mut leading = LeadingOracle::new(r, footprint);
        split(&mut leading, 5., 10., 10.1, n);
        assert!(leading.worst_deg >= 175., "{n} moves: slot read {}", leading.worst_deg);
    }
    let w = 1.;
    let exact = (1f64 - w / r).acos().to_degrees();
    for n in [1, 200] {
        let mut leading = LeadingOracle::new(r, footprint);
        leading.feed(
            &Curve::Line {
                start: [-3., 5., 9.],
                end: [29., 5., 9.],
            },
            false,
        );
        split(&mut leading, 5. + w, 10., 10.1, n);
        assert!(
            leading.worst_deg <= exact + LEADING_SLACK_DEG && leading.worst_deg >= exact - 2.,
            "{n} moves: side step read {} against {exact}",
            leading.worst_deg
        );
    }
}

/// #190 review: an unchecked lead does not clear stock ahead of its end. A
/// short checked cut that continues a lead reads the full slot whether the
/// lead is one move or 2,000.
#[test]
fn leading_contact_check_reads_a_cut_after_a_lead_whatever_the_lead_split() {
    let r = 2.;
    let footprint = [[0., 0.], [26., 26.]];
    let line = |from: f64, to: f64| Curve::Line {
        start: [from, 5., 9.],
        end: [to, 5., 9.],
    };
    for n in [1, 2000] {
        let mut leading = LeadingOracle::new(r, footprint);
        for i in 0..n {
            let a = 9. + i as f64 / n as f64;
            leading.feed(&line(a, a + 1. / n as f64), false);
        }
        leading.feed(&line(10., 10.0005), true);
        assert!(
            leading.worst_deg >= 175.,
            "lead in {n} moves: the cut read {}",
            leading.worst_deg
        );
    }
}

fn notch_clearance(p: [f64; 2]) -> f64 {
    (11. - p[0]).max(0.).hypot((11. - p[1]).max(0.))
}
fn l_clearance(p: [f64; 2]) -> f64 {
    (p[0] - 3.)
        .min(p[1] - 3.)
        .min(23. - p[0])
        .min(23. - p[1])
        .min(notch_clearance(p))
}
fn assert_entire_l_sweep(c: &Curve, r: f64) {
    let a = c.start();
    let b = c.end();
    let mut points = vec![[a[0], a[1]], [b[0], b[1]]];
    match *c {
        Curve::Line { .. } => {
            let d = [b[0] - a[0], b[1] - a[1]];
            let l = d[0] * d[0] + d[1] * d[1];
            let mut ts = vec![];
            if l > 0. {
                ts.push(((11. - a[0]) * d[0] + (11. - a[1]) * d[1]) / l);
            }
            for k in 0..2 {
                if d[k] != 0. {
                    ts.push((11. - a[k]) / d[k]);
                }
            }
            points.extend(
                ts.into_iter()
                    .filter(|t| (0. ..=1.).contains(t))
                    .map(|t| [a[0] + t * d[0], a[1] + t * d[1]]),
            );
        }
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let rho = dist([a[0], a[1]], [center[0], center[1]]);
            let start = (a[1] - center[1]).atan2(a[0] - center[0]);
            let mut angles = vec![
                0.,
                PI / 2.,
                PI,
                3. * PI / 2.,
                (11. - center[1]).atan2(11. - center[0]),
            ];
            let x = (11. - center[0]) / rho;
            if x.abs() <= 1. {
                angles.extend([x.acos(), -x.acos()]);
            }
            let y = (11. - center[1]) / rho;
            if y.abs() <= 1. {
                angles.extend([y.asin(), PI - y.asin()]);
            }
            points.extend(
                angles
                    .into_iter()
                    .filter(|x| angle_on(*x, start, sweep_rad))
                    .map(|x| [center[0] + rho * x.cos(), center[1] + rho * x.sin()]),
            );
        }
        _ => panic!("unexpected helix"),
    }
    for p in points {
        assert!(
            l_clearance(p) >= r - 1e-7,
            "whole L sweep exits at{p:?}: {c:?}"
        );
    }
}
fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn angle_on(a: f64, start: f64, sweep: f64) -> bool {
    if sweep.abs() >= TAU - 1e-10 {
        return true;
    }
    let d = if sweep > 0. {
        (a - start).rem_euclid(TAU)
    } else {
        (start - a).rem_euclid(TAU)
    };
    d <= sweep.abs() + 1e-10 || d >= TAU - 1e-10
}
// Independent full-curve extrema from analytic line/circle stationary points.
fn curve_distance(c: &Curve, p: [f64; 2], maximum: bool) -> f64 {
    let a = c.start();
    let b = c.end();
    match *c {
        Curve::Line { .. } => {
            if maximum {
                return dist([a[0], a[1]], p).max(dist([b[0], b[1]], p));
            }
            let d = [b[0] - a[0], b[1] - a[1]];
            let l = d[0] * d[0] + d[1] * d[1];
            let t = if l > 0. {
                ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l
            } else {
                0.
            };
            let t = t.clamp(0., 1.);
            dist([a[0] + t * d[0], a[1] + t * d[1]], p)
        }
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let r = dist([a[0], a[1]], [center[0], center[1]]);
            let start = (a[1] - center[1]).atan2(a[0] - center[0]);
            let direction =
                (p[1] - center[1]).atan2(p[0] - center[0]) + if maximum { PI } else { 0. };
            let mut value = if maximum {
                dist([a[0], a[1]], p).max(dist([b[0], b[1]], p))
            } else {
                dist([a[0], a[1]], p).min(dist([b[0], b[1]], p))
            };
            if angle_on(direction, start, sweep_rad) {
                let d = dist([center[0], center[1]], p);
                value = if maximum {
                    value.max(d + r)
                } else {
                    value.min((d - r).abs())
                };
            }
            value
        }
        _ => panic!("unexpected helix"),
    }
}















/// #190: the adaptive pocket planned from the stock (`engagement_front`), on
/// the L pocket: it follows the boundary between cut and uncut stock, so
/// nothing is cut twice.
#[test]
fn engagement_front_pocket_clears_the_region_once_within_its_engagement() {
    use camlib::operation::OperationPlan;
    let cancel = AtomicBool::new(false);
    // The L: (3,3) (23,3) (23,11) (11,11) (11,23) (3,23), floor Z8, top Z10.
    let corners = [
        [3., 3.],
        [23., 3.],
        [23., 11.],
        [11., 11.],
        [11., 23.],
        [3., 23.],
    ];
    let inside = |p: [f64; 2]| {
        (p[0] > 3. && p[0] < 23. && p[1] > 3. && p[1] < 11.)
            || (p[0] > 3. && p[0] < 11. && p[1] > 3. && p[1] < 23.)
    };
    let to_wall = |p: [f64; 2]| {
        (0..6)
            .map(|k| {
                let (a, b) = (corners[k], corners[(k + 1) % 6]);
                let e = [b[0] - a[0], b[1] - a[1]];
                let t = (((p[0] - a[0]) * e[0] + (p[1] - a[1]) * e[1])
                    / (e[0] * e[0] + e[1] * e[1]))
                    .clamp(0., 1.);
                dist([a[0] + t * e[0], a[1] + t * e[1]], p)
            })
            .fold(f64::INFINITY, f64::min)
    };
    let run = || {
        let m = fixture();
        let op = operation();
        let tool = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            Operation::AdaptivePocket(op),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let mut motions = vec![];
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert!(motions.len() < 100_000);
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            motions.push(motion);
        }
        plan.finish_report(&cancel).unwrap();
        (motions, stock, serde_json::to_value(plan.report()).unwrap())
    };
    let (motions, stock, report) = run();

    // Independent of the planner: at depth, the cutter (radius 2) stays
    // inside the L and off its walls, over every curve.
    for motion in &motions {
        let z = motion.curve.start()[2].min(motion.curve.end()[2]);
        if z >= 10. {
            continue;
        }
        let path = motion.curve.prepare(1e-5).unwrap();
        for i in 0..=64 {
            let p = path.position(i as f64 / 64.).unwrap();
            let q = [p[0], p[1]];
            assert!(
                inside(q) && to_wall(q) >= 2. - 1e-6,
                "cutter in the wall at {p:?}"
            );
        }
    }
    // The stock: everything the cutter can reach is at the floor, and
    // nothing outside the pocket is touched. (A 0.4 mm cell is 0.29 mm
    // across its half diagonal; a 2 mm cutter leaves the inside corners.)
    for (i, cell) in stock.cells().iter().enumerate() {
        let p = stock.cell_center(i).unwrap();
        let corner = corners.iter().any(|c| dist(*c, p) < 2.5);
        // With the wall pass, right up to the walls: a cell whose centre is
        // more than its half diagonal inside is wholly inside.
        if inside(p) && to_wall(p) > 0.3 && !corner {
            assert!(cell.upper_mm <= 8. + 1e-4, "uncut at {p:?}: {cell:?}");
        } else if !inside(p) && to_wall(p) > 0.6 {
            assert_eq!(cell.lower_mm, 10., "cut outside the pocket at {p:?}");
        }
    }
    // Two depth levels, each entered by one helix, every cutting stroke
    // proven within acos(1 - stepover / r) = acos(1 - 1.5 / 2).
    let limit = (1f64 - 1.5 / 2.).acos().to_degrees();
    assert_eq!(report["strategy"], "adaptive_pocket");
    assert_eq!(report["clearing"], "engagement_front");
    assert_eq!(report["completed_layers"], 2);
    assert!((report["max_engagement_deg"].as_f64().unwrap() - limit).abs() < 1e-9);
    for layer in report["layers"].as_array().unwrap() {
        let c = &layer["clearing"];
        assert_eq!(c["helical_entries"], 1);
        // One contour-style pass along the L's wall, none skipped.
        assert_eq!(c["wall_passes"], 1);
        assert!(c.get("wall_passes_skipped").is_none());
        // Only the stretch into each inside corner, where the contact is
        // not proven within the limit, runs as entry motion: a small part
        // of the 65 mm wall loop, not whole walls.
        let confined = c
            .get("confined_stroke_mm")
            .map_or(0., |v| v.as_f64().unwrap());
        assert!(confined < 8., "{confined} mm of entry-feed strokes: {c}");
        assert!(c["proven_max_engagement_deg"].as_f64().unwrap() <= limit);
        assert!(c["engagement_proved_strokes"].as_u64().unwrap() > 10);
    }
    // Independent of that proof (#190): the sampled leading contact of every
    // cutting and link stroke, against the 26 x 26 mm stock.
    let worst = leading_contact_deg(&motions, 2., [[0., 0.], [26., 26.]]);
    assert!(
        worst <= limit + LEADING_SLACK_DEG,
        "{worst} > {limit} + {LEADING_SLACK_DEG} degree sampling slack"
    );
    // The radial width is the stepover: wider than the cutter radius is refused.
    let m = fixture();
    let mut op = operation();
    op.pocket.stepover_mm = 2.5;
    let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let error = OperationPlan::prepare_with_stock(
        Operation::AdaptivePocket(op),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .err()
    .unwrap();
    assert!(
        error.contains("stepover_mm") && error.contains("cutter radius"),
        "{error}"
    );
}

/// One run of an adaptive pocket on the L fixture: every motion with the
/// stock's upper-bound volume before and after it, the stock cells as they
/// were before it, and the nominal time of the feed moves.
struct FrontRun {
    motions: Vec<Motion>,
    removed_mm3: Vec<f64>,
    before: Vec<Vec<HeightBounds>>,
    feed_seconds: f64,
    centres: Vec<[f64; 2]>,
}
fn front_run(edit: impl FnOnce(&mut AdaptivePocketOperation)) -> Result<FrontRun, String> {
    use camlib::operation::OperationPlan;
    let cancel = AtomicBool::new(false);
    let m = fixture();
    let mut op = operation();
    edit(&mut op);
    // The wire format round-trips the options.
    let wire = serde_json::to_value(op).unwrap();
    let op: AdaptivePocketOperation = serde_json::from_value(wire).map_err(|e| e.to_string())?;
    let op = Operation::AdaptivePocket(op);
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        op,
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )?;
    let centres = (0..stock.cells().len())
        .map(|i| stock.cell_center(i).unwrap())
        .collect();
    let mut run = FrontRun {
        motions: vec![],
        removed_mm3: vec![],
        before: vec![],
        feed_seconds: 0.,
        centres,
    };
    let mut at = m.initial_position_mm;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel)? {
        assert!(run.motions.len() < 100_000);
        let start = motion.curve.start();
        assert!(
            (0..3).all(|k| (start[k] - at[k]).abs() < 1e-9),
            "motion {} starts at {start:?}, tool at {at:?}",
            motion.id
        );
        at = motion.curve.end();
        if let Some(feed) = motion.feed_mm_min {
            run.feed_seconds += motion.curve.prepare(1e-5).unwrap().length_mm() / feed * 60.;
        }
        let volume = stock.volume().remaining_upper_mm3;
        run.before.push(stock.cells().to_vec());
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        run.removed_mm3
            .push(volume - stock.volume().remaining_upper_mm3);
        run.motions.push(motion);
    }
    // `before[i + 1]` is the stock after motion i.
    run.before.push(stock.cells().to_vec());
    Ok(run)
}
#[test]
fn corner_limit_keeps_l_pocket_coverage_and_refuses_unproven_strokes() {
    let plain = front_run(|op| op.pocket.stepdown_mm = 2.).unwrap();
    let bounded = front_run(|op| {
        op.pocket.stepdown_mm = 2.;
        op.max_corner_engagement_deg = Some(150.);
    })
    .unwrap();
    assert_eq!(plain.before.last(), bounded.before.last());
    for m in bounded.motions.iter().filter(|m| m.kind == MoveKind::Cut) {
        assert_entire_l_sweep(&m.curve, 2.);
    }
    let Err(error) = front_run(|op| {
        op.pocket.stepdown_mm = 2.;
        op.max_corner_engagement_deg = Some(65.);
    }) else {
        panic!("an unproven corner must be refused");
    };
    assert!(error.contains("not proven within max_corner_engagement_deg 65"), "{error}");
}

#[test]
fn corner_cap_steering_width_is_reproducible() {
    let m = fixture();
    let mut op = operation();
    op.path_strategy = AdaptivePathStrategy::EngagementFront;
    op.pocket.stepdown_mm = 2.;
    op.max_corner_engagement_deg = Some(45.14);
    let plan = camlib::pocket_front::PocketFrontPlan::prepare(
        op, &m.tools[0], m.stock.height().unwrap(), m.initial_position_mm, 1, &|| false,
    ).unwrap();
    // Radius 2 mm times (1 - cos(45.14 degrees)), with binary64 intermediates.
    assert_eq!(plan.report().max_radial_engagement_mm, 0.5892462315868332);
}

#[test]
fn corner_cap_refusal_never_completes_on_retry() {
    use camlib::operation::{OperationPlan, OperationReport};
    use camlib::pocket::ResidualStatus;

    let cancel = AtomicBool::new(false);
    for angle in [100., 65.] {
        let m = fixture();
        let mut op = operation();
        op.path_strategy = AdaptivePathStrategy::EngagementFront;
        op.pocket.stepdown_mm = 2.;
        op.max_corner_engagement_deg = Some(angle);
        let tool = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            Operation::AdaptivePocket(op),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let refusal = loop {
            match plan.next_motion_with_stock(&stock, &cancel) {
                Ok(Some(motion)) => {
                    let delta = stock.preview_sweep(&tool, &motion, 1., m.sweep, &cancel).unwrap();
                    stock.apply_delta(&delta).unwrap();
                }
                Ok(None) => panic!("{angle} degrees must be refused before completion"),
                Err(error) => break error,
            }
        };
        assert!(refusal.contains("not proven within max_corner_engagement_deg"), "{refusal}");
        let generation = stock.generation();
        for _ in 0..4 {
            assert_eq!(plan.next_motion_with_stock(&stock, &cancel).unwrap_err(), refusal);
            assert_eq!(plan.finish_report(&cancel).unwrap_err(), refusal);
            let OperationReport::AdaptiveFront(report) = plan.report() else { panic!("adaptive report") };
            assert_eq!(report.completed_layers, 0);
            assert_eq!(report.geometric_residual_status, ResidualStatus::Pending);
        }
        assert_eq!(stock.generation(), generation);
    }
}

/// Straight links end to end on one line are one move: a link with a link
/// feed is emitted in stretches, one per feed.
fn joined(motions: &[Motion]) -> Vec<(MoveKind, Curve)> {
    let mut out: Vec<(MoveKind, Curve)> = vec![];
    for m in motions {
        if m.kind == MoveKind::Link
            && let Curve::Line { start: b, end: c } = m.curve
            && let Some((MoveKind::Link, Curve::Line { start: a, end })) = out.last_mut()
            && *end == b
        {
            let cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
            let dot = (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]);
            if cross.abs() <= 1e-9 * (1. + dist([a[0], a[1]], [c[0], c[1]])) && dot > 0. {
                *end = c;
                continue;
            }
        }
        out.push((m.kind, m.curve.clone()));
    }
    out
}

/// #211: AdaptivePocket takes the link feed Roughing's adaptive clearing
/// has. A stay-down link runs at it only where it meets no stock, checked
/// here against the stock itself (the heights every earlier move left), not
/// the planner's report; the path is the one planned without it.
#[test]
fn engagement_front_pocket_links_feed_fast_only_clear_of_stock() {
    let plain = front_run(|_| ()).unwrap();
    let fast = front_run(|op| op.link_feed_mm_min = Some(3000.)).unwrap();
    let (r, cutting) = (2., 600.);
    // #190: link and cutting strokes keep their sampled leading contact
    // within acos(1 - 1.5 / 2), with and without the link feed.
    let limit = (1f64 - 1.5 / r).acos().to_degrees();
    for run in [&plain, &fast] {
        let worst = leading_contact_deg(&run.motions, r, [[0., 0.], [26., 26.]]);
        assert!(worst <= limit + LEADING_SLACK_DEG, "{worst} > {limit}");
    }
    let (mut fast_mm, mut slow_mm) = (0f64, 0f64);
    for (i, motion) in fast.motions.iter().enumerate() {
        if motion.kind != MoveKind::Link {
            // Only links take the link feed.
            assert_ne!(motion.feed_mm_min, Some(3000.), "{motion:?}");
            continue;
        }
        let Curve::Line { start, end } = motion.curve else {
            panic!("a link is a straight move: {motion:?}")
        };
        let length = dist([start[0], start[1]], [end[0], end[1]]);
        if motion.feed_mm_min != Some(3000.) {
            // Not proven clear: the cutting feed.
            assert_eq!(motion.feed_mm_min, Some(cutting), "{motion:?}");
            slow_mm += length;
            continue;
        }
        fast_mm += length;
        // The stock loses nothing to a fast link: no cell changes.
        assert_eq!(fast.removed_mm3[i], 0., "fast link {motion:?} cut stock");
        assert!(
            fast.before[i] == fast.before[i + 1],
            "fast link {motion:?} changed the stock"
        );
        // Nothing stood in its way. Exactly: every point under the cutter
        // anywhere along the link was swept, at this depth or lower, by an
        // earlier feed move (its distance to that move's centre line, in
        // closed form, is within the cutter radius).
        let z = start[2];
        let swept = |p: [f64; 2]| {
            fast.motions[..i].iter().any(|old| {
                let (a, b) = (old.curve.start(), old.curve.end());
                let flat = a[2] == b[2] && !matches!(old.curve, Curve::Helix { .. });
                let plunge = matches!(old.curve, Curve::Line { .. }) && a[0] == b[0] && a[1] == b[1];
                !old.kind.is_rapid()
                    && (flat || plunge)
                    && a[2].min(b[2]) <= z + 1e-9
                    && curve_distance(&old.curve, p, false) <= r + 1e-6
            })
        };
        let n = (length / 0.1).ceil() as usize;
        for k in 0..=n {
            let t = k as f64 / n as f64;
            let c = [
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
            ];
            for radial in [0., 0.25, 0.5, 0.75, 0.98] {
                for a in 0..24 {
                    let a = TAU * a as f64 / 24.;
                    let p = [c[0] + radial * r * a.cos(), c[1] + radial * r * a.sin()];
                    assert!(swept(p), "fast link {motion:?} over uncut stock at {p:?}");
                }
            }
        }
        // And in the stock itself: every cell wholly under the cutter (its
        // centre more than the 0.4 mm cell's half diagonal inside the rim)
        // is down at the link's depth.
        for (cell, c) in fast.before[i].iter().zip(&fast.centres) {
            if curve_distance(&motion.curve, *c, false) <= r - 0.29 {
                assert!(
                    cell.upper_mm <= z + 1e-3,
                    "fast link {motion:?} over stock {cell:?} at {c:?}"
                );
            }
        }
    }
    assert!(fast_mm > 10., "{fast_mm} mm of fast links");
    // Some link stretches do run beside stock; they stay at the cutting feed.
    assert!(slow_mm > 0., "no link stretch kept the cutting feed");
    // The same path: only link feeds (and where a link is split) differ.
    let (a, b) = (joined(&plain.motions), joined(&fast.motions));
    assert_eq!(a.len(), b.len());
    for (a, b) in a.iter().zip(&b) {
        assert_eq!(a, b);
    }
    let others = |run: &FrontRun| -> Vec<(MoveKind, Curve, Option<f64>)> {
        run.motions
            .iter()
            .filter(|m| m.kind != MoveKind::Link)
            .map(|m| (m.kind, m.curve.clone(), m.feed_mm_min))
            .collect()
    };
    assert!(others(&plain) == others(&fast));
    assert!(plain.motions.iter().all(|m| m.feed_mm_min != Some(3000.)));
    // The fast stretches take a fifth of the time (600 against 3000 mm/min).
    let saved = fast_mm / cutting * 60. * (1. - cutting / 3000.);
    assert!(
        (plain.feed_seconds - fast.feed_seconds - saved).abs() < 1e-6 && saved > 1.,
        "{} s without, {} s with, {fast_mm} mm fast",
        plain.feed_seconds,
        fast.feed_seconds
    );
    eprintln!(
        "links: {fast_mm:.1} mm at the link feed, {slow_mm:.1} mm at the cutting feed; feed time {:.1} -> {:.1} s",
        plain.feed_seconds, fast.feed_seconds
    );

    // Chip thinning: stretches beside stock run between the two feeds.
    let thinned = front_run(|op| {
        op.link_feed_mm_min = Some(3000.);
        op.link_chip_thinning = true;
    })
    .unwrap();
    assert_eq!(joined(&thinned.motions), joined(&fast.motions));
    for (i, motion) in thinned.motions.iter().enumerate() {
        let feed = motion.feed_mm_min;
        if motion.kind == MoveKind::Link {
            assert!(feed.is_some_and(|f| (cutting..=3000.).contains(&f)));
            if feed == Some(3000.) {
                assert_eq!(thinned.removed_mm3[i], 0.);
            }
        }
    }
    assert!(thinned.feed_seconds <= fast.feed_seconds);
}

/// #211: the entry options. With a rapid-descent clearance the second level
/// is entered by a rapid through the level above it; no rapid touches stock
/// (the stock's own heights under the whole cutter), and the cutting moves
/// are the ones planned without the options.
#[test]
fn engagement_front_pocket_rapid_descent_stays_clear_of_stock() {
    let plain = front_run(|_| ()).unwrap();
    let quick = front_run(|op| {
        op.rapid_descent_clearance_mm = Some(0.25);
        op.ramp_clearance_mm = Some(0.5);
    })
    .unwrap();
    let r = 2.;
    let lowest = |run: &FrontRun| {
        run.motions
            .iter()
            .filter(|m| m.kind == MoveKind::Rapid)
            .map(|m| m.curve.end()[2])
            .fold(f64::INFINITY, f64::min)
    };
    // Without the option every rapid stops at the feed height above the
    // stock top (Z10); with it, rapids reach into the first level (floor Z9).
    assert!(lowest(&plain) >= 10., "{}", lowest(&plain));
    assert!(lowest(&quick) < 9.5, "{}", lowest(&quick));
    for (i, motion) in quick.motions.iter().enumerate() {
        if motion.kind != MoveKind::Rapid {
            continue;
        }
        assert_eq!(quick.removed_mm3[i], 0., "rapid {motion:?} cut stock");
        let (a, b) = (motion.curve.start(), motion.curve.end());
        let low = a[2].min(b[2]);
        let vertical = a[0] == b[0] && a[1] == b[1];
        for (cell, c) in quick.before[i].iter().zip(&quick.centres) {
            // Whole cells (0.4 mm): on a descent, every cell whose square
            // the cutter's disk overlaps; on a traverse, any cell within
            // half a cell diagonal of the swept disk.
            let over = if vertical {
                ((c[0] - a[0]).abs() - 0.2)
                    .max(0.)
                    .hypot(((c[1] - a[1]).abs() - 0.2).max(0.))
                    <= r
            } else {
                curve_distance(&motion.curve, *c, false) <= r + 0.29
            };
            if over {
                assert!(
                    cell.upper_mm <= low,
                    "rapid {motion:?} reaches stock {cell:?} at {c:?}"
                );
            }
        }
    }
    // The plunges that remain are shorter in total.
    let plunged = |run: &FrontRun| -> f64 {
        run.motions
            .iter()
            .filter(|m| m.kind == MoveKind::Plunge)
            .map(|m| m.curve.start()[2] - m.curve.end()[2])
            .sum()
    };
    assert!(
        plunged(&quick) < plunged(&plain),
        "{} against {}",
        plunged(&quick),
        plunged(&plain)
    );
    // The cutting passes are the same moves. An entry whose descent is moved
    // back from the material it cuts adds a short link at depth (less than
    // the cutter radius); every other link is unchanged.
    let of = |run: &FrontRun, kind: MoveKind| -> Vec<Curve> {
        run.motions
            .iter()
            .filter(|m| m.kind == kind)
            .map(|m| m.curve.clone())
            .collect()
    };
    assert_eq!(of(&plain, MoveKind::Cut), of(&quick, MoveKind::Cut));
    let (before, after) = (of(&plain, MoveKind::Link), of(&quick, MoveKind::Link));
    assert!(before.iter().all(|c| after.contains(c)));
    for c in after.iter().filter(|c| !before.contains(c)) {
        let (a, b) = (c.start(), c.end());
        assert!(dist([a[0], a[1]], [b[0], b[1]]) <= r, "added link {c:?}");
    }
}

/// #211: the options are validated by Roughing's rules.
#[test]
fn adaptive_pocket_front_options_are_validated() {
    type Edit = fn(&mut AdaptivePocketOperation);
    let invalid: [(&str, Edit); 5] = [
        ("link_feed_mm_min", |op| op.link_feed_mm_min = Some(0.)),
        ("link_feed_mm_min", |op| {
            op.link_feed_mm_min = Some(2e6)
        }),
        ("rapid_descent_clearance_mm", |op| {
            op.rapid_descent_clearance_mm = Some(-1.)
        }),
        ("ramp_clearance_mm", |op| op.ramp_clearance_mm = Some(0.)),
        ("link_chip_thinning needs link_feed_mm_min", |op| {
            op.link_chip_thinning = true
        }),
    ];
    for (name, edit) in invalid {
        let error = front_run(edit).err().unwrap();
        assert!(error.contains(name), "{name}: {error}");
    }
    // Unset, the wire format is what it was: no new keys.
    let wire = serde_json::to_value(operation()).unwrap();
    for name in [
        "link_feed_mm_min",
        "link_chip_thinning",
        "rapid_descent_clearance_mm",
        "ramp_clearance_mm",
    ] {
        assert!(wire.get(name).is_none(), "{name} written when unset");
    }
}

/// #311: what a level reports as its residual is what its committed strokes
/// left, the wall pass included. A square pocket's leftover is the four
/// inside corners a round cutter cannot reach; measuring the emitted paths
/// independently and comparing is what catches a report that still counts
/// material the wall pass removed (it did: 12.74 mm2 reported against 3.36 mm2
/// of corners here).
#[test]
fn engagement_front_reports_the_material_its_wall_pass_left() {
    use camlib::operation::OperationPlan;
    let m = fixture();
    let mut op = operation();
    op.pocket.stepdown_mm = 2.;
    op.pocket.region = PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "square pocket".into(),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component {
            outer: Boundary {
                vertices: vec![[3., 3., 0.], [23., 3., 0.], [23., 23., 0.], [3., 23., 0.]],
            },
            holes: vec![],
        }],
    };
    let r = m.tools[0].cutter.diameter_mm / 2.;
    let level = op.pocket.floor_z_mm;
    let op_stepover = op.pocket.stepover_mm;
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        Operation::AdaptivePocket(op),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let (mut flat, mut motions) = (vec![], vec![]);
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        if matches!(motion.kind, MoveKind::Cut | MoveKind::Lead | MoveKind::Link)
            && motion.curve.start()[2] == motion.curve.end()[2]
            && motion.curve.start()[2] == level
        {
            flat.push(motion.curve.clone());
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        motions.push(motion);
    }
    plan.finish_report(&|| false).unwrap();
    // #190: the sampled leading contact of every cutting and link stroke,
    // wall pass included, within acos(1 - stepover / r).
    let limit = (1f64 - op_stepover / r).acos().to_degrees();
    let worst = leading_contact_deg(&motions, r, [[0., 0.], [26., 26.]]);
    assert!(worst <= limit + LEADING_SLACK_DEG, "{worst} > {limit}");
    let report = serde_json::to_value(plan.report()).unwrap();
    let layers = report["layers"].as_array().unwrap();
    assert_eq!(layers.len(), 1, "{report}");
    let clearing = &layers[0]["clearing"];
    assert_eq!(clearing["wall_passes"].as_u64().unwrap(), 1, "{clearing}");
    // Independent of the planner: a point of the pocket at 0.1 mm is material
    // when no emitted path passed a cutter's radius from it. (A sampling of
    // the emitted paths to cross-check the reported estimate, not a stock
    // proof.)
    let left = (0..200)
        .flat_map(|j| {
            (0..200).map(move |i| {
                [3. + (i as f64 + 0.5) * 0.1, 3. + (j as f64 + 0.5) * 0.1]
            })
        })
        .filter(|p| {
            flat.iter()
                .all(|c| curve_distance(c, *p, false) >= r - 1e-4)
        })
        .count() as f64
        * 0.01;
    let reported = clearing["geometric_residual_area_mm2"]
        .as_f64()
        .unwrap();
    assert_eq!(clearing["geometric_residual_status"], "estimated");
    assert!(
        (reported - left).abs() < 0.8,
        "reported {reported} mm2 against {left} mm2 sampled from the paths: {clearing}"
    );
    // What a 4 mm cutter leaves in a square pocket is its corners: r² (1 -
    // π/4) of them each.
    let corners = 4. * r * r * (1. - PI / 4.);
    assert!(
        (left - corners).abs() < 1.2,
        "{left} mm2 sampled against {corners} mm2 of corners"
    );
    // Reachable, unresolved and unreachable remain a partition of the whole.
    let sum = ["residual_reachable_area_mm2", "residual_unresolved_area_mm2", "residual_unreachable_area_mm2"]
        .iter()
        .map(|name| clearing[name].as_f64().unwrap())
        .fold(0., |a, b| a + b);
    assert!(
        (sum - reported).abs() < 1e-9,
        "{sum} against {reported}: {clearing}"
    );
}

/// #311 on a curved wall: a circular pocket has no corners, so what the
/// level leaves after its wall pass is numerical dust. The report must say
/// so — measured against an independent sampling of the emitted paths, like
/// the square pocket above — instead of counting the skin the wall pass
/// removed (it did: the reported residual was the whole ring the pass cut).
#[test]
fn engagement_front_reports_the_material_a_curved_wall_pass_left() {
    use camlib::operation::OperationPlan;
    let m = fixture();
    let mut op = operation();
    op.pocket.stepdown_mm = 2.;
    let (segments, radius) = (12, 8.);
    let centre = [13., 13.];
    let vertices: Vec<[f64; 3]> = (0..segments)
        .map(|i| {
            let a = i as f64 * TAU / segments as f64;
            [
                centre[0] + radius * a.cos(),
                centre[1] + radius * a.sin(),
                (TAU / segments as f64 / 4.).tan(),
            ]
        })
        .collect();
    op.pocket.region = PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "circular pocket".into(),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component {
            outer: Boundary { vertices },
            holes: vec![],
        }],
    };
    let r = m.tools[0].cutter.diameter_mm / 2.;
    let level = op.pocket.floor_z_mm;
    let op_stepover = op.pocket.stepover_mm;
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        Operation::AdaptivePocket(op),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let (mut flat, mut motions) = (vec![], vec![]);
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        if matches!(motion.kind, MoveKind::Cut | MoveKind::Lead | MoveKind::Link)
            && motion.curve.start()[2] == motion.curve.end()[2]
            && motion.curve.start()[2] == level
        {
            flat.push(motion.curve.clone());
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        motions.push(motion);
    }
    plan.finish_report(&|| false).unwrap();
    // #190: the sampled leading contact, as for the square pocket.
    let limit = (1f64 - op_stepover / r).acos().to_degrees();
    let worst = leading_contact_deg(&motions, r, [[0., 0.], [26., 26.]]);
    assert!(worst <= limit + LEADING_SLACK_DEG, "{worst} > {limit}");
    let report = serde_json::to_value(plan.report()).unwrap();
    let clearing = &report["layers"][0]["clearing"];
    assert_eq!(clearing["wall_passes"].as_u64().unwrap(), 1, "{clearing}");
    // Independent of the planner: a point of the pocket at 0.1 mm is material
    // when no emitted path passed a cutter's radius from it.
    let left = (0..260)
        .flat_map(|j| {
            (0..260).map(move |i| {
                [5. + (i as f64 + 0.5) * 0.1, 5. + (j as f64 + 0.5) * 0.1]
            })
        })
        .filter(|p| {
            (p[0] - centre[0]).hypot(p[1] - centre[1]) <= radius
                && flat
                    .iter()
                    .all(|c| curve_distance(c, *p, false) >= r - 1e-4)
        })
        .count() as f64
        * 0.01;
    let reported = clearing["geometric_residual_area_mm2"]
        .as_f64()
        .unwrap();
    assert_eq!(clearing["geometric_residual_status"], "estimated");
    assert!(
        (reported - left).abs() < 0.8,
        "reported {reported} mm2 against {left} mm2 sampled from the paths: {clearing}"
    );
    // A circle leaves no corners: what stands is the sampling's own dust.
    assert!(left < 0.5, "{left} mm2 sampled from the paths: {clearing}");
}
