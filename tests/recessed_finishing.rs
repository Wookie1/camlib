use camlib::{
    facing::TraverseAxis,
    motion::{Curve, MoveKind},
    operation::OperationPlan,
    pocket::Operation,
    stock::Stock,
    stream::StreamManifest,
};
use std::sync::atomic::AtomicBool;

// Independent exact segment/AABB distance: between coordinate-bound crossings
// distance squared is quadratic; check its endpoints and stationary point.
fn box_distance(a: [f64; 3], b: [f64; 3], low: [f64; 3], high: [f64; 3]) -> f64 {
    let d: [f64; 3] = std::array::from_fn(|k| b[k] - a[k]);
    let mut times = vec![0., 1.];
    for k in 0..3 {
        if d[k] != 0. {
            for edge in [low[k], high[k]] {
                let t = (edge - a[k]) / d[k];
                if t > 0. && t < 1. {
                    times.push(t);
                }
            }
        }
    }
    times.sort_by(f64::total_cmp);
    times.dedup();
    let mut minimum = f64::INFINITY;
    for pair in times.windows(2) {
        let mid = (pair[0] + pair[1]) / 2.;
        let (mut aa, mut ab) = (0., 0.);
        for k in 0..3 {
            let x = a[k] + mid * d[k];
            let (s, c) = if x < low[k] {
                (-d[k], low[k] - a[k])
            } else if x > high[k] {
                (d[k], a[k] - high[k])
            } else {
                (0., 0.)
            };
            aa += s * s;
            ab += s * c;
        }
        let t = if aa > 0. {
            (-ab / aa).clamp(pair[0], pair[1])
        } else {
            pair[0]
        };
        for t in [pair[0], pair[1], t] {
            let distance = (0..3)
                .map(|k| {
                    (low[k] - a[k] - t * d[k])
                        .max(a[k] + t * d[k] - high[k])
                        .max(0.)
                        .powi(2)
                })
                .sum::<f64>();
            minimum = minimum.min(distance);
        }
    }
    minimum.sqrt()
}
fn target_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    // Exact CAD solid: base plus the four walls around [4,14]x[4,12].
    [
        ([0., 0., 0.], [24., 16., 3.]),
        ([0., 0., 3.], [4., 16., 5.]),
        ([14., 0., 3.], [24., 16., 5.]),
        ([4., 0., 3.], [14., 4., 5.]),
        ([4., 12., 3.], [14., 16., 5.]),
    ]
    .into_iter()
    .map(|(lo, hi)| box_distance(a, b, lo, hi))
    .fold(f64::INFINITY, f64::min)
}
fn swept_height(curve: &Curve, xy: [f64; 2]) -> f64 {
    assert!(matches!(curve, Curve::Line { .. }));
    let a = curve.start();
    let b = curve.end();
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0. {
        let projection = ((xy[0] - a[0]) * dx + (xy[1] - a[1]) * dy) / l2;
        let perpendicular = ((xy[0] - a[0]) * dy - (xy[1] - a[1]) * dx).powi(2) / l2;
        if perpendicular > 4. {
            return f64::INFINITY;
        }
        (projection - dz * ((4. - perpendicular) / (l2 * (l2 + dz * dz))).sqrt()).clamp(0., 1.)
    } else if dz < 0. {
        1.
    } else {
        0.
    };
    let distance = (xy[0] - a[0] - t * dx).powi(2) + (xy[1] - a[1] - t * dy).powi(2);
    if distance > 4. {
        f64::INFINITY
    } else {
        a[2] + t * dz + 2. - (4. - distance).sqrt()
    }
}

#[test]
fn actual_recessed_cad_paths_preserve_entire_solid_sweeps_and_all_stock_cells() {
    let cancel = AtomicBool::new(false);
    for (kind, axis, reverse) in [
        ("parallel", TraverseAxis::X, false),
        ("parallel", TraverseAxis::X, true),
        ("parallel", TraverseAxis::Y, false),
        ("waterline", TraverseAxis::X, false),
    ] {
        let text = if kind == "parallel" {
            include_str!("../fixtures/operations/recessed-parallel.json")
        } else {
            include_str!("../fixtures/operations/recessed-waterline.json")
        };
        let m: StreamManifest = serde_json::from_str(text).unwrap();
        m.validate().unwrap();
        let mut op = m.operations[1].clone();
        if let Operation::Waterline(p) = &op {
            assert!(p.bottom_z_mm > 3. && p.bottom_z_mm < 3.02);
        }
        if let Operation::Parallel(p) = &mut op {
            p.axis = axis;
            p.reverse = reverse;
        }
        let tool = &m.tools[1];
        let prepared = tool.prepare().unwrap();
        let mut plan = OperationPlan::prepare(
            op,
            tool,
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut position = m.initial_position_mm;
        let mut curves = vec![];
        let mut cuts = 0;
        let mut inner_bottom_cuts = 0;
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(motion.curve.start(), position);
            position = motion.curve.end();
            let mut a = motion.curve.start();
            let mut b = motion.curve.end();
            a[2] += 2.;
            b[2] += 2.;
            let distance = target_distance(a, b);
            assert!(
                distance >= 2. - 1e-9,
                "whole CAD solid gouge: {kind} {axis:?} {distance} {:?}",
                motion.curve
            );
            cuts += usize::from(motion.kind == MoveKind::Cut);
            if kind == "waterline"
                && motion.kind == MoveKind::Cut
                && motion.curve.start()[2] < 3.02
                && [motion.curve.start(), motion.curve.end()]
                    .iter()
                    .all(|p| p[0] > 4. && p[0] < 14. && p[1] > 4. && p[1] < 12.)
            {
                inner_bottom_cuts += 1;
            }
            curves.push(motion.curve.clone());
            let delta = stock
                .preview_sweep(&prepared, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
        }
        plan.finish_report(&cancel).unwrap();
        assert!(cuts > 10);
        if kind == "waterline" {
            assert!(
                inner_bottom_cuts >= 4,
                "bottom pocket contour disappeared: {inner_bottom_cuts}"
            );
        }
        assert!(stock.volume().removed_lower_mm3 > 0.);
        let mut floor_cells = 0;
        for (i, interval) in stock.cells().iter().enumerate() {
            let xy = stock.cell_center(i).unwrap();
            let actual = curves
                .iter()
                .map(|c| swept_height(c, xy))
                .fold(6f64, f64::min);
            assert!(
                interval.lower_mm <= actual + 1e-8 && interval.upper_mm >= actual - 1e-8,
                "{kind} {xy:?}: {interval:?}, swept {actual}"
            );
            let in_pocket = xy[0] > 4. && xy[0] < 14. && xy[1] > 4. && xy[1] < 12.;
            let target = if in_pocket { 3. } else { 5. };
            assert!(
                actual >= target - 1e-8,
                "material gouge {kind} {xy:?}: {actual}"
            );
            if kind == "waterline" && xy == [6.25, 8.25] {
                assert!(actual < 3.1, "bottom wall strip was not cut: {actual}");
            }
            if kind == "parallel" && xy[0] >= 6.5 && xy[0] <= 11.5 && xy[1] >= 6.5 && xy[1] <= 9.5 {
                assert!(
                    actual <= 3.2,
                    "accessible floor remains unfinished {axis:?} {xy:?}: {actual}"
                );
                floor_cells += 1;
            }
        }
        if kind == "parallel" {
            assert!(floor_cells > 30);
        }
    }
}

#[test]
fn vertical_offset_transitions_preserve_clearance_stock_and_cancelled_prefixes() {
    use std::cell::Cell;
    let mut manifest: StreamManifest = serde_json::from_str(include_str!(
        "../fixtures/operations/recessed-parallel.json"
    ))
    .unwrap();
    manifest.tools[1].cutter.diameter_mm = 2.;
    let tool = &manifest.tools[1];
    let prepared = tool.prepare().unwrap();
    let mut stock = Stock::new(manifest.stock.height().unwrap().clone()).unwrap();
    let mut reference_stock = Stock::new(manifest.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare(
        manifest.operations[1].clone(),
        tool,
        manifest.stock.height().unwrap(),
        manifest.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut reference = OperationPlan::prepare(
        manifest.operations[1].clone(),
        tool,
        manifest.stock.height().unwrap(),
        manifest.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut cancelled = 0;
    let mut moves = 0;
    loop {
        assert!(
            plan.next_motion_with_stock(&stock, &|| true)
                .unwrap_err()
                .contains("cancel")
        );
        let polls = Cell::new(0);
        let attempted = plan.next_motion_with_stock(&stock, &|| {
            polls.set(polls.get() + 1);
            polls.get() == 3
        });
        let motion = match attempted {
            Ok(m) => m,
            Err(e) => {
                assert!(e.contains("cancel"), "{e}");
                cancelled += 1;
                plan.next_motion_with_stock(&stock, &|| false).unwrap()
            }
        };
        let expected = reference
            .next_motion_with_stock(&reference_stock, &|| false)
            .unwrap();
        assert_eq!(
            serde_json::to_value(&motion).unwrap(),
            serde_json::to_value(&expected).unwrap()
        );
        let Some(motion) = motion else {
            break;
        };
        moves += 1;
        let mut a = motion.curve.start();
        let mut b = motion.curve.end();
        a[2] += 1.;
        b[2] += 1.;
        assert!(
            target_distance(a, b) >= 1. - 1e-9,
            "transition gouges original solid: {:?}",
            motion
        );
        let delta = stock
            .preview_sweep(&prepared, &motion, 1., manifest.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        let delta = reference_stock
            .preview_sweep(&prepared, &motion, 1., manifest.sweep, &|| false)
            .unwrap();
        reference_stock.apply_delta(&delta).unwrap();
    }
    plan.finish_report(&|| false).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert!(report["vertical_transitions"].as_u64().unwrap() > 0);
    assert_eq!(
        report["transition_moves"].as_u64().unwrap(),
        2 * report["vertical_transitions"].as_u64().unwrap()
    );
    let Operation::Parallel(operation) = &manifest.operations[1] else {
        unreachable!()
    };
    assert!(report["max_certified_path_excess_mm"].as_f64().unwrap() <= operation.path_error_mm);
    assert!(moves > 100 && cancelled > 10);
    assert_eq!(stock.cells(), reference_stock.cells());
}
