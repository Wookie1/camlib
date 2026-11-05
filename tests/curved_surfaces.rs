use camlib::{motion::Curve, operation::OperationPlan, stock::Stock, stream::StreamManifest};
use std::sync::atomic::AtomicBool;

fn fixture() -> StreamManifest {
    let m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/hemisphere.json")).unwrap();
    m.validate().unwrap();
    m
}
// Closed-form minimum distance from the WHOLE cutter-center line to the actual
// smooth sphere. Independent of triangles, planner witnesses and endpoint tests.
fn assert_sphere_clear(curve: &Curve) {
    assert!(matches!(curve, Curve::Line { .. }));
    let a = curve.start();
    let b = curve.end();
    let c = [a[0] - 8., a[1] - 8., a[2]]; // tip + ball R2 minus sphere center z2
    let d: [f64; 3] = std::array::from_fn(|k| b[k] - a[k]);
    let l2: f64 = d.iter().map(|x| x * x).sum();
    let t = if l2 > 0. {
        (-c.iter().zip(d).map(|(x, y)| x * y).sum::<f64>() / l2).clamp(0., 1.)
    } else {
        0.
    };
    let distance = (0..3)
        .map(|k| (c[k] + t * d[k]).powi(2))
        .sum::<f64>()
        .sqrt();
    assert!(
        distance >= 6. - 1e-8,
        "smooth sphere gouged: {distance}, {curve:?}"
    );
    assert!(a[2].min(b[2]) >= 2. - 1e-8, "base gouged: {curve:?}");
}
// Exact lower envelope of a radius-two ball translated along a 3D line.
fn swept_height(curve: &Curve, xy: [f64; 2]) -> f64 {
    let a = curve.start();
    let b = curve.end();
    let d: [f64; 3] = std::array::from_fn(|k| b[k] - a[k]);
    let l2 = d[0] * d[0] + d[1] * d[1];
    let t = if l2 > 0. {
        let projection = ((xy[0] - a[0]) * d[0] + (xy[1] - a[1]) * d[1]) / l2;
        let perpendicular = ((xy[0] - a[0]) * d[1] - (xy[1] - a[1]) * d[0]).powi(2) / l2;
        if perpendicular > 4. {
            return f64::INFINITY;
        }
        (projection - d[2] * ((4. - perpendicular) / (l2 * (l2 + d[2] * d[2]))).sqrt())
            .clamp(0., 1.)
    } else if d[2] < 0. {
        1.
    } else {
        0.
    };
    let r2 = (xy[0] - a[0] - t * d[0]).powi(2) + (xy[1] - a[1] - t * d[1]).powi(2);
    if r2 > 4. {
        f64::INFINITY
    } else {
        a[2] + t * d[2] + 2. - (4. - r2).sqrt()
    }
}
#[test]
fn curved_cad_parallel_preserves_smooth_target_and_bounds_actual_stock() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    {
        let op = m.operations[0].clone();
        let mut plan = OperationPlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let prepared = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut curves = vec![];
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_sphere_clear(&motion.curve);
            let delta = stock
                .preview_sweep(&prepared, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            curves.push(motion.curve);
        }
        plan.finish_report(&cancel).unwrap();
        assert!(!curves.is_empty());
        for (i, h) in stock.cells().iter().enumerate() {
            let xy = stock.cell_center(i).unwrap();
            let rho2 = (xy[0] - 8.).powi(2) + (xy[1] - 8.).powi(2);
            let target = 2. + (16. - rho2).max(0.).sqrt();
            let actual = curves
                .iter()
                .map(|c| swept_height(c, xy))
                .fold(8f64, f64::min);
            assert!(
                actual >= target - 1e-7,
                "parallel: target gouge at {xy:?}: {actual} < {target}"
            );
            assert!(
                h.lower_mm <= actual + 1e-7 && h.upper_mm >= actual - 1e-7,
                "parallel: exact swept stock {actual} outside interval {h:?} at {xy:?}"
            );
            // The sharp sphere/base foot is inaccessible to a finite ball.
            // Only the accessible cap is checked for finishing height here.
            if rho2 <= 9. {
                assert!(
                    actual - target <= 0.1,
                    "cap residual {} at {xy:?}",
                    actual - target
                );
            }
        }
        assert!(stock.volume().removed_lower_mm3 > 100.);
        assert!(stock.volume().removed_lower_mm3 <= 2048. - 646.0412865531645);
    }
}

#[test]
fn curved_cad_waterline_preserves_smooth_target_and_bounds_actual_stock() {
    let mut m = fixture();
    let source = serde_json::to_value(&m.operations[0]).unwrap();
    let template: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/operations/waterline.json")).unwrap();
    let mut op = template["operations"][0].clone();
    op["surface"] = source["surface"].clone();
    op["floor_z_mm"] = 2.into();
    op["top_z_mm"] = 6.into();
    op["clearance_z_mm"] = 10.into();
    op["contour_method"] = "support".into();
    m.operations = vec![serde_json::from_value(op).unwrap()];
    m.validate().unwrap();
    let cancel = AtomicBool::new(false);
    let mut plan = OperationPlan::prepare(
        m.operations[0].clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let prepared = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut curves = vec![];
    let mut cuts = 0;
    let error = source["surface"]["surface_error_mm"].as_f64().unwrap();
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        assert_sphere_clear(&motion.curve);
        if motion.kind == camlib::motion::MoveKind::Cut {
            cuts += 1;
            // The norm of an affine centre segment is convex: its maximum
            // lies at an endpoint. This bounds the ENTIRE cutting segment's
            // normal excess against the actual sphere, independently of the
            // faceted support polygons and their stored certificates.
            for p in [motion.curve.start(), motion.curve.end()] {
                let radial = ((p[0] - 8.).powi(2) + (p[1] - 8.).powi(2) + p[2].powi(2)).sqrt();
                assert!(radial <= 6. + 2. * error + 0.005 + 0.01 + 0.0002);
            }
        }
        let delta = stock
            .preview_sweep(&prepared, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        curves.push(motion.curve);
    }
    plan.finish_report(&cancel).unwrap();
    assert!(cuts > 8);
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(report["levels_completed"], 3);
    assert_eq!(report["contour_method"], "support");
    assert!(report["certificate_tests"].as_u64().unwrap() < 20_000_000);
    assert!(
        report["level_reports"]
            .as_array()
            .unwrap()
            .iter()
            .all(|level| { level["boolean_operations"] == 0 && level["boolean_error_mm"] == 0. })
    );
    for (i, h) in stock.cells().iter().enumerate() {
        let xy = stock.cell_center(i).unwrap();
        let rho2 = (xy[0] - 8.).powi(2) + (xy[1] - 8.).powi(2);
        let target = 2. + (16. - rho2).max(0.).sqrt();
        let actual = curves
            .iter()
            .map(|c| swept_height(c, xy))
            .fold(8f64, f64::min);
        assert!(actual >= target - 1e-7, "waterline gouge at {xy:?}");
        assert!(
            h.lower_mm <= actual + 1e-7 && h.upper_mm >= actual - 1e-7,
            "waterline: exact stock {actual} outside {h:?} at {xy:?}"
        );
    }
    assert!(stock.volume().removed_lower_mm3 > 10.);
    assert!(stock.volume().removed_lower_mm3 <= 2048. - 646.0412865531645);
}
