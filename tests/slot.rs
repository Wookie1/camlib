use camlib::{motion::*, pocket::Operation, slot::*, stock::*, stream::StreamManifest};
use std::{cell::Cell, f64::consts::TAU, sync::atomic::AtomicBool};

fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/trochoidal-slot.json")).unwrap()
}
fn operation() -> TrochoidalSlotOperation {
    match fixture().operations.remove(0) {
        Operation::TrochoidalSlot(op) => op,
        _ => panic!("wrong fixture"),
    }
}
fn plan(op: TrochoidalSlotOperation) -> SlotPlan {
    let m = fixture();
    SlotPlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .unwrap()
}
fn commit(stock: &mut Stock, m: &Motion) -> f64 {
    let t = fixture().tools[0].prepare().unwrap();
    let d = stock
        .preview_sweep(&t, m, 1., SweepOptions::default(), &AtomicBool::new(false))
        .unwrap();
    stock.apply_delta(&d).unwrap();
    d.stats().max_path_error_mm + d.stats().numerical_guard_mm
}
fn distance(a: Vec3, b: Vec3) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn axis_distance(p: Vec3, a: [f64; 2], b: [f64; 2]) -> f64 {
    let v = [b[0] - a[0], b[1] - a[1]];
    let n = v[0] * v[0] + v[1] * v[1];
    let t = (((p[0] - a[0]) * v[0] + (p[1] - a[1]) * v[1]) / n).clamp(0., 1.);
    (p[0] - a[0] - t * v[0]).hypot(p[1] - a[1] - t * v[1])
}
// Independent triangle/intersection oracle for exposed edge angle. This is
// evaluated at whole-curve maximum distance, not only emitted endpoints.
fn exposed_angle(q: f64, r: f64, cleared: f64) -> f64 {
    if q + r <= cleared {
        return 0.;
    }
    if q >= cleared + r {
        return 360.;
    }
    let intersection_x = (q * q + r * r - cleared * cleared) / (2. * q);
    2. * (-intersection_x / r).clamp(-1., 1.).acos().to_degrees()
}

#[test]
fn slot_engagement_coverage_and_containment_hold_for_rotations_widths_and_angles() {
    let cancel = AtomicBool::new(false);
    for (start, end, width, angle, cw) in [
        ([8., 12.], [14., 12.], 6., 45., true),
        ([14., 12.], [8., 12.], 8., 90., false),
        ([10., 8.], [14., 13.], 4.5, 135., true),
    ] {
        let mut op = operation();
        op.start_mm = start;
        op.end_mm = end;
        op.width_mm = width;
        op.max_engagement_deg = angle;
        op.clockwise = cw;
        op.floor_z_mm = 9.;
        let mut p = plan(op.clone());
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let r = 2.;
        let mut previous_circle = None;
        let mut seed = true;
        let mut circles = 0;
        let mut height_uncertainty = 0f64;
        let mut last_position = fixture().initial_position_mm;
        while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(m.curve.start(), last_position);
            last_position = m.curve.end();
            match m.curve {
                Curve::Arc {
                    center,
                    start,
                    sweep_rad,
                    ..
                } => {
                    let orbit = distance(center, start);
                    let cleared = orbit + r;
                    assert!(axis_distance(center, op.start_mm, op.end_mm) < 1e-10);
                    // Every point of the disk sweep lies within the nominal capsule.
                    assert!(orbit + r <= width / 2.);
                    assert_eq!(sweep_rad, if cw { -TAU } else { TAU });
                    let direction = [end[0] - op.start_mm[0], end[1] - op.start_mm[1]];
                    let radial = [start[0] - center[0], start[1] - center[1]];
                    let tangent = if cw {
                        [radial[1], -radial[0]]
                    } else {
                        [-radial[1], radial[0]]
                    };
                    // Orbit entry/exit must point along the next connector in
                    // BOTH senses, avoiding a hidden 180-degree reversal.
                    assert!(tangent[0] * direction[0] + tangent[1] * direction[1] > 0.);
                    assert!((tangent[0] * direction[1] - tangent[1] * direction[0]).abs() < 1e-8);
                    if let Some(old) = previous_circle {
                        let gap = distance(old, center);
                        let q = gap + orbit; // continuous full-orbit maximum, exact
                        assert!(exposed_angle(q, r, cleared) <= angle + 1e-8);
                        // Entire capsule side-band coverage between neighboring
                        // disks follows from their midpoint cross section.
                        let covered_half_width = (cleared * cleared - (gap / 2.).powi(2)).sqrt();
                        assert!(width / 2. - covered_half_width <= op.boundary_tolerance_mm);
                        // Cross-check circle intersection oracle against an
                        // independent angular occupancy sum at the worst center.
                        let n = 16384;
                        let outside = (0..n)
                            .filter(|i| {
                                let theta = TAU * (*i as f64 + 0.5) / n as f64;
                                (q + r * theta.cos()).hypot(r * theta.sin()) > cleared
                            })
                            .count() as f64
                            * 360.
                            / n as f64;
                        assert!((outside - exposed_angle(q, r, cleared)).abs() < 720. / n as f64);
                    }
                    assert_eq!(
                        m.feed_mm_min,
                        Some(if seed {
                            op.entry_feed_mm_min
                        } else {
                            op.feed_mm_min
                        })
                    );
                    seed = false;
                    previous_circle = Some(center);
                    circles += 1;
                }
                Curve::Line { start, end } if m.kind == MoveKind::Link => {
                    // Distance-to-convex-segment is convex along a line, so both
                    // endpoint distances bound the complete cutter envelope.
                    assert!(axis_distance(start, op.start_mm, op.end_mm) + r <= width / 2.);
                    assert!(axis_distance(end, op.start_mm, op.end_mm) + r <= width / 2.);
                    let old = previous_circle.unwrap();
                    let a = distance(old, start);
                    let b = distance(old, end);
                    let cleared = p.report().orbit_radius_mm + r;
                    assert!(exposed_angle(a.max(b), r, cleared) <= angle + 1e-8);
                }
                _ => {}
            }
            height_uncertainty = height_uncertainty.max(commit(&mut stock, &m));
        }
        assert!(circles > 2);
        assert_eq!(p.report().stock_clear_advances, 0);
        assert_eq!(p.report().seed_orbits, 1);
        assert!(p.report().max_bounded_engagement_deg <= angle);
        let last = previous_circle.unwrap();
        assert!(distance(last, [end[0], end[1], 9.]) < 1e-10);
        p.finish_report(&cancel).unwrap();
        // Actual stock upper heights over the interior and protected exterior
        // are a separate material check, with explicit cell-radius uncertainty.
        let cell_guard = stock.spec().cell_size_mm * 2.;
        for i in 0..stock.cells().len() {
            let q = stock.cell_center(i).unwrap();
            let d = axis_distance([q[0], q[1], 0.], start, end);
            let h = stock.cells()[i];
            if d < width / 2. - cell_guard {
                assert!(
                    h.upper_mm <= 9. + height_uncertainty + 1e-9,
                    "interior {q:?}: {h:?}"
                );
            }
            if d > width / 2. + cell_guard {
                assert_eq!(h.lower_mm, 10., "protected exterior {q:?}");
            }
        }
    }
}

#[test]
fn slot_adjustable_engagement_changes_path_density_and_each_depth_has_seed_exception() {
    let mut counts = Vec::new();
    for angle in [60., 120.] {
        let mut op = operation();
        op.end_mm = [12., 12.];
        op.max_engagement_deg = angle;
        let mut p = plan(op.clone());
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let mut count = 0;
        while let Some(m) = p
            .next_motion_with_stock(&stock, &AtomicBool::new(false))
            .unwrap()
        {
            commit(&mut stock, &m);
            count += 1;
        }
        assert_eq!(p.report().seed_orbits, 2);
        assert_eq!(p.report().depth_passes, 2);
        assert!(p.report().max_bounded_engagement_deg <= angle);
        assert!(p.report().seed_entry_policy.contains("NOT limited"));
        assert_eq!(p.report().target_z_mm, op.floor_z_mm);
        counts.push(count);
    }
    assert!(counts[0] > counts[1]);
}

#[test]
fn slot_stock_clear_corridor_can_advance_farther_without_adding_height_allowance() {
    let op = operation();
    let mut p = plan(op.clone());
    let mut snapshot = Stock::new(fixture().stock.height().unwrap().clone())
        .unwrap()
        .snapshot();
    snapshot.cells.fill(HeightBounds {
        lower_mm: 8.,
        upper_mm: 8.,
    });
    let mut stock = Stock::from_snapshot(snapshot).unwrap();
    let mut motions = 0;
    while let Some(m) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        commit(&mut stock, &m);
        motions += 1;
    }
    assert_eq!(p.report().stock_clear_advances, 2);
    assert_eq!(p.report().controlled_orbits, 2);
    assert!(motions < 20);
    // A thin uncertain/material wafer is NOT cleared under a manufacturing budget.
    let mut snapshot = Stock::new(fixture().stock.height().unwrap().clone())
        .unwrap()
        .snapshot();
    snapshot.cells.fill(HeightBounds {
        lower_mm: 9.,
        upper_mm: 9.005,
    });
    let mut stock = Stock::from_snapshot(snapshot).unwrap();
    let mut p = plan(op);
    while let Some(m) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        commit(&mut stock, &m);
    }
    assert_eq!(p.report().stock_clear_advances, 0);
}

#[test]
fn slot_rejects_stale_stock_and_cancelled_query_does_not_advance_cursor() {
    struct CancelAfter(Cell<usize>);
    impl CancelSignal for CancelAfter {
        fn is_cancelled(&self) -> bool {
            let n = self.0.get() + 1;
            self.0.set(n);
            n >= 3
        }
    }
    let cancel = AtomicBool::new(false);
    let mut p = plan(operation());
    let mut expected = plan(operation());
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    loop {
        let m = p.next_motion_with_stock(&stock, &cancel).unwrap().unwrap();
        assert_eq!(
            expected
                .next_motion_with_stock(&stock, &cancel)
                .unwrap()
                .unwrap(),
            m
        );
        assert!(
            p.next_motion_with_stock(&stock, &cancel)
                .unwrap_err()
                .contains("commit")
        );
        commit(&mut stock, &m);
        if matches!(m.curve, Curve::Arc { .. }) {
            break;
        }
    }
    assert!(
        p.next_motion_with_stock(&stock, &CancelAfter(Cell::new(0)))
            .unwrap_err()
            .contains("cancel")
    );
    assert_eq!(
        p.next_motion_with_stock(&stock, &cancel).unwrap(),
        expected.next_motion_with_stock(&stock, &cancel).unwrap()
    );
}

#[test]
fn slot_rejects_unsupported_geometry_and_unbounded_work() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    for change in 0..9 {
        let mut op = operation();
        match change {
            0 => op.width_mm = 4.,
            1 => op.width_mm = 8.01,
            2 => op.end_mm = op.start_mm,
            3 => op.max_engagement_deg = 0.,
            4 => op.boundary_tolerance_mm = 1e-5,
            5 => op.stepdown_mm = 1e-6,
            6 => op.end_mm = [f64::NAN, 12.],
            7 => op.floor_z_mm = -1.,
            _ => op.feed_mm_min = f64::NAN,
        }
        assert!(
            SlotPlan::prepare(
                op,
                &m.tools[0],
                m.stock.height().unwrap(),
                m.initial_position_mm,
                1,
                &cancel
            )
            .is_err(),
            "case{change}"
        );
    }
    let mut t = m.tools[0].clone();
    t.cutter.center_cutting = false;
    assert!(
        SlotPlan::prepare(
            operation(),
            &t,
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel
        )
        .is_err()
    );
    assert!(
        SlotPlan::prepare(
            operation(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            u64::MAX,
            &cancel
        )
        .is_err()
    );
}
#[test]
fn slot_passes_link_at_retract_height_and_end_at_clearance() {
    let m = fixture();
    let top = m.stock.height().unwrap().max_mm[2];
    let mut op = operation();
    let clearance = op.clearance_z_mm;
    let (retract, feed) = (top + (clearance - top) / 2., top + (clearance - top) / 4.);
    op.retract_z_mm = Some(retract);
    op.feed_z_mm = Some(feed);
    let mut p = plan(op.clone());
    assert!(p.report().depth_passes > 1);
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut motions = vec![];
    while let Some(motion) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        commit(&mut stock, &motion);
        motions.push(motion);
    }
    let mut at_retract = 0;
    for motion in &motions {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        match motion.kind {
            MoveKind::Rapid => {
                assert!(a[2] >= feed && b[2] >= feed, "{motion:?}");
                at_retract += usize::from(a[2] == retract && b[2] == retract);
            }
            MoveKind::Retract => assert_eq!(b[2], feed),
            MoveKind::Plunge => assert_eq!(a[2], feed),
            _ => {}
        }
    }
    assert!(at_retract > 0, "passes link at the retract height");
    assert_eq!(motions.last().unwrap().curve.end()[2], clearance);
    let mut bad = op;
    bad.feed_z_mm = Some(top);
    assert!(
        SlotPlan::prepare(
            bad,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false)
        )
        .err()
        .unwrap()
        .contains("stock top")
    );
}
