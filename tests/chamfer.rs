use camlib::{
    chamfer::*, contour::Boundaries, motion::*, operation::OperationPlan, planar::PlanarRegion,
    pocket::Operation, stock::*, stream::StreamManifest, tool::CutterShape,
};
use std::{
    cell::Cell,
    sync::{OnceLock, atomic::AtomicBool},
};
fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/chamfer.json")).unwrap()
}
fn operation() -> ChamferOperation {
    match fixture().operations.remove(1) {
        Operation::Chamfer(op) => op,
        _ => panic!("fixture"),
    }
}
fn rough() -> (Stock, Vec3, u64) {
    static BASE: OnceLock<(StockSnapshot, Vec3, u64)> = OnceLock::new();
    let (snap, pos, id) = BASE.get_or_init(|| {
        let m = fixture();
        let cancel = AtomicBool::new(false);
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut p = OperationPlan::prepare(
            m.operations[0].clone(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let mut pos = m.initial_position_mm;
        let mut next = 1;
        while let Some(motion) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            pos = motion.curve.end();
            next = motion.id + 1;
            let d = stock
                .preview_sweep(
                    &m.tools[0].prepare().unwrap(),
                    &motion,
                    1.,
                    m.sweep,
                    &cancel,
                )
                .unwrap();
            stock.apply_delta(&d).unwrap();
        }
        (stock.snapshot(), pos, next)
    });
    (Stock::from_snapshot(snap.clone()).unwrap(), *pos, *id)
}
fn commit(stock: &mut Stock, m: &Motion, tool: &camlib::tool::ToolAssembly) {
    let delta = stock
        .preview_sweep(
            &tool.prepare().unwrap(),
            m,
            1.,
            SweepOptions::default(),
            &AtomicBool::new(false),
        )
        .unwrap();
    stock.apply_delta(&delta).unwrap();
}

#[test]
fn chamfer_angle_tip_and_whole_cell_conical_oracle_preserve_lower_part() {
    for (angle, tip_diameter) in [(90f64, 0.5), (60., 0.), (120., 1.)] {
        let m = fixture();
        let mut tool = m.tools[1].clone();
        tool.cutter.shape = CutterShape::Chamfer {
            included_angle_rad: angle.to_radians(),
            tip_diameter_mm: tip_diameter,
        };
        let op = operation();
        let slope = (angle.to_radians() / 2.).tan();
        let tip_radius = tip_diameter / 2.;
        let (mut stock, pos, id) = rough();
        let mut p = ChamferPlan::prepare(
            op.clone(),
            &tool,
            m.stock.height().unwrap(),
            pos,
            id,
            &AtomicBool::new(false),
        )
        .unwrap();
        let expected_tip = 10. - 1.5 / slope;
        assert!((p.report().tool_tip_target_z_mm - expected_tip).abs() < 1e-10);
        assert!((p.report().axial_bevel_depth_mm - 1. / slope).abs() < 1e-10);
        let mut last = pos;
        while let Some(motion) = p
            .next_motion_with_stock(&stock, &AtomicBool::new(false))
            .unwrap()
        {
            assert_eq!(motion.curve.start(), last);
            last = motion.curve.end();
            commit(&mut stock, &motion, &tool);
        }
        assert_eq!(p.report().cut_edges_emitted, 4);
        assert_eq!(p.report().loops_completed, 2);
        assert!(p.report().entry_query_work > 0);
        assert_eq!(p.report().entry_checked_from_generation, Some(id - 1));
        p.finish_report(&AtomicBool::new(false)).unwrap();
        let source_error = op.region.boundary_error_mm;
        let rough_d = 2. + source_error + 0.005;
        let axis = tip_radius + 0.5 + source_error + 0.005;
        for (i, h) in stock.cells().iter().enumerate() {
            let q = stock.cell_center(i).unwrap();
            let radius = (q[0] - 20.).hypot(q[1] - 20.);
            let mut expected: f64 = if radius < 4. - rough_d + 2.
                || (8. + rough_d - 2. <= radius && radius <= 8. + rough_d + 2.)
            {
                2.
            } else {
                10.
            };
            for center in [8. + axis, 4. - axis] {
                let d = (radius - center).abs();
                if d <= 3. {
                    expected = expected.min(expected_tip + (d - tip_radius).max(0.) / slope);
                }
            }
            assert!(
                h.lower_mm <= expected + 1e-7 && h.upper_mm >= expected - 1e-7,
                "angle{angle} {q:?}: {h:?} expected{expected}"
            );
            if (4.5..7.5).contains(&radius) {
                assert!(
                    h.lower_mm >= 10. - 1. / slope - 0.003,
                    "lower part gouged {q:?}:{h:?}"
                );
            }
            if radius > 12.5 {
                assert_eq!(h.lower_mm, 10.);
            }
        }
    }
}
#[test]
fn chamfer_selection_reverse_and_partial_width_passes_are_explicit() {
    for (selection, area) in [
        (Boundaries::Outer, 33.),
        (Boundaries::Holes, 39.),
        (Boundaries::All, 24.),
    ] {
        let m = fixture();
        let mut op = operation();
        op.boundaries = selection;
        op.reverse = true;
        op.width_step_mm = Some(0.4);
        let (mut stock, pos, id) = rough();
        let mut p = ChamferPlan::prepare(
            op,
            &m.tools[1],
            m.stock.height().unwrap(),
            pos,
            id,
            &AtomicBool::new(false),
        )
        .unwrap();
        assert_eq!(p.report().width_passes, 3);
        assert!((p.report().top_target_area_mm2 - area * std::f64::consts::PI).abs() < 1e-6);
        let mut deepest = 12f64;
        while let Some(motion) = p
            .next_motion_with_stock(&stock, &AtomicBool::new(false))
            .unwrap()
        {
            deepest = deepest.min(motion.curve.end()[2]);
            if let Curve::Arc {
                start,
                center,
                sweep_rad,
                ..
            } = motion.curve
            {
                let r = (start[0] - center[0]).hypot(start[1] - center[1]);
                assert_eq!(sweep_rad.is_sign_negative(), r > 8.);
            }
            commit(&mut stock, &motion, &m.tools[1]);
        }
        assert!((deepest - 8.5).abs() < 1e-10);
        assert_eq!(p.report().loops_completed, p.report().loops * 3);
        for (i, h) in stock.cells().iter().enumerate() {
            let q = stock.cell_center(i).unwrap();
            let r = (q[0] - 20.).hypot(q[1] - 20.);
            if selection == Boundaries::Outer && (4.5..4.7).contains(&r) {
                assert_eq!(h.lower_mm, 10.);
            }
            if selection == Boundaries::Holes && (7.3..7.5).contains(&r) {
                assert_eq!(h.lower_mm, 10.);
            }
        }
    }
}
#[test]
fn chamfer_rectangular_retained_part_has_planar_bevel_and_corner_protection() {
    let m = fixture();
    let mut op = operation();
    op.boundaries = Boundaries::Outer;
    op.region = PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "square".into(),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![camlib::planar::Component {
            outer: camlib::planar::Boundary {
                vertices: vec![
                    [10., 10., 0.],
                    [30., 10., 0.],
                    [30., 30., 0.],
                    [10., 30., 0.],
                ],
            },
            holes: vec![],
        }],
    };
    op.lead_in_mm = 1.;
    op.lead_out_mm = 1.;
    let mut tool = m.tools[1].clone();
    tool.cutter.center_cutting = true;
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut p = ChamferPlan::prepare(
        op,
        &tool,
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .unwrap();
    assert!((p.report().top_target_area_mm2 - 324.).abs() < 1e-7);
    assert!(p.report().retained_analytic_arcs >= 4);
    while let Some(motion) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        commit(&mut stock, &motion, &tool);
    }
    for (i, h) in stock.cells().iter().enumerate() {
        let q = stock.cell_center(i).unwrap();
        if q[0] > 10. && q[0] < 30. && q[1] > 10. && q[1] < 30. {
            let edge = (q[0] - 10.).min(30. - q[0]).min(q[1] - 10.).min(30. - q[1]);
            let expected = (9.005 + edge).min(10.);
            assert!(
                h.lower_mm <= expected + 1e-7 && h.upper_mm >= expected - 1e-7,
                "plane{q:?}:{h:?},expected{expected}"
            );
            if edge > 0.5 {
                assert!(h.lower_mm >= 8.997);
            }
        }
    }
}
#[test]
fn chamfer_noncenter_entry_and_query_cancellation_never_advance_unchecked_motion() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut raw = ChamferPlan::prepare(
        operation(),
        &m.tools[1],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    assert!(
        raw.next_motion_with_stock(&stock, &cancel)
            .unwrap_err()
            .contains("cleared tip")
    );
    assert_eq!(stock.generation(), 0);
    struct CancelAfter(Cell<usize>);
    impl CancelSignal for CancelAfter {
        fn is_cancelled(&self) -> bool {
            let n = self.0.get() + 1;
            self.0.set(n);
            n >= 3
        }
    }
    let (mut stock, pos, id) = rough();
    let mut p = ChamferPlan::prepare(
        operation(),
        &m.tools[1],
        m.stock.height().unwrap(),
        pos,
        id,
        &cancel,
    )
    .unwrap();
    let mut expected = ChamferPlan::prepare(
        operation(),
        &m.tools[1],
        m.stock.height().unwrap(),
        pos,
        id,
        &cancel,
    )
    .unwrap();
    assert!(
        p.next_motion_with_stock(&stock, &CancelAfter(Cell::new(0)))
            .unwrap_err()
            .contains("cancel")
    );
    assert_eq!(p.report().entry_checked_from_generation, None);
    for _ in 0..5 {
        let motion = p.next_motion_with_stock(&stock, &cancel).unwrap().unwrap();
        assert_eq!(
            motion,
            expected
                .next_motion_with_stock(&stock, &cancel)
                .unwrap()
                .unwrap()
        );
        assert!(
            p.next_motion_with_stock(&stock, &cancel)
                .unwrap_err()
                .contains("commit")
        );
        commit(&mut stock, &motion, &m.tools[1]);
    }
}
#[test]
fn chamfer_topology_invalid_width_leads_and_nested_owner_references_are_checked() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    for case in 0..5 {
        let mut op = operation();
        match case {
            0 => op.width_mm = 2.,
            1 => op.width_mm = 4.,
            2 => {
                op.boundaries = Boundaries::Holes;
                op.lead_in_mm = 3.;
            }
            3 => op.top_z_mm = 1.,
            _ => op.width_step_mm = Some(0.),
        }
        assert!(
            ChamferPlan::prepare(
                op,
                &m.tools[1],
                m.stock.height().unwrap(),
                m.initial_position_mm,
                1,
                &cancel
            )
            .is_err(),
            "case{case}"
        );
    }
    let mut op = operation();
    op.boundaries = Boundaries::Outer;
    op.width_mm = 0.25;
    op.region.source = serde_json::Value::Null;
    op.region.components.push(
        PlanarRegion::circle("island", [20., 20.], 1.)
            .unwrap()
            .components
            .remove(0),
    );
    let mut tool = m.tools[1].clone();
    tool.cutter.center_cutting = true;
    let p = ChamferPlan::prepare(
        op,
        &tool,
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    assert_eq!(p.report().loops, 2);
    assert!((p.report().top_target_area_mm2 - 44.625 * std::f64::consts::PI).abs() < 1e-5);
}
