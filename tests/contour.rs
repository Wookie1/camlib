use camlib::{
    contour::*, motion::*, planar::*, pocket::Operation, stock::*, stream::StreamManifest,
};
use std::{f64::consts::TAU, sync::atomic::AtomicBool};
fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/contour.json")).unwrap()
}
fn operation() -> ContourOperation {
    match fixture().operations.remove(0) {
        Operation::Contour(p) => p,
        _ => panic!("fixture"),
    }
}
fn plan(op: ContourOperation) -> Result<ContourPlan, String> {
    let m = fixture();
    ContourPlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
}
fn commit(stock: &mut Stock, m: &Motion) {
    let t = fixture().tools[0].prepare().unwrap();
    let d = stock
        .preview_sweep(&t, m, 1., SweepOptions::default(), &AtomicBool::new(false))
        .unwrap();
    stock.apply_delta(&d).unwrap();
}

#[test]
fn contour_cad_annulus_keeps_protected_part_and_cuts_outer_and_hole_bands_continuously() {
    for reverse in [false, true] {
        let mut op = operation();
        op.reverse = reverse;
        let mut p = plan(op.clone()).unwrap();
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let cancel = AtomicBool::new(false);
        let mut position = fixture().initial_position_mm;
        let mut arcs = 0;
        while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(m.curve.start(), position);
            position = m.curve.end();
            if let Curve::Arc {
                start,
                center,
                sweep_rad,
                ..
            } = m.curve
            {
                assert!((center[0] - 20.).abs() < 1e-8 && (center[1] - 20.).abs() < 1e-8);
                let radius = (start[0] - 20.).hypot(start[1] - 20.);
                // Exact entire-circle clearance, independently of offset/scene checks.
                if radius > 8. {
                    assert!(radius - 2. > 8.);
                    assert_eq!(sweep_rad.is_sign_negative(), reverse);
                } else {
                    assert!(radius + 2. < 4.);
                    assert_eq!(sweep_rad.is_sign_negative(), !reverse);
                }
                assert!(sweep_rad.abs() <= TAU / 2. + 1e-10);
                arcs += 1;
            }
            assert!(m.curve.start()[2] >= 2. && m.curve.end()[2] >= 2.);
            commit(&mut stock, &m);
        }
        p.finish_report(&cancel).unwrap();
        assert_eq!(arcs, 16);
        assert_eq!(p.report().loops_completed, 8);
        assert!(p.report().preflight_nodes < 100);
        assert!(p.report().wall_completion.contains("not_certified"));
        for (i, h) in stock.cells().iter().enumerate() {
            let q = stock.cell_center(i).unwrap();
            let r = (q[0] - 20.).hypot(q[1] - 20.);
            if (4.5..7.5).contains(&r) || r > 12.5 {
                assert_eq!(h.lower_mm, 10., "protected {q:?}:{h:?}");
            }
            if r < 3.5 || (8.5..11.5).contains(&r) {
                assert!(h.upper_mm < 2.003, "uncut band {q:?}:{h:?}");
            }
            assert!(h.lower_mm >= 1.997);
        }
    }
}
#[test]
fn contour_rectangle_retains_corner_arcs_and_continuously_admits_tangent_leads() {
    let mut op = operation();
    op.boundaries = Boundaries::Outer;
    op.floor_z_mm = 8.;
    op.lead_in_mm = 1.;
    op.lead_out_mm = 1.;
    op.region = PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "rectangle".into(),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component {
            outer: Boundary {
                vertices: vec![
                    [10., 10., 0.],
                    [20., 10., 0.],
                    [20., 20., 0.],
                    [10., 20., 0.],
                ],
            },
            holes: vec![],
        }],
    };
    // The default ring start can be on a corner, so the source checker (rather
    // than an endpoint test) decides whether the requested tangents are legal.
    let mut p = plan(op).unwrap();
    assert!(p.report().retained_analytic_arcs >= 4);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let mut lead = 0;
    while let Some(m) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        if m.kind == MoveKind::Lead {
            lead += 1;
        }
        commit(&mut stock, &m);
    }
    assert_eq!(lead, 2);
    assert_eq!(p.report().loops_completed, 1);
    for (i, h) in stock.cells().iter().enumerate() {
        let q = stock.cell_center(i).unwrap();
        if q[0] > 10.5 && q[0] < 19.5 && q[1] > 10.5 && q[1] < 19.5 {
            assert_eq!(h.lower_mm, 10.);
        }
    }
}
#[test]
fn contour_allowances_partial_depth_and_direction_are_explicit() {
    let mut op = operation();
    op.radial_allowance_mm = 0.2;
    op.bottom_allowance_mm = 0.3;
    op.stepdown_mm = 3.;
    let mut p = plan(op).unwrap();
    assert_eq!(p.report().target_z_mm, 2.3);
    assert_eq!(p.report().depth_passes, 3);
    assert!((p.report().path_safety_margin_mm - 0.005).abs() < 1e-12);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let mut lowest = f64::INFINITY;
    while let Some(m) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        lowest = lowest.min(m.curve.end()[2]);
        commit(&mut stock, &m);
    }
    assert_eq!(lowest, 2.3);
}
#[test]
fn contour_does_not_silently_erase_holes_or_admit_a_lead_through_retained_material() {
    let mut small = operation();
    small.region.components[0].holes = vec![
        PlanarRegion::circle("small", [20., 20.], 1.5)
            .unwrap()
            .components
            .remove(0)
            .outer,
    ];
    assert!(plan(small.clone()).unwrap_err_string().contains("topology"));
    small.boundaries = Boundaries::Outer;
    assert!(plan(small).is_ok());
    let mut bad = operation();
    bad.boundaries = Boundaries::Holes;
    bad.lead_in_mm = 2.;
    assert!(plan(bad).unwrap_err_string().contains("admission"));
}
// Keep error assertions readable without requiring Debug for a native plan.
trait ErrorString {
    fn unwrap_err_string(self) -> String;
}
impl ErrorString for Result<ContourPlan, String> {
    fn unwrap_err_string(self) -> String {
        match self {
            Err(e) => e,
            Ok(_) => panic!("unexpected accepted contour"),
        }
    }
}
#[test]
fn contour_cancel_stale_and_invalid_inputs_preserve_next_motion() {
    let cancel = AtomicBool::new(false);
    let mut p = plan(operation()).unwrap();
    let mut expected = plan(operation()).unwrap();
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    assert!(p.finish_report(&cancel).is_err());
    for _ in 0..5 {
        let m = p.next_motion_with_stock(&stock, &cancel).unwrap().unwrap();
        assert_eq!(
            m,
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
        commit(&mut stock, &m);
        assert!(
            p.next_motion_with_stock(&stock, &AtomicBool::new(true))
                .unwrap_err()
                .contains("cancel")
        );
    }
    assert_eq!(
        p.next_motion_with_stock(&stock, &cancel).unwrap(),
        expected.next_motion_with_stock(&stock, &cancel).unwrap()
    );
    let mut bad = operation();
    bad.region.boundary_error_mm = 0.1;
    assert!(plan(bad).unwrap_err_string().contains("uncertainty"));
    let mut bad = operation();
    bad.stepdown_mm = 0.;
    assert!(plan(bad).is_err());
}
#[test]
fn contour_retract_and_feed_heights_link_between_passes_and_end_at_clearance() {
    let top = fixture().stock.height().unwrap().max_mm[2];
    let mut op = operation();
    op.stepdown_mm = 3.;
    let clearance = op.clearance_z_mm;
    let (retract, feed) = (top + (clearance - top) / 2., top + (clearance - top) / 4.);
    assert!(retract < clearance);
    op.retract_z_mm = Some(retract);
    op.feed_z_mm = Some(feed);
    let mut p = plan(op.clone()).unwrap();
    assert!(p.report().depth_passes > 1);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let mut motions = vec![];
    while let Some(m) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        commit(&mut stock, &m);
        motions.push(m);
    }
    let (mut travels_at_retract, mut descents) = (0, 0);
    for m in &motions {
        let (a, b) = (m.curve.start(), m.curve.end());
        match m.kind {
            // Rapids never go below the feed height (which is above the stock).
            MoveKind::Rapid => {
                assert!(a[2] >= feed && b[2] >= feed, "{m:?}");
                if a[2] == b[2] && (a[0], a[1]) != (b[0], b[1]) {
                    assert!(b[2] == retract || b[2] == clearance, "{m:?}");
                    travels_at_retract += usize::from(b[2] == retract);
                }
                if b[2] == feed && a[2] > feed {
                    descents += 1;
                }
            }
            // Leaving a pass: feed rate up to the feed height only.
            MoveKind::Retract => assert_eq!(b[2], feed),
            MoveKind::Plunge => assert_eq!(a[2], feed),
            _ => {}
        }
    }
    assert!(travels_at_retract > 0, "passes link at the retract height");
    assert!(descents > 1);
    assert_eq!(
        motions.last().unwrap().curve.end()[2],
        clearance,
        "operation ends at clearance"
    );

    for (r, f, expected) in [
        (Some(retract), Some(top), "stock top"),
        (Some(clearance + 1.), None, "clearance"),
        (Some(feed), Some(retract), "not be above retract"),
    ] {
        let mut bad = op.clone();
        bad.retract_z_mm = r;
        bad.feed_z_mm = f;
        assert!(plan(bad).err().unwrap().contains(expected));
    }
}

#[test]
fn holding_tabs_leave_material_on_the_lower_passes_only() {
    let cancel = AtomicBool::new(false);
    for shape in [TabShape::Triangle, TabShape::Rectangle] {
        let mut op = operation();
        op.boundaries = Boundaries::Outer;
        op.tabs = Some(Tabs {
            width_mm: 2.,
            height_mm: 3.,
            count: Some(4),
            spacing_mm: None,
            shape,
        });
        let mut p = plan(op).unwrap();
        let spec = fixture().stock.height().unwrap().clone();
        let mut stock = Stock::new(spec.clone()).unwrap();
        let mut top_hits = 0;
        while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            commit(&mut stock, &m);
            if m.kind != MoveKind::Rapid && m.curve.end()[2] == 5. && m.curve.start()[2] < 5. {
                top_hits += 1;
            }
            // The tool never cuts below the floor or above the tab top
            // while lifted.
            assert!(m.curve.end()[2] >= 2. - 1e-9);
        }
        // Two tabbed passes (Z4, Z2), four tabs each.
        assert_eq!(top_hits, 8, "{shape:?}");
        // Around the part, between its edge (r 8) and the cut's outside
        // (r 12): four tabs stand to Z5, the rest is cut to the floor.
        let height = |x: f64, y: f64| {
            let i = ((x - spec.min_mm[0]) / spec.cell_size_mm) as usize;
            let j = ((y - spec.min_mm[1]) / spec.cell_size_mm) as usize;
            let n = ((spec.max_mm[0] - spec.min_mm[0]) / spec.cell_size_mm).ceil() as usize;
            stock.cells()[j * n + i]
        };
        let tab: Vec<bool> = (0..720)
            .map(|k| {
                let a = k as f64 * TAU / 720.;
                let h = height(20. + 10. * a.cos(), 20. + 10. * a.sin());
                // Never above the tab top; triangular ramps leave a slope.
                assert!(h.lower_mm <= 5. + 1e-9, "{h:?}");
                if shape == TabShape::Rectangle {
                    assert!(h.upper_mm > 4.5 || h.upper_mm < 2.5, "{h:?}");
                }
                h.upper_mm > 4.5
            })
            .collect();
        let groups = (0..720)
            .filter(|&k| tab[k] && !tab[(k + 719) % 720])
            .count();
        assert_eq!(groups, 4, "{shape:?}");
    }
    // Rectangular tabs move straight up: not with cutter compensation.
    let mut op = operation();
    op.boundaries = Boundaries::Outer;
    op.lead_in_mm = 2.;
    op.lead_out_mm = 2.;
    op.compensation = Compensation::Wear;
    op.tabs = Some(Tabs {
        width_mm: 2.,
        height_mm: 3.,
        count: Some(4),
        spacing_mm: None,
        shape: TabShape::Rectangle,
    });
    assert!(plan(op.clone()).err().unwrap().contains("triangular"));
    op.tabs.as_mut().unwrap().count = Some(40);
    op.tabs.as_mut().unwrap().shape = TabShape::Triangle;
    assert!(plan(op).err().unwrap().contains("do not fit"));
}
