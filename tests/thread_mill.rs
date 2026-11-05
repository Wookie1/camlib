use camlib::{
    holes::{BoreOperation, BorePlan, ThreadMillOperation, ThreadMillPlan},
    motion::{Curve, Motion, MoveKind, Plane},
    pocket::Operation,
    stock::Stock,
    stream::StreamManifest,
    tool::{Neck, ToolAssembly},
};
use std::{f64::consts::TAU, sync::atomic::AtomicBool};

fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/holes.json")).unwrap()
}
/// Single-profile thread mill: 8 mm tooth, 1.2 mm high, 5 mm neck.
fn thread_tool() -> ToolAssembly {
    let mut tool = fixture().tools[1].clone();
    tool.id = "thread8".into();
    tool.cutter.id = "thread-mill".into();
    tool.cutter.diameter_mm = 8.;
    tool.cutter.cutting_length_mm = 1.2;
    tool.cutter.shank_diameter_mm = 8.;
    tool.cutter.center_cutting = false;
    tool.cutter.neck = Some(Neck {
        diameter_mm: 5.,
        length_mm: 10.,
    });
    tool
}
fn operation(internal: bool, center: [f64; 2]) -> ThreadMillOperation {
    ThreadMillOperation {
        limits: None,
        id: "thread".into(),
        tool: "thread8".into(),
        centers_mm: vec![center],
        internal,
        // M14x1.5 internal (12 mm hole); M12x1.75 external (minor 9.853).
        diameter_mm: if internal { 14. } else { 9.853 },
        start_diameter_mm: 12.,
        pitch_mm: if internal { 1.5 } else { 1.75 },
        right_hand: true,
        climb: true,
        top_z_mm: if internal { 9.5 } else { 10.5 },
        bottom_z_mm: 5.,
        radial_passes: 2,
        spring_passes: 1,
        clearance_z_mm: 12.,
        feed_mm_min: 300.,
        plunge_mm_min: 200.,
        tolerance_mm: 1e-5,
        spindle_rpm: 8000.,
        work_offset: 1,
        tool_number: 3,
        length_register: 3,
        coolant: camlib::motion::Coolant::Flood,
        retract_z_mm: None,
        feed_z_mm: None,
    }
}
fn commit(stock: &mut Stock, tool: &ToolAssembly, motion: &Motion) {
    let tool = tool.prepare().unwrap();
    let d = stock
        .preview_sweep(&tool, motion, 1., fixture().sweep, &AtomicBool::new(false))
        .unwrap_or_else(|e| panic!("{e}: {:?} {:?}", motion.kind, motion.curve));
    stock.apply_delta(&d).unwrap();
}
fn run(
    op: ThreadMillOperation,
    stock: &mut Stock,
) -> Result<(Vec<Motion>, ThreadMillPlan), String> {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let tool = thread_tool();
    let mut plan = ThreadMillPlan::prepare(
        op,
        &tool,
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )?;
    let mut position = m.initial_position_mm;
    let mut motions = vec![];
    while let Some(motion) = plan.next_motion_with_stock(stock, &cancel)? {
        assert_eq!(motion.curve.start(), position);
        position = motion.curve.end();
        commit(stock, &tool, &motion);
        motions.push(motion);
    }
    plan.finish_report(&cancel)?;
    Ok((motions, plan))
}
fn radius(p: [f64; 3], c: [f64; 2]) -> f64 {
    (p[0] - c[0]).hypot(p[1] - c[1])
}

#[test]
fn internal_thread_needs_the_hole_then_climbs_up_counter_clockwise() {
    let m = fixture();
    let center = [10.000000000000027, 12.];
    // Undrilled: refused before the tool goes down.
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let error = run(operation(true, center), &mut stock).err().unwrap();
    assert!(error.contains("not proven open"), "{error}");
    assert!(error.contains("drill the hole"), "{error}");

    // Bore the 12 mm hole to Z 4, then thread.
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let Operation::Bore(mut bore) = fixture().operations.remove(1) else {
        panic!()
    };
    bore.centers_mm = vec![center];
    let bore: BoreOperation = bore;
    let cancel = AtomicBool::new(false);
    let mut plan = BorePlan::prepare(
        bore,
        &m.tools[1],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        commit(&mut stock, &m.tools[1], &motion);
    }
    let before = stock.volume().removed_lower_mm3;
    let (motions, plan) = run(operation(true, center), &mut stock).unwrap();
    let report = plan.report();
    assert_eq!(report.rotation, "ccw");
    assert_eq!(report.z_direction, "up");
    assert_eq!(report.passes_per_hole, 3);
    // Tool radius 4: radial passes to 2.5 and 3, then the spring pass at 3.
    assert_eq!(report.path_radii_mm, vec![2.5, 3., 3.]);
    assert_eq!(report.turns_per_pass, 3.);
    let helices: Vec<_> = motions
        .iter()
        .filter(|m| matches!(m.curve, Curve::Helix { .. }))
        .collect();
    assert_eq!(helices.len(), 9);
    for (i, h) in helices.iter().enumerate() {
        let Curve::Helix {
            start,
            end,
            sweep_rad,
            plane,
            ..
        } = h.curve
        else {
            unreachable!()
        };
        assert_eq!(plane, Plane::Xy);
        assert!((sweep_rad - TAU).abs() < 1e-12, "full ccw turn");
        assert!(
            (end[2] - start[2] - 1.5).abs() < 1e-12,
            "one pitch per turn"
        );
        let r = report.path_radii_mm[i / 3];
        assert!((radius(start, center) - r).abs() < 1e-9);
        assert!((radius(end, center) - r).abs() < 1e-9);
        assert_eq!(h.kind, MoveKind::Cut);
    }
    assert_eq!(helices[0].curve.start()[2], 5.);
    assert_eq!(helices[2].curve.end()[2], 9.5);
    // Leads are tangent semicircles from and back to the hole center.
    let leads: Vec<_> = motions
        .iter()
        .filter(|m| m.kind == MoveKind::Lead)
        .collect();
    assert_eq!(leads.len(), 6);
    for lead in &leads {
        let Curve::Arc { sweep_rad, .. } = lead.curve else {
            panic!("lead must be an arc")
        };
        assert!((sweep_rad - std::f64::consts::PI).abs() < 1e-12);
    }
    assert_eq!(motions.last().unwrap().curve.end()[2], 12.);
    assert!(
        stock.volume().removed_lower_mm3 > before,
        "thread removes stock"
    );
    // The groove is an undercut: the wall top keeps its crests. Only the last
    // turn, whose tooth reaches the top face, can lower it (to its tip).
    let mut crests = 0;
    for (i, cell) in stock.cells().iter().enumerate() {
        let p = stock.cell_center(i).unwrap();
        let r = (p[0] - center[0]).hypot(p[1] - center[1]);
        if (6.3..6.9).contains(&r) {
            assert!(
                cell.upper_mm >= 8. - 1e-9,
                "wall top lowered at r {r}: {cell:?}"
            );
            crests += usize::from(cell.upper_mm == 10.);
        }
    }
    assert!(crests > 100, "{crests}");
    assert_eq!(report.open_columns_proven, 3);
}

#[test]
fn external_thread_goes_down_clockwise_beside_a_cleared_boss() {
    let m = fixture();
    let center = [20., 12.];
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    // Leave a 12 mm boss: clear the ring 6..18 mm around it to Z 4.
    let mill = &m.tools[1];
    let mut at = [center[0] + 8., center[1], 12.];
    for r in [8., 10., 12., 14., 16.] {
        let start = [center[0] + r, center[1], 4.];
        for curve in [
            Curve::Line {
                start: at,
                end: start,
            },
            Curve::Arc {
                start,
                end: start,
                center: [center[0], center[1], 4.],
                plane: Plane::Xy,
                sweep_rad: -TAU,
            },
        ] {
            let motion = Motion {
                id: 1,
                operation: "ring".into(),
                tool: mill.id.clone(),
                kind: MoveKind::Cut,
                curve,
                feed_mm_min: Some(600.),
            };
            commit(&mut stock, mill, &motion);
        }
        at = start;
    }
    let (motions, plan) = run(operation(false, center), &mut stock).unwrap();
    let report = plan.report();
    assert_eq!((report.rotation, report.z_direction), ("cw", "down"));
    let helices: Vec<_> = motions
        .iter()
        .filter(|m| matches!(m.curve, Curve::Helix { .. }))
        .collect();
    // (10.5 - 5) / 1.75 = 3.142... turns: 3 full turns and a partial one.
    assert_eq!(helices.len(), 4 * 3);
    let Curve::Helix { sweep_rad, .. } = helices[3].curve else {
        unreachable!()
    };
    assert!((sweep_rad + TAU * (5.5 / 1.75 - 3.)).abs() < 1e-9);
    assert_eq!(helices[3].curve.end()[2], 5.);
    assert_eq!(helices[0].curve.start()[2], 10.5);
    let last = report.path_radii_mm.last().unwrap();
    assert!((last - (9.853 / 2. + 4.)).abs() < 1e-12);
    // Without the cleared ring the descent beside the boss is refused.
    let mut solid = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let error = run(operation(false, center), &mut solid).err().unwrap();
    assert!(error.contains("clear around the boss"), "{error}");
}

#[test]
fn thread_parameters_fail_precisely() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let error = |op| {
        ThreadMillPlan::prepare(
            op,
            &thread_tool(),
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .err()
        .unwrap()
    };
    let mut op = operation(true, [10., 12.]);
    op.start_diameter_mm = 15.;
    assert!(error(op).contains("start_diameter_mm 15 must be smaller than diameter_mm 14"));
    let mut op = operation(true, [10., 12.]);
    op.start_diameter_mm = 7.;
    assert!(error(op).contains("does not fit in the start hole diameter 7"));
    let mut op = operation(true, [10., 12.]);
    op.bottom_z_mm = -2.;
    assert!(error(op).contains("bottom_z_mm -2 must be within the stock"));
    let mut short = thread_tool();
    short.cutter.neck = Some(Neck {
        diameter_mm: 5.,
        length_mm: 3.,
    });
    let reach = ThreadMillPlan::prepare(
        operation(true, [10., 12.]),
        &short,
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .err()
    .unwrap();
    assert!(
        reach.contains("depth 5 below the stock top exceeds the tooth and neck reach 4.2"),
        "{reach}"
    );
    let mut wide = thread_tool();
    wide.cutter.neck = Some(Neck {
        diameter_mm: 6.5,
        length_mm: 10.,
    });
    let neck = ThreadMillPlan::prepare(
        operation(true, [10., 12.]),
        &wide,
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .err()
    .unwrap();
    assert!(
        neck.contains("the neck diameter 6.5 must clear the thread crests: at most 6"),
        "{neck}"
    );
}
