//! Plasma/laser profile cutting: order, pierce points, kerf clearance, events
//! and the through-cut stock.
use camlib::{
    motion::*,
    planar::{Boundary, Component, PlanarRegion},
    stock::*,
    stock_contact,
    thermal::{ThermalCutOperation, ThermalCutPlan},
    tool::{Cutter, CutterShape, HolderSection, ToolAssembly},
    verification::Status,
};
use std::sync::atomic::AtomicBool;

const KERF: f64 = 1.5;

fn torch() -> ToolAssembly {
    ToolAssembly {
        id: "torch".into(),
        cutter: Cutter {
            id: "nozzle".into(),
            name: String::new(),
            shape: CutterShape::Beam,
            diameter_mm: KERF,
            cutting_length_mm: 1.,
            overall_length_mm: 40.,
            shank_diameter_mm: 8.,
            flutes: 1,
            center_cutting: true,
            material: String::new(),
            neck: None,
        },
        stickout_mm: 30.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 30.,
            z_max_mm: 60.,
            radius_min_mm: 15.,
            radius_max_mm: 15.,
        }],
    }
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Boundary {
    Boundary {
        vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
    }
}
fn circle(c: [f64; 2], r: f64) -> Boundary {
    Boundary {
        vertices: vec![[c[0] + r, c[1], 1.], [c[0] - r, c[1], 1.]],
    }
}

fn spec() -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [200., 120., 6.],
        cell_size_mm: 0.25,
        max_cells: 1 << 22,
    }
}

/// A plate with a large hole holding a nested washer, a bolt hole and a
/// small hole; the washer has its own hole.
fn parts() -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "parts".into(),
        boundary_error_mm: 0.,
        components: vec![
            Component {
                outer: rect(20., 20., 120., 100.),
                holes: vec![
                    circle([60., 60.], 25.),
                    circle([100., 40.], 6.),
                    circle([100., 80.], 3.),
                ],
            },
            Component {
                outer: circle([60., 60.], 15.),
                holes: vec![circle([60., 60.], 5.)],
            },
        ],
        source: serde_json::Value::Null,
    }
}

fn operation(extra: serde_json::Value) -> ThermalCutOperation {
    let mut value = serde_json::json!({
        "id": "plasma", "tool": "torch", "parts": parts(), "feed_mm_min": 2500.,
        "cut_height_mm": 1.5, "height_mode": "program", "pierce_height_mm": 3.8,
        "travel_height_mm": 10., "plunge_mm_min": 1000., "pierce_delay_s": 0.5,
        "lead_in": {"length_mm": 2., "radius_mm": 3.}, "lead_out": {"length_mm": 1., "radius_mm": 2.},
        "overcut_mm": -0.5, "height_control": true,
        "small_holes": {"feed_percent": 60.},
        "tolerance_mm": 1e-5, "work_offset": 1, "tool_number": 1, "length_register": 1
    });
    for (k, v) in extra.as_object().unwrap() {
        value[k] = v.clone();
    }
    serde_json::from_value(value).unwrap()
}

fn run(op: ThermalCutOperation) -> Result<(Vec<Event>, ThermalCutPlan, Stock), String> {
    let cancel = AtomicBool::new(false);
    let tool = torch();
    let prepared = tool.prepare().unwrap();
    let mut stock = Stock::new(spec()).unwrap();
    stock.enable_contact_queries(&cancel).unwrap();
    let initial = [0., 0., 40.];
    let mut plan = ThermalCutPlan::prepare(op, &tool, &spec(), initial, 1, &cancel)?;
    let mut events = plan.prefix_events();
    events.extend(plan.take_events());
    while let Some(m) = plan.next_motion_with_stock(&stock, &cancel)? {
        let report =
            stock_contact::check(&stock, &prepared, &m, Default::default(), &cancel).unwrap();
        assert_eq!(report.status, Status::Clear, "{m:?}");
        let d = stock
            .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        events.push(Event::Motion { motion: m });
        events.extend(plan.take_events());
    }
    events.extend(plan.suffix_events());
    plan.finish_report(&cancel).unwrap();
    // The whole program is valid: beam on for every cut, THC only with it.
    let program = Program {
        version: 1,
        id: "p".into(),
        revision: "1".into(),
        units: "mm".into(),
        tolerance_mm: 1e-5,
        frame: WorkFrame::identity(),
        initial_position_mm: initial,
        tools: vec![tool],
        events: events.clone(),
    };
    let mut v = ProgramValidator::new(&program, None).unwrap();
    if let Err(e) = v.consume(&events) {
        for (i, ev) in events.iter().enumerate().skip(30).take(14) {
            if let Event::Motion { motion } = ev {
                eprintln!(
                    "{i} {:?} {:?} -> {:?}",
                    motion.kind,
                    motion.curve.start(),
                    motion.curve.end()
                );
            } else {
                eprintln!("{i} {ev:?}");
            }
        }
        panic!("{e}");
    }
    Ok((events, plan, stock))
}

fn top_at(stock: &Stock, p: [f64; 2]) -> (f64, f64) {
    let s = stock.spec();
    let nx = ((s.max_mm[0] - s.min_mm[0]) / s.cell_size_mm).round() as usize;
    let i = ((p[0] - s.min_mm[0]) / s.cell_size_mm) as usize;
    let j = ((p[1] - s.min_mm[1]) / s.cell_size_mm) as usize;
    let c = stock.cells()[j * nx + i];
    (c.lower_mm, c.upper_mm)
}

#[test]
fn cuts_inner_features_first_and_keeps_every_part() {
    let (events, plan, stock) = run(operation(serde_json::json!({}))).unwrap();
    let r = plan.report();
    // Washer (nested in the plate's big hole) first: its hole, its outline;
    // then the plate's holes, the plate outline last.
    assert_eq!(r.order.len(), 6);
    assert_eq!(r.order[0], (1, Some(0)));
    assert_eq!(r.order[1], (1, None));
    assert_eq!(*r.order.last().unwrap(), (0, None));
    assert_eq!(r.loops_completed, 6);
    // The 6 mm hole is small: 1.5 x the 6 mm sheet is 9 mm.
    assert_eq!(r.small_holes, 1);
    let count = |f: &dyn Fn(&Event) -> bool| events.iter().filter(|e| f(e)).count();
    assert_eq!(
        count(&|e| matches!(e, Event::Beam { power_percent } if *power_percent > 0.)),
        6
    );
    assert_eq!(
        count(&|e| matches!(e, Event::Beam { power_percent } if *power_percent == 0.)),
        6
    );
    assert_eq!(count(&|e| matches!(e, Event::Dwell { .. })), 6);
    // THC on every loop but the small hole.
    assert_eq!(
        count(&|e| matches!(e, Event::HeightControl { on: true })),
        5
    );
    // The small hole is cut at 60% feed.
    assert!(events.iter().any(|e| matches!(e, Event::Motion { motion } if motion.kind == MoveKind::Cut && motion.feed_mm_min == Some(1500.))));
    // Kerf lines are cut through; part material and scrap far away are not.
    let kerf_point = [20. - KERF / 2. - 0.005, 60.];
    assert_eq!(top_at(&stock, kerf_point), (0., 0.));
    assert_eq!(top_at(&stock, [70., 30.]), (6., 6.)); // plate
    assert_eq!(top_at(&stock, [60., 50.]), (6., 6.)); // washer
    assert_eq!(top_at(&stock, [180., 110.]), (6., 6.)); // scrap
    // Every cut and lead is at the cut height, travel at the travel height.
    for e in &events {
        if let Event::Motion { motion } = e {
            match motion.kind {
                MoveKind::Cut | MoveKind::Lead => {
                    assert_eq!(motion.curve.start()[2], 7.5);
                    assert_eq!(motion.curve.end()[2], 7.5);
                }
                MoveKind::Plunge => assert_eq!(motion.curve.start()[2], 9.8),
                _ => {}
            }
        }
    }
}

#[test]
fn fixed_height_mode_has_no_z_moves_and_refusals_name_the_problem() {
    let op = operation(serde_json::json!({
        "height_mode": "fixed", "pierce_height_mm": null, "travel_height_mm": null, "plunge_mm_min": null
    }));
    let (events, _, _) = run(op).unwrap();
    for e in &events[3..] {
        if let Event::Motion { motion } = e {
            assert_eq!(motion.curve.start()[2], 7.5, "{motion:?}");
            assert_eq!(motion.curve.end()[2], 7.5, "{motion:?}");
        }
    }
    // A lead longer than the gap to the next part is not proven clear.
    let err = run(operation(
        serde_json::json!({"lead_in": {"length_mm": 40., "radius_mm": 3.}}),
    ))
    .err()
    .unwrap();
    assert!(err.contains("not proven clear"), "{err}");
    // A milling cutter is not a beam.
    let mut mill = torch();
    mill.cutter.shape = CutterShape::Flat;
    let err = ThermalCutPlan::prepare(
        operation(serde_json::json!({})),
        &mill,
        &spec(),
        [0., 0., 40.],
        1,
        &AtomicBool::new(false),
    )
    .err()
    .unwrap();
    assert!(err.contains("beam tool"), "{err}");
}

#[test]
fn micro_joints_leave_bridges_on_outlines() {
    let op = operation(serde_json::json!({
        "micro_joints": {"width_mm": 1.0, "count": 4}
    }));
    let (events, plan, stock) = run(op).unwrap();
    // Two outlines (plate, washer), four joints each; the beam goes off and
    // on again over each.
    assert_eq!(plan.report().micro_joints, 8);
    let on = events
        .iter()
        .filter(|e| matches!(e, Event::Beam { power_percent } if *power_percent > 0.))
        .count();
    assert_eq!(on, 6 + 8);
    // The joint material at the middle of the plate's bottom edge... is
    // wherever a joint landed: some kerf cells along the outline stay solid.
    let s = stock.spec().clone();
    let solid_on_kerf = (0..400)
        .map(|k| [20. + 100. * k as f64 / 400., 20. - KERF / 2. - 0.005])
        .filter(|p| top_at(&stock, *p).1 == s.max_mm[2])
        .count();
    assert!(solid_on_kerf > 0, "a joint keeps the outline attached");
    // With wear compensation micro-joints are refused.
    let err = run(operation(serde_json::json!({
        "micro_joints": {"width_mm": 1.0, "count": 4}, "compensation": "wear"
    })))
    .err()
    .unwrap();
    assert!(err.contains("micro-joints"), "{err}");
}

#[test]
fn machining_limits_cover_beam_depth_and_travel() {
    let region = |x0, y0, x1, y1| {
        serde_json::json!({
            "version": 1, "units": "mm", "name": "limit", "source": null, "components": [{"outer": rect(x0,y0,x1,y1), "holes": []}],
            "boundary_error_mm": 0.0
        })
    };
    let prepare = |extra| {
        ThermalCutPlan::prepare(
            operation(extra),
            &torch(),
            &spec(),
            [0., 0., 40.],
            1,
            &AtomicBool::new(false),
        )
    };
    prepare(serde_json::json!({"limits": {
        "contain": region(0.,0.,200.,120.),
        "avoid": [{"region": region(180.,100.,190.,110.), "top_z_mm": 30.}]
    }}))
    .unwrap_or_else(|e| panic!("clear thermal limits: {e}"));
    // Below the nozzle but above the sheet bottom: the beam still crosses it.
    assert!(
        prepare(serde_json::json!({"limits": {"avoid": [
            {"region": region(15.,15.,125.,105.), "top_z_mm": 1.}
        ]}}))
        .is_err()
    );
    assert!(prepare(serde_json::json!({"limits": {"contain": region(40.,40.,80.,80.)}})).is_err());
    // Travel to the first pierce passes through this tall obstacle.
    assert!(
        prepare(serde_json::json!({"limits": {"avoid": [
            {"region": region(-5.,-5.,200.,120.), "top_z_mm": 50.}
        ]}}))
        .is_err()
    );
}
