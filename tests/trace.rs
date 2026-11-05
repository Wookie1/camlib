use camlib::{
    motion::{Curve, MoveKind},
    stock::{Stock, SweepOptions},
    stream::StreamManifest,
    trace::{TraceOperation, TracePath, TracePlan, TraceSegment},
};
use std::{f64::consts::PI, sync::atomic::AtomicBool};

fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/contour.json")).unwrap()
}

fn operation(paths: Vec<TracePath>) -> TraceOperation {
    let m = fixture();
    TraceOperation {
        limits: None,
        id: "engrave".into(),
        tool: m.tools[0].id.clone(),
        paths,
        top_z_mm: m.stock.height().unwrap().max_mm[2],
        depth_z_mm: m.stock.height().unwrap().max_mm[2] - 1.5,
        stepdown_mm: 0.6,
        clearance_z_mm: m.initial_position_mm[2],
        feed_mm_min: 400.,
        plunge_mm_min: 100.,
        tolerance_mm: 1e-5,
        spindle_rpm: 12000.,
        work_offset: 1,
        tool_number: 1,
        length_register: 1,
        coolant: camlib::motion::Coolant::Off,
        retract_z_mm: None,
        feed_z_mm: None,
    }
}

fn paths() -> Vec<TracePath> {
    vec![
        // Closed square.
        TracePath {
            start_mm: [10., 10.],
            segments: vec![
                TraceSegment::Line { end_mm: [20., 10.] },
                TraceSegment::Line { end_mm: [20., 20.] },
                TraceSegment::Line { end_mm: [10., 20.] },
                TraceSegment::Line { end_mm: [10., 10.] },
            ],
        },
        // Open path: a line then a clockwise half circle.
        TracePath {
            start_mm: [25., 5.],
            segments: vec![
                TraceSegment::Line { end_mm: [30., 5.] },
                TraceSegment::Arc {
                    center_mm: [30., 8.],
                    sweep_rad: -PI,
                },
            ],
        },
    ]
}

fn run(op: TraceOperation) -> (Vec<camlib::motion::Motion>, Stock, TracePlan) {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut plan = TracePlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut motions = vec![];
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        motions.push(motion);
    }
    plan.finish_report(&cancel).unwrap();
    (motions, stock, plan)
}

#[test]
fn trace_follows_paths_exactly_at_each_depth_and_removes_stock() {
    let m = fixture();
    let op = operation(paths());
    let (top, depth) = (op.top_z_mm, op.depth_z_mm);
    let clearance = op.clearance_z_mm;
    let (motions, stock, plan) = run(op);
    let report = plan.report();
    assert_eq!(
        (report.paths, report.segments, report.closed_paths),
        (2, 6, 1)
    );
    assert_eq!(report.depth_passes, 3);
    let cuts: Vec<_> = motions.iter().filter(|m| m.kind == MoveKind::Cut).collect();
    assert_eq!(cuts.len(), 6 * 3);
    assert_eq!(report.cut_segments_emitted, cuts.len());
    // Pass depths: top - 0.6, top - 1.2, then the exact final depth.
    let mut levels: Vec<f64> = cuts.iter().map(|m| m.curve.start()[2]).collect();
    levels.dedup();
    assert_eq!(levels, vec![top - 0.6, top - 1.2, depth]);
    for cut in &cuts {
        assert_eq!(cut.curve.start()[2], cut.curve.end()[2], "cuts are planar");
    }
    // The square closes exactly on its start; the arc end follows from its sweep.
    let square_last = cuts[3].curve.end();
    assert_eq!([square_last[0], square_last[1]], [10., 10.]);
    match &cuts[5].curve {
        Curve::Arc { end, sweep_rad, .. } => {
            assert_eq!(*sweep_rad, -PI);
            assert!(
                (end[0] - 30.).abs() < 1e-12 && (end[1] - 11.).abs() < 1e-12,
                "{end:?}"
            );
        }
        other => panic!("expected an arc, got {other:?}"),
    }
    assert_eq!(motions.last().unwrap().curve.end()[2], clearance);
    assert!(stock.volume().removed_upper_mm3 > 0., "real stock removal");
    for motion in &motions {
        if motion.kind == MoveKind::Rapid {
            assert!(motion.curve.start()[2] >= m.stock.height().unwrap().max_mm[2]);
            assert!(motion.curve.end()[2] >= m.stock.height().unwrap().max_mm[2]);
        }
    }
}

#[test]
fn trace_heights_and_precise_errors() {
    let m = fixture();
    let mut op = operation(paths());
    op.retract_z_mm = Some(m.stock.height().unwrap().max_mm[2] + 2.);
    op.feed_z_mm = Some(m.stock.height().unwrap().max_mm[2] + 0.5);
    let (motions, _, _) = run(op.clone());
    for motion in &motions {
        if motion.kind == MoveKind::Rapid {
            assert!(motion.curve.start()[2] >= m.stock.height().unwrap().max_mm[2] + 0.5);
            assert!(motion.curve.end()[2] >= m.stock.height().unwrap().max_mm[2] + 0.5);
        }
    }
    let cancel = AtomicBool::new(false);
    let error = |op: TraceOperation| {
        TracePlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .err()
        .unwrap()
    };
    let mut bad = op.clone();
    bad.depth_z_mm = bad.top_z_mm;
    assert!(error(bad).contains("depth_z_mm"));
    let mut bad = op.clone();
    bad.paths[1].segments.push(TraceSegment::Arc {
        center_mm: [30., 11.],
        sweep_rad: 1.,
    });
    assert!(error(bad).contains("path 1, segment 2: arc"));
    let mut bad = op.clone();
    bad.paths[0].segments[0] = TraceSegment::Line { end_mm: [10., 10.] };
    assert!(error(bad).contains("segment 0: line"));
    let mut bad = op;
    bad.paths.clear();
    assert!(error(bad).contains("at least one path"));
}
