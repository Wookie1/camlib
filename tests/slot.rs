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

/// One orbit, grouped from the motion stream alone: (centre, first start,
/// last end, swept angle, pieces). A controlled orbit is a D-shaped loop:
/// cut arcs about one centre plus a return across the proven-air stretch
/// (a straight line between two fillet arcs about other centres); a return
/// piece's swept angle is the signed angle its ends make at the centre.
type Orbit = (Vec3, Vec3, Vec3, f64, usize);

#[derive(PartialEq, Debug)]
enum Piece {
    /// The first arc of a new orbit.
    First,
    /// A later arc of the open orbit.
    Arc,
    /// A piece of the open orbit's return (a fillet arc or the line).
    Return,
    Other,
}

#[derive(Default)]
struct OrbitWalk {
    open: Option<Orbit>,
    closed: Vec<Orbit>,
    /// Returns seen (consecutive return pieces count once).
    returns: usize,
    in_return: bool,
}

impl OrbitWalk {
    fn close(&mut self) {
        if let Some(o) = self.open.take() {
            self.closed.push(o);
        }
    }
    fn step(&mut self, m: &Motion) -> Piece {
        let piece = self.classify(m);
        if piece == Piece::Return && !self.in_return {
            self.returns += 1;
        }
        self.in_return = piece == Piece::Return;
        piece
    }
    fn classify(&mut self, m: &Motion) -> Piece {
        let cutting = matches!(m.kind, MoveKind::Cut | MoveKind::Air);
        let (start, end) = (m.curve.start(), m.curve.end());
        let horizontal = start[2] == end[2];
        if let Curve::Arc {
            center, sweep_rad, ..
        } = m.curve
            && cutting
            && let Some(o) = self.open.as_mut()
            && close3(o.0, center)
            && close3(o.2, start)
            && (o.3 + sweep_rad).abs() <= TAU + 1e-9
        {
            o.2 = end;
            o.3 += sweep_rad;
            o.4 += 1;
            return Piece::Arc;
        }
        if matches!(m.curve, Curve::Arc { .. } | Curve::Line { .. })
            && cutting
            && horizontal
            && let Some(o) = self.open.as_mut()
            && close3(o.2, start)
            && o.3.abs() < TAU - 1e-9
            && (!matches!(m.curve, Curve::Arc { center, .. } if close3(o.0, center)))
        {
            let angle = |q: Vec3| (q[1] - o.0[1]).atan2(q[0] - o.0[0]);
            let mut d = angle(end) - angle(start);
            if d > std::f64::consts::PI {
                d -= TAU;
            } else if d <= -std::f64::consts::PI {
                d += TAU;
            }
            o.2 = end;
            o.3 += d;
            o.4 += 1;
            return Piece::Return;
        }
        self.close();
        if let Curve::Arc {
            center, sweep_rad, ..
        } = m.curve
            && cutting
        {
            self.open = Some((center, start, end, sweep_rad, 1));
            return Piece::First;
        }
        Piece::Other
    }
    fn finish(mut self) -> Vec<Orbit> {
        self.close();
        self.closed
    }
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
        op.start_mm = Some(start);
        op.end_mm = Some(end);
        let (sa, sb) = (start, end);
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
        let mut walk = OrbitWalk::default();
        let mut prior_circle: Option<Vec3> = None;
        while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(m.curve.start(), last_position);
            last_position = m.curve.end();
            let piece = walk.step(&m);
            match m.curve {
                Curve::Arc {
                    center, sweep_rad, ..
                } if piece == Piece::Arc => {
                    // A later cut arc of a D-shaped loop: same centre, same
                    // sense, cutting feed.
                    assert_eq!(Some(center), previous_circle);
                    assert_eq!(sweep_rad.signum(), if cw { -1. } else { 1. });
                    assert_eq!(m.feed_mm_min, Some(op.feed_mm_min));
                }
                _ if piece == Piece::Return => {
                    // The return runs inside the disk the previous orbit
                    // cleared (sampled): it exposes no edge at all.
                    let old = prior_circle.expect("a return needs a previous orbit");
                    let rho = p.report().orbit_radius_mm;
                    let prepared = m.curve.prepare(1e-7).unwrap();
                    for i in 0..=256 {
                        let q = prepared.position(i as f64 / 256.).unwrap();
                        assert!(distance(old, q) <= rho - 16. * op.tolerance_mm + 1e-12);
                    }
                    assert_eq!(m.kind, MoveKind::Cut);
                    assert_eq!(m.feed_mm_min, Some(op.feed_mm_min));
                }
                Curve::Arc {
                    center,
                    start,
                    sweep_rad,
                    ..
                } => {
                    assert_eq!(piece, Piece::First);
                    let orbit = distance(center, start);
                    let cleared = orbit + r;
                    assert!(axis_distance(center, sa, sb) < 1e-10);
                    // Every point of the disk sweep lies within the nominal capsule.
                    assert!(orbit + r <= width / 2.);
                    assert_eq!(sweep_rad.signum(), if cw { -1. } else { 1. });
                    let direction = [end[0] - sa[0], end[1] - sa[1]];
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
                        // Continuous full-orbit maximum: exact for a full
                        // circle, an upper bound for a D-shaped loop.
                        let q = gap + orbit;
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
                    prior_circle = previous_circle;
                    previous_circle = Some(center);
                    circles += 1;
                }
                Curve::Line { start, end } if m.kind == MoveKind::Link => {
                    // Distance-to-convex-segment is convex along a line, so both
                    // endpoint distances bound the complete cutter envelope.
                    assert!(axis_distance(start, sa, sb) + r <= width / 2.);
                    assert!(axis_distance(end, sa, sb) + r <= width / 2.);
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
        // Every orbit, D-shaped or not, sweeps exactly one turn and closes.
        let chords = walk.returns;
        let orbits = walk.finish();
        assert_eq!(orbits.len(), circles);
        for (_, start, end, sweep, _) in &orbits {
            assert!((sweep.abs() - TAU).abs() < 1e-9, "orbit sweep {sweep}");
            assert!(close3(*start, *end));
        }
        // Every controlled orbit after the seed is a D-shaped loop.
        assert_eq!(chords, circles - 1);
        assert_eq!(p.report().return_chords, chords);
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
        op.end_mm = Some([12., 12.]);
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
            6 => op.end_mm = Some([f64::NAN, 12.]),
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
            // Straight plunges start at the feed height; the helix turns
            // below them continue from where the plunge ended.
            MoveKind::Plunge if matches!(motion.curve, Curve::Line { .. }) => {
                assert_eq!(a[2], feed)
            }
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

// ---- Curved slots (#9) ----

/// Independent centreline model: (a, b, bulge) pieces; the arc centre and
/// radius come from the sagitta relation, not from the planner.
fn path_distance(p: [f64; 2], path: &[[f64; 3]]) -> f64 {
    let mut best = f64::INFINITY;
    for w in path.windows(2) {
        let (a, b, bulge) = ([w[0][0], w[0][1]], [w[1][0], w[1][1]], w[0][2]);
        if bulge == 0. {
            best = best.min(axis_distance([p[0], p[1], 0.], a, b));
            continue;
        }
        let chord = (b[0] - a[0]).hypot(b[1] - a[1]);
        let sagitta = bulge.abs() * chord / 2.;
        let radius = (chord * chord / 4. + sagitta * sagitta) / (2. * sagitta);
        // Centre on the bisector, |radius - sagitta| from the chord, on the
        // side away from the bulge's arc.
        let mid = [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.];
        let left = [-(b[1] - a[1]) / chord, (b[0] - a[0]) / chord];
        let offset = (radius - sagitta) * bulge.signum();
        let c = [mid[0] + left[0] * offset, mid[1] + left[1] * offset];
        let angle = |q: [f64; 2]| (q[1] - c[1]).atan2(q[0] - c[0]);
        let (a0, ap, sweep) = (angle(a), angle(p), 4. * bulge.atan());
        let rel = if sweep > 0. {
            (ap - a0).rem_euclid(TAU)
        } else {
            (a0 - ap).rem_euclid(TAU)
        };
        if rel <= sweep.abs() {
            best = best.min(((p[0] - c[0]).hypot(p[1] - c[1]) - radius).abs());
        } else {
            best = best
                .min((p[0] - a[0]).hypot(p[1] - a[1]))
                .min((p[0] - b[0]).hypot(p[1] - b[1]));
        }
    }
    best
}

fn curved(path: Vec<[f64; 3]>, clockwise: bool, angle: f64) -> TrochoidalSlotOperation {
    let mut op = operation();
    op.start_mm = None;
    op.end_mm = None;
    op.path_mm = Some(path);
    op.clockwise = clockwise;
    op.max_engagement_deg = angle;
    op.floor_z_mm = 9.;
    op
}

/// Plans the slot against real stock and checks, independently of the
/// planner: continuity, every orbit centred on the centreline, every cutter
/// position inside the slot, the exposed edge angle against the previous
/// orbit's disk over the whole of each link and orbit, and the final stock
/// (cut to the floor inside, untouched outside).
fn check_curved(op: &TrochoidalSlotOperation) -> SlotPlan {
    let cancel = AtomicBool::new(false);
    let path = op.path_mm.clone().unwrap();
    let (width, angle, r) = (op.width_mm, op.max_engagement_deg, 2.);
    let mut p = plan(op.clone());
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let mut last = fixture().initial_position_mm;
    let mut previous: Option<Vec3> = None;
    let mut uncertainty = 0f64;
    let mut orbits = 0;
    let mut walk = OrbitWalk::default();
    let mut prior: Option<Vec3> = None;
    let rho = p.report().orbit_radius_mm;
    let cleared = rho + r;
    while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
        assert_eq!(m.curve.start(), last);
        last = m.curve.end();
        let prepared = m.curve.prepare(1e-5).unwrap();
        let samples: Vec<Vec3> = (0..=512)
            .map(|i| prepared.position(i as f64 / 512.).unwrap())
            .collect();
        let below_top = m.curve.start()[2] < 10. || m.curve.end()[2] < 10.;
        if below_top {
            for q in &samples {
                assert!(
                    path_distance([q[0], q[1]], &path) + r <= width / 2. + 1e-9,
                    "cutter leaves the slot at {q:?} ({:?})",
                    m.kind
                );
            }
        }
        let piece = walk.step(&m);
        match m.curve {
            Curve::Arc {
                center, sweep_rad, ..
            } if piece == Piece::First => {
                assert!(path_distance([center[0], center[1]], &path) < 1e-9);
                assert_eq!(sweep_rad.signum(), if op.clockwise { -1. } else { 1. });
                if let Some(old) = previous {
                    // The farthest point of a full orbit is opposite: exact
                    // for a circle, an upper bound for a D-shaped loop.
                    let q = distance(old, center) + rho;
                    assert!(exposed_angle(q, r, cleared) <= angle + 1e-8);
                }
                prior = previous;
                previous = Some(center);
                orbits += 1;
            }
            _ if piece == Piece::Return => {
                // The return stays in the previous orbit's cleared disk,
                // sampled independently of the planner's proof.
                let old = prior.expect("a return needs a previous orbit");
                for q in &samples {
                    assert!(distance(old, *q) <= rho - 16. * op.tolerance_mm + 1e-12);
                }
            }
            _ if m.kind == MoveKind::Link => {
                let old = previous.unwrap();
                // Dense samples plus the sampling gap (the curve moves at
                // most length/512 between samples).
                let gap = prepared.length_mm() / 512.;
                let q = samples.iter().map(|q| distance(old, *q)).fold(0., f64::max) + gap;
                assert!(
                    exposed_angle(q, r, cleared) <= angle + 1e-6,
                    "link exposes {} deg",
                    exposed_angle(q, r, cleared)
                );
            }
            _ => {}
        }
        uncertainty = uncertainty.max(commit(&mut stock, &m));
    }
    assert!(orbits > 4);
    let walked = walk.finish();
    assert_eq!(walked.len(), orbits);
    for (_, start, end, sweep, _) in &walked {
        assert!((sweep.abs() - TAU).abs() < 1e-9, "orbit sweep {sweep}");
        assert!(close3(*start, *end));
    }
    assert!(p.report().return_chords > 0);
    assert!(p.report().max_bounded_engagement_deg <= angle);
    p.finish_report(&cancel).unwrap();
    let cell_guard = stock.spec().cell_size_mm * 2.;
    for i in 0..stock.cells().len() {
        let q = stock.cell_center(i).unwrap();
        let d = path_distance([q[0], q[1]], &path);
        let h = stock.cells()[i];
        if d < width / 2. - cell_guard {
            assert!(
                h.upper_mm <= 9. + uncertainty + 1e-9,
                "interior {q:?}: {h:?}"
            );
        }
        if d > width / 2. + cell_guard {
            assert_eq!(h.lower_mm, 10., "protected exterior {q:?}");
        }
    }
    p
}

#[test]
fn arc_slot_bounds_engagement_and_covers_the_width_in_both_senses() {
    // A half-turn arc of radius 12: the links run outside it (radius 13) for
    // clockwise orbits and inside it (radius 11) for counter-clockwise.
    for cw in [true, false] {
        let p = check_curved(&curved(vec![[8., 6., -1.], [32., 6., 0.]], cw, 90.));
        assert_eq!(p.report().path_pieces, 1);
        assert_eq!(p.report().corner_links, 0);
    }
}

#[test]
fn s_slot_bounds_engagement_through_the_inflection() {
    // Two radius-7 half turns, tangent where they meet: no corner link.
    for (cw, angle) in [(true, 60.), (false, 120.)] {
        let p = check_curved(&curved(
            vec![[6., 12., 1.], [20., 12., -1.], [34., 12., 0.]],
            cw,
            angle,
        ));
        assert_eq!(p.report().path_pieces, 2);
        assert_eq!(p.report().corner_links, 0);
    }
}

#[test]
fn corner_slot_links_inside_the_vertex_orbit() {
    // A left turn: clockwise orbits link on the left, the inside of the turn
    // (a chord); counter-clockwise ones on the right, the outside (an arc).
    for cw in [true, false] {
        let p = check_curved(&curved(
            vec![[8., 6., 0.], [26., 6., 0.], [26., 20., 0.]],
            cw,
            90.,
        ));
        assert_eq!(p.report().corner_links, 1);
        assert_eq!(p.report().corner_arcs, if cw { 0 } else { 1 });
    }
}

/// Unit XY direction of a curve at its start (`at_end` false) or end.
fn heading(curve: &Curve, at_end: bool) -> [f64; 2] {
    let (a, b) = match *curve {
        Curve::Line { start, end } => ([start[0], start[1]], [end[0], end[1]]),
        Curve::Arc {
            start,
            end,
            center,
            sweep_rad,
            ..
        } => {
            let p = if at_end { end } else { start };
            let (rx, ry) = (p[0] - center[0], p[1] - center[1]);
            let s = sweep_rad.signum();
            ([0., 0.], [-s * ry, s * rx])
        }
        _ => panic!("unexpected curve {curve:?}"),
    };
    let n = (b[0] - a[0]).hypot(b[1] - a[1]);
    [(b[0] - a[0]) / n, (b[1] - a[1]) / n]
}

/// At an outer corner the link is an arc of the vertex orbit's own circle in
/// the orbit's sense, tangent to the orbit it leaves and to the link that
/// follows, checked from the motion stream alone, in both turn directions.
#[test]
fn outer_corner_links_are_tangent_arcs_on_the_vertex_orbit() {
    let cancel = AtomicBool::new(false);
    // (path, clockwise, vertex, turn): each has its links on the outside.
    for (path, cw, vertex, turn) in [
        (
            vec![[8., 6., 0.], [26., 6., 0.], [26., 20., 0.]],
            false,
            [26., 6.],
            std::f64::consts::FRAC_PI_2,
        ),
        (
            vec![[8., 20., 0.], [26., 20., 0.], [26., 6., 0.]],
            true,
            [26., 20.],
            -std::f64::consts::FRAC_PI_2,
        ),
        // A 135 degree turn.
        (
            vec![[8., 6., 0.], [26., 6., 0.], [16., 16., 0.]],
            false,
            [26., 6.],
            3. * std::f64::consts::FRAC_PI_4,
        ),
    ] {
        let op = curved(path, cw, 90.);
        let mut p = plan(op.clone());
        let rho = p.report().orbit_radius_mm;
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let mut motions = Vec::new();
        while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            commit(&mut stock, &m);
            motions.push(m);
        }
        let mut arcs = 0;
        for (i, m) in motions.iter().enumerate() {
            let Curve::Arc {
                start,
                end,
                center,
                sweep_rad,
                ..
            } = m.curve
            else {
                continue;
            };
            if m.kind != MoveKind::Link || !close3(center, [vertex[0], vertex[1], center[2]]) {
                continue;
            }
            assert!((distance(center, start) - rho).abs() < 1e-9);
            assert!((distance(center, end) - rho).abs() < 1e-9);
            assert!((sweep_rad - turn).abs() < 1e-9, "sweep {sweep_rad}");
            // Tangent to the vertex orbit before it and the link after it.
            let before = heading(&motions[i - 1].curve, true);
            let after = heading(&motions[i + 1].curve, false);
            let (h0, h1) = (heading(&m.curve, false), heading(&m.curve, true));
            assert!((h0[0] - before[0]).hypot(h0[1] - before[1]) < 1e-9);
            assert!((h1[0] - after[0]).hypot(h1[1] - after[1]) < 1e-9);
            arcs += 1;
        }
        // One per depth pass.
        assert_eq!(arcs, p.report().depth_passes);
        assert_eq!(p.report().corner_arcs, arcs);
    }
}

#[test]
fn oblique_outer_corner_has_reproducible_sweep() {
    let op = curved(
        vec![[8., 6., 0.], [26., 6., 0.], [18., 13., 0.]],
        false,
        90.,
    );
    let mut plan = plan(op);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let cancel = AtomicBool::new(false);
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        if motion.kind == MoveKind::Link
            && let Curve::Arc {
                center, sweep_rad, ..
            } = motion.curve
            && center[..2] == [26., 6.]
        {
            // Signed angle between the two rounded offset radii at this vertex.
            assert_eq!(sweep_rad, 2.4227626539681677);
            return;
        }
        commit(&mut stock, &motion);
    }
    panic!("missing outer corner arc");
}

/// The stock-clear fast path also runs on an arc piece: one link along the
/// rest of the arc when the stock is already clear there.
#[test]
fn arc_slot_stock_clear_corridor_links_the_rest_of_the_arc() {
    for cw in [true, false] {
        let op = curved(vec![[8., 6., -1.], [32., 6., 0.]], cw, 90.);
        let mut snapshot = Stock::new(fixture().stock.height().unwrap().clone())
            .unwrap()
            .snapshot();
        snapshot.cells.fill(HeightBounds {
            lower_mm: 8.,
            upper_mm: 8.,
        });
        let mut stock = Stock::from_snapshot(snapshot).unwrap();
        let mut p = plan(op.clone());
        let mut motions = 0;
        while let Some(m) = p
            .next_motion_with_stock(&stock, &AtomicBool::new(false))
            .unwrap()
        {
            commit(&mut stock, &m);
            motions += 1;
        }
        assert_eq!(p.report().stock_clear_advances, p.report().depth_passes);
        assert_eq!(p.report().controlled_orbits, p.report().depth_passes);
        assert!(motions < 20, "{motions} motions");
        // Stock left on the arc: no stock-clear advance, the full slot is cut.
        let fresh = check_curved(&op);
        assert_eq!(fresh.report().stock_clear_advances, 0);
    }
}

/// One occupied cell near the far end of a cleared arc defeats the
/// stock-clear query from every station, and each station rescans nearly the
/// same arc. The operation's query work is one shared budget: once it is
/// spent the plan stops querying and keeps its bounded orbits, still cutting
/// the cell.
#[test]
fn arc_stock_clear_queries_share_one_work_budget() {
    // `STOCK_QUERY_WORK` in src/slot.rs.
    const BUDGET: usize = 1_000_000;
    let cancel = AtomicBool::new(false);
    let m = fixture();
    let mut spec = m.stock.height().unwrap().clone();
    spec.cell_size_mm = 0.0625;
    let op = curved(vec![[8., 6., -1.], [32., 6., 0.]], true, 90.);
    let mut p = SlotPlan::prepare(
        op.clone(),
        &m.tools[0],
        &spec,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut snapshot = Stock::new(spec).unwrap().snapshot();
    snapshot.cells.fill(HeightBounds {
        lower_mm: 8.,
        upper_mm: 8.,
    });
    let stock = Stock::from_snapshot(snapshot).unwrap();
    // The cell on the centreline just before the far end point (32, 6).
    let target = [31.97, 6.03];
    let cell = (0..stock.cells().len())
        .min_by(|&a, &b| {
            let d = |i: usize| {
                let c = stock.cell_center(i).unwrap();
                (c[0] - target[0]).hypot(c[1] - target[1])
            };
            d(a).total_cmp(&d(b))
        })
        .unwrap();
    let mut snapshot = stock.snapshot();
    snapshot.cells[cell] = HeightBounds {
        lower_mm: 10.,
        upper_mm: 10.,
    };
    let mut stock = Stock::from_snapshot(snapshot).unwrap();
    let mut uncertainty = 0f64;
    while let Some(motion) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
        uncertainty = uncertainty.max(commit_measured(&mut stock, &motion).1);
    }
    let report = p.report();
    assert!(
        report.stock_query_work <= BUDGET,
        "query work {} exceeds the shared budget",
        report.stock_query_work
    );
    assert!(report.stock_query_work_exhausted);
    assert!(report.controlled_orbits > 100, "{}", report.controlled_orbits);
    // The occupied cell is still cut to the floor (within the stock's own
    // height uncertainty, as the slot's interior checks allow).
    let h = stock.cells()[cell];
    assert!(
        h.upper_mm <= op.floor_z_mm + uncertainty + 1e-9,
        "{h:?}, uncertainty {uncertainty}"
    );
}

#[test]
fn bends_tighter_than_the_orbit_are_refused_with_the_radius() {
    // Radius 0.9 bending toward the clockwise orbits' side; the orbit
    // radius is about width/2 - r = 1.
    let op = curved(vec![[10., 12., 1.], [11.8, 12., 0.]], true, 90.);
    let m = fixture();
    let error = SlotPlan::prepare(
        op.clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .err()
    .unwrap();
    assert!(
        error.contains("bends toward the orbit side with radius 0.9000"),
        "{error}"
    );
    // The same bend away from the orbit side plans.
    let mut away = op;
    away.clockwise = false;
    assert!(
        SlotPlan::prepare(
            away,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false),
        )
        .is_ok()
    );
}

#[test]
fn path_and_straight_endpoints_are_exclusive_and_validated() {
    let m = fixture();
    let prepare = |op: TrochoidalSlotOperation| {
        SlotPlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false),
        )
    };
    let mut both = operation();
    both.path_mm = Some(vec![[8., 12., 0.], [28., 12., 0.]]);
    assert!(
        prepare(both)
            .err()
            .unwrap()
            .contains("either start_mm and end_mm")
    );
    for path in [
        vec![[8., 12., 0.]],
        vec![[8., 12., 0.], [28., 12., 0.5]],
        vec![[8., 12., 1.5], [28., 12., 0.]],
        vec![[8., 12., 0.], [8., 12., 0.]],
    ] {
        assert!(
            prepare(curved(path.clone(), true, 90.)).is_err(),
            "{path:?}"
        );
    }
    // A one-line path is the straight slot, motion for motion.
    let cancel = AtomicBool::new(false);
    let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut straight = plan(operation());
    let mut line = operation();
    line.path_mm = Some(vec![[8., 12., 0.], [28., 12., 0.]]);
    line.start_mm = None;
    line.end_mm = None;
    let mut line = plan(line);
    let mut stock_a = stock.clone();
    while let Some(a) = straight.next_motion_with_stock(&stock_a, &cancel).unwrap() {
        let b = line
            .next_motion_with_stock(&stock_a, &cancel)
            .unwrap()
            .unwrap();
        assert_eq!(a, b);
        commit(&mut stock_a, &a);
    }
    assert!(
        line.next_motion_with_stock(&stock_a, &cancel)
            .unwrap()
            .is_none()
    );
}

// ---- Air feed on the proven-air part of each controlled orbit (#34) ----

fn close3(a: Vec3, b: Vec3) -> bool {
    (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]) < 1e-9
}

/// Commits the motion and returns (removed volume measured by the stock,
/// height uncertainty). The volume is the stock's own cell-bound record of
/// the commit, independent of the planner.
fn commit_measured(stock: &mut Stock, m: &Motion) -> (f64, f64) {
    let t = fixture().tools[0].prepare().unwrap();
    let d = stock
        .preview_sweep(&t, m, 1., SweepOptions::default(), &AtomicBool::new(false))
        .unwrap();
    let size = stock.spec().cell_size_mm;
    let removed = d
        .changes()
        .iter()
        .map(|c| (c.before.upper_mm - c.after.upper_mm).max(0.))
        .sum::<f64>()
        * size
        * size;
    stock.apply_delta(&d).unwrap();
    (
        removed,
        d.stats().max_path_error_mm + d.stats().numerical_guard_mm,
    )
}

/// Drives the plan to the end, committing every motion to `stock`. Returns
/// the motions, the volume each commit removed, the finished plan and the
/// accumulated height uncertainty.
fn drive(
    op: &TrochoidalSlotOperation,
    stock: &mut Stock,
) -> (Vec<Motion>, Vec<f64>, SlotPlan, f64) {
    let cancel = AtomicBool::new(false);
    let mut p = plan(op.clone());
    let mut motions = Vec::new();
    let mut removed = Vec::new();
    let mut uncertainty = 0f64;
    let mut last = fixture().initial_position_mm;
    while let Some(m) = p.next_motion_with_stock(stock, &cancel).unwrap() {
        assert_eq!(m.curve.start(), last);
        last = m.curve.end();
        let (volume, height) = commit_measured(stock, &m);
        uncertainty = uncertainty.max(height);
        motions.push(m);
        removed.push(volume);
    }
    p.finish_report(&cancel).unwrap();
    (motions, removed, p, uncertainty)
}

/// The straight fixture and the curved/S/corner slots, both senses.
fn air_feed_cases() -> Vec<(&'static str, TrochoidalSlotOperation)> {
    let mut cases = Vec::new();
    for cw in [true, false] {
        let mut straight = operation();
        straight.clockwise = cw;
        cases.push(("straight", straight));
        cases.push(("arc", curved(vec![[8., 6., -1.], [32., 6., 0.]], cw, 90.)));
        cases.push((
            "s",
            curved(
                vec![[6., 12., 1.], [20., 12., -1.], [34., 12., 0.]],
                cw,
                90.,
            ),
        ));
        cases.push((
            "corner",
            curved(vec![[8., 6., 0.], [26., 6., 0.], [26., 20., 0.]], cw, 90.),
        ));
    }
    cases
}

/// Total swept length of the motions fed at `feed`.
fn fed_length(motions: &[Motion], feed: f64) -> f64 {
    motions
        .iter()
        .filter(|m| m.feed_mm_min == Some(feed))
        .map(|m| m.curve.prepare(1e-5).unwrap().length_mm())
        .sum()
}

#[test]
fn oblique_slot_return_has_reproducible_fillet_geometry() {
    let m = fixture();
    let mut op = operation();
    op.start_mm = Some([0., 0.]);
    op.end_mm = Some([17., 27.]);
    op.floor_z_mm = 9.;
    let mut spec = m.stock.height().unwrap().clone();
    spec.min_mm[..2].copy_from_slice(&[-10., -10.]);
    let cancel = AtomicBool::new(false);
    let mut plan =
        SlotPlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap();
    let mut stock = Stock::new(spec).unwrap();
    // The fillet is the 135th motion after the first pass's seed orbit (the
    // first full-turn cut arc), whatever entry comes before that orbit.
    let mut fillet = None;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        if fillet.is_none()
            && motion.kind == MoveKind::Cut
            && matches!(motion.curve, Curve::Arc { sweep_rad, .. } if sweep_rad.abs() == TAU)
        {
            fillet = Some(motion.id + 135);
        }
        if Some(motion.id) == fillet {
            let Curve::Arc { center, end, .. } = motion.curve else {
                panic!("expected return fillet")
            };
            // Reference fillet from the preceding orbit, evaluating atan2 and sin
            // at 90 decimal digits and rounding each result to binary64.
            assert_eq!(center[1], 3.938138415666419);
            assert_eq!(end[1], 3.8535286713828856);
            return;
        }
        commit(&mut stock, &motion);
    }
    panic!("missing return fillet");
}

#[test]
fn slot_air_feed_arcs_remove_no_stock() {
    for (name, base) in air_feed_cases() {
        let mut with_air = base.clone();
        with_air.air_feed_mm_min = Some(base.feed_mm_min * 4.);
        let air_feed = with_air.air_feed_mm_min.unwrap();
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let (motions, removed, p, _) = drive(&with_air, &mut stock);
        let air_arcs = motions
            .iter()
            .filter(|m| m.kind == MoveKind::Air && matches!(m.curve, Curve::Arc { .. }))
            .count();
        assert!(air_arcs > 0, "{name}: no air-fed arcs");
        assert_eq!(p.report().air_fed_arcs, air_arcs);
        assert!(p.report().air_fed_mm > 0., "{name}: no air-fed length");
        for (m, removed) in motions.iter().zip(&removed) {
            if m.feed_mm_min == Some(air_feed) {
                assert_eq!(
                    *removed, 0.,
                    "{name}: air-fed motion {} removed {removed} mm3",
                    m.id
                );
            }
        }
        // The same job without the air feed, for the cut-feed length.
        let mut fresh = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let (none_motions, _, _, _) = drive(&base, &mut fresh);
        let cut_air = fed_length(&motions, base.feed_mm_min);
        let cut_none = fed_length(&none_motions, base.feed_mm_min);
        assert!(
            cut_air <= 0.65 * cut_none,
            "{name}: cut-feed length {cut_air:.3} is not at least 35% below {cut_none:.3}"
        );
    }
    // The stock-clear corridor scene: both controlled orbits follow a
    // stock-clear advance, so no orbit has the adjacent previous orbit the
    // proof needs and none is split.
    let mut op = operation();
    op.air_feed_mm_min = Some(op.feed_mm_min * 4.);
    let mut snapshot = Stock::new(fixture().stock.height().unwrap().clone())
        .unwrap()
        .snapshot();
    snapshot.cells.fill(HeightBounds {
        lower_mm: 8.,
        upper_mm: 8.,
    });
    let mut stock = Stock::from_snapshot(snapshot).unwrap();
    let (_, _, p, _) = drive(&op, &mut stock);
    assert_eq!(p.report().stock_clear_advances, 2);
    assert_eq!(p.report().air_fed_arcs, 0);
    // A piece so short that the stock-clear jump lands within one orbit
    // diameter of the previous orbit: the proof's predecessor is still not
    // the adjacent orbit the advance link implies, so the next orbit stays
    // one full circle.
    let mut short = operation();
    short.end_mm = Some([9.5, 12.]);
    short.air_feed_mm_min = Some(short.feed_mm_min * 4.);
    let mut snapshot = Stock::new(fixture().stock.height().unwrap().clone())
        .unwrap()
        .snapshot();
    snapshot.cells.fill(HeightBounds {
        lower_mm: 8.,
        upper_mm: 8.,
    });
    let mut stock = Stock::from_snapshot(snapshot).unwrap();
    let (_, _, p, _) = drive(&short, &mut stock);
    assert_eq!(p.report().stock_clear_advances, 2);
    assert_eq!(p.report().air_fed_arcs, 0);
}

#[test]
fn slot_without_air_feed_return_chords_cut_at_feed() {
    let cancel = AtomicBool::new(false);
    let mut op = operation();
    op.air_feed_mm_min = None;
    let mut p = plan(op.clone());
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let mut moves = Vec::new();
    while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
        match m.feed_mm_min {
            None => assert_eq!(m.kind, MoveKind::Rapid, "{m:?}"),
            Some(feed) => assert!(
                feed == op.feed_mm_min || feed == op.entry_feed_mm_min,
                "{m:?}"
            ),
        }
        assert_ne!(m.kind, MoveKind::Air, "{m:?}");
        moves.push(m.clone());
        commit(&mut stock, &m);
    }
    // Without an air feed the return chords are cut moves at the cutting
    // feed, and every orbit still sweeps one turn.
    let orbits = orbit_groups(&moves);
    assert_eq!(
        orbits.len(),
        p.report().seed_orbits + p.report().controlled_orbits
    );
    for (_, start, end, sweep, _) in &orbits {
        assert!((sweep.abs() - TAU).abs() < 1e-9, "orbit sweep {sweep}");
        assert!(close3(*start, *end));
    }
    assert_eq!(p.report().air_fed_arcs, 0);
    assert!(p.report().return_chords > 0);
}

#[test]
fn air_returns_too_short_to_round_report_no_chord() {
    // A narrow slot and a fine tolerance: every proven-air stretch is shorter
    // than 64 guards, so each return is the air arc itself. No chord is cut
    // and none may be counted or credited with a saving.
    let cancel = AtomicBool::new(false);
    let mut op = operation();
    op.width_mm = 4.4;
    op.tolerance_mm = 0.001;
    op.boundary_tolerance_mm = 0.05;
    op.max_engagement_deg = 90.;
    op.air_feed_mm_min = Some(op.feed_mm_min * 4.);
    let mut p = plan(op);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let (mut air_arcs, mut air_lines) = (0, 0);
    while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
        if m.kind == MoveKind::Air {
            match m.curve {
                Curve::Arc { .. } => air_arcs += 1,
                _ => air_lines += 1,
            }
        }
        commit(&mut stock, &m);
    }
    assert!(air_arcs > 0, "the case must produce air returns");
    assert_eq!(air_lines, 0);
    assert_eq!(p.report().return_chords, 0);
    assert_eq!(p.report().return_chord_saved_mm, 0.);
}

/// One record per orbit (see `OrbitWalk`).
fn orbit_groups(motions: &[Motion]) -> Vec<Orbit> {
    let mut walk = OrbitWalk::default();
    for m in motions {
        walk.step(m);
    }
    walk.finish()
}

#[test]
fn slot_air_feed_same_path() {
    for (name, base) in air_feed_cases() {
        let mut with_air = base.clone();
        with_air.air_feed_mm_min = Some(base.feed_mm_min * 4.);
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let (air_motions, _, air_plan, uncertainty) = drive(&with_air, &mut stock);
        assert!(air_plan.report().air_fed_arcs > 0, "{name}");
        let mut fresh = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let (none_motions, _, _, _) = drive(&base, &mut fresh);
        // The swept path is the same with and without the air feed: the same
        // orbits, same centres, same start and end points, each orbit's
        // pieces summing to one full turn and closing at its start.
        let air_orbits = orbit_groups(&air_motions);
        let none_orbits = orbit_groups(&none_motions);
        assert_eq!(air_orbits.len(), none_orbits.len(), "{name}");
        let turn = if base.clockwise { -TAU } else { TAU };
        let mut split = 0;
        for (air, none) in air_orbits.iter().zip(&none_orbits) {
            let (center, start, end, sweep, pieces) = air;
            assert!(close3(*center, none.0), "{name}: orbit centre moved");
            assert!(close3(*start, none.1), "{name}: orbit start moved");
            assert!(close3(*end, none.2), "{name}: orbit end moved");
            assert!(
                (sweep - turn).abs() < 1e-9,
                "{name}: orbit sweep {sweep} is not one turn"
            );
            assert!(close3(*start, *end), "{name}: orbit does not close");
            if *pieces > 1 {
                split += 1;
            }
        }
        assert!(split > 0, "{name}: no orbit was split into air-fed pieces");
        // Engagement and coverage (the existing main test's asserts) with
        // air feed on: every cutter position inside the slot, every orbit
        // centred on the centreline and inside the width, the exposed edge
        // angle against the previous orbit's disk, side-band coverage
        // between neighbouring disks on lines, and the final stock (interior
        // to the floor, exterior untouched).
        let r = 2.;
        let rho = air_plan.report().orbit_radius_mm;
        let cleared = rho + r;
        let path = base.path_mm.clone();
        let mut previous: Option<Vec3> = None;
        let mut walk = OrbitWalk::default();
        for m in &air_motions {
            let piece = walk.step(m);
            let prepared = m.curve.prepare(1e-5).unwrap();
            if m.curve.start()[2] < 10. || m.curve.end()[2] < 10. {
                for i in 0..=512 {
                    let q = prepared.position(i as f64 / 512.).unwrap();
                    let d = match &path {
                        Some(p) => path_distance([q[0], q[1]], p),
                        None => axis_distance(q, base.start_mm.unwrap(), base.end_mm.unwrap()),
                    };
                    assert!(
                        d + r <= base.width_mm / 2. + 1e-9,
                        "{name}: cutter leaves the slot at {q:?} ({:?})",
                        m.kind
                    );
                }
            }
            let Curve::Arc { center, .. } = m.curve else {
                continue;
            };
            if !matches!(m.kind, MoveKind::Cut | MoveKind::Air) || piece == Piece::Return {
                // Return fillets turn about their own centres, inside the
                // orbit (checked in the disk test).
                continue;
            }
            let on_path = match &path {
                Some(p) => path_distance([center[0], center[1]], p) < 1e-9,
                None => {
                    axis_distance(center, base.start_mm.unwrap(), base.end_mm.unwrap()) < 1e-10
                }
            };
            assert!(on_path, "{name}: orbit off the centreline");
            assert!(rho + r <= base.width_mm / 2.);
            if let Some(old) = previous.filter(|old| old[2] == center[2]) {
                // The proof needs the previous orbit at the same depth; a
                // new pass starts fresh at its seed orbit.
                let q = distance(old, center) + rho;
                assert!(
                    exposed_angle(q, r, cleared) <= base.max_engagement_deg + 1e-8,
                    "{name}: exposed {} deg",
                    exposed_angle(q, r, cleared)
                );
                if path.is_none() {
                    let gap = distance(old, center);
                    let covered = (cleared * cleared - (gap / 2.).powi(2)).sqrt();
                    assert!(base.width_mm / 2. - covered <= base.boundary_tolerance_mm);
                }
            }
            previous = Some(center);
        }
        let cell_guard = stock.spec().cell_size_mm * 2.;
        for i in 0..stock.cells().len() {
            let q = stock.cell_center(i).unwrap();
            let d = match &path {
                Some(p) => path_distance([q[0], q[1]], p),
                None => {
                    axis_distance([q[0], q[1], 0.], base.start_mm.unwrap(), base.end_mm.unwrap())
                }
            };
            let h = stock.cells()[i];
            if d < base.width_mm / 2. - cell_guard {
                assert!(
                    h.upper_mm <= base.floor_z_mm + uncertainty + 1e-9,
                    "{name}: interior {q:?}: {h:?}"
                );
            }
            if d > base.width_mm / 2. + cell_guard {
                assert_eq!(h.lower_mm, 10., "{name}: protected exterior {q:?}");
            }
        }
    }
}

#[test]
fn slot_air_feed_must_not_be_below_feed() {
    let m = fixture();
    let mut op = operation();
    op.air_feed_mm_min = Some(op.feed_mm_min - 1.);
    let error = SlotPlan::prepare(
        op.clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .err()
    .unwrap();
    assert!(
        error.contains(&format!("air_feed_mm_min {}", op.air_feed_mm_min.unwrap()))
            && error.contains(&format!("feed_mm_min {}", op.feed_mm_min)),
        "{error}"
    );
    let mut op = operation();
    op.air_feed_mm_min = Some(f64::NAN);
    assert!(
        SlotPlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false)
        )
        .is_err()
    );
    // Equal to the cutting feed is allowed.
    let mut op = operation();
    op.air_feed_mm_min = Some(op.feed_mm_min);
    assert!(
        SlotPlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false)
        )
        .is_ok()
    );
}

/// The proof's own bound, checked from the motion stream alone: every
/// return chord of a D-shaped loop must lie inside the disk the previous
/// complete orbit cleared, |P - C_prev| <= rho - 16 * tolerance_mm in XY
/// (the guard g of `air_interval`), sampled along the whole chord, with and
/// without an air feed. The stock grid cannot see this bound: g = 1.6e-4 mm
/// is far below one 0.25 mm cell.
#[test]
fn slot_return_chords_stay_inside_the_previous_orbit_disk() {
    for (name, base) in air_feed_cases() {
        for air_feed in [None, Some(base.feed_mm_min * 4.)] {
            let mut op = base.clone();
            op.air_feed_mm_min = air_feed;
            let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
            let (motions, _, p, _) = drive(&op, &mut stock);
            let rho = p.report().orbit_radius_mm;
            let bound = rho - 16. * base.tolerance_mm;
            // Walk the motion stream only (`OrbitWalk`). When an orbit
            // ends, its centre becomes the proof's predecessor. Forget that
            // centre when z changes (a new depth pass starts fresh at its
            // seed orbit) and when a link longer than the orbit diameter
            // occurs between orbits (a stock-clear jump leaves the previous
            // orbit's disk behind), so a jump never counts as "previous".
            let mut walk = OrbitWalk::default();
            let mut previous: Option<(f64, Vec3)> = None;
            let mut open: Option<Vec3> = None;
            let mut z_tracked: Option<f64> = None;
            let mut checked = 0usize;
            for m in &motions {
                let z0 = m.curve.start()[2];
                let z1 = m.curve.end()[2];
                let piece = walk.step(m);
                match piece {
                    Piece::First => {
                        if let Some(c) = open.take() {
                            previous = Some((c[2], c));
                        }
                        if let Curve::Arc { center, .. } = m.curve {
                            open = Some(center);
                        }
                    }
                    Piece::Arc | Piece::Return => {}
                    Piece::Other => {
                        if let Some(c) = open.take() {
                            previous = Some((c[2], c));
                        }
                        if m.kind == MoveKind::Link
                            && m.curve.prepare(1e-5).unwrap().length_mm() > 2. * rho
                        {
                            previous = None;
                        }
                    }
                }
                if z_tracked != Some(z0) || z1 != z0 {
                    previous = None;
                }
                z_tracked = Some(z1);
                let is_air = m.kind == MoveKind::Air;
                assert_eq!(is_air, air_feed.is_some() && piece == Piece::Return, "{name}: {m:?}");
                if is_air {
                    assert_eq!(m.feed_mm_min, air_feed);
                }
                if piece == Piece::Return {
                    assert!(
                        previous.is_some(),
                        "{name}: chord {} has no remembered previous complete orbit",
                        m.id
                    );
                    let (prev_z, prev_c) = previous.unwrap();
                    assert_eq!(
                        prev_z, z0,
                        "{name}: chord {} runs at z {z0}, the remembered orbit at {prev_z}",
                        m.id
                    );
                    let prepared = m.curve.prepare(1e-5).unwrap();
                    for i in 0..=256 {
                        let q = prepared.position(i as f64 / 256.).unwrap();
                        let d = distance(prev_c, q);
                        assert!(
                            d <= bound + 1e-12,
                            "{name}: chord {} sample {i}/256 exceeds the previous orbit's disk by {} mm (|P - C_prev| = {d}, bound = {bound})",
                            m.id,
                            d - bound
                        );
                    }
                    checked += 1;
                }
            }
            assert!(checked > 0, "{name}: no return was checked");
            assert_eq!(walk.returns, p.report().return_chords, "{name}");
        }
    }
}

#[test]
fn air_intent_requires_committed_clearance_and_never_clears_stock() {
    let cancel = AtomicBool::new(false);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let tool = fixture().tools[0].prepare().unwrap();
    let seed = Motion {
        id: 1,
        operation: "air-proof".into(),
        tool: tool.definition().id.clone(),
        kind: MoveKind::Cut,
        curve: Curve::Arc {
            start: [9., 12., 9.], end: [9., 12., 9.],
            center: [8., 12., 9.], plane: Plane::Xy, sweep_rad: TAU,
        },
        feed_mm_min: Some(600.),
    };
    let mut air = Motion {
        id: 2, kind: MoveKind::Air, feed_mm_min: Some(2400.),
        curve: Curve::Arc {
            start: [8.6, 12., 9.], end: [7.6, 12., 9.],
            center: [8.1, 12., 9.], plane: Plane::Xy, sweep_rad: std::f64::consts::PI,
        },
        ..seed.clone()
    };
    let sweep = SweepOptions::default();
    let before = stock.generation();
    assert!(stock.preview_sweep(&tool, &air, 1., sweep, &cancel).unwrap_err().contains("air motion"));
    assert_eq!(stock.generation(), before);
    commit(&mut stock, &seed);
    let delta = stock.preview_sweep(&tool, &air, 1., sweep, &cancel).unwrap();
    assert!(delta.changes().is_empty());
    stock.apply_delta(&delta).unwrap();
    // An in-plane displacement reaching beyond the cleared disk must refuse,
    // even if its ordinary cutting sweep would be valid.
    air.curve = Curve::Arc {
        start: [10., 12., 9.], end: [8., 12., 9.],
        center: [9., 12., 9.], plane: Plane::Xy, sweep_rad: std::f64::consts::PI,
    };
    let before = stock.generation();
    assert!(stock.preview_sweep(&tool, &air, 1., sweep, &cancel).unwrap_err().contains("air motion"));
    assert_eq!(stock.generation(), before);
    air.kind = MoveKind::Cut;
    assert!(stock.preview_sweep(&tool, &air, 1., sweep, &cancel).is_ok());
    // Exact horizontal floor contact is allowed; going even slightly below it is not.
    air.kind = MoveKind::Air;
    air.curve = Curve::Line { start: [8., 12., 8.999999], end: [8.1, 12., 8.999999] };
    assert!(stock.preview_sweep(&tool, &air, 1., sweep, &cancel).is_err());
}

/// Unit XY direction of travel of a line or arc at its start or end.
fn travel(curve: &Curve, at_end: bool) -> [f64; 2] {
    let v = match *curve {
        Curve::Line { start, end } => [end[0] - start[0], end[1] - start[1]],
        Curve::Arc {
            start,
            end,
            center,
            sweep_rad,
            ..
        } => {
            let p = if at_end { end } else { start };
            let s = sweep_rad.signum();
            [-s * (p[1] - center[1]), s * (p[0] - center[0])]
        }
        _ => panic!("unexpected curve {curve:?}"),
    };
    let n = v[0].hypot(v[1]);
    [v[0] / n, v[1] / n]
}

/// Every controlled orbit runs without a corner: each piece (cut arc,
/// return fillet, return line) leaves in the direction the one before it
/// arrived, checked from the motion stream alone, with and without the air
/// feed, on the straight, arc, S and corner slots.
#[test]
fn d_loop_returns_are_tangent_throughout() {
    for (name, base) in air_feed_cases() {
        for air_feed in [None, Some(base.feed_mm_min * 4.)] {
            let mut op = base.clone();
            op.air_feed_mm_min = air_feed;
            let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
            let (motions, _, p, _) = drive(&op, &mut stock);
            let mut walk = OrbitWalk::default();
            let mut joins = 0;
            let mut previous: Option<&Motion> = None;
            for m in &motions {
                let piece = walk.step(m);
                if matches!(piece, Piece::Arc | Piece::Return) {
                    let before = travel(&previous.unwrap().curve, true);
                    let after = travel(&m.curve, false);
                    let gap = (before[0] - after[0]).hypot(before[1] - after[1]);
                    assert!(gap < 1e-9, "{name}: a {gap} kink before motion {}", m.id);
                    joins += 1;
                }
                previous = Some(m);
            }
            // Each return: fillet, line, fillet, then the cut arc after it.
            assert!(joins >= 4 * p.report().return_chords, "{name}");
            assert!(walk.returns > 0, "{name}");
        }
    }
}
fn helix_entry(angle_deg: f64) -> camlib::pocket::PocketEntry {
    serde_json::from_value(serde_json::json!({
        "kind": "helix",
        "angle_rad": angle_deg.to_radians(),
    }))
    .unwrap()
}

/// Every straight plunge removes no stock (it runs through air), and each
/// depth pass below the stock then descends on whole-turn helices around
/// the first orbit's centre, no steeper than the declared angle, ending at
/// the orbit start at the pass depth.
#[test]
fn slot_plunges_only_through_air_and_helixes_down_at_the_declared_angle() {
    for (name, mut op) in air_feed_cases() {
        let angle = 2.5f64;
        op.entry = Some(helix_entry(angle));
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let (motions, removed, p, _) = drive(&op, &mut stock);
        let rho = p.report().orbit_radius_mm;
        let advance = TAU * rho * angle.to_radians().tan();
        let mut helices = 0;
        let mut plunge_ends = Vec::new();
        for (m, &volume) in motions.iter().zip(&removed) {
            match m.curve {
                Curve::Line { start, end } if m.kind == MoveKind::Plunge => {
                    assert!(volume <= 1e-9, "{name}: plunge to Z{} removed {volume}", end[2]);
                    assert_eq!(m.feed_mm_min, Some(op.entry_feed_mm_min));
                    assert_eq!((start[0], start[1]), (end[0], end[1]));
                    plunge_ends.push(end[2]);
                }
                Curve::Helix {
                    start,
                    end,
                    center,
                    sweep_rad,
                    ..
                } => {
                    assert_eq!(m.kind, MoveKind::Plunge, "{name}");
                    assert_eq!(m.feed_mm_min, Some(op.entry_feed_mm_min));
                    assert_eq!(sweep_rad, if op.clockwise { -TAU } else { TAU });
                    assert!((distance(start, center) - rho).abs() < 1e-9, "{name}");
                    assert_eq!((start[0], start[1]), (end[0], end[1]));
                    let drop = start[2] - end[2];
                    assert!(drop > 0. && drop <= advance + 1e-12, "{name}: {drop} > {advance}");
                    helices += 1;
                }
                _ => assert_ne!(m.kind, MoveKind::Plunge, "{name}"),
            }
        }
        let passes = p.report().depth_passes;
        assert_eq!(p.report().helical_entries, passes, "{name}");
        assert_eq!(p.report().helix_turns, helices, "{name}");
        // The first pass plunges to the stock top; each later one to the
        // depth the previous pass cut at this centre.
        for (k, z) in plunge_ends.iter().enumerate() {
            let expected = op.top_z_mm - k as f64 * op.stepdown_mm;
            assert!((z - expected).abs() < 1e-9, "{name}: pass {k} plunged to {z}");
        }
        assert_eq!(plunge_ends.len(), passes, "{name}");
        // Each helix chain ends where the seed orbit starts, at depth.
        for w in motions.windows(2) {
            if matches!(w[0].curve, Curve::Helix { .. })
                && !matches!(w[1].curve, Curve::Helix { .. })
            {
                let Curve::Arc { sweep_rad, .. } = w[1].curve else {
                    panic!("{name}: a helix is followed by the seed orbit");
                };
                assert_eq!(sweep_rad.abs(), TAU);
            }
        }
        assert!(p.report().seed_entry_policy.contains("helix"));
    }
}

/// No entry declared: the first pass below the stock top is refused
/// before any motion cuts, naming the pass and the remedy.
#[test]
fn slot_without_an_entry_refuses_to_plunge_into_stock() {
    let mut op = operation();
    op.entry = None;
    let mut p = plan(op);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let error = loop {
        match p.next_motion_with_stock(&stock, &AtomicBool::new(false)) {
            Ok(Some(m)) => {
                assert_ne!(m.kind, MoveKind::Plunge);
                commit(&mut stock, &m);
            }
            Ok(None) => panic!("a plunge into stock was planned"),
            Err(e) => break e,
        }
    };
    assert!(error.contains("depth pass 1 would plunge"), "{error}");
    assert!(error.contains("HelicalEntry"), "{error}");
}

/// Over stock a previous cut already cleared (here a wider slot on the
/// same centreline, so the narrower one's disk stays clear of the grid's
/// rim cells), no entry is needed: every pass plunges straight, through
/// air.
#[test]
fn slot_over_cleared_stock_plunges_without_an_entry() {
    // A wider slot first, then the same slot again (its identical rerun: the
    // outer edge of each orbit has no clearance margin over the first cut).
    for first_width in [7.9, operation().width_mm] {
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let mut first = operation();
        first.width_mm = first_width;
        drive(&first, &mut stock);
        let mut op = operation();
        op.entry = None;
        let (motions, removed, p, _) = drive(&op, &mut stock);
        assert_eq!(p.report().helical_entries, 0, "after a {first_width} mm slot");
        assert!(!motions.iter().any(|m| matches!(m.curve, Curve::Helix { .. })));
        for (m, v) in motions.iter().zip(&removed) {
            if m.kind == MoveKind::Plunge {
                assert!(*v <= 1e-9);
            }
        }
        assert!(p.report().seed_entry_policy.contains("else_refused"));
    }
}

/// The straight-plunge proofs of all depth passes share one work budget.
/// Each pass's proof revisits the committed history of the passes before it,
/// so a per-pass allowance grows with the square of the pass count: 133
/// shallow passes on a fresh slot spend about two million visits that way.
/// Shared, the work stays within the budget and every pass still enters on
/// its helix.
#[test]
fn slot_plunge_proofs_share_one_work_budget() {
    // `PLUNGE_PROOF_WORK` in src/slot.rs.
    const BUDGET: usize = 1_000_000;
    let mut op = operation();
    op.floor_z_mm = 6.;
    op.stepdown_mm = 0.03;
    op.entry = Some(helix_entry(3.));
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let (motions, _, p, _) = drive(&op, &mut stock);
    let report = p.report();
    assert!(report.depth_passes > 130, "{}", report.depth_passes);
    assert!(
        report.plunge_proof_work <= BUDGET,
        "plunge-proof work {} exceeds the shared budget",
        report.plunge_proof_work
    );
    assert_eq!(report.helical_entries, report.depth_passes);
    assert!(motions.iter().any(|m| matches!(m.curve, Curve::Helix { .. })));
}

#[test]
fn slot_entry_must_be_a_helix_on_the_orbit_circle_in_the_slot_sense() {
    let m = fixture();
    let refuse = |entry: serde_json::Value, needle: &str| {
        let mut op = operation();
        op.entry = Some(serde_json::from_value(entry).unwrap());
        let error = SlotPlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false),
        )
        .err()
        .expect("refused");
        assert!(error.contains(needle), "{error}");
    };
    refuse(
        serde_json::json!({"kind": "ramp", "angle_rad": 0.05}),
        "HelicalEntry only",
    );
    refuse(
        serde_json::json!({"kind": "stock_edge_or_helix"}),
        "HelicalEntry only",
    );
    refuse(
        serde_json::json!({"kind": "helix", "radius_mm": 0.5}),
        "no radius_mm",
    );
    refuse(
        serde_json::json!({"kind": "helix", "clockwise": false}),
        "no clockwise other",
    );
    refuse(
        serde_json::json!({"kind": "helix", "angle_rad": 0.0}),
        "valid angle",
    );
    // The slot's own sense is accepted, and the helix's own feed is used.
    let mut op = operation();
    op.entry = Some(
        serde_json::from_value(
            serde_json::json!({"kind": "helix", "clockwise": true, "feed_mm_min": 90.0}),
        )
        .unwrap(),
    );
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let (motions, ..) = drive(&op, &mut stock);
    assert!(motions
        .iter()
        .filter(|m| matches!(m.curve, Curve::Helix { .. }))
        .all(|m| m.feed_mm_min == Some(90.)));
}

/// Stock thinner than `tolerance_mm` above the pass is still stock: without
/// an entry the pass is refused, and with one it helixes down from the stock
/// top. Untouched stock at Z10, one pass at Z9.995, tolerance 0.01 mm.
#[test]
fn slot_stock_thinner_than_the_tolerance_is_not_air() {
    let mut op = operation();
    op.floor_z_mm = 9.995;
    op.tolerance_mm = 0.01;
    op.boundary_tolerance_mm = 0.4;
    op.max_engagement_deg = 150.;
    op.entry = None;
    let mut p = plan(op.clone());
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let error = loop {
        match p.next_motion_with_stock(&stock, &AtomicBool::new(false)) {
            Ok(Some(m)) => {
                assert_ne!(m.kind, MoveKind::Plunge, "{m:?}");
                commit(&mut stock, &m);
            }
            Ok(None) => panic!("a 0.005 mm plunge into stock was planned"),
            Err(e) => break e,
        }
    };
    assert!(error.contains("depth pass 1 would plunge"), "{error}");
    op.entry = Some(helix_entry(3.));
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let (motions, _, p, _) = drive(&op, &mut stock);
    let plunge = motions.iter().find(|m| m.kind == MoveKind::Plunge).unwrap();
    assert_eq!(plunge.curve.end()[2], op.top_z_mm);
    let helices: Vec<_> = motions
        .iter()
        .filter(|m| matches!(m.curve, Curve::Helix { .. }))
        .collect();
    assert!(!helices.is_empty());
    assert_eq!(helices[0].curve.start()[2], op.top_z_mm);
    assert_eq!(helices.last().unwrap().curve.end()[2], op.floor_z_mm);
    assert_eq!(p.report().max_air_plunge_mm, 0.);
}

/// A cut elsewhere in the same stock cell proves nothing about the entry's
/// footprint. With the fixture's stock as one 40 mm cell, a thin slot at Y18
/// (pass at Z9.995 under the stock top Z10) needs a helix on fresh stock and
/// still needs one after a separate slot at Y5 lowered that cell's lower
/// bound: refused without an entry, a helix from the stock top with one.
#[test]
fn slot_a_cut_elsewhere_in_the_cell_does_not_clear_the_entry() {
    let m = fixture();
    let mut spec = m.stock.height().unwrap().clone();
    spec.cell_size_mm = 40.;
    let cancel = AtomicBool::new(false);
    let prepare = |op: TrochoidalSlotOperation| {
        SlotPlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap()
    };
    let mut thin = operation();
    thin.start_mm = Some([8., 18.]);
    thin.end_mm = Some([28., 18.]);
    thin.floor_z_mm = 9.995;
    thin.tolerance_mm = 0.01;
    thin.boundary_tolerance_mm = 0.4;
    thin.max_engagement_deg = 150.;
    // Runs `op` on `stock`: Ok(motions) or the planner's refusal.
    let run = |op: &TrochoidalSlotOperation, stock: &mut Stock| -> Result<Vec<Motion>, String> {
        let mut p = prepare(op.clone());
        let mut motions = vec![];
        while let Some(motion) = p.next_motion_with_stock(stock, &cancel)? {
            commit(stock, &motion);
            motions.push(motion);
        }
        Ok(motions)
    };
    let mut first = operation();
    first.start_mm = Some([8., 5.]);
    first.end_mm = Some([28., 5.]);
    let fresh = || Stock::new(spec.clone()).unwrap();
    let thin_floor = thin.floor_z_mm;
    let after_first = || {
        let mut stock = fresh();
        run(&first, &mut stock).unwrap();
        assert!(stock.cells()[0].lower_mm < thin_floor);
        stock
    };
    for (scene, mut stock) in [("fresh", fresh()), ("after the Y5 slot", after_first())] {
        let mut bare = thin.clone();
        bare.entry = None;
        let error = run(&bare, &mut stock.clone()).unwrap_err();
        assert!(error.contains("depth pass 1 would plunge"), "{scene}: {error}");
        thin.entry = Some(helix_entry(3.));
        let motions = run(&thin, &mut stock).unwrap();
        let plunge = motions.iter().find(|m| m.kind == MoveKind::Plunge).unwrap();
        assert_eq!(plunge.curve.end()[2], thin.top_z_mm, "{scene}");
        let helix = motions
            .iter()
            .find(|m| matches!(m.curve, Curve::Helix { .. }))
            .unwrap_or_else(|| panic!("{scene}: no helix"));
        assert_eq!(helix.curve.start()[2], thin.top_z_mm, "{scene}");
    }
}
