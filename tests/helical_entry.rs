//! Independent original-CAD footprint, whole-entry, engagement and stock oracles.
use camlib::{
    motion::*, operation::OperationPlan, planar::*, pocket::*, stock::*, stream::StreamManifest,
};
use std::{
    f64::consts::{PI, TAU},
    sync::atomic::{AtomicBool, Ordering},
};
fn fixture(adaptive: bool) -> StreamManifest {
    serde_json::from_str(if adaptive {
        include_str!("../fixtures/operations/helical-adaptive.json")
    } else {
        include_str!("../fixtures/operations/helical-pocket.json")
    })
    .unwrap()
}
fn pocket(m: &mut StreamManifest) -> &mut PocketOperation {
    match &mut m.operations[0] {
        Operation::Pocket(p) => p,
        Operation::AdaptivePocket(p) => &mut p.pocket,
        _ => panic!("fixture"),
    }
}
fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn angle_on(a: f64, start: f64, sweep: f64) -> bool {
    let t = if sweep > 0. {
        (a - start).rem_euclid(TAU)
    } else {
        (start - a).rem_euclid(TAU)
    };
    sweep.abs() >= TAU - 1e-10 || t <= sweep.abs() + 1e-10 || TAU - t <= 1e-10
}
fn circular(c: &Curve) -> Option<(Vec3, Vec3, Vec3, f64)> {
    match *c {
        Curve::Arc {
            start,
            end,
            center,
            sweep_rad,
            plane: Plane::Xy,
        }
        | Curve::Helix {
            start,
            end,
            center,
            sweep_rad,
            plane: Plane::Xy,
        } => Some((start, end, center, sweep_rad)),
        _ => None,
    }
}
fn point(c: &Curve, f: f64) -> Vec3 {
    let a = c.start();
    let b = c.end();
    if f == 0. {
        return a;
    }
    if f == 1. {
        return b;
    }
    if let Some((_, _, o, s)) = circular(c) {
        let r = distance([a[0], a[1]], [o[0], o[1]]);
        let t = (a[1] - o[1]).atan2(a[0] - o[0]) + s * f;
        [
            o[0] + r * t.cos(),
            o[1] + r * t.sin(),
            a[2] + (b[2] - a[2]) * f,
        ]
    } else {
        std::array::from_fn(|k| a[k] + (b[k] - a[k]) * f)
    }
}
fn part(c: &Curve, a: f64, b: f64) -> Curve {
    let start = point(c, a);
    let end = point(c, b);
    if let Some((_, _, mut center, s)) = circular(c) {
        center[2] = start[2];
        Curve::Arc {
            start,
            end,
            center,
            plane: Plane::Xy,
            sweep_rad: s * (b - a),
        }
    } else {
        Curve::Line { start, end }
    }
}
fn axis_distance(c: &Curve, p: [f64; 2], maximum: bool) -> f64 {
    let a = c.start();
    let b = c.end();
    let endpoints = [distance([a[0], a[1]], p), distance([b[0], b[1]], p)];
    if let Some((_, _, o, sweep)) = circular(c) {
        let r = distance([a[0], a[1]], [o[0], o[1]]);
        let d = distance(p, [o[0], o[1]]);
        let theta = (a[1] - o[1]).atan2(a[0] - o[0]);
        let phase = (p[1] - o[1]).atan2(p[0] - o[0]) + if maximum { PI } else { 0. };
        let radial = if maximum { d + r } else { (d - r).abs() };
        if angle_on(phase, theta, sweep) {
            if maximum {
                endpoints[0].max(endpoints[1]).max(radial)
            } else {
                endpoints[0].min(endpoints[1]).min(radial)
            }
        } else if maximum {
            endpoints[0].max(endpoints[1])
        } else {
            endpoints[0].min(endpoints[1])
        }
    } else if maximum {
        endpoints[0].max(endpoints[1])
    } else {
        let d = [b[0] - a[0], b[1] - a[1]];
        let n = d[0] * d[0] + d[1] * d[1];
        let t = if n == 0. {
            0.
        } else {
            ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / n
        }
        .clamp(0., 1.);
        distance([a[0] + t * d[0], a[1] + t * d[1]], p)
    }
}
fn source_tube(c: &Curve) {
    let a = c.start();
    let b = c.end();
    let mut points = vec![[a[0], a[1]], [b[0], b[1]]];
    if let Some((_, _, o, sweep)) = circular(c) {
        let r = distance([a[0], a[1]], [o[0], o[1]]);
        let theta = (a[1] - o[1]).atan2(a[0] - o[0]);
        for t in [0., PI / 2., PI, 3. * PI / 2.] {
            if angle_on(t, theta, sweep) {
                points.push([o[0] + r * t.cos(), o[1] + r * t.sin()]);
            }
        }
    }
    for p in points {
        assert!(
            p[0] >= 5. - 1e-7 && p[0] <= 27. + 1e-7 && p[1] >= 5. - 1e-7 && p[1] <= 19. + 1e-7,
            "outer wall crossed by{c:?}"
        );
    }
    assert!(
        axis_distance(c, [16., 12.], false) >= 5. - 1e-7,
        "island crossed by{c:?}"
    );
    assert!(a[2].min(b[2]) >= 6. - 1e-7, "base gouged by{c:?}");
}
// Solve complete contact parameter intervals, independently of native sweeps.
fn exact_height(c: &Curve, p: [f64; 2]) -> f64 {
    let a = c.start();
    let b = c.end();
    if let Some((_, _, o, sweep)) = circular(c) {
        let r = distance([a[0], a[1]], [o[0], o[1]]);
        let rho = distance(p, [o[0], o[1]]);
        if rho == 0. {
            return if r <= 2. { a[2].min(b[2]) } else { 10. };
        }
        let cosine = (rho * rho + r * r - 4.) / (2. * rho * r);
        if cosine > 1. + 1e-12 {
            return 10.;
        }
        let alpha = cosine.clamp(-1., 1.).acos();
        let phi = (p[1] - o[1]).atan2(p[0] - o[0]);
        let start = (a[1] - o[1]).atan2(a[0] - o[0]);
        let end = start + sweep;
        let first = ((start.min(end) - phi - alpha) / TAU).ceil() as i64;
        let last = ((start.max(end) - phi + alpha) / TAU).floor() as i64;
        let mut z = 10f64;
        for k in [first, last] {
            let x = (phi - alpha + k as f64 * TAU - start) / sweep;
            let y = (phi + alpha + k as f64 * TAU - start) / sweep;
            let lo = x.min(y).max(0.);
            let hi = x.max(y).min(1.);
            if lo <= hi {
                z = z.min(a[2] + (b[2] - a[2]) * if b[2] < a[2] { hi } else { lo });
            }
        }
        z
    } else {
        let d = [b[0] - a[0], b[1] - a[1]];
        let q = [a[0] - p[0], a[1] - p[1]];
        let aa = d[0] * d[0] + d[1] * d[1];
        let bb = 2. * (q[0] * d[0] + q[1] * d[1]);
        let cc = q[0] * q[0] + q[1] * q[1] - 4.;
        let (lo, hi) = if aa == 0. {
            if cc > 0. {
                return 10.;
            }
            (0., 1.)
        } else {
            let disc = bb * bb - 4. * aa * cc;
            if disc < 0. {
                return 10.;
            }
            (
                (-bb - disc.sqrt()) / (2. * aa),
                (-bb + disc.sqrt()) / (2. * aa),
            )
        };
        let lo = lo.max(0.);
        let hi = hi.min(1.);
        if lo > hi {
            10.
        } else {
            a[2] + (b[2] - a[2]) * if b[2] < a[2] { hi } else { lo }
        }
    }
}
fn exposure(q: f64, a: f64) -> f64 {
    if q + 2. <= a {
        0.
    } else if q >= a + 2. {
        360.
    } else {
        2. * ((a * a - q * q - 4.) / (4. * q))
            .clamp(-1., 1.)
            .acos()
            .to_degrees()
    }
}
fn certify(c: &Curve, disks: &[([f64; 2], f64)]) {
    let mut stack = vec![(0., 1., 0)];
    let mut work = 0;
    while let Some((a, b, depth)) = stack.pop() {
        let sub = part(c, a, b);
        let mut clear = false;
        for &(center, radius) in disks.iter().rev() {
            work += 1;
            if exposure(axis_distance(&sub, center, true), radius) <= 90. + 1e-6 {
                clear = true;
                break;
            }
        }
        if !clear {
            assert!(depth < 24 && work < 50_000_000, "uncertified exposure{c:?}");
            let mid = (a + b) / 2.;
            stack.extend([(a, mid, depth + 1), (mid, b, depth + 1)]);
        }
    }
}
// Independent complete disk coverage from previously emitted horizontal sweeps.
// Distance to a closed curve is 1-Lipschitz: center distance + box half diagonal
// bounds EVERY point in each tile. A sample alone never certifies a tile.
fn certify_air_entry(c: &Curve, history: &[Curve]) {
    let p = [c.end()[0], c.end()[1]];
    assert_eq!(&c.start()[..2], &c.end()[..2]);
    let mut stack = vec![([p[0] - 2., p[1] - 2.], [p[0] + 2., p[1] + 2.], 0)];
    let mut work = 0;
    while let Some((lo, hi, depth)) = stack.pop() {
        let near = [p[0].clamp(lo[0], hi[0]), p[1].clamp(lo[1], hi[1])];
        if distance(near, p) > 2. {
            continue;
        }
        let mid = [(lo[0] + hi[0]) / 2., (lo[1] + hi[1]) / 2.];
        let extent = distance(lo, hi) / 2.;
        let clear = history.iter().any(|old| {
            work += 1;
            old.start()[2] == old.end()[2]
                && old.start()[2] < c.end()[2]
                && axis_distance(old, mid, false) + extent < 2. - 1e-7
        });
        if !clear {
            assert!(
                depth < 24 && work < 10_000_000,
                "air entry not enclosed by earlier complete sweeps"
            );
            for q in 0..4 {
                stack.push((
                    std::array::from_fn(|k| if q & (1 << k) == 0 { lo[k] } else { mid[k] }),
                    std::array::from_fn(|k| if q & (1 << k) == 0 { mid[k] } else { hi[k] }),
                    depth + 1,
                ));
            }
        }
    }
}
fn check(mut m: StreamManifest, clockwise: bool, radius: Option<f64>, all_cells: bool) {
    if let Operation::AdaptivePocket(op) = &mut m.operations[0] {
        op.clockwise = clockwise;
    }
    let p = pocket(&mut m);
    p.boundary_clockwise = Some(clockwise);
    let Some(PocketEntry::Helix(h)) = &mut p.entry else {
        panic!("entry");
    };
    h.radius_mm = radius;
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
    let t = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let adaptive = matches!(m.operations[0], Operation::AdaptivePocket(_));
    let mut curves = vec![];
    let mut disks = vec![];
    let mut helices = 0;
    // Entries are emitted one full turn per helix motion; consecutive turns
    // of one descent share endpoints.
    let mut last_helix_end: Option<[f64; 3]> = None;
    let mut turns = 0;
    let mut floor_circle = None;
    let mut previous_floor = 10.;
    let mut air_entries = 0;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let c = &motion.curve;
        if motion.kind != MoveKind::Rapid {
            source_tube(c);
        }
        if motion.kind == MoveKind::Link && c.start()[2] > c.end()[2] {
            certify_air_entry(c, &curves);
            air_entries += 1;
        }
        if let Some((_, _, center, sweep)) = circular(c) {
            let r = distance([c.start()[0], c.start()[1]], [center[0], center[1]]);
            if matches!(c, Curve::Helix { .. }) {
                let continuation = last_helix_end == Some(c.start());
                last_helix_end = Some(c.end());
                if !continuation {
                    assert!(floor_circle.is_none());
                    helices += 1;
                    disks.clear();
                    if adaptive || helices == 1 {
                        assert_eq!(c.start()[2], 10.);
                    } else {
                        assert!(
                            c.start()[2] > previous_floor && c.start()[2] <= previous_floor + 0.011
                        );
                    }
                }
                previous_floor = c.end()[2];
                assert!(r > 0. && r <= 2.);
                let n = sweep.abs() / TAU;
                assert!(
                    (n - 1.).abs() < 1e-10,
                    "entry helix motion must be one turn"
                );
                assert!(
                    ((c.start()[2] - c.end()[2]) / (r * sweep.abs())).atan() <= PI / 60. + 1e-12
                );
                assert_eq!(sweep < 0., clockwise);
                assert_eq!(motion.feed_mm_min, Some(150.));
                turns += n.round() as usize;
                floor_circle = Some((center, r, c.end()[2]));
            } else if let Some((origin, radius, z)) = floor_circle.take() {
                assert_eq!(center[0], origin[0]);
                assert_eq!(center[1], origin[1]);
                assert!((r - radius).abs() < 1e-8);
                assert_eq!(c.start()[2], z);
                assert!((sweep.abs() - TAU).abs() < 1e-9);
            }
        } else if motion.kind == MoveKind::Plunge {
            assert!(c.end()[2] >= 10., "straight plunge into virgin material");
        }
        if adaptive && matches!(motion.kind, MoveKind::Cut | MoveKind::Link) && !disks.is_empty() {
            certify(c, &disks);
        }
        if adaptive && motion.kind == MoveKind::Cut {
            let (_, _, o, sweep) = circular(c).unwrap();
            assert!((sweep.abs() - TAU).abs() < 1e-9);
            disks.push((
                [o[0], o[1]],
                2. + distance([c.start()[0], c.start()[1]], [o[0], o[1]]),
            ));
        }
        let d = stock
            .preview_sweep(&t, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        curves.push(motion.curve);
    }
    plan.finish_report(&cancel).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(helices, 2);
    assert_eq!(air_entries, usize::from(!adaptive));
    assert!(floor_circle.is_none());
    assert_eq!(report["helical_entries"], 2);
    assert_eq!(report["entry_turns"].as_u64().unwrap(), turns as u64);
    assert!(report["entry_geometry_work"].as_u64().unwrap() > 0);
    if adaptive {
        assert_eq!(report["seed_orbits"], 2);
        assert!(report["max_bounded_engagement_deg"].as_f64().unwrap() <= 90.);
    }
    if all_cells {
        for (i, h) in stock.cells().iter().enumerate() {
            let p = stock.cell_center(i).unwrap();
            let actual = curves
                .iter()
                .map(|c| exact_height(c, p))
                .fold(10f64, f64::min);
            assert!(
                h.lower_mm <= actual + 1e-6 && h.upper_mm >= actual - 1e-6,
                "{p:?}: {h:?} fails exact swept height {actual}"
            );
            if p[0] > 5.05
                && p[0] < 26.95
                && p[1] > 5.05
                && p[1] < 18.95
                && distance(p, [16., 12.]) > 5.05
            {
                assert!(actual <= 6. + 1e-7, "reachable core uncut{p:?}");
            }
        }
    }
    assert!(stock.volume().removed_lower_mm3 > 1500.);
    assert!(stock.volume().removed_lower_mm3 <= 4. * (468. - 9. * PI));
}
#[test]
fn real_cad_helix_entries_preserve_whole_source_stock_and_adaptive_exposure() {
    for adaptive in [false, true] {
        for clockwise in [false, true] {
            check(fixture(adaptive), clockwise, None, true);
        }
    }
}
#[test]
fn explicit_entry_radius_uses_a_bounded_adaptive_approach_and_growth() {
    for radius in [0.5, 0.1] {
        check(fixture(true), true, Some(radius), false);
    }
}
#[test]
fn no_fit_bad_parameters_and_turn_limit_fail_before_any_motion() {
    for adaptive in [false, true] {
        for (field, value) in [
            ("angle", 0.),
            ("angle", 1e-8),
            ("radius", 0.),
            ("radius", 3.),
            ("feed", -1.),
        ] {
            let mut m = fixture(adaptive);
            let Some(PocketEntry::Helix(h)) = &mut pocket(&mut m).entry else {
                panic!()
            };
            match field {
                "angle" => h.angle_rad = value,
                "radius" => h.radius_mm = Some(value),
                _ => h.feed_mm_min = Some(value),
            }
            assert!(
                OperationPlan::prepare(
                    m.operations[0].clone(),
                    &m.tools[0],
                    m.stock.height().unwrap(),
                    m.initial_position_mm,
                    1,
                    &AtomicBool::new(false)
                )
                .is_err()
            );
        }
    }
    for (width, radius, success) in [(4.15, None, false), (5., Some(1.), false), (5., None, true)] {
        let mut m = fixture(false);
        let p = pocket(&mut m);
        p.region = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "narrow entry test".into(),
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
            components: vec![Component {
                outer: Boundary {
                    vertices: vec![
                        [3., 3., 0.],
                        [29., 3., 0.],
                        [29., 3. + width, 0.],
                        [3., 3. + width, 0.],
                    ],
                },
                holes: vec![],
            }],
        };
        let Some(PocketEntry::Helix(h)) = &mut p.entry else {
            panic!()
        };
        h.radius_mm = radius;
        let p = OperationPlan::prepare(
            m.operations[0].clone(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false),
        );
        assert_eq!(p.is_ok(), success, "width{width}/radius{radius:?}");
    }
}
#[test]
fn cancelled_and_stale_entry_generation_preserves_pending_motion() {
    for adaptive in [false, true] {
        let m = fixture(adaptive);
        let cancel = AtomicBool::new(false);
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let t = m.tools[0].prepare().unwrap();
        let make = || {
            OperationPlan::prepare(
                m.operations[0].clone(),
                &m.tools[0],
                m.stock.height().unwrap(),
                m.initial_position_mm,
                1,
                &AtomicBool::new(false),
            )
            .unwrap()
        };
        let mut p = make();
        let mut fresh = make();
        for _ in 0..4 {
            let motion = p.next_motion_with_stock(&stock, &cancel).unwrap().unwrap();
            let expected = fresh
                .next_motion_with_stock(&stock, &cancel)
                .unwrap()
                .unwrap();
            assert_eq!(
                serde_json::to_value(&motion).unwrap(),
                serde_json::to_value(expected).unwrap()
            );
            assert!(p.next_motion_with_stock(&stock, &cancel).is_err());
            if !adaptive {
                // Matching generation alone cannot authorize queued entry moves.
                // Reject both a partial native commit and a different full move,
                // then continue successfully from the exact expected transaction.
                let before = serde_json::to_value(p.report()).unwrap();
                for fraction in [0.5, 1.] {
                    let mut wrong = stock.clone();
                    let mut other = motion.clone();
                    if fraction == 1. {
                        other.id += 1;
                    }
                    let delta = wrong
                        .preview_sweep(&t, &other, fraction, m.sweep, &cancel)
                        .unwrap();
                    wrong.apply_delta(&delta).unwrap();
                    assert!(
                        p.next_motion_with_stock(&wrong, &cancel)
                            .unwrap_err()
                            .contains("complete preceding")
                    );
                    assert_eq!(serde_json::to_value(p.report()).unwrap(), before);
                }
            }
            let d = stock
                .preview_sweep(&t, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&d).unwrap();
            let before = serde_json::to_value(p.report()).unwrap();
            cancel.store(true, Ordering::Relaxed);
            assert!(p.next_motion_with_stock(&stock, &cancel).is_err());
            assert_eq!(serde_json::to_value(p.report()).unwrap(), before);
            cancel.store(false, Ordering::Relaxed);
        }
    }
}

#[test]
fn disconnected_rooms_and_authored_wall_floor_allowances_keep_entry_semantics() {
    let rooms = [[3., 3., 13., 11.], [19., 13., 29., 21.]];
    for adaptive in [false, true] {
        let mut m = fixture(adaptive);
        let p = pocket(&mut m);
        p.radial_allowance_mm = 0.02;
        p.bottom_allowance_mm = 0.02;
        p.region = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "two authored rooms".into(),
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
            components: rooms
                .iter()
                .map(|&[a, b, c, d]| Component {
                    outer: Boundary {
                        vertices: vec![[a, b, 0.], [c, b, 0.], [c, d, 0.], [a, d, 0.]],
                    },
                    holes: vec![],
                })
                .collect(),
        };
        let cancel = AtomicBool::new(false);
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let t = m.tools[0].prepare().unwrap();
        let mut plan = OperationPlan::prepare(
            m.operations[0].clone(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let mut curves = vec![];
        let mut entries = 0;
        let mut last_entry_end: Option<[f64; 3]> = None;
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            let c = &motion.curve;
            if motion.kind != MoveKind::Rapid {
                let a = c.start();
                let b = c.end();
                let mut points = vec![[a[0], a[1]], [b[0], b[1]]];
                if let Some((_, _, o, sweep)) = circular(c) {
                    let r = distance([a[0], a[1]], [o[0], o[1]]);
                    let start = (a[1] - o[1]).atan2(a[0] - o[0]);
                    for theta in [0., PI / 2., PI, 3. * PI / 2.] {
                        if angle_on(theta, start, sweep) {
                            points.push([o[0] + r * theta.cos(), o[1] + r * theta.sin()]);
                        }
                    }
                }
                assert!(
                    rooms.iter().any(|&[x, y, u, v]| points.iter().all(|p| p[0]
                        >= x + 2.02 - 1e-7
                        && p[0] <= u - 2.02 + 1e-7
                        && p[1] >= y + 2.02 - 1e-7
                        && p[1] <= v - 2.02 + 1e-7)),
                    "entry/link crossed retained room boundary"
                );
                assert!(a[2].min(b[2]) >= 6.02 - 1e-7);
            }
            if matches!(c, Curve::Helix { .. }) {
                entries += usize::from(last_entry_end != Some(c.start()));
                last_entry_end = Some(c.end());
            }
            let d = stock
                .preview_sweep(&t, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&d).unwrap();
            curves.push(motion.curve);
        }
        plan.finish_report(&cancel).unwrap();
        let report = serde_json::to_value(plan.report()).unwrap();
        assert!(entries >= 4);
        assert_eq!(report["helical_entries"].as_u64().unwrap(), entries as u64);
        for p in [[8., 7.], [24., 17.]] {
            assert!(
                curves
                    .iter()
                    .map(|c| exact_height(c, p))
                    .fold(10f64, f64::min)
                    <= 6.020001
            );
        }
        assert!(stock.volume().removed_lower_mm3 > 450.);
        assert!(stock.volume().removed_lower_mm3 <= 160. * 3.98);
    }
}
