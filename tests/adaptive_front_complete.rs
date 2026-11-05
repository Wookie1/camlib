//! Full-operation regression for the frozen real CAD case. Independent prism
//! checks are continuous; point/angular probes are explicitly corroboration.
use camlib::{
    motion::{Curve, Motion, MoveKind, WorkFrame},
    operation::OperationPlan,
    planar::{Boundary, Component, PlanarRegion},
    stock::Stock,
    stream::StreamManifest,
    verification::{BodyRole, Options, Prism, Representation, Scene, Status},
};
use std::f64::consts::TAU;
fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "independent prism".into(),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component {
            outer: Boundary {
                vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
            },
            holes: vec![],
        }],
    }
}
fn scene() -> Scene {
    let boxes = [
        (0., 0., 26., 3., 10.),
        (0., 23., 26., 26., 10.),
        (0., 3., 3., 23., 10.),
        (23., 3., 26., 23., 10.),
        (11., 11., 23., 23., 10.),
        (0., 0., 26., 26., 8.),
    ];
    Scene {
        version: 1,
        units: "mm".into(),
        id: "independent-l-part".into(),
        revision: "1".into(),
        work_to_machine: WorkFrame {
            origin_mm: [0.; 3],
            x: [1., 0., 0.],
            y: [0., 1., 0.],
            z: [0., 0., 1.],
        },
        machine: None,
        bodies: boxes
            .into_iter()
            .enumerate()
            .map(|(i, (x0, y0, x1, y1, z))| Prism {
                id: format!("part-{i}"),
                role: BodyRole::ProtectedTarget,
                footprint: rectangle(x0, y0, x1, y1),
                z_min_mm: 0.,
                z_max_mm: z,
                z_error_mm: 0.,
                representation: Representation::SolidPrism,
            })
            .collect(),
    }
}
fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l = d[0] * d[0] + d[1] * d[1];
    let t = if l == 0. {
        0.
    } else {
        ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l
    }
    .clamp(0., 1.);
    dist(p, [a[0] + t * d[0], a[1] + t * d[1]])
}
fn distance(curve: &Curve, p: [f64; 2]) -> f64 {
    let (a, b) = (curve.start(), curve.end());
    let (a, b) = ([a[0], a[1]], [b[0], b[1]]);
    match *curve {
        Curve::Line { .. } => segment(p, a, b),
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let c = [center[0], center[1]];
            let start = (a[1] - c[1]).atan2(a[0] - c[0]);
            let theta = (p[1] - c[1]).atan2(p[0] - c[0]);
            let travel = if sweep_rad > 0. {
                (theta - start).rem_euclid(TAU)
            } else {
                (start - theta).rem_euclid(TAU)
            };
            let end = dist(p, a).min(dist(p, b));
            if sweep_rad.abs() >= TAU - 1e-10 || travel <= sweep_rad.abs() + 1e-10 {
                end.min((dist(p, c) - dist(a, c)).abs())
            } else {
                end
            }
        }
        _ => f64::INFINITY,
    }
}
// Independent exact C = erosion of the analytic L by radius3. In addition to
// six line boundaries, its re-entrant corner is the radius3 quarter circle.
fn distance_to_centres(p: [f64; 2]) -> f64 {
    if (6. ..=20.).contains(&p[0])
        && (6. ..=20.).contains(&p[1])
        && (11. - p[0]).max(0.).hypot((11. - p[1]).max(0.)) >= 3.
    {
        return 0.;
    }
    let edges = [
        ([6., 6.], [20., 6.]),
        ([20., 6.], [20., 8.]),
        ([20., 8.], [11., 8.]),
        ([8., 11.], [8., 20.]),
        ([8., 20.], [6., 20.]),
        ([6., 20.], [6., 6.]),
    ];
    let mut d = edges
        .into_iter()
        .map(|(a, b)| segment(p, a, b))
        .fold(f64::INFINITY, f64::min);
    d = d.min(dist(p, [8., 11.])).min(dist(p, [11., 8.]));
    if p[0] <= 11. && p[1] <= 11. {
        d = d.min((dist(p, [11., 11.]) - 3.).abs());
    }
    d
}
fn known_empty(history: &[Motion], p: [f64; 2], z: f64, r: f64) -> bool {
    if (0..2).any(|k| p[k] < 0. || p[k] > 26.) {
        return true;
    }
    history.iter().any(|m| {
        m.kind != MoveKind::Rapid
            && m.curve.start()[2] == m.curve.end()[2]
            && m.curve.start()[2] <= z
            && distance(&m.curve, p) <= r + 1e-7
    })
}
#[test]
fn saved_real_cad_front_completes_reachable_pocket_instead_of_stalling() {
    saved_real_cad_case(false, false);
}

#[test]
fn saved_real_cad_conventional_front_completes_reachable_pocket() {
    saved_real_cad_case(true, false);
}

#[test]
fn saved_real_cad_rotated_climb_front_completes_reachable_pocket() {
    saved_real_cad_case(false, true);
}

fn saved_real_cad_case(conventional: bool, rotated: bool) {
    let saved: serde_json::Value = serde_json::from_str(include_str!(
        "../fixtures/regressions/disconnected-pocket.json"
    ))
    .unwrap();
    let mut manifest: StreamManifest = serde_json::from_value(saved["manifest"].clone()).unwrap();
    let camlib::pocket::Operation::AdaptivePocket(op) = &mut manifest.operations[0] else {
        panic!("saved case must be adaptive");
    };
    op.cutting_sense = Some(if conventional {
        camlib::adaptive::CuttingSense::Conventional
    } else {
        camlib::adaptive::CuttingSense::Climb
    });
    let mut geometry = scene();
    if rotated {
        for region in std::iter::once(&mut op.pocket.region)
            .chain(geometry.bodies.iter_mut().map(|body| &mut body.footprint))
        {
            for component in &mut region.components {
                for boundary in std::iter::once(&mut component.outer).chain(&mut component.holes) {
                    for p in &mut boundary.vertices {
                        let [x, y, _] = *p;
                        p[0] = 26. - y;
                        p[1] = x;
                    }
                }
            }
        }
    }
    let tool = manifest.tools[0].prepare().unwrap();
    let r = tool.cutter().definition().diameter_mm / 2.;
    let verifier = geometry
        .prepare(Options {
            spatial_resolution_mm: 1e-5,
            ..Default::default()
        })
        .unwrap();
    let mut stock = Stock::new(manifest.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        manifest.operations[0].clone(),
        &manifest.tools[0],
        &stock,
        manifest.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut count = 0;
    let mut history = Vec::new();
    let mut reentry_descents = 0;
    let started = std::time::Instant::now();
    loop {
        let next = plan.next_motion_with_stock(&stock, &|| false);
        let motion = match next {
            Ok(Some(m)) => m,
            Ok(None) => break,
            Err(e) => panic!(
                "incomplete after {count}: {e}; last={:?}; report={}",
                history.last(),
                serde_json::to_string(&plan.report()).unwrap()
            ),
        };
        count += 1;
        if count % 64 == 0 {
            let report = serde_json::to_value(plan.report()).unwrap();
            eprintln!(
                "front {count} {:?} volume={} front={}",
                started.elapsed(),
                stock.volume().removed_lower_mm3,
                report["front"]
            );
        }
        assert!(count < 10000, "front did not complete in bounded work");
        let checked = verifier.verify_motion(&tool, &motion, &|| false).unwrap();
        assert_eq!(
            checked.status,
            Status::Clear,
            "independent full-motion part/assembly check: {:?}",
            checked.findings
        );
        if motion.kind == MoveKind::Plunge && motion.curve.end()[2] == 9. && !history.is_empty() {
            reentry_descents += 1;
            assert_eq!(motion.feed_mm_min, Some(150.));
            let p = motion.curve.end();
            for radial in [0., 0.5, 1.] {
                for i in 0..16 {
                    let a = TAU * i as f64 / 16.;
                    let q = [p[0] + radial * r * a.cos(), p[1] + radial * r * a.sin()];
                    assert!(
                        known_empty(&history, q, p[2], r),
                        "re-entry into uncut column at{q:?}"
                    );
                }
            }
        }
        if count > 4
            && count % 97 == 0
            && motion.curve.start()[2] == 9.
            && motion.curve.end()[2] == 9.
            && matches!(motion.kind, MoveKind::Cut | MoveKind::Link)
        {
            let path = motion.curve.prepare(1e-5).unwrap();
            for i in 0..=4 {
                let f = i as f64 / 4.;
                let p = path.position(f).unwrap();
                let heading = match motion.curve {
                    Curve::Line { start, end } => (end[1] - start[1]).atan2(end[0] - start[0]),
                    Curve::Arc {
                        start,
                        center,
                        sweep_rad,
                        ..
                    } => {
                        (start[1] - center[1]).atan2(start[0] - center[0])
                            + f * sweep_rad
                            + sweep_rad.signum() * std::f64::consts::PI / 2.
                    }
                    _ => unreachable!(),
                };
                let mut contact = 0;
                for j in 0..720 {
                    let a = TAU * (j as f64 + 0.5) / 720.;
                    let q = [p[0] + r * a.cos(), p[1] + r * a.sin()];
                    if !known_empty(&history, q, 9., r) {
                        contact += 1;
                        assert!(
                            if conventional {
                                (a - heading).sin() >= -1e-7
                            } else {
                                (a - heading).sin() <= 1e-7
                            },
                            "sampled material on wrong side at{q:?}"
                        );
                    }
                }
                assert!(
                    contact as f64 / 2. <= 92.,
                    "sampled engagement exceeds bound with angular-bin reserve"
                );
            }
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., manifest.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        history.push(motion);
    }
    plan.finish_report(&|| false).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    eprintln!(
        "complete CAD front conventional={conventional},rotated={rotated}, {count} motions: {report}"
    );
    assert_eq!(report["front"]["coverage_complete"], true);
    assert_eq!(report["geometric_residual_status"], "estimated");
    assert!(
        report["front"]["max_bounded_engagement_deg"]
            .as_f64()
            .unwrap()
            <= 90.
    );
    assert_eq!(report["seed_orbits"], 1);
    assert_eq!(report["helical_entries"], 1);
    assert!(
        reentry_descents > 0,
        "did not exercise the continuation branch"
    );
    assert!(stock.volume().removed_lower_mm3 > 180.);
    // Independent dense remaining-stock corroboration across the analytic
    // reachable domain, inset0.02mm to separate construction/numeric boundaries.
    let [nx, _] = stock.dimensions();
    let widths = stock.cell_widths_mm();
    for ix in 0..130 {
        for iy in 0..130 {
            let p = [(ix as f64 + 0.37) * 0.2, (iy as f64 + 0.37) * 0.2];
            let empty = known_empty(&history, p, 9., r);
            let source_p = if rotated { [p[1], 26. - p[0]] } else { p };
            if distance_to_centres(source_p) < 2.98 {
                assert!(empty, "reachable stock remains at{p:?}");
            }
            let exact = if empty { 9. } else { 10. };
            let x = (p[0] / widths[0]) as usize;
            let y = (p[1] / widths[1]) as usize;
            let h = stock.cells()[y * nx + x];
            assert!(
                h.lower_mm <= exact + 1e-8 && h.upper_mm >= exact - 1e-8,
                "stock bounds miss independent height at{p:?}: {h:?} vs{exact}"
            );
        }
    }
}

// Small independent rectilinear corpus exercises real component/depth resets.
// The full L cases above cover concavity and both material sides.
#[test]
fn stock_front_completes_disconnected_components_at_two_depths() {
    let saved: serde_json::Value = serde_json::from_str(include_str!(
        "../fixtures/regressions/disconnected-pocket.json"
    ))
    .unwrap();
    let mut manifest: StreamManifest = serde_json::from_value(saved["manifest"].clone()).unwrap();
    manifest.stock.height_mut().unwrap().max_mm = [28., 16., 10.];
    let camlib::pocket::Operation::AdaptivePocket(op) = &mut manifest.operations[0] else {
        panic!("saved case must be adaptive");
    };
    let rectangles = [(3., 3., 13., 13.), (15., 3., 25., 13.)];
    op.pocket.region.components = rectangles
        .iter()
        .map(|&(a, b, c, d)| rectangle(a, b, c, d).components.remove(0))
        .collect();
    op.pocket.region.name = "two disconnected pockets".into();
    op.pocket.region.source = serde_json::Value::Null;
    op.pocket.floor_z_mm = 8.;
    op.pocket.stepdown_mm = 1.;
    let tool = manifest.tools[0].prepare().unwrap();
    let mut geometry = scene();
    let boxes = [
        (0., 0., 28., 3., 10.),
        (0., 13., 28., 16., 10.),
        (0., 3., 3., 13., 10.),
        (13., 3., 15., 13., 10.),
        (25., 3., 28., 13., 10.),
        (0., 0., 28., 16., 8.),
    ];
    geometry.bodies = boxes
        .into_iter()
        .enumerate()
        .map(|(i, (a, b, c, d, z))| Prism {
            id: format!("independent-part-{i}"),
            role: BodyRole::ProtectedTarget,
            footprint: rectangle(a, b, c, d),
            z_min_mm: 0.,
            z_max_mm: z,
            z_error_mm: 0.,
            representation: Representation::SolidPrism,
        })
        .collect();
    let verifier = geometry
        .prepare(Options {
            spatial_resolution_mm: 1e-5,
            // Explicit 10micron machining tolerance; fixtures/noncutting parts
            // and all side/bottom boundaries below the top cap remain strict.
            target_floor_tolerance_mm: Some(0.01),
            ..Default::default()
        })
        .unwrap();
    let mut stock = Stock::new(manifest.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        manifest.operations[0].clone(),
        &manifest.tools[0],
        &stock,
        manifest.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut history = Vec::new();
    let mut floors = std::collections::BTreeSet::new();
    while let Some(motion) = plan
        .next_motion_with_stock(&stock, &|| false)
        .unwrap_or_else(|e| {
            panic!(
                "multi-component/depth incomplete at {}: {e}; {}",
                history.len(),
                serde_json::to_string(&plan.report()).unwrap()
            )
        })
    {
        assert!(
            history.len() < 10000,
            "multi-component/depth did not complete"
        );
        let checked = verifier.verify_motion(&tool, &motion, &|| false).unwrap();
        assert_eq!(
            checked.status,
            Status::Clear,
            "motion {motion:?}: {:?}",
            checked.findings
        );
        if motion.curve.start()[2] == motion.curve.end()[2] && motion.kind != MoveKind::Rapid {
            floors.insert(motion.curve.end()[2] as i32);
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., manifest.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        history.push(motion);
    }
    plan.finish_report(&|| false).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    eprintln!(
        "complete disconnected/depth {} motions: {}",
        history.len(),
        report["front"]
    );
    assert_eq!(report["front"]["completed_components"], 4);
    assert_eq!(report["front"]["coverage_complete"], true);
    assert_eq!(report["geometric_residual_status"], "estimated");
    assert_eq!(report["depth_passes"], 2);
    assert_eq!(report["seed_orbits"], 4);
    assert_eq!(floors, std::collections::BTreeSet::from([8, 9]));
    assert!(stock.volume().removed_lower_mm3 > 300.);
    // Analytic reachable C for each 10mm square is its radius3 inset.
    // Dense probes corroborate remaining stock, while production completion
    // still requires the conservative whole-tile domain certificate.
    for &(a, b, c, d) in &rectangles {
        for i in 0..50 {
            for j in 0..50 {
                let p = [a + (i as f64 + 0.37) * 0.2, b + (j as f64 + 0.37) * 0.2];
                let q = [p[0].clamp(a + 3., c - 3.), p[1].clamp(b + 3., d - 3.)];
                if dist(p, q) < 2.98 {
                    assert!(known_empty(&history, p, 8., 2.), "stock remains at {p:?}");
                }
            }
        }
    }
}
