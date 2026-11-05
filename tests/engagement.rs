use camlib::{engagement::*, motion::*, stock::*};
use std::{
    f64::consts::{PI, TAU},
    sync::atomic::AtomicBool,
};
#[test]
fn clear_prescreen_poses_never_certify_an_unsafe_interval_between_them() {
    let scene = EngagementScene::new(
        StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [20., 20., 10.],
            cell_size_mm: 1.,
            max_cells: 1000,
        },
        vec![],
    )
    .unwrap();
    let motion = Motion {
        id: 1,
        operation: "between-poses".into(),
        tool: "t".into(),
        kind: MoveKind::Cut,
        feed_mm_min: Some(600.),
        curve: Curve::Arc {
            start: [-3.1, -1.1, 5.],
            end: [-3.1, -1.1, 5.],
            center: [-1.1, -1.1, 5.],
            plane: Plane::Xy,
            sweep_rad: 2. * TAU,
        },
    };
    let path = motion.curve.prepare(1e-5).unwrap();
    for f in [0., 0.5, 1.] {
        let p = path.position(f).unwrap();
        assert!(
            p[0] + 1. < 0.,
            "whole cutter outside stock at each prescreen pose"
        );
    }
    // At the intervening diagonal the actual quadrant contact is
    // pi/2+2asin(sqrt(2)-1.1), exceeding the authored90degree limit.
    assert!(PI / 2. + 2. * (2f64.sqrt() - 1.1).asin() > PI / 2.);
    let result = scene
        .check(&motion, 1., 90., EngagementOptions::default(), &|| false)
        .unwrap();
    assert_eq!(result.status, EngagementStatus::Unresolved);
    assert!(
        result.nodes > 4,
        "clear poses must still reach whole-interval subdivision"
    );
}

fn stock() -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [-100., -100., 0.],
        max_mm: [100., 100., 10.],
        cell_size_mm: 1.,
        max_cells: 100000,
    }
}
fn motion(curve: Curve) -> Motion {
    Motion {
        id: 1,
        operation: "front".into(),
        tool: "t".into(),
        kind: MoveKind::Cut,
        curve,
        feed_mm_min: Some(600.),
    }
}
fn line(a: [f64; 3], b: [f64; 3]) -> Motion {
    motion(Curve::Line { start: a, end: b })
}
fn arc(c: [f64; 3], r: f64, start: f64, sweep: f64) -> Motion {
    motion(Curve::Arc {
        start: [c[0] + r * start.cos(), c[1] + r * start.sin(), c[2]],
        end: [
            c[0] + r * (start + sweep).cos(),
            c[1] + r * (start + sweep).sin(),
            c[2],
        ],
        center: c,
        plane: Plane::Xy,
        sweep_rad: sweep,
    })
}
fn support(m: &Motion, r: f64) -> ClearedSweep {
    ClearedSweep::from_motion(m, r, 1e-5).unwrap()
}

#[test]
fn one_whole_filled_sweep_witness_discharges_all_sides_before_unneeded_history_work() {
    let mut history: Vec<_> = (0..128)
        .map(|i| {
            let x = 20. + i as f64 * 0.01;
            support(&line([x, 20., 5.], [x + 0.2, 20., 5.]), 1.)
        })
        .collect();
    history.push(support(&arc([0., 0., 5.], 0.5, 0., TAU), 1.));
    let scene = EngagementScene::new(stock(), history).unwrap();
    let candidate = arc([0., 0., 5.], 0.2, 0., 2. * TAU);
    // Whole-distance certificate: for every cutter-axis position,
    // distance_to_seed_centre + cutter_radius <= .2+1 < .5+1.
    // Only the latest leaf needs visiting; budget exhaustion proves nothing.
    let options = EngagementOptions {
        max_work: 2,
        ..Default::default()
    };
    for result in [
        scene.check(&candidate, 1., 0., options, &|| false).unwrap(),
        scene
            .check_material_side(&candidate, 1., true, options, &|| false)
            .unwrap(),
        scene
            .check_material_side(&candidate, 1., false, options, &|| false)
            .unwrap(),
    ] {
        assert_eq!(result.status, EngagementStatus::Bounded);
        assert_eq!(result.upper_angle_deg, 0.);
        assert_eq!(result.support_tests + result.index_visits, 2);
    }
    let exhausted = EngagementOptions {
        max_work: 0,
        ..Default::default()
    };
    assert_eq!(
        scene
            .check(&candidate, 1., 0., exhausted, &|| false)
            .unwrap()
            .status,
        EngagementStatus::Unresolved
    );
    let annular =
        EngagementScene::new(stock(), vec![support(&arc([0., 0., 5.], 2., 0., TAU), 1.)]).unwrap();
    assert_eq!(
        annular
            .check(&candidate, 1., 0., options, &|| false)
            .unwrap()
            .status,
        EngagementStatus::Unresolved,
        "the real inner hole stays material"
    );
    assert!(scene.check(&candidate, 1., 0., options, &|| true).is_err());
}
fn check(scene: &EngagementScene, m: &Motion, r: f64, limit: f64) -> EngagementReport {
    scene
        .check(
            m,
            r,
            limit,
            EngagementOptions::default(),
            &AtomicBool::new(false),
        )
        .unwrap()
}

#[test]
fn larger_seed_disk_bounds_a_whole_circular_front_without_tiny_steps() {
    let old = arc([0., 0., 5.], 0.5, 0., TAU);
    let scene = EngagementScene::new(stock(), vec![support(&old, 1.)]).unwrap();
    for sweep in [TAU, -TAU, PI / 2.] {
        let candidate = arc([0., 0., 5.], 0.58, 0., sweep);
        let result = check(&scene, &candidate, 1., 83.);
        assert_eq!(result.status, EngagementStatus::Bounded);
        assert_eq!(result.nodes, 1);
        let exact = 2.
            * ((1.5f64.powi(2) - 0.58f64.powi(2) - 1.) / (2. * 0.58))
                .acos()
                .to_degrees();
        assert!(result.upper_angle_deg >= exact && result.upper_angle_deg < exact + 0.05);
    }
    let unsafe_path = arc([0., 0., 5.], 0.7, 0., TAU);
    assert_eq!(
        check(&scene, &unsafe_path, 1., 83.).status,
        EngagementStatus::Unresolved
    );
}

#[test]
fn angular_union_counts_both_uncovered_contacts_not_only_largest_gap() {
    let a = arc([-0.5, 0., 5.], 0.25, 0., TAU);
    let b = arc([0.5, 0., 5.], 0.25, 0., -TAU);
    let query = line([0., 0., 5.], [0.000001, 0., 5.]);
    for m in [&a, &b] {
        let scene = EngagementScene::new(stock(), vec![support(m, 0.75)]).unwrap();
        assert_eq!(
            check(&scene, &query, 1., 170.).status,
            EngagementStatus::Unresolved
        );
    }
    let scene = EngagementScene::new(stock(), vec![support(&a, 0.75), support(&b, 0.75)]).unwrap();
    let result = check(&scene, &query, 1., 60.);
    let exact = (TAU - 4. * 0.25f64.acos()).to_degrees();
    assert_eq!(result.status, EngagementStatus::Bounded);
    assert!(result.upper_angle_deg >= exact && result.upper_angle_deg < exact + 0.1);
    assert!(
        result.upper_angle_deg > 50.,
        "must count both contact intervals"
    );
}

#[test]
fn long_parallel_capsule_front_is_certified_as_one_whole_interval() {
    let cleared = line([0., 0., 5.], [40., 0., 5.]);
    let query = line([2., 0.24, 5.], [38., 0.24, 5.]);
    let scene = EngagementScene::new(stock(), vec![support(&cleared, 1.)]).unwrap();
    let result = check(&scene, &query, 1., 83.);
    let exact = 2. * 0.76f64.acos().to_degrees();
    assert_eq!(result.status, EngagementStatus::Bounded);
    assert_eq!(
        result.nodes, 1,
        "full directional extrema avoid thousands of endpoint samples"
    );
    assert!(result.upper_angle_deg >= exact && result.upper_angle_deg < exact + 0.1);
    assert_eq!(
        check(&scene, &line([2., 0.4, 5.], [38., 0.4, 5.]), 1., 83.).status,
        EngagementStatus::Unresolved
    );
}

#[test]
fn full_arc_tube_preserves_its_annular_hole_and_multiple_turns() {
    for sweep in [TAU, -TAU, 3. * TAU] {
        let ring = arc([0., 0., 5.], 3., 0., sweep);
        let scene = EngagementScene::new(stock(), vec![support(&ring, 0.5)]).unwrap();
        let in_hole = line([0., 0., 5.], [0.001, 0., 5.]);
        assert_eq!(
            check(&scene, &in_hole, 0.1, 90.).status,
            EngagementStatus::Unresolved
        );
        let in_ring = line([3., 0., 5.], [3., 0.001, 5.]);
        assert_eq!(
            check(&scene, &in_ring, 0.1, 1.).status,
            EngagementStatus::Bounded
        );
    }
}

#[test]
fn partial_arc_does_not_fill_its_missing_sector_or_interior() {
    for (start, sweep) in [(0., PI), (PI, -PI)] {
        let half = arc([0., 0., 5.], 3., start, sweep);
        let scene = EngagementScene::new(stock(), vec![support(&half, 0.5)]).unwrap();
        assert_eq!(
            check(&scene, &line([0., 3., 5.], [0.001, 3., 5.]), 0.1, 1.).status,
            EngagementStatus::Bounded
        );
        assert_eq!(
            check(&scene, &line([0., -3., 5.], [0.001, -3., 5.]), 0.1, 90.).status,
            EngagementStatus::Unresolved
        );
        assert_eq!(
            check(&scene, &line([0., 0., 5.], [0.001, 0., 5.]), 0.1, 90.).status,
            EngagementStatus::Unresolved
        );
        // The endpoint disk is real support even outside the radial sector.
        assert_eq!(
            check(&scene, &line([3.1, -0.1, 5.], [3.101, -0.1, 5.]), 0.1, 1.).status,
            EngagementStatus::Bounded
        );
    }
}

#[test]
fn arc_interior_is_checked_even_when_both_endpoints_are_outside_stock() {
    let mut s = stock();
    s.min_mm = [0., 0., 0.];
    s.max_mm = [20., 20., 10.];
    let scene = EngagementScene::new(s, vec![]).unwrap();
    let through = arc([-3., 10., 5.], 10., -PI / 2., PI);
    assert_eq!(
        check(&scene, &through, 1., 90.).status,
        EngagementStatus::Unresolved
    );
    let outside = arc([-3., 10., 5.], 10., -PI / 2., -PI);
    let r = check(&scene, &outside, 1., 0.);
    assert_eq!(r.status, EngagementStatus::Bounded);
    assert_eq!(r.upper_angle_deg, 0.);
}

#[test]
fn higher_floor_cannot_clear_a_lower_layer_or_admit_unsupported_curves() {
    let old = arc([0., 0., 5.], 0.5, 0., TAU);
    let scene = EngagementScene::new(stock(), vec![support(&old, 1.)]).unwrap();
    assert_eq!(
        check(&scene, &arc([0., 0., 4.], 0.58, 0., PI), 1., 83.).status,
        EngagementStatus::Unresolved
    );
    assert_eq!(
        check(&scene, &arc([0., 0., 6.], 0.58, 0., PI), 1., 83.).status,
        EngagementStatus::Bounded
    );
    let helix = motion(Curve::Helix {
        start: [1., 0., 6.],
        end: [1., 0., 5.],
        center: [0., 0., 6.],
        plane: Plane::Xy,
        sweep_rad: TAU,
    });
    assert!(ClearedSweep::from_motion(&helix, 1., 1e-5).is_err());
    assert!(
        scene
            .check(
                &helix,
                1.,
                83.,
                EngagementOptions::default(),
                &AtomicBool::new(false)
            )
            .is_err()
    );
}

#[test]
fn work_resolution_and_cancellation_never_turn_uncertainty_into_success() {
    let old = arc([0., 0., 5.], 0.5, 0., TAU);
    let scene = EngagementScene::new(stock(), vec![support(&old, 1.)]).unwrap();
    let query = arc([0., 0., 5.], 0.58, 0., PI);
    for options in [
        EngagementOptions {
            max_nodes: 0,
            ..Default::default()
        },
        EngagementOptions {
            max_work: 0,
            ..Default::default()
        },
    ] {
        let r = scene
            .check(&query, 1., 83., options, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(r.status, EngagementStatus::Unresolved);
        assert_eq!(r.reason, Some(UnresolvedReason::Work));
        assert_eq!(r.upper_angle_deg, 360.);
    }
    assert!(
        scene
            .check(
                &query,
                1.,
                83.,
                EngagementOptions::default(),
                &AtomicBool::new(true)
            )
            .is_err()
    );
    assert!(
        scene
            .check(
                &query,
                f64::NAN,
                83.,
                EngagementOptions::default(),
                &AtomicBool::new(false)
            )
            .is_err()
    );
    assert!(
        scene
            .check(
                &query,
                1.,
                83.,
                EngagementOptions {
                    resolution_mm: 0.,
                    ..Default::default()
                },
                &AtomicBool::new(false)
            )
            .is_err()
    );
    assert_eq!(
        check(&scene, &query, 1., 83.).status,
        EngagementStatus::Bounded
    );
}

#[test]
fn append_only_index_prunes_irrelevant_history_and_accounts_for_index_work() {
    let mut scene = EngagementScene::new(stock(), vec![]).unwrap();
    scene
        .append(support(&arc([0., 0., 5.], 0.5, 0., TAU), 1.))
        .unwrap();
    for i in 0..4096 {
        let x = 20. + (i % 64) as f64 * 0.1;
        let y = 20. + (i / 64) as f64 * 0.1;
        scene
            .append(support(&line([x, y, 5.], [x + 0.05, y, 5.]), 0.1))
            .unwrap();
    }
    let q = arc([0., 0., 5.], 0.58, 0., TAU);
    let r = check(&scene, &q, 1., 83.);
    assert_eq!(r.status, EngagementStatus::Bounded);
    assert!(
        r.support_tests < 8,
        "irrelevant supports must be pruned: {r:?}"
    );
    assert!(r.index_visits < 100, "balanced subtree pruning: {r:?}");
    let limited = scene
        .check(
            &q,
            1.,
            83.,
            EngagementOptions {
                max_work: 1,
                ..Default::default()
            },
            &AtomicBool::new(false),
        )
        .unwrap();
    assert_eq!(limited.status, EngagementStatus::Unresolved);
    assert_eq!(limited.reason, Some(UnresolvedReason::Work));
    assert!(EngagementScene::new_with_cancel(stock(), vec![], &AtomicBool::new(true)).is_err());
}

// Independent closest-point distance, not angular events or interval proofs.
fn clearance_distance(curve: &Curve, p: [f64; 2]) -> f64 {
    let a = curve.start();
    let b = curve.end();
    let norm = |x: [f64; 2], y: [f64; 2]| (x[0] - y[0]).hypot(x[1] - y[1]);
    match *curve {
        Curve::Line { .. } => {
            let d = [b[0] - a[0], b[1] - a[1]];
            let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (d[0] * d[0] + d[1] * d[1]))
                .clamp(0., 1.);
            norm(p, [a[0] + t * d[0], a[1] + t * d[1]])
        }
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let r = (a[0] - center[0]).hypot(a[1] - center[1]);
            let start = (a[1] - center[1]).atan2(a[0] - center[0]);
            let angle = (p[1] - center[1]).atan2(p[0] - center[0]);
            let travel = if sweep_rad > 0. {
                (angle - start).rem_euclid(TAU)
            } else {
                (start - angle).rem_euclid(TAU)
            };
            if sweep_rad.abs() >= TAU || travel <= sweep_rad.abs() {
                ((p[0] - center[0]).hypot(p[1] - center[1]) - r).abs()
            } else {
                norm(p, [a[0], a[1]]).min(norm(p, [b[0], b[1]]))
            }
        }
        _ => unreachable!(),
    }
}
fn axis_position(curve: &Curve, t: f64) -> [f64; 2] {
    let a = curve.start();
    let b = curve.end();
    match *curve {
        Curve::Line { .. } => [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])],
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let r = (a[0] - center[0]).hypot(a[1] - center[1]);
            let angle = (a[1] - center[1]).atan2(a[0] - center[0]) + t * sweep_rad;
            [center[0] + r * angle.cos(), center[1] + r * angle.sin()]
        }
        _ => unreachable!(),
    }
}

#[test]
fn independent_dense_angle_probes_corroborate_union_bounds_at_translated_origins() {
    let mut bounded = 0;
    for shift in [[0., 0.], [10000.123, -20000.456]] {
        let p = |x: f64, y: f64| [x + shift[0], y + shift[1], 5.];
        let old = [
            (arc(p(0., 0.), 0.5, 0., TAU), 1.),
            (line(p(0., 0.), p(6., 0.)), 1.),
            (arc(p(6., 2.), 2., -PI / 2., PI), 0.8),
            (arc(p(10., 2.), 3., 0., -TAU), 0.6),
        ];
        let mut spec = stock();
        for (k, offset) in shift.iter().enumerate() {
            spec.min_mm[k] += offset;
            spec.max_mm[k] += offset;
        }
        let scene =
            EngagementScene::new(spec, old.iter().map(|(m, r)| support(m, *r)).collect()).unwrap();
        for (x, y) in [
            (0.4, 0.2),
            (2., 0.7),
            (4., 0.7),
            (6.2, 0.3),
            (7.7, 2.),
            (9.5, 1.5),
            (10., 2.),
        ] {
            for sweep in [-0.4, 0.4] {
                let q = arc(p(x, y), 0.1, 0., sweep);
                let report = check(&scene, &q, 0.35, 170.);
                if report.status == EngagementStatus::Bounded {
                    bounded += 1;
                }
                let mut peak = 0f64;
                for step in 0..=32 {
                    let axis = axis_position(&q.curve, step as f64 / 32.);
                    let mut contact = 0;
                    for n in 0..4096 {
                        let theta = TAU * (n as f64 + 0.37) / 4096.;
                        let pt = [axis[0] + 0.35 * theta.cos(), axis[1] + 0.35 * theta.sin()];
                        let empty = old
                            .iter()
                            .any(|(m, r)| clearance_distance(&m.curve, pt) <= *r);
                        contact += usize::from(!empty);
                    }
                    peak = peak.max(360. * contact as f64 / 4096.);
                }
                // Sampling corroborates only; continuous success is established
                // by the implementation's analytic interval enclosures.
                assert!(
                    peak <= report.upper_angle_deg + 1.,
                    "underbounded dense contact at {shift:?}/{x}/{y}/{sweep}: {peak}/{report:?}"
                );
            }
        }
    }
    assert!(bounded >= 8, "must exercise successful conditional proofs");
}

#[test]
fn material_side_follows_instantaneous_feed_instead_of_orbit_sign_alone() {
    let old = arc([0., 0., 5.], 0.5, 0., TAU);
    let scene = EngagementScene::new(stock(), vec![support(&old, 1.)]).unwrap();
    for (sweep, right) in [(TAU, true), (-TAU, false), (3. * TAU, true)] {
        let candidate = arc([0., 0., 5.], 0.58, 2.9, sweep);
        let right_result = scene
            .check_material_side(&candidate, 1., right, EngagementOptions::default(), &|| {
                false
            })
            .unwrap();
        assert_eq!(
            right_result.status,
            EngagementStatus::Bounded,
            "{right_result:?}"
        );
        assert_eq!(right_result.upper_angle_deg, 0.);
        let wrong_result = scene
            .check_material_side(
                &candidate,
                1.,
                !right,
                EngagementOptions::default(),
                &|| false,
            )
            .unwrap();
        assert_eq!(wrong_result.status, EngagementStatus::Unresolved);
    }
    // The same straight front reverses physical cutting sense when feed reverses.
    let old = line([0., 0., 5.], [40., 0., 5.]);
    let scene = EngagementScene::new(stock(), vec![support(&old, 1.)]).unwrap();
    for (a, b, right) in [
        ([38., 0.24, 5.], [2., 0.24, 5.], true),
        ([2., 0.24, 5.], [38., 0.24, 5.], false),
        ([2., -0.24, 5.], [38., -0.24, 5.], true),
    ] {
        let candidate = line(a, b);
        assert_eq!(
            scene
                .check_material_side(&candidate, 1., right, EngagementOptions::default(), &|| {
                    false
                })
                .unwrap()
                .status,
            EngagementStatus::Bounded
        );
        assert_eq!(
            scene
                .check_material_side(
                    &candidate,
                    1.,
                    !right,
                    EngagementOptions::default(),
                    &|| false
                )
                .unwrap()
                .status,
            EngagementStatus::Unresolved
        );
    }
}

#[test]
fn material_side_keeps_real_annular_holes_and_unknown_budget_outcomes() {
    let old = arc([0., 0., 5.], 2., 0., TAU);
    let scene = EngagementScene::new(stock(), vec![support(&old, 1.)]).unwrap();
    // Inner front of an annulus: fresh material is LEFT for CCW feed.
    let candidate = arc([0., 0., 5.], 1.85, 0., TAU);
    assert_eq!(
        scene
            .check_material_side(&candidate, 1., false, EngagementOptions::default(), &|| {
                false
            })
            .unwrap()
            .status,
        EngagementStatus::Bounded
    );
    assert_eq!(
        scene
            .check_material_side(&candidate, 1., true, EngagementOptions::default(), &|| {
                false
            })
            .unwrap()
            .status,
        EngagementStatus::Unresolved
    );
    let limited = EngagementOptions {
        max_work: 0,
        ..Default::default()
    };
    assert_eq!(
        scene
            .check_material_side(&candidate, 1., false, limited, &|| false)
            .unwrap()
            .status,
        EngagementStatus::Unresolved
    );
    assert!(
        scene
            .check_material_side(&candidate, 1., false, EngagementOptions::default(), &|| {
                true
            })
            .is_err()
    );
}
