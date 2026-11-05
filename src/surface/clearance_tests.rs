//! Exhaustive original facet-distance oracle, independent of the new bounds.
use super::*;
use std::cell::Cell;

fn prepare(triangles: Vec<[Vec3; 3]>, floor: f64) -> PreparedSurface {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: triangles.into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
    }
    .prepare(floor, 1e-5, &|| false)
    .unwrap()
}

// Exhaustive original leaf order, without ANY nearest-node bounds or pruning.
// Preserve the old base/inside/min_top/strict-update semantics, including ties.
fn exhaustive_distance(
    surface: &PreparedSurface,
    p: Vec3,
    min_top: Option<f64>,
) -> Option<PointDistance> {
    let mut best = if min_top.is_none() {
        (p[2] - surface.floor).max(0.).powi(2)
    } else {
        f64::INFINITY
    };
    let mut witness = DistanceWitness::Base;
    for node in surface.nodes.iter().rev().filter(|n| n.children.is_none()) {
        for index in node.range.clone() {
            if min_top.is_some_and(|h| surface.triangles[index].max[2] < h) {
                continue;
            }
            let distance = surface.prism_distance_sq(p, index, &mut SurfaceWork::default());
            if distance < best {
                best = distance;
                witness = DistanceWitness::Prism(index);
            }
            if best == 0. {
                return Some(PointDistance {
                    distance_mm: 0.,
                    witness,
                });
            }
        }
    }
    best.is_finite().then_some(PointDistance {
        distance_mm: best.sqrt(),
        witness,
    })
}

#[test]
fn nearest_first_preserves_exhaustive_distances_inside_base_filters_and_ties() {
    let witness = |w: DistanceWitness| match w {
        DistanceWitness::Base => None,
        DistanceWitness::Prism(i) => Some(i),
    };
    for shift in [[0., 0., 0.], [100000., -200000., 1234.]] {
        let point = |x, y, z| [x + shift[0], y + shift[1], z + shift[2]];
        let mut triangles = vec![];
        for i in 0..20 {
            let x = i as f64;
            let height = 2. + (i % 3) as f64;
            triangles.push([
                point(x, 0., height),
                point(x + 1., 0., height),
                point(x, 1., height + 0.5),
            ]);
        }
        triangles.push(triangles[3]); // exact duplicate creates genuine witness ties
        triangles.push([point(2., 2., 1.), point(2., 2., 4.), point(4., 2., 1.)]);
        let surface = prepare(triangles, shift[2]);
        for x in [-1., 0., 0.5, 2., 3.5, 8., 16., 22.] {
            for y in [-1., 0., 0.5, 2., 3.] {
                for z in [-0.1, 0., 0.3, 2., 5.] {
                    for cutoff in [None, Some(2.5 + shift[2]), Some(20. + shift[2])] {
                        let p = point(x, y, z);
                        let expected = exhaustive_distance(&surface, p, cutoff);
                        let actual = surface
                            .distance(p, cutoff, &mut SurfaceWork::default(), &|| false)
                            .unwrap();
                        match (actual, expected) {
                            (Some(a), Some(b)) => {
                                assert_eq!(a.distance_mm, b.distance_mm);
                                assert_eq!(witness(a.witness), witness(b.witness));
                            }
                            (None, None) => {}
                            _ => panic!("nearest traversal changed empty/filled membership"),
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn nearest_first_keeps_final_budget_and_cancellation_checks() {
    let triangles = (0..24)
        .map(|i| {
            let x = i as f64;
            [[x, 0., 2.], [x + 1., 0., 2.], [x, 1., 3.]]
        })
        .collect();
    let surface = prepare(triangles, 0.);
    let p = [8.5, 1.5, 3.];
    let polls = Cell::new(0);
    let mut work = SurfaceWork::default();
    surface
        .point_distance(p, &mut work, &|| {
            polls.set(polls.get() + 1);
            false
        })
        .unwrap();
    let required = work.nodes + work.distance_tests + work.contact_tests;
    let mut exhausted = SurfaceWork {
        nodes: 20_000_000 - required + 1,
        ..Default::default()
    };
    // Work is counted, never capped: a large prior count still completes.
    assert!(surface.point_distance(p, &mut exhausted, &|| false).is_ok());
    assert!(exhausted.nodes + exhausted.distance_tests + exhausted.contact_tests >= 20_000_000);
    for limit in [1, polls.get() / 2, polls.get()] {
        let calls = Cell::new(0);
        assert!(
            surface
                .point_distance(p, &mut SurfaceWork::default(), &|| {
                    calls.set(calls.get() + 1);
                    calls.get() >= limit
                })
                .unwrap_err()
                .contains("cancelled")
        );
    }
}

fn exhaustive(surface: &PreparedSurface, query: [Vec3; 3], radius: f64) -> bool {
    if query.iter().any(|p| p[2] < surface.floor + radius) {
        return false;
    }
    for i in 0..surface.triangles.len() {
        // Interior points have zero solid distance; checking just a skin
        // would miss query triangles wholly below a facet's top.
        if surface.prism_distance_sq(query[0], i, &mut SurfaceWork::default()) < radius * radius {
            return false;
        }
        let p = surface.triangles[i].p;
        let mut facets = vec![p];
        for j in 0..3 {
            let a = p[j];
            let b = p[(j + 1) % 3];
            let af = [a[0], a[1], surface.floor];
            let bf = [b[0], b[1], surface.floor];
            facets.extend([[a, b, bf], [a, bf, af]]);
        }
        for facet in facets {
            let normal = cross(sub(facet[1], facet[0]), sub(facet[2], facet[0]));
            if dot(normal, normal) <= 1e-30 {
                continue;
            }
            for j in 0..3 {
                if segment_triangle_sq(query[j], query[(j + 1) % 3], facet) < radius * radius
                    || segment_triangle_sq(facet[j], facet[(j + 1) % 3], query) < radius * radius
                {
                    return false;
                }
            }
        }
    }
    true
}

#[test]
fn whole_triangle_plane_bounds_match_exhaustive_disconnected_sloped_and_vertical_solids() {
    let mut clear = 0;
    let mut blocked = 0;
    for shift in [
        [0., 0., 0.],
        [100000., -200000., 1234.],
        [-200000., 100000., -1234.],
    ] {
        for disconnected in [false, true] {
            let point = |x, y, z| [x + shift[0], y + shift[1], z + shift[2]];
            let mut triangles = vec![];
            for y in [0., 6.] {
                for x in [0., 6.] {
                    let width = if disconnected { 2. } else { 6. };
                    let top = |x, y| point(x, y, 4. + 0.25 * x + 0.1 * y);
                    let [a, b, c, d] = [
                        top(x, y),
                        top(x + width, y),
                        top(x + width, y + width),
                        top(x, y + width),
                    ];
                    triangles.extend([[a, b, c], [a, c, d]]);
                }
            }
            triangles.push([point(3., 3., 1.), point(3., 3., 7.), point(3., 5., 7.)]);
            let surface = prepare(triangles, shift[2]);
            for x in [-1., 1., 3., 5., 7., 9.] {
                for y in [-1., 1., 3., 5., 7., 9.] {
                    for z in [2., 5.5, 8., 13.] {
                        for span in [0.4, 6.] {
                            let query = [
                                point(x, y, z),
                                point(x + span, y, z + 0.2),
                                point(x, y + span, z - 0.1),
                            ];
                            for radius in [0.25, 1.] {
                                let expected = exhaustive(&surface, query, radius);
                                let got = surface
                                    .triangle_clear(
                                        query,
                                        radius,
                                        &mut SurfaceWork::default(),
                                        &|| false,
                                    )
                                    .unwrap();
                                assert_eq!(
                                    got, expected,
                                    "shift={shift:?},disconnected={disconnected},query={query:?},radius={radius}"
                                );
                                if got { clear += 1 } else { blocked += 1 }
                            }
                        }
                    }
                }
            }
        }
    }
    assert!(clear > 500 && blocked > 200);
}

#[test]
fn plane_encloses_entire_prisms_and_near_contact_uses_original_facet_checks() {
    for shift in [[0., 0., 0.], [100000., -200000., 1234.]] {
        let point = |x, y, z| [x + shift[0], y + shift[1], z + shift[2]];
        let surface = prepare(
            vec![[point(0., 0., 4.), point(8., 0., 4.), point(0., 8., 4.)]],
            shift[2],
        );
        let source = &surface.triangles[0];
        let plane = PlaneBound::new(source.normal, source.p.into_iter(), source.p[0]).unwrap();
        for height in [0., 2., 4.] {
            assert!(!plane.separates([point(1., 1., height); 3], 1.));
        }
        for height in [5f64.next_down(), 5., 5f64.next_up(), 5. - 1e-8, 5. + 1e-8] {
            let query = [
                point(1., 1., height),
                point(2., 1., height),
                point(1., 2., height),
            ];
            assert_eq!(
                surface
                    .triangle_clear(query, 1., &mut SurfaceWork::default(), &|| false)
                    .unwrap(),
                exhaustive(&surface, query, 1.)
            );
        }
        // All query corners are far from the small peak; its interior and
        // perimeter intersect the filled query. A whole-region bound cannot
        // be replaced by clearance at the three query vertices.
        let peak = prepare(
            vec![[
                point(-0.1, -0.1, 4.),
                point(0.1, -0.1, 4.),
                point(0., 0.1, 4.),
            ]],
            shift[2],
        );
        let crossing = [point(-4., -3., 4.), point(4., -3., 4.), point(0., 4., 4.)];
        for p in crossing {
            assert!(
                peak.point_distance(p, &mut SurfaceWork::default(), &|| false)
                    .unwrap()
                    .distance_mm
                    > 1.
            );
        }
        assert!(
            !peak
                .triangle_clear(crossing, 1., &mut SurfaceWork::default(), &|| false)
                .unwrap()
        );
    }
}

#[test]
fn query_directed_finite_prism_plane_saves_facet_work_and_preserves_limits() {
    let surface = prepare(vec![[[0., 0., 4.], [2., 0., 4.], [0., 2., 4.]]], 0.);
    let query = [[3., 3., 2.5], [3.1, 3., 2.5], [3., 3.1, 2.5]];
    let source = &surface.triangles[0];
    let top = PlaneBound::new(source.normal, source.p.into_iter(), source.p[0]).unwrap();
    assert!(!top.separates(query, 2.), "below an infinite top plane");
    assert!(source.separates_triangle(query, 2., surface.floor));
    assert!(exhaustive(&surface, query, 2.));
    let mut point_work = SurfaceWork::default();
    assert!(
        surface
            .point_distance(query[0], &mut point_work, &|| false)
            .unwrap()
            .distance_mm
            > 2.
    );
    let mut whole_work = SurfaceWork::default();
    assert!(
        surface
            .triangle_clear(query, 2., &mut whole_work, &|| false)
            .unwrap()
    );
    assert!(whole_work.nodes <= point_work.nodes);
    assert!(whole_work.distance_tests > 0);
    assert!(whole_work.distance_tests < point_work.distance_tests);
    // Work is counted, never capped.
    assert!(
        surface
            .triangle_clear(
                query,
                2.,
                &mut SurfaceWork {
                    nodes: 20_000_000,
                    ..SurfaceWork::default()
                },
                &|| false
            )
            .is_ok()
    );
    assert!(
        surface
            .triangle_clear(query, 2., &mut SurfaceWork::default(), &|| true)
            .is_err()
    );
    let polls = Cell::new(0);
    let cancel = || {
        polls.set(polls.get() + 1);
        polls.get() >= 3
    };
    assert!(
        surface
            .triangle_clear(query, 2., &mut SurfaceWork::default(), &cancel)
            .unwrap_err()
            .contains("cancelled")
    );
    assert_eq!(polls.get(), 3);
    let mut invalid = query;
    invalid[1][0] = f64::NAN;
    assert!(
        surface
            .triangle_clear(invalid, 1., &mut SurfaceWork::default(), &|| false)
            .is_err()
    );
    let mut outside = query;
    outside[0][0] = 1e6 + 1.;
    assert!(
        surface
            .triangle_clear(outside, 1., &mut SurfaceWork::default(), &|| false)
            .is_err()
    );
    for radius in [0., -1., f64::INFINITY, f64::NAN] {
        assert!(
            surface
                .triangle_clear(query, radius, &mut SurfaceWork::default(), &|| false)
                .is_err()
        );
    }
}

#[test]
fn full_segment_plane_pruning_matches_exhaustive_prism_sweeps() {
    for shift in [[0., 0., 0.], [100000., -200000., 1234.]] {
        let p = |x, y, z| [x + shift[0], y + shift[1], z + shift[2]];
        let surface = prepare(
            vec![
                [p(0., 0., 4.), p(8., 0., 5.), p(0., 8., 6.)],
                [p(9., 0., 3.), p(12., 0., 3.), p(9., 8., 3.)],
                [p(5., 2., 1.), p(5., 6., 7.), p(5., 2., 7.)],
            ],
            shift[2],
        );
        for x in [-1., 1., 4., 7., 10.] {
            for z in [1., 3., 5., 7., 10.] {
                for radius in [0.1, 0.5, 1.5] {
                    let a = p(x, -1., z);
                    let b = p(x + 2., 9., z + 0.25);
                    let mut expected = a[2].min(b[2]) >= surface.floor + radius;
                    for i in 0..surface.triangles.len() {
                        if surface.prism_distance_sq(a, i, &mut SurfaceWork::default())
                            < radius * radius
                        {
                            expected = false;
                        }
                        let t = surface.triangles[i].p;
                        if segment_triangle_sq(a, b, t) < radius * radius {
                            expected = false;
                        }
                        for j in 0..3 {
                            let c = t[j];
                            let d = t[(j + 1) % 3];
                            let cf = [c[0], c[1], surface.floor];
                            let df = [d[0], d[1], surface.floor];
                            for side in [[c, d, df], [c, df, cf]] {
                                let normal = cross(sub(side[1], side[0]), sub(side[2], side[0]));
                                if dot(normal, normal) > 1e-30
                                    && segment_triangle_sq(a, b, side) < radius * radius
                                {
                                    expected = false;
                                }
                            }
                        }
                    }
                    let actual = surface
                        .segment_clear(a, b, radius, &mut SurfaceWork::default(), &|| false)
                        .unwrap();
                    assert_eq!(actual, expected, "{a:?} {b:?} r={radius}");
                }
            }
        }
    }
}

// Original unpruned contact kernel: independent of node boxes/planes, branch
// order and early rejection. Every source facet and every edge is evaluated.
fn exhaustive_contact(surface: &PreparedSurface, xy: [f64; 2], radius: f64) -> f64 {
    let mut height = surface.floor + radius;
    for triangle in &surface.triangles {
        if let Some(z) = triangle.face_contact(xy, radius) {
            height = height.max(z);
        }
        for i in 0..3 {
            let a = triangle.p[i];
            let b = triangle.p[(i + 1) % 3];
            let dx = b[0] - a[0];
            let dy = b[1] - a[1];
            let dz = b[2] - a[2];
            let l2 = dx * dx + dy * dy;
            let t = if l2 == 0. {
                if dz > 0. { 1. } else { 0. }
            } else {
                let projection = ((xy[0] - a[0]) * dx + (xy[1] - a[1]) * dy) / l2;
                let perpendicular = ((xy[0] - a[0]) * dy - (xy[1] - a[1]) * dx).powi(2) / l2;
                if perpendicular > radius * radius {
                    continue;
                }
                (projection
                    + dz * ((radius * radius - perpendicular) / (l2 * (l2 + dz * dz)))
                        .max(0.)
                        .sqrt())
                .clamp(0., 1.)
            };
            let point = add(a, mul(sub(b, a), t));
            let distance2 = (xy[0] - point[0]).powi(2) + (xy[1] - point[1]).powi(2);
            if distance2 <= radius * radius {
                height = height.max(point[2] + (radius * radius - distance2).max(0.).sqrt());
            }
        }
    }
    height
}

#[test]
fn grouped_contact_plane_and_order_match_every_facet_on_nonconvex_surfaces() {
    let mut exhaustive_tests = 0;
    let mut indexed_tests = 0;
    for shift in [
        [0., 0., 0.],
        [100000., -200000., 1234.],
        [-100000., 200000., -1234.],
    ] {
        let point = |x: f64, y: f64| {
            [
                x + shift[0],
                y + shift[1],
                shift[2] + 5. + 0.15 * x + (x * 0.7).sin() + (y * 0.4).cos(),
            ]
        };
        let mut triangles = vec![];
        for i in 0..12 {
            for j in 0..12 {
                if i == 5 || j == 7 {
                    continue;
                }
                let x = f64::from(i);
                let y = f64::from(j);
                triangles.extend([
                    [point(x, y), point(x + 1., y), point(x, y + 1.)],
                    [point(x + 1., y), point(x + 1., y + 1.), point(x, y + 1.)],
                ]);
            }
        }
        triangles.push([
            [shift[0] + 3., shift[1] + 1., shift[2] + 1.],
            [shift[0] + 3., shift[1] + 6., shift[2] + 9.],
            [shift[0] + 3., shift[1] + 1., shift[2] + 9.],
        ]);
        let surface = prepare(triangles, shift[2]);
        for x in -1..14 {
            for y in -1..14 {
                for radius in [0.1, 0.8, 2.] {
                    let xy = [
                        shift[0] + f64::from(x) + 0.137,
                        shift[1] + f64::from(y) + 0.241,
                    ];
                    let reference = exhaustive_contact(&surface, xy, radius);
                    let mut work = SurfaceWork::default();
                    let actual = surface
                        .contact(xy, radius, &mut work, &|| false)
                        .unwrap()
                        .center_z;
                    assert!(
                        (actual - reference).abs() < 2e-8,
                        "{xy:?} r={radius}: {actual} vs {reference}"
                    );
                    exhaustive_tests += surface.triangles.len();
                    indexed_tests += work.contact_tests;
                }
            }
        }
    }
    assert!(indexed_tests < exhaustive_tests / 3);
}

#[test]
fn horizontal_interval_union_bounds_seams_real_gaps_and_translations() {
    for shift in [[0., 0., 0.], [100000., -200000., 1234.]] {
        for gap in [0., 0.02, 2., 8.] {
            let mut triangles = vec![];
            for (low, high) in [(-1., 10. - gap / 2.), (10. + gap / 2., 21.)] {
                for i in 0..40 {
                    let x0 = low + (high - low) * f64::from(i) / 40.;
                    let x1 = low + (high - low) * f64::from(i + 1) / 40.;
                    let p = |x, y| [x + shift[0], y + shift[1], 5. + shift[2]];
                    triangles.extend([
                        [p(x0, 0.), p(x1, 0.), p(x0, 10.)],
                        [p(x1, 0.), p(x1, 10.), p(x0, 10.)],
                    ]);
                }
            }
            let surface = prepare(triangles, shift[2]);
            let a = [shift[0], shift[1] + 5., shift[2] + 7.001];
            let b = [shift[0] + 20., shift[1] + 5., shift[2] + 7.001];
            let bound = surface
                .horizontal_excess_bound(a, b, 2., 0.005, &mut SurfaceWork::default(), &|| false)
                .unwrap();
            if gap <= 0.02 {
                let bound = bound.expect("flat triangle seams should not force short cuts");
                let exact_excess = 0.001 + 2. - (4. - (gap / 2.).powi(2)).sqrt();
                assert!(
                    bound + 1e-10 >= exact_excess && bound <= 0.005,
                    "{gap} {bound} {exact_excess}"
                );
                assert!(
                    surface
                        .segment_clear(a, b, 2., &mut SurfaceWork::default(), &|| false)
                        .unwrap()
                );
            } else {
                assert!(
                    bound.is_none(),
                    "real missing material must not be filled by a coverage claim"
                );
            }
            assert!(
                surface
                    .horizontal_excess_bound(a, b, 2., 0.005, &mut SurfaceWork::default(), &|| true)
                    .is_err()
            );
        }
    }
}

#[test]
fn segment_boxes_fall_back_at_contact_and_enclose_downward_prisms() {
    for shift in [[0., 0., 0.], [100000., -200000., 1234.]] {
        let p = |x, y, z| [x + shift[0], y + shift[1], z + shift[2]];
        let surface = prepare(
            vec![
                [p(0., 0., 4.), p(1., 0., 4.), p(1., 1., 4.)],
                [p(0., 0., 4.), p(1., 1., 4.), p(0., 1., 4.)],
            ],
            shift[2],
        );
        // Whole vertical segments below the top face. Their nearest source
        // point is on a prism side, not on any of the input top triangles.
        for x in [2f64.next_down(), 2., 2f64.next_up(), 2. - 1e-8, 2. + 1e-8] {
            let a = p(x, 0.5, 1.5);
            let b = p(x, 0.5, 2.5);
            let distance = a[0] - p(1., 0., 0.)[0];
            assert_eq!(
                surface
                    .segment_clear(a, b, 1., &mut SurfaceWork::default(), &|| false)
                    .unwrap(),
                distance >= 1.,
            );
            if distance <= 1. + 1e-10 {
                assert!(!segment_box_separates(
                    a,
                    b,
                    p(0., 0., 4.),
                    p(1., 1., 4.),
                    shift[2],
                    1.
                ));
            }
        }
    }
}
