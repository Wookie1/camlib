use camlib::{
    material::*,
    stock::{HeightBounds, Stock, StockSpec},
    surface::SurfaceMesh,
};
use std::sync::atomic::AtomicBool;
fn flat(x0: f64, x1: f64, z: f64, shift: [f64; 3]) -> Vec<[[f64; 3]; 3]> {
    let p = |x, y| [x + shift[0], y + shift[1], z + shift[2]];
    vec![
        [p(x0, -5.), p(x1, -5.), p(x1, 5.)],
        [p(x0, -5.), p(x1, 5.), p(x0, 5.)],
    ]
}
fn target(triangles: Vec<[[f64; 3]; 3]>, base: f64, scope: FinishingScope) -> SurfaceTarget {
    SurfaceTarget {
        version: 1,
        units: "mm".into(),
        id: "target".into(),
        revision: "1".into(),
        surface: SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: triangles.into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        },
        base_z_mm: base,
        source_enclosure: SurfaceEnclosure::Bounded,
        contact_witnesses: None,
        inner_patches: vec![],
        finishing: Some(scope),
    }
}
fn stock(shift: [f64; 3], height: f64) -> Stock {
    let spec = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [shift[0] - 0.1, shift[1] - 0.1, shift[2]],
        max_mm: [shift[0] + 0.1, shift[1] + 0.1, shift[2] + 6.],
        cell_size_mm: 0.1,
        max_cells: 100,
    };
    let mut snapshot = Stock::new(spec).unwrap().snapshot();
    snapshot.cells.fill(HeightBounds {
        lower_mm: height + shift[2],
        upper_mm: height + shift[2],
    });
    Stock::from_snapshot(snapshot).unwrap()
}
fn encloses(bounds: [f64; 2], value: f64) {
    assert!(
        bounds[0] <= value + 1e-9 && bounds[1] >= value - 1e-9,
        "{bounds:?} misses {value}"
    );
}
#[test]
fn depth_and_normal_allowance_partition_residual_without_hiding_gouges() {
    let scope = FinishingScope {
        floor_z_mm: 3.,
        tool_radius_mm: 1.,
        stock_to_leave_mm: 0.2,
        ..Default::default()
    };
    let mut d = target(flat(-5., 5., 2., [0.; 3]), 0., scope);
    let s = stock([0.; 3], 6.);
    let t = d.prepare(Options::default()).unwrap();
    let r = t.check_stock(&s, &AtomicBool::new(false)).unwrap();
    let f = r.finishing.as_ref().unwrap();
    assert!(r.complete && f.complete);
    encloses(r.residual.volume_mm3, 0.16);
    encloses(f.within_depth_exclusion.volume_mm3, 0.04);
    encloses(f.within_allowance.volume_mm3, 0.);
    encloses(f.in_scope_residual.volume_mm3, 0.12);
    assert!(f.reachable_residual.volume_mm3[0] > 0.119);
    d.finishing.as_mut().unwrap().floor_z_mm = 0.;
    let r = d
        .prepare(Options::default())
        .unwrap()
        .check_stock(&s, &AtomicBool::new(false))
        .unwrap();
    let f = r.finishing.unwrap();
    encloses(f.within_depth_exclusion.volume_mm3, 0.);
    encloses(f.within_allowance.volume_mm3, 0.008);
    encloses(f.in_scope_residual.volume_mm3, 0.152);
    assert_eq!(f.tool_limited_residual.definite_cells, 0);
    let s = stock([0.; 3], 1.);
    let scoped = d
        .prepare(Options::default())
        .unwrap()
        .check_stock(&s, &AtomicBool::new(false))
        .unwrap();
    d.finishing = None;
    let raw = d
        .prepare(Options::default())
        .unwrap()
        .check_stock(&s, &AtomicBool::new(false))
        .unwrap();
    assert!(scoped.gouge.definite_cells > 0);
    assert_eq!(scoped.gouge.volume_mm3, raw.gouge.volume_mm3);
    assert_eq!(scoped.residual.volume_mm3, raw.residual.volume_mm3);
}
#[test]
fn whole_center_domain_proves_ball_limited_slot_against_analytic_closing() {
    for shift in [[0.; 3], [100000., -200000., 1234.]] {
        let mut triangles = flat(-5., -0.6, 4., shift);
        triangles.extend(flat(0.6, 5., 4., shift));
        let scope = FinishingScope {
            floor_z_mm: shift[2],
            tool_radius_mm: 1.,
            access_error_mm: 0.02,
            max_refinement_depth: 5,
            ..Default::default()
        };
        let d = target(triangles, shift[2], scope);
        let s = stock(shift, 6.);
        let r = d
            .prepare(Options::default())
            .unwrap()
            .check_stock(&s, &AtomicBool::new(false))
            .unwrap();
        let f = r.finishing.unwrap();
        assert!(f.complete);
        assert!(f.inner_contact_available);
        assert!(f.tool_limited_residual.definite_cells > 0);
        assert!(f.reachable_residual.definite_cells > 0);
        // Infinite parallel walls at x=±0.6,Z4. A unit ball's optimal center is
        // X0: on this cell, moving off center raises wall contact faster than
        // it lowers the ball's bottom. K(x)=4.8-sqrt(1-x²), independent of Y.
        let inaccessible =
            4.8 * 0.04 - 0.2 * (0.1_f64 * (1. - 0.1_f64.powi(2)).sqrt() + 0.1_f64.asin());
        encloses(f.tool_limited_residual.volume_mm3, inaccessible);
        encloses(f.reachable_residual.volume_mm3, 0.24 - inaccessible);
        assert!(f.tool_limited_residual.volume_mm3[0] > 0.14);
        for finding in f.findings {
            let [x0, _, x1, _] = finding.xy_mm;
            let a = x0 - shift[0];
            let b = x1 - shift[0];
            let near = if a <= 0. && b >= 0. {
                0.
            } else {
                a.abs().min(b.abs())
            };
            let far = a.abs().max(b.abs());
            assert!(
                finding.attainable_height_mm[0]
                    <= shift[2] + 4.8 - (1. - near * near).sqrt() + 1e-8
            );
            assert!(
                finding.attainable_height_mm[1] >= shift[2] + 4.8 - (1. - far * far).sqrt() - 1e-8
            );
        }
    }
}
#[test]
fn uncertain_source_and_bounded_work_cannot_invent_inaccessibility_or_completion() {
    let mut triangles = flat(-5., -0.6, 4., [0.; 3]);
    triangles.extend(flat(0.6, 5., 4., [0.; 3]));
    let mut d = target(
        triangles,
        0.,
        FinishingScope {
            tool_radius_mm: 1.,
            ..Default::default()
        },
    );
    d.surface.surface_error_mm = 0.05;
    let s = stock([0.; 3], 6.);
    let r = d
        .prepare(Options::default())
        .unwrap()
        .check_stock(&s, &AtomicBool::new(false))
        .unwrap();
    let f = r.finishing.unwrap();
    assert!(f.complete);
    assert!(!f.inner_contact_available);
    assert_eq!(f.tool_limited_residual.definite_cells, 0);
    assert!(f.access_uncertain_cells > 0);
    // Cancellation after preparation/hash and during geometry/access work must
    // publish no report; retry on the same prepared target is deterministic.
    d.surface.surface_error_mm = 0.;
    let prepared = d.prepare(Options::default()).unwrap();
    let expected =
        serde_json::to_value(prepared.check_stock(&s, &AtomicBool::new(false)).unwrap()).unwrap();
    for stop in [2, 10, 40] {
        let polls = std::cell::Cell::new(0);
        let cancelled = || {
            polls.set(polls.get() + 1);
            polls.get() >= stop
        };
        assert!(prepared.check_stock(&s, &cancelled).is_err());
        assert_eq!(polls.get(), stop);
        assert_eq!(
            serde_json::to_value(prepared.check_stock(&s, &AtomicBool::new(false)).unwrap())
                .unwrap(),
            expected
        );
    }
    d.finishing.as_mut().unwrap().max_access_work = 1;
    let t = d.prepare(Options::default()).unwrap();
    let r = t.check_stock(&s, &AtomicBool::new(false)).unwrap();
    assert!(!r.complete);
    let f = r.finishing.unwrap();
    assert!(!f.complete);
    assert_eq!(f.classified_cells, 0);
    assert_eq!(f.in_scope_residual.assessment, Assessment::Incomplete);
    assert!(f.reachable_residual.volume_mm3[1] >= 0.24);
    assert!(t.check_stock(&s, &AtomicBool::new(true)).is_err());
    d.finishing.as_mut().unwrap().tool_radius_mm = f64::NAN;
    assert!(d.prepare(Options::default()).is_err());
}

#[test]
fn dense_faceted_access_completes_with_default_budget_and_analytic_residual_bounds() {
    // Dense facets over a flat top with a shallow roof: the exact analytic solid
    // is deliberately known independently of tessellation and contact traversal.
    let mut triangles = vec![];
    let p = |x: f64, y: f64| [x, y, 4. + 0.05 * x.abs()];
    for x in -40..40 {
        for y in -30..30 {
            let (x, y) = (x as f64 * 0.5, y as f64 * 0.5);
            triangles.extend([
                [p(x, y), p(x + 0.5, y), p(x + 0.5, y + 0.5)],
                [p(x, y), p(x + 0.5, y + 0.5), p(x, y + 0.5)],
            ]);
        }
    }
    let spec = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [-12., -8., 0.],
        max_mm: [12., 8., 8.],
        cell_size_mm: 0.1,
        max_cells: 40000,
    };
    let s = Stock::new(spec).unwrap();
    let mut d = target(
        triangles,
        0.,
        FinishingScope {
            floor_z_mm: 3.,
            tool_radius_mm: 2.,
            ..Default::default()
        },
    );
    // Outer-only CAD premise must stay outer-only even though this test knows
    // the true solid. Never create a lower contact proof from its cached point.
    d.surface.surface_error_mm = 0.005;
    let r = d
        .prepare(Options::default())
        .unwrap()
        .check_stock(&s, &|| false)
        .unwrap();
    let f = r.finishing.unwrap();
    assert!(
        r.complete && f.complete,
        "{} classified at {} work",
        f.classified_cells,
        f.access_work
    );
    assert_eq!(f.classified_cells, 38400);
    assert!(!f.inner_contact_available);
    assert_eq!(f.tool_limited_residual.definite_cells, 0);
    assert!(f.access_work <= 2_000_000);
    // Everywhere the radius2 ball can reach within .01mm of this gently sloped
    // convex roof. Remaining volume is integral(8-(4+0.05|x|)) = 1420.8mm³.
    encloses(f.in_scope_residual.volume_mm3, 1420.8);
    assert!(f.reachable_residual.volume_mm3[0] > 1410.);
    assert!(f.reachable_residual.volume_mm3[1] >= 1420.8 - 4.);
}

#[test]
fn stock_ceiling_eliminates_irrelevant_contact_without_inventing_inner_access() {
    let mut d = target(
        flat(-5., 5., 6., [0.; 3]),
        0.,
        FinishingScope {
            tool_radius_mm: 1.,
            floor_z_mm: 3.,
            max_access_work: 1,
            ..Default::default()
        },
    );
    d.surface.surface_error_mm = 0.01;
    let r = d
        .prepare(Options::default())
        .unwrap()
        .check_stock(&stock([0.; 3], 6.), &|| false)
        .unwrap();
    let f = r.finishing.unwrap();
    assert!(r.complete && f.complete);
    assert_eq!(f.access_work, 0);
    assert_eq!(f.classified_cells, 4);
    assert!(!f.inner_contact_available);
    assert_eq!(f.tool_limited_residual.definite_cells, 0);
    // Classified is NOT clear: the outer-only source premise still leaves
    // possible in-scope residual and access uncertainty in all four cells.
    assert_eq!(f.in_scope_residual.assessment, Assessment::PossibleExcess);
    assert_eq!(f.reachable_residual.assessment, Assessment::PossibleExcess);
    assert_eq!(f.access_uncertain_cells, 4);
    assert!(f.reachable_residual.volume_mm3[1] >= 0.12);
}

#[test]
fn stock_below_scope_needs_no_access_search_but_keeps_raw_gouge_and_depth_residual() {
    let d = target(
        flat(-5., 5., 2., [0.; 3]),
        0.,
        FinishingScope {
            floor_z_mm: 3.,
            tool_radius_mm: 1.,
            max_access_work: 1,
            ..Default::default()
        },
    );
    for height in [1., 2.5] {
        let s = stock([0.; 3], height);
        let r = d
            .prepare(Options::default())
            .unwrap()
            .check_stock(&s, &|| false)
            .unwrap();
        let f = r.finishing.unwrap();
        assert!(r.complete && f.complete);
        assert_eq!(f.access_work, 0);
        assert_eq!(f.in_scope_residual.assessment, Assessment::WithinTolerance);
        assert_eq!(f.reachable_residual.assessment, Assessment::WithinTolerance);
        assert_eq!(
            f.tool_limited_residual.assessment,
            Assessment::WithinTolerance
        );
        if height < 2. {
            assert_eq!(r.gouge.assessment, Assessment::DefiniteExcess);
        } else {
            encloses(f.within_depth_exclusion.volume_mm3, 0.02);
        }
    }
}

#[test]
fn zero_allowance_is_exact_even_with_source_uncertainty_or_unexamined_cells() {
    for enclosure in [
        SurfaceEnclosure::Bounded,
        SurfaceEnclosure::Inscribed,
        SurfaceEnclosure::Offset,
    ] {
        for budget in [1, 2_000_000] {
            let mut d = target(
                flat(-5., 5., 2., [0.; 3]),
                0.,
                FinishingScope {
                    floor_z_mm: 0.,
                    tool_radius_mm: 1.,
                    max_access_work: budget,
                    ..Default::default()
                },
            );
            d.surface.surface_error_mm = 0.05;
            d.source_enclosure = enclosure;
            let r = d
                .prepare(Options::default())
                .unwrap()
                .check_stock(&stock([0.; 3], 6.), &|| false)
                .unwrap();
            let f = r.finishing.unwrap();
            assert_eq!(f.within_allowance.volume_mm3, [0.; 2]);
            assert_eq!(f.within_allowance.assessment, Assessment::WithinTolerance);
            assert_eq!(
                f.within_allowance.definite_cells + f.within_allowance.possible_only_cells,
                0
            );
            if budget == 1 {
                assert!(!r.complete && !f.complete);
                assert_eq!(f.reachable_residual.assessment, Assessment::Incomplete);
                assert!(f.reachable_residual.volume_mm3[1] >= 0.24);
            } else {
                assert!(r.complete && f.complete);
            }
        }
    }
}

#[test]
fn normal_allowance_is_applied_before_clipping_to_the_stock_analysis_box() {
    for shift in [[0.; 3], [100000., -200000., 1234.]] {
        for enclosure in [
            SurfaceEnclosure::Bounded,
            SurfaceEnclosure::Inscribed,
            SurfaceEnclosure::Offset,
        ] {
            for allowance in [0.5, 2.5, 3.5, 5.] {
                let mut d = target(
                    flat(-5., 5., 2., shift),
                    shift[2] - 2.,
                    FinishingScope {
                        floor_z_mm: shift[2] + 5.,
                        tool_radius_mm: 1.,
                        stock_to_leave_mm: allowance,
                        ..Default::default()
                    },
                );
                d.surface.surface_error_mm = 0.05;
                d.source_enclosure = enclosure;
                let spec = StockSpec {
                    version: 1,
                    units: "mm".into(),
                    min_mm: [shift[0] - 0.1, shift[1] - 0.1, shift[2] + 5.],
                    max_mm: [shift[0] + 0.1, shift[1] + 0.1, shift[2] + 11.],
                    cell_size_mm: 0.1,
                    max_cells: 100,
                };
                let mut snapshot = Stock::new(spec).unwrap().snapshot();
                snapshot.cells.fill(HeightBounds {
                    lower_mm: shift[2] + 6.,
                    upper_mm: shift[2] + 6.,
                });
                let r = d
                    .prepare(Options::default())
                    .unwrap()
                    .check_stock(&Stock::from_snapshot(snapshot).unwrap(), &|| false)
                    .unwrap();
                let f = r.finishing.unwrap();
                assert!(r.complete && f.complete);
                // True surface is the plane Z2: its normal offset is exactly
                // Z(2+a). Only material inside Z[5,6] is in the analysis box.
                let allowed = ((2. + allowance).min(6.) - 5.).max(0.) * 0.04;
                let residual = (6. - (2. + allowance).max(5.)).max(0.) * 0.04;
                encloses(f.within_allowance.volume_mm3, allowed);
                encloses(f.in_scope_residual.volume_mm3, residual);
                if allowance < 3. {
                    assert!(f.in_scope_residual.volume_mm3[0] > 0.0399);
                }
                for finding in f.findings {
                    for bounds in [
                        finding.allowed_target_height_mm,
                        finding.attainable_height_mm,
                        finding.reachable_residual_height_mm,
                        finding.tool_limited_residual_height_mm,
                    ] {
                        assert!(bounds[0] <= bounds[1], "inverted {bounds:?}");
                    }
                }
            }
        }
    }
}

#[test]
fn finding_uncertainty_counts_include_truncated_cells_and_preserve_inner_premises() {
    let mut d = target(
        flat(-5., 5., 2., [0.; 3]),
        0.,
        FinishingScope {
            tool_radius_mm: 1.,
            ..Default::default()
        },
    );
    d.surface.surface_error_mm = 0.05;
    let mut snapshot = stock([0.; 3], 6.).snapshot();
    snapshot.cells.fill(HeightBounds {
        lower_mm: 5.8,
        upper_mm: 6.,
    });
    let s = Stock::from_snapshot(snapshot).unwrap();
    let r = d
        .prepare(Options {
            max_findings: 1,
            ..Default::default()
        })
        .unwrap()
        .check_stock(&s, &|| false)
        .unwrap();
    let f = r.finishing.unwrap();
    let u = f.uncertainty.unwrap();
    assert!(f.complete && f.findings_truncated);
    assert_eq!(f.findings.len(), 1);
    assert_eq!(u.finding_cells, 4);
    assert_eq!(u.missing_inner_contact_cells, 4);
    assert_eq!(u.stock_height_interval_cells, 4);
    assert_eq!(u.access_height_interval_cells, 4);
    assert_eq!(u.cell_exceeds_ball_footprint_cells, 0);
    let flags = f.findings[0].uncertainty.unwrap();
    assert!(
        flags.missing_inner_contact && flags.stock_height_interval && flags.access_height_interval
    );
    assert_eq!(f.tool_limited_residual.definite_cells, 0);
    // An explicit inner premise changes contact availability; no flag may
    // silently upgrade the original bounded-source claim to that premise.
    d.source_enclosure = SurfaceEnclosure::Inscribed;
    let f = d
        .prepare(Options::default())
        .unwrap()
        .check_stock(&s, &|| false)
        .unwrap()
        .finishing
        .unwrap();
    assert!(f.inner_contact_available);
    assert_eq!(f.uncertainty.unwrap().missing_inner_contact_cells, 0);
    assert!(
        f.findings
            .iter()
            .all(|f| !f.uncertainty.unwrap().missing_inner_contact)
    );
}

#[test]
fn source_point_witnesses_tighten_access_without_inventing_a_mesh_inner_volume() {
    let cancel = AtomicBool::new(false);
    for shift in [[0.; 3], [10000., -20000., 1000.]] {
        for allowance in [0., 0.2] {
            let scope = FinishingScope {
                floor_z_mm: shift[2],
                tool_radius_mm: 1.,
                stock_to_leave_mm: allowance,
                max_refinement_depth: 4,
                ..Default::default()
            };
            let mut definition = target(flat(-5., 5., 2., shift), shift[2], scope);
            definition.surface.surface_error_mm = 0.005;
            let s = stock(shift, 3.);
            let original = definition.prepare(Options::default()).unwrap();
            let before = original.check_stock(&s, &cancel).unwrap();
            definition.contact_witnesses = Some(ContactWitnesses {
                points_mm: (-8..=8)
                    .flat_map(|i| {
                        (-8..=8).map(move |j| {
                            [
                                shift[0] + f64::from(i) * 0.25,
                                shift[1] + f64::from(j) * 0.25,
                                shift[2] + 2.,
                            ]
                        })
                    })
                    .collect(),
                error_mm: 1e-6,
                source: serde_json::json!({"premise": "points on analytic plane"}),
            });
            let prepared = definition.prepare(Options::default()).unwrap();
            assert_ne!(original.digest(), prepared.digest());
            let after = prepared.check_stock(&s, &cancel).unwrap();
            let f = after.finishing.as_ref().unwrap();
            assert!(after.complete && f.complete && f.inner_contact_available);
            assert!(
                f.reachable_residual.volume_mm3[1]
                    < before.finishing.unwrap().reachable_residual.volume_mm3[1] * 0.6
            );
            for finding in &f.findings {
                encloses(finding.attainable_height_mm, shift[2] + 2. + allowance);
            }
            assert_eq!(after.gouge.volume_mm3, before.gouge.volume_mm3);
            assert_eq!(after.residual.volume_mm3, before.residual.volume_mm3);
            assert!(f.access_model.contains("sparse points"));
            assert_eq!(
                f.uncertainty.as_ref().unwrap().missing_inner_contact_cells,
                0
            );
            definition.contact_witnesses.as_mut().unwrap().error_mm = f64::NAN;
            assert!(definition.prepare(Options::default()).is_err());
        }
    }
}

#[test]
fn contact_witness_budget_and_cancellation_remain_explicit() {
    let mut definition = target(flat(-5., 5., 2., [0.; 3]), 0., FinishingScope::default());
    definition.surface.surface_error_mm = 0.005;
    definition.contact_witnesses = Some(ContactWitnesses {
        points_mm: vec![[0., 0., 2.]],
        error_mm: 0.,
        source: serde_json::Value::Null,
    });
    assert!(
        definition
            .prepare_cancel(Options::default(), &AtomicBool::new(true))
            .is_err()
    );
    definition.finishing.as_mut().unwrap().max_access_work = 1;
    let r = definition
        .prepare(Options::default())
        .unwrap()
        .check_stock(&stock([0.; 3], 3.), &AtomicBool::new(false))
        .unwrap();
    let f = r.finishing.unwrap();
    assert!(!f.complete);
    assert!(f.unexamined_cells > 0);
    assert!(f.reachable_residual.volume_mm3[1] > 0.1);
    definition
        .contact_witnesses
        .as_mut()
        .unwrap()
        .points_mm
        .clear();
    assert!(definition.prepare(Options::default()).is_err());
}
