use camlib::{Fixture, Verdict, fixtures, geometry::Region, session::Session};

#[test]
fn exact_rectangle_oracle_across_dense_subdivision() {
    // Independent analytic answer for an axis-aligned rectangle with a rectangular
    // island. Exercises inside, island, and outside queries, including ray vertices.
    fn distance_to_box(p: [f64; 2], min: [f64; 2], max: [f64; 2]) -> f64 {
        let dx = (min[0] - p[0]).max(p[0] - max[0]).max(0.0);
        let dy = (min[1] - p[1]).max(p[1] - max[1]).max(0.0);
        if dx > 0.0 || dy > 0.0 {
            dx.hypot(dy)
        } else {
            (p[0] - min[0])
                .min(max[0] - p[0])
                .min(p[1] - min[1])
                .min(max[1] - p[1])
        }
    }
    for subdivisions in [1, 16, 128] {
        let region = Region::prepare(&fixtures::rectangle_island(subdivisions)).unwrap();
        for x in -2..83 {
            for y in -2..63 {
                let p = [x as f64, y as f64];
                let outer = distance_to_box(p, [0., 0.], [80., 60.]);
                let hole = distance_to_box(p, [30., 20.], [50., 40.]);
                let inside_outer = (0..=80).contains(&x) && (0..=60).contains(&y);
                let inside_hole = (30..=50).contains(&x) && (20..=40).contains(&y);
                let expected = if inside_outer && !inside_hole {
                    outer.min(hole)
                } else {
                    -outer.min(hole)
                };
                assert!(
                    (region.clearance(p).unwrap() - expected).abs() < 1e-11,
                    "{p:?}"
                );
            }
        }
    }
}

#[test]
fn narrow_channel_closes_at_cutter_diameter() {
    let fixture = fixtures::narrow_channel();
    let region = Region::prepare(&fixture).unwrap();
    for c in fixture.checks {
        assert_eq!(
            region.query(c.point, c.radius_mm).unwrap().verdict,
            c.expected
        );
    }
}

#[test]
fn tangency_and_boundary_approximation_are_uncertain() {
    let mut f = fixtures::rectangle_island(1);
    f.boundary_error_mm = 0.01;
    let r = Region::prepare(&f).unwrap();
    assert_eq!(r.query([3., 10.], 3.).unwrap().verdict, Verdict::Uncertain);
    assert_eq!(
        r.query([3.005, 10.], 3.).unwrap().verdict,
        Verdict::Uncertain
    );
    assert_eq!(r.query([3.02, 10.], 3.).unwrap().verdict, Verdict::Allowed);
    assert_eq!(r.query([2.98, 10.], 3.).unwrap().verdict, Verdict::Blocked);
    assert_eq!(r.query([0., 10.], 0.).unwrap().verdict, Verdict::Uncertain);
}

#[test]
fn orientation_and_translation_do_not_change_membership() {
    let base = fixtures::narrow_channel();
    let original = Region::prepare(&base).unwrap();
    let mut changed = base.clone();
    changed.outer.reverse();
    for p in &mut changed.outer {
        p[0] += 999_000.;
        p[1] -= 999_000.;
    }
    let moved = Region::prepare(&changed).unwrap();
    for p in [[30., 10.], [30., 5.], [5., 5.], [20., 8.]] {
        assert_eq!(
            original.clearance(p).unwrap(),
            moved.clearance([p[0] + 999_000., p[1] - 999_000.]).unwrap()
        );
    }
}

#[test]
fn indexed_queries_match_scan_on_concavity_and_multiple_islands() {
    let mut f = fixtures::narrow_channel();
    f.holes = vec![
        vec![[2., 2.], [4., 2.], [4., 4.], [2., 4.]],
        vec![[45., 2.], [48., 2.], [48., 5.], [45., 5.]],
    ];
    let r = Region::prepare(&f).unwrap();
    for p in fixtures::probes(r.bounds(), 10_000) {
        assert!((r.clearance(p).unwrap() - r.clearance_linear(p).unwrap()).abs() < 1e-10);
    }
}

#[test]
fn malformed_topology_is_rejected() {
    let mut f = fixtures::rectangle_island(1);
    f.outer = vec![[0., 0.], [80., 60.], [80., 0.], [0., 60.]];
    assert!(Region::prepare(&f).is_err(), "self crossing");
    f = fixtures::rectangle_island(1);
    f.holes[0][0] = [0., 0.];
    assert!(Region::prepare(&f).is_err(), "touching hole");
    f = fixtures::rectangle_island(1);
    f.holes[0] = vec![[90., 0.], [95., 0.], [95., 5.], [90., 5.]];
    assert!(Region::prepare(&f).is_err(), "outside hole");
    f = fixtures::rectangle_island(1);
    f.holes
        .push(vec![[35., 25.], [40., 25.], [40., 30.], [35., 30.]]);
    assert!(Region::prepare(&f).is_err(), "nested hole");
    f = fixtures::rectangle_island(1);
    f.outer = vec![[0., 0.], [40., 0.], [20., 0.], [80., 60.], [0., 60.]];
    assert!(Region::prepare(&f).is_err(), "backtracking adjacent edges");
    f = fixtures::rectangle_island(1);
    f.outer.push(f.outer[0]);
    assert!(Region::prepare(&f).is_err(), "duplicated closing point");
}

#[test]
fn invalid_numbers_and_unsupported_contracts_are_rejected() {
    for value in [f64::NAN, f64::INFINITY, 1_000_001., 1e-300] {
        let mut f = fixtures::rectangle_island(1);
        f.outer[0][0] = value;
        assert!(Region::prepare(&f).is_err());
    }
    let mut f = fixtures::rectangle_island(1);
    f.units = "inch".into();
    assert!(Region::prepare(&f).is_err());
    f = fixtures::rectangle_island(1);
    f.boundary_error_mm = -1.;
    assert!(Region::prepare(&f).is_err());
    let r = Region::prepare(&fixtures::rectangle_island(1)).unwrap();
    for radius in [-1., f64::NAN, f64::INFINITY] {
        assert!(r.query([10., 10.], radius).is_err());
    }
}

#[test]
fn radius_and_feed_changes_reuse_geometry_and_distances() {
    let mut session = Session::default();
    session.load(&fixtures::narrow_channel()).unwrap();
    session.probe(&[[30., 10.]]).unwrap();
    assert_eq!(session.classify(1.9).unwrap()[0].verdict, Verdict::Allowed);
    session.set_feed(1200.).unwrap();
    let first = session.classify(1.9).unwrap();
    let second = session.classify(1.9).unwrap();
    assert!(std::sync::Arc::ptr_eq(&first, &second));
    assert_eq!(session.counters.classifications, 1);
    assert_eq!(session.counters.classification_cache_hits, 2);
    assert_eq!(session.classify(2.1).unwrap()[0].verdict, Verdict::Blocked);
    assert_eq!(session.counters.geometry_preparations, 1);
    assert_eq!(session.counters.clearance_batches, 1);
    assert_eq!(session.counters.clearance_points, 1);
    session.load(&fixtures::rectangle_island(1)).unwrap();
    assert!(
        session.classify(1.).is_err(),
        "new geometry invalidates distances"
    );
}

#[test]
fn failed_updates_preserve_valid_session() {
    let mut session = Session::default();
    session.load(&fixtures::narrow_channel()).unwrap();
    session.probe(&[[30., 10.]]).unwrap();
    let mut invalid = fixtures::narrow_channel();
    invalid.outer.clear();
    assert!(session.load(&invalid).is_err());
    assert!(session.probe(&[[f64::NAN, 0.]]).is_err());
    assert_eq!(session.classify(1.).unwrap()[0].verdict, Verdict::Allowed);
    assert_eq!(session.counters.geometry_preparations, 1);
    assert_eq!(session.counters.clearance_batches, 1);
}

#[test]
fn exported_cadquery_fixtures_satisfy_checks() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures");
    let mut count = 0;
    for path in std::fs::read_dir(root).unwrap() {
        let path = path.unwrap().path();
        if path.extension().and_then(|s| s.to_str()) != Some("json") {
            continue;
        }
        let f: Fixture = camlib::load_fixture(&path).unwrap();
        let r = Region::prepare(&f).unwrap();
        for check in &f.checks {
            assert_eq!(
                r.query(check.point, check.radius_mm).unwrap().verdict,
                check.expected,
                "{}",
                f.name
            );
        }
        count += 1;
    }
    assert!(count >= 1);
}

#[test]
fn circle_island_agrees_with_independent_analytic_clearance_within_error_band() {
    let f = camlib::load_fixture(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("fixtures/cadquery-round-island.json"),
    )
    .unwrap();
    let region = Region::prepare(&f).unwrap();
    for p in fixtures::probes(region.bounds(), 5000) {
        let outer = (p[0] - 20.).min(60. - p[0]).min(p[1] - 15.).min(45. - p[1]);
        let island = (p[0] - 40.).hypot(p[1] - 30.) - 5.;
        let exact = if island < 0. {
            island
        } else {
            outer.min(island)
        };
        let result = region.query(p, 3.).unwrap();
        assert!((result.signed_clearance_mm - exact).abs() <= result.guard_mm);
        match result.verdict {
            Verdict::Allowed => assert!(exact > 3.),
            Verdict::Blocked => assert!(exact < 3.),
            Verdict::Uncertain => {}
        }
    }
}
