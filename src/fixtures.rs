//! Deterministic workloads. Dense rings subdivide straight edges so that all
//! scales describe the same geometry and complexity is the only changing input.
use crate::geometry::Bounds;
use crate::{Check, Fixture, Point, Verdict};

pub fn rectangle_island(subdivisions: usize) -> Fixture {
    let outer = subdivide(&[[0., 0.], [80., 0.], [80., 60.], [0., 60.]], subdivisions);
    let holes = vec![subdivide(
        &[[30., 20.], [50., 20.], [50., 40.], [30., 40.]],
        subdivisions,
    )];
    Fixture {
        version: 1,
        units: "mm".into(),
        name: format!("island_{}_edges", 8 * subdivisions),
        outer,
        holes,
        boundary_error_mm: 0.0,
        source: serde_json::json!({"kind":"synthetic", "generator":"rectangle_island_v1"}),
        checks: vec![
            Check {
                point: [10., 10.],
                radius_mm: 3.,
                expected: Verdict::Allowed,
            },
            Check {
                point: [40., 30.],
                radius_mm: 3.,
                expected: Verdict::Blocked,
            },
            Check {
                point: [-1., 10.],
                radius_mm: 0.,
                expected: Verdict::Blocked,
            },
            Check {
                point: [3., 10.],
                radius_mm: 3.,
                expected: Verdict::Uncertain,
            },
            Check {
                point: [27., 30.],
                radius_mm: 3.,
                expected: Verdict::Uncertain,
            },
        ],
    }
}

pub fn narrow_channel() -> Fixture {
    Fixture {
        version: 1,
        units: "mm".into(),
        name: "narrow_channel".into(),
        outer: vec![
            [0., 0.],
            [20., 0.],
            [20., 8.],
            [40., 8.],
            [40., 0.],
            [60., 0.],
            [60., 20.],
            [40., 20.],
            [40., 12.],
            [20., 12.],
            [20., 20.],
            [0., 20.],
        ],
        holes: vec![],
        boundary_error_mm: 0.0,
        source: serde_json::json!({"kind":"synthetic"}),
        checks: vec![
            Check {
                point: [30., 10.],
                radius_mm: 1.9,
                expected: Verdict::Allowed,
            },
            Check {
                point: [30., 10.],
                radius_mm: 2.,
                expected: Verdict::Uncertain,
            },
            Check {
                point: [30., 10.],
                radius_mm: 2.1,
                expected: Verdict::Blocked,
            },
            Check {
                point: [30., 5.],
                radius_mm: 0.,
                expected: Verdict::Blocked,
            },
        ],
    }
}

pub fn subdivide(ring: &[Point], n: usize) -> Vec<Point> {
    assert!(n > 0);
    let mut points = Vec::with_capacity(ring.len() * n);
    for (i, &a) in ring.iter().enumerate() {
        let b = ring[(i + 1) % ring.len()];
        for j in 0..n {
            let t = j as f64 / n as f64;
            points.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
        }
    }
    points
}

/// Repeatable dyadic fractions: benchmark runs use exactly the same probes.
pub fn probes(bounds: Bounds, n: usize) -> Vec<Point> {
    let mut seed = 0x825a_fef4_3141_5926_u64;
    let mut next = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 11) as f64 / (1_u64 << 53) as f64
    };
    (0..n)
        .map(|_| {
            let x = next();
            let y = next();
            [
                bounds.min[0] + x * (bounds.max[0] - bounds.min[0]),
                bounds.min[1] + y * (bounds.max[1] - bounds.min[1]),
            ]
        })
        .collect()
}
