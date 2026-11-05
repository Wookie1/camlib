use super::*;
use sha2::{Digest, Sha256};
use std::{hint::black_box, path::Path, time::Instant};

fn transform(p: Point, angle: f64, shift: Point) -> Point {
    let (s, c) = angle.sin_cos();
    [
        shift[0] + c * p[0] - s * p[1],
        shift[1] + s * p[0] + c * p[1],
    ]
}
fn fixture(
    curved: bool,
    angle: f64,
    shift: Point,
    allowance: f64,
    error: f64,
    tolerance: f64,
) -> PocketPlan {
    let mut m: crate::stream::StreamManifest = serde_json::from_str(include_str!(
        "../../fixtures/operations/adaptive-pocket.json"
    ))
    .unwrap();
    let Operation::AdaptivePocket(adaptive) = m.operations.remove(0) else {
        panic!("adaptive fixture");
    };
    let mut op = adaptive.pocket;
    if curved {
        let mut region = PlanarRegion::circle(
            "curved source with hole and second component",
            [13., 13.],
            12.,
        )
        .unwrap();
        region.components[0].holes.push(Boundary {
            vertices: vec![[10., 13., -1.], [16., 13., -1.]],
        });
        region.components.push(crate::planar::Component {
            outer: Boundary {
                vertices: vec![[30., 2., 0.], [44., 2., 0.], [44., 18., 0.], [30., 18., 0.]],
            },
            holes: vec![],
        });
        op.region = region;
    }
    op.region.boundary_error_mm = error;
    op.radial_allowance_mm = allowance;
    op.tolerance_mm = tolerance;
    op.entry = None;
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for boundary in op
        .region
        .components
        .iter_mut()
        .flat_map(|c| std::iter::once(&mut c.outer).chain(&mut c.holes))
    {
        for v in &mut boundary.vertices {
            let p = transform([v[0], v[1]], angle, shift);
            v[0] = p[0];
            v[1] = p[1];
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }
    // Pad for circular extrema as well as all endpoint coordinates. Stock
    // preparation is outside any measured source-query interval.
    m.stock.height_mut().unwrap().min_mm = [lo[0] - 20., lo[1] - 20., 0.];
    m.stock.height_mut().unwrap().max_mm = [hi[0] + 20., hi[1] + 20., 10.];
    m.stock.height_mut().unwrap().cell_size_mm = 0.5;
    PocketPlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        [lo[0] - 21., lo[1] - 21., 12.],
        1,
        &|| false,
    )
    .unwrap()
}
fn strokes(count: usize, angle: f64, shift: Point) -> Vec<Stroke> {
    (0..count)
        .map(|i| {
            let p = [
                2. + (i % 16) as f64 * 2.5,
                2. + ((i / 16) % 16) as f64 * 1.5,
            ];
            let heading = i as f64 * 0.37;
            let (end, arc) = if i % 3 == 0 {
                let length = 0.2 + (i % 11) as f64 * 0.7;
                (
                    [p[0] + length * heading.cos(), p[1] + length * heading.sin()],
                    None,
                )
            } else {
                let radius = 0.25 + (i % 7) as f64 * 0.6;
                let center = [p[0] - radius * heading.cos(), p[1] - radius * heading.sin()];
                let sweep = [PI / 2., -PI / 2., PI, -PI, TAU, -TAU, 2. * TAU][(i / 3) % 7];
                (
                    [
                        center[0] + radius * (heading + sweep).cos(),
                        center[1] + radius * (heading + sweep).sin(),
                    ],
                    Some((center, sweep)),
                )
            };
            Stroke {
                a: transform(p, angle, shift),
                b: transform(end, angle, shift),
                arc: arc.map(|(c, s)| (transform(c, angle, shift), s)),
            }
        })
        .collect()
}
fn reference(plan: &PocketPlan, stroke: Stroke, radius: f64) -> bool {
    plan.guide_radius_bounds(stroke)
        .is_some_and(|(lo, _)| lo >= radius)
}

#[test]
fn source_admission_matches_existing_bounds_for_curves_frames_and_guarded_boundaries() {
    for curved in [false, true] {
        for (angle, shift) in [
            (0., [0., 0.]),
            (0.47, [123., -52.]),
            (1.1, [-1000., 2000.]),
            (0.31, [500_000., -300_000.]),
        ] {
            for (allowance, error, tolerance) in [(0., 0., 1e-5), (0.1, 1e-4, 0.0005)] {
                let plan = fixture(curved, angle, shift, allowance, error, tolerance);
                for stroke in strokes(256, angle, shift) {
                    let mut radii = vec![0.05, 0.5, 1., 2., 4.];
                    if let Some((lo, _)) = plan.guide_radius_bounds(stroke)
                        && lo > 0.
                    {
                        radii.extend([lo.next_down(), lo, lo.next_up()]);
                    }
                    for radius in radii {
                        assert_eq!(
                            plan.guide_tube_fits(stroke, radius),
                            reference(&plan, stroke, radius),
                            "curved={curved} angle={angle} shift={shift:?} stroke={stroke:?} radius={radius}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn complete_boundary_checks_refuse_a_curve_whose_endpoints_and_midpoint_are_clear() {
    let plan = fixture(true, 0., [0., 0.], 0., 0., 1e-5);
    let stroke = Stroke::line([3., 13.], [42., 13.]);
    for p in [stroke.a, stroke_point(stroke, 0.5), stroke.b] {
        assert!(plan.source.query(p).unwrap().signed_clearance_mm.unwrap() > 0.5);
    }
    // Crosses both the round hole and the gap between disconnected components.
    assert!(!plan.guide_tube_fits(stroke, 0.5));
    assert!(!reference(&plan, stroke, 0.5));
    for radius in [0., -1., f64::NAN, f64::INFINITY] {
        assert!(!plan.guide_tube_fits(Stroke::line([3., 13.], [4., 13.]), radius));
    }
}

#[derive(Serialize)]
struct Sample {
    wall_ms: f64,
    queries: usize,
    admitted: usize,
}
fn run(plans: &[PocketPlan], queries: &[(usize, Stroke, f64)], admission: bool) -> Sample {
    let start = Instant::now();
    let mut admitted = 0;
    for _ in 0..16 {
        for &(index, stroke, radius) in queries {
            let stroke = black_box(stroke);
            let radius = black_box(radius);
            let plan = &plans[index];
            admitted += usize::from(black_box(if admission {
                plan.guide_tube_fits(stroke, radius)
            } else {
                reference(plan, stroke, radius)
            }));
        }
    }
    Sample {
        wall_ms: start.elapsed().as_secs_f64() * 1000.,
        queries: 16 * queries.len(),
        admitted,
    }
}
fn stats(samples: &[Sample]) -> serde_json::Value {
    let mut values = samples.iter().map(|s| s.wall_ms).collect::<Vec<_>>();
    values.sort_by(f64::total_cmp);
    serde_json::json!({"samples":values.len(),"p50_ms":values[values.len()/2],"p95_ms":values[values.len()*95/100],"max_ms":values[values.len()-1]})
}
fn digest(path: &Path) -> String {
    format!("{:x}", Sha256::digest(std::fs::read(path).unwrap()))
}

// Normal suite: decision equivalence only. The explicit isolated native command
// enables timing; no public CAM API/feature or ignored test is added for a bench.
#[test]
fn native_source_admission_measurement() {
    let plans = [
        fixture(false, 0., [0., 0.], 0.1, 1e-7, 1e-5),
        fixture(true, 0.47, [100., -50.], 0.1, 1e-7, 1e-5),
    ];
    let mut queries = Vec::new();
    for (index, (angle, shift)) in [(0., [0., 0.]), (0.47, [100., -50.])]
        .into_iter()
        .enumerate()
    {
        queries.extend(
            strokes(2048, angle, shift)
                .into_iter()
                .map(|s| (index, s, 2.)),
        );
    }
    let mut expected = 0;
    for &(index, stroke, radius) in &queries {
        let old = reference(&plans[index], stroke, radius);
        assert_eq!(plans[index].guide_tube_fits(stroke, radius), old);
        expected += usize::from(old);
    }
    assert!(expected > 0 && expected < queries.len());
    let Some(output) = std::env::var_os("CAMLIB_SOURCE_ADMISSION_MEASUREMENT") else {
        return;
    };
    assert!(
        !black_box(cfg!(debug_assertions)),
        "release-only measurement"
    );
    let output = Path::new(&output);
    assert!(!output.exists(), "preserve prior report");
    let input = serde_json::json!({"operations":plans.iter().map(|p|&p.operation).collect::<Vec<_>>(),
        "queries":queries.iter().map(|(i,s,r)|serde_json::json!({"plan":i,"curve":s.curve(5.),"required_radius_mm":r})).collect::<Vec<_>>()});
    let input_sha = format!("{:x}", Sha256::digest(serde_json::to_vec(&input).unwrap()));
    run(&plans, &queries, false);
    run(&plans, &queries, true);
    let (mut old, mut new) = (vec![], vec![]);
    for i in 0..21 {
        if i % 2 == 0 {
            old.push(run(&plans, &queries, false));
            new.push(run(&plans, &queries, true));
        } else {
            new.push(run(&plans, &queries, true));
            old.push(run(&plans, &queries, false));
        }
    }
    assert!(old.iter().chain(&new).all(|s| s.admitted == 16 * expected));
    let old_stats = stats(&old);
    let new_stats = stats(&new);
    let ratio = new_stats["p95_ms"].as_f64().unwrap() / old_stats["p95_ms"].as_f64().unwrap();
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let source_hashes = [
        "Cargo.toml",
        "Cargo.lock",
        "src/pocket.rs",
        "src/pocket/admission_tests.rs",
        "src/adaptive_front.rs",
        "fixtures/operations/adaptive-pocket.json",
    ]
    .into_iter()
    .map(|name| (name, digest(&root.join(name))))
    .collect::<std::collections::BTreeMap<_, _>>();
    let report = serde_json::json!({"scope":"Release native private source queries: old two-bound API versus lower-only whole-guide boolean admission, same prepared planar regions/curves/radii. Includes actual midpoint/BVH/error/minimum boundary work and boolean consumption. Excludes source/tool/stock preparation, generation/stock/CAD/Python/UI/posts/controller/physical cutting. Equality probes are not independent sweep qualification.",
        "contract":"One excluded warmup per method,21sequential samples each with alternating order,4096queries*16repeats/sample, one native test thread, no owned concurrent checks/builds/benchmarks",
        "cpu":std::fs::read_to_string("/proc/cpuinfo").unwrap().lines().find(|l|l.starts_with("model name")).unwrap(),
        "compiler":String::from_utf8(std::process::Command::new("rustc").arg("-Vv").output().unwrap().stdout).unwrap(),
        "executable_sha256":digest(&std::env::current_exe().unwrap()),"source_sha256":source_hashes,
        "input":input,"input_sha256":input_sha,"admitted_per_batch":expected,
        "reference":old_stats,"admission":new_stats,"p95_ratio":ratio,
        "gates":{"admission_p95_at_least_20_percent_lower":ratio<=0.8,"all_decisions_identical":true},
        "raw_reference":old,"raw_admission":new});
    std::fs::write(output, serde_json::to_vec_pretty(&report).unwrap()).unwrap();
    println!(
        "{}",
        serde_json::json!({"reference":report["reference"],"admission":report["admission"],"p95_ratio":ratio,"gates":report["gates"]})
    );
}

#[test]
fn source_approval_reuses_only_actual_complete_proofs_from_the_same_owner() {
    let source = fixture(false, 0., [0., 0.], 0., 0., 1e-5);
    let curve = Curve::Line {
        start: [5., 5., 8.],
        end: [6., 5., 8.],
    };
    let approval = source
        .approve_guide_tube(&curve, 0.5)
        .expect("interior complete tube");
    assert!(source.guide_tube_fits(Stroke::line([5., 5.], [6., 5.]), 0.5));
    assert!(approval.matches(&source, &curve, 0.5, 1e-5));
    assert!(!approval.matches(&source, &curve, 0.6, 1e-5));
    assert!(!approval.matches(&source, &curve, 0.5, 2e-5));
    for changed in [
        Curve::Line {
            start: [5., 5., 7.],
            end: [6., 5., 7.],
        },
        Curve::Line {
            start: [5., 5., 8.],
            end: [6.1, 5., 8.],
        },
        Curve::Line {
            start: [6., 5., 8.],
            end: [5., 5., 8.],
        },
    ] {
        assert!(!approval.matches(&source, &changed, 0.5, 1e-5));
    }
    let foreign = fixture(false, 0., [0., 0.], 0., 0., 1e-5);
    assert!(!approval.matches(&foreign, &curve, 0.5, 1e-5));
    let outside = Curve::Line {
        start: [-100., -100., 8.],
        end: [-90., -100., 8.],
    };
    assert!(source.approve_guide_tube(&outside, 0.5).is_none());
    let crossing = Curve::Line {
        start: [5., 5., 8.],
        end: [40., 5., 8.],
    };
    assert!(source.approve_guide_tube(&crossing, 0.5).is_none());
    let curved = fixture(true, 0., [0., 0.], 0., 0., 1e-5);
    let arc = Curve::Arc {
        start: [20., 13., 8.],
        end: [13., 20., 8.],
        center: [13., 13., 8.],
        plane: Plane::Xy,
        sweep_rad: FRAC_PI_2,
    };
    let arc_approval = curved.approve_guide_tube(&arc, 0.5).unwrap();
    assert!(arc_approval.matches(&curved, &arc, 0.5, 1e-5));
    let mut other_sweep = arc.clone();
    if let Curve::Arc { sweep_rad, .. } = &mut other_sweep {
        *sweep_rad -= TAU;
    }
    assert!(!arc_approval.matches(&curved, &other_sweep, 0.5, 1e-5));
    let mut other_plane = arc;
    if let Curve::Arc { plane, .. } = &mut other_plane {
        *plane = Plane::Yz;
    }
    assert!(curved.approve_guide_tube(&other_plane, 0.5).is_none());
    let moved = source;
    assert!(approval.matches(&moved, &curve, 0.5, 1e-5));
    drop(moved);
    let replacement = fixture(false, 0., [0., 0.], 0., 0., 1e-5);
    assert!(!approval.matches(&replacement, &curve, 0.5, 1e-5));
}
