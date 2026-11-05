use super::*;

fn contains(ranges: &[[f64; 2]], theta: f64) -> bool {
    ranges.iter().any(|r| theta >= r[0] && theta <= r[1])
}
fn measure(ranges: &[[f64; 2]]) -> f64 {
    ranges.iter().map(|r| r[1] - r[0]).sum()
}

// Original event-sort/midpoint classifier retained independently of the new
// interval intersection algorithm. Probes corroborate, never certify a sweep.
fn reference(mut roots: Vec<f64>, pad: f64, inside: impl Fn(f64) -> bool) -> Vec<[f64; 2]> {
    roots.extend([0., TAU]);
    roots.sort_unstable_by(f64::total_cmp);
    roots.dedup();
    let mut out = vec![];
    for pair in roots.windows(2) {
        let [a, b] = [pair[0], pair[1]];
        let mid = (a + b) / 2.;
        if mid > a && mid < b && inside(mid) {
            let lo = a + if a == 0. { 0. } else { pad };
            let hi = b - if b == TAU { 0. } else { pad };
            if lo < hi {
                out.push([lo, hi]);
            }
        }
    }
    out
}
fn line_roots(h: [f64; 3], roots: &mut Vec<f64>) {
    let amplitude = h[0].hypot(h[1]);
    if amplitude == 0. {
        return;
    }
    let k = h[2] / amplitude;
    if !(-1. ..=1.).contains(&k) {
        return;
    }
    let phase = h[1].atan2(h[0]);
    let half = k.acos();
    roots.extend([
        (phase - half).rem_euclid(TAU),
        (phase + half).rem_euclid(TAU),
    ]);
}
fn circle_roots(p: [f64; 2], r: f64, c: [f64; 2], radius: f64, roots: &mut Vec<f64>) {
    let d = [c[0] - p[0], c[1] - p[1]];
    let length = d[0].hypot(d[1]);
    if length == 0. || radius <= 0. {
        return;
    }
    let k = (length * length + r * r - radius * radius) / (2. * length * r);
    if !(-1. ..=1.).contains(&k) {
        return;
    }
    let phase = d[1].atan2(d[0]);
    let half = k.acos();
    roots.extend([
        (phase - half).rem_euclid(TAU),
        (phase + half).rem_euclid(TAU),
    ]);
}
fn check_ranges<const N: usize>(set: &Intervals<N>, inside: impl Fn(f64) -> bool) {
    let mut end = 0.;
    for range in set.as_slice() {
        assert!(
            range[0] >= end && range[0] < range[1] && range[1] <= TAU,
            "{set:?}"
        );
        end = range[1];
        for f in [0., 0.1, 0.5, 0.9, 1.] {
            let theta = range[0] + (range[1] - range[0]) * f;
            assert!(
                inside(theta),
                "admitted nonempty geometry at {theta}: {set:?}"
            );
        }
    }
    for i in 0..2048 {
        let theta = TAU * i as f64 / 2048.;
        if contains(set.as_slice(), theta) {
            assert!(inside(theta), "unsafe angle {theta}");
        }
    }
}

#[test]
fn analytic_halfplanes_and_rotated_rectangles_match_independent_classification() {
    let pad = 1e-5;
    for scale in [0.001, 2., 10000.] {
        for i in 0..32 {
            let theta = TAU * i as f64 / 32.;
            let (s, c) = theta.sin_cos();
            for ratio in [-1.2, -0.95, -0.5, 0., 0.5, 0.95, 1.2] {
                let h = [scale * c, scale * s, scale * ratio];
                let set = halfplanes([h], pad).unwrap();
                let inside = |a: f64| h[0] * a.cos() + h[1] * a.sin() < h[2];
                check_ranges(&set, inside);
                let mut roots = vec![];
                line_roots(h, &mut roots);
                let old = reference(roots, pad, inside);
                assert!(measure(set.as_slice()) + 64. * pad >= measure(&old));
            }
            let constraints = [
                [scale * c, scale * s, scale * 0.8],
                [-scale * c, -scale * s, scale * 0.6],
                [-scale * s, scale * c, scale * 0.9],
                [scale * s, -scale * c, scale * 0.7],
            ];
            let set = halfplanes(constraints, pad).unwrap();
            let inside = |a: f64| {
                constraints
                    .iter()
                    .all(|h| h[0] * a.cos() + h[1] * a.sin() < h[2])
            };
            check_ranges(&set, inside);
            let mut roots = vec![];
            for h in constraints {
                line_roots(h, &mut roots);
            }
            let old = reference(roots, pad, inside);
            assert!(measure(set.as_slice()) + 64. * pad >= measure(&old));
        }
    }
}

#[test]
fn disks_cover_seams_translations_scales_and_squared_distance_cancellation() {
    let pad = 1e-5;
    for scale in [0.001, 1., 10000.] {
        for shift in [0., 123.456, 1e6] {
            let p = [shift, shift];
            for offset in [0.2, 0.9, 1.4, 2.4] {
                let c = [shift + scale * offset, shift - scale * offset * 0.3];
                let r = scale;
                let radius = scale * 1.7;
                if let Some(set) = disk(p, r, c, radius, pad) {
                    check_ranges(&set, |a| {
                        let d = [p[0] - c[0] + r * a.cos(), p[1] - c[1] + r * a.sin()];
                        d[0].hypot(d[1]) < radius
                    });
                }
            }
        }
    }
    // Squared terms dwarf the cutter radius. Reserve must follow those terms,
    // not only their cancelled result or the halfplane's small amplitude.
    for radius in [1e6 - 0.000001, 1e6, 1e6 + 0.000001] {
        if let Some(set) = disk([0., 0.], 1e-6, [1e6, 0.], radius, pad) {
            check_ranges(&set, |a| {
                (1e-6 * a.cos() - 1e6).hypot(1e-6 * a.sin()) < radius
            });
        }
    }
}

fn arc_inside(p: [f64; 2], r: f64, tube: ArcTube, theta: f64) -> bool {
    let v = [r * theta.cos(), r * theta.sin()];
    if tube.sweep.abs() < TAU
        && [tube.start, tube.end]
            .into_iter()
            .any(|q| (p[0] - q[0] + v[0]).hypot(p[1] - q[1] + v[1]) < tube.clearance)
    {
        return true;
    }
    let d = [p[0] - tube.center[0] + v[0], p[1] - tube.center[1] + v[1]];
    let travel = if tube.sweep > 0. {
        (d[1].atan2(d[0]) - tube.angle).rem_euclid(TAU)
    } else {
        (tube.angle - d[1].atan2(d[0])).rem_euclid(TAU)
    };
    (tube.sweep.abs() >= TAU || travel > tube.shrink && travel < tube.sweep.abs() - tube.shrink)
        && (d[0].hypot(d[1]) - tube.radius).abs() < tube.clearance
}

#[test]
fn annular_minor_major_and_full_arc_sets_match_the_original_sector_predicate() {
    let pad = 1e-5;
    let mut fast = 0;
    for scale in [0.01, 1., 10000.] {
        for sign in [-1., 1.] {
            for sweep in [0.000001, 0.4, PI, 4.8, TAU - 1e-6, TAU, 2. * TAU] {
                for clearance in [0.4, 1.2, 2.0, 3.1] {
                    for rotation in [0.0_f64, 0.47, 3.2, 6.1] {
                        let center = [123.456 * scale, -42.0 * scale];
                        let radius = 2. * scale;
                        let sweep = sign * sweep;
                        let tube = ArcTube {
                            start: [
                                center[0] + radius * rotation.cos(),
                                center[1] + radius * rotation.sin(),
                            ],
                            end: [
                                center[0] + radius * (rotation + sweep).cos(),
                                center[1] + radius * (rotation + sweep).sin(),
                            ],
                            center,
                            radius,
                            angle: rotation,
                            sweep,
                            clearance: clearance * scale,
                            shrink: 1e-7,
                        };
                        let p = [center[0] + scale * 0.7, center[1] - scale * 0.3];
                        let r = scale * 1.1;
                        let sector = ArcSector::new(tube.angle, tube.sweep);
                        for cached in [None, Some(&sector)] {
                            let Some(set) = arc_tube(p, r, tube, pad, cached) else {
                                continue;
                            };
                            fast += 1;
                            let inside = |a| arc_inside(p, r, tube, a);
                            check_ranges(&set, inside);
                            let mut roots = vec![];
                            circle_roots(p, r, center, radius + tube.clearance, &mut roots);
                            if radius > tube.clearance {
                                circle_roots(p, r, center, radius - tube.clearance, &mut roots);
                            }
                            if sweep.abs() < TAU {
                                for q in [tube.start, tube.end] {
                                    circle_roots(p, r, q, tube.clearance, &mut roots);
                                }
                                for angle in [
                                    rotation + sweep.signum() * tube.shrink,
                                    rotation + sweep - sweep.signum() * tube.shrink,
                                ] {
                                    let n = [-angle.sin(), angle.cos()];
                                    line_roots(
                                        [
                                            r * n[0],
                                            r * n[1],
                                            (center[0] - p[0]) * n[0] + (center[1] - p[1]) * n[1],
                                        ],
                                        &mut roots,
                                    );
                                }
                            }
                            let old = reference(roots, pad, inside);
                            assert!(
                                measure(set.as_slice()) + 128. * pad >= measure(&old),
                                "lost clear region: {tube:?}, cached={}",
                                cached.is_some()
                            );
                        }
                    }
                }
            }
        }
    }
    assert!(fast > 400, "most geometry fell back: {fast}");
}

#[test]
fn ambiguous_tangencies_invalid_inputs_and_capacity_fail_closed_to_fallback() {
    for h in [
        [1., 0., 1.],
        [1., 0., -1.],
        [f64::NAN, 0., 0.],
        [1., 0., f64::INFINITY],
    ] {
        assert!(halfplanes([h], 1e-5).is_none());
    }
    assert!(halfplanes([[1., 0., 0.]], 0.).is_none());
    assert!(halfplanes([[1., 0., 0.]; 5], 1e-5).is_none());
    assert_eq!(
        halfplanes([[0., 0., 1.]], 1e-5).unwrap().as_slice(),
        &[[0., TAU]]
    );
    assert!(
        halfplanes([[0., 0., 0.]], 1e-5)
            .unwrap()
            .as_slice()
            .is_empty()
    );
    let mut set = Intervals::<TUBE>::empty();
    for i in 0..TUBE {
        set.push(i as f64 * 0.1, i as f64 * 0.1 + 0.01).unwrap();
    }
    assert!(set.push(2., 2.1).is_none());
}

#[test]
fn wrap_tau_matches_rem_euclid_bit_for_bit() {
    let mut values = vec![
        0.,
        -0.,
        TAU,
        -TAU,
        2. * TAU,
        -2. * TAU,
        PI,
        -PI,
        3. * PI,
        f64::MIN_POSITIVE,
        -f64::MIN_POSITIVE,
        1e300,
        -1e300,
    ];
    for edge in [0., TAU, -TAU, 2. * TAU] {
        let mut up = edge;
        let mut down = edge;
        for _ in 0..64 {
            up = f64::from_bits(if up >= 0. {
                up.to_bits() + 1
            } else {
                up.to_bits() - 1
            });
            down = f64::from_bits(if down > 0. {
                down.to_bits() - 1
            } else {
                down.to_bits() + 1
            });
            values.extend([up, down]);
        }
    }
    let mut state = 0x9e37_79b9_7f4a_7c15_u64;
    for _ in 0..1_000_000 {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        values.push((state >> 11) as f64 / (1u64 << 53) as f64 * 6. * TAU - 3. * TAU);
    }
    for x in values {
        assert_eq!(wrap_tau(x).to_bits(), x.rem_euclid(TAU).to_bits(), "{x:e}");
    }
}
