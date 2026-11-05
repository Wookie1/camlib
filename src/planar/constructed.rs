//! Bounded cleanup of generated linear boundaries, never authored input.
//! Collapsing AB by moving B to A sweeps only triangle ABC and displaces the
//! boundary by at most |AB|. An empty swept triangle preserves the embedding;
//! full ordinary region validation still follows. Curved regions only admit
//! removal of exact duplicate points joined by a zero-bulge segment: no arc or
//! displaced boundary is approximated by that identity operation. A redundant
//! closed spur may also be removed when independently retained straight segments
//! enclose its entire line/arc path within a reported displacement budget.
use super::{Component, Point};
use robust::{Coord, orient2d};
mod curved;
pub(super) use curved::normalize as normalize_short_curves;

fn orient(a: Point, b: Point, c: Point) -> f64 {
    orient2d(
        Coord { x: a[0], y: a[1] },
        Coord { x: b[0], y: b[1] },
        Coord { x: c[0], y: c[1] },
    )
}
fn on(p: Point, a: Point, b: Point) -> bool {
    orient(a, b, p) == 0. && (0..2).all(|k| p[k] >= a[k].min(b[k]) && p[k] <= a[k].max(b[k]))
}
fn intersects(a: Point, b: Point, c: Point, d: Point, fixed: &[Point]) -> bool {
    let side = [
        orient(a, b, c),
        orient(a, b, d),
        orient(c, d, a),
        orient(c, d, b),
    ];
    if side[0].signum() != side[1].signum()
        && side[2].signum() != side[3].signum()
        && side.iter().all(|s| *s != 0.)
    {
        return true;
    }
    for (p, u, v) in [(a, c, d), (b, c, d), (c, a, b), (d, a, b)] {
        if on(p, u, v) && !(fixed.contains(&p) && (p == a || p == b) && (p == c || p == d)) {
            return true;
        }
    }
    false
}
fn in_triangle(p: Point, t: [Point; 3]) -> bool {
    if orient(t[0], t[1], t[2]) == 0. {
        return false;
    }
    let s = [
        orient(t[0], t[1], p),
        orient(t[1], t[2], p),
        orient(t[2], t[0], p),
    ];
    s.iter().all(|x| *x >= 0.) || s.iter().all(|x| *x <= 0.)
}
fn count(work: &mut usize) -> Result<(), String> {
    // Counted for reporting; cleanup terminates on its own (finite chains).
    *work += 1;
    Ok(())
}
fn empty_ear(
    components: &[Component],
    owner: (usize, usize, usize),
    work: &mut usize,
) -> Result<bool, String> {
    let (ci, bi, i) = owner;
    let ring = if bi == 0 {
        &components[ci].outer
    } else {
        &components[ci].holes[bi - 1]
    };
    let n = ring.vertices.len();
    let point = |k: usize| {
        let p = ring.vertices[k % n];
        [p[0], p[1]]
    };
    let t = [point(i), point(i + 1), point(i + 2)];
    if t[0] == t[1] {
        return Ok(true);
    }
    if orient(t[0], t[1], t[2]) == 0.
        && (t[1][0] - t[0][0]) * (t[2][0] - t[1][0]) + (t[1][1] - t[0][1]) * (t[2][1] - t[1][1])
            < 0.
    {
        return Ok(false);
    }
    for (c, component) in components.iter().enumerate() {
        for (b, boundary) in std::iter::once(&component.outer)
            .chain(&component.holes)
            .enumerate()
        {
            let same = (c, b) == (ci, bi);
            // Both A and C are stationary endpoints of the replacement chord.
            let fixed_points = [t[0], t[2]];
            let fixed: &[Point] = if same { &fixed_points } else { &[] };
            for j in 0..boundary.vertices.len() {
                count(work)?;
                if same && (j == i || j == (i + 1) % n) {
                    continue;
                }
                let p = boundary.vertices[j];
                let q = boundary.vertices[(j + 1) % boundary.vertices.len()];
                let (p, q) = ([p[0], p[1]], [q[0], q[1]]);
                if [p, q]
                    .iter()
                    .any(|p| !fixed.contains(p) && in_triangle(*p, t))
                {
                    return Ok(false);
                }
                for k in 0..3 {
                    if intersects(p, q, t[k], t[(k + 1) % 3], fixed) {
                        return Ok(false);
                    }
                }
            }
        }
    }
    Ok(true)
}

// Every point on a minor arc lies within its sagitta of its chord. Bound the
// whole chord by adaptive intervals, each enclosed by ONE retained straight
// segment. Distance to a convex segment is convex along the chord, so its endpoint
// maximum bounds the whole interval; sagitta then covers the full minor arc.
fn shadow_bound(
    a: [f64; 3],
    b: [f64; 3],
    retained: &[(Point, Point)],
    budget: f64,
    work: &mut usize,
) -> Result<Option<f64>, String> {
    if a[2].abs() > 1. {
        return Ok(None);
    }
    let length = (a[0] - b[0]).hypot(a[1] - b[1]);
    let scale = a[..2]
        .iter()
        .chain(&b[..2])
        .map(|v| v.abs())
        .fold(1., f64::max);
    let reserve = a[2].abs() * length / 2. + 256. * f64::EPSILON * (scale + length);
    if reserve >= budget {
        return Ok(None);
    }
    let mut stack = vec![([a[0], a[1]], [b[0], b[1]], 0u8)];
    let mut bound = 0f64;
    while let Some((a, b, depth)) = stack.pop() {
        count(work)?;
        let p = [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.];
        let mut best = f64::INFINITY;
        for &(x, y) in retained {
            count(work)?;
            count(work)?;
            let d = crate::geometry::point_segment_distance_sq(a, x, y)
                .sqrt()
                .max(crate::geometry::point_segment_distance_sq(b, x, y).sqrt());
            best = best.min(d + reserve);
        }
        if best <= budget {
            bound = bound.max(best);
        } else if depth == 20 {
            return Ok(None);
        } else {
            stack.push((a, p, depth + 1));
            stack.push((p, b, depth + 1));
        }
    }
    Ok(Some(bound))
}

fn shadowed_spur(
    boundary: &super::Boundary,
    budget: f64,
    work: &mut usize,
) -> Result<Option<(usize, usize, f64)>, String> {
    let v = &boundary.vertices;
    let n = v.len();
    if n < 4 || budget <= 0. {
        return Ok(None);
    }
    let key = |p: [f64; 3]| [p[0], p[1]].map(|x| if x == 0. { 0 } else { x.to_bits() });
    let mut positions = std::collections::HashMap::new();
    let mut candidates = vec![];
    for (j, &p) in v.iter().enumerate() {
        count(work)?;
        if let Some(i) = positions.insert(key(p), j) {
            let span = j - i;
            if span >= 2 && n - span >= 2 {
                candidates.push((span, i));
                candidates.push((n - span, j));
            }
        }
    }
    let sort_work = candidates.len() * (1 + candidates.len().max(1).ilog2() as usize);
    *work += sort_work;
    candidates.sort_unstable();
    for (span, i) in candidates {
        let mut retained = vec![];
        for k in 0..n {
            count(work)?;
            if (k + n - i) % n >= span && v[k][2] == 0. {
                let a = v[k];
                let b = v[(k + 1) % n];
                retained.push(([a[0], a[1]], [b[0], b[1]]));
            }
        }
        if retained.is_empty() {
            continue;
        }
        let mut bound = 0f64;
        let mut shadowed = true;
        for k in 0..span {
            let a = v[(i + k) % n];
            let b = v[(i + k + 1) % n];
            if let Some(d) = shadow_bound(a, b, &retained, budget, work)? {
                bound = bound.max(d);
            } else {
                shadowed = false;
                break;
            }
        }
        if shadowed {
            return Ok(Some((i, span, bound)));
        }
    }
    Ok(None)
}

pub(super) fn normalize(
    components: &mut [Component],
    tolerance: f64,
) -> Result<(f64, usize), String> {
    // Generated kernels can leave exact A->B->A straight retraces. Their two
    // oriented segments cancel identically (zero winding everywhere off the
    // segment); deleting them preserves the regular filled set and every other
    // edge exactly. This is not a near-point/arc approximation or deletion of a
    // whole boundary. Authored geometry never enters this cleanup.
    let mut work = 0;
    let mut identity_removed = 0;
    for c in components.iter_mut() {
        for boundary in std::iter::once(&mut c.outer).chain(&mut c.holes) {
            let mut i = 0;
            while boundary.vertices.len() > 3 && i < boundary.vertices.len() {
                count(&mut work)?;
                let n = boundary.vertices.len();
                let a = boundary.vertices[i];
                let b = boundary.vertices[(i + 1) % n];
                let c = boundary.vertices[(i + 2) % n];
                if a[2] == 0. && b[2] == 0. && a[..2] == c[..2] {
                    boundary.vertices.rotate_left(i);
                    boundary.vertices[0][2] = c[2];
                    boundary.vertices.drain(1..=2);
                    identity_removed += 2;
                    i = 0;
                } else {
                    i += 1;
                }
            }
        }
    }
    if components
        .iter()
        .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
        .any(|b| b.vertices.iter().any(|p| p[2] != 0.))
    {
        let (mut error, mut removed) = (0., identity_removed);
        for c in components {
            for boundary in std::iter::once(&mut c.outer).chain(&mut c.holes) {
                let mut i = 0;
                while i < boundary.vertices.len() {
                    count(&mut work)?;
                    let n = boundary.vertices.len();
                    let a = boundary.vertices[i];
                    let b = boundary.vertices[(i + 1) % n];
                    if a[2] == 0. && a[0] == b[0] && a[1] == b[1] {
                        if n <= 2 {
                            return Err(
                                "constructed exact duplicate would collapse a whole boundary"
                                    .into(),
                            );
                        }
                        // Remove A, retaining B's outgoing arc. The predecessor
                        // still ends at the exact same point. Zero displacement.
                        boundary.vertices.remove(i);
                        removed += 1;
                    } else {
                        i += 1;
                    }
                }
                loop {
                    let budget = (tolerance - error) / 8.;
                    let Some((i, span, displacement)) = shadowed_spur(boundary, budget, &mut work)?
                    else {
                        break;
                    };
                    boundary.vertices.rotate_left(i);
                    boundary.vertices[0][2] = boundary.vertices[span][2];
                    boundary.vertices.drain(1..=span);
                    removed += span;
                    error += displacement;
                }
            }
        }
        return Ok((error, removed));
    }
    let threshold = 1e-6_f64.max(tolerance / 4.) * 1.001;
    // Each current edge represents a contiguous original boundary chain.
    // Its entry bounds their bidirectional Hausdorff distance. Replacing
    // AB+BC by AC adds at most |AB| to the MAX of those two chain bounds;
    // independent edits elsewhere do not add to that displacement. Keep
    // accumulating when a previously edited chain participates again.
    let mut chain_errors: Vec<Vec<Vec<f64>>> = components
        .iter()
        .map(|c| {
            std::iter::once(&c.outer)
                .chain(&c.holes)
                .map(|b| vec![0.; b.vertices.len()])
                .collect()
        })
        .collect();
    let (mut error, mut removed) = (0f64, identity_removed);
    loop {
        let mut candidate = None;
        'search: for (ci, c) in components.iter().enumerate() {
            for (bi, b) in std::iter::once(&c.outer).chain(&c.holes).enumerate() {
                for i in 0..b.vertices.len() {
                    count(&mut work)?;
                    let a = b.vertices[i];
                    let next = b.vertices[(i + 1) % b.vertices.len()];
                    let d = (a[0] - next[0]).hypot(a[1] - next[1]);
                    if d < threshold {
                        candidate = Some((ci, bi, i, d));
                        break 'search;
                    }
                }
            }
        }
        let Some((ci, bi, i, d)) = candidate else {
            break;
        };
        let ring = if bi == 0 {
            &components[ci].outer
        } else {
            &components[ci].holes[bi - 1]
        };
        let next = (i + 1) % ring.vertices.len();
        let local_error = chain_errors[ci][bi][i].max(chain_errors[ci][bi][next]) + d;
        if ring.vertices.len() <= 3 || local_error > tolerance {
            return Err(format!(
                "constructed short edge cannot be removed within topology/error budget: vertices={}, accumulated_error={error}, edge={d}, ring={:?}",
                ring.vertices.len(),
                &ring.vertices[..ring.vertices.len().min(6)]
            ));
        }
        if !empty_ear(components, (ci, bi, i), &mut work)? {
            return Err(
                "constructed short-edge collapse would sweep another boundary or backtrack".into(),
            );
        }
        let ring = if bi == 0 {
            &mut components[ci].outer
        } else {
            &mut components[ci].holes[bi - 1]
        };
        chain_errors[ci][bi][i] = local_error;
        chain_errors[ci][bi].remove(next);
        ring.vertices.remove((i + 1) % ring.vertices.len());
        error = error.max(local_error);
        removed += 1;
    }
    Ok((error, removed))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planar::Boundary;
    fn component(points: &[[f64; 2]]) -> Component {
        Component {
            outer: Boundary {
                vertices: points.iter().map(|p| [p[0], p[1], 0.]).collect(),
            },
            holes: vec![],
        }
    }
    #[test]
    fn generated_ear_displacement_is_reported_and_input_is_not_a_cleanup_api() {
        let mut c = vec![component(&[
            [0., 0.],
            [10., 0.],
            [10.0000004, 0.0000003],
            [10., 10.],
            [0., 10.],
        ])];
        let (error, removed) = normalize(&mut c, 1e-5).unwrap();
        assert_eq!(removed, 1);
        assert!((error - 5e-7).abs() < 1e-14);
        assert_eq!(c[0].outer.vertices.len(), 4);
    }
    #[test]
    fn independent_generated_ears_use_max_chain_error_including_wraparound() {
        for rotate in [0, 1] {
            let mut points = vec![];
            for i in 0..64 {
                let angle = i as f64 * std::f64::consts::TAU / 64.;
                let next = (i + 1) as f64 * std::f64::consts::TAU / 64.;
                let a = [10. * angle.cos(), 10. * angle.sin()];
                let c = [10. * next.cos(), 10. * next.sin()];
                let d = [c[0] - a[0], c[1] - a[1]];
                let length = d[0].hypot(d[1]);
                // A small, noncollinear outward ear on each independent side.
                let b = [
                    a[0] + (d[0] + 0.3 * d[1]) * 1e-6 / length,
                    a[1] + (d[1] - 0.3 * d[0]) * 1e-6 / length,
                ];
                points.extend([a, b]);
            }
            points.rotate_left(rotate);
            let original = component(&points);
            let mut generated = vec![original.clone()];
            let (error, removed) = normalize(&mut generated, 1e-5).unwrap();
            assert_eq!(removed, 64);
            assert!((1e-6..1.1e-6).contains(&error), "{error}");
            assert_eq!(generated[0].outer.vertices.len(), 64);
            // Each A-B-C chain is replaced by A-C. Distance to that convex
            // chord is convex on both source segments: endpoint maxima bound
            // their WHOLE lengths independently of the cleanup predicates.
            for i in 0..64 {
                let a = original.outer.vertices[(2 * i + 128 - rotate) % 128];
                let b = original.outer.vertices[(2 * i + 129 - rotate) % 128];
                let c = original.outer.vertices[(2 * i + 130 - rotate) % 128];
                let distance = crate::geometry::point_segment_distance_sq(
                    [b[0], b[1]],
                    [a[0], a[1]],
                    [c[0], c[1]],
                )
                .sqrt();
                assert!(distance <= error);
            }
        }
    }
    #[test]
    fn repeated_edits_to_one_chain_still_accumulate_and_refuse_excess_error() {
        let mut generated = vec![component(&[
            [0., 0.],
            [2e-7, 0.],
            [4e-7, 0.],
            [6e-7, 0.],
            [8e-7, 0.],
            [10., 0.],
            [10., 10.],
            [0., 10.],
        ])];
        assert!(
            normalize(&mut generated, 1e-6)
                .unwrap_err()
                .contains("topology/error budget")
        );
    }
    #[test]
    fn collapse_cannot_delete_a_tiny_loop_or_sweep_another_component() {
        let mut tiny = vec![component(&[[0., 0.], [4e-7, 3e-7], [0., 1.]])];
        assert!(
            normalize(&mut tiny, 1e-5)
                .unwrap_err()
                .contains("topology/error")
        );
        let mut c = vec![
            component(&[[0., 0.], [4e-7, 3e-7], [1., 0.], [1., 1.], [0., 1.]]),
            component(&[[0.1, 1e-7], [0.2, 1e-7], [0.15, 2e-7]]),
        ];
        assert!(
            normalize(&mut c, 1e-5)
                .unwrap_err()
                .contains("sweep another boundary")
        );
    }
    #[test]
    fn generated_exact_line_retraces_cancel_without_changing_retained_edges_or_arcs() {
        let canonical = vec![[0., 0., 0.], [10., 0., 1.], [10., 10., 0.], [0., 10., 0.]];
        for shift in [0, 3, 5] {
            let mut vertices = canonical.clone();
            // Retain the outgoing semicircle at the repeated endpoint.
            vertices.splice(1..2, [[10., 0., 0.], [300., -50., 0.], [10., 0., 1.]]);
            vertices.rotate_left(shift);
            let mut c = vec![Component {
                outer: super::super::Boundary { vertices },
                holes: vec![],
            }];
            let (error, removed) = normalize(&mut c, 1e-5).unwrap();
            assert_eq!((error, removed), (0., 2));
            let ring = &mut c[0].outer.vertices;
            let start = ring.iter().position(|v| *v == canonical[0]).unwrap();
            ring.rotate_left(start);
            assert_eq!(*ring, canonical);
        }
        // A nonzero-area detour, or any returning arc, is not this identity.
        for detour in [
            vec![
                [0., 0., 0.],
                [5., -2., 0.],
                [1e-3, 0., 0.],
                [10., 0., 0.],
                [10., 10., 0.],
                [0., 10., 0.],
            ],
            vec![
                [0., 0., 1.],
                [5., -2., 0.],
                [0., 0., 0.],
                [10., 0., 0.],
                [10., 10., 0.],
                [0., 10., 0.],
            ],
        ] {
            let mut c = vec![Component {
                outer: super::super::Boundary {
                    vertices: detour.clone(),
                },
                holes: vec![],
            }];
            let (_, removed) = normalize(&mut c, 1e-5).unwrap();
            assert_eq!(removed, 0);
            assert_eq!(c[0].outer.vertices, detour);
        }
    }

    #[test]
    fn curved_generated_loops_only_remove_exact_zero_bulge_duplicates_without_changing_arcs() {
        let canonical = vec![[-2., 0., 1.], [2., 0., 1.]];
        for insertion in 0..2 {
            let mut vertices = canonical.clone();
            let p = vertices[insertion];
            vertices.insert(insertion, [p[0], p[1], 0.]);
            let mut c = vec![Component {
                outer: crate::planar::Boundary { vertices },
                holes: vec![],
            }];
            assert_eq!(normalize(&mut c, 1e-5).unwrap(), (0., 1));
            assert_eq!(c[0].outer.vertices, canonical);
        }
        let mut short = vec![Component {
            outer: crate::planar::Boundary {
                vertices: vec![[-2., 0., 0.], [-2. + 1e-8, 0., 1.], [2., 0., 1.]],
            },
            holes: vec![],
        }];
        let before = short[0].outer.clone();
        assert_eq!(normalize(&mut short, 1e-5).unwrap(), (0., 0));
        assert_eq!(
            short[0].outer, before,
            "curved near-duplicate was approximated"
        );
    }
    #[test]
    fn only_a_retained_chord_can_shadow_a_generated_redundant_lens_with_reported_sagitta() {
        let mut points = vec![
            [0., 0., 0.],
            [10., 0., 0.],
            [10., 10., 0.],
            [0., 10., 0.],
            [0., 0., 0.],
            [10., 0., -1e-7],
        ];
        let mut c = vec![Component {
            outer: crate::planar::Boundary {
                vertices: points.clone(),
            },
            holes: vec![],
        }];
        let (error, removed) = normalize(&mut c, 1e-5).unwrap();
        assert_eq!(removed, 2);
        assert!((5e-7..=1.25e-6).contains(&error));
        let p = crate::planar::PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "lens".into(),
            boundary_error_mm: error,
            components: c,
            source: serde_json::Value::Null,
        }
        .prepare(crate::planar::ConstructionOptions::default())
        .unwrap();
        assert!((p.summary().area_mm2 - 100.).abs() < 1e-12);
        // No retained matching chord: the protruding lens must not disappear.
        points[1] = [10., 1., 0.];
        let mut c = vec![Component {
            outer: crate::planar::Boundary {
                vertices: points.clone(),
            },
            holes: vec![],
        }];
        assert_eq!(normalize(&mut c, 1e-5).unwrap(), (0., 0));
        assert_eq!(c[0].outer.vertices, points);
        // A physically wider lens cannot spend more than the specified error.
        points[1] = [10., 0., 0.];
        points[5][2] = -0.01;
        let mut c = vec![Component {
            outer: crate::planar::Boundary {
                vertices: points.clone(),
            },
            holes: vec![],
        }];
        assert_eq!(normalize(&mut c, 1e-5).unwrap(), (0., 0));
        assert_eq!(c[0].outer.vertices, points);
    }
}
