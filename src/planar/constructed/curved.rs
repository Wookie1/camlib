//! Generated short arc/line chains only. Replace short minor AB + straight BC
//! or straight AB + short minor BC by AC; both retained endpoints stay fixed.
//! The old minor arc is at most its sagitta from its chord; AB+BC is
//! bidirectionally within the short edge length of AC. Empty convex enclosures
//! of the cap and triangle ABC protect
//! the embedding. Other curved edges are enclosed, never treated as chords.
use super::*;

fn polygon_contains(p: Point, polygon: &[Point]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let sides: Vec<_> = (0..polygon.len())
        .map(|i| orient(polygon[i], polygon[(i + 1) % polygon.len()], p))
        .collect();
    sides.iter().any(|x| *x != 0.)
        && (sides.iter().all(|x| *x >= 0.) || sides.iter().all(|x| *x <= 0.))
}
fn enclosure(a: [f64; 3], b: [f64; 3]) -> Option<Vec<Point>> {
    let (p, q) = ([a[0], a[1]], [b[0], b[1]]);
    if a[2] == 0. {
        return Some(vec![p, q]);
    }
    let d = [q[0] - p[0], q[1] - p[1]];
    let length = d[0].hypot(d[1]);
    let scale = p.iter().chain(&q).map(|x| x.abs()).fold(1., f64::max);
    let guard = 512. * f64::EPSILON * (scale + length);
    if length <= 16. * guard || !length.is_finite() {
        return None;
    }
    let bulge = a[2];
    let mid = [p[0] + d[0] / 2., p[1] + d[1] / 2.];
    if bulge.abs() < 0.95 {
        // Minor arc is inside endpoints + tangent intersection. Move that
        // intersection farther outward to cover floating-point construction.
        let factor = bulge.signum() * (2. * (bulge / (1. - bulge * bulge)).abs() + guard / length);
        let t = [mid[0] + d[1] * factor, mid[1] - d[0] * factor];
        Some(vec![p, t, q])
    } else {
        // A full-circle box also encloses semicircles and major arcs. It may
        // refuse a valid cleanup; no unsupported curved edge disappears.
        let factor = (1. - bulge * bulge) / (4. * bulge);
        let center = [mid[0] - d[1] * factor, mid[1] + d[0] * factor];
        let r = length * (1. + bulge * bulge) / (4. * bulge.abs()) + guard;
        if !r.is_finite() || center.iter().any(|v| !v.is_finite()) {
            return None;
        }
        Some(vec![
            [center[0] - r, center[1] - r],
            [center[0] + r, center[1] - r],
            [center[0] + r, center[1] + r],
            [center[0] - r, center[1] + r],
        ])
    }
}
fn empty_patch(
    components: &[Component],
    owner: (usize, usize),
    skip: &[usize],
    patch: &[Point],
    fixed: &[Point],
    work: &mut usize,
) -> Result<bool, String> {
    for (ci, c) in components.iter().enumerate() {
        for (bi, b) in std::iter::once(&c.outer).chain(&c.holes).enumerate() {
            let same = (ci, bi) == owner;
            for i in 0..b.vertices.len() {
                count(work)?;
                if same && skip.contains(&i) {
                    continue;
                }
                let a = b.vertices[i];
                let z = b.vertices[(i + 1) % b.vertices.len()];
                let Some(poly) = enclosure(a, z) else {
                    return Ok(false);
                };
                if (0..2).any(|axis| {
                    let bounds = |points: &[Point]| {
                        points
                            .iter()
                            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                                (lo.min(p[axis]), hi.max(p[axis]))
                            })
                    };
                    let (a, b) = bounds(patch);
                    let (c, d) = bounds(&poly);
                    b < c || d < a
                }) {
                    continue;
                }
                let actual = [[a[0], a[1]], [z[0], z[1]]];
                let allowed: Vec<_> = fixed
                    .iter()
                    .copied()
                    .filter(|p| same && actual.contains(p))
                    .collect();
                if poly
                    .iter()
                    .any(|p| !allowed.contains(p) && polygon_contains(*p, patch))
                    || patch
                        .iter()
                        .any(|p| !allowed.contains(p) && polygon_contains(*p, &poly))
                {
                    return Ok(false);
                }
                for j in 0..poly.len() {
                    for k in 0..patch.len() {
                        count(work)?;
                        if intersects(
                            poly[j],
                            poly[(j + 1) % poly.len()],
                            patch[k],
                            patch[(k + 1) % patch.len()],
                            &allowed,
                        ) {
                            return Ok(false);
                        }
                    }
                }
            }
        }
    }
    Ok(true)
}

pub(in crate::planar) fn normalize(
    components: &mut [Component],
    tolerance: f64,
) -> Result<(f64, usize), String> {
    let threshold = 1e-6_f64.max(tolerance / 4.) * 1.001;
    let mut errors: Vec<Vec<Vec<f64>>> = components
        .iter()
        .map(|c| {
            std::iter::once(&c.outer)
                .chain(&c.holes)
                .map(|b| vec![0.; b.vertices.len()])
                .collect()
        })
        .collect();
    let (mut work, mut removed, mut error) = (0, 0, 0f64);
    loop {
        let mut candidate = None;
        'search: for (ci, c) in components.iter().enumerate() {
            for (bi, b) in std::iter::once(&c.outer).chain(&c.holes).enumerate() {
                let n = b.vertices.len();
                if n <= 3 {
                    continue;
                }
                for i in 0..n {
                    count(&mut work)?;
                    let next = (i + 1) % n;
                    let (a, b, c) = (b.vertices[i], b.vertices[next], b.vertices[(i + 2) % n]);
                    let ab = (a[0] - b[0]).hypot(a[1] - b[1]);
                    let bc = (b[0] - c[0]).hypot(b[1] - c[1]);
                    let (d, arc_a, arc_b, edge) =
                        if ab > 0. && ab < threshold && a[2].abs() <= 1. && b[2] == 0. {
                            (ab, a, b, i)
                        } else if bc > 0. && bc < threshold && b[2].abs() <= 1. && a[2] == 0. {
                            (bc, b, c, next)
                        } else {
                            continue;
                        };
                    let points = [[a[0], a[1]], [b[0], b[1]], [c[0], c[1]]];
                    let scale = points.iter().flatten().map(|x| x.abs()).fold(1., f64::max);
                    let bound = errors[ci][bi][i].max(errors[ci][bi][next])
                        + d
                        + arc_a[2].abs() * d / 2.
                        + 512. * f64::EPSILON * scale;
                    if bound > tolerance {
                        continue;
                    }
                    if orient(points[0], points[1], points[2]) == 0.
                        && (points[1][0] - points[0][0]) * (points[2][0] - points[1][0])
                            + (points[1][1] - points[0][1]) * (points[2][1] - points[1][1])
                            < 0.
                    {
                        continue;
                    }
                    if arc_a[2] != 0. {
                        let Some(cap) = enclosure(arc_a, arc_b) else {
                            continue;
                        };
                        let fixed = [[arc_a[0], arc_a[1]], [arc_b[0], arc_b[1]]];
                        if !empty_patch(components, (ci, bi), &[edge], &cap, &fixed, &mut work)? {
                            continue;
                        }
                    }
                    if empty_patch(
                        components,
                        (ci, bi),
                        &[i, next],
                        &points,
                        &[points[0], points[2]],
                        &mut work,
                    )? {
                        candidate = Some((ci, bi, i, next, bound));
                        break 'search;
                    }
                }
            }
        }
        let Some((ci, bi, i, next, bound)) = candidate else {
            break;
        };
        let b = if bi == 0 {
            &mut components[ci].outer
        } else {
            &mut components[ci].holes[bi - 1]
        };
        b.vertices[i][2] = 0.;
        b.vertices.remove(next);
        errors[ci][bi][i] = bound;
        errors[ci][bi].remove(next);
        error = error.max(bound);
        removed += 1;
    }
    Ok((error, removed))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planar::Boundary;
    fn ring(shift: f64) -> Component {
        Component {
            outer: Boundary {
                vertices: vec![
                    [shift, shift, 0.1],
                    [shift + 1e-6, shift, 0.],
                    [shift + 10., shift, 0.],
                    [shift + 10., shift + 10., 0.],
                    [shift, shift + 10., 0.],
                ],
            },
            holes: vec![],
        }
    }
    #[test]
    fn generated_minor_cap_and_line_have_bounded_displacement_at_translated_coordinates() {
        for shift in [0., 100_000., -200_000.] {
            let mut c = vec![ring(shift)];
            let original = c[0].clone();
            let (error, count) = normalize(&mut c, 1e-5).unwrap();
            assert_eq!(count, 1);
            assert_eq!(c[0].outer.vertices.len(), 4);
            let d = original.outer.vertices[1][0] - original.outer.vertices[0][0];
            assert!(error >= d * 1.05 && error < 1.1e-6, "{error}");
            assert_eq!(c[0].outer.vertices[0], [shift, shift, 0.]);
            assert_eq!(&c[0].outer.vertices[1..], &original.outer.vertices[2..]);
        }
    }
    #[test]
    fn short_arc_after_a_line_has_the_same_bound_in_reverse_orientation() {
        for shift in [0., 100_000.] {
            let mut c = ring(shift);
            let old = c.outer.vertices.clone();
            c.outer.vertices = (0..old.len())
                .rev()
                .map(|i| {
                    [
                        old[i][0],
                        old[i][1],
                        -old[(i + old.len() - 1) % old.len()][2],
                    ]
                })
                .collect();
            let mut components = vec![c];
            let (error, count) = normalize(&mut components, 1e-5).unwrap();
            assert_eq!(count, 1);
            assert!((1e-6..1.1e-6).contains(&error));
            assert_eq!(components[0].outer.vertices.len(), 4);
        }
    }
    #[test]
    fn an_arc_crossing_the_cap_cannot_be_hidden_by_its_clear_chord() {
        let obstruction = Component {
            outer: Boundary {
                vertices: vec![
                    [0.25e-6, -0.15e-6, -0.6],
                    [0.75e-6, -0.15e-6, 0.],
                    [0.75e-6, -1e-6, 0.],
                    [0.25e-6, -1e-6, 0.],
                ],
            },
            holes: vec![],
        };
        let mut c = vec![ring(0.), obstruction];
        let old = c[0].outer.clone();
        normalize(&mut c, 1e-5).unwrap();
        assert_eq!(c[0].outer, old);
    }
    #[test]
    fn no_major_arc_long_neighbor_arc_or_whole_small_loop_is_deleted() {
        let mut major = ring(0.);
        major.outer.vertices[0][2] = 2.;
        let mut neighbor = ring(0.);
        neighbor.outer.vertices[1][2] = 0.5;
        let mut tiny = ring(0.);
        tiny.outer.vertices.truncate(3);
        for c in [major, tiny] {
            let mut copy = vec![c.clone()];
            assert_eq!(normalize(&mut copy, 1e-5).unwrap(), (0., 0));
            assert_eq!(copy[0].outer, c.outer);
        }
        // The new symmetric cleanup may remove the preceding SHORT arc,
        // but the long curved neighbor keeps both endpoints and its bulge.
        let original = neighbor.outer.vertices.clone();
        let mut copy = vec![neighbor];
        let (error, count) = normalize(&mut copy, 1e-5).unwrap();
        assert_eq!(count, 1);
        assert!(error > 0. && error < 1e-5);
        assert_eq!(copy[0].outer.vertices, original[1..]);
    }
}
