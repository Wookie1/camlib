//! Whole-rectangle upper-envelope bounds, independent of cutter paths.
use super::*;
use std::collections::BTreeMap;

type EdgeKey = ([u64; 2], [u64; 2]);

fn key(p: Vec3) -> [u64; 2] {
    [p[0], p[1]].map(|x| if x == 0. { 0 } else { x.to_bits() })
}
fn point(p: [u64; 2]) -> Vec3 {
    [f64::from_bits(p[0]), f64::from_bits(p[1]), 0.]
}
fn edge_meets_box(a: Vec3, b: Vec3, r: [f64; 4]) -> bool {
    if a[0].max(b[0]) < r[0]
        || a[0].min(b[0]) > r[2]
        || a[1].max(b[1]) < r[1]
        || a[1].min(b[1]) > r[3]
    {
        return false;
    }
    let corners = [
        [r[0], r[1], 0.],
        [r[2], r[1], 0.],
        [r[2], r[3], 0.],
        [r[0], r[3], 0.],
    ];
    // The segment's supporting line misses this convex rectangle iff all
    // corners lie strictly on one side. Combined with the AABB this is a
    // conservative intersection test (false positives only).
    let signs = corners.map(|p| orientation(a, b, p, 2));
    !(signs.iter().all(|s| *s > 0.) || signs.iter().all(|s| *s < 0.))
}
fn charge(work: &mut u64, limit: u64, amount: u64) -> bool {
    if amount > limit.saturating_sub(*work) {
        return false;
    }
    *work += amount;
    true
}

impl PreparedSurface {
    pub(crate) fn material_bounds(
        &self,
        xy: [f64; 4],
        tolerance: f64,
        work: &mut u64,
        limit: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<[f64; 2]>, String> {
        let r = [
            xy[0] - tolerance,
            xy[1] - tolerance,
            xy[2] + tolerance,
            xy[3] + tolerance,
        ];
        let mut stack = vec![0];
        let mut candidates = Vec::new();
        let mut upper = self.floor;
        while let Some(i) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("surface material check cancelled".into());
            }
            if !charge(work, limit, 1) {
                return Ok(None);
            }
            let node = &self.nodes[i];
            if node.max[0] < r[0] || node.min[0] > r[2] || node.max[1] < r[1] || node.min[1] > r[3]
            {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
                continue;
            }
            for i in node.range.clone() {
                if !charge(work, limit, 1) {
                    return Ok(None);
                }
                let t = &self.triangles[i];
                if t.max[0] < r[0] || t.min[0] > r[2] || t.max[1] < r[1] || t.min[1] > r[3] {
                    continue;
                }
                let area = orientation(t.p[0], t.p[1], t.p[2], 2);
                let mut lo = t.min[2];
                let mut hi = t.max[2];
                if area != 0. {
                    let d = sub(t.p[1], t.p[0]);
                    let e = sub(t.p[2], t.p[0]);
                    let gx = (d[2] * e[1] - e[2] * d[1]) / area;
                    let gy = (d[0] * e[2] - e[0] * d[2]) / area;
                    let scale = 1.
                        + t.p.iter().flatten().map(|x| x.abs()).sum::<f64>()
                        + r.iter().map(|x| x.abs()).sum::<f64>();
                    let condition =
                        (d[0].abs() * e[1].abs() + e[0].abs() * d[1].abs()) / area.abs();
                    let guard = (tolerance + 512. * f64::EPSILON * scale * (1. + condition))
                        * (1. + gx.abs() + gy.abs());
                    let z = [[r[0], r[1]], [r[0], r[3]], [r[2], r[1]], [r[2], r[3]]]
                        .map(|p| t.p[0][2] + gx * (p[0] - t.p[0][0]) + gy * (p[1] - t.p[0][1]));
                    if guard.is_finite() && z.iter().all(|v| v.is_finite()) {
                        lo = lo.max(z.iter().copied().fold(f64::INFINITY, f64::min) - guard);
                        hi = hi.min(z.iter().copied().fold(f64::NEG_INFINITY, f64::max) + guard);
                    }
                }
                if lo > hi {
                    continue;
                } // affine plane excludes this AABB candidate
                upper = upper.max(hi);
                if area != 0. {
                    candidates.push((lo, i));
                }
            }
        }
        candidates.sort_unstable_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        let mut edges: BTreeMap<EdgeKey, u8> = BTreeMap::new();
        let center = [(r[0] + r[2]) * 0.5, (r[1] + r[3]) * 0.5, 0.];
        let mut covered_center = false;
        for (lower, i) in candidates {
            if cancel.is_cancelled() {
                return Err("surface material check cancelled".into());
            }
            if !charge(work, limit, 3) {
                return Ok(None);
            }
            let t = &self.triangles[i];
            covered_center |= inside(t.p, center, 2);
            for j in 0..3 {
                let (a, b) = (key(t.p[j]), key(t.p[(j + 1) % 3]));
                let (a, b) = if a < b { (a, b) } else { (b, a) };
                let side = orientation(point(a), point(b), t.p[(j + 2) % 3], 2);
                *edges.entry((a, b)).or_default() |= if side > 0. { 1 } else { 2 };
            }
            if !covered_center {
                continue;
            }
            let mut boundary = false;
            for (&(a, b), &sides) in &edges {
                if !charge(work, limit, 1) {
                    return Ok(None);
                }
                if sides != 3 && edge_meets_box(point(a), point(b), r) {
                    boundary = true;
                    break;
                }
            }
            // Every boundary of this union is among the uncancelled edges.
            // Shared edges cancel only with triangles on BOTH sides, using
            // exact endpoint identity and robust orientation; no small gaps or
            // projected facets are rounded away. A connected box with one
            // covered point and no possible boundary crossing is fully covered.
            if !boundary {
                return Ok(Some([self.floor.max(lower), upper]));
            }
        }
        Ok(Some([self.floor, upper]))
    }
}
