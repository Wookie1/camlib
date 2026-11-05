//! Guarded angular sets for the existing known-empty geometry, not pose samples.
//! The caller already reserves path/support error in the supplied clearance.
use std::f64::consts::{PI, TAU};

// Capacities hold every set built here, so the capacity refusal never
// triggers: a halfplane or disk has at most 2 ranges; intersecting sets of a
// and b ranges gives at most a + b - 1, a union at most a + b. halfplanes
// (N <= 4) <= 5; arc_tube: ring <= 3, sector <= 4, ring and sector <= 6, with
// both endpoint disks <= 10. Small capacities keep the sets in registers.
const PAIR: usize = 2;
const HALFPLANES: usize = 5;
const TUBE: usize = 10;

/// x.rem_euclid(TAU), bit for bit, without fmod for x in (-TAU, 2 TAU):
/// below TAU fmod returns x, and x - TAU is exact for x in [TAU, 2 TAU]
/// (Sterbenz), as is fmod's result; negative x takes rem_euclid's x + TAU.
#[inline(always)]
pub(super) fn wrap_tau(x: f64) -> f64 {
    if (0. ..TAU).contains(&x) {
        x
    } else if (TAU..2. * TAU).contains(&x) {
        x - TAU
    } else if x < 0. && x > -TAU {
        x + TAU
    } else {
        x.rem_euclid(TAU)
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct Intervals<const N: usize> {
    ranges: [[f64; 2]; N],
    len: usize,
}
impl<const N: usize> Intervals<N> {
    #[inline(always)]
    fn empty() -> Self {
        Self {
            ranges: [[0.; 2]; N],
            len: 0,
        }
    }
    #[inline(always)]
    fn full() -> Self {
        let mut result = Self::empty();
        result.ranges[0] = [0., TAU];
        result.len = 1;
        result
    }
    #[inline(always)]
    fn push(&mut self, lo: f64, hi: f64) -> Option<()> {
        if lo >= hi {
            return Some(());
        }
        if self.len > 0 && self.ranges[self.len - 1][1] >= lo {
            self.ranges[self.len - 1][1] = self.ranges[self.len - 1][1].max(hi);
        } else {
            if self.len == N {
                return None;
            }
            self.ranges[self.len] = [lo, hi];
            self.len += 1;
        }
        Some(())
    }
    /// The same ranges in another capacity.
    #[inline(always)]
    fn resize<const O: usize>(&self) -> Option<Intervals<O>> {
        let mut out = Intervals::empty();
        for range in self.as_slice() {
            out.push(range[0], range[1])?;
        }
        Some(out)
    }
    pub(super) fn as_slice(&self) -> &[[f64; 2]] {
        &self.ranges[..self.len]
    }
    pub(super) fn append_to(&self, out: &mut Vec<[f64; 2]>) {
        out.extend_from_slice(self.as_slice());
    }
    #[inline(always)]
    fn intersection<const M: usize, const O: usize>(
        &self,
        other: &Intervals<M>,
    ) -> Option<Intervals<O>> {
        let mut out = Intervals::empty();
        let (mut a, mut b) = (0, 0);
        while a < self.len && b < other.len {
            let x = self.ranges[a];
            let y = other.ranges[b];
            out.push(x[0].max(y[0]), x[1].min(y[1]))?;
            if x[1] < y[1] {
                a += 1;
            } else {
                b += 1;
            }
        }
        Some(out)
    }
    #[inline(always)]
    fn union<const M: usize, const O: usize>(&self, other: &Intervals<M>) -> Option<Intervals<O>> {
        let mut out = Intervals::empty();
        let (mut a, mut b) = (0, 0);
        while a < self.len || b < other.len {
            let next = if b == other.len || a < self.len && self.ranges[a][0] <= other.ranges[b][0]
            {
                let x = self.ranges[a];
                a += 1;
                x
            } else {
                let x = other.ranges[b];
                b += 1;
                x
            };
            out.push(next[0], next[1])?;
        }
        Some(out)
    }
}

/// A*cos(theta)+B*sin(theta)<C. Shrink true roots before clipping to the seam,
/// so an artificial 0/TAU boundary never cuts a hole in a connected clear arc.
#[inline(always)]
fn halfplane(h: [f64; 3], pad: f64) -> Option<Intervals<PAIR>> {
    if h.iter().any(|v| !v.is_finite())
        || !pad.is_finite()
        || !(64. * f64::EPSILON.sqrt()..=PI).contains(&pad)
    {
        return None;
    }
    // sqrt of the sum of squares is within 2 ulps of hypot (well inside the
    // 64-ulp guard below); outside the normal range use hypot itself.
    let squares = h[0] * h[0] + h[1] * h[1];
    let amplitude = if squares.is_normal() {
        squares.sqrt()
    } else {
        h[0].hypot(h[1])
    };
    if !amplitude.is_finite() {
        return None;
    }
    if amplitude == 0. {
        return Some(if h[2] > 0. {
            Intervals::full()
        } else {
            Intervals::empty()
        });
    }
    // Reserve hypot/division/threshold arithmetic. Near tangency use the
    // caller's old root classifier rather than divide by an uncertain margin.
    let guard = 64. * f64::EPSILON * (amplitude + h[2].abs());
    if h[2] > amplitude + guard {
        return Some(Intervals::full());
    }
    if h[2] < -amplitude - guard {
        return Some(Intervals::empty());
    }
    if (h[2].abs() - amplitude).abs() <= guard {
        return None;
    }
    let half = (h[2] / amplitude).acos();
    let width = TAU - 2. * half - 2. * pad;
    if width <= 0. {
        return Some(Intervals::empty());
    }
    let lo = wrap_tau(h[1].atan2(h[0]) + half + pad);
    let hi = lo + width;
    let mut out = Intervals::empty();
    if hi > TAU {
        out.push(0., hi - TAU)?;
        out.push(lo, TAU)?;
    } else {
        out.push(lo, hi)?;
    }
    Some(out)
}

pub(super) fn halfplanes<const N: usize>(
    h: [[f64; 3]; N],
    pad: f64,
) -> Option<Intervals<HALFPLANES>> {
    if N > 4 {
        return None;
    }
    let mut out = Intervals::<HALFPLANES>::full();
    for constraint in h {
        out = out.intersection(&halfplane(constraint, pad)?)?;
        if out.len == 0 {
            break;
        }
    }
    Some(out)
}

#[inline(always)]
fn disk_set(
    p: [f64; 2],
    r: f64,
    c: [f64; 2],
    radius: f64,
    inside: bool,
    pad: f64,
) -> Option<Intervals<PAIR>> {
    if !r.is_finite() || r <= 0. || !radius.is_finite() || radius < 0. {
        return None;
    }
    let d = [p[0] - c[0], p[1] - c[1]];
    let squared = [radius * radius, d[0] * d[0], d[1] * d[1], r * r];
    let rhs = squared[0] - squared[1] - squared[2] - squared[3];
    // Bound cancellation in squared distances independently of their result.
    // This is a numerical reserve, not a manufacturing tolerance increase.
    let reserve = 64. * f64::EPSILON * squared.iter().sum::<f64>();
    let sign = if inside { 1. } else { -1. };
    halfplane(
        [
            sign * 2. * r * d[0],
            sign * 2. * r * d[1],
            sign * rhs - reserve,
        ],
        pad,
    )
}
#[inline(always)]
pub(super) fn disk(
    p: [f64; 2],
    r: f64,
    c: [f64; 2],
    radius: f64,
    pad: f64,
) -> Option<Intervals<PAIR>> {
    if radius <= 0. {
        return Some(Intervals::empty());
    }
    disk_set(p, r, c, radius, true, pad)
}

#[derive(Clone, Copy, Debug)]
pub(super) struct ArcTube {
    pub start: [f64; 2],
    pub end: [f64; 2],
    pub center: [f64; 2],
    pub radius: f64,
    pub angle: f64,
    pub sweep: f64,
    pub clearance: f64,
    pub shrink: f64,
}

/// Trigonometry of immutable sector rays. Query-specific numerical erosion is
/// still applied below; these rays do not carry a clearance certificate.
#[derive(Clone, Copy, Debug)]
pub(super) struct ArcSector {
    lower: (f64, f64),
    upper: (f64, f64),
}
impl ArcSector {
    pub(super) fn new(angle: f64, sweep: f64) -> Self {
        let (lower, upper) = if sweep >= 0. {
            (angle, angle + sweep)
        } else {
            (angle + sweep, angle)
        };
        Self {
            lower: lower.sin_cos(),
            upper: upper.sin_cos(),
        }
    }
    /// Whether direction v (from the arc centre) lies in this sector of
    /// width `width` (closed; a direction within rounding of a ray may go
    /// either way, which moves a distance by O(radius * ulp^2) only).
    #[inline(always)]
    pub(super) fn contains(&self, v: [f64; 2], width: f64) -> bool {
        let (sl, cl) = self.lower;
        let (su, cu) = self.upper;
        let after_lower = cl * v[1] - sl * v[0] >= 0.;
        let before_upper = cu * v[1] - su * v[0] <= 0.;
        if width <= PI {
            after_lower && before_upper
        } else {
            after_lower || before_upper
        }
    }
    #[inline(always)]
    fn eroded(&self, shrink: f64) -> [(f64, f64); 2] {
        let (s, c) = shrink.sin_cos();
        let (sl, cl) = self.lower;
        let (su, cu) = self.upper;
        [
            (sl * c + cl * s, cl * c - sl * s),
            (su * c - cu * s, cu * c + su * s),
        ]
    }
}

/// Same model as the old predicate: annulus intersect oriented angular sector,
/// union actual endpoint disks. A major sector uses OR, a minor sector AND.
pub(super) fn arc_tube(
    p: [f64; 2],
    r: f64,
    tube: ArcTube,
    pad: f64,
    sector: Option<&ArcSector>,
) -> Option<Intervals<TUBE>> {
    let ArcTube {
        start,
        end,
        center,
        radius,
        angle,
        sweep,
        clearance,
        shrink,
    } = tube;
    if !radius.is_finite()
        || radius <= 0.
        || !angle.is_finite()
        || !sweep.is_finite()
        || !clearance.is_finite()
        || clearance <= 0.
        || !shrink.is_finite()
        || shrink < 0.
    {
        return None;
    }
    let outer = disk(p, r, center, radius + clearance, pad)?;
    let mut ring: Intervals<3> = if outer.len > 0 && radius >= clearance {
        outer.intersection(&disk_set(p, r, center, radius - clearance, false, pad)?)?
    } else {
        outer.resize()?
    };
    if sweep.abs() >= TAU {
        return ring.resize();
    }
    let width = sweep.abs() - 2. * shrink;
    if width <= 0. {
        ring = Intervals::empty();
    } else if ring.len > 0 {
        let lower = if sweep >= 0. {
            angle + shrink
        } else {
            angle + sweep + shrink
        };
        let upper = if sweep >= 0. {
            angle + sweep - shrink
        } else {
            angle - shrink
        };
        let d = [p[0] - center[0], p[1] - center[1]];
        let [(sl, cl), (su, cu)] = sector.map_or_else(
            || [lower.sin_cos(), upper.sin_cos()],
            |sector| sector.eroded(shrink),
        );
        let left = halfplane([r * sl, -r * cl, cl * d[1] - sl * d[0]], pad)?;
        let right = halfplane([-r * su, r * cu, su * d[0] - cu * d[1]], pad)?;
        let sector: Intervals<4> = if width <= PI {
            left.intersection(&right)?
        } else {
            left.union(&right)?
        };
        let clipped: Intervals<6> = ring.intersection(&sector)?;
        return clipped
            .union::<PAIR, 8>(&disk(p, r, start, clearance, pad)?)?
            .union(&disk(p, r, end, clearance, pad)?);
    }
    ring.union::<PAIR, 5>(&disk(p, r, start, clearance, pad)?)?
        .union(&disk(p, r, end, clearance, pad)?)
}

#[cfg(test)]
#[path = "angular/tests.rs"]
mod tests;
