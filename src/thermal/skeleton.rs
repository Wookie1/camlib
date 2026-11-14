//! Skeleton cut-up.
//!
//! When every part is cut, what is left of the sheet is a lace that is
//! awkward and heavy to lift off the table. Nesting software adds dedicated
//! cuts that divide it into pieces small enough to handle (Hypertherm,
//! ProNest "Skeleton Cut-Up": "dedicated cuts to a sheet, making it easy to
//! remove skeleton material from the table when the job is complete").
//!
//! The cuts here are straight lines on a grid over the declared area, no
//! further apart than `max_piece_mm`. A line is cut only where it is over
//! the lace: it stops short of every part and never runs over a hole (there
//! is no plate under the torch there, and a plasma arc is lost). Each
//! stretch is pierced once, at its end nearer a part, far enough from it for
//! the pierce crater (and for the torch to travel there, where rapids keep
//! off parts already cut free), and cut to its other end: `part_clearance_mm` short of
//! a part's kerf (0: into the kerf, so the piece is severed there), or
//! `run_off_mm` past the area's edge. The short neck left at the pierced
//! end, where it faces a part, holds the piece until the lace is lifted.
use serde::{Deserialize, Serialize};

type P2 = [f64; 2];

/// Cut the lace into pieces. Every value is the caller's.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SkeletonCutUp {
    /// The largest piece: no two neighbouring lines are further apart.
    pub max_piece_mm: f64,
    /// How far short of a part's kerf a cut stops (0: it runs into the
    /// kerf).
    pub part_clearance_mm: f64,
    /// How far past the area's edge a cut runs where it reaches it.
    pub run_off_mm: f64,
    /// The area cut up, `[x0, y0, x1, y1]` in the sheet's coordinates: the
    /// sheet, or the part of it the nest used (the rest being a remnant).
    /// No default: the planner does not know which edges are the sheet's.
    pub area_mm: [f64; 4],
}

impl SkeletonCutUp {
    pub(crate) fn validate(&self) -> Result<(), String> {
        let [x0, y0, x1, y1] = self.area_mm;
        if !(self.max_piece_mm.is_finite() && self.max_piece_mm > 0. && self.max_piece_mm <= 1e5) {
            return Err(format!(
                "skeleton: max_piece_mm {} must be positive (the largest piece of lace to be lifted)",
                self.max_piece_mm
            ));
        }
        if !(self.part_clearance_mm.is_finite() && (0. ..=1e3).contains(&self.part_clearance_mm)) {
            return Err(format!(
                "skeleton: part_clearance_mm {} must not be negative (how far short of a part's kerf a cut stops)",
                self.part_clearance_mm
            ));
        }
        if !(self.run_off_mm.is_finite() && (0. ..=1e3).contains(&self.run_off_mm)) {
            return Err(format!(
                "skeleton: run_off_mm {} must not be negative (how far a cut runs past the area's edge)",
                self.run_off_mm
            ));
        }
        if !(self.area_mm.iter().all(|v| v.is_finite() && v.abs() <= 1e6) && x1 > x0 && y1 > y0) {
            return Err(format!(
                "skeleton: area_mm {:?} must be [x0, y0, x1, y1] with x1 > x0 and y1 > y0",
                self.area_mm
            ));
        }
        Ok(())
    }
}

/// One cut: pierced at `from`, cut straight to `to`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Cut {
    pub from: P2,
    pub to: P2,
}

/// The distances the planner keeps, from a part's drawn edge to a cut's
/// centre line.
pub(crate) struct Reach {
    /// Along a cut and at its far end.
    pub cut: f64,
    /// At a pierce point.
    pub pierce: f64,
    /// Shortest cut worth piercing for.
    pub min_length: f64,
    /// Spacing of the samples along a line.
    pub step: f64,
}

/// The cuts. `open(p)` is the distance from `p` to the nearest part (its
/// drawn edge), or None where `p` is in a part or over a hole.
/// `reachable(p)` says whether the torch can travel to and from `p`
/// (rapids kept off parts already cut free): asked of each pierce and of
/// each cut's end.
pub(crate) fn plan(
    rules: &SkeletonCutUp,
    reach: &Reach,
    mut open: impl FnMut(P2) -> Result<Option<f64>, String>,
    mut reachable: impl FnMut(P2) -> bool,
) -> Result<Vec<Cut>, String> {
    let [x0, y0, x1, y1] = rules.area_mm;
    let mut cuts = vec![];
    // Lines of constant x, then of constant y; each walked from the low end.
    for (axis, (lo, hi), (start, end)) in [(0, (x0, x1), (y0, y1)), (1, (y0, y1), (x0, x1))] {
        let pieces = ((hi - lo) / rules.max_piece_mm).ceil().max(1.) as usize;
        if pieces > 10_000 {
            return Err(
                "skeleton: max_piece_mm gives more than 10000 lines across the area".into(),
            );
        }
        for k in 1..pieces {
            let at = lo + (hi - lo) * k as f64 / pieces as f64;
            let point = |t: f64| if axis == 0 { [at, t] } else { [t, at] };
            let n = (((end - start) / reach.step).ceil() as usize).clamp(1, 4_000_000);
            let h = (end - start) / n as f64;
            // The stretches over the lace, with the clearance of each sample.
            let mut inside: Option<f64> = None;
            let mut stretches: Vec<(f64, f64)> = vec![];
            let mut clearances = Vec::with_capacity(n + 1);
            for i in 0..=n {
                let t = start + h * i as f64;
                let d = open(point(t))?;
                clearances.push(d);
                let free = d.is_some_and(|d| d >= reach.cut);
                match (free, inside) {
                    (true, None) => inside = Some(t),
                    (false, Some(s)) => {
                        stretches.push((s, t - h));
                        inside = None;
                    }
                    _ => {}
                }
            }
            if let Some(s) = inside {
                stretches.push((s, end));
            }
            let clearance_at = |t: f64| {
                let i = (((t - start) / h).round() as usize).min(n);
                clearances[i]
            };
            for (s, e) in stretches {
                let (at_low_edge, at_high_edge) = (s <= start, e >= end);
                // The pierce: from the end that faces a part (the low end
                // when both do, or neither), the first sample far enough
                // from every part for the crater; at the area's edge, that
                // far inside it.
                let forward = !(at_low_edge && !at_high_edge);
                let (near, far) = if forward { (s, e) } else { (e, s) };
                let sign = if forward { 1. } else { -1. };
                let mut pierce = None;
                let steps = ((e - s) / h).round() as usize;
                for i in 0..=steps {
                    let t = near + sign * h * i as f64;
                    let inset = (t - near).abs();
                    let at_edge = if forward { at_low_edge } else { at_high_edge };
                    if clearance_at(t).is_some_and(|d| d >= reach.pierce)
                        && (!at_edge || inset >= reach.pierce)
                        && reachable(point(t))
                    {
                        pierce = Some(t);
                        break;
                    }
                }
                let Some(pierce) = pierce else {
                    continue;
                };
                let far_at_edge = if forward { at_high_edge } else { at_low_edge };
                let mut to = if far_at_edge {
                    far + sign * rules.run_off_mm
                } else {
                    far
                };
                // The torch lifts where the cut ends: that too must be a
                // place it can travel from.
                while (to - pierce) * sign > 0. && !reachable(point(to)) {
                    to -= sign * h;
                }
                if (to - pierce).abs() < reach.min_length {
                    continue;
                }
                cuts.push(Cut {
                    from: point(pierce),
                    to: point(to),
                });
            }
        }
    }
    Ok(cuts)
}

#[cfg(test)]
mod tests {
    use super::*;

    const REACH: Reach = Reach {
        cut: 1.,
        pierce: 2.,
        min_length: 3.,
        step: 0.25,
    };
    fn rules(max_piece_mm: f64) -> SkeletonCutUp {
        SkeletonCutUp {
            max_piece_mm,
            part_clearance_mm: 0.,
            run_off_mm: 2.,
            area_mm: [0., 0., 100., 60.],
        }
    }
    /// The distance to a square part at (40..60, 20..40), None inside it.
    fn square(p: P2) -> Result<Option<f64>, String> {
        let dx = (40. - p[0]).max(p[0] - 60.);
        let dy = (20. - p[1]).max(p[1] - 40.);
        Ok((dx > 0. || dy > 0.).then(|| dx.max(0.).hypot(dy.max(0.))))
    }

    #[test]
    fn an_empty_area_is_cut_edge_to_edge_on_an_even_grid() {
        let cuts = plan(&rules(40.), &REACH, |_| Ok(Some(f64::INFINITY)), |_| true).unwrap();
        // 100 wide: three columns (two lines); 60 high: two rows (one line).
        assert_eq!(cuts.len(), 3);
        let third = 100. / 3.;
        // Pierced the crater's reach inside the low edge, run off the high one.
        assert_eq!(
            cuts[0],
            Cut {
                from: [third, 2.],
                to: [third, 62.]
            }
        );
        assert_eq!(
            cuts[1],
            Cut {
                from: [2. * third, 2.],
                to: [2. * third, 62.]
            }
        );
        assert_eq!(
            cuts[2],
            Cut {
                from: [2., 30.],
                to: [102., 30.]
            }
        );
        // No piece is larger than declared.
        assert!(third <= 40. && 30. <= 40.);
        // An area no larger than a piece needs no cut.
        assert!(
            plan(&rules(100.), &REACH, |_| Ok(Some(f64::INFINITY)), |_| true)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn a_line_stops_at_a_part_and_is_pierced_clear_of_it() {
        let cuts = plan(&rules(50.), &REACH, square, |_| true).unwrap();
        // One line each way through the middle of the part: x = 50 and y = 30.
        assert_eq!(cuts.len(), 4);
        // Below the part: its low end is the area's edge, so it is pierced
        // at the part's side, the crater's reach from it, and runs off.
        assert_eq!(
            cuts[0],
            Cut {
                from: [50., 18.],
                to: [50., -2.]
            }
        );
        // Above the part: pierced beside it, run off the far edge.
        assert_eq!(
            cuts[1],
            Cut {
                from: [50., 42.],
                to: [50., 62.]
            }
        );
        assert_eq!(
            cuts[2],
            Cut {
                from: [38., 30.],
                to: [-2., 30.]
            }
        );
        assert_eq!(
            cuts[3],
            Cut {
                from: [62., 30.],
                to: [102., 30.]
            }
        );
        for c in &cuts {
            // No cut comes nearer the part than declared, and no pierce
            // nearer than the crater needs.
            assert!(square(c.from).unwrap().unwrap() >= REACH.pierce);
            for i in 0..=100 {
                let t = i as f64 / 100.;
                let p = [
                    c.from[0] + t * (c.to[0] - c.from[0]),
                    c.from[1] + t * (c.to[1] - c.from[1]),
                ];
                assert!(square(p).unwrap().is_some_and(|d| d >= REACH.cut), "{p:?}");
            }
        }
    }

    #[test]
    fn values_are_declared_sensibly() {
        assert!(rules(40.).validate().is_ok());
        for bad in [
            SkeletonCutUp {
                max_piece_mm: 0.,
                ..rules(40.)
            },
            SkeletonCutUp {
                part_clearance_mm: -1.,
                ..rules(40.)
            },
            SkeletonCutUp {
                run_off_mm: f64::NAN,
                ..rules(40.)
            },
            SkeletonCutUp {
                area_mm: [0., 0., 0., 60.],
                ..rules(40.)
            },
        ] {
            assert!(bad.validate().is_err());
        }
    }
}
