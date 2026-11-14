//! Bridges between parts.
//!
//! Small parts cut free of the sheet can tilt or drop between the table's
//! slats. A bridge keeps two neighbouring parts joined to each other by a
//! strip of the scrap between them, so they come off the table together and
//! are broken apart afterwards (Amada AP100US, "Chain and Bridge Cutting":
//! "Use Bridges", "Bridge Size", "Max Bridge Length": the size of the
//! bridges and the greatest distance between parts allowed for one).
//!
//! Where two parts have straight edges facing each other, each outline's
//! kerf path is opened over the bridge's width and the two openings are
//! joined by two cuts across the gap. The two outlines then are one closed
//! loop round both parts, cut on one pierce, and the strip between the two
//! crossing cuts is never cut: it is the bridge, `width_mm` wide between the
//! kerfs' edges.
//!
//! Each pair of parts gets at most one bridge, and a group of bridged parts
//! is a tree (two bridges between one pair, or a ring of bridged parts,
//! would enclose a window of scrap that needs a pierce of its own). A part
//! with no facing neighbour within reach keeps its own outline.
use crate::pocket::Stroke;
use serde::{Deserialize, Serialize};

type P2 = [f64; 2];

/// Bridges between neighbouring parts. Every value is the caller's.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Bridges {
    /// The uncut width of a bridge, between the edges of the two kerfs that
    /// bound it.
    pub width_mm: f64,
    /// The greatest distance between two parts' edges a bridge may span.
    pub max_length_mm: f64,
    /// The most parts joined into one group.
    pub max_parts: usize,
}

impl Bridges {
    pub(crate) fn validate(&self) -> Result<(), String> {
        if !(self.width_mm.is_finite() && self.width_mm > 0. && self.width_mm <= 1e3) {
            return Err(format!(
                "bridges: width_mm {} must be positive (the uncut strip that joins two parts)",
                self.width_mm
            ));
        }
        if !(self.max_length_mm.is_finite() && self.max_length_mm > 0. && self.max_length_mm <= 1e4)
        {
            return Err(format!(
                "bridges: max_length_mm {} must be positive (the greatest distance between two parts a bridge may span)",
                self.max_length_mm
            ));
        }
        if !(2..=10_000).contains(&self.max_parts) {
            return Err(format!(
                "bridges: max_parts {} must be at least 2 (the most parts joined into one group)",
                self.max_parts
            ));
        }
        Ok(())
    }
}

/// One planned bridge, for the report.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct BridgeReport {
    pub parts: [usize; 2],
    /// The middle of the bridge.
    pub at_mm: P2,
    /// The distance between the two parts' edges it spans.
    pub length_mm: f64,
}

/// An outline to bridge: its part, how deep the part is nested, and its
/// kerf path (closed, in cutting direction).
pub(crate) struct Outline {
    pub part: usize,
    pub depth: usize,
    pub strokes: Vec<Stroke>,
}

/// The outlines after bridging (one per group, under its lowest part), the
/// groups of parts joined (two or more parts each) and the bridges.
pub(crate) struct Plan {
    pub outlines: Vec<Outline>,
    pub groups: Vec<Vec<usize>>,
    pub bridges: Vec<BridgeReport>,
}

fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}
fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn along(a: P2, d: P2, t: f64) -> P2 {
    [a[0] + t * d[0], a[1] + t * d[1]]
}

/// The least distance from the segment p-q to a stroke, by its ends, the
/// stroke's ends and (for a line) a crossing.
fn segment_to_stroke(p: P2, q: P2, s: &Stroke) -> f64 {
    let point_segment = |x: P2, a: P2, b: P2| {
        let (d, v) = (sub(b, a), sub(x, a));
        let l2 = dot(d, d);
        let t = if l2 > 0. {
            (dot(v, d) / l2).clamp(0., 1.)
        } else {
            0.
        };
        let c = sub(v, [t * d[0], t * d[1]]);
        c[0].hypot(c[1])
    };
    match s.arc {
        None => {
            let (d1, d2) = (sub(q, p), sub(s.b, s.a));
            let denominator = cross(d1, d2);
            if denominator != 0. {
                let w = sub(s.a, p);
                let (t, u) = (cross(w, d2) / denominator, cross(w, d1) / denominator);
                if (0. ..=1.).contains(&t) && (0. ..=1.).contains(&u) {
                    return 0.;
                }
            }
            point_segment(p, s.a, s.b)
                .min(point_segment(q, s.a, s.b))
                .min(point_segment(s.a, p, q))
                .min(point_segment(s.b, p, q))
        }
        Some((c, sweep)) => {
            // Sampled along the arc, finely enough for a bridge's scale.
            let radius = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
            let start = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
            let n = ((sweep.abs() * radius / 0.05).ceil() as usize).clamp(8, 4096);
            (0..=n)
                .map(|k| {
                    let angle = start + sweep * k as f64 / n as f64;
                    let x = [c[0] + radius * angle.cos(), c[1] + radius * angle.sin()];
                    point_segment(x, p, q)
                })
                .fold(f64::INFINITY, f64::min)
        }
    }
}

/// A place for a bridge: straight strokes `i` of outline `a` and `j` of
/// outline `b`, the opening on each (in its own travel direction) and the
/// gap between the two kerf paths.
#[derive(Clone, Copy)]
struct Candidate {
    a: usize,
    b: usize,
    /// Opening on a's stroke, from `a1` to `a2` along its travel.
    a1: P2,
    a2: P2,
    /// Opening on b's stroke, from `b1` to `b2` along its travel: `b1` faces
    /// `a2` and `b2` faces `a1`.
    b1: P2,
    b2: P2,
    gap: f64,
}

/// The stroke of a closed loop that carries both p and q (in that order
/// along its travel), split there: the loop from q round to p.
fn opened(strokes: &[Stroke], p: P2, q: P2, tolerance: f64) -> Option<Vec<Stroke>> {
    let on = |s: &Stroke, x: P2| {
        if s.arc.is_some() {
            return None;
        }
        let d = sub(s.b, s.a);
        let length = d[0].hypot(d[1]);
        if length <= 0. {
            return None;
        }
        let v = sub(x, s.a);
        let t = dot(v, d) / (length * length);
        ((cross(v, d) / length).abs() <= tolerance && (0. ..=1.).contains(&t)).then_some(t)
    };
    let k = (0..strokes.len()).find(|&k| {
        matches!(
            (on(&strokes[k], p), on(&strokes[k], q)),
            (Some(tp), Some(tq)) if tp < tq
        )
    })?;
    let s = strokes[k];
    let mut out = Vec::with_capacity(strokes.len() + 1);
    if q != s.b {
        out.push(Stroke::line(q, s.b));
    }
    out.extend_from_slice(&strokes[k + 1..]);
    out.extend_from_slice(&strokes[..k]);
    if s.a != p {
        out.push(Stroke::line(s.a, p));
    }
    Some(out)
}

/// Plan the bridges. `kerf` is the kerf's width, `offset` the kerf path's
/// distance from the drawn edge, `scrap_left` whether the scrap is on the
/// left of travel (the default cutting direction).
pub(crate) fn plan(
    outlines: Vec<Outline>,
    bridges: &Bridges,
    kerf: f64,
    offset: f64,
    scrap_left: bool,
    tolerance: f64,
) -> Plan {
    let opening = bridges.width_mm + kerf;
    // Clear of the stroke's ends: a crossing cut's kerf stays on the edge.
    let margin = kerf;
    let mut candidates: Vec<Candidate> = vec![];
    for (ia, oa) in outlines.iter().enumerate() {
        for ob_index in ia + 1..outlines.len() {
            let ob = &outlines[ob_index];
            if oa.depth != ob.depth {
                continue;
            }
            let mut best: Option<Candidate> = None;
            for sa in oa.strokes.iter().filter(|s| s.arc.is_none()) {
                let da = sub(sa.b, sa.a);
                let la = da[0].hypot(da[1]);
                if la < opening + 2. * margin {
                    continue;
                }
                let u = [da[0] / la, da[1] / la];
                // Towards the scrap.
                let n = if scrap_left {
                    [-u[1], u[0]]
                } else {
                    [u[1], -u[0]]
                };
                for sb in ob.strokes.iter().filter(|s| s.arc.is_none()) {
                    let db = sub(sb.b, sb.a);
                    let lb = db[0].hypot(db[1]);
                    if lb <= 0. || (cross(u, db) / lb).abs() > 1e-9 || dot(u, db) >= 0. {
                        continue;
                    }
                    let gap = dot(sub(sb.a, sa.a), n);
                    // Between the parts' own edges, each `offset` behind its path.
                    if gap <= kerf || gap + 2. * offset > bridges.max_length_mm + tolerance {
                        continue;
                    }
                    // The stretch of a's stroke that b's stroke faces.
                    let (t0, t1) = (dot(sub(sb.b, sa.a), u), dot(sub(sb.a, sa.a), u));
                    let (low, high) = (t0.max(0.) + margin, t1.min(la) - margin);
                    if high - low < opening {
                        continue;
                    }
                    let middle = 0.5 * (low + high);
                    let a1 = along(sa.a, u, middle - 0.5 * opening);
                    let a2 = along(sa.a, u, middle + 0.5 * opening);
                    let candidate = Candidate {
                        a: ia,
                        b: ob_index,
                        a1,
                        a2,
                        b1: along(a2, n, gap),
                        b2: along(a1, n, gap),
                        gap,
                    };
                    // The two crossing cuts keep a kerf from every other
                    // outline (the planner's kerf proof is the authority;
                    // this only passes over places it would refuse).
                    let clear = outlines.iter().enumerate().all(|(k, o)| {
                        k == ia
                            || k == ob_index
                            || o.strokes.iter().all(|s| {
                                segment_to_stroke(candidate.a1, candidate.b2, s) > kerf
                                    && segment_to_stroke(candidate.a2, candidate.b1, s) > kerf
                            })
                    });
                    if clear && best.is_none_or(|b| gap < b.gap) {
                        best = Some(candidate);
                    }
                }
            }
            candidates.extend(best);
        }
    }
    // Nearest parts first; ties by the outlines' order.
    candidates.sort_by(|x, y| x.gap.total_cmp(&y.gap).then((x.a, x.b).cmp(&(y.a, y.b))));
    // Groups as a forest: each bridge joins two groups.
    let mut group: Vec<usize> = (0..outlines.len()).collect();
    let mut loops: Vec<Option<Vec<Stroke>>> =
        outlines.iter().map(|o| Some(o.strokes.clone())).collect();
    let mut members: Vec<Vec<usize>> = outlines.iter().map(|o| vec![o.part]).collect();
    let mut reports = vec![];
    for c in candidates {
        let (ga, gb) = (group[c.a], group[c.b]);
        if ga == gb || members[ga].len() + members[gb].len() > bridges.max_parts {
            continue;
        }
        let (Some(la), Some(lb)) = (loops[ga].as_ref(), loops[gb].as_ref()) else {
            continue;
        };
        // An earlier bridge may already have opened this stretch.
        let (Some(mut joined), Some(second)) = (
            opened(la, c.a1, c.a2, tolerance),
            opened(lb, c.b1, c.b2, tolerance),
        ) else {
            continue;
        };
        // a's loop runs from a2 round to a1, crosses to b2, b's loop runs
        // from b2 round to b1, and crosses back to a2.
        joined.push(Stroke::line(c.a1, c.b2));
        joined.extend(second);
        joined.push(Stroke::line(c.b1, c.a2));
        loops[ga] = Some(joined);
        loops[gb] = None;
        let moved = std::mem::take(&mut members[gb]);
        members[ga].extend(moved);
        for g in &mut group {
            if *g == gb {
                *g = ga;
            }
        }
        reports.push(BridgeReport {
            parts: [outlines[c.a].part, outlines[c.b].part],
            at_mm: [
                0.25 * (c.a1[0] + c.a2[0] + c.b1[0] + c.b2[0]),
                0.25 * (c.a1[1] + c.a2[1] + c.b1[1] + c.b2[1]),
            ],
            length_mm: c.gap + 2. * offset,
        });
    }
    let mut merged = vec![];
    let mut groups = vec![];
    for (k, strokes) in loops.into_iter().enumerate() {
        let Some(strokes) = strokes else {
            continue;
        };
        let mut parts = std::mem::take(&mut members[k]);
        parts.sort_unstable();
        merged.push(Outline {
            part: parts[0],
            depth: outlines[k].depth,
            strokes,
        });
        if parts.len() > 1 {
            groups.push(parts);
        }
    }
    merged.sort_by_key(|o| o.part);
    Plan {
        outlines: merged,
        groups,
        bridges: reports,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A clockwise square (the part on the right of travel).
    fn square(x: f64, y: f64, size: f64) -> Vec<Stroke> {
        let p = [[x, y], [x, y + size], [x + size, y + size], [x + size, y]];
        (0..4).map(|k| Stroke::line(p[k], p[(k + 1) % 4])).collect()
    }
    fn outline(part: usize, strokes: Vec<Stroke>) -> Outline {
        Outline {
            part,
            depth: 0,
            strokes,
        }
    }
    fn closed(strokes: &[Stroke]) -> bool {
        (0..strokes.len()).all(|k| strokes[k].b == strokes[(k + 1) % strokes.len()].a)
    }
    fn length(strokes: &[Stroke]) -> f64 {
        strokes.iter().map(crate::profiles::stroke_length).sum()
    }

    #[test]
    fn two_facing_squares_become_one_loop_with_the_strip_left() {
        let rules = Bridges {
            width_mm: 2.,
            max_length_mm: 6.,
            max_parts: 4,
        };
        // Kerf paths 4 mm apart; the drawn edges 0.5 mm behind each.
        let plan = plan(
            vec![
                outline(0, square(0., 0., 20.)),
                outline(1, square(24., 0., 20.)),
            ],
            &rules,
            1.,
            0.5,
            true,
            1e-9,
        );
        assert_eq!(plan.groups, vec![vec![0, 1]]);
        assert_eq!(plan.outlines.len(), 1);
        let strokes = &plan.outlines[0].strokes;
        assert!(closed(strokes), "{strokes:?}");
        // Both squares less the two openings (3 mm each), and two crossings.
        assert!((length(strokes) - (160. - 6. + 8.)).abs() < 1e-9);
        let bridge = &plan.bridges[0];
        assert_eq!(bridge.parts, [0, 1]);
        assert!((bridge.length_mm - 5.).abs() < 1e-12);
        assert!((bridge.at_mm[0] - 22.).abs() < 1e-12 && (bridge.at_mm[1] - 10.).abs() < 1e-12);
        // The strip: no stroke comes nearer the bridge's middle line than
        // half the opening.
        for s in strokes {
            let d = segment_to_stroke([20., 10.], [24., 10.], s);
            assert!(d >= 1.5 - 1e-9, "{s:?} {d}");
        }
        // Too far apart for the declared length: nothing is bridged.
        let far = plan_far(&rules);
        assert!(far.groups.is_empty() && far.outlines.len() == 2 && far.bridges.is_empty());
    }
    fn plan_far(rules: &Bridges) -> Plan {
        plan(
            vec![
                outline(0, square(0., 0., 20.)),
                outline(1, square(26., 0., 20.)),
            ],
            rules,
            1.,
            0.5,
            true,
            1e-9,
        )
    }

    #[test]
    fn a_row_is_a_tree_within_the_declared_group_size() {
        let row = |n: usize| -> Vec<Outline> {
            (0..n)
                .map(|k| outline(k, square(24. * k as f64, 0., 20.)))
                .collect()
        };
        let rules = |max_parts| Bridges {
            width_mm: 2.,
            max_length_mm: 6.,
            max_parts,
        };
        let all = plan(row(5), &rules(8), 1., 0.5, true, 1e-9);
        assert_eq!(all.groups, vec![vec![0, 1, 2, 3, 4]]);
        assert_eq!((all.outlines.len(), all.bridges.len()), (1, 4));
        assert!(closed(&all.outlines[0].strokes));
        let pairs = plan(row(5), &rules(2), 1., 0.5, true, 1e-9);
        assert_eq!(pairs.groups, vec![vec![0, 1], vec![2, 3]]);
        assert_eq!((pairs.outlines.len(), pairs.bridges.len()), (3, 2));
        assert!(pairs.outlines.iter().all(|o| closed(&o.strokes)));
    }

    #[test]
    fn values_are_declared_sensibly() {
        let good = Bridges {
            width_mm: 2.,
            max_length_mm: 6.,
            max_parts: 2,
        };
        assert!(good.validate().is_ok());
        for bad in [
            Bridges {
                width_mm: 0.,
                ..good
            },
            Bridges {
                max_length_mm: -1.,
                ..good
            },
            Bridges {
                max_parts: 1,
                ..good
            },
        ] {
            assert!(bad.validate().is_err());
        }
    }
}
