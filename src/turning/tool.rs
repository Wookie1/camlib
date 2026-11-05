//! Turning tools: inserts and holders as 2D shapes in the (r, z) half-plane.
//!
//! Tool frame: the origin is the nose-radius center; r points away from the
//! spindle axis for an outside tool (toward it for a boring tool, by the
//! mirror in `TurnSide`), z along the spindle axis. A right-hand tool cuts
//! toward -z (toward the chuck), a left-hand tool toward +z.
use super::stock::TurnSide;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Hand {
    /// Cuts toward -z (toward the chuck): the usual turning direction.
    Right,
    /// Cuts toward +z.
    Left,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Insert {
    /// A turning insert: `included_angle_deg` (80 C, 55 D, 35 V, 90 S, 60
    /// T), `approach_angle_deg` between the major cutting edge and the feed
    /// direction (e.g. 93 or 95 for a profiling holder), and the edge length.
    Turning {
        included_angle_deg: f64,
        approach_angle_deg: f64,
        edge_length_mm: f64,
    },
    /// A grooving or parting insert of `width_mm`, `depth_mm` deep, with the
    /// nose radius at both corners. The reference point is its leading
    /// corner's nose center (the corner toward -z for a right-hand tool).
    Groove { width_mm: f64, depth_mm: f64 },
    /// A face grooving insert: a grooving blade pointing along -z into the
    /// front face, `width_mm` across in r, reaching `depth_mm` deep, with the
    /// nose radius at both corners. The reference point is its inner
    /// corner's nose center (the corner toward the axis).
    FaceGroove { width_mm: f64, depth_mm: f64 },
    /// A threading insert with `profile_angle_deg` (60 metric/UN), reaching
    /// `depth_mm` beyond its nose.
    Thread {
        profile_angle_deg: f64,
        depth_mm: f64,
    },
    /// A drill on the spindle axis (twist or insert drill): `diameter_mm`,
    /// `point_angle_deg` (118, 140; 180 for a flat bottom) and the length of
    /// its flutes. Its reference point is the tip on the axis.
    Drill {
        diameter_mm: f64,
        point_angle_deg: f64,
        flute_length_mm: f64,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TurningTool {
    pub id: String,
    pub insert: Insert,
    pub nose_radius_mm: f64,
    pub side: TurnSide,
    pub hand: Hand,
    /// Holder (or boring bar) body behind the insert: its extent along z
    /// (`width_mm`) and away from the cut (`length_mm`), starting
    /// `offset_mm` behind the nose center.
    pub holder_width_mm: f64,
    pub holder_length_mm: f64,
    #[serde(default)]
    pub holder_offset_mm: f64,
}

/// Convex polygons in the tool frame (r, z), outside-tool orientation.
#[derive(Clone, Debug)]
pub struct ToolShape {
    /// The insert, the nose arc replaced by a circumscribed polygon so the
    /// polygon contains the true insert.
    pub insert: Vec<[f64; 2]>,
    pub holder: Vec<[f64; 2]>,
    /// Directions (unit, r-z) of the two edges leaving the nose: the
    /// leading (major) edge and the trailing edge. A move may cut along the
    /// profile only between them.
    pub leading: [f64; 2],
    pub trailing: [f64; 2],
    /// The insert's straight sides, exact (its outline is these and the
    /// nose arcs): what sweeps the material besides the noses.
    pub edges: Vec<[[f64; 2]; 2]>,
}

fn dir(angle_deg: f64) -> [f64; 2] {
    // Angle measured from +r toward +z.
    let a = angle_deg.to_radians();
    [a.cos(), a.sin()]
}

impl TurningTool {
    pub fn validate(&self) -> Result<(), String> {
        let positive = |v: f64| v.is_finite() && v > 0. && v <= 1e4;
        if !crate::tool::valid_id(&self.id)
            || !(self.nose_radius_mm.is_finite()
                && self.nose_radius_mm >= 0.
                && self.nose_radius_mm <= 50.)
            || !positive(self.holder_width_mm)
            || !positive(self.holder_length_mm)
            || !(self.holder_offset_mm.is_finite() && self.holder_offset_mm >= 0.)
        {
            return Err(
                "turning tool needs an id, a nose radius 0..50 mm and a positive holder size"
                    .into(),
            );
        }
        match self.insert {
            Insert::Turning {
                included_angle_deg: e,
                approach_angle_deg: k,
                edge_length_mm: l,
            } => {
                if !(5. ..=120.).contains(&e) || !(30. ..=120.).contains(&k) || !positive(l) {
                    return Err("turning insert: included angle 5..120, approach angle 30..120 degrees, positive edge length".into());
                }
                if k + e >= 180. {
                    return Err("turning insert: approach plus included angle must leave a trailing clearance (< 180 degrees)".into());
                }
                if l <= self.nose_radius_mm {
                    return Err("turning insert edge must be longer than the nose radius".into());
                }
            }
            Insert::Groove { width_mm, depth_mm } | Insert::FaceGroove { width_mm, depth_mm } => {
                if !positive(width_mm) || !positive(depth_mm) || width_mm < 2. * self.nose_radius_mm
                {
                    return Err("grooving insert needs a positive width (at least two nose radii) and depth".into());
                }
            }
            Insert::Thread {
                profile_angle_deg,
                depth_mm,
            } => {
                if !(20. ..=90.).contains(&profile_angle_deg) || !positive(depth_mm) {
                    return Err(
                        "threading insert: profile angle 20..90 degrees and a positive depth"
                            .into(),
                    );
                }
            }
            Insert::Drill {
                diameter_mm,
                point_angle_deg,
                flute_length_mm,
            } => {
                if !positive(diameter_mm)
                    || !(60. ..=180.).contains(&point_angle_deg)
                    || !(flute_length_mm.is_finite() && flute_length_mm > diameter_mm / 2.)
                {
                    return Err("drill: positive diameter, point angle 60..180 degrees, flutes longer than the point".into());
                }
            }
        }
        Ok(())
    }

    /// The tool's shape in its frame, for an outside, right-hand tool;
    /// the caller mirrors z for a left-hand tool and r for a boring tool.
    /// Angles are measured from +r toward +z.
    pub fn shape(&self) -> Result<ToolShape, String> {
        self.validate()?;
        let rn = self.nose_radius_mm;
        // A polygon around the nose arc from one direction to another (in
        // steps of at most 2 degrees: within 0.00015 x the nose radius of
        // it) that contains the arc.
        // Tangent points at both ends and, between them, the corners where
        // the tangent lines at successive sample angles meet.
        let arc = |from: f64, to: f64| -> Vec<[f64; 2]> {
            let n = (((to - from).abs() / 2.).ceil() as usize).max(1);
            let step = (to - from) / n as f64;
            let grow = rn / (step.abs().to_radians() / 2.).cos();
            let at = |a: f64, r: f64| {
                let d = dir(a);
                [r * d[0], r * d[1]]
            };
            let mut points = vec![at(from, rn)];
            points.extend((0..n).map(|k| at(from + step * (k as f64 + 0.5), grow)));
            points.push(at(to, rn));
            points
        };
        // The far end of an edge leaving the nose along `angle`, tangent to
        // it on the `side` (+1: the tangent point at angle + 90), reaching
        // `length` along the edge.
        let end = |angle: f64, side: f64, length: f64| {
            let d = dir(angle);
            let t = dir(angle + side * 90.);
            [rn * t[0] + length * d[0], rn * t[1] + length * d[1]]
        };
        let holder = |z0: f64, z1: f64, from: f64| {
            let r0 = from + self.holder_offset_mm;
            let r1 = r0 + self.holder_length_mm;
            vec![[r0, z0], [r1, z0], [r1, z1], [r0, z1]]
        };
        Ok(match self.insert {
            Insert::Turning {
                included_angle_deg: e,
                approach_angle_deg: k,
                edge_length_mm: l,
            } => {
                // Feed toward -z (angle -90). The major edge leaves the nose
                // at the approach angle from the feed direction; the minor
                // (trailing) edge the included angle further round. The nose
                // faces the cut between them.
                let major = k - 90.;
                let minor = major + e;
                let mut insert = vec![end(minor, 1., l)];
                insert.extend(arc(minor + 90., major + 270.));
                insert.push(end(major, -1., l));
                let reach = insert.iter().map(|p| p[0]).fold(0., f64::max);
                let n = insert.len();
                // Minor edge, major edge (each from its tangent point on the
                // nose, which `arc` starts and ends with exactly) and back.
                let edges = vec![
                    [insert[0], insert[1]],
                    [insert[n - 2], insert[n - 1]],
                    [insert[n - 1], insert[0]],
                ];
                ToolShape {
                    edges,
                    // The shank runs radially behind the insert, set back
                    // from its leading (-z) side to the nose center and
                    // extending toward +z.
                    holder: holder(0., self.holder_width_mm, reach),
                    leading: dir(major),
                    trailing: dir(minor),
                    insert,
                }
            }
            Insert::Groove { width_mm, depth_mm } | Insert::FaceGroove { width_mm, depth_mm } => {
                // Nose centers at z = 0 (leading corner) and z = w; the
                // blade reaches depth_mm toward +r.
                let w = width_mm - 2. * rn;
                let mut insert = vec![[depth_mm, w + rn], [depth_mm, -rn]];
                insert.extend(arc(270., 180.));
                insert.extend(arc(180., 90.).into_iter().map(|p| [p[0], p[1] + w]));
                // Top, both sides and the flat between the corner noses.
                let edges = vec![
                    [[depth_mm, w + rn], [depth_mm, -rn]],
                    [[depth_mm, -rn], [0., -rn]],
                    [[-rn, 0.], [-rn, w]],
                    [[0., w + rn], [depth_mm, w + rn]],
                ];
                ToolShape {
                    edges,
                    holder: holder(-rn, w + rn, depth_mm),
                    leading: dir(0.),
                    trailing: dir(0.),
                    insert,
                }
            }
            Insert::Thread {
                profile_angle_deg,
                depth_mm,
            } => {
                // A V pointing toward the axis: flanks at +-half the
                // profile angle from +r.
                let half = profile_angle_deg / 2.;
                let length = (depth_mm + rn) / half.to_radians().cos();
                let mut insert = vec![end(half, 1., length)];
                insert.extend(arc(half + 90., 270. - half));
                insert.push(end(-half, -1., length));
                ToolShape {
                    edges: vec![],
                    holder: holder(
                        -self.holder_width_mm / 2.,
                        self.holder_width_mm / 2.,
                        depth_mm,
                    ),
                    leading: dir(-half),
                    trailing: dir(half),
                    insert,
                }
            }
            Insert::Drill {
                diameter_mm,
                point_angle_deg,
                flute_length_mm,
            } => {
                // The half on the +r side of the axis (the drill is
                // symmetric): tip on the axis, the point's cone to the full
                // diameter, the flutes; the holder (chuck) behind them.
                let r = diameter_mm / 2.;
                let cone = r / (point_angle_deg / 2.).to_radians().tan();
                ToolShape {
                    edges: vec![],
                    insert: vec![
                        [0., 0.],
                        [r, cone],
                        [r, flute_length_mm],
                        [0., flute_length_mm],
                    ],
                    holder: {
                        let z0 = flute_length_mm + self.holder_offset_mm;
                        let w = self.holder_width_mm / 2.;
                        vec![
                            [0., z0],
                            [w, z0],
                            [w, z0 + self.holder_length_mm],
                            [0., z0 + self.holder_length_mm],
                        ]
                    },
                    leading: dir(-90.),
                    trailing: dir(-90.),
                }
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tool(insert: Insert) -> TurningTool {
        TurningTool {
            id: "t".into(),
            insert,
            nose_radius_mm: 0.4,
            side: TurnSide::Outer,
            hand: Hand::Right,
            holder_width_mm: 20.,
            holder_length_mm: 100.,
            holder_offset_mm: 0.,
        }
    }
    fn convex(p: &[[f64; 2]]) -> bool {
        let n = p.len();
        let sign = |i: usize| {
            let (a, b, c) = (p[i], p[(i + 1) % n], p[(i + 2) % n]);
            let cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
            if cross.abs() < 1e-9 {
                0.
            } else {
                cross.signum()
            }
        };
        // Collinear triples (a tangent edge running into the nose arc)
        // have no turn; every other turn has the same sense.
        let turns: Vec<f64> = (0..n).map(sign).filter(|s| *s != 0.).collect();
        turns.iter().all(|s| *s == turns[0])
    }

    #[test]
    fn inserts_are_convex_and_contain_their_nose() {
        for insert in [
            Insert::Turning {
                included_angle_deg: 80.,
                approach_angle_deg: 95.,
                edge_length_mm: 12.,
            },
            Insert::Turning {
                included_angle_deg: 35.,
                approach_angle_deg: 93.,
                edge_length_mm: 16.,
            },
            Insert::Groove {
                width_mm: 3.,
                depth_mm: 15.,
            },
            Insert::Thread {
                profile_angle_deg: 60.,
                depth_mm: 3.,
            },
        ] {
            let s = tool(insert.clone()).shape().unwrap();
            assert!(convex(&s.insert), "{insert:?} {:?}", s.insert);
            // The nose point toward the axis (-r) is on or inside the polygon.
            let lowest = s.insert.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
            assert!(lowest <= -0.4 + 1e-9, "{insert:?}");
            assert!(s.holder.iter().all(|p| p[0] >= 0.));
        }
        // An 80 degree insert at 95 degrees approach: the major edge leans 5
        // degrees back from +r, the minor edge 85 degrees on.
        let s = tool(Insert::Turning {
            included_angle_deg: 80.,
            approach_angle_deg: 95.,
            edge_length_mm: 12.,
        })
        .shape()
        .unwrap();
        assert!((s.leading[1] - 5f64.to_radians().sin()).abs() < 1e-12);
        assert!((s.trailing[1] - 85f64.to_radians().sin()).abs() < 1e-12);
    }
}
