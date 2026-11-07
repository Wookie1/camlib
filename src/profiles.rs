//! Shared closed offset profiles and source ownership for contour/chamfer.
use crate::{
    Verdict,
    contour::Boundaries,
    planar::{Component, ConstructionOptions, PlanarRegion, PreparedRegion},
    pocket::Stroke,
    stock::CancelSignal,
};
pub(crate) struct ProfileLoop {
    pub strokes: Vec<Stroke>,
    pub component: usize,
    pub hole: Option<usize>,
}
pub(crate) struct Profiles {
    pub loops: Vec<ProfileLoop>,
    pub edges: usize,
    pub arcs: usize,
    pub diagnostics: Vec<String>,
}

pub(crate) struct ProfileSettings {
    pub id: String,
    pub tool: String,
    pub heights: crate::heights::Heights,
    pub feed: f64,
    pub plunge: f64,
    pub tolerance: f64,
    pub lead_in: f64,
    pub lead_out: f64,
    pub spindle: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: crate::motion::Coolant,
}
pub(crate) enum ProfileDepths {
    Fixed {
        z: f64,
    },
    Linear {
        top: f64,
        target: f64,
        step: f64,
    },
    Chamfer {
        top: f64,
        reference_height: f64,
        height_per_width: f64,
        width: f64,
        step: f64,
        final_tip: f64,
    },
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Lift,
    Approach,
    Entry,
    LeadIn,
    Cut,
    LeadOut,
    Retract,
    /// Rapid from the feed height to the retract height (clearance after the
    /// final pass). Zero length, so skipped, with default heights.
    Rise,
    /// Rapid down to the feed height above the entry. Zero length with defaults.
    Descend,
    Done,
}
pub(crate) struct ProfileMotion {
    settings: ProfileSettings,
    spec: crate::stock::StockSpec,
    initial: crate::motion::Vec3,
    position: crate::motion::Vec3,
    loops: Vec<Vec<Stroke>>,
    depths: ProfileDepths,
    passes: usize,
    pass: usize,
    ring: usize,
    edge: usize,
    phase: Phase,
    next_id: u64,
    expected_generation: Option<u64>,
    pub drained: bool,
    pub cuts: usize,
    pub closed_loops: usize,
    /// Cutter compensation side per loop (empty: none), and events due.
    pub sides: Vec<crate::motion::CompensationSide>,
    events: Vec<crate::motion::Event>,
    /// Holding tabs, and the current loop's cut moves (ring, pass, moves).
    pub tabs: Option<TabPlan>,
    cut: Option<(usize, usize, Vec<CutMove>)>,
    /// Machining limits: stretches they block are lifted over.
    pub limits: Option<crate::limits::PreparedLimits>,
    /// Ramp: the first pass of each loop descends from this height to its
    /// depth along the loop (entered at this height).
    pub ramp_from: Option<f64>,
    /// Per loop, an XY point the tool plunges at instead of the tangent
    /// lead point (None for a loop: the tangent lead). Set by an operation
    /// that has proven the horizontal lead from it to the loop start clear.
    pub entry_points: Vec<Option<[f64; 2]>>,
}
type CutMove = (crate::motion::Curve, crate::motion::MoveKind);
/// Holding tabs on the tool-center loops: around each center (arc length),
/// the tool stays at `top` over `half` either side, with 45 degree ramps
/// (`ramp`) or straight up and down.
#[derive(Clone, Debug)]
pub(crate) struct TabPlan {
    pub top: f64,
    pub half: f64,
    pub ramp: bool,
    pub centers: Vec<Vec<f64>>,
}
pub(crate) fn stroke_length(s: &Stroke) -> f64 {
    match s.arc {
        None => (s.b[0] - s.a[0]).hypot(s.b[1] - s.a[1]),
        Some((c, sweep)) => (s.a[0] - c[0]).hypot(s.a[1] - c[1]) * sweep.abs(),
    }
}
/// The point at parameter t of a stroke, exactly its ends at 0 and 1.
pub(crate) fn stroke_point(s: &Stroke, t: f64) -> crate::Point {
    if t <= 0. {
        return s.a;
    }
    if t >= 1. {
        return s.b;
    }
    match s.arc {
        None => [
            s.a[0] + t * (s.b[0] - s.a[0]),
            s.a[1] + t * (s.b[1] - s.a[1]),
        ],
        Some((c, sweep)) => {
            let r = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
            let angle = (s.a[1] - c[1]).atan2(s.a[0] - c[0]) + t * sweep;
            [c[0] + r * angle.cos(), c[1] + r * angle.sin()]
        }
    }
}
/// The part of a stroke from t0 to t1, its height going from z0 to z1.
pub(crate) fn sub_curve(s: &Stroke, t0: f64, t1: f64, z0: f64, z1: f64) -> crate::motion::Curve {
    use crate::motion::{Curve, Plane};
    let (a, b) = (stroke_point(s, t0), stroke_point(s, t1));
    let (start, end) = ([a[0], a[1], z0], [b[0], b[1], z1]);
    match s.arc {
        None => Curve::Line { start, end },
        Some((c, sweep)) => {
            let center = [c[0], c[1], z0];
            let sweep_rad = sweep * (t1 - t0);
            if z0 == z1 {
                Curve::Arc {
                    start,
                    end,
                    center,
                    plane: Plane::Xy,
                    sweep_rad,
                }
            } else {
                Curve::Helix {
                    start,
                    end,
                    center,
                    plane: Plane::Xy,
                    sweep_rad,
                }
            }
        }
    }
}
/// A loop's cut moves at depth z, lifted over holding tabs.
pub(crate) fn tabbed_loop(
    strokes: &[Stroke],
    z: f64,
    start_z: f64,
    tabs: Option<(&TabPlan, &[f64])>,
    lifts: &[(f64, f64, f64)],
) -> Result<Vec<CutMove>, String> {
    use crate::motion::{Curve, MoveKind};
    let tabs = tabs.filter(|(p, _)| z < p.top);
    if tabs.is_none() && lifts.is_empty() && start_z == z {
        return Ok(strokes
            .iter()
            .map(|s| (s.curve(z), MoveKind::Cut))
            .collect());
    }
    let lengths: Vec<f64> = strokes.iter().map(stroke_length).collect();
    let total: f64 = lengths.iter().sum();
    // The base height: z, or a ramp from start_z down to z along the loop.
    let base = |s: f64| {
        if s >= total {
            z
        } else {
            start_z + (z - start_z) * s / total
        }
    };
    // Raised stretches: tabs (with ramps) and lifts over blocked stretches,
    // as (start, end, corners of the height profile); they must not overlap.
    type Raised = (f64, f64, Vec<(f64, f64)>);
    let mut raised: Vec<Raised> = vec![];
    if let Some((plan, centers)) = tabs {
        let ramp = if plan.ramp { plan.top - z } else { 0. };
        for &c in centers {
            let (s0, s1) = (c - plan.half, c + plan.half);
            raised.push((
                s0 - ramp,
                s1 + ramp,
                vec![
                    (s0 - ramp, base(s0 - ramp)),
                    (s0, plan.top),
                    (s1, plan.top),
                    (s1 + ramp, base(s1 + ramp)),
                ],
            ));
        }
    }
    for &(a, b, lift) in lifts {
        if a <= 0. || b >= total {
            return Err("the limits block the start or end of a profile loop (where it enters and leaves): change the loop start, leads or limits".into());
        }
        raised.push((a, b, vec![(a, base(a)), (a, lift), (b, lift), (b, base(b))]));
    }
    raised.sort_by(|x, y| x.0.total_cmp(&y.0));
    if raised.windows(2).any(|w| w[1].0 < w[0].1) {
        return Err(
            "holding tabs overlap a stretch the limits block: move the tabs or change the limits"
                .into(),
        );
    }
    // The height profile along the loop: (arc length, z) corners.
    let mut profile = vec![(0., start_z)];
    for (_, _, corners) in &raised {
        profile.extend(corners.iter().copied());
    }
    profile.push((total, z));
    let mut starts = Vec::with_capacity(strokes.len());
    let mut acc = 0.;
    for l in &lengths {
        starts.push(acc);
        acc += l;
    }
    // Stroke parameter of arc length s (the last stroke owns the end).
    let locate = |s: f64| -> (usize, f64) {
        let k = starts.iter().rposition(|&c| c <= s).unwrap_or(0);
        let t = if lengths[k] > 0. {
            ((s - starts[k]) / lengths[k]).clamp(0., 1.)
        } else {
            0.
        };
        (k, t)
    };
    let mut moves = vec![];
    for w in profile.windows(2) {
        let ((sa, za), (sb, zb)) = (w[0], w[1]);
        if sb <= sa {
            if za != zb {
                let (k, t) = locate(sa);
                let p = stroke_point(&strokes[k], t);
                let kind = if zb < za {
                    MoveKind::Plunge
                } else {
                    MoveKind::Cut
                };
                moves.push((
                    Curve::Line {
                        start: [p[0], p[1], za],
                        end: [p[0], p[1], zb],
                    },
                    kind,
                ));
            }
            continue;
        }
        let height = |s: f64| {
            if s >= sb {
                zb
            } else {
                za + (zb - za) * (s - sa) / (sb - sa)
            }
        };
        for k in 0..strokes.len() {
            let (from, to) = (sa.max(starts[k]), sb.min(starts[k] + lengths[k]));
            if to <= from {
                continue;
            }
            let t0 = (from - starts[k]) / lengths[k];
            let t1 = if to >= starts[k] + lengths[k] {
                1.
            } else {
                (to - starts[k]) / lengths[k]
            };
            let t0 = if from <= starts[k] { 0. } else { t0 };
            moves.push((
                sub_curve(&strokes[k], t0, t1, height(from), height(to)),
                MoveKind::Cut,
            ));
        }
    }
    Ok(moves)
}
pub(crate) fn lead_point(edges: &[Stroke], z: f64, length: f64, end: bool) -> crate::motion::Vec3 {
    let stroke = if end {
        *edges.last().unwrap()
    } else {
        edges[0]
    };
    let mut p = if end {
        stroke.curve(z).end()
    } else {
        stroke.curve(z).start()
    };
    let t = stroke.tangent(end);
    let sign = if end { 1. } else { -1. };
    p[0] += sign * length * t[0];
    p[1] += sign * length * t[1];
    p
}
impl ProfileMotion {
    pub fn new(
        settings: ProfileSettings,
        spec: &crate::stock::StockSpec,
        initial: crate::motion::Vec3,
        loops: Vec<Vec<Stroke>>,
        depths: ProfileDepths,
        passes: usize,
        first_id: u64,
    ) -> Self {
        Self {
            settings,
            spec: spec.clone(),
            initial,
            position: initial,
            loops,
            depths,
            passes,
            pass: 0,
            ring: 0,
            edge: 0,
            phase: Phase::Lift,
            next_id: first_id,
            expected_generation: None,
            drained: false,
            cuts: 0,
            closed_loops: 0,
            sides: vec![],
            events: vec![],
            tabs: None,
            cut: None,
            limits: None,
            ramp_from: None,
            entry_points: vec![],
        }
    }
    /// The stretches of a loop at depth z the limits block, as arc-length
    /// intervals, and the height to lift over them.
    fn lifts(&self, ring: usize, z: f64) -> Result<Vec<(f64, f64, f64)>, String> {
        let Some(limits) = &self.limits else {
            return Ok(vec![]);
        };
        let s = &self.settings;
        let mut blocked: Vec<(f64, f64)> = vec![];
        let mut start = 0.;
        for stroke in &self.loops[ring] {
            let length = stroke_length(stroke);
            let allowed = limits.allowed_spans(&stroke.curve(z), true)?;
            let mut t = 0.;
            for span in allowed.iter().chain([&[1., 1.]]) {
                if span[0] > t {
                    let (a, b) = (start + t * length, start + span[0] * length);
                    match blocked.last_mut() {
                        Some(last) if last.1 >= a => last.1 = b,
                        _ => blocked.push((a, b)),
                    }
                }
                t = span[1];
            }
            start += length;
        }
        if blocked.is_empty() {
            return Ok(vec![]);
        }
        if !self.sides.is_empty() {
            return Err("the limits block part of a compensated contour: lifting over them moves straight up and down, which cutter compensation does not allow".into());
        }
        let lift = limits.avoid_top().map_or(s.heights.retract, |top| {
            s.heights.retract.max(top + 16. * s.tolerance)
        });
        if lift > s.heights.clearance {
            return Err(format!(
                "lifting over an avoid area needs Z {lift:.4}, above clearance_z_mm {}",
                s.heights.clearance
            ));
        }
        Ok(blocked.into_iter().map(|(a, b)| (a, b, lift)).collect())
    }
    /// Events due after the motion just handed out (cutter compensation
    /// on before the lead-in, off before the lead-out).
    pub fn take_events(&mut self) -> Vec<crate::motion::Event> {
        std::mem::take(&mut self.events)
    }
    pub fn check_stock(
        &self,
        stock: &crate::stock::Stock,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self
                .expected_generation
                .is_some_and(|g| g != stock.generation())
        {
            return Err("commit preceding profile motion to matching stock".into());
        }
        if cancel.is_cancelled() {
            return Err("profile motion cancelled".into());
        }
        Ok(())
    }
    pub fn prefix_events(&self) -> Vec<crate::motion::Event> {
        self.settings.prefix_events(self.initial)
    }
    pub fn suffix_events(&self) -> Vec<crate::motion::Event> {
        self.settings.suffix_events()
    }
}
impl ProfileSettings {
    pub fn prefix_events(&self, initial: crate::motion::Vec3) -> Vec<crate::motion::Event> {
        use crate::motion::*;
        let s = self;
        vec![
            Event::WorkOffset {
                number: s.work_offset,
            },
            Event::ToolChange {
                tool: s.tool.clone(),
                number: s.tool_number,
                length_register: s.length_register,
                position_after_mm: initial,
            },
            Event::Spindle {
                rpm: s.spindle,
                direction: SpindleDirection::Cw,
            },
            Event::Coolant { mode: s.coolant },
        ]
    }
    pub fn suffix_events(&self) -> Vec<crate::motion::Event> {
        use crate::motion::*;
        vec![
            Event::Coolant { mode: Coolant::Off },
            Event::Spindle {
                rpm: 0.,
                direction: SpindleDirection::Cw,
            },
        ]
    }
}
impl ProfileMotion {
    /// Where the current loop is entered at height z: its proven entry
    /// point, else the tangent lead point.
    fn entry_point(&self, z: f64, lead_in: f64) -> crate::motion::Vec3 {
        match self.entry_points.get(self.ring).copied().flatten() {
            Some(p) => [p[0], p[1], z],
            None => lead_point(&self.loops[self.ring], z, lead_in, false),
        }
    }
    pub fn next(
        &mut self,
        stock: &crate::stock::Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<crate::motion::Motion>, String> {
        use crate::motion::*;
        loop {
            self.check_stock(stock, cancel)?;
            if self.phase == Phase::Done {
                self.drained = true;
                return Ok(None);
            }
            let z = match self.depths {
                ProfileDepths::Fixed { z } => z,
                ProfileDepths::Linear { top, target, step } => {
                    (top - (self.pass + 1) as f64 * step).max(target)
                }
                ProfileDepths::Chamfer {
                    top,
                    reference_height,
                    height_per_width,
                    width,
                    step,
                    final_tip,
                } => {
                    if self.pass + 1 == self.passes {
                        final_tip
                    } else {
                        top - reference_height
                            - ((self.pass + 1) as f64 * step).min(width) * height_per_width
                    }
                }
            };
            // A ramp pass enters at the height it descends from.
            let entry_z = match self.ramp_from {
                Some(from) if self.pass == 0 => from.max(z),
                _ => z,
            };
            let first = self.loops[self.ring][0].curve(entry_z).start();
            let s = &self.settings;
            let (curve, kind, next) = match self.phase {
                Phase::Lift => (
                    Curve::Line {
                        start: self.position,
                        end: [self.position[0], self.position[1], s.heights.clearance],
                    },
                    MoveKind::Rapid,
                    Phase::Approach,
                ),
                Phase::Approach => {
                    // Travel at the current rapid height (clearance before the
                    // first pass, the retract height between passes).
                    let mut end = self.entry_point(z, s.lead_in);
                    end[2] = self.position[2];
                    (
                        Curve::Line {
                            start: self.position,
                            end,
                        },
                        MoveKind::Rapid,
                        Phase::Descend,
                    )
                }
                Phase::Descend => (
                    Curve::Line {
                        start: self.position,
                        end: [self.position[0], self.position[1], s.heights.feed],
                    },
                    MoveKind::Rapid,
                    Phase::Entry,
                ),
                Phase::Entry => (
                    Curve::Line {
                        start: self.position,
                        end: self.entry_point(entry_z, s.lead_in),
                    },
                    MoveKind::Plunge,
                    Phase::LeadIn,
                ),
                Phase::LeadIn => (
                    Curve::Line {
                        start: self.position,
                        end: first,
                    },
                    MoveKind::Lead,
                    Phase::Cut,
                ),
                Phase::Cut => {
                    if self
                        .cut
                        .as_ref()
                        .is_none_or(|c| (c.0, c.1) != (self.ring, self.pass))
                    {
                        let tabs = self
                            .tabs
                            .as_ref()
                            .map(|p| (p, p.centers[self.ring].as_slice()));
                        let lifts = self.lifts(self.ring, z)?;
                        self.cut = Some((
                            self.ring,
                            self.pass,
                            tabbed_loop(&self.loops[self.ring], z, entry_z, tabs, &lifts)?,
                        ));
                    }
                    let (curve, kind) = self.cut.as_ref().unwrap().2[self.edge].clone();
                    (curve, kind, Phase::Cut)
                }
                Phase::LeadOut => (
                    Curve::Line {
                        start: self.position,
                        end: lead_point(&self.loops[self.ring], z, s.lead_out, true),
                    },
                    MoveKind::Lead,
                    Phase::Retract,
                ),
                Phase::Retract => (
                    Curve::Line {
                        start: self.position,
                        end: [self.position[0], self.position[1], s.heights.feed],
                    },
                    MoveKind::Retract,
                    Phase::Rise,
                ),
                Phase::Rise => {
                    let last = self.ring + 1 == self.loops.len() && self.pass + 1 == self.passes;
                    let z = if last {
                        s.heights.clearance
                    } else {
                        s.heights.retract
                    };
                    (
                        Curve::Line {
                            start: self.position,
                            end: [self.position[0], self.position[1], z],
                        },
                        MoveKind::Rapid,
                        Phase::Approach,
                    )
                }
                Phase::Done => unreachable!(),
            };
            // Entries, leads, retracts and rapids must respect the limits as
            // they are (the loop's cut moves were already lifted over them).
            if self.phase != Phase::Cut
                && let Some(limits) = &self.limits
                && !matches!(curve, Curve::Line { start, end } if start == end)
                && !limits
                    .allows_curve(&curve, !matches!(kind, MoveKind::Rapid | MoveKind::Link))?
            {
                return Err(format!(
                    "a {kind:?} move from {:?} to {:?} enters an avoid area (or a lead or entry leaves the containment): change leads, heights or limits",
                    curve.start(),
                    curve.end()
                ));
            }
            let motion = if matches!(curve,Curve::Line {start,end}if start==end) {
                None
            } else {
                curve.prepare(s.tolerance)?;
                let m = Motion {
                    id: self.next_id,
                    operation: s.id.clone(),
                    tool: s.tool.clone(),
                    kind,
                    curve,
                    feed_mm_min: if kind == MoveKind::Rapid {
                        None
                    } else {
                        Some(if kind == MoveKind::Plunge {
                            s.plunge
                        } else {
                            s.feed
                        })
                    },
                };
                self.position = m.curve.end();
                self.next_id += 1;
                self.expected_generation = Some(stock.generation() + 1);
                Some(m)
            };
            let phase = self.phase;
            self.phase = next;
            if phase == Phase::Entry
                && let Some(side) = self.sides.get(self.ring)
            {
                self.events.push(Event::CutterCompensation {
                    side: *side,
                    plane: crate::motion::Plane::Xy,
                });
            }
            if phase == Phase::Cut {
                self.edge += 1;
                self.cuts += 1;
                if self.edge == self.cut.as_ref().map_or(0, |c| c.2.len()) {
                    self.edge = 0;
                    self.phase = Phase::LeadOut;
                    self.closed_loops += 1;
                    if !self.sides.is_empty() {
                        self.events.push(Event::CutterCompensation {
                            side: CompensationSide::Off,
                            plane: crate::motion::Plane::Xy,
                        });
                    }
                }
            }
            if phase == Phase::Rise {
                self.ring += 1;
                if self.ring == self.loops.len() {
                    self.ring = 0;
                    self.pass += 1;
                }
                if self.pass == self.passes {
                    self.phase = Phase::Done;
                }
            }
            if motion.is_some() {
                return Ok(motion);
            }
        }
    }
}
fn single(source: &PlanarRegion, component: Component) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: source.name.clone(),
        boundary_error_mm: source.boundary_error_mm,
        source: serde_json::Value::Null,
        components: vec![component],
    }
}
pub(crate) fn offset_profiles(
    source: &PreparedRegion,
    distance: f64,
    selection: Boundaries,
    reverse: bool,
    options: ConstructionOptions,
    cancel: &impl CancelSignal,
) -> Result<Profiles, String> {
    let offset = source.offset(-distance)?;
    let center = offset.region.prepare(options)?;
    let definition = source.definition();
    let count_holes = |r: &PlanarRegion| r.components.iter().map(|c| c.holes.len()).sum::<usize>();
    if (selection != Boundaries::Holes
        && definition.components.len() != center.definition().components.len())
        || (selection != Boundaries::Outer
            && count_holes(definition) != count_holes(center.definition()))
    {
        return Err("profile offset loses/splits selected topology; cutter cannot reach selected boundaries".into());
    }
    let mut originals = Vec::new();
    if selection != Boundaries::Outer {
        for (i, c) in definition.components.iter().enumerate() {
            for (j, hole) in c.holes.iter().enumerate() {
                originals.push((
                    i,
                    j,
                    single(
                        definition,
                        Component {
                            outer: hole.clone(),
                            holes: vec![],
                        },
                    )
                    .prepare(options)?,
                ));
            }
        }
    }
    let mut assigned = vec![false; originals.len()];
    let mut outer_assigned = vec![false; definition.components.len()];
    let mut loops = Vec::new();
    let mut edges = 0;
    let mut arcs = 0;
    for c in &center.definition().components {
        if cancel.is_cancelled() {
            return Err("profile ownership cancelled".into());
        }
        if selection != Boundaries::Holes {
            let owners = if definition.components.len() == 1 {
                vec![0]
            } else {
                let prepared = single(center.definition(), c.clone()).prepare(options)?;
                definition
                    .components
                    .iter()
                    .enumerate()
                    .filter_map(|(i, source)| {
                        let p = source.outer.vertices[0];
                        match prepared.query([p[0], p[1]]) {
                            Ok(q) if matches!(q.verdict, Verdict::Allowed) => Some(i),
                            _ => None,
                        }
                    })
                    .collect::<Vec<_>>()
            };
            if owners.len() != 1 || outer_assigned[owners[0]] {
                return Err("profile outer ownership is not a source bijection".into());
            }
            outer_assigned[owners[0]] = true;
            loops.push(ProfileLoop {
                strokes: Stroke::from_boundary(&c.outer),
                component: owners[0],
                hole: None,
            });
        }
        if selection != Boundaries::Outer {
            for hole in &c.holes {
                let p = hole.vertices[0];
                // A part nested in another part's hole has holes inside
                // that hole too: the owner is the innermost (smallest)
                // original hole containing the offset hole.
                let owners = originals
                    .iter()
                    .enumerate()
                    .filter_map(|(i, (_, _, h))| match h.query([p[0], p[1]]) {
                        Ok(q) if matches!(q.verdict, Verdict::Allowed) => Some(i),
                        _ => None,
                    })
                    .min_by(|a, b| {
                        originals[*a]
                            .2
                            .summary()
                            .area_mm2
                            .total_cmp(&originals[*b].2.summary().area_mm2)
                    })
                    .into_iter()
                    .collect::<Vec<_>>();
                if owners.len() != 1 || assigned[owners[0]] {
                    return Err("profile hole offset is not a source bijection".into());
                }
                assigned[owners[0]] = true;
                let (i, j, _) = &originals[owners[0]];
                loops.push(ProfileLoop {
                    strokes: Stroke::from_boundary(hole),
                    component: *i,
                    hole: Some(*j),
                });
            }
        }
    }
    if assigned.iter().any(|a| !*a)
        || (selection != Boundaries::Holes && outer_assigned.iter().any(|a| !*a))
    {
        return Err("profile selected boundary disappeared or split".into());
    }
    for ring in &mut loops {
        if reverse {
            ring.strokes = ring
                .strokes
                .iter()
                .rev()
                .copied()
                .map(Stroke::reversed)
                .collect();
        }
        edges += ring.strokes.len();
        arcs += ring
            .strokes
            .iter()
            .filter(|s| matches!(s.curve(0.), crate::motion::Curve::Arc { .. }))
            .count();
    }
    if loops.is_empty() {
        return Err("profile has no selected loops".into());
    }
    Ok(Profiles {
        loops,
        edges,
        arcs,
        diagnostics: offset.diagnostics,
    })
}
