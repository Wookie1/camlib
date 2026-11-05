//! Lazy engagement-bounded straight slots. See docs/trochoidal-slot.md for proof
//! scope, explicit seed exceptions and the separate stock-clearance certificate.
use crate::{
    Point,
    motion::{Coolant, Curve, Event, Motion, MoveKind, Plane, SpindleDirection, Vec3},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrochoidalSlotOperation {
    pub id: String,
    pub tool: String,
    pub start_mm: Point,
    pub end_mm: Point,
    pub width_mm: f64,
    pub top_z_mm: f64,
    pub floor_z_mm: f64,
    pub stepdown_mm: f64,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    pub entry_feed_mm_min: f64,
    pub max_engagement_deg: f64,
    pub boundary_tolerance_mm: f64,
    pub tolerance_mm: f64,
    pub clockwise: bool,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Rapid travel height between passes (absolute Z). Default: clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// Rapid descent stops here and plunge feed begins; leaving a pass, feed
    /// changes to rapid here (absolute Z, above the stock top). Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}

#[derive(Debug, Serialize)]
pub struct SlotReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub depth_passes: usize,
    pub target_z_mm: f64,
    pub width_mm: f64,
    pub orbit_radius_mm: f64,
    pub max_engagement_deg: f64,
    pub boundary_underclearance_bound_mm: f64,
    pub bounded_advance_mm: f64,
    pub max_bounded_engagement_deg: f64,
    pub seed_entry_policy: &'static str,
    pub seed_orbits: usize,
    pub controlled_orbits: usize,
    pub stock_clear_advances: usize,
    pub stock_query_work: usize,
    pub stock_query_work_exhausted: bool,
}

/// Stores only a scalar path cursor, not a list of generated orbits/motions.
pub struct SlotPlan {
    op: TrochoidalSlotOperation,
    heights: crate::heights::Heights,
    spec: StockSpec,
    initial: Vec3,
    position: Vec3,
    direction: Point,
    normal: Point,
    length: f64,
    radius: f64,
    cutter_radius: f64,
    guard: f64,
    pass: usize,
    distance: f64,
    // 0=up,1=across,2=plunge,3=seed,4=advance,5=orbit,6=retract,7=done
    phase: u8,
    next_id: u64,
    expected_generation: Option<u64>,
    report: SlotReport,
}

impl SlotPlan {
    pub fn prepare(
        op: TrochoidalSlotOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_motion_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("slot planning cancelled".into());
        }
        spec.grid()?;
        let tool = assembly.prepare()?;
        let cutter = tool.cutter().definition();
        let r = cutter.diameter_mm / 2.;
        let values = [
            op.width_mm,
            op.top_z_mm,
            op.floor_z_mm,
            op.stepdown_mm,
            op.clearance_z_mm,
            op.feed_mm_min,
            op.entry_feed_mm_min,
            op.max_engagement_deg,
            op.boundary_tolerance_mm,
            op.tolerance_mm,
            op.spindle_rpm,
        ];
        let guard = 16. * op.tolerance_mm;
        let length = (op.end_mm[0] - op.start_mm[0]).hypot(op.end_mm[1] - op.start_mm[1]);
        crate::heights::require(
            "trochoidal slot",
            [
                (
                    valid_id(&op.id),
                    format!("id {:?} is not a valid identifier", op.id),
                ),
                (
                    op.tool == assembly.id,
                    format!(
                        "tool {:?} does not match the assembly {:?}",
                        op.tool, assembly.id
                    ),
                ),
                (
                    matches!(cutter.shape, CutterShape::Flat) && cutter.center_cutting,
                    "a trochoidal slot needs a flat, center-cutting end mill".into(),
                ),
                (
                    values.iter().all(|v| v.is_finite())
                        && op
                            .start_mm
                            .iter()
                            .chain(&op.end_mm)
                            .chain(&initial)
                            .all(|v| v.is_finite() && v.abs() <= 1e6),
                    "all numeric parameters and points must be finite".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    format!("tolerance_mm {} must be within 1e-7..0.01", op.tolerance_mm),
                ),
                (
                    (1.0..=170.0).contains(&op.max_engagement_deg),
                    format!(
                        "max_engagement_deg {} must be within 1..170",
                        op.max_engagement_deg
                    ),
                ),
                (
                    op.boundary_tolerance_mm > 2. * guard && op.boundary_tolerance_mm <= r / 4.,
                    format!(
                        "boundary_tolerance_mm {} must be positive and at most a quarter of the cutter radius ({})",
                        op.boundary_tolerance_mm,
                        r / 4.
                    ),
                ),
                (
                    op.width_mm > 2. * r + 4. * guard && op.width_mm <= 4. * r,
                    format!(
                        "width_mm {} must be wider than the cutter diameter {} and at most twice it",
                        op.width_mm,
                        2. * r
                    ),
                ),
                (
                    length > guard,
                    "start_mm and end_mm must be different points".into(),
                ),
                (
                    op.stepdown_mm > guard && op.stepdown_mm <= cutter.cutting_length_mm,
                    format!(
                        "stepdown_mm {} must be positive and at most the cutting length {}",
                        op.stepdown_mm, cutter.cutting_length_mm
                    ),
                ),
                (
                    op.floor_z_mm >= spec.min_mm[2] && op.floor_z_mm < op.top_z_mm,
                    format!(
                        "floor_z_mm {} must be below top_z_mm {} and not below the stock bottom {}",
                        op.floor_z_mm, op.top_z_mm, spec.min_mm[2]
                    ),
                ),
                (
                    op.top_z_mm == spec.max_mm[2],
                    format!(
                        "top_z_mm {} must equal the stock top {}",
                        op.top_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    op.top_z_mm - op.floor_z_mm <= cutter.cutting_length_mm,
                    format!(
                        "slot depth {} exceeds the cutting length {}",
                        op.top_z_mm - op.floor_z_mm,
                        cutter.cutting_length_mm
                    ),
                ),
                (
                    op.clearance_z_mm > op.top_z_mm + guard && op.clearance_z_mm.abs() <= 1e6,
                    format!(
                        "clearance_z_mm {} must be above top_z_mm {}",
                        op.clearance_z_mm, op.top_z_mm
                    ),
                ),
                (
                    initial[2] >= op.clearance_z_mm,
                    format!(
                        "start Z {} is below clearance_z_mm {}",
                        initial[2], op.clearance_z_mm
                    ),
                ),
                (
                    op.feed_mm_min > 0. && op.entry_feed_mm_min > 0. && op.spindle_rpm > 0.,
                    format!(
                        "feed_mm_min {}, entry_feed_mm_min {} and spindle_rpm {} must be positive",
                        op.feed_mm_min, op.entry_feed_mm_min, op.spindle_rpm
                    ),
                ),
                (
                    (1..=9).contains(&op.work_offset),
                    format!("work_offset {} must be 1..9", op.work_offset),
                ),
                (
                    op.tool_number > 0 && op.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
            ],
        )?;
        let radius = op.width_mm / 2. - r - guard;
        let a = radius + r;
        let half_phi = op.max_engagement_deg.to_radians() / 2.;
        let engagement_pitch =
            (a * a - (r * half_phi.sin()).powi(2)).sqrt() - r * half_phi.cos() - radius - guard;
        let t = op.boundary_tolerance_mm - guard;
        let coverage_pitch = 2. * (2. * a * t - t * t).sqrt();
        let pitch = engagement_pitch.min(coverage_pitch) * (1. - 1e-9);
        let passes = ((op.top_z_mm - op.floor_z_mm) / op.stepdown_mm).ceil() as usize;
        if !pitch.is_finite()
            || pitch <= guard
            || passes == 0
            || first_motion_id == 0
            || first_motion_id.checked_add(1).is_none()
        {
            return Err(
                "slot engagement/geometry tolerance or motion budget is too restrictive".into(),
            );
        }
        let direction = [
            (op.end_mm[0] - op.start_mm[0]) / length,
            (op.end_mm[1] - op.start_mm[1]) / length,
        ];
        let report = SlotReport {
            strategy: "trochoidal_slot",
            geometric_residual_status: ResidualStatus::Pending,
            depth_passes: passes,
            target_z_mm: op.floor_z_mm,
            width_mm: op.width_mm,
            orbit_radius_mm: radius,
            max_engagement_deg: op.max_engagement_deg,
            boundary_underclearance_bound_mm: op.boundary_tolerance_mm,
            bounded_advance_mm: pitch,
            max_bounded_engagement_deg: 0.,
            seed_entry_policy: "center_cutting_plunge_and_seed_orbit_at_entry_feed; seed engagement is NOT limited",
            seed_orbits: 0,
            controlled_orbits: 0,
            stock_clear_advances: 0,
            stock_query_work: 0,
            stock_query_work_exhausted: false,
        };
        // Both senses leave/rejoin the forward connector tangentially.
        let normal = if op.clockwise {
            [-direction[1], direction[0]]
        } else {
            [direction[1], -direction[0]]
        };
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            guard,
        )?;
        Ok(Self {
            heights,
            op,
            spec: spec.clone(),
            initial,
            position: initial,
            direction,
            normal,
            length,
            radius,
            cutter_radius: r,
            guard,
            pass: 0,
            distance: 0.,
            phase: 0,
            next_id: first_motion_id,
            expected_generation: None,
            report,
        })
    }
    pub fn report(&self) -> &SlotReport {
        &self.report
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("slot report cancelled".into());
        }
        if self.phase != 7 {
            return Err("drain slot motions before final report".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Estimated;
        Ok(())
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        vec![
            Event::WorkOffset {
                number: self.op.work_offset,
            },
            Event::ToolChange {
                tool: self.op.tool.clone(),
                number: self.op.tool_number,
                length_register: self.op.length_register,
                position_after_mm: self.initial,
            },
            Event::Spindle {
                rpm: self.op.spindle_rpm,
                direction: SpindleDirection::Cw,
            },
            Event::Coolant {
                mode: self.op.coolant,
            },
        ]
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        vec![
            Event::Coolant { mode: Coolant::Off },
            Event::Spindle {
                rpm: 0.,
                direction: SpindleDirection::Cw,
            },
        ]
    }
    fn center(&self, distance: f64, z: f64) -> Vec3 {
        let p = if distance == self.length {
            self.op.end_mm
        } else {
            [
                self.op.start_mm[0] + self.direction[0] * distance,
                self.op.start_mm[1] + self.direction[1] * distance,
            ]
        };
        [p[0], p[1], z]
    }
    fn orbit_start(&self, distance: f64, z: f64) -> Vec3 {
        let c = self.center(distance, z);
        [
            c[0] + self.normal[0] * self.radius,
            c[1] + self.normal[1] * self.radius,
            z,
        ]
    }
    fn emit(&mut self, curve: Curve, kind: MoveKind, entry: bool) -> Result<Motion, String> {
        curve.prepare(self.op.tolerance_mm)?;
        let m = Motion {
            id: self.next_id,
            operation: self.op.id.clone(),
            tool: self.op.tool.clone(),
            kind,
            curve,
            feed_mm_min: if kind == MoveKind::Rapid {
                None
            } else {
                Some(if entry {
                    self.op.entry_feed_mm_min
                } else {
                    self.op.feed_mm_min
                })
            },
        };
        self.next_id += 1;
        self.position = m.curve.end();
        Ok(m)
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self
                .expected_generation
                .is_some_and(|g| g != stock.generation())
        {
            return Err("commit preceding slot motion to matching stock before planning".into());
        }
        loop {
            if cancel.is_cancelled() {
                return Err("slot planning cancelled".into());
            }
            let z = (self.op.top_z_mm - (self.pass + 1) as f64 * self.op.stepdown_mm)
                .max(self.op.floor_z_mm);
            let (curve, kind, entry, next_phase) = match self.phase {
                0 => (
                    Curve::Line {
                        start: self.position,
                        end: [self.position[0], self.position[1], self.op.clearance_z_mm],
                    },
                    MoveKind::Rapid,
                    false,
                    1,
                ),
                // Travel at the current rapid height, then descend to the
                // feed height (phase 9) before plunging.
                1 => (
                    Curve::Line {
                        start: self.position,
                        end: self.orbit_start(0., self.position[2]),
                    },
                    MoveKind::Rapid,
                    false,
                    9,
                ),
                9 => (
                    Curve::Line {
                        start: self.position,
                        end: self.orbit_start(0., self.heights.feed),
                    },
                    MoveKind::Rapid,
                    false,
                    2,
                ),
                2 => (
                    Curve::Line {
                        start: self.position,
                        end: self.orbit_start(0., z),
                    },
                    MoveKind::Plunge,
                    true,
                    3,
                ),
                3 | 5 => {
                    let seed = self.phase == 3;
                    let curve = Curve::Arc {
                        start: self.position,
                        end: self.position,
                        center: self.center(self.distance, z),
                        plane: Plane::Xy,
                        sweep_rad: if self.op.clockwise { -TAU } else { TAU },
                    };
                    let motion = self.emit(curve, MoveKind::Cut, seed)?;
                    if seed {
                        self.report.seed_orbits += 1;
                    } else {
                        self.report.controlled_orbits += 1;
                    }
                    self.phase = 4;
                    self.expected_generation = Some(stock.generation() + 1);
                    return Ok(Some(motion));
                }
                4 => {
                    if self.distance == self.length {
                        self.phase = 6;
                        continue;
                    }
                    let remaining = self.length - self.distance;
                    let mut advance = self.report.bounded_advance_mm.min(remaining);
                    let mut known_clear = false;
                    if remaining > advance && !self.report.stock_query_work_exhausted {
                        let query = Motion {
                            id: self.next_id,
                            operation: self.op.id.clone(),
                            tool: self.op.tool.clone(),
                            kind: MoveKind::Link,
                            curve: Curve::Line {
                                start: self.center(self.distance, z),
                                end: self.center(self.length, z),
                            },
                            feed_mm_min: Some(self.op.feed_mm_min),
                        };
                        let mut work = usize::MAX;
                        let before = work;
                        known_clear = !stock.may_remove_material(
                            &query,
                            self.radius + self.cutter_radius,
                            0.,
                            self.op.tolerance_mm,
                            &mut work,
                            cancel,
                        )?;
                        self.report.stock_query_work += before - work;
                        self.report.stock_query_work_exhausted |= work == 0;
                        if known_clear {
                            advance = remaining;
                        }
                    }
                    // Choose exact endpoint when consuming the remainder; no cumulative overshoot.
                    let next = if advance == remaining {
                        self.length
                    } else {
                        self.distance + advance
                    };
                    let motion = self.emit(
                        Curve::Line {
                            start: self.position,
                            end: self.orbit_start(next, z),
                        },
                        MoveKind::Link,
                        false,
                    )?;
                    if known_clear {
                        self.report.stock_clear_advances += 1;
                    } else {
                        let r = self.cutter_radius;
                        let a = self.radius + r;
                        let q = self.radius + advance + self.guard;
                        let phi = 2.
                            * ((a * a - q * q - r * r) / (2. * q * r))
                                .clamp(-1., 1.)
                                .acos()
                                .to_degrees();
                        self.report.max_bounded_engagement_deg =
                            self.report.max_bounded_engagement_deg.max(phi);
                    }
                    self.distance = next;
                    self.phase = 5;
                    self.expected_generation = Some(stock.generation() + 1);
                    return Ok(Some(motion));
                }
                6 => (
                    Curve::Line {
                        start: self.position,
                        end: [self.position[0], self.position[1], self.heights.feed],
                    },
                    MoveKind::Retract,
                    false,
                    8,
                ),
                // Feed height -> retract height; clearance after the last pass.
                8 => {
                    let last = self.pass == self.report.depth_passes;
                    let z = if last {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    (
                        Curve::Line {
                            start: self.position,
                            end: [self.position[0], self.position[1], z],
                        },
                        MoveKind::Rapid,
                        false,
                        if last { 7 } else { 1 },
                    )
                }
                _ => return Ok(None),
            };
            if curve.start() == curve.end() {
                self.phase = next_phase;
                continue;
            }
            let motion = self.emit(curve, kind, entry)?;
            if self.phase == 6 {
                self.pass += 1;
                self.distance = 0.;
                self.phase = next_phase;
            } else {
                self.phase = next_phase;
            }
            self.expected_generation = Some(stock.generation() + 1);
            return Ok(Some(motion));
        }
    }
}
