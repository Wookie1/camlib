//! Native stock-front steering under development. Admission is continuous;
//! sampled potential-contact ranking is deliberately only a heuristic.
use crate::{
    Point,
    engagement::{ClearedSweep, EngagementOptions, EngagementScene, EngagementStatus},
    geometry::Bounds,
    motion::{Curve, Motion, MoveKind, Plane, Vec3},
    planar::PreparedRegion,
    pocket::{GuideTubeApproval, PocketPlan, Stroke},
    stock::{CancelSignal, Stock},
    tool::ToolAssembly,
};
use serde::Serialize;
use std::{
    collections::{HashSet, VecDeque},
    f64::consts::{PI, TAU},
};
mod peeling;
use peeling::PeelContour;
mod convex;
use convex::{ConvexContour, ConvexFront, ConvexMaterial};
mod channel;
use channel::{ChannelFront, ChannelRecipe};
mod linear_channel;
use linear_channel::{LinearChannel, LinearRecipe};
mod enclosed;
use enclosed::{EnclosedContour, EnclosedFront};

enum BulkContour {
    Linear(LinearRecipe),
    Channel(ChannelRecipe),
    Rectangle(PeelContour),
    Convex(ConvexContour),
    Enclosed(EnclosedContour),
}
impl BulkContour {
    fn expects(&self, motion: &Motion, radius: f64, tolerance: f64) -> Result<bool, String> {
        match self {
            Self::Linear(r) => r.expects(motion, radius, tolerance),
            Self::Channel(r) => r.expects(motion, radius, tolerance),
            Self::Rectangle(r) => r.expects(motion, radius, tolerance),
            Self::Convex(r) => r.expects(motion, radius, tolerance),
            Self::Enclosed(r) => r.expects(motion, radius, tolerance),
        }
    }
    fn certified_angle(&self, motion: &Motion) -> Option<f64> {
        match self {
            Self::Linear(r) => r.certified_angle(motion),
            Self::Channel(r) => r.certified_angle(motion),
            Self::Convex(r) => r.certified_angle(motion),
            Self::Enclosed(r) => r.certified_angle(motion),
            Self::Rectangle(r) => r.certified_angle(motion),
        }
    }
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct FrontReport {
    pub committed_supports: usize,
    pub front_moves: usize,
    /// Positive sampled potential among locally ranked candidates, not a
    /// certificate of actual material removal.
    pub productive_candidates: usize,
    pub navigation_moves: usize,
    /// Cut/lead moves in fixed contour recipes. They are not ranked and make
    /// no claim about positive removal or known-air travel.
    pub unranked_contour_moves: usize,
    pub candidate_tests: usize,
    pub source_rejections: usize,
    pub recovery_rejections: usize,
    /// Ranked admission uses the joint contact/side proof: its refusals.
    pub joint_rejections: usize,
    pub max_bounded_engagement_deg: f64,
    pub coverage_complete: bool,
    pub completed_components: usize,
    pub coverage_goals: usize,
    pub coverage_tiles: usize,
    pub coverage_cache_hits: usize,
    pub coverage_reused_branches: usize,
    pub coverage_envelopes: usize,
    pub reentries: usize,
    pub reentry_moves: usize,
    pub reentry_budget_stops: usize,
    pub cleared_reentry_links: usize,
    pub cleared_link_work: u64,
    pub cleared_link_budget_stops: usize,
    pub steering_budget_stops: usize,
    pub bulk_contours: usize,
    pub bulk_committed_moves: usize,
    pub bulk_abandoned: usize,
    pub bulk_material_bounds_mm: Option<[Point; 2]>,
    pub bulk_material_floor_mm: Option<f64>,
    pub convex_contours: usize,
    pub convex_resets: usize,
    pub enclosed_contours: usize,
    pub channel_contours: usize,
    pub channel_committed_moves: usize,
    pub channel_resets: usize,
    pub linear_contours: usize,
    pub linear_committed_moves: usize,
    pub linear_resets: usize,
    pub enclosed_resets: usize,
    pub enclosed_committed_moves: usize,
    pub status: &'static str,
}

type RankedFrontMove = (f64, Motion, f64, bool, u8);

// Coarse generator: 2 initial +17 grid +8 local +2 radius limits +3 goal hints,
// each at8 lengths. These are proposals, never emitted/cleared motion history.
const MAX_FRONT_PROPOSALS: usize = 32 * 8;

struct FrontProposal {
    score: f64,
    motion: Motion,
    novelty: bool,
    tier: u8,
}

struct RefinedAdvance {
    best: Option<RankedFrontMove>,
    budget_stopped: bool,
}

pub(crate) struct FrontDriver {
    scene: Option<EngagementScene>,
    tool: ToolAssembly,
    pending: Option<(u64, Motion)>,
    identity: Option<u64>,
    heading: Option<f64>,
    curvature: f64,
    navigation_steps: usize,
    progress_epoch: usize,
    pending_novelty: bool,
    coverage: Option<FrontCoverage>,
    initial_coverage_checked: bool,
    goal: Option<Point>,
    transfer: VecDeque<Transfer>,
    bulk_enabled: bool,
    bulk_used_in_component: bool,
    bulk: Option<BulkContour>,
    source_approvals: Vec<GuideTubeApproval>,
    convex_front: Option<ConvexFront>,
    enclosed_front: Option<EnclosedFront>,
    channel_front: Option<ChannelFront>,
    linear_front: Option<LinearChannel>,
    linear_rails: Option<VecDeque<Stroke>>,
    last_floor_circle: Option<Motion>,
    pending_heading: Option<(u64, f64)>,
    visited: HashSet<(u64, u64, u8, usize)>,
    pub report: FrontReport,
}
impl FrontDriver {
    pub fn new(tool: &ToolAssembly) -> Self {
        Self {
            scene: None,
            tool: tool.clone(),
            pending: None,
            identity: None,
            heading: None,
            curvature: 0.,
            navigation_steps: 0,
            progress_epoch: 0,
            pending_novelty: false,
            coverage: None,
            initial_coverage_checked: false,
            goal: None,
            transfer: VecDeque::new(),
            bulk_enabled: true,
            bulk_used_in_component: false,
            bulk: None,
            source_approvals: Vec::new(),
            convex_front: None,
            enclosed_front: None,
            channel_front: None,
            linear_front: None,
            linear_rails: None,
            last_floor_circle: None,
            pending_heading: None,
            visited: HashSet::new(),
            report: FrontReport {
                status: "startup",
                ..Default::default()
            },
        }
    }
    /// Called before any planner mutation. Matching generation alone is not
    /// enough: an unrelated/partial/snapshot-restored transaction cannot clear
    /// the pending geometric sweep. A failed synchronization retains pending.
    pub fn synchronize(
        &mut self,
        stock: &Stock,
        radius: f64,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("front.synchronize");
        if cancel.is_cancelled() {
            return Err("stock-front planning cancelled".into());
        }
        stock.request_sweep_receipts();
        if let Some((base, pending)) = &self.pending {
            let receipt = stock.last_sweep_receipt().ok_or("stock-front needs the native committed preceding sweep; snapshot/wire/undo is not sweep authority")?;
            if receipt.base_identity != *base
                || !receipt.motion.commits(pending)
                || receipt.tool != self.tool
                || receipt.fraction != 1.
                || receipt.tolerance_mm > tolerance
            {
                return Err(
                    "stock-front preceding motion/tool/fraction/stock identity mismatch".into(),
                );
            }
            let bulk_commit = self
                .bulk
                .as_ref()
                .map(|b| b.expects(pending, radius, receipt.tolerance_mm))
                .transpose()?
                .unwrap_or(false);
            self.last_floor_circle = match &pending.curve {
                Curve::Arc {
                    start,
                    end,
                    plane: Plane::Xy,
                    sweep_rad,
                    ..
                } if pending.kind == MoveKind::Cut && start == end && sweep_rad.abs() == TAU => {
                    Some(pending.clone())
                }
                _ => None,
            };
            if pending.kind != MoveKind::Rapid
                && pending.curve.start()[2] == pending.curve.end()[2]
                && matches!(
                    pending.curve,
                    Curve::Line { .. }
                        | Curve::Arc {
                            plane: Plane::Xy,
                            ..
                        }
                )
            {
                let support = ClearedSweep::from_motion(pending, radius, tolerance)?;
                self.scene
                    .as_mut()
                    .ok_or("stock-front missing scene")?
                    .append(support)?;
                self.report.committed_supports += 1;
                self.heading = Some(tangent(&pending.curve, 1.));
                self.curvature = match pending.curve {
                    Curve::Arc {
                        start,
                        center,
                        sweep_rad,
                        ..
                    } => sweep_rad.signum() / (start[0] - center[0]).hypot(start[1] - center[1]),
                    _ => 0.,
                };
            }
            if bulk_commit {
                if matches!(self.bulk, Some(BulkContour::Linear(_))) {
                    self.report.linear_committed_moves += 1;
                } else if matches!(self.bulk, Some(BulkContour::Channel(_))) {
                    self.report.channel_committed_moves += 1;
                } else if matches!(self.bulk, Some(BulkContour::Enclosed(_))) {
                    self.report.enclosed_committed_moves += 1;
                } else {
                    self.report.bulk_committed_moves += 1;
                }
                let completed = match self.bulk.as_mut().unwrap() {
                    BulkContour::Linear(recipe) => {
                        if recipe.advance() {
                            self.linear_front
                                .as_mut()
                                .ok_or("linear channel lost owner")?
                                .committed(recipe)?;
                            self.report.linear_contours += 1;
                            self.bulk = None;
                        }
                        None
                    }
                    BulkContour::Channel(recipe) => {
                        if recipe.advance() {
                            self.channel_front
                                .as_mut()
                                .ok_or("channel lost its owner")?
                                .committed(recipe)?;
                            self.report.channel_contours += 1;
                            self.bulk_used_in_component = true;
                            self.bulk = None;
                        }
                        None
                    }
                    BulkContour::Enclosed(recipe) => {
                        if recipe.advance() {
                            let owner = self
                                .enclosed_front
                                .as_mut()
                                .ok_or("enclosed contour lost owner")?;
                            owner.committed(recipe)?;
                            let coverage = self
                                .coverage
                                .as_mut()
                                .ok_or("enclosed contour has no coverage domain")?;
                            coverage.enclosed_complete = owner.covers_offset(
                                coverage.inflation,
                                coverage.domain.definition().boundary_error_mm
                                    + coverage.centres.definition().boundary_error_mm,
                                coverage.floor,
                                radius,
                                tolerance,
                            );
                            self.report.enclosed_contours += 1;
                            self.bulk = None;
                        }
                        None
                    }
                    BulkContour::Rectangle(recipe) => recipe.committed(),
                    BulkContour::Convex(recipe) => {
                        if recipe.advance() {
                            let owner = self
                                .convex_front
                                .as_mut()
                                .ok_or("convex contour lost its envelope owner")?;
                            owner.committed(recipe)?;
                            self.coverage
                                .as_mut()
                                .ok_or("convex contour has no coverage domain")?
                                .material = owner.material_envelope();
                            self.report.convex_contours += 1;
                            Some(recipe.envelope())
                        } else {
                            None
                        }
                    }
                };
                if let Some((mut bounds, floor)) = completed {
                    let old = self.scene.as_ref().unwrap().material_bounds(floor);
                    for k in 0..2 {
                        bounds[0][k] = bounds[0][k].max(old[0][k]);
                        bounds[1][k] = bounds[1][k].min(old[1][k]);
                    }
                    self.scene
                        .as_mut()
                        .unwrap()
                        .append_rectangular_shell(bounds, floor)?;
                    self.bulk = None;
                    self.report.bulk_contours += 1;
                    self.bulk_used_in_component = true;
                    self.report.bulk_material_bounds_mm = Some(bounds);
                    self.report.bulk_material_floor_mm = Some(floor);
                }
                if self.bulk.is_none() {
                    self.source_approvals.clear();
                    // A fully committed envelope starts a fresh local search
                    // if contour generation later stops. Sampled novelty on
                    // individual recipe curves does not control this state.
                    self.navigation_steps = 0;
                    self.progress_epoch += 1;
                    self.visited.clear();
                }
            }
            if let Some((id, heading)) = self.pending_heading {
                if pending.id != id {
                    return Err("stock-front re-entry commit ID mismatch".into());
                }
                self.heading = Some(heading);
                self.curvature = 0.;
                self.navigation_steps = 0;
                self.pending_heading = None;
            }
            if self.pending_novelty {
                self.progress_epoch += 1;
                self.visited.clear();
                self.pending_novelty = false;
            }
            self.pending = None;
        } else if self.identity.is_some_and(|id| id != stock.identity()) {
            return Err("stock-front stock changed without the pending motion".into());
        }
        if self.scene.is_none() {
            // Only actual native full flat sweeps carry across operations.
            // Snapshot cells and a matching nominal generation are not sweeps.
            self.scene = Some(EngagementScene::new_with_cancel(
                stock.spec().clone(),
                stock.committed_clearance(tolerance, cancel)?,
                cancel,
            )?);
        }
        self.identity = Some(stock.identity());
        Ok(())
    }
    pub fn emitted(&mut self, stock: &Stock, motion: &Motion) {
        self.pending = Some((stock.identity(), motion.clone()));
    }
    #[allow(clippy::too_many_arguments)]
    pub fn begin_component(
        &mut self,
        source: &PocketPlan,
        seed: Point,
        key: (usize, usize),
        floor: f64,
        cutter_radius: f64,
        minimum_radius: f64,
        inflation: f64,
        tolerance: f64,
        boundary_budget: f64,
    ) -> Result<(), String> {
        if self.coverage.as_ref().is_some_and(|c| c.key == key) {
            return Ok(());
        }
        if !self.transfer.is_empty() || self.pending.is_some() {
            return Err("stock-front component change has pending motion".into());
        }
        let (centres, domain) = source.front_component_region(seed, inflation)?;
        self.enclosed_front = self.last_floor_circle.as_ref().and_then(|motion| {
            EnclosedFront::new(
                centres.definition(),
                motion,
                cutter_radius,
                minimum_radius,
                tolerance,
            )
        });
        self.channel_front = self.last_floor_circle.as_ref().and_then(|motion| {
            let channel = ChannelFront::new(
                centres.definition(),
                motion,
                cutter_radius,
                minimum_radius,
                tolerance,
            )?;
            let (rail, radius) = channel.rail_tube();
            source.guide_tube_fits(rail, radius).then_some(channel)
        });
        self.coverage = Some(FrontCoverage::new(
            domain,
            seed,
            key,
            floor,
            tolerance,
            boundary_budget,
            self.curvature.recip().abs(),
            centres,
            inflation,
        )?);
        self.goal = None;
        self.initial_coverage_checked = false;
        self.linear_front = None;
        self.linear_rails = None;
        self.visited.clear();
        self.navigation_steps = 0;
        self.report.coverage_complete = false;
        self.bulk_enabled = true;
        self.bulk_used_in_component = false;
        self.bulk = None;
        self.source_approvals.clear();
        let material = self
            .scene
            .as_ref()
            .ok_or("stock-front component has no committed scene")?
            .material_bounds(floor);
        self.convex_front = ConvexFront::new(
            source.front_obstacles(seed)?,
            material,
            floor,
            source.cutter_axis_clearance(cutter_radius) + 32. * tolerance,
            tolerance,
        );
        Ok(())
    }
    #[allow(clippy::too_many_arguments)]
    fn prepare_convex(
        &mut self,
        source: &PocketPlan,
        position: Vec3,
        radius: f64,
        minimum_radius: f64,
        max_angle: f64,
        boundary_budget: f64,
        tolerance: f64,
        right: bool,
        prototype: &Motion,
        clearance: LinkHeights,
        plunge_feed: f64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let Some(front) = self.convex_front.as_ref() else {
            return Ok(false);
        };
        let had_commit = front.has_commit();
        let recipe = front.prepare(
            position,
            radius,
            minimum_radius,
            max_angle,
            boundary_budget,
            tolerance,
            right,
            cancel,
        )?;
        let Some(recipe) = recipe else {
            self.convex_front = None;
            if had_commit {
                self.bulk_enabled = false;
                return Ok(true);
            }
            return Ok(false);
        };
        let mut approvals = Vec::new();
        for curve in recipe.leads.iter().chain(&recipe.contour) {
            if cancel.is_cancelled() {
                return Err("convex source admission cancelled".into());
            }
            self.report.candidate_tests += 1;
            let Some(approval) = source.approve_guide_tube(curve, radius) else {
                self.report.source_rejections += 1;
                self.convex_front = None;
                if had_commit {
                    self.bulk_enabled = false;
                    return Ok(true);
                }
                return Ok(false);
            };
            approvals.push(approval);
        }
        let mut queue = VecDeque::new();
        if recipe.reset {
            let p = [recipe.start[0], recipe.start[1]];
            // The reset disk's whole-empty premise is proved by the current
            // convex envelope; its entire source tube is still independently fit.
            if !source.link_geometry_fits_radius(p, p, radius + 64. * tolerance) {
                self.convex_front = None;
                return Ok(false);
            }
            push_relink(
                &mut queue,
                position,
                recipe.start,
                clearance,
                prototype.feed_mm_min,
                plunge_feed,
                Some(recipe.heading),
            );
            self.report.convex_resets += 1;
            self.report.reentries += 1;
        } else if recipe.start != position {
            return Err("convex continuing seam mismatch".into());
        }
        for curve in &recipe.leads {
            queue.push_back(Transfer {
                curve: curve.clone(),
                kind: MoveKind::Lead,
                feed: prototype.feed_mm_min,
                heading: None,
            });
        }
        for curve in &recipe.contour {
            queue.push_back(Transfer {
                curve: curve.clone(),
                kind: MoveKind::Cut,
                feed: prototype.feed_mm_min,
                heading: None,
            });
        }
        self.bulk = Some(BulkContour::Convex(recipe));
        self.transfer = queue;
        self.source_approvals = approvals;
        self.report.status = "convex_contour";
        Ok(true)
    }
    #[allow(clippy::too_many_arguments)]
    fn prepare_enclosed(
        &mut self,
        source: &PocketPlan,
        position: Vec3,
        radius: f64,
        max_angle: f64,
        tolerance: f64,
        right: bool,
        prototype: &Motion,
        clearance: LinkHeights,
        plunge_feed: f64,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let recipe = self
            .enclosed_front
            .as_ref()
            .unwrap()
            .prepare(position, max_angle, tolerance, right, cancel)?;
        let Some(recipe) = recipe else {
            self.enclosed_front = None;
            self.bulk_enabled = false;
            return Ok(());
        };
        let mut approvals = Vec::new();
        for curve in recipe.leads.iter().chain(&recipe.contour) {
            if cancel.is_cancelled() {
                return Err("enclosed source admission cancelled".into());
            }
            self.report.candidate_tests += 1;
            let Some(approval) = source.approve_guide_tube(curve, radius) else {
                self.report.source_rejections += 1;
                self.enclosed_front = None;
                self.bulk_enabled = false;
                return Ok(());
            };
            approvals.push(approval);
        }
        let mut queue = VecDeque::new();
        if recipe.reset {
            let p = [recipe.start[0], recipe.start[1]];
            let pad = 64. * tolerance;
            if !source.link_geometry_fits_radius(p, p, radius + pad)
                || !self
                    .scene
                    .as_ref()
                    .ok_or("enclosed entry has no scene")?
                    .clear_disk(
                        p,
                        radius + pad,
                        position[2],
                        tolerance,
                        &mut 2_000_000,
                        cancel,
                    )?
            {
                self.enclosed_front = None;
                self.bulk_enabled = false;
                return Ok(());
            }
            push_relink(
                &mut queue,
                position,
                recipe.start,
                clearance,
                prototype.feed_mm_min,
                plunge_feed,
                Some(recipe.heading),
            );
            self.report.enclosed_resets += 1;
            self.report.reentries += 1;
        }
        for curve in &recipe.leads {
            queue.push_back(Transfer {
                curve: curve.clone(),
                kind: MoveKind::Lead,
                feed: prototype.feed_mm_min,
                heading: None,
            });
        }
        for curve in &recipe.contour {
            queue.push_back(Transfer {
                curve: curve.clone(),
                kind: MoveKind::Cut,
                feed: prototype.feed_mm_min,
                heading: None,
            });
        }
        if cancel.is_cancelled() {
            return Err("enclosed contour cancelled".into());
        }
        self.bulk = Some(BulkContour::Enclosed(recipe));
        self.transfer = queue;
        self.source_approvals = approvals;
        self.report.status = "enclosed_contour";
        Ok(())
    }
    #[allow(clippy::too_many_arguments)]
    fn prepare_bulk(
        &mut self,
        source: &PocketPlan,
        _stock: &Stock,
        position: Vec3,
        radius: f64,
        minimum_radius: f64,
        max_angle: f64,
        right: bool,
        prototype: &Motion,
        tolerance: f64,
        clearance: LinkHeights,
        plunge_feed: f64,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("front.bulk");
        if self.bulk.is_some() {
            return Err("stock-front contour needs its preceding full commit".into());
        }
        self.source_approvals.clear();
        if cancel.is_cancelled() {
            return Err("stock-front contour preparation cancelled".into());
        }
        let boundary_budget = self
            .coverage
            .as_ref()
            .ok_or("stock-front contour has no coverage domain")?
            .goal_pad
            * 4.;
        if let Some(channel) = &self.channel_front {
            let recipe = match channel.prepare(position, max_angle, tolerance, right, cancel)? {
                Some(recipe) => Some(recipe),
                None => channel.prepare_inner(
                    max_angle,
                    tolerance,
                    right,
                    |stroke| source.guide_tube_fits(stroke, radius),
                    cancel,
                )?,
            };
            if let Some(recipe) = recipe {
                let mut approvals = Vec::new();
                for curve in &recipe.curves {
                    if cancel.is_cancelled() {
                        return Err("channel source admission cancelled".into());
                    }
                    self.report.candidate_tests += 1;
                    let Some(approval) = source.approve_guide_tube(curve, radius) else {
                        self.report.source_rejections += 1;
                        self.channel_front = None;
                        self.bulk_enabled = false;
                        return Ok(());
                    };
                    approvals.push(approval);
                }
                let mut queue = VecDeque::new();
                if recipe.reset {
                    let start = recipe.curves[0].start();
                    let p = [start[0], start[1]];
                    let pad = 64. * tolerance;
                    if !source.link_geometry_fits_radius(p, p, radius + pad)
                        || !self
                            .scene
                            .as_ref()
                            .ok_or("channel entry has no scene")?
                            .clear_disk(
                                p,
                                radius + pad,
                                position[2],
                                tolerance,
                                &mut 2_000_000,
                                cancel,
                            )?
                    {
                        self.channel_front = None;
                        self.bulk_enabled = false;
                        return Ok(());
                    }
                    push_relink(
                        &mut queue,
                        position,
                        start,
                        clearance,
                        prototype.feed_mm_min,
                        plunge_feed,
                        Some(tangent(&recipe.curves[0], 0.)),
                    );
                    self.report.channel_resets += 1;
                    self.report.reentries += 1;
                }
                queue.extend(recipe.curves.iter().map(|curve| Transfer {
                    curve: curve.clone(),
                    kind: MoveKind::Cut,
                    feed: prototype.feed_mm_min,
                    heading: None,
                }));
                self.transfer = queue;
                self.source_approvals = approvals;
                self.bulk = Some(BulkContour::Channel(recipe));
                self.report.status = "channel_contour";
            } else {
                self.channel_front = None;
                self.bulk_enabled = false;
            }
            return Ok(());
        }
        if self.enclosed_front.is_some() {
            return self.prepare_enclosed(
                source,
                position,
                radius,
                max_angle,
                tolerance,
                right,
                prototype,
                clearance,
                plunge_feed,
                cancel,
            );
        }
        let scene = self
            .scene
            .as_ref()
            .ok_or("stock-front contour has no committed scene")?;
        let material = scene.material_bounds(position[2]);
        let continuing_convex = self
            .convex_front
            .as_ref()
            .is_some_and(ConvexFront::has_commit);
        // Keep the outer contours close to rectangular stock. Once peeling can
        // no longer advance, switch to the retained-part hull using only the
        // material envelope established by completed native recipes.
        // The fitted rectangle performs a bounded search. Retain only actual
        // successful proofs temporarily, then prune to the selected recipe.
        let mut rectangle_approvals = Vec::new();
        let rectangle = if continuing_convex {
            None
        } else {
            PeelContour::prepare_fitted(
                material,
                position[2],
                radius,
                minimum_radius,
                max_angle,
                boundary_budget,
                tolerance,
                right,
                |curve| {
                    if cancel.is_cancelled() {
                        return Err("stock-front contour preparation cancelled".into());
                    }
                    self.report.candidate_tests += 1;
                    let approval = source.approve_guide_tube(curve, radius);
                    let fits = approval.is_some();
                    if let Some(approval) = approval {
                        rectangle_approvals.push(approval);
                    }
                    self.report.source_rejections += usize::from(!fits);
                    Ok(fits)
                },
            )?
        };
        let pad = 64. * tolerance;
        let rectangle = if let Some(recipe) = rectangle {
            let p = [recipe.start[0], recipe.start[1]];
            if source.link_geometry_fits_radius(p, p, radius + pad)
                && scene.clear_disk(
                    p,
                    radius + pad,
                    position[2],
                    tolerance,
                    &mut 2_000_000,
                    cancel,
                )?
            {
                Some(recipe)
            } else {
                None
            }
        } else {
            None
        };
        let Some(recipe) = rectangle else {
            if !continuing_convex && let Some(front) = self.convex_front.as_mut() {
                front.restrict_material(material, tolerance)?;
            }
            if !self.prepare_convex(
                source,
                position,
                radius,
                minimum_radius,
                max_angle,
                boundary_budget,
                tolerance,
                right,
                prototype,
                clearance,
                plunge_feed,
                cancel,
            )? {
                self.bulk_enabled = false;
            }
            return Ok(());
        };
        let mut queue = VecDeque::new();
        push_relink(
            &mut queue,
            position,
            recipe.start,
            clearance,
            prototype.feed_mm_min,
            plunge_feed,
            Some(recipe.heading),
        );
        for curve in &recipe.leads {
            queue.push_back(Transfer {
                curve: curve.clone(),
                kind: MoveKind::Lead,
                feed: prototype.feed_mm_min,
                heading: None,
            });
        }
        for curve in &recipe.contour {
            queue.push_back(Transfer {
                curve: curve.clone(),
                kind: MoveKind::Cut,
                feed: prototype.feed_mm_min,
                heading: None,
            });
        }
        if cancel.is_cancelled() {
            return Err("stock-front contour preparation cancelled".into());
        }
        self.source_approvals = recipe
            .leads
            .iter()
            .chain(&recipe.contour)
            .filter_map(|curve| {
                rectangle_approvals
                    .iter()
                    .position(|p| p.matches(source, curve, radius, tolerance))
                    .map(|i| rectangle_approvals.swap_remove(i))
            })
            .collect();
        self.bulk = Some(BulkContour::Rectangle(recipe));
        self.transfer = queue;
        self.report.reentries += 1;
        self.report.status = "bulk_contour";
        Ok(())
    }
    #[allow(clippy::too_many_arguments)]
    fn prepare_linear(
        &mut self,
        source: &PocketPlan,
        position: Vec3,
        radius: f64,
        minimum: f64,
        max_angle: f64,
        right: bool,
        prototype: &Motion,
        tolerance: f64,
        clearance: LinkHeights,
        plunge_feed: f64,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("linear continuation cancelled".into());
        }
        let coverage = self
            .coverage
            .as_ref()
            .ok_or("linear channel has no domain")?;
        let orbit = coverage.seed_orbit.max(minimum).min(radius);
        let scene = self.scene.as_ref().ok_or("linear channel has no scene")?;
        let pad = 64. * tolerance;
        if self.linear_rails.is_none() {
            self.linear_rails = Some(
                source
                    .front_channel_rails(
                        coverage.seed,
                        scene.material_bounds(position[2]),
                        radius + orbit + 2. * pad,
                    )?
                    .into(),
            );
        }
        loop {
            if cancel.is_cancelled() {
                return Err("linear continuation cancelled".into());
            }
            if self.linear_front.is_none() {
                // Admission can be interrupted inside the scene proof. Keep the
                // unstarted proposal available until that proof has completed.
                let Some(mut rail) = self.linear_rails.as_ref().unwrap().front().copied() else {
                    return Ok(());
                };
                let distance = |p: Point| (position[0] - p[0]).hypot(position[1] - p[1]);
                if distance(rail.b) < distance(rail.a) {
                    std::mem::swap(&mut rail.a, &mut rail.b);
                }
                self.report.candidate_tests += 1;
                if !source.guide_tube_fits(rail, radius + orbit + pad)
                    || !scene.clear_disk(
                        rail.a,
                        radius + orbit + pad,
                        position[2],
                        tolerance,
                        &mut 2_000_000,
                        cancel,
                    )?
                {
                    self.linear_rails.as_mut().unwrap().pop_front();
                    continue;
                }
                self.linear_rails.as_mut().unwrap().pop_front();
                self.linear_front =
                    LinearChannel::new(rail, position[2], orbit, radius, minimum, tolerance);
                if self.linear_front.is_none() {
                    continue;
                }
            }
            let recipe = self.linear_front.as_ref().unwrap().prepare(
                position,
                max_angle,
                tolerance,
                right,
                |stroke| source.guide_tube_fits(stroke, radius),
                cancel,
            )?;
            let Some(recipe) = recipe else {
                self.linear_front = None;
                continue;
            };
            if recipe
                .curves
                .iter()
                .any(|c| !source.guide_tube_fits(stroke_from_curve(c), radius))
            {
                self.linear_front = None;
                continue;
            }
            let mut queue = VecDeque::new();
            if recipe.reset {
                let start = recipe.curves[0].start();
                let p = [start[0], start[1]];
                if !source.link_geometry_fits_radius(p, p, radius + pad)
                    || !scene.clear_disk(
                        p,
                        radius + pad,
                        position[2],
                        tolerance,
                        &mut 2_000_000,
                        cancel,
                    )?
                {
                    self.linear_front = None;
                    continue;
                }
                push_relink(
                    &mut queue,
                    position,
                    start,
                    clearance,
                    prototype.feed_mm_min,
                    plunge_feed,
                    Some(tangent(&recipe.curves[0], 0.)),
                );
                self.report.linear_resets += 1;
                self.report.reentries += 1;
            }
            queue.extend(recipe.curves.iter().map(|c| Transfer {
                curve: c.clone(),
                kind: MoveKind::Cut,
                feed: prototype.feed_mm_min,
                heading: None,
            }));
            self.transfer = queue;
            self.bulk = Some(BulkContour::Linear(recipe));
            self.report.status = "linear_channel";
            return Ok(());
        }
    }
    #[allow(clippy::too_many_arguments)]
    pub fn next(
        &mut self,
        source: &PocketPlan,
        stock: &Stock,
        position: Vec3,
        radius: f64,
        minimum_radius: f64,
        max_angle: f64,
        material_on_right: bool,
        prototype: Motion,
        tolerance: f64,
        clearance: LinkHeights,
        plunge_feed: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("front.next");
        // A valid clearing recipe need not remove material. Check actual stock
        // before starting one, so a repeated operation can finish at its seed.
        // Only the existing whole-domain proof may declare coverage complete.
        if !self.initial_coverage_checked {
            let coverage = self
                .coverage
                .as_mut()
                .ok_or("stock-front has no component coverage domain")?;
            let revision = (
                self.identity
                    .ok_or("stock-front requires synchronized stock")?,
                stock.generation(),
            );
            let checked = if coverage
                .material_gap(stock, revision, position, tolerance, cancel)?
                .is_some()
            {
                // This proves remaining material, not a navigation target.
                // A fixed recipe needs no target; if it cannot proceed, the
                // generic steering branch obtains its original quadtree goal.
                None
            } else {
                Some(
                    coverage.check(
                        self.scene
                            .as_ref()
                            .ok_or("stock-front requires committed stock")?,
                        stock,
                        position,
                        radius,
                        tolerance,
                        &mut self.report.coverage_tiles,
                        &mut self.report.coverage_envelopes,
                        &mut self.report.coverage_cache_hits,
                        &mut self.report.coverage_reused_branches,
                        cancel,
                    )?,
                )
            };
            self.initial_coverage_checked = true;
            match checked {
                Some(CoverageResult::Complete) => {
                    self.report.coverage_complete = true;
                    self.report.completed_components += 1;
                    self.report.status = "covered";
                    return Ok(None);
                }
                Some(CoverageResult::Gap(goal)) => self.goal = Some(goal),
                None => {}
            }
        }
        if self.transfer.is_empty() && self.bulk_enabled && !self.report.coverage_complete {
            self.prepare_bulk(
                source,
                stock,
                position,
                radius,
                minimum_radius,
                max_angle,
                material_on_right,
                &prototype,
                tolerance,
                clearance,
                plunge_feed,
                cancel,
            )?;
        }
        if self.transfer.is_empty() && !self.bulk_enabled && self.bulk_used_in_component {
            self.prepare_linear(
                source,
                position,
                radius,
                minimum_radius,
                max_angle,
                material_on_right,
                &prototype,
                tolerance,
                clearance,
                plunge_feed,
                cancel,
            )?;
        }
        if let Some(transfer) = self.transfer.front() {
            if cancel.is_cancelled() {
                return Err("stock-front transfer cancelled".into());
            }
            let motion = Motion {
                curve: transfer.curve.clone(),
                kind: transfer.kind,
                feed_mm_min: transfer.feed,
                ..prototype.clone()
            };
            if motion.curve.start() != position {
                return Err("stock-front transfer cursor mismatch".into());
            }
            let lead = if matches!(motion.kind, MoveKind::Lead | MoveKind::Cut) {
                if self.report.front_moves >= 8192 {
                    return Err("stock-front motion work limit; remaining coverage unproven".into());
                }
                let approved = self.bulk.is_some()
                    && self
                        .source_approvals
                        .iter()
                        .any(|p| p.matches(source, &motion.curve, radius, tolerance));
                if !approved && !source.guide_tube_fits(stroke_from_curve(&motion.curve), radius) {
                    return Err("stock-front lead source unresolved; coverage unproven".into());
                }
                let scene = self
                    .scene
                    .as_ref()
                    .ok_or("stock-front lead has no committed scene")?;
                let certified_angle = self.bulk.as_ref().and_then(|b| b.certified_angle(&motion));
                let checked = if let Some(angle) = certified_angle {
                    (EngagementStatus::Bounded, angle)
                } else {
                    let checked = scene.check_front(
                        &motion,
                        radius,
                        max_angle,
                        material_on_right,
                        EngagementOptions {
                            tolerance_mm: tolerance,
                            resolution_mm: (32. * tolerance).max(0.001),
                            max_nodes: 8192,
                            max_work: 2_000_000,
                            ..Default::default()
                        },
                        cancel,
                    )?;
                    (checked.status, checked.upper_angle_deg)
                };
                if checked.0 != EngagementStatus::Bounded {
                    if self.bulk.is_some() {
                        // No completed shell exists: retain actual swept
                        // supports, abandon only the proposed remainder.
                        self.bulk = None;
                        self.convex_front = None;
                        self.enclosed_front = None;
                        self.linear_front = None;
                        self.bulk_enabled = false;
                        self.transfer.clear();
                        self.report.bulk_abandoned += 1;
                        return self.next(
                            source,
                            stock,
                            position,
                            radius,
                            minimum_radius,
                            max_angle,
                            material_on_right,
                            prototype,
                            tolerance,
                            clearance,
                            plunge_feed,
                            cancel,
                        );
                    }
                    return Err(
                        "stock-front lead engagement/side unresolved; coverage unproven".into(),
                    );
                }
                let novelty = if self.bulk.is_some() {
                    // The recipe has already fixed this curve. Ranking its
                    // potential removal cannot change the selected motion.
                    None
                } else {
                    let (_, novelty, _) = potential_score(
                        scene,
                        stock,
                        &motion,
                        radius,
                        material_on_right,
                        tolerance,
                        &mut 2_000_000,
                        self.goal,
                        self.coverage
                            .as_ref()
                            .ok_or("stock-front lead has no coverage domain")?
                            .goal_pad,
                        cancel,
                    )?;
                    Some(novelty)
                };
                Some((checked.1, novelty))
            } else {
                None
            };
            if cancel.is_cancelled() {
                return Err("stock-front transfer cancelled".into());
            }
            let transfer = self.transfer.pop_front().unwrap();
            self.pending_novelty = lead.is_some_and(|(_, novelty)| novelty == Some(true));
            if let Some((angle, novelty)) = lead {
                self.report.front_moves += 1;
                self.report.max_bounded_engagement_deg =
                    self.report.max_bounded_engagement_deg.max(angle);
                match novelty {
                    Some(true) => {
                        self.report.productive_candidates += 1;
                        self.navigation_steps = 0;
                    }
                    Some(false) => {
                        self.report.navigation_moves += 1;
                        self.navigation_steps += 1;
                    }
                    None => self.report.unranked_contour_moves += 1,
                }
            }
            if let Some(heading) = transfer.heading {
                self.pending_heading = Some((motion.id, heading));
            }
            if motion.kind != MoveKind::Cut {
                self.report.reentry_moves += 1;
            }
            return Ok(Some(motion));
        }
        if self.report.coverage_complete {
            return Ok(None);
        }
        self.report.status = "steering";
        if self.report.front_moves >= 8192 {
            self.report.status = "work_limit_coverage_unproven";
            return Err("stock-front motion work limit; remaining coverage unproven".into());
        }
        let heading = self
            .heading
            .ok_or("stock-front requires a committed flat floor seed")?;
        let scene = self
            .scene
            .as_ref()
            .ok_or("stock-front requires committed stock")?;
        let coverage = self
            .coverage
            .as_mut()
            .ok_or("stock-front has no component coverage domain")?;
        let mut coverage_work = 20_000_000;
        let goal_known_clear = if let Some(goal) = self.goal {
            let pad = coverage.goal_pad;
            let bounds = [goal.map(|v| v - pad), goal.map(|v| v + pad)];
            scene.clear_box(bounds, position[2], &mut coverage_work, cancel)?
                || stock.bounds_clear(bounds, position[2], &mut coverage_work, cancel)?
        } else {
            true
        };
        if goal_known_clear {
            let had_goal = self.goal.is_some();
            let checked = coverage.check(
                scene,
                stock,
                position,
                radius,
                tolerance,
                &mut self.report.coverage_tiles,
                &mut self.report.coverage_envelopes,
                &mut self.report.coverage_cache_hits,
                &mut self.report.coverage_reused_branches,
                cancel,
            )?;
            if had_goal {
                self.progress_epoch += 1;
                self.visited.clear();
                self.report.coverage_goals += 1;
            }
            match checked {
                CoverageResult::Complete => {
                    self.report.coverage_complete = true;
                    self.report.completed_components += 1;
                    self.report.status = "covered";
                    return Ok(None);
                }
                CoverageResult::Gap(goal) => self.goal = Some(goal),
            }
        }
        let sign = if material_on_right { 1. } else { -1. };
        let options = EngagementOptions {
            tolerance_mm: tolerance,
            resolution_mm: (tolerance * 32.).max(0.001),
            max_nodes: 8192,
            max_work: 2_000_000,
            ..Default::default()
        };
        let mut navigation = Vec::new();
        let mut ranking_work = 2_000_000;
        let mut proposals = Vec::with_capacity(MAX_FRONT_PROPOSALS);
        let mut rank_error = None;
        let mut curvatures = vec![self.curvature, 0.];
        for i in -8..=8 {
            curvatures.push(i as f64 * sign / (4. * radius));
        }
        for k in [
            -0.25, -0.125, -0.0625, -0.03125, 0.03125, 0.0625, 0.125, 0.25,
        ] {
            curvatures.push(self.curvature + k / radius);
        }
        curvatures.extend([-1. / minimum_radius, 1. / minimum_radius]);
        if let Some(goal) = self.goal {
            let target = coverage.centre_near(goal, tolerance)?;
            if let Some(k) = goal_curvature(position, heading, target, tolerance) {
                // A tangent circle through a remaining-domain centre supplies
                // geometry-dependent proposals between the fixed grid choices.
                // Blends let admission choose a gentler approach to the front.
                // None of these proposals carries clearance/engagement authority.
                curvatures.extend([k, k * 0.5, (k + self.curvature) * 0.5]);
            }
        }
        curvatures.sort_by(f64::total_cmp);
        curvatures.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        // Local cleanup follows a remaining-coverage goal with the validated
        // curvature order. Bulk contours handle coherent high-volume advances.

        'steering: for k in curvatures {
            if k != 0. && k.recip().abs() + tolerance < minimum_radius {
                continue;
            }
            let mut previous_length: Option<f64> = None;
            for scale in [4., 2., 1., 0.5, 0.25, 0.125, 0.0625, 0.03125] {
                if cancel.is_cancelled() {
                    return Err("stock-front steering cancelled".into());
                }
                let length = (radius * scale).min(if k == 0. {
                    f64::INFINITY
                } else {
                    PI / 2. / k.abs()
                });
                if previous_length.is_some_and(|old| (old - length).abs() < tolerance) {
                    continue;
                }
                previous_length = Some(length);
                let curve = candidate(position, heading, k, length);
                let stroke = stroke_from_curve(&curve);
                self.report.candidate_tests += 1;
                if !source.guide_tube_fits(stroke, radius) {
                    self.report.source_rejections += 1;
                    continue;
                }
                let end = curve.end();
                let end_heading = tangent(&curve, 1.);
                let recovery = candidate(
                    end,
                    end_heading,
                    sign / minimum_radius,
                    TAU * minimum_radius,
                );
                let recovery_fits = source.guide_tube_fits(stroke_from_curve(&recovery), radius);
                if !recovery_fits {
                    self.report.recovery_rejections += 1;
                }
                let motion = Motion {
                    curve,
                    ..prototype.clone()
                };
                let ranked = goal_score(
                    scene,
                    stock,
                    &motion,
                    radius,
                    material_on_right,
                    tolerance,
                    &mut ranking_work,
                    self.goal,
                    coverage.goal_pad,
                    cancel,
                );
                let (score, novelty, tier) = match ranked {
                    Ok(rank) => rank,
                    Err(e) if ranking_work == 0 && !e.contains("cancelled") => {
                        // Preserve the ranked prefix. Its highest actually
                        // admitted member can still be returned, as with the
                        // former incremental checked-best selection.
                        rank_error = Some(e);
                        break 'steering;
                    }
                    Err(e) => return Err(e),
                };
                let score = score * if recovery_fits { 1. } else { 0.9 };
                if score <= tolerance {
                    navigation.push(((k - sign / minimum_radius).abs(), motion));
                    continue;
                }
                if proposals.len() >= MAX_FRONT_PROPOSALS {
                    return Err("stock-front candidate buffer work limit; coverage unproven".into());
                }
                proposals.push(FrontProposal {
                    score,
                    motion,
                    novelty,
                    tier,
                });
            }
        }
        let mut best = admit_ranked_front(
            proposals,
            rank_error,
            scene,
            radius,
            max_angle,
            material_on_right,
            options,
            &mut self.report,
            cancel,
        )?;
        if best.is_none() && self.navigation_steps < 16 {
            // Reach another part of the front through admitted tangent LINKS.
            // They are not an empty-stock assertion or a startup exception.
            navigation.sort_by(|a, b| a.0.total_cmp(&b.0));
            for (_, mut motion) in navigation {
                let checked = scene.check_front(
                    &motion,
                    radius,
                    max_angle,
                    material_on_right,
                    options,
                    cancel,
                )?;
                if checked.status == EngagementStatus::Bounded {
                    motion.kind = MoveKind::Link;
                    best = Some((0., motion, checked.upper_angle_deg, false, 0));
                    break;
                }
            }
        }
        if best.is_none() && self.bulk_used_in_component {
            // A recorded straight-edge transition had a useful 4r advance near
            // k=-0.109/r between coarse choices; the old grid chose a 1r oscillation.
            // Rescue a stalled post-bulk cleanup with the remaining budget.
            let mut refinement_work = ranking_work.min(1_000_000);
            let refined = refine_long_advance(
                scene,
                stock,
                position,
                heading,
                radius,
                minimum_radius,
                max_angle,
                material_on_right,
                &prototype,
                tolerance,
                self.goal,
                coverage.goal_pad,
                &mut refinement_work,
                |curve| {
                    self.report.candidate_tests += 1;
                    if !source.guide_tube_fits(stroke_from_curve(curve), radius) {
                        self.report.source_rejections += 1;
                        return None;
                    }
                    let recovery = candidate(
                        curve.end(),
                        tangent(curve, 1.),
                        sign / minimum_radius,
                        TAU * minimum_radius,
                    );
                    let fits = source.guide_tube_fits(stroke_from_curve(&recovery), radius);
                    self.report.recovery_rejections += usize::from(!fits);
                    Some(if fits { 1. } else { 0.9 })
                },
                cancel,
            )?;
            self.report.steering_budget_stops += usize::from(refined.budget_stopped);
            best = refined.best;
        }
        let Some((_, mut motion, angle, novelty, _)) = best else {
            // Re-evaluate the WHOLE remaining domain; a failed candidate set is
            // neither completed coverage nor proof of an actual violation.
            let goal = match coverage.check(
                scene,
                stock,
                position,
                radius,
                tolerance,
                &mut self.report.coverage_tiles,
                &mut self.report.coverage_envelopes,
                &mut self.report.coverage_cache_hits,
                &mut self.report.coverage_reused_branches,
                cancel,
            )? {
                CoverageResult::Complete => {
                    self.report.coverage_complete = true;
                    self.report.completed_components += 1;
                    self.report.status = "covered";
                    return Ok(None);
                }
                CoverageResult::Gap(goal) => goal,
            };
            let search = |segment_entry| {
                plan_reentry(
                    scene,
                    source,
                    stock,
                    coverage.seed,
                    coverage.seed_orbit,
                    minimum_radius,
                    max_angle,
                    coverage.goal_pad,
                    coverage.centre_near(goal, tolerance)?,
                    position,
                    goal,
                    radius,
                    tolerance,
                    material_on_right,
                    self.progress_epoch,
                    &self.visited,
                    segment_entry,
                    1_000_000,
                    &prototype,
                    clearance,
                    plunge_feed,
                    cancel,
                )
            };
            // Share the existing two-million-work ceiling across distinct
            // proposal families. No cursor/stock mutation occurs during either
            // search; the fallback must independently prove its descent/exit.
            let result = match search(true) {
                Ok(Some(found)) => Some(found),
                Ok(None) => search(false)?,
                Err(e) if e.contains("work limit") && !e.contains("cancelled") => search(false)?,
                Err(e) => return Err(e),
            };
            let Some((key, mut queue, truncated)) = result else {
                self.report.status = "steering_stalled_coverage_unproven";
                return Err(
                    "stock-front cannot reach remaining coverage through proven-clear re-entry"
                        .into(),
                );
            };
            // Replace only the approach to an already checked re-entry pose.
            // Keep the lead and its chosen heading; a proven empty horizontal
            // link may change direction there without introducing engagement.
            let mut link_work = 100_000;
            let linked = clear_reentry_link(
                scene,
                source,
                stock,
                position,
                radius,
                tolerance,
                &prototype,
                &mut queue,
                &mut link_work,
                cancel,
            );
            self.report.cleared_link_work += 100_000 - link_work;
            match linked {
                Ok(true) => self.report.cleared_reentry_links += 1,
                Ok(false) => {}
                Err(e) if link_work == 0 && !e.contains("cancelled") => {
                    self.report.cleared_link_budget_stops += 1;
                }
                Err(e) => return Err(e),
            }
            if cancel.is_cancelled() {
                return Err("stock-front re-entry cancelled".into());
            }
            self.goal = Some(goal);
            self.report.reentry_budget_stops += usize::from(truncated);
            self.visited.insert(key);
            self.pending_novelty = false;
            self.transfer = queue;
            self.report.reentries += 1;
            self.report.status = "reentering";
            let transfer = self
                .transfer
                .pop_front()
                .ok_or("stock-front empty re-entry")?;
            let motion = Motion {
                curve: transfer.curve,
                kind: transfer.kind,
                feed_mm_min: transfer.feed,
                ..prototype
            };
            if let Some(heading) = transfer.heading {
                self.pending_heading = Some((motion.id, heading));
            }
            self.report.reentry_moves += 1;
            return Ok(Some(motion));
        };
        if cancel.is_cancelled() {
            return Err("stock-front steering cancelled".into());
        }
        self.pending_novelty = novelty;
        if novelty {
            self.navigation_steps = 0;
            self.report.productive_candidates += 1;
        } else {
            motion.kind = MoveKind::Link;
            self.navigation_steps += 1;
            self.report.navigation_moves += 1;
        }
        self.report.front_moves += 1;
        self.report.max_bounded_engagement_deg = self.report.max_bounded_engagement_deg.max(angle);
        Ok(Some(motion))
    }
}
/// Rank only immutable, source-admitted proposals. A stable descending order
/// preserves the old first-in-generator tie choice. First jointly bounded is
/// the highest-ranked proposal admitted by the shared continuous proof.
/// Uncertainty, ranking, point probes and exhausted work never admit a motion.
#[allow(clippy::too_many_arguments)]
fn admit_ranked_front(
    mut proposals: Vec<FrontProposal>,
    rank_error: Option<String>,
    scene: &EngagementScene,
    radius: f64,
    max_angle: f64,
    right: bool,
    options: EngagementOptions,
    report: &mut FrontReport,
    cancel: &impl CancelSignal,
) -> Result<Option<RankedFrontMove>, String> {
    if cancel.is_cancelled() {
        return Err("stock-front steering cancelled".into());
    }
    if proposals.len() > MAX_FRONT_PROPOSALS {
        return Err("stock-front candidate buffer work limit; coverage unproven".into());
    }
    proposals.sort_by(|a, b| {
        b.tier
            .cmp(&a.tier)
            .then_with(|| b.score.total_cmp(&a.score))
    });
    let mut selected = None;
    for proposal in proposals {
        if cancel.is_cancelled() {
            return Err("stock-front steering cancelled".into());
        }
        // One continuous tree shares the angular unions for total contact and
        // the instantaneous forbidden half. Separate queries rebuild the same
        // history geometry. Neither an uncertain side nor a 360deg contact
        // allowance may admit a candidate without both whole-motion proofs.
        let checked =
            scene.check_front(&proposal.motion, radius, max_angle, right, options, cancel)?;
        if checked.status != EngagementStatus::Bounded {
            report.joint_rejections += 1;
            continue;
        }
        selected = Some((
            proposal.score,
            proposal.motion,
            checked.upper_angle_deg,
            proposal.novelty,
            proposal.tier,
        ));
        break;
    }
    if cancel.is_cancelled() {
        return Err("stock-front steering cancelled".into());
    }
    if let Some(error) = rank_error {
        if selected.is_none() {
            return Err(error);
        }
        report.steering_budget_stops += 1;
    }
    Ok(selected)
}

/// Bounded search, not an assumption that contact is monotone in curvature.
/// Every returned candidate has its OWN source and joint whole-motion proof.
#[allow(clippy::too_many_arguments)]
fn refine_long_advance(
    scene: &EngagementScene,
    stock: &Stock,
    position: Vec3,
    heading: f64,
    radius: f64,
    minimum_radius: f64,
    max_angle: f64,
    right: bool,
    prototype: &Motion,
    tolerance: f64,
    goal: Option<Point>,
    goal_pad: f64,
    work: &mut u64,
    mut source_score: impl FnMut(&Curve) -> Option<f64>,
    cancel: &impl CancelSignal,
) -> Result<RefinedAdvance, String> {
    #[cfg(feature = "profile-planning")]
    let _profile = crate::planning_profile::span("front.refine");
    let extent = (PI / (8. * radius)).min(minimum_radius.recip());
    let step = extent / 4.;
    let mut best: Option<RankedFrontMove> = None;
    let mut best_curvature = 0.;
    let mut tested: Vec<f64> = Vec::with_capacity(19);
    let mut budget_stopped = false;
    // Nine initial probes, then at most ten local midpoint probes. Neither
    // an unsafe midpoint nor a failed source check rules out its neighbours.
    for round in 0..=5 {
        let anchor = best_curvature;
        let probes = if round == 0 {
            (-4..=4).map(|i| i as f64 * step).collect::<Vec<_>>()
        } else {
            let delta = step / 2f64.powi(round);
            vec![anchor - delta, anchor + delta]
        };
        for k in probes {
            if cancel.is_cancelled() {
                return Err("stock-front curvature search cancelled".into());
            }
            if k.abs() > extent || tested.iter().any(|old| (*old - k).abs() * radius < 1e-12) {
                continue;
            }
            if *work == 0 {
                budget_stopped = true;
                break;
            }
            *work -= 1;
            tested.push(k);
            let curve = candidate(position, heading, k, 4. * radius);
            let Some(preference) = source_score(&curve) else {
                continue;
            };
            let motion = Motion {
                curve,
                ..prototype.clone()
            };
            let ranked = potential_score(
                scene, stock, &motion, radius, right, tolerance, work, goal, goal_pad, cancel,
            );
            let (score, novelty, tier) = match ranked {
                Ok(rank) => rank,
                Err(error) if *work == 0 && !error.contains("cancelled") => {
                    budget_stopped = true;
                    break;
                }
                Err(error) => return Err(error),
            };
            let score = score * preference;
            if tier < 2
                || best.as_ref().is_some_and(|(old, _, _, _, old_tier)| {
                    tier < *old_tier || (tier == *old_tier && score <= *old)
                })
            {
                continue;
            }
            if *work == 0 {
                budget_stopped = true;
                break;
            }
            let checked = scene.check_front(
                &motion,
                radius,
                max_angle,
                right,
                EngagementOptions {
                    tolerance_mm: tolerance,
                    resolution_mm: (32. * tolerance).max(0.001),
                    max_nodes: 8192,
                    max_work: *work,
                    ..Default::default()
                },
                cancel,
            )?;
            *work = work.saturating_sub(checked.support_tests + checked.index_visits);
            if checked.status == EngagementStatus::Bounded {
                best = Some((score, motion, checked.upper_angle_deg, novelty, tier));
                best_curvature = k;
            }
        }
        if budget_stopped || best.is_none() {
            break;
        }
    }
    Ok(RefinedAdvance {
        best,
        budget_stopped,
    })
}

fn goal_curvature(p: Vec3, heading: f64, target: Point, tolerance: f64) -> Option<f64> {
    let d = [target[0] - p[0], target[1] - p[1]];
    let length = d[0].hypot(d[1]);
    if length <= tolerance || !length.is_finite() {
        return None;
    }
    let (sin, cos) = heading.sin_cos();
    Some(2. * (-sin * (d[0] / length) + cos * (d[1] / length)) / length)
}

/// Optional optimization. Mutate the queue only after a whole-footprint proof;
/// uncertainty, work exhaustion and cancellation retain the original approach.
#[allow(clippy::too_many_arguments)]
fn clear_reentry_link(
    scene: &EngagementScene,
    source: &PocketPlan,
    stock: &Stock,
    position: Vec3,
    radius: f64,
    tolerance: f64,
    prototype: &Motion,
    queue: &mut VecDeque<Transfer>,
    work: &mut u64,
    cancel: &impl CancelSignal,
) -> Result<bool, String> {
    let Some(plunge_index) = queue.iter().position(|t| t.kind == MoveKind::Plunge) else {
        return Ok(false);
    };
    let plunge = &queue[plunge_index];
    let end = plunge.curve.end();
    if position[2] != end[2] || position == end {
        return Ok(false);
    }
    let Some(feed) = prototype.feed_mm_min else {
        return Ok(false);
    };
    let length = (end[0] - position[0]).hypot(end[1] - position[1]);
    let fed_approach: f64 = queue
        .iter()
        .take(plunge_index + 1)
        .filter_map(|t| {
            t.feed
                .map(|f| (t.curve.end()[2] - t.curve.start()[2]).abs() / f)
        })
        .sum();
    if length / feed >= fed_approach {
        return Ok(false);
    }
    let curve = Curve::Line {
        start: position,
        end,
    };
    if !source.guide_tube_fits(stroke_from_curve(&curve), radius) {
        return Ok(false);
    }
    let motion = Motion {
        curve,
        kind: MoveKind::Link,
        ..prototype.clone()
    };
    let mut stock_work = *work as usize;
    let potential =
        stock.may_remove_material(&motion, radius, 0., tolerance, &mut stock_work, cancel)?;
    *work = stock_work as u64;
    if potential {
        let guard = crate::verification::path_error(&motion, tolerance)
            + 512. * f64::EPSILON * (1. + radius);
        if !scene.clear_line(
            [position[0], position[1]],
            [end[0], end[1]],
            radius + guard,
            position[2],
            tolerance,
            work,
            cancel,
        )? {
            return Ok(false);
        }
    }
    if cancel.is_cancelled() {
        return Err("stock-front clear link cancelled".into());
    }
    let heading = plunge.heading;
    queue.drain(..=plunge_index);
    queue.push_front(Transfer {
        curve: motion.curve,
        kind: MoveKind::Link,
        feed: Some(feed),
        heading,
    });
    Ok(true)
}

fn candidate(p: Vec3, heading: f64, curvature: f64, length: f64) -> Curve {
    if curvature == 0. {
        return Curve::Line {
            start: p,
            end: [
                p[0] + length * heading.cos(),
                p[1] + length * heading.sin(),
                p[2],
            ],
        };
    }
    let c = [
        p[0] - heading.sin() / curvature,
        p[1] + heading.cos() / curvature,
        p[2],
    ];
    let a = (p[1] - c[1]).atan2(p[0] - c[0]);
    let sweep = length * curvature;
    let r = curvature.recip().abs();
    Curve::Arc {
        start: p,
        end: [
            c[0] + r * (a + sweep).cos(),
            c[1] + r * (a + sweep).sin(),
            p[2],
        ],
        center: c,
        plane: Plane::Xy,
        sweep_rad: sweep,
    }
}
fn stroke_from_curve(curve: &Curve) -> Stroke {
    let (a, b) = (curve.start(), curve.end());
    Stroke {
        a: [a[0], a[1]],
        b: [b[0], b[1]],
        arc: match *curve {
            Curve::Arc {
                center, sweep_rad, ..
            } => Some(([center[0], center[1]], sweep_rad)),
            _ => None,
        },
    }
}
fn tangent(curve: &Curve, f: f64) -> f64 {
    match *curve {
        Curve::Line { start, end } => (end[1] - start[1]).atan2(end[0] - start[0]),
        Curve::Arc {
            start,
            center,
            sweep_rad,
            ..
        } => {
            (start[1] - center[1]).atan2(start[0] - center[0])
                + f * sweep_rad
                + sweep_rad.signum() * PI / 2.
        }
        _ => unreachable!(),
    }
}
#[allow(clippy::too_many_arguments)]
fn potential_score(
    scene: &EngagementScene,
    stock: &Stock,
    motion: &Motion,
    r: f64,
    right: bool,
    tolerance: f64,
    remaining_work: &mut u64,
    goal: Option<Point>,
    goal_pad: f64,
    cancel: &impl CancelSignal,
) -> Result<(f64, bool, u8), String> {
    #[cfg(feature = "profile-planning")]
    let _profile = crate::planning_profile::span("front.area_rank");
    let path = motion.curve.prepare(tolerance)?;
    let side = if right { -1. } else { 1. };
    let mut potential = 0.;
    let mut count: f64 = 0.;
    let spec = stock.spec();
    let [nx, ny] = stock.dimensions();
    let step = stock.cell_widths_mm();
    // Sample each travel interval at its midpoint. Right endpoints miss all
    // removal when a corner clip finishes within the first quarter of a move.
    for f in [0.125, 0.375, 0.625, 0.875] {
        let p = path.position(f)?;
        let heading = tangent(&motion.curve, f);
        for ray in 0..8 {
            let a = (ray as f64 + 0.5) / 16.;
            let weight = (PI * a).cos();
            // Air and already-cleared columns are zero contact, not absent
            // observations. Conditioning the denominator on potential stock
            // inflated air-heavy traverses to a fully productive score.
            count += weight;
            let angle = heading + side * PI * a;
            let interior = r - 4. * tolerance;
            let q: Point = [p[0] + interior * angle.cos(), p[1] + interior * angle.sin()];
            // Transport of the cutter disk adds area only on its FORWARD
            // boundary: ds * max(feed_direction dot outward_normal,0).
            // Suppress boundary already covered by this candidate's prefix,
            // especially the inner side of a tight turn. Ranking only.
            let boundary = [p[0] + r * angle.cos(), p[1] + r * angle.sin()];
            if crate::verification::circle_axis_distance(motion, &path, [0., f], boundary)
                .is_some_and(|d| d < r - 8. * tolerance)
            {
                continue;
            }
            if (0..2).any(|k| q[k] < spec.min_mm[k] || q[k] >= spec.max_mm[k]) {
                continue;
            }
            let x = ((q[0] - spec.min_mm[0]) / step[0]) as usize;
            let y = ((q[1] - spec.min_mm[1]) / step[1]) as usize;
            if x >= nx || y >= ny || stock.cells()[y * nx + x].upper_mm <= p[2] {
                continue;
            }
            let near = scene
                .point_near_cleared_sweep(
                    q,
                    p[2],
                    (0.002 * r).max(32. * tolerance),
                    remaining_work,
                    cancel,
                )?
                .ok_or("stock-front ranking work limit; remaining coverage unproven")?;
            if !near {
                potential += weight;
            }
        }
    }
    // A ranking heuristic, not evidence of positive material or completion.
    let sampled = path.length_mm() * r * potential / count.max(1.);
    let mut hits_goal = false;
    let directed = if let Some(goal) = goal {
        let start = motion.curve.start();
        let old = (start[0] - goal[0]).hypot(start[1] - goal[1]);
        let distance = crate::verification::circle_axis_distance(motion, &path, [0., 1.], goal)
            .unwrap_or(f64::INFINITY);
        let end = motion.curve.end();
        let gain = (old - (end[0] - goal[0]).hypot(end[1] - goal[1])).max(0.);
        let cover = ((r - 2. * goal_pad) - distance).clamp(0., goal_pad) / goal_pad;
        hits_goal = cover > 0.;
        gain + cover * r
    } else {
        0.
    };
    Ok((
        if potential > 0. {
            sampled + 0.05 * r * directed
        } else {
            r * directed
        },
        potential > 0.,
        if potential > 0. {
            3
        } else if hits_goal {
            2
        } else if directed > 0. {
            1
        } else {
            0
        },
    ))
}

#[allow(clippy::too_many_arguments)]
fn goal_score(
    scene: &EngagementScene,
    stock: &Stock,
    motion: &Motion,
    r: f64,
    right: bool,
    tolerance: f64,
    remaining_work: &mut u64,
    goal: Option<Point>,
    goal_pad: f64,
    cancel: &impl CancelSignal,
) -> Result<(f64, bool, u8), String> {
    #[cfg(feature = "profile-planning")]
    let _profile = crate::planning_profile::span("front.goal_rank");
    let path = motion.curve.prepare(tolerance)?;
    let side = if right { -1. } else { 1. };
    let mut potential = 0.;
    let mut count: f64 = 0.;
    let spec = stock.spec();
    let [nx, ny] = stock.dimensions();
    let step = stock.cell_widths_mm();
    for f in [0.25, 0.5, 0.75, 1.] {
        let p = path.position(f)?;
        let heading = tangent(&motion.curve, f);
        for a in [0.25, 0.5, 0.75] {
            let angle = heading + side * PI * a;
            let interior = r - 4. * tolerance;
            let q: Point = [p[0] + interior * angle.cos(), p[1] + interior * angle.sin()];
            if (0..2).any(|k| q[k] < spec.min_mm[k] || q[k] >= spec.max_mm[k]) {
                continue;
            }
            let x = ((q[0] - spec.min_mm[0]) / step[0]) as usize;
            let y = ((q[1] - spec.min_mm[1]) / step[1]) as usize;
            if x >= nx || y >= ny || stock.cells()[y * nx + x].upper_mm <= p[2] {
                continue;
            }
            count += 1.;
            let near = scene
                .point_near_cleared_sweep(
                    q,
                    p[2],
                    (0.002 * r).max(32. * tolerance),
                    remaining_work,
                    cancel,
                )?
                .ok_or("stock-front ranking work limit; remaining coverage unproven")?;
            if !near {
                potential += goal.map_or(1., |goal| {
                    1. / (1. + (q[0] - goal[0]).hypot(q[1] - goal[1]) / r)
                });
            }
        }
    }
    // A ranking heuristic, not evidence of positive material or completion.
    let sampled = path.length_mm() * potential / count.max(1.);
    let mut hits_goal = false;
    let directed = if let Some(goal) = goal {
        let start = motion.curve.start();
        let old = (start[0] - goal[0]).hypot(start[1] - goal[1]);
        let distance = crate::verification::circle_axis_distance(motion, &path, [0., 1.], goal)
            .unwrap_or(f64::INFINITY);
        let end = motion.curve.end();
        let gain = (old - (end[0] - goal[0]).hypot(end[1] - goal[1])).max(0.);
        let cover = ((r - 2. * goal_pad) - distance).clamp(0., goal_pad) / goal_pad;
        hits_goal = cover > 0.;
        gain + cover * r
    } else {
        0.
    };
    Ok((
        sampled + directed,
        potential > 0.,
        if potential > 0. || hits_goal {
            2
        } else if directed > 0. {
            1
        } else {
            0
        },
    ))
}

/// Retract and feed heights for relinks (the operation's heights).
#[derive(Clone, Copy, Debug)]
pub(crate) struct LinkHeights {
    pub retract: f64,
    pub feed: f64,
}
impl From<f64> for LinkHeights {
    /// Link at one height (clearance).
    fn from(z: f64) -> Self {
        Self {
            retract: z,
            feed: z,
        }
    }
}
/// Leave the cut at feed up to the feed height, rapid up to the retract
/// height, across, and down to the feed height; plunge to `target` at feed.
/// With all heights equal (the default) this is: feed up, rapid across,
/// plunge down.
fn push_relink(
    queue: &mut VecDeque<Transfer>,
    position: Vec3,
    target: Vec3,
    heights: LinkHeights,
    retract_feed: Option<f64>,
    plunge_feed: f64,
    heading: Option<f64>,
) {
    let travel = heights.retract.max(position[2]);
    let lifted = [position[0], position[1], heights.feed.max(position[2])];
    let mut steps: Vec<(Vec3, MoveKind, Option<f64>, Option<f64>)> = vec![
        (lifted, MoveKind::Retract, retract_feed, None),
        (
            [position[0], position[1], travel],
            MoveKind::Rapid,
            None,
            None,
        ),
        ([target[0], target[1], travel], MoveKind::Rapid, None, None),
    ];
    if heights.feed < travel && heights.feed > target[2] {
        steps.push((
            [target[0], target[1], heights.feed],
            MoveKind::Rapid,
            None,
            None,
        ));
    }
    steps.push((target, MoveKind::Plunge, Some(plunge_feed), heading));
    let mut at = position;
    for (end, kind, feed, heading) in steps {
        if end != at {
            queue.push_back(Transfer {
                curve: Curve::Line { start: at, end },
                kind,
                feed,
                heading,
            });
            at = end;
        }
    }
}
struct Transfer {
    curve: Curve,
    kind: MoveKind,
    feed: Option<f64>,
    heading: Option<f64>,
}
struct FrontCoverage {
    domain: PreparedRegion,
    material: Option<ConvexMaterial>,
    // Bound to the same immutable centre/domain construction and synchronized
    // native history as this owner. Never expose it as general empty stock.
    inflation: f64,
    enclosed_complete: bool,
    // Coverage only: a tile may contain protected material outside the domain.
    // Never expose this as empty stock or use it for motion/descent admission.
    // The owning driver binds all checks to one component, floor, immutable
    // operation and monotone native sweep history; foreign/undo states fail
    // synchronize before these proofs can be reused.
    covered_tiles: HashSet<u64>,
    clear_hint: Option<usize>,
    split_tiles: HashSet<u64>,
    centres: PreparedRegion,
    goal_pad: f64,
    seed_orbit: f64,
    bounds: Bounds,
    seed: Point,
    key: (usize, usize),
    floor: f64,
}
enum CoverageResult {
    Complete,
    Gap(Point),
}
impl FrontCoverage {
    #[allow(clippy::too_many_arguments)]
    fn new(
        domain: PreparedRegion,
        seed: Point,
        key: (usize, usize),
        floor: f64,
        tolerance: f64,
        boundary_budget: f64,
        seed_orbit: f64,
        centres: PreparedRegion,
        inflation: f64,
    ) -> Result<Self, String> {
        let mut bounds: Option<Bounds> = None;
        for component in &domain.definition().components {
            for boundary in std::iter::once(&component.outer).chain(&component.holes) {
                for stroke in Stroke::from_boundary(boundary) {
                    let b = stroke.curve(floor).prepare(tolerance)?.bounds();
                    let b = Bounds {
                        min: [b.min[0], b.min[1]],
                        max: [b.max[0], b.max[1]],
                    };
                    bounds = Some(bounds.map_or(b, |old| old.union(b)));
                }
            }
        }
        Ok(Self {
            domain,
            material: None,
            inflation,
            enclosed_complete: false,
            covered_tiles: HashSet::new(),
            clear_hint: None,
            split_tiles: HashSet::new(),
            centres,
            goal_pad: boundary_budget / 4.,
            seed_orbit,
            bounds: bounds.unwrap_or(Bounds {
                min: seed,
                max: seed,
            }),
            seed,
            key,
            floor,
        })
    }
    /// Positive material witness for the initial search, NEVER completion or
    /// clearance. At most ten deterministic cells are considered. A cell lower
    /// bound encloses its whole physical column from below; strict domain
    /// clearance encloses a small neighborhood of its centre inside the requested
    /// region. Their intersection therefore contains unremoved material above
    /// this actual floor. No sweep/stock certificate is inferred from sampling.
    /// The immutable stock borrow and synchronized revision bind this one call;
    /// no witness is cached across native transactions. Uncertainty falls back.
    fn material_gap(
        &self,
        stock: &Stock,
        revision: (u64, u64),
        position: Vec3,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Point>, String> {
        if cancel.is_cancelled() {
            return Err("stock-front material witness cancelled".into());
        }
        if revision != (stock.identity(), stock.generation())
            || position[2] != self.floor
            || self.domain.definition().components.is_empty()
        {
            return Ok(None);
        }
        let spec = stock.spec();
        let [nx, ny] = stock.dimensions();
        let step = stock.cell_widths_mm();
        let low: Point = std::array::from_fn(|k| self.bounds.min[k].max(spec.min_mm[k]));
        let high: Point = std::array::from_fn(|k| self.bounds.max[k].min(spec.max_mm[k]));
        if (0..2).any(|k| low[k] >= high[k]) {
            return Ok(None);
        }
        let index = |p: Point| {
            let x = ((p[0] - spec.min_mm[0]) / step[0])
                .floor()
                .clamp(0., (nx - 1) as f64) as usize;
            let y = ((p[1] - spec.min_mm[1]) / step[1])
                .floor()
                .clamp(0., (ny - 1) as f64) as usize;
            y * nx + x
        };
        let mut candidates = vec![index([position[0], position[1]])];
        for x in [0.125, 0.5, 0.875] {
            for y in [0.125, 0.5, 0.875] {
                let i = index([
                    low[0] + x * (high[0] - low[0]),
                    low[1] + y * (high[1] - low[1]),
                ]);
                if !candidates.contains(&i) {
                    candidates.push(i);
                }
            }
        }
        candidates.sort_by(|a, b| {
            let distance = |i| {
                let p = stock.cell_center(i).unwrap();
                (p[0] - position[0]).hypot(p[1] - position[1])
            };
            distance(*a).total_cmp(&distance(*b))
        });
        let probe = step[0].min(step[1]) / 8.;
        for index in candidates {
            if cancel.is_cancelled() {
                return Err("stock-front material witness cancelled".into());
            }
            let p = stock.cell_center(index).unwrap();
            let guard = 64. * tolerance
                + 4096.
                    * f64::EPSILON
                    * (1.
                        + p[0].abs()
                        + p[1].abs()
                        + self.floor.abs()
                        + spec.max_mm[2].abs()
                        + probe);
            if probe <= guard || stock.cells()[index].lower_mm <= self.floor + guard {
                continue;
            }
            let q = self.domain.query(p)?;
            if q.signed_clearance_mm
                .is_some_and(|d| d > q.uncertainty_mm + probe + guard)
            {
                return Ok(Some(p));
            }
        }
        Ok(None)
    }
    /// A geometric search position, NEVER a clearance certificate. Project
    /// to the real line/arc centre boundary rather than quantizing to stock cells.
    fn centre_near(&self, p: Point, tolerance: f64) -> Result<Point, String> {
        let q = self.centres.query(p)?;
        if q.signed_clearance_mm.is_some_and(|d| d > q.uncertainty_mm) {
            return Ok(p);
        }
        let mut best = None;
        let mut distance = f64::INFINITY;
        for component in &self.centres.definition().components {
            for boundary in std::iter::once(&component.outer).chain(&component.holes) {
                for s in Stroke::from_boundary(boundary) {
                    let nearest = if let Some((c, sweep)) = s.arc {
                        let start = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
                        let angle = (p[1] - c[1]).atan2(p[0] - c[0]);
                        let travel = if sweep > 0. {
                            (angle - start).rem_euclid(TAU)
                        } else {
                            (start - angle).rem_euclid(TAU)
                        };
                        if travel <= sweep.abs() || sweep.abs() >= TAU {
                            let r = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
                            [c[0] + r * angle.cos(), c[1] + r * angle.sin()]
                        } else if (p[0] - s.a[0]).hypot(p[1] - s.a[1])
                            < (p[0] - s.b[0]).hypot(p[1] - s.b[1])
                        {
                            s.a
                        } else {
                            s.b
                        }
                    } else {
                        let d = [s.b[0] - s.a[0], s.b[1] - s.a[1]];
                        let l = d[0] * d[0] + d[1] * d[1];
                        let t = if l == 0. {
                            0.
                        } else {
                            ((p[0] - s.a[0]) * d[0] + (p[1] - s.a[1]) * d[1]) / l
                        }
                        .clamp(0., 1.);
                        [s.a[0] + t * d[0], s.a[1] + t * d[1]]
                    };
                    let d = (p[0] - nearest[0]).hypot(p[1] - nearest[1]);
                    if d < distance {
                        distance = d;
                        best = Some(nearest);
                    }
                }
            }
        }
        let mut c = best.ok_or("stock-front centre component has no boundary")?;
        if distance > tolerance {
            let inset = 32. * tolerance + self.centres.definition().boundary_error_mm;
            for k in 0..2 {
                c[k] += (c[k] - p[k]) * inset / distance;
            }
        }
        Ok(c)
    }
    /// Morton path with a leading root bit. At depth<=28 it fits in u64.
    /// A parent is covered only when all four child tiles have proofs. Keep
    /// children too, so already queued nodes remain hits after parent folding.
    /// Saturation merely stops memoization; the original proof still runs.
    fn remember_covered(&mut self, mut tile: u64) {
        while self.covered_tiles.len() + self.split_tiles.len() < 262144 {
            self.covered_tiles.insert(tile);
            if tile == 1 {
                break;
            }
            let siblings = tile & !3;
            if !(0..4).all(|q| self.covered_tiles.contains(&(siblings | q))) {
                break;
            }
            tile >>= 2;
        }
    }
    #[allow(clippy::too_many_arguments)]
    fn check(
        &mut self,
        scene: &EngagementScene,
        stock: &Stock,
        position: Vec3,
        radius: f64,
        tolerance: f64,
        tiles: &mut usize,
        envelopes: &mut usize,
        cache_hits: &mut usize,
        reused_branches: &mut usize,
        cancel: &impl CancelSignal,
    ) -> Result<CoverageResult, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("front.coverage");
        if cancel.is_cancelled() {
            return Err("stock-front coverage cancelled".into());
        }
        if self.domain.definition().components.is_empty() {
            return Ok(CoverageResult::Complete);
        }
        if self.enclosed_complete {
            *envelopes += 1;
            return Ok(CoverageResult::Complete);
        }
        let mut work = 20_000_000u64;
        if let Some(material) = &self.material
            && material.excludes_domain(&self.domain, self.floor, tolerance, &mut work, cancel)?
        {
            *envelopes += 1;
            return Ok(CoverageResult::Complete);
        }
        let mut nodes = 0usize;
        let mut stack = VecDeque::from([(self.bounds, 0u8, 1u64)]);
        let segments = self.domain.summary().segments as u64;
        while let Some((b, depth, tile)) = stack.pop_front() {
            if cancel.is_cancelled() {
                return Err("stock-front coverage cancelled".into());
            }
            nodes += 1;
            *tiles += 1;
            if nodes > 262144 {
                return Err("stock-front coverage work limit; coverage unproven".into());
            }
            if self.covered_tiles.contains(&tile) {
                *cache_hits += 1;
                continue;
            }
            let p: Point = std::array::from_fn(|k| (b.min[k] + b.max[k]) / 2.);
            let half = (b.max[0] - b.min[0]).hypot(b.max[1] - b.min[1]) / 2.;
            // A previously split node cannot acquire a new gap: its geometry
            // is fixed, and any proved-clear goal box stays clear as native
            // sweeps remove material. Only its children need new proofs.
            // Still check the root to recognize stock that became wholly clear.
            if tile != 1 && self.split_tiles.contains(&tile) {
                *reused_branches += 1;
            } else {
                if work < segments + 1 {
                    return Err("stock-front coverage work limit; coverage unproven".into());
                }
                work -= segments + 1;
                let q = {
                    #[cfg(feature = "profile-planning")]
                    let _profile = crate::planning_profile::span("coverage.domain_query");
                    self.domain.query(p)?
                };
                let guard = 1024. * f64::EPSILON * (1. + p[0].abs() + p[1].abs() + half + radius);
                if q.signed_clearance_mm
                    .is_some_and(|d| d + half + q.uncertainty_mm + guard < 0.)
                {
                    self.remember_covered(tile);
                    continue;
                }
                if let Some(material) = &self.material
                    && material.excludes_box(
                        [b.min, b.max],
                        self.floor,
                        tolerance,
                        &mut work,
                        cancel,
                    )?
                {
                    self.remember_covered(tile);
                    continue;
                }
                if scene.clear_box_with_hint(
                    [b.min, b.max],
                    self.floor,
                    &mut self.clear_hint,
                    &mut work,
                    cancel,
                )? || stock.bounds_clear([b.min, b.max], self.floor, &mut work, cancel)?
                {
                    self.remember_covered(tile);
                    continue;
                }
                // This point is only a steering counterexample candidate. It does
                // not certify material/violation; completion still needs ALL tiles.
                if half <= radius / 2. {
                    let pad = self.goal_pad;
                    let goal_bounds = [p.map(|v| v - pad), p.map(|v| v + pad)];
                    let goal_clear =
                        scene.clear_box_with_hint(
                            goal_bounds,
                            self.floor,
                            &mut self.clear_hint,
                            &mut work,
                            cancel,
                        )? || stock.bounds_clear(goal_bounds, self.floor, &mut work, cancel)?;
                    if goal_clear
                        && (0..2)
                            .all(|k| b.min[k] >= goal_bounds[0][k] && b.max[k] <= goal_bounds[1][k])
                    {
                        self.remember_covered(tile);
                        continue;
                    }
                    if !goal_clear
                        && (q
                            .signed_clearance_mm
                            .is_some_and(|d| d > q.uncertainty_mm + guard)
                            || depth >= 28
                            || half <= (16. * tolerance).min(pad / 2.))
                    {
                        return Ok(CoverageResult::Gap(p));
                    }
                }
            }
            if depth >= 28 {
                return Err(
                    "stock-front coverage subdivision unresolved; coverage unproven".into(),
                );
            }
            // At most65535 retained branch nodes describe262141 total tree
            // nodes, within the original per-check visit bound. Cache pressure
            // falls back to the unchanged proof rather than dropping a region.
            if self.split_tiles.len() < 65535
                && self.covered_tiles.len() + self.split_tiles.len() < 262144
            {
                self.split_tiles.insert(tile);
            }
            let mut children = Vec::with_capacity(4);
            for quadrant in 0..4 {
                let bounds = Bounds {
                    min: std::array::from_fn(|k| {
                        if quadrant & (1 << k) == 0 {
                            b.min[k]
                        } else {
                            p[k]
                        }
                    }),
                    max: std::array::from_fn(|k| {
                        if quadrant & (1 << k) == 0 {
                            p[k]
                        } else {
                            b.max[k]
                        }
                    }),
                };
                let mid: Point = std::array::from_fn(|k| (bounds.min[k] + bounds.max[k]) / 2.);
                let distance = (mid[0] - position[0]).hypot(mid[1] - position[1]);
                children.push((distance, bounds, (tile << 2) | quadrant));
            }
            children.sort_by(|a, b| a.0.total_cmp(&b.0));
            for (_, child, key) in children {
                stack.push_back((child, depth + 1, key));
            }
        }
        Ok(CoverageResult::Complete)
    }
}

type ReentryKey = (u64, u64, u8, usize);
#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn plan_reentry(
    scene: &EngagementScene,
    source: &PocketPlan,
    stock: &Stock,
    seed: Point,
    seed_orbit: f64,
    minimum_radius: f64,
    max_angle: f64,
    goal_pad: f64,
    projected_centre: Point,
    position: Vec3,
    goal: Point,
    radius: f64,
    tolerance: f64,
    right: bool,
    progress: usize,
    visited: &HashSet<ReentryKey>,
    segment_entry: bool,
    work_limit: u64,
    prototype: &Motion,
    clearance: LinkHeights,
    plunge_feed: f64,
    cancel: &impl CancelSignal,
) -> Result<Option<(ReentryKey, VecDeque<Transfer>, bool)>, String> {
    #[cfg(feature = "profile-planning")]
    let _profile = crate::planning_profile::span("front.reentry");
    let [nx, ny] = stock.dimensions();
    let stride = nx.max(ny).div_ceil(256).max(1);
    let mut polar = vec![seed];
    for scale in [0.25, 0.5, 0.75, 0.9] {
        for i in 0..16 {
            let a = TAU * i as f64 / 16.;
            polar.push([
                seed[0] + seed_orbit * scale * a.cos(),
                seed[1] + seed_orbit * scale * a.sin(),
            ]);
        }
    }
    let mut candidates: Vec<_> = polar
        .into_iter()
        .chain((0..ny).step_by(stride).flat_map(|y| {
            (0..nx)
                .step_by(stride)
                .filter_map(move |x| stock.cell_center(y * nx + x))
        }))
        .map(|p| (p, None))
        .collect();
    let mut recovery_centres = vec![projected_centre];
    // Original outside-stock space is available along EVERY face, not just
    // near the one entry seed. The forbidden half of a turning cutter reaches
    // hypot(tool_radius, axis_radius) from its circle centre. Use that as a
    // search stand-off; full source/clearance/contact/side checks below remain
    // the authority. No future circle is emitted or marked cleared here.
    let spec = stock.spec();
    let stand_off = radius.hypot(minimum_radius) + 64. * tolerance;
    if segment_entry {
        for axis in 0..2 {
            for side in [0, 1] {
                let mut centre: Point =
                    std::array::from_fn(|k| goal[k].clamp(spec.min_mm[k], spec.max_mm[k]));
                centre[axis] = if side == 0 {
                    spec.min_mm[axis] - stand_off
                } else {
                    spec.max_mm[axis] + stand_off
                };
                recovery_centres.push(centre);
            }
        }
    }
    for centre in recovery_centres {
        for i in 0..32 {
            let a = TAU * i as f64 / 32.;
            candidates.push((
                [
                    centre[0] + minimum_radius * a.cos(),
                    centre[1] + minimum_radius * a.sin(),
                ],
                Some(a + if right { PI / 2. } else { -PI / 2. }),
            ));
        }
    }
    let mut work = work_limit;
    if segment_entry {
        for inset in [128. * tolerance, 0.05 * minimum_radius] {
            candidates.extend(
                scene
                    .reentry_positions(goal, position[2], inset, right, &mut work, cancel)?
                    .into_iter()
                    .map(|(p, heading)| (p, Some(heading))),
            );
        }
    }
    // Nearest to the goal first; each distance computed once (stable sort on
    // the same keys: the same order as comparing recomputed distances).
    let mut keyed: Vec<(f64, (Point, Option<f64>))> = candidates
        .into_iter()
        .map(|c| ((c.0[0] - goal[0]).hypot(c.0[1] - goal[1]), c))
        .collect();
    keyed.sort_by(|a, b| a.0.total_cmp(&b.0));
    let candidates = keyed.into_iter().map(|(_, c)| c);
    // Error-only attribution, in the same existing shared work units:
    // source/history, stock, full disk, ranking, joint total/side (last reserved).
    let mut costs = [work_limit - work, 0, 0, 0, 0, 0];
    let mut best: Option<(f64, u8, Point, f64, ReentryKey, Option<Curve>)> = None;
    let mut truncated = false;
    let sign = if right { 1. } else { -1. };
    'search: for (mut p, hint) in candidates {
        let canonical = (p[0].to_bits(), p[1].to_bits());
        if (p[0] - position[0]).hypot(p[1] - position[1]) < 1e-9 {
            p = [position[0], position[1]];
        }
        if cancel.is_cancelled() {
            return Err("stock-front re-entry cancelled".into());
        }
        if work == 0 {
            if best.is_some() {
                truncated = true;
                break;
            }
            return Err("stock-front re-entry search work limit; coverage unproven".into());
        }
        work -= 1;
        costs[0] += 1;
        let distance = (p[0] - goal[0]).hypot(p[1] - goal[1]);
        if !segment_entry
            && best
                .as_ref()
                .is_some_and(|b| distance - minimum_radius >= -b.0)
        {
            break;
        }
        let source_work = source.link_work_per_query() as u64;
        if work < source_work {
            if best.is_some() {
                truncated = true;
                break;
            }
            return Err("stock-front re-entry source work limit; coverage unproven".into());
        }
        work -= source_work;
        costs[0] += source_work;
        let pad = 64. * tolerance;
        if !source.link_geometry_fits_radius(p, p, radius + pad) {
            continue;
        }
        let query = Motion {
            curve: Curve::Line {
                start: [p[0], p[1], position[2]],
                end: [p[0] + pad, p[1], position[2]],
            },
            ..prototype.clone()
        };
        let stock_before = work;
        let mut stock_work = work as usize;
        let potential =
            stock.may_remove_material(&query, radius, 0., tolerance, &mut stock_work, cancel)?;
        work = stock_work as u64;
        costs[1] += stock_before - work;
        let mut clearance_proven = !potential;
        if potential && !segment_entry {
            let before = work;
            let disk = scene.clear_disk(p, radius + pad, position[2], tolerance, &mut work, cancel);
            costs[2] += before - work;
            match disk {
                Ok(true) => clearance_proven = true,
                Ok(false) => continue,
                Err(e) if best.is_some() && work == 0 && !e.contains("cancelled") => {
                    truncated = true;
                    break 'search;
                }
                Err(e) => return Err(e),
            }
        }
        if potential && segment_entry {
            let before = work;
            let possible = scene.disk_may_be_clear(p, radius + pad, position[2], &mut work, cancel);
            costs[2] += before - work;
            if !possible? {
                continue;
            }
        }
        let base = (goal[1] - p[1]).atan2(goal[0] - p[0]) + sign * PI / 2.;
        let headings = hint
            .into_iter()
            .chain((0..16).map(|offset| base + offset as f64 * TAU / 16.));
        // A near-goal pose may have many inadmissible directions. Reserve
        // search work for other poses instead of spending the whole re-entry
        // budget proving one pose unhelpful. This limits proposals, not proof
        // accuracy: an unresolved candidate is never accepted.
        let pose_floor = work.saturating_sub(65_536);
        for heading in headings {
            if cancel.is_cancelled() {
                return Err("stock-front heading search cancelled".into());
            }
            let sector = (heading.rem_euclid(TAU) * 32. / TAU).floor() as u8;
            let key = (canonical.0, canonical.1, sector, progress);
            if visited.contains(&key) {
                continue;
            }
            if !segment_entry {
                let curve = candidate(
                    [p[0], p[1], position[2]],
                    heading,
                    sign / minimum_radius,
                    TAU * minimum_radius,
                );
                let Curve::Arc { center, .. } = curve else {
                    unreachable!()
                };
                let centre = [center[0], center[1]];
                let score = -(centre[0] - goal[0]).hypot(centre[1] - goal[1]);
                if best.as_ref().is_some_and(|b| score <= b.0) {
                    continue;
                }
                if work < source_work {
                    if best.is_some() {
                        truncated = true;
                        break 'search;
                    }
                    return Err("stock-front recovery heading work limit; coverage unproven".into());
                }
                work -= source_work;
                costs[0] += source_work;
                if !source.link_geometry_fits_radius(centre, centre, radius + minimum_radius) {
                    continue;
                }
                let lookahead = Motion {
                    curve,
                    ..prototype.clone()
                };
                let before = work;
                let rank = goal_score(
                    scene,
                    stock,
                    &lookahead,
                    radius,
                    right,
                    tolerance,
                    &mut work,
                    Some(goal),
                    goal_pad,
                    cancel,
                );
                costs[3] += before - work;
                let (_, _, tier) = match rank {
                    Ok(rank) => rank,
                    Err(e) if best.is_some() && work == 0 && !e.contains("cancelled") => {
                        truncated = true;
                        break 'search;
                    }
                    Err(e) => return Err(e),
                };
                if tier < 2 {
                    continue;
                }
                if work == 0 {
                    if best.is_some() {
                        truncated = true;
                        break 'search;
                    }
                    return Err("stock-front recovery proof work limit; coverage unproven".into());
                }
                let checked = scene.check_front(
                    &lookahead,
                    radius,
                    max_angle,
                    right,
                    EngagementOptions {
                        tolerance_mm: tolerance,
                        resolution_mm: (32. * tolerance).max(0.001),
                        max_nodes: 8192,
                        max_work: work,
                        ..Default::default()
                    },
                    cancel,
                )?;
                costs[4] += (checked.support_tests + checked.index_visits).min(work);
                work = work.saturating_sub(checked.support_tests + checked.index_visits);
                if checked.status == EngagementStatus::Bounded {
                    best = Some((score, tier, p, heading, key, None));
                }
                continue;
            }
            // Search the actual forward segment, not an entire hypothetical
            // recovery orbit. The latter can be empty/useless even when a
            // gentle outward advance from this same clear pose is productive.
            for curve in reentry_leads(
                [p[0], p[1], position[2]],
                heading,
                radius,
                minimum_radius,
                right,
            ) {
                if work.saturating_sub(pose_floor) <= source_work {
                    continue 'search;
                }
                if cancel.is_cancelled() {
                    return Err("stock-front lead search cancelled".into());
                }
                if work < source_work {
                    if best.is_some() {
                        truncated = true;
                        break 'search;
                    }
                    return Err(format!(
                        "stock-front heading work limit; coverage unproven; goal={goal:?}, candidate={p:?}, source/stock/disk/rank/jointFront/reserved={costs:?}"
                    ));
                }
                work -= source_work;
                costs[0] += source_work;
                if !source.guide_tube_fits(stroke_from_curve(&curve), radius) {
                    continue;
                }
                let lookahead = Motion {
                    curve,
                    ..prototype.clone()
                };
                let before = work;
                let ranked = goal_score(
                    scene,
                    stock,
                    &lookahead,
                    radius,
                    right,
                    tolerance,
                    &mut work,
                    Some(goal),
                    goal_pad,
                    cancel,
                );
                costs[3] += before - work;
                let (score, _, tier) = match ranked {
                    Ok(rank) => rank,
                    Err(e) if best.is_some() && work == 0 && !e.contains("cancelled") => {
                        truncated = true;
                        break 'search;
                    }
                    Err(e) => {
                        return Err(format!(
                            "{e}; re-entry goal={goal:?}, candidate={p:?}, heading={heading}, source/stock/disk/rank/jointFront/reserved={costs:?}"
                        ));
                    }
                };
                if tier < 2
                    || best.as_ref().is_some_and(|(old, old_tier, ..)| {
                        tier < *old_tier || (tier == *old_tier && score <= *old)
                    })
                {
                    continue;
                }
                if work == 0 {
                    if best.is_some() {
                        truncated = true;
                        break 'search;
                    }
                    return Err("stock-front lead work limit; coverage unproven".into());
                }
                let options = EngagementOptions {
                    tolerance_mm: tolerance,
                    resolution_mm: (32. * tolerance).max(0.001),
                    max_nodes: 8192,
                    max_work: work.saturating_sub(pose_floor).max(1),
                    ..Default::default()
                };
                let checked =
                    scene.check_front(&lookahead, radius, max_angle, right, options, cancel)?;
                costs[4] += (checked.support_tests + checked.index_visits).min(work);
                work = work.saturating_sub(checked.support_tests + checked.index_visits);
                if checked.status != EngagementStatus::Bounded {
                    continue;
                }
                if !clearance_proven {
                    // First find a useful, fully checked forward move. An
                    // expensive whole-disk union proof for a pose with no
                    // usable exit cannot contribute a re-entry candidate.
                    // Descent is still forbidden until this proof succeeds.
                    let before = work;
                    let disk = scene.clear_disk(
                        p,
                        radius + pad,
                        position[2],
                        tolerance,
                        &mut work,
                        cancel,
                    );
                    costs[2] += before - work;
                    match disk {
                        Ok(true) => clearance_proven = true,
                        Ok(false) => continue 'search,
                        Err(e) if best.is_some() && work == 0 && !e.contains("cancelled") => {
                            truncated = true;
                            break 'search;
                        }
                        Err(e) => return Err(e),
                    }
                }
                // Positions are sorted by distance to the remaining goal.
                // Compare useful advances from this pose before choosing: the
                // first admissible tiny arc must not hide a longer useful one.
                best = Some((score, tier, p, heading, key, Some(lookahead.curve)));
            }
        }
        if segment_entry && best.is_some() {
            break;
        }
    }
    let Some((_, _, p, heading, key, lead)) = best else {
        return Ok(None);
    };
    let down = [p[0], p[1], position[2]];
    let mut queue = VecDeque::new();
    push_relink(
        &mut queue,
        position,
        down,
        clearance,
        prototype.feed_mm_min,
        plunge_feed,
        Some(heading),
    );
    if let Some(lead) = lead {
        queue.push_back(Transfer {
            curve: lead,
            kind: MoveKind::Lead,
            feed: prototype.feed_mm_min,
            heading: None,
        });
    }
    Ok(Some((key, queue, truncated)))
}

/// Candidate geometry only. No candidate carries stock-clearance authority.
/// Short advances do not always reach material from a clear descent disk: the
/// first quarter of a recovery turn can be air. Include longer turns in the
/// requested sense without requiring an orbit for other useful advances.
/// The caller must prove source and continuous contact/side before queueing.
fn reentry_leads(
    position: Vec3,
    heading: f64,
    radius: f64,
    minimum_radius: f64,
    right: bool,
) -> Vec<Curve> {
    let sign = if right { 1. } else { -1. };
    let mut curvatures = vec![
        0.,
        -sign / (4. * radius),
        sign / (4. * radius),
        -sign / radius,
        sign / radius,
        -sign / minimum_radius,
        sign / minimum_radius,
    ];
    curvatures.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    let mut result = Vec::new();
    for sweep in [PI, TAU] {
        result.push(candidate(
            position,
            heading,
            sign / minimum_radius,
            sweep * minimum_radius,
        ));
    }
    for k in curvatures {
        let mut previous: Option<f64> = None;
        for scale in [4., 1., 0.25, 0.0625] {
            let length = (radius * scale).min(if k == 0. {
                f64::INFINITY
            } else {
                PI / (2. * k.abs())
            });
            if previous.is_some_and(|old| (old - length).abs() < 1e-9) {
                continue;
            }
            previous = Some(length);
            result.push(candidate(position, heading, k, length));
        }
    }
    result
}

#[cfg(test)]
mod ranking_tests {
    use super::*;
    use crate::stock::StockSpec;
    #[test]
    fn initial_material_witness_uses_strict_domain_and_current_stock_lower_bounds() {
        use crate::{
            planar::{Boundary, Component, ConstructionOptions, PlanarRegion},
            stock::HeightBounds,
        };
        let square = |a, b| Boundary {
            vertices: vec![[a, a, 0.], [b, a, 0.], [b, b, 0.], [a, b, 0.]],
        };
        let definition = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "coverage".into(),
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
            components: vec![Component {
                outer: square(2., 8.),
                holes: vec![square(4., 6.)],
            }],
        };
        let make = |definition: &PlanarRegion| {
            FrontCoverage::new(
                definition.prepare(ConstructionOptions::default()).unwrap(),
                [2.5, 2.5],
                (0, 0),
                5.,
                1e-5,
                0.01,
                0.5,
                definition.prepare(ConstructionOptions::default()).unwrap(),
                0.,
            )
            .unwrap()
        };
        let coverage = make(&definition);
        let stock = Stock::new(StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [10., 10., 10.],
            cell_size_mm: 0.25,
            max_cells: 10000,
        })
        .unwrap();
        let revision = (stock.identity(), stock.generation());
        let point = coverage
            .material_gap(&stock, revision, [2., 2., 5.], 1e-5, &|| false)
            .unwrap()
            .unwrap();
        // Independent rectangle/hole membership; no classifier result reused.
        assert!(point.iter().all(|v| *v > 2. && *v < 8.));
        assert!(point.iter().any(|v| *v < 4. || *v > 6.));
        assert_eq!(stock.cells()[0].lower_mm, 10.);
        assert!(
            coverage
                .material_gap(&stock, revision, [2., 2., 4.], 1e-5, &|| false)
                .unwrap()
                .is_none()
        );
        assert!(
            coverage
                .material_gap(
                    &stock,
                    (revision.0, revision.1 + 1),
                    [2., 2., 5.],
                    1e-5,
                    &|| false
                )
                .unwrap()
                .is_none()
        );
        assert!(
            coverage
                .material_gap(&stock, revision, [2., 2., 5.], 1e-5, &|| true)
                .is_err()
        );
        let foreign = Stock::from_snapshot(stock.snapshot()).unwrap();
        assert!(
            coverage
                .material_gap(&foreign, revision, [2., 2., 5.], 1e-5, &|| false)
                .unwrap()
                .is_none()
        );
        let mut snapshot = stock.snapshot();
        for (i, h) in snapshot.cells.iter_mut().enumerate() {
            let p = stock.cell_center(i).unwrap();
            if p.iter().all(|v| *v >= 1. && *v <= 9.) && p.iter().any(|v| *v < 4.7 || *v > 5.3) {
                *h = HeightBounds {
                    lower_mm: 5.,
                    upper_mm: 5.,
                };
            }
        }
        let cleared = Stock::from_snapshot(snapshot).unwrap();
        assert!(
            coverage
                .material_gap(
                    &cleared,
                    (cleared.identity(), cleared.generation()),
                    [2., 2., 5.],
                    1e-5,
                    &|| false
                )
                .unwrap()
                .is_none()
        );
        let scene = EngagementScene::new(cleared.spec().clone(), vec![]).unwrap();
        let mut oracle = make(&definition);
        let (mut tiles, mut envelopes, mut hits, mut branches) = (0, 0, 0, 0);
        assert!(matches!(
            oracle
                .check(
                    &scene,
                    &cleared,
                    [2., 2., 5.],
                    1.,
                    1e-5,
                    &mut tiles,
                    &mut envelopes,
                    &mut hits,
                    &mut branches,
                    &|| false
                )
                .unwrap(),
            CoverageResult::Complete
        ));
        // A partial job with definite remaining material still supplies a Gap.
        let mut partial = cleared.snapshot();
        let index = stock
            .cells()
            .iter()
            .enumerate()
            .find(|(i, _)| stock.cell_center(*i) == Some(point))
            .unwrap()
            .0;
        partial.cells[index] = HeightBounds {
            lower_mm: 10.,
            upper_mm: 10.,
        };
        let partial = Stock::from_snapshot(partial).unwrap();
        assert_eq!(
            coverage
                .material_gap(
                    &partial,
                    (partial.identity(), partial.generation()),
                    [2., 2., 5.],
                    1e-5,
                    &|| false
                )
                .unwrap(),
            Some(point)
        );
        let mut uncertain = stock.snapshot();
        for h in &mut uncertain.cells {
            h.lower_mm = 5. + 1e-5;
        }
        let uncertain = Stock::from_snapshot(uncertain).unwrap();
        assert!(
            coverage
                .material_gap(
                    &uncertain,
                    (uncertain.identity(), uncertain.generation()),
                    [2., 2., 5.],
                    1e-5,
                    &|| false
                )
                .unwrap()
                .is_none()
        );
        let mut uncertain_domain = definition;
        uncertain_domain.boundary_error_mm = 1.;
        assert!(
            make(&uncertain_domain)
                .material_gap(&stock, revision, [2., 2., 5.], 1e-5, &|| false)
                .unwrap()
                .is_none()
        );
    }
    #[test]
    fn clear_reentry_link_preserves_queue_on_uncertainty_work_limit_and_cancel() {
        let m: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json"))
                .unwrap();
        let crate::pocket::Operation::AdaptivePocket(op) = m.operations[0].clone() else {
            panic!("fixture")
        };
        let source = PocketPlan::prepare(
            op.pocket,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let prior = Motion {
            id: 1,
            operation: "prior".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            feed_mm_min: Some(600.),
            curve: Curve::Arc {
                start: [7.5, 5., 9.],
                end: [7.5, 5., 9.],
                center: [7., 5., 9.],
                plane: Plane::Xy,
                sweep_rad: TAU,
            },
        };
        let scene = EngagementScene::new(
            m.stock.height().unwrap().clone(),
            vec![ClearedSweep::from_motion(&prior, 4., 1e-5).unwrap()],
        )
        .unwrap();
        let empty = EngagementScene::new(m.stock.height().unwrap().clone(), vec![]).unwrap();
        let make_queue = || {
            VecDeque::from([
                Transfer {
                    curve: Curve::Line {
                        start: [5., 5., 9.],
                        end: [5., 5., 12.],
                    },
                    kind: MoveKind::Retract,
                    feed: Some(600.),
                    heading: None,
                },
                Transfer {
                    curve: Curve::Line {
                        start: [5., 5., 12.],
                        end: [9., 5., 12.],
                    },
                    kind: MoveKind::Rapid,
                    feed: None,
                    heading: None,
                },
                Transfer {
                    curve: Curve::Line {
                        start: [9., 5., 12.],
                        end: [9., 5., 9.],
                    },
                    kind: MoveKind::Plunge,
                    feed: Some(150.),
                    heading: Some(0.5),
                },
                Transfer {
                    curve: Curve::Line {
                        start: [9., 5., 9.],
                        end: [9.5, 5., 9.],
                    },
                    kind: MoveKind::Lead,
                    feed: Some(600.),
                    heading: None,
                },
            ])
        };
        for (known, mut budget, cancelled) in [
            (&empty, 100000, false),
            (&scene, 0, false),
            (&scene, 100000, true),
        ] {
            let mut queue = make_queue();
            let result = clear_reentry_link(
                known,
                &source,
                &stock,
                [5., 5., 9.],
                1.,
                1e-5,
                &prior,
                &mut queue,
                &mut budget,
                &|| cancelled,
            );
            assert!(!matches!(result, Ok(true)));
            if cancelled {
                assert!(result.unwrap_err().contains("cancelled"));
            }
            for (got, expected) in queue.iter().zip(make_queue()) {
                assert_eq!(
                    (&got.curve, got.kind, got.feed, got.heading),
                    (
                        &expected.curve,
                        expected.kind,
                        expected.feed,
                        expected.heading
                    )
                );
            }
            assert_eq!(queue.len(), 4);
        }
        let mut queue = make_queue();
        assert!(
            clear_reentry_link(
                &scene,
                &source,
                &stock,
                [5., 5., 9.],
                1.,
                1e-5,
                &prior,
                &mut queue,
                &mut 100000,
                &|| false
            )
            .unwrap()
        );
        assert_eq!(queue.len(), 2);
        assert_eq!(queue[0].kind, MoveKind::Link);
        assert_eq!(queue[0].curve.start(), [5., 5., 9.]);
        assert_eq!(queue[0].curve.end(), [9., 5., 9.]);
        assert_eq!(queue[0].heading, Some(0.5));
        assert_eq!(queue[1].kind, MoveKind::Lead);
    }
    #[test]
    fn cancelling_channel_entry_proof_retains_the_unstarted_rail() {
        use std::cell::Cell;
        let mut m: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json"))
                .unwrap();
        m.tools[0].cutter.diameter_mm = 2.;
        m.tools[0].cutter.shank_diameter_mm = 2.;
        let crate::pocket::Operation::AdaptivePocket(mut op) = m.operations.remove(0) else {
            panic!("fixture");
        };
        op.pocket.floor_z_mm = 9.;
        op.pocket.stepover_mm = 0.5;
        let source = PocketPlan::prepare(
            op.pocket,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let seed = [5., 5.];
        let (centres, domain) = source.front_component_region(seed, 1.49).unwrap();
        let prior = Motion {
            id: 1,
            operation: "prior".into(),
            tool: "large".into(),
            kind: MoveKind::Cut,
            feed_mm_min: Some(600.),
            curve: Curve::Arc {
                start: [5.5, 5., 9.],
                end: [5.5, 5., 9.],
                center: [5., 5., 9.],
                plane: Plane::Xy,
                sweep_rad: TAU,
            },
        };
        let mut driver = FrontDriver::new(&m.tools[0]);
        driver.scene = Some(
            EngagementScene::new(
                m.stock.height().unwrap().clone(),
                vec![ClearedSweep::from_motion(&prior, 3., 1e-5).unwrap()],
            )
            .unwrap(),
        );
        driver.coverage = Some(
            FrontCoverage::new(domain, seed, (0, 0), 9., 1e-5, 0.01, 0.5, centres, 1.49).unwrap(),
        );
        let rail = Stroke::line(seed, [9., 5.]);
        driver.linear_rails = Some(VecDeque::from([rail]));
        let polls = Cell::new(0);
        let cancel = || {
            polls.set(polls.get() + 1);
            polls.get() >= 3
        };
        let result = driver.prepare_linear(
            &source,
            [5., 5., 9.],
            1.,
            0.2,
            82.,
            true,
            &prior,
            1e-5,
            LinkHeights::from(12.),
            150.,
            &cancel,
        );
        assert!(result.unwrap_err().contains("cancelled"));
        assert!(
            driver.transfer.is_empty() && driver.pending.is_none() && driver.linear_front.is_none()
        );
        assert_eq!(driver.linear_rails.as_ref().unwrap().len(), 1);
        driver
            .prepare_linear(
                &source,
                [5., 5., 9.],
                1.,
                0.2,
                82.,
                true,
                &prior,
                1e-5,
                LinkHeights::from(12.),
                150.,
                &|| false,
            )
            .unwrap();
        assert!(driver.linear_front.is_some());
        assert!(!driver.transfer.is_empty());
        assert!(driver.linear_rails.as_ref().unwrap().is_empty());
        assert_eq!(driver.report.linear_resets, 1);
    }
    #[test]
    fn coverage_memo_keeps_protected_stock_and_is_bound_to_floor_domain_and_identity() {
        use crate::planar::{Boundary, Component, ConstructionOptions, PlanarRegion};
        use crate::stock::HeightBounds;
        let square = |a: f64, b: f64| Boundary {
            vertices: vec![[a, a, 0.], [b, a, 0.], [b, b, 0.], [a, b, 0.]],
        };
        let domain = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "protected centre".into(),
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
            components: vec![Component {
                outer: square(0., 8.),
                holes: vec![square(2., 6.)],
            }],
        };
        let make = |definition: &PlanarRegion, floor| {
            let prepared = definition.prepare(ConstructionOptions::default()).unwrap();
            FrontCoverage::new(
                definition.prepare(ConstructionOptions::default()).unwrap(),
                [1., 1.],
                (0, 0),
                floor,
                1e-5,
                0.01,
                0.5,
                prepared,
                0.,
            )
            .unwrap()
        };
        let spec = StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [8., 8., 10.],
            cell_size_mm: 0.25,
            max_cells: 2000,
        };
        let initial = Stock::new(spec.clone()).unwrap();
        let mut snapshot = initial.snapshot();
        for (i, h) in snapshot.cells.iter_mut().enumerate() {
            let p = initial.cell_center(i).unwrap();
            if p.iter().any(|v| *v < 3. || *v > 5.) {
                *h = HeightBounds {
                    lower_mm: 9.,
                    upper_mm: 9.,
                };
            }
        }
        let stock = Stock::from_snapshot(snapshot).unwrap();
        let scene = EngagementScene::new(spec, vec![]).unwrap();
        let mut coverage = make(&domain, 9.);
        let (mut tiles, mut envelopes, mut hits, mut branches) = (0, 0, 0, 0);
        assert!(matches!(
            coverage
                .check(
                    &scene,
                    &stock,
                    [1., 1., 9.],
                    1.,
                    1e-5,
                    &mut tiles,
                    &mut envelopes,
                    &mut hits,
                    &mut branches,
                    &|| false
                )
                .unwrap(),
            CoverageResult::Complete
        ));
        assert!(coverage.covered_tiles.contains(&1));
        assert!(stock.cells().iter().any(|h| h.lower_mm == 10.));
        let old_tiles = tiles;
        assert!(matches!(
            coverage
                .check(
                    &scene,
                    &stock,
                    [7., 7., 9.],
                    1.,
                    1e-5,
                    &mut tiles,
                    &mut envelopes,
                    &mut hits,
                    &mut branches,
                    &|| false
                )
                .unwrap(),
            CoverageResult::Complete
        ));
        assert_eq!(tiles, old_tiles + 1);
        assert!(hits > 0);
        assert!(
            coverage
                .check(
                    &scene,
                    &stock,
                    [7., 7., 9.],
                    1.,
                    1e-5,
                    &mut tiles,
                    &mut envelopes,
                    &mut hits,
                    &mut branches,
                    &|| true
                )
                .is_err()
        );
        // A new floor/domain must start without any of those coverage proofs.
        let mut opened = domain.clone();
        opened.components[0].holes.clear();
        for mut other in [make(&domain, 8.), make(&opened, 9.)] {
            assert!(other.covered_tiles.is_empty());
            assert!(matches!(
                other
                    .check(
                        &scene,
                        &stock,
                        [4., 4., other.floor],
                        1.,
                        1e-5,
                        &mut tiles,
                        &mut envelopes,
                        &mut hits,
                        &mut branches,
                        &|| false
                    )
                    .unwrap(),
                CoverageResult::Gap(_)
            ));
        }
        // Reusing split nodes must still find the same unresolved material as
        // fresh checks, even when changing pose changes breadth-first ordering.
        let mut gap_snapshot = stock.snapshot();
        for (i, h) in gap_snapshot.cells.iter_mut().enumerate() {
            let p = stock.cell_center(i).unwrap();
            if p.iter().all(|v| *v > 0.75 && *v < 1.25) {
                *h = HeightBounds {
                    lower_mm: 10.,
                    upper_mm: 10.,
                };
            }
        }
        let gap_stock = Stock::from_snapshot(gap_snapshot).unwrap();
        let mut retained = make(&domain, 9.);
        let mut reused = 0;
        for p in [[7., 7., 9.], [1., 1., 9.], [7., 1., 9.], [1., 7., 9.]] {
            let mut cold = make(&domain, 9.);
            let expected = cold
                .check(
                    &scene,
                    &gap_stock,
                    p,
                    1.,
                    1e-5,
                    &mut 0,
                    &mut 0,
                    &mut 0,
                    &mut 0,
                    &|| false,
                )
                .unwrap();
            let actual = retained
                .check(
                    &scene,
                    &gap_stock,
                    p,
                    1.,
                    1e-5,
                    &mut 0,
                    &mut 0,
                    &mut 0,
                    &mut reused,
                    &|| false,
                )
                .unwrap();
            match (expected, actual) {
                (CoverageResult::Gap(a), CoverageResult::Gap(b)) => assert_eq!(a, b),
                _ => panic!("unresolved requested stock must remain a gap"),
            }
        }
        assert!(reused > 0);
        assert!(!retained.covered_tiles.contains(&1));

        let fixture: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json"))
                .unwrap();
        let mut driver = FrontDriver::new(&fixture.tools[0]);
        driver.synchronize(&stock, 1., 1e-5, &|| false).unwrap();
        driver.coverage = Some(coverage);
        let restored = Stock::from_snapshot(stock.snapshot()).unwrap();
        assert_eq!(restored.generation(), stock.generation());
        assert!(driver.synchronize(&restored, 1., 1e-5, &|| false).is_err());
        driver.synchronize(&stock, 1., 1e-5, &|| false).unwrap();
        assert!(driver.coverage.unwrap().covered_tiles.contains(&1));
    }

    #[test]
    fn ranked_admission_matches_incremental_native_choice_ties_and_exhausted_prefix() {
        let spec = StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [20., 20., 10.],
            cell_size_mm: 1.,
            max_cells: 1000,
        };
        let stock = Stock::new(spec.clone()).unwrap();
        // This real flat sweep is wholly outside stock. Keeping its index
        // makes zero ranking work genuinely exhaust a primitive/index query.
        let old = Motion {
            id: 0,
            operation: "prior".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            feed_mm_min: Some(600.),
            curve: Curve::Line {
                start: [-2., -2., 5.],
                end: [-1.8, -2., 5.],
            },
        };
        let scene = EngagementScene::new(
            spec,
            vec![ClearedSweep::from_motion(&old, 1., 1e-5).unwrap()],
        )
        .unwrap();
        let limit = 2. * 0.75f64.acos().to_degrees();
        let options = EngagementOptions::default();
        for right in [true, false] {
            let heading = if right { PI / 2. } else { -PI / 2. };
            let goal = Some([0.1, if right { 10.1 } else { 9.9 }]);
            let curves = [
                candidate([-0.8, 10., 5.], heading, 0., 0.125),
                candidate([-0.8, 10., 5.], heading, if right { -0.5 } else { 0.5 }, PI),
                candidate([-0.8, 10., 5.], heading, 0., 0.25),
                candidate([-0.8, 10., 5.], heading, 0., 0.25),
            ];
            let proposals = || {
                curves
                    .iter()
                    .enumerate()
                    .map(|(i, curve)| {
                        // Independent whole-source certificate: every axis bounding
                        // box plus1.1mm fits this free machining envelope.
                        let b = curve.prepare(1e-5).unwrap().bounds();
                        assert!((0..2).all(|k| b.min[k] - 1.1 > -3. && b.max[k] + 1.1 < 23.));
                        let motion = Motion {
                            id: i as u64 + 1,
                            curve: curve.clone(),
                            ..old.clone()
                        };
                        let (score, novelty, tier) = goal_score(
                            &scene,
                            &stock,
                            &motion,
                            1.,
                            right,
                            1e-5,
                            &mut 200000,
                            goal,
                            0.0025,
                            &|| false,
                        )
                        .unwrap();
                        assert!(score > 1e-5);
                        FrontProposal {
                            score,
                            motion,
                            novelty,
                            tier,
                        }
                    })
                    .collect::<Vec<_>>()
            };
            let ranked = proposals();
            assert!(
                ranked[1].score > ranked[2].score,
                "highest rank must be genuinely unsafe"
            );
            assert_eq!(ranked[2].score, ranked[3].score, "two equal safe ranks");
            // Independent reference control flow, using ACTUAL native
            // total/side queries rather than a synthetic boolean predicate.
            let mut reference: Option<RankedFrontMove> = None;
            for p in &ranked {
                if reference.as_ref().is_some_and(|(score, _, _, _, tier)| {
                    p.tier < *tier || (p.tier == *tier && p.score <= *score)
                }) {
                    continue;
                }
                let a = scene
                    .check(&p.motion, 1., limit, options, &|| false)
                    .unwrap();
                if a.status == EngagementStatus::Bounded
                    && scene
                        .check_material_side(&p.motion, 1., right, options, &|| false)
                        .unwrap()
                        .status
                        == EngagementStatus::Bounded
                {
                    reference = Some((
                        p.score,
                        p.motion.clone(),
                        a.upper_angle_deg,
                        p.novelty,
                        p.tier,
                    ));
                }
            }
            assert_eq!(
                reference.as_ref().unwrap().1.id,
                3,
                "first safe tie in generator order"
            );
            let mut report = FrontReport::default();
            let selected = admit_ranked_front(
                ranked,
                None,
                &scene,
                1.,
                limit,
                right,
                options,
                &mut report,
                &|| false,
            )
            .unwrap();
            assert_eq!(selected, reference);
            assert_eq!(
                report.joint_rejections, 1,
                "unsafe best rank was actually checked"
            );
            // A contact bound, even 360deg, cannot admit the wrong material
            // side. This line has the same bounded contact as the safe tie,
            // but reversed feed puts stock on the forbidden half throughout.
            let mut wrong_side = proposals().remove(2);
            wrong_side.motion.id = 100;
            wrong_side.motion.curve = candidate([-0.8, 10., 5.], heading + PI, 0., 0.25);
            wrong_side.score *= 2.;
            assert_eq!(
                scene
                    .check(&wrong_side.motion, 1., limit, options, &|| false)
                    .unwrap()
                    .status,
                EngagementStatus::Bounded
            );
            assert_eq!(
                scene
                    .check_material_side(&wrong_side.motion, 1., right, options, &|| false)
                    .unwrap()
                    .status,
                EngagementStatus::Unresolved
            );
            let wrong = wrong_side.motion.clone();
            let mut safe_ties = proposals();
            let second_tie = safe_ties.pop().unwrap();
            let first_tie = safe_ties.pop().unwrap();
            let mut report = FrontReport::default();
            let side_selected = admit_ranked_front(
                vec![wrong_side, first_tie, second_tie],
                None,
                &scene,
                1.,
                360.,
                right,
                options,
                &mut report,
                &|| false,
            )
            .unwrap()
            .unwrap();
            assert_eq!(side_selected.1.id, 3);
            assert_eq!(report.joint_rejections, 1);
            // Reuse neither a successful side witness nor a past failure
            // across calls. The same candidate reverses safety with feed side.
            let opposite = scene
                .check_front(&wrong, 1., limit, !right, options, &|| false)
                .unwrap();
            assert_eq!(opposite.status, EngagementStatus::Bounded);
            let error = goal_score(
                &scene,
                &stock,
                &proposals()[3].motion,
                1.,
                right,
                1e-5,
                &mut 0,
                goal,
                0.0025,
                &|| false,
            )
            .unwrap_err();
            assert!(error.contains("work limit"));
            let mut prefix = proposals();
            prefix.pop();
            let mut report = FrontReport::default();
            assert_eq!(
                admit_ranked_front(
                    prefix,
                    Some(error.clone()),
                    &scene,
                    1.,
                    limit,
                    right,
                    options,
                    &mut report,
                    &|| false
                )
                .unwrap(),
                reference
            );
            assert_eq!(report.steering_budget_stops, 1);
            let unsafe_only = vec![proposals().remove(1)];
            assert_eq!(
                admit_ranked_front(
                    unsafe_only,
                    Some(error.clone()),
                    &scene,
                    1.,
                    limit,
                    right,
                    options,
                    &mut FrontReport::default(),
                    &|| false
                )
                .unwrap_err(),
                error
            );
            let exhausted = EngagementOptions {
                max_work: 0,
                ..options
            };
            assert!(
                admit_ranked_front(
                    proposals(),
                    None,
                    &scene,
                    1.,
                    limit,
                    right,
                    exhausted,
                    &mut FrontReport::default(),
                    &|| false
                )
                .unwrap()
                .is_none()
            );
            assert!(
                admit_ranked_front(
                    proposals(),
                    None,
                    &scene,
                    1.,
                    limit,
                    right,
                    options,
                    &mut FrontReport::default(),
                    &|| true
                )
                .unwrap_err()
                .contains("cancelled")
            );
        }
    }

    #[test]
    fn clear_reentry_can_need_more_than_a_quarter_turn_before_cutting() {
        let spec = StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [20., 20., 10.],
            cell_size_mm: 1.,
            max_cells: 1000,
        };
        let stock = Stock::new(spec.clone()).unwrap();
        let scene = EngagementScene::new(spec, vec![]).unwrap();
        for right in [true, false] {
            // Whole forbidden-side support of a turning radius-.2 cutter is
            // hypot(1,.2) from the circle centre. The bottom of the circle
            // supplies a clear descent; its first quarter remains in air.
            let stand_off = 1f64.hypot(0.2) + 0.001;
            let start = [10., -stand_off - 0.2, 5.];
            let heading = if right { 0. } else { PI };
            let prototype = Motion {
                id: 1,
                operation: "delayed-entry".into(),
                tool: "t".into(),
                kind: MoveKind::Lead,
                curve: candidate(start, heading, if right { 5. } else { -5. }, PI * 0.1),
                feed_mm_min: Some(600.),
            };
            let score = |curve| {
                goal_score(
                    &scene,
                    &stock,
                    &Motion {
                        curve,
                        ..prototype.clone()
                    },
                    1.,
                    right,
                    1e-5,
                    &mut 200000,
                    None,
                    0.0025,
                    &|| false,
                )
                .unwrap()
            };
            assert_eq!(score(prototype.curve.clone()).2, 0, "first quarter is air");
            assert!(
                scene
                    .clear_disk(
                        [start[0], start[1]],
                        1.00064,
                        5.,
                        1e-5,
                        &mut 200000,
                        &|| false
                    )
                    .unwrap()
            );
            let curve = reentry_leads(start, heading, 1., 0.2, right)
                .into_iter()
                .find(|curve| {
                    matches!(curve, Curve::Arc { sweep_rad, .. } if sweep_rad.abs() >= PI)
                        && score(curve.clone()).2 >= 2
                })
                .expect("long turn reaches the front after an empty first quarter");
            let checked = scene
                .check_front(
                    &Motion {
                        curve,
                        ..prototype.clone()
                    },
                    1.,
                    2. * 0.75f64.acos().to_degrees(),
                    right,
                    EngagementOptions::default(),
                    &|| false,
                )
                .unwrap();
            assert_eq!(checked.status, EngagementStatus::Bounded);
        }
    }
    #[test]
    fn recorded_straight_front_has_a_long_checked_advance_between_coarse_curvatures() {
        let data: serde_json::Value = serde_json::from_str(include_str!(
            "../fixtures/operations/stock-front-straight-transition.json"
        ))
        .unwrap();
        let manifest: crate::stream::StreamManifest =
            serde_json::from_value(data["manifest"].clone()).unwrap();
        let history: Vec<Motion> = serde_json::from_value(data["history"].clone()).unwrap();
        let old_next: Motion = serde_json::from_value(data["old_next"].clone()).unwrap();
        let tool = manifest.tools[0].prepare().unwrap();
        let mut stock = Stock::new(manifest.stock.height().unwrap().clone()).unwrap();
        let mut scene =
            EngagementScene::new(manifest.stock.height().unwrap().clone(), vec![]).unwrap();
        for motion in &history {
            let delta = stock
                .preview_sweep(&tool, motion, 1., manifest.sweep, &|| false)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            if motion.kind != MoveKind::Rapid && motion.curve.start()[2] == motion.curve.end()[2] {
                scene
                    .append(ClearedSweep::from_motion(motion, 1., 1e-5).unwrap())
                    .unwrap();
            }
        }
        let start = history.last().unwrap().curve.end();
        assert_eq!(start, old_next.curve.start());
        let heading = tangent(&history.last().unwrap().curve, 1.);
        let score = |motion: &Motion| {
            potential_score(
                &scene,
                &stock,
                motion,
                1.,
                true,
                1e-5,
                &mut 2_000_000,
                None,
                0.0025,
                &|| false,
            )
            .unwrap()
            .0
        };
        let old_score = score(&old_next);
        let crate::pocket::Operation::Roughing(op) = &manifest.operations[0] else {
            panic!("roughing fixture")
        };
        // Independent sufficient source certificate for this left-edge case:
        // whole curve bounds lie in the expanded stock envelope and strictly
        // left of every retained triangle that can reach this cutting floor.
        let part_x = op
            .target
            .triangles
            .iter()
            .filter(|triangle| {
                triangle
                    .iter()
                    .any(|p| p[2] >= start[2] - op.bottom_allowance_mm)
            })
            .flatten()
            .map(|p| p[0])
            .fold(f64::INFINITY, f64::min);
        let mut work = 1_000_000;
        let refined = refine_long_advance(
            &scene,
            &stock,
            start,
            heading,
            1.,
            0.2,
            2. * 0.75f64.acos().to_degrees(),
            true,
            &old_next,
            1e-5,
            None,
            0.0025,
            &mut work,
            |curve| {
                let b = curve.prepare(1e-5).unwrap().bounds();
                let reserve = 1. + op.radial_allowance_mm + 0.01;
                if b.max[0] + reserve < part_x
                    && (0..2).all(|k| {
                        b.min[k] - reserve > manifest.stock.height().unwrap().min_mm[k] - 3.
                            && b.max[k] + reserve < manifest.stock.height().unwrap().max_mm[k] + 3.
                    })
                {
                    Some(1.)
                } else {
                    None
                }
            },
            &|| false,
        )
        .unwrap();
        let (rank, motion, angle, novel, _) = refined.best.expect("long checked continuation");
        eprintln!(
            "recorded straight-front old length={} score={old_score}; refined length={} score={rank}, angle={angle}, work={}",
            old_next.curve.prepare(1e-5).unwrap().length_mm(),
            motion.curve.prepare(1e-5).unwrap().length_mm(),
            1_000_000 - work
        );
        assert!(novel && rank > old_score * 1.5);
        assert_eq!(motion.curve.start(), start);
        assert!(motion.curve.prepare(1e-5).unwrap().length_mm() >= 3.99);
        assert!(angle <= 2. * 0.75f64.acos().to_degrees());
        assert!(!refined.budget_stopped);
        assert_eq!(
            scene
                .check_material_side(&motion, 1., true, EngagementOptions::default(), &|| false)
                .unwrap()
                .status,
            EngagementStatus::Bounded
        );
        assert!(
            refine_long_advance(
                &scene,
                &stock,
                start,
                heading,
                1.,
                0.2,
                90.,
                true,
                &old_next,
                1e-5,
                None,
                0.0025,
                &mut 1_000_000,
                |_| Some(1.),
                &|| true
            )
            .err()
            .unwrap()
            .contains("cancelled")
        );
        let limited = refine_long_advance(
            &scene,
            &stock,
            start,
            heading,
            1.,
            0.2,
            90.,
            true,
            &old_next,
            1e-5,
            None,
            0.0025,
            &mut 0,
            |_| Some(1.),
            &|| false,
        )
        .unwrap();
        assert!(limited.best.is_none() && limited.budget_stopped);
    }
    #[test]
    fn useful_reentry_advance_does_not_require_a_productive_full_recovery_orbit() {
        let spec = StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [20., 20., 10.],
            cell_size_mm: 1.,
            max_cells: 1000,
        };
        let stock = Stock::new(spec.clone()).unwrap();
        let scene = EngagementScene::new(spec, vec![]).unwrap();
        let start = [-1.001, 10., 5.];
        for right in [true, false] {
            let sign = if right { 1. } else { -1. };
            let heading = sign * PI / 2.;
            let prototype = Motion {
                id: 1,
                operation: "reentry".into(),
                tool: "t".into(),
                kind: MoveKind::Lead,
                curve: candidate(start, heading, sign / 0.2, TAU * 0.2),
                feed_mm_min: Some(600.),
            };
            // A whole guarded descent disk is initially outside the stock.
            assert!(
                scene
                    .clear_disk([-1.001, 10.], 1.00064, 5., 1e-5, &mut 200000, &|| false)
                    .unwrap()
            );
            let rank = |motion: &Motion| {
                potential_score(
                    &scene,
                    &stock,
                    motion,
                    1.,
                    right,
                    1e-5,
                    &mut 200000,
                    None,
                    0.0025,
                    &|| false,
                )
                .unwrap()
            };
            // The old mandatory recovery circle lies entirely in air.
            assert!(!rank(&prototype).1);
            let admitted = reentry_leads(start, heading, 1., 0.2, right)
                .into_iter()
                .map(|curve| Motion {
                    curve,
                    ..prototype.clone()
                })
                .find(|motion| {
                    rank(motion).1
                        && scene
                            .check_front(
                                motion,
                                1.,
                                2. * 0.75f64.acos().to_degrees(),
                                right,
                                EngagementOptions::default(),
                                &|| false,
                            )
                            .unwrap()
                            .status
                            == EngagementStatus::Bounded
                })
                .expect("a checked partial advance reaches material on the requested side");
            assert_eq!(admitted.curve.start(), start);
            assert!(admitted.curve.end()[0] > start[0] + 0.05);
            assert!(admitted.curve.prepare(1e-5).unwrap().length_mm() >= 0.5);
            if let Curve::Arc { sweep_rad, .. } = admitted.curve {
                assert!(sweep_rad.abs() <= PI / 2.);
            }
        }
    }
    #[test]
    fn air_heavy_motion_does_not_score_as_fully_productive() {
        let spec = StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [20., 20., 10.],
            cell_size_mm: 1.,
            max_cells: 1000,
        };
        let stock = Stock::new(spec.clone()).unwrap();
        let scene = EngagementScene::new(spec, vec![]).unwrap();
        let score = |start, end| {
            let motion = Motion {
                id: 1,
                operation: "rank".into(),
                tool: "t".into(),
                kind: MoveKind::Cut,
                curve: Curve::Line { start, end },
                feed_mm_min: Some(600.),
            };
            potential_score(
                &scene,
                &stock,
                &motion,
                1.,
                true,
                1e-5,
                &mut 200000,
                None,
                0.0025,
                &|| false,
            )
            .unwrap()
        };
        // Equal 4mm paths. One only clips the stock corner during its first
        // quarter, then spends the rest in air; the other encounters material
        // throughout. This exercises the real geometry/stock ranking query.
        let (air, air_novel, air_tier) = score([1., -0.8, 5.], [-3., -0.8, 5.]);
        let (bulk, bulk_novel, bulk_tier) = score([10., 10., 5.], [6., 10., 5.]);
        assert!(air_novel && bulk_novel);
        assert_eq!(air_tier, bulk_tier);
        assert!(air < 1. && bulk > 3.9, "air={air}, bulk={bulk}");
    }
    #[test]
    fn forward_swept_area_ranking_distinguishes_chip_width_from_side_contact() {
        let spec = StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [20., 20., 10.],
            cell_size_mm: 1.,
            max_cells: 1000,
        };
        let stock = Stock::new(spec.clone()).unwrap();
        let scene = EngagementScene::new(spec, vec![]).unwrap();
        let area = |width: f64| {
            let x = -(1. - width);
            let motion = Motion {
                id: 1,
                operation: "rank".into(),
                tool: "t".into(),
                kind: MoveKind::Cut,
                curve: Curve::Line {
                    start: [x, 5., 5.],
                    end: [x, 9., 5.],
                },
                feed_mm_min: Some(600.),
            };
            potential_score(
                &scene,
                &stock,
                &motion,
                1.,
                true,
                1e-5,
                &mut 200000,
                None,
                0.0025,
                &|| false,
            )
            .unwrap()
            .0
        };
        // Against a straight wall the exact new area is travel*radial width.
        // Quadrature is a ranking approximation, never a motion certificate.
        let wide = area(0.25) / 4.;
        let thin = area(0.02) / 4.;
        assert!(wide > 0.2 && wide < 0.35, "wide chip estimate {wide}");
        assert!(thin < 0.05, "thin chip estimate {thin}");
        assert!(
            wide > 6. * thin,
            "side-contact point count cannot resolve width: {wide}/{thin}"
        );
    }
}
