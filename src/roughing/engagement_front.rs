//! Engagement-front clearing of one roughing layer (`path_strategy =
//! "engagement_front"`, issues #26/#27).
//!
//! Established practice: constant-engagement ("adaptive", "dynamic",
//! "trochoidal-free high-efficiency") roughing keeps the cutter's radial
//! engagement near a target width by steering along the boundary between cut
//! and uncut stock, entering from air or already-cleared stock wherever it
//! can and helixing only into enclosed material. This module is a clean-room
//! implementation of that idea; it shares no code with any other CAM kernel.
//!
//! The steering model is a fine height grid seeded from the committed stock
//! (whole-cell upper bounds, so it never sees less material than there is)
//! and this layer's own planned sweeps. It is an *estimate* that only sizes
//! the engagement; it is not a safety proof. Safety comes from:
//! - every cutting/link stroke is certified exactly against the layer region
//!   (cutter radius + radial allowance + construction uncertainty) and a
//!   stroke that fails is refused with an error, never adjusted;
//! - every vertical air entry is proven clear by the committed-sweep scene
//!   (the same proof pocket air entries use); a start that cannot be proven
//!   is not used;
//! - the stream verifier still checks the run against stock, part and machine.
use crate::{
    Point,
    entry::{EntryPolicy, PocketEntry},
    heights::Heights,
    motion::{Curve, Event, Motion, MoveKind, Vec3},
    planar::Boundary,
    pocket::{PocketOperation, PocketPlan, ResidualStatus, Stroke},
    stock::{CancelSignal, Stock, StockSpec},
    tool::ToolAssembly,
};
use serde::Serialize;
use std::{
    collections::VecDeque,
    f64::consts::{PI, TAU},
};

/// Fine-grid geometry shared by every layer of one roughing operation.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Grid {
    x0: f64,
    y0: f64,
    c: f64,
    nx: usize,
    ny: usize,
}
impl Grid {
    fn idx(&self, i: usize, j: usize) -> usize {
        j * self.nx + i
    }
    fn center(&self, k: usize) -> Point {
        [
            self.x0 + ((k % self.nx) as f64 + 0.5) * self.c,
            self.y0 + ((k / self.nx) as f64 + 0.5) * self.c,
        ]
    }
    fn cell(&self, p: Point) -> Option<(usize, usize)> {
        let i = ((p[0] - self.x0) / self.c).floor();
        let j = ((p[1] - self.y0) / self.c).floor();
        (i >= 0. && j >= 0. && i < self.nx as f64 && j < self.ny as f64)
            .then_some((i as usize, j as usize))
    }
    /// Cells whose centres lie within `rad` of `p`.
    fn for_disk(&self, p: Point, rad: f64, mut f: impl FnMut(usize, Point, f64)) {
        let lo = [
            ((p[0] - rad - self.x0) / self.c - 0.5).ceil().max(0.) as usize,
            ((p[1] - rad - self.y0) / self.c - 0.5).ceil().max(0.) as usize,
        ];
        let hi = [
            (((p[0] + rad - self.x0) / self.c - 0.5).floor()).min(self.nx as f64 - 1.),
            (((p[1] + rad - self.y0) / self.c - 0.5).floor()).min(self.ny as f64 - 1.),
        ];
        if hi[0] < 0. || hi[1] < 0. {
            return;
        }
        for j in lo[1]..=hi[1] as usize {
            let cy = self.y0 + (j as f64 + 0.5) * self.c;
            for i in lo[0]..=hi[0] as usize {
                let cx = self.x0 + (i as f64 + 0.5) * self.c;
                let d = (cx - p[0]).hypot(cy - p[1]);
                if d <= rad {
                    f(self.idx(i, j), [cx, cy], d);
                }
            }
        }
    }
}

/// Heights this operation's own committed layers cleared on the fine grid,
/// handed from one layer to the next. Valid only for the stock generation it
/// was produced against.
pub(crate) struct FrontMemory {
    grid: Grid,
    cleared: Vec<f64>,
    generation: u64,
}

#[derive(Clone, Debug, Serialize)]
pub struct EngagementFrontReport {
    pub strategy: &'static str,
    pub target_z_mm: f64,
    pub target_radial_engagement_mm: f64,
    /// Largest radial width the steering grid predicted for a planned step.
    /// A grid estimate (see `engagement_grid_mm`), not a verified bound.
    pub max_estimated_engagement_mm: f64,
    pub engagement_grid_mm: f64,
    pub passes: usize,
    pub stay_down_links: usize,
    pub lifts: usize,
    pub stock_edge_entries: usize,
    pub helical_entries: usize,
    pub entry_turns: usize,
    pub unproven_air_starts: usize,
    pub grid_refreshes: usize,
    /// Why passes ended: material exhausted ahead, the far side would
    /// overload, the front thinned out, or the region boundary blocked it.
    pub pass_ends_exhausted: usize,
    pub pass_ends_overload: usize,
    pub pass_ends_thin: usize,
    pub pass_ends_boundary: usize,
    pub certified_strokes: usize,
    pub arcs: usize,
    pub motions: usize,
    pub stock_top_carried: bool,
    pub geometric_residual_status: ResidualStatus,
    pub geometric_residual_area_mm2: Option<f64>,
    pub diagnostics: Vec<String>,
}

impl Default for EngagementFrontReport {
    fn default() -> Self {
        Self {
            strategy: "engagement_front",
            target_z_mm: 0.,
            target_radial_engagement_mm: 0.,
            max_estimated_engagement_mm: 0.,
            engagement_grid_mm: 0.,
            passes: 0,
            stay_down_links: 0,
            lifts: 0,
            stock_edge_entries: 0,
            helical_entries: 0,
            entry_turns: 0,
            unproven_air_starts: 0,
            grid_refreshes: 0,
            pass_ends_exhausted: 0,
            pass_ends_overload: 0,
            pass_ends_thin: 0,
            pass_ends_boundary: 0,
            certified_strokes: 0,
            arcs: 0,
            motions: 0,
            stock_top_carried: false,
            geometric_residual_status: ResidualStatus::Pending,
            geometric_residual_area_mm2: None,
            diagnostics: vec![],
        }
    }
}

struct Layer {
    g: Grid,
    /// Top of possible material per fine cell.
    top: Vec<f64>,
    /// This operation's cleared height per fine cell.
    cleared: Vec<f64>,
    allowed: Vec<bool>,
    /// Material the cutter can reach from an allowed centre.
    reachable: Vec<bool>,
    /// Distance from the cell centre to the outside of the region (mm).
    room: Vec<f32>,
    dead: Vec<bool>,
    candidates: Vec<usize>,
    z: f64,
    material_guard: f64,
}
impl Layer {
    fn material(&self, k: usize) -> bool {
        self.top[k].min(self.cleared[k]) > self.z + self.material_guard
    }
    fn allowed_at(&self, p: Point) -> bool {
        self.g
            .cell(p)
            .is_some_and(|(i, j)| self.allowed[self.g.idx(i, j)])
    }
    fn cut(&mut self, p: Point, rad: f64) {
        let z = self.z;
        let cleared = &mut self.cleared;
        self.g.for_disk(p, rad, |k, _, _| {
            if cleared[k] > z {
                cleared[k] = z;
            }
        });
    }
    /// Material cell centres a step of up to `reach` from `p` can meet.
    fn gather(&self, p: Point, reach: f64, out: &mut Vec<Point>) {
        out.clear();
        self.g.for_disk(p, reach, |k, q, _| {
            if self.material(k) {
                out.push(q);
            }
        });
    }
}

/// Smoothed material area (mm²) inside the cutter disk centred at `p`.
fn area(cells: &[Point], p: Point, r: f64, c: f64) -> f64 {
    let mut n = 0.;
    for q in cells {
        let d = (q[0] - p[0]).hypot(q[1] - p[1]);
        n += ((r - d) / c + 0.5).clamp(0., 1.);
    }
    n * c * c
}

/// Exact squared Euclidean distance transform (Felzenszwalb & Huttenlocher,
/// "Distance Transforms of Sampled Functions", 2012): distance in mm from each
/// cell centre to the nearest `true` cell centre (1e9 when none).
fn edt(g: &Grid, mask: &[bool]) -> Vec<f64> {
    const INF: f64 = 1e20;
    fn pass(f: &[f64], out: &mut [f64], v: &mut [usize], z: &mut [f64]) {
        let n = f.len();
        let mut k = 0usize;
        v[0] = 0;
        z[0] = f64::NEG_INFINITY;
        z[1] = f64::INFINITY;
        for q in 1..n {
            loop {
                let p = v[k];
                let s = ((f[q] + (q * q) as f64) - (f[p] + (p * p) as f64))
                    / (2. * q as f64 - 2. * p as f64);
                if s <= z[k] {
                    if k == 0 {
                        v[0] = q;
                        z[0] = f64::NEG_INFINITY;
                        z[1] = f64::INFINITY;
                        break;
                    }
                    k -= 1;
                } else {
                    k += 1;
                    v[k] = q;
                    z[k] = s;
                    z[k + 1] = f64::INFINITY;
                    break;
                }
            }
        }
        k = 0;
        for (q, o) in out.iter_mut().enumerate() {
            while z[k + 1] < q as f64 {
                k += 1;
            }
            let d = q as f64 - v[k] as f64;
            *o = d * d + f[v[k]];
        }
    }
    let (nx, ny) = (g.nx, g.ny);
    let mut f: Vec<f64> = mask.iter().map(|&m| if m { 0. } else { INF }).collect();
    let n = nx.max(ny);
    let (mut v, mut zz) = (vec![0usize; n], vec![0f64; n + 1]);
    let (mut col, mut out) = (vec![0f64; ny], vec![0f64; ny]);
    for i in 0..nx {
        for j in 0..ny {
            col[j] = f[j * nx + i];
        }
        pass(&col, &mut out, &mut v, &mut zz);
        for j in 0..ny {
            f[j * nx + i] = out[j];
        }
    }
    let mut row = vec![0f64; nx];
    let mut out = vec![0f64; nx];
    for j in 0..ny {
        row.copy_from_slice(&f[j * nx..(j + 1) * nx]);
        pass(&row, &mut out, &mut v, &mut zz);
        f[j * nx..(j + 1) * nx].copy_from_slice(&out);
    }
    f.iter()
        .map(|&d| if d >= INF / 2. { 1e9 } else { d.sqrt() * g.c })
        .collect()
}

/// Flatten a region boundary to straight segments with sagitta <= `sag`.
fn flatten(b: &Boundary, sag: f64, out: &mut Vec<(Point, Point)>) {
    for s in Stroke::from_boundary(b) {
        let Some((c, sweep)) = s.arc else {
            out.push((s.a, s.b));
            continue;
        };
        let rho = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
        let step = 2. * (1. - sag / rho).clamp(-1., 1.).acos();
        let n = ((sweep.abs() / step.max(1e-6)).ceil() as usize).clamp(1, 1 << 16);
        let a0 = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
        let mut prev = s.a;
        for t in 1..=n {
            let p = if t == n {
                s.b
            } else {
                let a = a0 + sweep * t as f64 / n as f64;
                [c[0] + rho * a.cos(), c[1] + rho * a.sin()]
            };
            out.push((prev, p));
            prev = p;
        }
    }
}

/// Greedy line/arc fit of a polyline: each output stroke passes within `tol`
/// of every point it replaces.
fn fit_strokes(pts: &[Point], tol: f64, max_radius: f64) -> Vec<Stroke> {
    fn one(q: &[Point], tol: f64, max_radius: f64) -> Option<Option<(Point, f64)>> {
        let (a, b) = (q[0], *q.last().unwrap());
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l = dx.hypot(dy);
        if l < 1e-9 {
            return None;
        }
        if q.iter().all(|p| {
            let along = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l;
            ((p[0] - a[0]) * dy - (p[1] - a[1]) * dx).abs() / l <= tol
                && along >= -tol
                && along <= l + tol
        }) {
            return Some(None);
        }
        if q.len() < 3 {
            return None;
        }
        let m = q[q.len() / 2];
        let d = 2. * (a[0] * (m[1] - b[1]) + m[0] * (b[1] - a[1]) + b[0] * (a[1] - m[1]));
        if d.abs() < 1e-12 {
            return None;
        }
        let (a2, m2, b2) = (
            a[0] * a[0] + a[1] * a[1],
            m[0] * m[0] + m[1] * m[1],
            b[0] * b[0] + b[1] * b[1],
        );
        let c = [
            (a2 * (m[1] - b[1]) + m2 * (b[1] - a[1]) + b2 * (a[1] - m[1])) / d,
            (a2 * (b[0] - m[0]) + m2 * (a[0] - b[0]) + b2 * (m[0] - a[0])) / d,
        ];
        let rho = (a[0] - c[0]).hypot(a[1] - c[1]);
        if rho > max_radius
            || q.iter()
                .any(|p| ((p[0] - c[0]).hypot(p[1] - c[1]) - rho).abs() > tol)
        {
            return None;
        }
        let mut sweep = 0.;
        for w in q.windows(2) {
            let mut da =
                (w[1][1] - c[1]).atan2(w[1][0] - c[0]) - (w[0][1] - c[1]).atan2(w[0][0] - c[0]);
            if da > PI {
                da -= TAU;
            }
            if da < -PI {
                da += TAU;
            }
            // A fitted arc must turn monotonically through its points.
            if sweep != 0. && da != 0. && da.signum() != f64::signum(sweep) {
                return None;
            }
            sweep += da;
        }
        (sweep.abs() <= 0.9 * PI && sweep != 0.).then_some(Some((c, sweep)))
    }
    let mut out = vec![];
    let mut i = 0;
    while i + 1 < pts.len() {
        let mut best = (i + 1, None);
        let mut j = i + 1;
        while j < pts.len() {
            match one(&pts[i..=j], tol, max_radius) {
                Some(f) => {
                    best = (j, f);
                    j += 1;
                }
                None if j == i + 1 => {
                    // Coincident points: skip.
                    best = (j, None);
                    break;
                }
                None => break,
            }
        }
        let (j, arc) = best;
        if pts[i] != pts[j] {
            out.push(Stroke {
                a: pts[i],
                b: pts[j],
                arc,
            });
        }
        i = j;
    }
    out
}

type Pending = (Curve, MoveKind, Option<f64>);

pub(crate) struct EngagementFrontPlan {
    guides: PocketPlan,
    op: PocketOperation,
    entry: PocketEntry,
    heights: Heights,
    r: f64,
    width: f64,
    climb: bool,
    z: f64,
    spec: StockSpec,
    layer: Option<Layer>,
    memory: Option<FrontMemory>,
    pending: VecDeque<Pending>,
    position: Vec3,
    next_id: u64,
    generation: Option<u64>,
    finished: bool,
    report: EngagementFrontReport,
    scratch: Vec<Point>,
}

impl EngagementFrontPlan {
    pub(crate) fn prepare(
        op: PocketOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        width: f64,
        climb: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let entry = op
            .entry
            .clone()
            .ok_or("engagement_front needs an entry policy")?;
        if entry.policy() == EntryPolicy::Helix {
            return Err("engagement_front enters from air or cleared stock and helixes only into enclosed material; use StockEdgeEntry (stock_edge_or_helix or stock_edge), not HelicalEntry".into());
        }
        let r = tool.prepare()?.cutter().definition().diameter_mm / 2.;
        if !width.is_finite() || width <= 0. || width > r {
            return Err(
                "engagement_front radial engagement must be positive and at most the cutter radius"
                    .into(),
            );
        }
        let guard = 16. * op.tolerance_mm;
        let heights = Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2].max(op.top_z_mm),
            guard,
        )?;
        let guides = PocketPlan::prepare(op.clone(), tool, spec, initial, 1, cancel)?;
        let z = guides.report().target_z_mm;
        let report = EngagementFrontReport {
            strategy: "engagement_front",
            target_z_mm: z,
            target_radial_engagement_mm: width,
            geometric_residual_status: ResidualStatus::Pending,
            diagnostics: vec![
                "Engagement is steered on a fine grid seeded from committed whole-cell stock bounds; max_estimated_engagement_mm is a grid estimate, not a verified bound".into(),
                "Every cutting and link stroke is certified exactly against the layer region; air entries are proven by committed sweeps; enclosed material is entered by helix".into(),
            ],
            ..Default::default()
        };
        Ok(Self {
            guides,
            op,
            entry,
            heights,
            r,
            width,
            climb,
            z,
            spec: spec.clone(),
            layer: None,
            memory: None,
            pending: VecDeque::new(),
            position: initial,
            next_id: 1,
            generation: None,
            finished: false,
            report,
            scratch: vec![],
        })
    }
    pub(crate) fn construction_error_mm(&self) -> f64 {
        self.guides.construction_error_mm()
    }
    pub(crate) fn activate_layer(
        &mut self,
        initial: Vec3,
        first_id: u64,
        stock: &StockSpec,
    ) -> Result<(), String> {
        if self.report.motions != 0 || self.generation.is_some() {
            return Err("cannot activate a used engagement_front layer".into());
        }
        self.guides.activate_layer(initial, first_id, stock)?;
        self.position = initial;
        self.next_id = first_id;
        self.spec = stock.clone();
        Ok(())
    }
    pub(crate) fn give_memory(&mut self, memory: FrontMemory) {
        self.memory = Some(memory);
    }
    pub(crate) fn take_memory(&mut self) -> Option<FrontMemory> {
        let layer = self.layer.take()?;
        Some(FrontMemory {
            grid: layer.g,
            cleared: layer.cleared,
            generation: self.generation?,
        })
    }
    pub(crate) fn report(&self) -> &EngagementFrontReport {
        &self.report
    }
    pub(crate) fn prefix_events(&self) -> Vec<Event> {
        self.guides.prefix_events()
    }
    pub(crate) fn suffix_events(&self) -> Vec<Event> {
        self.guides.suffix_events()
    }
    pub(crate) fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("engagement_front report cancelled".into());
        }
        if !self.finished || !self.pending.is_empty() {
            return Err("drain engagement_front motions before report".into());
        }
        if self.report.geometric_residual_status == ResidualStatus::Pending {
            match &self.layer {
                Some(layer) => {
                    let c2 = layer.g.c * layer.g.c;
                    let left = (0..layer.top.len())
                        .filter(|&k| layer.reachable[k] && layer.material(k))
                        .count() as f64
                        * c2;
                    self.report.geometric_residual_area_mm2 = Some(left);
                    self.report.geometric_residual_status = ResidualStatus::Estimated;
                }
                None => self.report.geometric_residual_status = ResidualStatus::Unknown,
            }
        }
        Ok(())
    }

    fn grid_for(&self, stock: &Stock) -> Result<Grid, String> {
        let step = stock.cell_widths_mm();
        // Steering resolution: fine enough to resolve the target width.
        let want = (self.r / 20.).min(self.width / 5.);
        let c = step[0] / (step[0] / want).ceil();
        let margin = 3. * self.r + self.op.radial_allowance_mm + 24. * c;
        let x0 = self.spec.min_mm[0] - (margin / c).ceil() * c;
        let y0 = self.spec.min_mm[1] - (margin / c).ceil() * c;
        let nx = ((self.spec.max_mm[0] + margin - x0) / c).ceil() as usize;
        let ny = ((self.spec.max_mm[1] + margin - y0) / c).ceil() as usize;
        crate::resources::check_memory((nx * ny) as u128 * 32, "engagement_front steering grid")?;
        if nx * ny > 64_000_000 {
            return Err(format!(
                "engagement_front steering grid of {nx} x {ny} cells exceeds 64M; use a larger tool or split the stock"
            ));
        }
        Ok(Grid { x0, y0, c, nx, ny })
    }

    /// Clearance a cutter centre needs on the grid: the exact certification
    /// requirement plus grid, chord and fit error.
    fn grid_clearance(&self, c: f64) -> f64 {
        let region = self.guides.source_region();
        let uncertainty = region.definition().boundary_error_mm + self.op.tolerance_mm;
        self.r
            + self.op.radial_allowance_mm
            + uncertainty
            + 4. * self.op.tolerance_mm
            + 1.5 * c
            + c / 8.
            + c / 10.
    }

    fn build_layer(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<(), String> {
        let g = self.grid_for(stock)?;
        let n = g.nx * g.ny;
        // Region interior at cell centres (even-odd over flattened edges).
        let mut edges = vec![];
        for comp in &self.guides.source_region().definition().components {
            flatten(&comp.outer, g.c / 8., &mut edges);
            for h in &comp.holes {
                flatten(h, g.c / 8., &mut edges);
            }
        }
        let mut inside = vec![false; n];
        let mut xs = vec![];
        for j in 0..g.ny {
            if j % 256 == 0 && cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            let y = g.y0 + (j as f64 + 0.5) * g.c;
            xs.clear();
            for &(a, b) in &edges {
                if (a[1] > y) != (b[1] > y) {
                    xs.push(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]));
                }
            }
            xs.sort_by(f64::total_cmp);
            for pair in xs.chunks_exact(2) {
                let i0 = ((pair[0] - g.x0) / g.c - 0.5).ceil().max(0.) as usize;
                let i1 = ((pair[1] - g.x0) / g.c - 0.5).floor().min(g.nx as f64 - 1.);
                if i1 < 0. {
                    continue;
                }
                for i in i0..=i1 as usize {
                    inside[g.idx(i, j)] = true;
                }
            }
        }
        // The grid border counts as outside.
        for i in 0..g.nx {
            inside[g.idx(i, 0)] = false;
            inside[g.idx(i, g.ny - 1)] = false;
        }
        for j in 0..g.ny {
            inside[g.idx(0, j)] = false;
            inside[g.idx(g.nx - 1, j)] = false;
        }
        let outside: Vec<bool> = inside.iter().map(|b| !b).collect();
        let room: Vec<f32> = edt(&g, &outside).into_iter().map(|d| d as f32).collect();
        let need = self.grid_clearance(g.c);
        let allowed: Vec<bool> = room.iter().map(|&d| d as f64 >= need).collect();
        let dallowed = edt(&g, &allowed);
        // Stock: whole-cell upper bounds, so no material is ever missed.
        let [snx, sny] = stock.dimensions();
        let step = stock.cell_widths_mm();
        let cells = stock.cells();
        let smin = stock.spec().min_mm;
        let top: Vec<f64> = (0..n)
            .map(|k| {
                let p = g.center(k);
                let i = ((p[0] - smin[0]) / step[0]).floor();
                let j = ((p[1] - smin[1]) / step[1]).floor();
                if i < 0. || j < 0. || i >= snx as f64 || j >= sny as f64 {
                    f64::NEG_INFINITY
                } else {
                    cells[j as usize * snx + i as usize].upper_mm
                }
            })
            .collect();
        let cleared = match self.memory.take() {
            Some(m) if m.grid == g && Some(m.generation) == Some(stock.generation()) => {
                self.report.stock_top_carried = true;
                m.cleared
            }
            Some(_) => {
                self.report.diagnostics.push("previous layer's steering memory did not match the committed stock; steering from whole-cell stock bounds only".into());
                vec![f64::INFINITY; n]
            }
            None => vec![f64::INFINITY; n],
        };
        let reach_limit = self.r - 0.5 * g.c;
        let reachable = dallowed.iter().map(|&d| d <= reach_limit).collect();
        self.report.engagement_grid_mm = g.c;
        self.layer = Some(Layer {
            g,
            top,
            cleared,
            allowed,
            reachable,
            room,
            dead: vec![false; n],
            candidates: vec![],
            z: self.z,
            material_guard: 16. * self.op.tolerance_mm,
        });
        Ok(())
    }

    /// Recompute air/cleared start candidates: allowed centres whose cutter
    /// disk is clear of material while reachable material is within a few
    /// grid cells. Returns the reachable material area left.
    fn refresh(&mut self) -> f64 {
        let r = self.r;
        let layer = self.layer.as_mut().unwrap();
        let g = layer.g;
        let n = g.nx * g.ny;
        let mat: Vec<bool> = (0..n).map(|k| layer.material(k)).collect();
        let reach: Vec<bool> = (0..n).map(|k| mat[k] && layer.reachable[k]).collect();
        let left = reach.iter().filter(|&&b| b).count() as f64 * g.c * g.c;
        let dmat = edt(&g, &mat);
        let dk = edt(&g, &reach);
        layer.candidates = (0..n)
            .filter(|&k| {
                layer.allowed[k] && !layer.dead[k] && dmat[k] > r + g.c && dk[k] <= r + 3. * g.c
            })
            .collect();
        self.report.grid_refreshes += 1;
        left
    }

    /// Validate candidate `k` locally against the current grid: returns the
    /// unit direction towards the nearest reachable material.
    fn candidate_direction(&self, k: usize) -> Option<Point> {
        let layer = self.layer.as_ref().unwrap();
        if !layer.allowed[k] || layer.dead[k] {
            return None;
        }
        let p = layer.g.center(k);
        let c = layer.g.c;
        let mut blocked = false;
        let mut nearest: Option<(f64, Point)> = None;
        layer.g.for_disk(p, self.r + 3. * c, |q, qc, d| {
            if layer.material(q) {
                if d <= self.r + c {
                    blocked = true;
                }
                if layer.reachable[q] && nearest.is_none_or(|(b, _)| d < b) {
                    nearest = Some((d, qc));
                }
            }
        });
        if blocked {
            return None;
        }
        let (d, q) = nearest?;
        Some([(q[0] - p[0]) / d.max(1e-12), (q[1] - p[1]) / d.max(1e-12)])
    }

    fn mark_dead(&mut self, p: Point, cells: i64) {
        let layer = self.layer.as_mut().unwrap();
        let Some((i, j)) = layer.g.cell(p) else {
            return;
        };
        for dj in -cells..=cells {
            for di in -cells..=cells {
                let (ii, jj) = (i as i64 + di, j as i64 + dj);
                if ii >= 0 && jj >= 0 && (ii as usize) < layer.g.nx && (jj as usize) < layer.g.ny {
                    let k = layer.g.idx(ii as usize, jj as usize);
                    layer.dead[k] = true;
                }
            }
        }
    }

    /// Follow the material front from `start`. Returns the centre polyline
    /// (first point = start) and whether any material was cut.
    fn follow(&mut self, start: Point, heading: f64, approach: bool) -> (Vec<Point>, bool) {
        let (r, w) = (self.r, self.width);
        let side = if self.climb { 1. } else { -1. };
        let layer = self.layer.as_mut().unwrap();
        let c = layer.g.c;
        let ds = (0.15 * r).min(0.6 * w).max(2. * c);
        let target = w * ds;
        let fmax = 0.6 * PI;
        let mut cells = std::mem::take(&mut self.scratch);
        let mut pts = vec![start];
        let mut p = start;
        let mut th = heading;
        let mut engaged = !approach;
        let mut approached = 0.;
        let mut cut_any = false;
        let mut max_eng: f64 = 0.;
        let mut end = 0;
        for _ in 0..2_000_000 {
            layer.gather(p, r + ds + 2. * c, &mut cells);
            let f = |phi: f64, layer: &Layer| {
                let q = [p[0] + ds * (th + phi).cos(), p[1] + ds * (th + phi).sin()];
                if !layer.allowed_at(q) {
                    return f64::INFINITY;
                }
                area(&cells, q, r, c)
            };
            if !engaged {
                let a0 = f(-side * PI / 2., layer);
                if a0 == 0. && approached < 0.4 * r {
                    p = [
                        p[0] + ds * (th - side * PI / 2.).cos(),
                        p[1] + ds * (th - side * PI / 2.).sin(),
                    ];
                    approached += ds;
                    pts.push(p);
                    continue;
                }
                engaged = true;
            }
            // Material side is -side*fmax; the far side is +side*fmax.
            let into = f(-side * fmax, layer);
            let away = f(side * fmax, layer);
            if into.min(1e9) < 0.25 * target && away < 0.25 * target {
                let ahead = f(0., layer);
                if ahead.is_infinite() || ahead < 0.25 * target {
                    end = 0;
                    break;
                }
            }
            if away > 1.3 * target {
                end = 1;
                break;
            }
            // Turn from the far side towards the material until a step
            // would remove the target area, then bisect that bracket. The
            // area is not monotone in the turn, so scan before bisecting.
            const SCAN: usize = 16;
            let mut prev = (fmax, away);
            let mut best = prev;
            let mut bracket = None;
            for i in 1..=SCAN {
                let u = fmax - 2. * fmax * i as f64 / SCAN as f64;
                let a = f(side * u, layer);
                if a > target {
                    bracket = Some((prev.0, u));
                    break;
                }
                if a > best.1 {
                    best = (u, a);
                }
                prev = (u, a);
            }
            let phi = match bracket {
                Some((mut ok, mut over)) => {
                    for _ in 0..10 {
                        let mid = 0.5 * (ok + over);
                        if f(side * mid, layer) > target {
                            over = mid;
                        } else {
                            ok = mid;
                        }
                    }
                    side * ok
                }
                None => side * best.0,
            };
            let a = f(phi, layer);
            if !a.is_finite() || a < 0.02 * target {
                end = if a.is_finite() { 2 } else { 3 };
                break;
            }
            th += phi;
            p = [p[0] + ds * th.cos(), p[1] + ds * th.sin()];
            max_eng = max_eng.max(a / ds);
            layer.cut(p, r);
            cut_any = true;
            pts.push(p);
        }
        self.scratch = cells;
        if cut_any {
            match end {
                0 => self.report.pass_ends_exhausted += 1,
                1 => self.report.pass_ends_overload += 1,
                2 => self.report.pass_ends_thin += 1,
                _ => self.report.pass_ends_boundary += 1,
            }
            self.report.max_estimated_engagement_mm =
                self.report.max_estimated_engagement_mm.max(max_eng);
        } else {
            pts.truncate(1);
        }
        (pts, cut_any)
    }

    /// Front step length: short against the cutter and the target width.
    fn step_mm(&self, c: f64) -> f64 {
        (0.15 * self.r).min(0.6 * self.width).max(2. * c)
    }
    /// Whether a straight stay-down link keeps the engagement bounded: every
    /// short step may remove at most the target width times its length
    /// (plus one grid cell of slack). Returns the samples to cut.
    fn link_samples(&self, a: Point, b: Point) -> Option<(Vec<Point>, f64)> {
        let layer = self.layer.as_ref().unwrap();
        let c = layer.g.c;
        let r = self.r;
        let len = (b[0] - a[0]).hypot(b[1] - a[1]);
        let n = (len / self.step_mm(c)).ceil().max(1.) as usize;
        let h = len / n as f64;
        let mut cells = vec![];
        let mut samples = Vec::with_capacity(n + 1);
        let mut prev = a;
        let mut widest: f64 = 0.;
        samples.push(a);
        for t in 1..=n {
            let f = t as f64 / n as f64;
            let p = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
            if !layer.allowed_at(p) {
                return None;
            }
            layer.gather(p, r + c, &mut cells);
            let mut new = 0.;
            for q in &cells {
                if (q[0] - prev[0]).hypot(q[1] - prev[1]) > r {
                    new += ((r - (q[0] - p[0]).hypot(q[1] - p[1])) / c + 0.5).clamp(0., 1.);
                }
            }
            new *= c * c;
            if new > self.width * h + 0.5 * c * c {
                return None;
            }
            widest = widest.max(new / h);
            samples.push(p);
            prev = p;
        }
        Some((samples, widest))
    }

    fn certify(&mut self, s: Stroke) -> Result<(), String> {
        if !self.guides.stroke_geometry_fits_radius(s, self.r) {
            return Err(format!(
                "engagement_front planned a stroke from {:?} to {:?} that the exact region check rejects; this is a planner defect, please report it",
                s.a, s.b
            ));
        }
        self.report.certified_strokes += 1;
        Ok(())
    }

    fn queue(&mut self, curve: Curve, kind: MoveKind, feed: Option<f64>) {
        if curve.start() != curve.end() || matches!(curve, Curve::Helix { .. } | Curve::Arc { .. })
        {
            self.pending.push_back((curve, kind, feed));
        }
    }
    fn line(&mut self, a: Vec3, b: Vec3, kind: MoveKind) {
        self.queue(Curve::Line { start: a, end: b }, kind, None);
    }
    /// Lift at feed to the feed height, then rapid to `to_z`; returns the pose.
    fn lift(&mut self, from: Vec3, to_z: f64) -> Vec3 {
        let feed_z = self.heights.feed.max(from[2]);
        let safe = [from[0], from[1], feed_z];
        let up = [from[0], from[1], to_z.max(feed_z)];
        self.line(from, safe, MoveKind::Retract);
        self.line(safe, up, MoveKind::Rapid);
        up
    }
    fn travel(&mut self, from: Vec3, to: Point) -> Vec3 {
        let up = if (from[2] - self.heights.retract).abs() < 1e-12 || from[2] >= self.heights.feed {
            let up = [from[0], from[1], from[2].max(self.heights.retract)];
            self.line(from, up, MoveKind::Rapid);
            up
        } else {
            self.lift(from, self.heights.retract)
        };
        let across = [to[0], to[1], up[2]];
        self.line(up, across, MoveKind::Rapid);
        let above = [to[0], to[1], self.heights.feed];
        self.line(across, above, MoveKind::Rapid);
        above
    }

    fn emit_cuts(&mut self, pts: &[Point]) -> Result<(), String> {
        let c = self.layer.as_ref().unwrap().g.c;
        let z = self.z;
        for s in fit_strokes(pts, c / 10., 1000. * self.r) {
            self.certify(s)?;
            if s.arc.is_some() {
                self.report.arcs += 1;
            }
            self.queue(s.curve(z), MoveKind::Cut, None);
        }
        Ok(())
    }

    /// Plan the next pass (link or entry plus cut) into `pending`. Returns
    /// false when the layer is complete.
    fn plan_pass(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<bool, String> {
        let r = self.r;
        let z = self.z;
        let min_area = 0.02 * r * r;
        let keep_down = 6. * r;
        let mut refreshed = false;
        loop {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            let pos = [self.position[0], self.position[1]];
            // Nearest valid start among the cached candidates.
            let mut found = None;
            loop {
                let layer = self.layer.as_ref().unwrap();
                let best = layer
                    .candidates
                    .iter()
                    .enumerate()
                    .map(|(i, &k)| {
                        let q = layer.g.center(k);
                        (i, k, (q[0] - pos[0]).hypot(q[1] - pos[1]))
                    })
                    .min_by(|a, b| a.2.total_cmp(&b.2));
                let Some((i, k, _)) = best else { break };
                self.layer.as_mut().unwrap().candidates.swap_remove(i);
                if let Some(dir) = self.candidate_direction(k) {
                    found = Some((k, dir));
                    break;
                }
            }
            let Some((k, dir)) = found else {
                if refreshed {
                    return self.helix_pass(stock, cancel);
                }
                let left = self.refresh();
                refreshed = true;
                if left < min_area {
                    return Ok(false);
                }
                continue;
            };
            let q = self.layer.as_ref().unwrap().g.center(k);
            let here = self.position;
            let at_layer = (here[2] - z).abs() < 1e-9;
            let len = (q[0] - here[0]).hypot(q[1] - here[1]);
            let link = if at_layer && len <= keep_down {
                self.link_samples([here[0], here[1]], q).filter(|_| {
                    self.guides
                        .stroke_geometry_fits_radius(Stroke::line([here[0], here[1]], q), r)
                })
            } else {
                None
            };
            if link.is_none() {
                // A vertical entry needs the committed sweeps (or air outside
                // the original stock) to prove the whole column clear.
                let (height, _) = self
                    .guides
                    .cleared_entry_start(q, 0., z, Some(stock), cancel)?;
                if height - z > 0.01_f64.max(64. * self.op.tolerance_mm) {
                    self.report.unproven_air_starts += 1;
                    self.mark_dead(q, 1);
                    continue;
                }
            }
            let side = if self.climb { 1. } else { -1. };
            let heading = (side * dir[0]).atan2(-side * dir[1]);
            let (pts, cut) = self.follow(q, heading, true);
            if !cut {
                self.mark_dead(q, 3);
                continue;
            }
            if let Some((samples, widest)) = link {
                let layer = self.layer.as_mut().unwrap();
                for p in samples {
                    layer.cut(p, r);
                }
                self.report.max_estimated_engagement_mm =
                    self.report.max_estimated_engagement_mm.max(widest);
                self.report.stay_down_links += 1;
                self.report.certified_strokes += 1;
                self.line(here, [q[0], q[1], z], MoveKind::Link);
            } else {
                let above = self.travel(here, q);
                self.line(above, [q[0], q[1], z], MoveKind::Plunge);
                self.report.lifts += 1;
                self.report.stock_edge_entries += 1;
            }
            self.emit_cuts(&pts)?;
            let end = *pts.last().unwrap();
            self.position = [end[0], end[1], z];
            self.report.passes += 1;
            return Ok(true);
        }
    }

    /// Enclosed material: helix down at the most open reachable point.
    fn helix_pass(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<bool, String> {
        let r = self.r;
        let z = self.z;
        loop {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            let layer = self.layer.as_ref().unwrap();
            let c = layer.g.c;
            let need = self.grid_clearance(c);
            let mut best: Option<(f64, usize)> = None;
            for k in 0..layer.top.len() {
                if layer.allowed[k] && !layer.dead[k] && layer.material(k) {
                    let room = layer.room[k] as f64 - need;
                    if best.is_none_or(|(b, _)| room > b) {
                        best = Some((room, k));
                    }
                }
            }
            let Some((room, k)) = best else {
                return Ok(false);
            };
            let q = layer.g.center(k);
            let helix = self.entry.helix().clone();
            let radii = helix.radii(r, r / 2., 0.);
            let Some(rh) = radii.into_iter().find(|&h| h <= room - c && h > 4. * c) else {
                self.mark_dead(q, 1);
                continue;
            };
            if self.entry.policy() == EntryPolicy::StockEdge {
                return Err(format!(
                    "engagement_front: enclosed material near ({:.3}, {:.3}) at Z{z} needs a helical entry; StockEdgeEntry(helix=None) forbids it",
                    q[0], q[1]
                ));
            }
            for half in [0., PI] {
                let a = [q[0] + rh * half.cos(), q[1] + rh * half.sin()];
                let b = [q[0] - rh * half.cos(), q[1] - rh * half.sin()];
                self.certify(Stroke {
                    a,
                    b,
                    arc: Some((q, PI)),
                })?;
            }
            let (top, opposite) = self
                .guides
                .cleared_entry_start(q, rh, z, Some(stock), cancel)?;
            let curves = helix.curves(q, rh, top.max(z), z, !self.climb, opposite)?;
            let start = curves[0].start();
            let here = self.position;
            let above = self.travel(here, [start[0], start[1]]);
            self.line(above, start, MoveKind::Plunge);
            let feed = Some(helix.feed_mm_min.unwrap_or(self.op.plunge_mm_min));
            self.report.entry_turns += curves.len() - 1;
            for curve in curves {
                self.queue(curve, MoveKind::Lead, feed);
            }
            self.layer.as_mut().unwrap().cut(q, r + rh);
            self.report.helical_entries += 1;
            self.report.lifts += 1;
            self.position = [start[0], start[1], z];
            // Continue clearing from the helix: follow the new front.
            let side = if self.climb { 1. } else { -1. };
            let p = [start[0], start[1]];
            let radial = [(p[0] - q[0]) / rh, (p[1] - q[1]) / rh];
            let heading = (side * radial[0]).atan2(-side * radial[1]);
            let (pts, cut) = self.follow(p, heading, false);
            if cut {
                self.emit_cuts(&pts)?;
                let end = *pts.last().unwrap();
                self.position = [end[0], end[1], z];
            }
            self.report.passes += 1;
            return Ok(true);
        }
    }

    pub(crate) fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self.generation.is_some_and(|g| g != stock.generation())
        {
            return Err(
                "commit preceding engagement_front motion to matching stock before planning".into(),
            );
        }
        while self.pending.is_empty() && !self.finished {
            if self.layer.is_none() {
                self.build_layer(stock, cancel)?;
            }
            if !self.plan_pass(stock, cancel)? {
                let here = self.position;
                self.lift(here, self.op.clearance_z_mm);
                self.finished = true;
            }
        }
        let Some((curve, kind, entry_feed)) = self.pending.pop_front() else {
            return Ok(None);
        };
        curve.prepare(self.op.tolerance_mm)?;
        let motion = Motion {
            id: self.next_id,
            operation: self.op.id.clone(),
            tool: self.op.tool.clone(),
            feed_mm_min: match kind {
                MoveKind::Rapid => None,
                MoveKind::Plunge => Some(self.op.plunge_mm_min),
                _ => Some(entry_feed.unwrap_or(self.op.feed_mm_min)),
            },
            kind,
            curve,
        };
        self.position = motion.curve.end();
        self.next_id += 1;
        self.report.motions += 1;
        self.generation = Some(stock.generation() + 1);
        Ok(Some(motion))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edt_matches_brute_force() {
        let g = Grid {
            x0: 0.,
            y0: 0.,
            c: 0.5,
            nx: 9,
            ny: 7,
        };
        let mask: Vec<bool> = (0..63).map(|k| k == 10 || k == 50).collect();
        let d = edt(&g, &mask);
        for (k, got) in d.iter().enumerate() {
            let p = g.center(k);
            let want = [10, 50]
                .iter()
                .map(|&m| {
                    let q = g.center(m);
                    (p[0] - q[0]).hypot(p[1] - q[1])
                })
                .fold(f64::INFINITY, f64::min);
            assert!((got - want).abs() < 1e-9, "{k}: {got} vs {want}");
        }
    }

    #[test]
    fn fitted_strokes_stay_within_tolerance_of_every_point() {
        let pts: Vec<Point> = (0..60)
            .map(|i| {
                let a = i as f64 * 0.05;
                if i < 30 {
                    [3. * a.cos(), 3. * a.sin()]
                } else {
                    [3. * 1.45f64.cos() + (a - 1.45), 3. * 1.45f64.sin()]
                }
            })
            .collect();
        let strokes = fit_strokes(&pts, 0.005, 1000.);
        assert!(strokes.len() < 10, "{}", strokes.len());
        assert!(strokes.iter().any(|s| s.arc.is_some()));
        for p in &pts {
            let near = strokes.iter().any(|s| match s.arc {
                None => {
                    let (d, l) = (
                        [s.b[0] - s.a[0], s.b[1] - s.a[1]],
                        (s.b[0] - s.a[0]).hypot(s.b[1] - s.a[1]),
                    );
                    let t =
                        (((p[0] - s.a[0]) * d[0] + (p[1] - s.a[1]) * d[1]) / (l * l)).clamp(0., 1.);
                    (s.a[0] + t * d[0] - p[0]).hypot(s.a[1] + t * d[1] - p[1]) <= 0.005 + 1e-9
                }
                Some((c, _)) => {
                    ((p[0] - c[0]).hypot(p[1] - c[1]) - (s.a[0] - c[0]).hypot(s.a[1] - c[1])).abs()
                        <= 0.005 + 1e-9
                }
            });
            assert!(near, "{p:?}");
        }
    }
}
