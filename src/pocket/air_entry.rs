//! Feed-controlled vertical entry through native committed empty columns.
//! No sampled poses or inferred generation increments establish clearance.
use super::*;
use crate::engagement::EngagementScene;

pub(super) fn none(n: &usize) -> bool {
    *n == 0
}

impl PocketPlan {
    /// The lowest tip height at `point` above which every whole-cell stock
    /// upper bound lies clear of the tool assembly's sections (each section's
    /// radius plus a guard, over every cell its disk overlaps). The stream's
    /// stock-contact check reads these same bounds, so a rapid that stops
    /// above this height can be certified by it, not only by the exact
    /// committed sweeps. Outside the stock grid nothing is stock.
    pub(crate) fn cell_entry_height(&self, stock: &Stock, point: Point) -> f64 {
        self.cell_sweep_height(stock, point, 0.)
    }

    /// `cell_entry_height` for every tool position within `around` of
    /// `point` (a helix bore of radius `around` about centre `point`): the
    /// lowest tip height above which no whole-cell stock upper bound meets
    /// any tool section anywhere on that disk.
    pub(crate) fn cell_sweep_height(&self, stock: &Stock, point: Point, around: f64) -> f64 {
        let [nx, ny] = stock.dimensions();
        let [wx, wy] = stock.cell_widths_mm();
        let min = stock.spec().min_mm;
        let cells = stock.cells();
        let guard = 32. * self.operation.tolerance_mm;
        let mut height = f64::NEG_INFINITY;
        for &(radius, bottom) in &self.air_entry_parts {
            let reach = radius + around + guard;
            let span = |v: f64, o: f64, w: f64, n: usize| {
                let lo = ((v - reach - o) / w).floor().max(0.);
                let hi = ((v + reach - o) / w).floor().min(n as f64 - 1.);
                (lo as usize, hi)
            };
            let (i0, i1) = span(point[0], min[0], wx, nx);
            let (j0, j1) = span(point[1], min[1], wy, ny);
            if i1 < 0. || j1 < 0. {
                continue;
            }
            for j in j0..=j1 as usize {
                for i in i0..=i1 as usize {
                    let (x0, y0) = (min[0] + i as f64 * wx, min[1] + j as f64 * wy);
                    let dx = (x0 - point[0]).max(point[0] - x0 - wx).max(0.);
                    let dy = (y0 - point[1]).max(point[1] - y0 - wy).max(0.);
                    if dx.hypot(dy) <= reach {
                        height = height.max(cells[j * nx + i].upper_mm - bottom);
                    }
                }
            }
        }
        height
    }
    /// The largest radius of any tool section (cutter, neck, shank, holder).
    pub(crate) fn widest_part_radius(&self) -> f64 {
        self.air_entry_parts
            .iter()
            .map(|&(radius, _)| radius)
            .fold(0., f64::max)
    }
    /// `cell_entry_height` for every tool position in the axis-aligned
    /// rectangle spanned by `a` and `b`: the lowest tip height above which
    /// no whole-cell stock upper bound meets any tool section anywhere in
    /// that rectangle. The rectangle, not just the segment `a`-`b`, so a
    /// horizontal rapid at that height is clear whichever route the
    /// controller takes between its ends: the straight line or either
    /// axis-first dogleg (G0 interpolation is controller-dependent; see
    /// LinuxCNC G0 and FANUC non-linear positioning, parameter LRP).
    pub(crate) fn cell_route_height(&self, stock: &Stock, a: Point, b: Point) -> f64 {
        let [nx, ny] = stock.dimensions();
        let [wx, wy] = stock.cell_widths_mm();
        let min = stock.spec().min_mm;
        let cells = stock.cells();
        let guard = 32. * self.operation.tolerance_mm;
        let (lo, hi) = (
            [a[0].min(b[0]), a[1].min(b[1])],
            [a[0].max(b[0]), a[1].max(b[1])],
        );
        let mut height = f64::NEG_INFINITY;
        for &(radius, bottom) in &self.air_entry_parts {
            let reach = radius + guard;
            let span = |l: f64, h: f64, o: f64, w: f64, n: usize| {
                let first = ((l - reach - o) / w).floor().max(0.);
                let last = ((h + reach - o) / w).floor().min(n as f64 - 1.);
                (first as usize, last)
            };
            let (i0, i1) = span(lo[0], hi[0], min[0], wx, nx);
            let (j0, j1) = span(lo[1], hi[1], min[1], wy, ny);
            if i1 < 0. || j1 < 0. {
                continue;
            }
            for j in j0..=j1 as usize {
                for i in i0..=i1 as usize {
                    let (x0, y0) = (min[0] + i as f64 * wx, min[1] + j as f64 * wy);
                    // Gap between the cell and the rectangle, per axis.
                    let dx = (x0 - hi[0]).max(lo[0] - x0 - wx).max(0.);
                    let dy = (y0 - hi[1]).max(lo[1] - y0 - wy).max(0.);
                    if dx.hypot(dy) <= reach {
                        height = height.max(cells[j * nx + i].upper_mm - bottom);
                    }
                }
            }
        }
        height
    }
    /// `cell_entry_height` for every tool position on the straight paths
    /// `path` (each within `pad` of the true path: a flattened arc's chord
    /// error): the lowest tip height above which no whole-cell stock upper
    /// bound meets any tool section anywhere along them. A cell counts when
    /// its centre is within reach plus its half diagonal of a path, so no
    /// cell a section overlaps is missed.
    pub(crate) fn cell_path_height(&self, stock: &Stock, path: &[(Point, Point)], pad: f64) -> f64 {
        self.air_entry_parts
            .iter()
            .map(|&(radius, bottom)| self.part_path_top(stock, path, radius + pad) - bottom)
            .fold(f64::NEG_INFINITY, f64::max)
    }
    /// The highest whole-cell stock upper bound a disk of `radius` (plus a
    /// guard) can meet anywhere along the straight paths `path`.
    fn part_path_top(&self, stock: &Stock, path: &[(Point, Point)], radius: f64) -> f64 {
        let [nx, ny] = stock.dimensions();
        let [wx, wy] = stock.cell_widths_mm();
        let min = stock.spec().min_mm;
        let cells = stock.cells();
        let reach = radius + 32. * self.operation.tolerance_mm;
        let half = wx.hypot(wy) / 2.;
        let mut top = f64::NEG_INFINITY;
        for &(a, b) in path {
            let d = [b[0] - a[0], b[1] - a[1]];
            let l2 = d[0] * d[0] + d[1] * d[1];
            let span = |lo: f64, hi: f64, o: f64, w: f64, n: usize| {
                let first = ((lo - reach - o) / w).floor().max(0.);
                let last = ((hi + reach - o) / w).floor().min(n as f64 - 1.);
                (first as usize, last)
            };
            let (i0, i1) = span(a[0].min(b[0]), a[0].max(b[0]), min[0], wx, nx);
            let (j0, j1) = span(a[1].min(b[1]), a[1].max(b[1]), min[1], wy, ny);
            if i1 < 0. || j1 < 0. {
                continue;
            }
            for j in j0..=j1 as usize {
                for i in i0..=i1 as usize {
                    let c = [
                        min[0] + (i as f64 + 0.5) * wx,
                        min[1] + (j as f64 + 0.5) * wy,
                    ];
                    let t = if l2 > 0. {
                        (((c[0] - a[0]) * d[0] + (c[1] - a[1]) * d[1]) / l2).clamp(0., 1.)
                    } else {
                        0.
                    };
                    let near = [a[0] + t * d[0], a[1] + t * d[1]];
                    if (c[0] - near[0]).hypot(c[1] - near[1]) <= reach + half {
                        top = top.max(cells[j * nx + i].upper_mm);
                    }
                }
            }
        }
        top
    }
    /// Whether every tool section above the flutes (neck, shank, holder) is
    /// proven clear of stock along the straight stroke `a`-`b` with the tip
    /// at `z`: the section is above the stock, or under no whole-cell stock
    /// bound it could meet, or its whole sweep lies in columns the committed
    /// cuts emptied (`Stock::history_clearance`, #200: a shank as wide as
    /// the cutter where the levels above cut). Unproven is `false`.
    pub(crate) fn clear_above_flutes(
        &self,
        a: Point,
        b: Point,
        z: f64,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let Some(tool) = &self.rest_tool else {
            return Ok(false);
        };
        let flutes = tool.cutter.cutting_length_mm;
        let motion = Motion {
            id: self.next_id,
            operation: self.operation.id.clone(),
            tool: self.operation.tool.clone(),
            kind: MoveKind::Cut,
            curve: Stroke::line(a, b).curve(z),
            feed_mm_min: Some(self.operation.feed_mm_min),
        };
        let mut work = 1_000_000;
        for &(radius, bottom) in &self.air_entry_parts {
            if bottom < flutes - 1e-9
                || z + bottom >= stock.spec().max_mm[2]
                || self.part_path_top(stock, &[(a, b)], radius) <= z + bottom
            {
                continue;
            }
            if !stock.history_clearance(&motion, radius, bottom, &mut work, cancel)? {
                return Ok(false);
            }
        }
        Ok(true)
    }
    /// The lowest tip height, between `floor` and the operation's top, at
    /// which the committed sweeps prove every tool section clear of stock
    /// along the whole of `path` (lines and XY arcs): `cleared_entry_start`
    /// for a path instead of a point. The tool may come straight down to
    /// that height anywhere on the path and travel along it there in air.
    ///
    /// The proof is `Stock::history_clearance` (#200), section by section:
    /// each section's exact sweep along the path lies in columns the
    /// committed cuts emptied, with no margin beyond the path error, so a
    /// path the same cutter has cut before is proven, right up to a wall,
    /// and so is a shank as wide as the cutter. Returns the operation's top
    /// when nothing is proven (out of work or uncertain proves nothing).
    pub(crate) fn cleared_path_start(
        &self,
        path: &[Stroke],
        floor: f64,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<f64, String> {
        let top = self.operation.top_z_mm;
        let skin = crate::entry::skin(self.operation.tolerance_mm);
        let clear = |tip: f64| -> Result<bool, String> {
            let mut work = 1_000_000;
            for &stroke in path {
                let motion = Motion {
                    id: self.next_id,
                    operation: self.operation.id.clone(),
                    tool: self.operation.tool.clone(),
                    kind: MoveKind::Cut,
                    curve: stroke.curve(tip),
                    feed_mm_min: Some(self.operation.feed_mm_min),
                };
                for &(radius, bottom) in &self.air_entry_parts {
                    if tip + bottom > stock.spec().max_mm[2] {
                        continue;
                    }
                    if !stock.history_clearance(&motion, radius, bottom, &mut work, cancel)? {
                        return Ok(false);
                    }
                }
            }
            Ok(true)
        };
        let (mut low, mut high) = (floor.min(top), top);
        // The level's own depth first: a path cut before is clear to it.
        if high - low > skin && clear(low + skin)? {
            return Ok(low + skin);
        }
        // Halve until the bracket is within the skin its callers accept:
        // each step is a proof, and the number of steps follows from the
        // depth instead of a fixed count.
        while high - low > skin {
            if cancel.is_cancelled() {
                return Err("committed entry search cancelled".into());
            }
            let middle = (low + high) / 2.;
            if !(middle > low && middle < high) {
                break;
            }
            if clear(middle)? {
                high = middle;
            } else {
                low = middle;
            }
        }
        Ok(high)
    }
    /// Use `cache` for committed-clearance entry queries (shared across plans).
    pub(crate) fn share_entry_scene(&mut self, cache: &EntrySceneCache) {
        self.entry_scene = cache.clone();
    }
    /// Whether the committed sweeps prove a horizontal stroke `a`-`b` with
    /// the tool tip at `z` clear of stock for every tool section (each
    /// section's radius plus a guard), up to a skin of 0.01 mm (or 64
    /// tolerances) above `z`, the slack a vertical entry is allowed too.
    /// Work exhaustion or any uncertainty proves nothing.
    pub(crate) fn cleared_link(
        &self,
        a: Point,
        b: Point,
        z: f64,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let mut cache = self
            .entry_scene
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let cached = cache.take();
        let (mut scene, mut cursor) = match cached {
            Some((scene, cursor)) if scene.stock_spec() == stock.spec() => (scene, cursor),
            _ => (
                EngagementScene::new_with_cancel(stock.spec().clone(), vec![], cancel)?,
                crate::stock::ClearanceCursor::default(),
            ),
        };
        let (rebuild, sweeps) =
            stock.committed_clearance_since(&mut cursor, self.operation.tolerance_mm, cancel)?;
        if rebuild {
            scene = EngagementScene::new_with_cancel(stock.spec().clone(), sweeps, cancel)?;
        } else {
            for sweep in sweeps {
                scene.append(sweep)?;
            }
        }
        let scene = &cache.insert((scene, cursor)).0;
        let skin = crate::entry::skin(self.operation.tolerance_mm);
        let guard = 32. * self.operation.tolerance_mm
            + 4096.
                * f64::EPSILON
                * (1. + a[0].abs() + a[1].abs() + b[0].abs() + b[1].abs() + z.abs());
        let mut work = 100_000;
        for &(radius, bottom) in &self.air_entry_parts {
            let height = z + skin + bottom - guard;
            if height > stock.spec().max_mm[2] {
                continue;
            }
            match scene.clear_line(
                a,
                b,
                radius + guard,
                height,
                self.operation.tolerance_mm,
                &mut work,
                cancel,
            ) {
                Ok(true) => (),
                Ok(false) => return Ok(false),
                Err(_) if !cancel.is_cancelled() => return Ok(false),
                Err(e) => return Err(e),
            }
        }
        Ok(true)
    }
    pub(crate) fn cleared_entry_start(
        &self,
        center: Point,
        orbit: f64,
        floor: f64,
        stock: Option<&Stock>,
        cancel: &impl CancelSignal,
    ) -> Result<(f64, bool), String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("pocket.committed_entry");
        let top = self.operation.top_z_mm;
        let Some(stock) = stock else {
            return Ok((top, false));
        };
        // Reuse the scene across entries and append only sweeps committed
        // since the last entry; rebuilding it from the whole history at every
        // entry made long jobs quadratic. A failed update drops the cache.
        let mut cache = self
            .entry_scene
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let cached = cache.take();
        let (mut scene, mut cursor) = match cached {
            Some((scene, cursor)) if scene.stock_spec() == stock.spec() => (scene, cursor),
            _ => (
                EngagementScene::new_with_cancel(stock.spec().clone(), vec![], cancel)?,
                crate::stock::ClearanceCursor::default(),
            ),
        };
        let (rebuild, sweeps) =
            stock.committed_clearance_since(&mut cursor, self.operation.tolerance_mm, cancel)?;
        if rebuild {
            scene = EngagementScene::new_with_cancel(stock.spec().clone(), sweeps, cancel)?;
        } else {
            for sweep in sweeps {
                scene.append(sweep)?;
            }
        }
        let scene = &cache.insert((scene, cursor)).0;
        // No early exit without committed sweeps: a column outside the
        // original stock is air on a fresh stock too (#167: every entry
        // from beside the stock was otherwise unproven until something had
        // been cut, so the first pass could only start with a helix).
        let height = |point: Point| -> Result<f64, String> {
            let guard = 32. * self.operation.tolerance_mm
                + 4096. * f64::EPSILON * (1. + point[0].abs() + point[1].abs() + top.abs());
            let mut work = 100_000;
            let (mut low, mut high) = (floor, top);
            // A supported flat pocket has cylindrical cutter/neck/shank sections
            // and linear holder tapers. Maximum endpoint radius encloses every
            // section, from its lowest Z through the entire vertical descent.
            // Halve until the bracket is within the skin the callers accept
            // (`entry::skin`, the one definition both sides use). A fixed 12
            // halvings left (top - floor) / 4096, over that skin once the
            // floor was ~41 mm below the top, so every deep entry was
            // refused (#237). Each step is a proof; the bracket halves, so
            // the loop ends, and a bracket that can no longer be halved (or
            // is not a number) ends it too.
            let skin = crate::entry::skin(self.operation.tolerance_mm);
            // Whether every tool section is proven clear with the tip at
            // `tip`; `None` when the work ran out (nothing is proven then).
            let mut clear_at = |tip: f64| -> Result<Option<bool>, String> {
                for &(radius, bottom) in &self.air_entry_parts {
                    let height = tip + bottom - guard;
                    if height > stock.spec().max_mm[2] {
                        continue;
                    }
                    match scene.clear_disk(
                        point,
                        radius + guard,
                        height,
                        self.operation.tolerance_mm,
                        &mut work,
                        cancel,
                    ) {
                        Ok(true) => (),
                        Ok(false) => return Ok(Some(false)),
                        Err(_) if work == 0 && !cancel.is_cancelled() => return Ok(None),
                        Err(e) => return Err(e),
                    }
                }
                Ok(Some(true))
            };
            // The level itself first: "this start reaches the level" is one
            // proof at one height, the skin above the floor, so the answer
            // does not depend on where the halving below happens to land
            // (the same column was otherwise accepted under one top and
            // refused under another).
            if high - low > skin {
                // The height its callers accept: no more than the skin above
                // the floor after rounding too.
                let mut level = low + skin;
                if level - low > skin {
                    level = level.next_down();
                }
                match clear_at(level)? {
                    Some(true) => return Ok(level),
                    Some(false) => low = level,
                    None => return Ok(high),
                }
            }
            while high - low > skin {
                if cancel.is_cancelled() {
                    return Err("committed entry search cancelled".into());
                }
                let middle = (low + high) / 2.;
                if !(middle > low && middle < high) {
                    break;
                }
                match clear_at(middle)? {
                    Some(true) => high = middle,
                    Some(false) => low = middle,
                    None => return Ok(high),
                }
            }
            // High is either the unchanged authored top or a whole-assembly proof.
            // Stock uncertainty/work exhaustion can only retain more cutting entry.
            Ok(high)
        };
        let positive = height([center[0] + orbit, center[1]])?;
        // The fitted disk admits the complete circle, independent of phase.
        // Its +X extremity may sit on an old sweep's uncertainty band. Try the
        // opposite phase before deciding to repeat a deep cutting descent.
        if positive - floor <= crate::entry::skin(self.operation.tolerance_mm) {
            return Ok((positive, false));
        }
        let negative = height([center[0] - orbit, center[1]])?;
        Ok(if negative < positive - 0.01 {
            (negative, true)
        } else {
            (positive, false)
        })
    }
}

#[cfg(test)]
mod route_height_tests {
    use super::*;
    use crate::stock::{HeightBounds, StockSnapshot};

    /// A floor at Z2 with one pillar to Z80 in the cell at (16, 12), and the
    /// fixture's tool: cutter and shank of radius 2, holder of radius 4 from
    /// 30 mm above the tip.
    fn pillar() -> (PocketPlan, Stock, [f64; 2]) {
        let manifest: crate::stream::StreamManifest = serde_json::from_str(include_str!(
            "../../fixtures/operations/helical-pocket.json"
        ))
        .unwrap();
        let Operation::Pocket(operation) = manifest.operations[0].clone() else {
            panic!("pocket fixture")
        };
        let plan = PocketPlan::prepare(
            operation,
            &manifest.tools[0],
            manifest.stock.height().unwrap(),
            manifest.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let mut spec = manifest.stock.height().unwrap().clone();
        spec.max_mm[2] = 100.;
        let blank = Stock::new(spec.clone()).unwrap();
        let [nx, ny] = blank.dimensions();
        let [wx, wy] = blank.cell_widths_mm();
        let (i, j) = ((16. / wx) as usize, (12. / wy) as usize);
        let mut cells = vec![
            HeightBounds {
                lower_mm: 2.,
                upper_mm: 2.,
            };
            nx * ny
        ];
        cells[j * nx + i] = HeightBounds {
            lower_mm: 80.,
            upper_mm: 80.,
        };
        let stock = Stock::from_snapshot(StockSnapshot {
            spec,
            generation: 0,
            cells,
        })
        .unwrap();
        // The pillar cell's far corner from the origin side.
        (plan, stock, [(i + 1) as f64 * wx, (j + 1) as f64 * wy])
    }

    #[test]
    fn a_route_is_as_high_as_every_tool_section_needs() {
        let (plan, stock, corner) = pillar();
        let radii: Vec<f64> = plan.air_entry_parts.iter().map(|p| p.0).collect();
        let (narrow, wide) = (
            radii.iter().copied().fold(f64::INFINITY, f64::min),
            radii.iter().copied().fold(0., f64::max),
        );
        assert!(
            wide > narrow + 1.,
            "the fixture's holder is wider: {radii:?}"
        );
        // A route parallel to x, `gap` beyond the pillar in y: what each
        // section needs there, from the sections themselves.
        for gap in [0.5 * narrow, (narrow + wide) / 2., wide + 1.] {
            let y = corner[1] + gap;
            let (a, b) = ([4., y], [28., y + 0.3]);
            let needed = plan
                .air_entry_parts
                .iter()
                .map(|&(radius, bottom)| if gap <= radius { 80. } else { 2. } - bottom)
                .fold(f64::NEG_INFINITY, f64::max);
            let height = plan.cell_route_height(&stock, a, b);
            assert!(
                (height - needed).abs() < 1e-9,
                "gap {gap}: the route is at {height}, the sections need {needed}"
            );
        }
        // Between the cutter's reach and the holder's, it is the holder over
        // the pillar that sets the height, not the cutter over the floor.
        let y = corner[1] + (narrow + wide) / 2.;
        assert!(plan.cell_route_height(&stock, [4., y], [28., y]) > 2. + 1.);
        // A dogleg: the pillar inside the rectangle of two ends whose
        // straight line passes well clear of it.
        let height = plan.cell_route_height(
            &stock,
            [corner[0] - 8., corner[1] - 8.],
            [corner[0] + 2., corner[1] + 2.],
        );
        assert!((height - 80.).abs() < 1e-9, "{height}");
    }
}
