//! Point-to-source witnesses supply exclusion constraints, never clearance.
//! Each point is within error_mm of some point in the intended closed target.
use super::*;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContactWitnesses {
    pub points_mm: Vec<[f64; 3]>,
    pub error_mm: f64,
    #[serde(default)]
    pub source: serde_json::Value,
}

pub(super) struct PreparedContacts {
    points: Vec<[f64; 3]>,
    nodes: Vec<Node>,
    error: f64,
}
struct Node {
    bounds: [f64; 4],
    max_z: f64,
    range: std::ops::Range<usize>,
    children: Option<[usize; 2]>,
}
impl ContactWitnesses {
    pub(super) fn prepare(&self, cancel: &impl CancelSignal) -> Result<PreparedContacts, String> {
        if self.points_mm.is_empty()
            || self.points_mm.len() > 65536
            || !self.error_mm.is_finite()
            || !(0. ..=0.1).contains(&self.error_mm)
            || self
                .points_mm
                .iter()
                .flatten()
                .any(|x| !x.is_finite() || x.abs() > 1e6)
        {
            return Err("invalid material contact witness points/error/budget".into());
        }
        let mut result = PreparedContacts {
            points: self.points_mm.clone(),
            nodes: vec![],
            error: self.error_mm,
        };
        // For one XY location, the highest target point dominates every lower
        // point in a vertical ball approach. Keep exact coordinates; never merge
        // nearby XY positions or discard the configured positional error.
        result.points.sort_unstable_by(|a, b| {
            a[0].total_cmp(&b[0])
                .then_with(|| a[1].total_cmp(&b[1]))
                .then_with(|| b[2].total_cmp(&a[2]))
        });
        result.points.dedup_by(|a, b| a[0] == b[0] && a[1] == b[1]);
        result.build(0..result.points.len(), cancel)?;
        Ok(result)
    }
}
impl PreparedContacts {
    fn build(
        &mut self,
        range: std::ops::Range<usize>,
        cancel: &impl CancelSignal,
    ) -> Result<usize, String> {
        cancelled(cancel)?;
        let mut bounds = [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ];
        let mut max_z = f64::NEG_INFINITY;
        for p in &self.points[range.clone()] {
            for a in 0..2 {
                bounds[a] = bounds[a].min(p[a]);
                bounds[a + 2] = bounds[a + 2].max(p[a]);
            }
            max_z = max_z.max(p[2]);
        }
        let index = self.nodes.len();
        self.nodes.push(Node {
            bounds,
            max_z,
            range: range.clone(),
            children: None,
        });
        if range.len() > 4 {
            let axis = usize::from(bounds[3] - bounds[1] > bounds[2] - bounds[0]);
            let middle = range.start + range.len() / 2;
            self.points[range.clone()]
                .select_nth_unstable_by(range.len() / 2, |a, b| a[axis].total_cmp(&b[axis]));
            let left = self.build(range.start..middle, cancel)?;
            let right = self.build(middle..range.end, cancel)?;
            self.nodes[index].children = Some([left, right]);
        }
        Ok(index)
    }

    /// Lower ball-center height for EVERY XY center in a rectangle. Use the
    /// farthest corner from each fixed point, shrinking only by point error.
    /// Caller subtracts the ORIGINAL cutter radius for a tip-height bound.
    /// This proves exclusion, never point-set coverage or clearance.
    pub(super) fn lower_height(
        &self,
        rectangle: [f64; 4],
        radius: f64,
        mut lower: f64,
        budget: (&mut u64, u64),
        guard: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<f64>, String> {
        let (work, limit) = budget;
        let center = [
            (rectangle[0] + rectangle[2]) * 0.5,
            (rectangle[1] + rectangle[3]) * 0.5,
        ];
        let half = [
            (rectangle[2] - rectangle[0]) * 0.5,
            (rectangle[3] - rectangle[1]) * 0.5,
        ];
        let radius = radius - self.error - guard;
        if radius <= 0. {
            return Ok(Some(lower));
        }
        let mut stack = vec![0];
        while let Some(i) = stack.pop() {
            cancelled(cancel)?;
            if *work >= limit {
                return Ok(None);
            }
            *work += 1;
            let node = &self.nodes[i];
            if node.max_z + radius <= lower {
                continue;
            }
            let dx = (node.bounds[0] - center[0])
                .max(center[0] - node.bounds[2])
                .max(0.);
            let dy = (node.bounds[1] - center[1])
                .max(center[1] - node.bounds[3])
                .max(0.);
            // Minimum, over this node's point box, of the farthest rectangle
            // corner distance. Including rectangle half-widths is much tighter
            // than the center-only upper bound for this minimax query.
            let distance = (dx + half[0]).hypot(dy + half[1]);
            if distance >= radius
                || node.max_z + ((radius - distance) * (radius + distance)).sqrt() <= lower
            {
                continue;
            }
            if let Some([a, b]) = node.children {
                let priority = |index: usize| {
                    let n = &self.nodes[index];
                    let dx = (n.bounds[0] + n.bounds[2]) * 0.5 - center[0];
                    let dy = (n.bounds[1] + n.bounds[3]) * 0.5 - center[1];
                    (n.max_z, -(dx * dx + dy * dy))
                };
                if priority(a) > priority(b) {
                    stack.extend([b, a]);
                } else {
                    stack.extend([a, b]);
                }
            } else {
                for p in &self.points[node.range.clone()] {
                    if *work >= limit {
                        return Ok(None);
                    }
                    *work += 1;
                    let distance = (p[0] - rectangle[0])
                        .abs()
                        .max((p[0] - rectangle[2]).abs())
                        .hypot((p[1] - rectangle[1]).abs().max((p[1] - rectangle[3]).abs()))
                        + guard;
                    if distance < radius {
                        lower = lower
                            .max(p[2] + ((radius - distance) * (radius + distance)).sqrt() - guard);
                    }
                }
            }
        }
        Ok(Some(lower))
    }
}

/// Revision-local memoization on a common center-space partition. A union of
/// complete grid rectangles covers each stock cell's possible cutter centers;
/// the minimum of their exclusion heights therefore bounds the entire domain.
/// No geometric work is silently skipped when the cache fills.
pub(super) struct ContactGrid<'a> {
    source: &'a PreparedContacts,
    step: f64,
    radius: f64,
    baseline: f64,
    guard: f64,
    cells: std::collections::HashMap<(i64, i64), f64>,
}
impl<'a> ContactGrid<'a> {
    pub(super) fn new(source: &'a PreparedContacts, scope: FinishingScope, guard: f64) -> Self {
        Self {
            source,
            step: 2. * scope.tool_radius_mm / f64::from(1u32 << scope.max_refinement_depth),
            radius: scope.tool_radius_mm + scope.stock_to_leave_mm,
            baseline: scope.floor_z_mm + scope.tool_radius_mm,
            guard,
            cells: std::collections::HashMap::new(),
        }
    }
    pub(super) fn lower_height(
        &mut self,
        rectangle: [f64; 4],
        work: &mut u64,
        limit: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<f64>, String> {
        let lo = [
            ((rectangle[0] - self.guard) / self.step).floor() as i64,
            ((rectangle[1] - self.guard) / self.step).floor() as i64,
        ];
        let hi = [
            ((rectangle[2] + self.guard) / self.step).floor() as i64,
            ((rectangle[3] + self.guard) / self.step).floor() as i64,
        ];
        let mut minimum = f64::INFINITY;
        for y in lo[1]..=hi[1] {
            cancelled(cancel)?;
            for x in lo[0]..=hi[0] {
                let height = if let Some(value) = self.cells.get(&(x, y)) {
                    *value
                } else {
                    let r = [
                        x as f64 * self.step - self.guard,
                        y as f64 * self.step - self.guard,
                        (x + 1) as f64 * self.step + self.guard,
                        (y + 1) as f64 * self.step + self.guard,
                    ];
                    let Some(value) = self.source.lower_height(
                        r,
                        self.radius,
                        self.baseline,
                        (work, limit),
                        self.guard,
                        cancel,
                    )?
                    else {
                        return Ok(None);
                    };
                    if self.cells.len() < 262144 {
                        self.cells.insert((x, y), value);
                    }
                    value
                };
                minimum = minimum.min(height);
                if minimum <= self.baseline {
                    return Ok(Some(self.baseline));
                }
            }
        }
        Ok(Some(minimum))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;

    #[test]
    fn rectangle_excludes_every_center_despite_point_error_and_cache_reuses_work() {
        let cancel = AtomicBool::new(false);
        let error = 0.02;
        let points: Vec<_> = (-5..=5)
            .flat_map(|i| {
                (-5..=5).map(move |j| {
                    [
                        f64::from(i) * 0.2,
                        f64::from(j) * 0.2,
                        2. + f64::from(i * j) * 0.01,
                    ]
                })
            })
            .collect();
        let prepared = ContactWitnesses {
            points_mm: points.clone(),
            error_mm: error,
            source: serde_json::Value::Null,
        }
        .prepare(&cancel)
        .unwrap();
        let radius = 1.5;
        let height = prepared
            .lower_height(
                [-0.2, -0.1, 0.2, 0.1],
                radius,
                0.,
                (&mut 0, 10000),
                1e-10,
                &cancel,
            )
            .unwrap()
            .unwrap();
        assert!(height > 3.);
        // Direct Euclidean intersection, independent of sqrt height querying:
        // an entire point-error ball is inside the cutter at every sampled
        // center. The full-domain proof uses the exact farthest corner.
        for i in -10..=10 {
            for j in -10..=10 {
                let center = [f64::from(i) * 0.02, f64::from(j) * 0.01, height - 1e-8];
                assert!(points.iter().any(|p| {
                    (p[0] - center[0])
                        .hypot(p[1] - center[1])
                        .hypot(p[2] - center[2])
                        + error
                        < radius
                }));
            }
        }
        assert!(
            prepared
                .lower_height([0.; 4], radius, 0., (&mut 0, 0), 1e-10, &cancel)
                .unwrap()
                .is_none()
        );
        assert!(
            prepared
                .lower_height(
                    [0.; 4],
                    radius,
                    0.,
                    (&mut 0, 10000),
                    1e-10,
                    &AtomicBool::new(true)
                )
                .is_err()
        );
        let mut grid = ContactGrid::new(&prepared, FinishingScope::default(), 1e-10);
        let mut work = 0;
        let first = grid
            .lower_height([-0.2, -0.1, 0.2, 0.1], &mut work, 10000, &cancel)
            .unwrap()
            .unwrap();
        let charged = work;
        let again = grid
            .lower_height([-0.2, -0.1, 0.2, 0.1], &mut work, 10000, &cancel)
            .unwrap()
            .unwrap();
        assert_eq!(first, again);
        assert_eq!(work, charged);
        assert!(first <= height);
        assert!(
            grid.lower_height(
                [-0.2, -0.1, 0.2, 0.1],
                &mut work,
                10000,
                &AtomicBool::new(true)
            )
            .is_err()
        );
    }
}
