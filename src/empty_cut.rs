//! Bounded whole-sweep emptiness proofs shared by roughing layer planners.
use crate::{
    motion::Motion,
    stock::{CancelSignal, Stock},
};
use serde::Serialize;

#[derive(Debug, Default, Serialize)]
pub struct EmptyCutReport {
    pub candidates: usize,
    pub outside_original_stock: usize,
    pub empty_committed_stock: usize,
    pub query_work: usize,
    pub query_work_exhausted: bool,
    pub navigation_candidates: usize,
    pub empty_links: usize,
}
#[derive(PartialEq)]
enum Proof {
    Outside,
    Committed,
    Unknown,
}
impl EmptyCutReport {
    /// The caller supplies a complete horizontal guide or support-disk circle.
    /// False includes uncertainty/exhaustion, so the original cut is retained.
    pub(crate) fn may_omit(
        &mut self,
        stock: &Stock,
        motion: &Motion,
        radius: f64,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let proof = self.prove(stock, motion, radius, tolerance, cancel)?;
        self.candidates += 1;
        self.outside_original_stock += usize::from(proof == Proof::Outside);
        self.empty_committed_stock += usize::from(proof == Proof::Committed);
        Ok(proof != Proof::Unknown)
    }
    pub(crate) fn may_link(
        &mut self,
        stock: &Stock,
        motion: &Motion,
        radius: f64,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let empty = self.prove(stock, motion, radius, tolerance, cancel)? != Proof::Unknown;
        self.navigation_candidates += 1;
        self.empty_links += usize::from(empty);
        Ok(empty)
    }
    fn prove(
        &mut self,
        stock: &Stock,
        motion: &Motion,
        radius: f64,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Proof, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("empty.prove");
        let outside = stock.sweep_misses_stock_envelope(motion, radius, tolerance, cancel)?;
        if outside {
            return Ok(Proof::Outside);
        }
        let mut remaining = usize::MAX;
        let before = remaining;
        let empty =
            !stock.may_remove_material(motion, radius, 0., tolerance, &mut remaining, cancel)?;
        self.query_work += before - remaining;
        self.query_work_exhausted |= remaining == 0;
        Ok(if empty {
            Proof::Committed
        } else {
            Proof::Unknown
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        motion::{Curve, MoveKind},
        stock::{HeightBounds, StockSpec},
    };
    use std::sync::atomic::AtomicBool;

    #[test]
    fn exhausted_uncertain_or_cancelled_proof_retains_material_and_never_changes_stock() {
        let cancel = AtomicBool::new(false);
        let stock = Stock::new(StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [4., 4., 10.],
            cell_size_mm: 0.5,
            max_cells: 1000,
        })
        .unwrap();
        let m = Motion {
            id: 1,
            operation: "rough".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            curve: Curve::Line {
                start: [1., 1., 5.],
                end: [3., 3., 5.],
            },
            feed_mm_min: Some(600.),
        };
        let before = serde_json::to_value(stock.snapshot()).unwrap();
        let mut p = EmptyCutReport::default();
        assert!(!p.may_omit(&stock, &m, 1., 1e-5, &cancel).unwrap());
        let mut s = stock.snapshot();
        s.cells.fill(HeightBounds {
            lower_mm: 0.,
            upper_mm: 5.,
        });
        let clear = Stock::from_snapshot(s.clone()).unwrap();
        assert!(p.may_omit(&clear, &m, 1., 1e-5, &cancel).unwrap());
        s.cells.last_mut().unwrap().upper_mm = 10.;
        let uncertain = Stock::from_snapshot(s).unwrap();
        assert!(!p.may_omit(&uncertain, &m, 1., 1e-5, &cancel).unwrap());
        // Query work is counted, never capped: a large prior count still
        // allows a proven-empty cut to be omitted.
        p.query_work = 1_000_000_000;
        assert!(p.may_omit(&clear, &m, 1., 1e-5, &cancel).unwrap());
        assert!(!p.query_work_exhausted);
        let counts = serde_json::to_value(&p).unwrap();
        assert!(
            p.may_omit(&clear, &m, 1., 1e-5, &AtomicBool::new(true))
                .is_err()
        );
        assert_eq!(serde_json::to_value(&p).unwrap(), counts);
        assert_eq!(serde_json::to_value(stock.snapshot()).unwrap(), before);
    }
}
