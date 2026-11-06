//! Feed-controlled vertical entry through native committed empty columns.
//! No sampled poses or inferred generation increments establish clearance.
use super::*;
use crate::engagement::EngagementScene;

pub(super) fn none(n: &usize) -> bool {
    *n == 0
}

impl PocketPlan {
    /// Use `cache` for committed-clearance entry queries (shared across plans).
    pub(crate) fn share_entry_scene(&mut self, cache: &EntrySceneCache) {
        self.entry_scene = cache.clone();
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
        if scene.support_count() == 0 {
            return Ok((top, false));
        }
        let height = |point: Point| -> Result<f64, String> {
            let guard = 32. * self.operation.tolerance_mm
                + 4096. * f64::EPSILON * (1. + point[0].abs() + point[1].abs() + top.abs());
            let mut work = 100_000;
            let (mut low, mut high) = (floor, top);
            // A supported flat pocket has cylindrical cutter/neck/shank sections
            // and linear holder tapers. Maximum endpoint radius encloses every
            // section, from its lowest Z through the entire vertical descent.
            for _ in 0..12 {
                if cancel.is_cancelled() {
                    return Err("committed entry search cancelled".into());
                }
                if high - low <= 0.01_f64.max(64. * self.operation.tolerance_mm) {
                    break;
                }
                let middle = (low + high) / 2.;
                let mut clear = true;
                for &(radius, bottom) in &self.air_entry_parts {
                    let height = middle + bottom - guard;
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
                        Ok(false) => {
                            clear = false;
                            break;
                        }
                        Err(_) if work == 0 && !cancel.is_cancelled() => return Ok(high),
                        Err(e) => return Err(e),
                    }
                }
                if clear {
                    high = middle;
                } else {
                    low = middle;
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
        if positive - floor <= 0.01_f64.max(64. * self.operation.tolerance_mm) {
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
