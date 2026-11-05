//! Clearance, retract and feed heights (the convention of commercial CAM).
//!
//! * clearance: the operation starts and ends here; above everything,
//!   fixtures included (the programmer's responsibility).
//! * retract: horizontal rapid travel between passes inside an operation.
//! * feed: rapid descent stops here; below it the tool moves at feed.
//!
//! Safety is proved, not assumed: both rapid heights must be above the top of
//! the material the operation can meet. Stock only loses material, so a rapid
//! at or descending to a height above that top cannot touch stock.

/// Resolved heights for one operation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Heights {
    pub clearance: f64,
    pub retract: f64,
    pub feed: f64,
}

impl Heights {
    /// `material_top`: the highest stock Z the operation can meet (the stock
    /// top, or a proven lower committed top). Defaults: retract = clearance,
    /// feed = retract, which reproduces clearance-only linking exactly.
    pub(crate) fn resolve(
        clearance: f64,
        retract: Option<f64>,
        feed: Option<f64>,
        material_top: f64,
        guard: f64,
    ) -> Result<Self, String> {
        let retract = retract.unwrap_or(clearance);
        let feed = feed.unwrap_or(retract);
        if !retract.is_finite() || !feed.is_finite() {
            return Err("retract_z_mm/feed_z_mm must be finite".into());
        }
        if feed <= material_top + guard {
            return Err(format!(
                "feed_z_mm {feed} must be above the stock top {material_top}: rapid moves descend to it"
            ));
        }
        if feed > retract {
            return Err(format!(
                "feed_z_mm {feed} must not be above retract_z_mm {retract}"
            ));
        }
        if retract > clearance {
            return Err(format!(
                "retract_z_mm {retract} must not be above clearance_z_mm {clearance}"
            ));
        }
        Ok(Self {
            clearance,
            retract,
            feed,
        })
    }
}

/// Check named requirements in order and report the first that fails as
/// `invalid {what} parameters: {message}`, so an error names the value at fault.
pub(crate) fn require(
    what: &str,
    checks: impl IntoIterator<Item = (bool, String)>,
) -> Result<(), String> {
    for (ok, message) in checks {
        if !ok {
            return Err(format!("invalid {what} parameters: {message}"));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn defaults_reproduce_clearance_and_order_is_enforced() {
        let h = Heights::resolve(20., None, None, 10., 1e-4).unwrap();
        assert_eq!((h.retract, h.feed), (20., 20.));
        let h = Heights::resolve(20., Some(15.), Some(11.), 10., 1e-4).unwrap();
        assert_eq!((h.clearance, h.retract, h.feed), (20., 15., 11.));
        assert!(
            Heights::resolve(20., Some(15.), Some(10.), 10., 1e-4)
                .unwrap_err()
                .contains("stock top")
        );
        assert!(
            Heights::resolve(20., Some(12.), Some(13.), 10., 1e-4)
                .unwrap_err()
                .contains("not be above retract")
        );
        assert!(
            Heights::resolve(20., Some(21.), None, 10., 1e-4)
                .unwrap_err()
                .contains("clearance")
        );
        assert!(Heights::resolve(20., Some(f64::NAN), None, 10., 1e-4).is_err());
    }
}
