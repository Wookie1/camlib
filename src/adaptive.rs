//! The adaptive pocket operation: its options and wire format. It is planned
//! by the engagement front (`crate::pocket_front`), the planner Roughing's
//! adaptive clearing uses (`crate::roughing::EngagementFrontPlan`).
use crate::pocket::PocketOperation;
use serde::{Deserialize, Serialize};

/// The adaptive clearing planner. One remains: the engagement front, which
/// follows the boundary between cut and uncut stock and proves every cutting
/// stroke within its contact angle. A saved program states it (#190: the
/// orbit planners `disk_front` and `stock_front` were removed).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdaptivePathStrategy {
    #[default]
    EngagementFront,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CuttingSense {
    Climb,
    Conventional,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(try_from = "serde_json::Value", into = "serde_json::Value")]
pub struct AdaptivePocketOperation {
    pub pocket: PocketOperation,
    pub path_strategy: AdaptivePathStrategy,
    pub cutting_sense: Option<CuttingSense>,
    /// An upper limit on the contact angle: below the stepover's own angle,
    /// acos(1 - stepover / r), it narrows the radial width.
    pub max_engagement_deg: f64,
    /// See `RoughingOperation::rapid_descent_clearance_mm`.
    /// The proof is per depth level: a descent into a level below one
    /// already cut is rapid down to this far above what the committed sweeps
    /// prove clear for the whole tool. None: rapid descent stops at `feed_z_mm`.
    pub rapid_descent_clearance_mm: Option<f64>,
    /// See `RoughingOperation::ramp_clearance_mm`.
    pub ramp_clearance_mm: Option<f64>,
    /// See `RoughingOperation::link_feed_mm_min`.
    /// None: every stay-down link feeds at `feed_mm_min`.
    pub link_feed_mm_min: Option<f64>,
    /// With `link_feed_mm_min`: see `RoughingOperation::link_chip_thinning`.
    pub link_chip_thinning: bool,
}
// Keep the public operation wire flat, like Pocket. Explicitly consume strategy
// fields before Pocket's strict unknown-field validator; serde flatten combined
// with deny_unknown_fields would weaken or break that validation.
impl AdaptivePocketOperation {
    pub(crate) fn front_motion_options(&self) -> crate::roughing::FrontMotionOptions {
        crate::roughing::FrontMotionOptions {
            rapid_descent_clearance_mm: self.rapid_descent_clearance_mm,
            // Roughing only for now: a pocket's rapids stay at the retract
            // height.
            travel_clearance_mm: None,
            ramp_clearance_mm: self.ramp_clearance_mm,
            link_feed_mm_min: self.link_feed_mm_min,
            link_chip_thinning: self.link_chip_thinning,
        }
    }
}
impl TryFrom<serde_json::Value> for AdaptivePocketOperation {
    type Error = String;
    fn try_from(value: serde_json::Value) -> Result<Self, String> {
        let serde_json::Value::Object(mut fields) = value else {
            return Err("adaptive operation must be an object".into());
        };
        let path_strategy = serde_json::from_value(
            fields
                .remove("path_strategy")
                .unwrap_or(serde_json::json!("engagement_front")),
        )
        .map_err(|e| format!("path_strategy: {e}"))?;
        let cutting_sense = serde_json::from_value(
            fields
                .remove("cutting_sense")
                .unwrap_or(serde_json::Value::Null),
        )
        .map_err(|e| e.to_string())?;
        let max_engagement_deg = serde_json::from_value(
            fields
                .remove("max_engagement_deg")
                .ok_or("missing max_engagement_deg")?,
        )
        .map_err(|e| e.to_string())?;
        let mut optional = |name: &str| -> Result<Option<f64>, String> {
            serde_json::from_value(fields.remove(name).unwrap_or(serde_json::Value::Null))
                .map_err(|e| format!("{name}: {e}"))
        };
        let rapid_descent_clearance_mm = optional("rapid_descent_clearance_mm")?;
        let ramp_clearance_mm = optional("ramp_clearance_mm")?;
        let link_feed_mm_min = optional("link_feed_mm_min")?;
        let link_chip_thinning = serde_json::from_value(
            fields
                .remove("link_chip_thinning")
                .unwrap_or(serde_json::Value::Bool(false)),
        )
        .map_err(|e| format!("link_chip_thinning: {e}"))?;
        let pocket =
            serde_json::from_value(serde_json::Value::Object(fields)).map_err(|e| e.to_string())?;
        Ok(Self {
            pocket,
            path_strategy,
            cutting_sense,
            max_engagement_deg,
            rapid_descent_clearance_mm,
            ramp_clearance_mm,
            link_feed_mm_min,
            link_chip_thinning,
        })
    }
}
impl From<AdaptivePocketOperation> for serde_json::Value {
    fn from(op: AdaptivePocketOperation) -> Self {
        let mut value = serde_json::to_value(op.pocket).expect("Pocket serialization");
        let fields = value.as_object_mut().expect("Pocket object");
        // Always written: a saved program states which planner made it.
        fields.insert(
            "path_strategy".into(),
            serde_json::to_value(op.path_strategy).unwrap(),
        );
        if let Some(sense) = op.cutting_sense {
            fields.insert("cutting_sense".into(), serde_json::to_value(sense).unwrap());
        }
        fields.insert("max_engagement_deg".into(), op.max_engagement_deg.into());
        // Absent when unset: an operation without them serializes as before.
        for (name, value) in [
            ("rapid_descent_clearance_mm", op.rapid_descent_clearance_mm),
            ("ramp_clearance_mm", op.ramp_clearance_mm),
            ("link_feed_mm_min", op.link_feed_mm_min),
        ] {
            if let Some(v) = value {
                fields.insert(name.into(), v.into());
            }
        }
        if op.link_chip_thinning {
            fields.insert("link_chip_thinning".into(), true.into());
        }
        value
    }
}
