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
    /// Hard upper bound on horizontal clearing and links, including confined
    /// retries at entry feed. Explicit helix and ramp entries are separate.
    pub max_corner_engagement_deg: Option<f64>,
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
/// Fields the removed orbit planners (#190) wrote. Saved programs that chose
/// `engagement_front` still carry them: the earlier serializers wrote
/// `boundary_tolerance_mm` (and, for an adaptive pocket, `clockwise`) for every
/// strategy and the orbit sizes as null, and the engagement front never read
/// them. Such a program is read without them. A program without a
/// `path_strategy` was planned by the old default, disk_front, and is refused:
/// reading it as engagement_front would silently plan a different path.
pub(crate) fn drop_removed_orbit_fields(
    fields: &mut serde_json::Map<String, serde_json::Value>,
    owner: &str,
    extra: &[&str],
) -> Result<(), String> {
    const ORBITS: [&str; 3] = [
        "orbit_radius_mm",
        "min_orbit_radius_mm",
        "local_orbit_tolerance_mm",
    ];
    let present: Vec<&str> = ORBITS
        .iter()
        .chain(["boundary_tolerance_mm"].iter())
        .chain(extra)
        .copied()
        .filter(|name| fields.contains_key(*name))
        .collect();
    let Some(first) = present.first() else {
        return Ok(());
    };
    match fields.get("path_strategy") {
        Some(serde_json::Value::String(s)) if s == "engagement_front" => {}
        Some(_) => return Ok(()), // the strategy itself is refused by name
        None => {
            return Err(format!(
                "{owner}: `{first}` without a path_strategy is a program saved for disk_front, the earlier default planner, which was removed (#190); set path_strategy \"engagement_front\" and remove the orbit fields to plan it with the engagement front"
            ));
        }
    }
    for name in present {
        let value = fields.remove(name).expect("present");
        let ok = if ORBITS.contains(&name) {
            value.is_null()
        } else if name == "boundary_tolerance_mm" {
            value.as_f64().is_some_and(f64::is_finite)
        } else {
            value.is_boolean()
        };
        if !ok {
            return Err(format!(
                "{owner}: {name} {value} belongs to the removed orbit planners (#190) and engagement_front does not use it; remove it"
            ));
        }
    }
    Ok(())
}
impl TryFrom<serde_json::Value> for AdaptivePocketOperation {
    type Error = String;
    fn try_from(value: serde_json::Value) -> Result<Self, String> {
        let serde_json::Value::Object(mut fields) = value else {
            return Err("adaptive operation must be an object".into());
        };
        drop_removed_orbit_fields(&mut fields, "adaptive pocket", &["clockwise"])?;
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
        let max_corner_engagement_deg = optional("max_corner_engagement_deg")?;
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
            max_corner_engagement_deg,
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
            ("max_corner_engagement_deg", op.max_corner_engagement_deg),
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
