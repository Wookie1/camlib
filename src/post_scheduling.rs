//! Bounded authored/scheduled attribution owned by the checked post transaction.
use crate::{
    dynamics::{self, Segment, Settings},
    motion::{Event, MoveKind, Program},
    post_lowering::Mapping,
};
use serde::Serialize;
use sha2::{Digest, Sha256};

#[derive(Clone, Serialize)]
pub struct StreamTiming {
    pub settings: Settings,
    pub policy_sha256: String,
    pub authored_prefix_sha256: String,
    pub completed_source_events: usize,
    pub completed_source_motions: usize,
    pub limited_source_motions: usize,
    pub decoded_motions: usize,
    pub source_exact_stop_seconds: f64,
    pub decoded_exact_stop_seconds: f64,
    pub scope: &'static str,
}
#[derive(Clone, Serialize)]
pub struct FeedAttribution {
    pub check_index: usize,
    pub event_index: usize,
    pub source_motion_id: u64,
    pub source_fraction_interval: [f64; 2],
    pub authored_feed_mm_min: Option<f64>,
    pub configured_feed_mm_min: Option<f64>,
    pub scheduled_feed_mm_min: Option<f64>,
    pub decoded_feed_mm_min: Option<f64>,
    pub scheduled_source_event_sha256: String,
    /// Only the final interval contributes the full source-block estimate.
    pub completed_source_exact_stop_seconds: Option<f64>,
    pub decoded_exact_stop_seconds: f64,
}
#[derive(Serialize)]
pub struct SchedulingOutput {
    pub summary: StreamTiming,
    pub records: Vec<FeedAttribution>,
}
#[derive(Clone)]
pub(crate) struct Scheduled {
    pub authored: Event,
    pub event: Event,
    pub segment: Option<Segment>,
    pub configured_feed: Option<f64>,
}
#[derive(Clone)]
pub(crate) struct State {
    pub summary: StreamTiming,
    hash: Sha256,
}
fn hash(value: &impl Serialize) -> Result<String, String> {
    Ok(format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(value).map_err(|e| e.to_string())?)
    ))
}
impl State {
    pub fn new(header: &Program, settings: Settings) -> Result<Self, String> {
        settings.validate()?;
        if settings.policy.junction_mode != dynamics::JunctionMode::ExactStop
            || !settings.adjust_feeds
        {
            return Err(
                "stream scheduling requires explicit feed limiting and exact-stop timing".into(),
            );
        }
        let mut prefix = Sha256::new();
        prefix.update(b"camlib-stream-policy-v1\0");
        prefix.update(serde_json::to_vec(header).map_err(|e| e.to_string())?);
        Ok(Self {
            summary: StreamTiming {
                policy_sha256: hash(&settings)?,
                settings,
                authored_prefix_sha256: format!("{:x}", prefix.clone().finalize()),
                completed_source_events: 0,
                completed_source_motions: 0,
                limited_source_motions: 0,
                decoded_motions: 0,
                source_exact_stop_seconds: 0.,
                decoded_exact_stop_seconds: 0.,
                scope: "Committed nominal exact-stop source/decoded motion time only; partial lowered sources contribute source time on completion. Separate scalar tangential/normal limits; excludes jerk, axis acceleration, geometric blending, rapid routing, endpoint correction and control-event overhead. Not physical cycle time.",
            },
            hash: prefix,
        })
    }
    pub fn prepare(
        &self,
        authored: &Event,
        index: usize,
        tolerance: f64,
        traverse: Option<f64>,
    ) -> Result<Scheduled, String> {
        let mut event = authored.clone();
        let mut configured_feed = None;
        let segment = if let Event::Motion { motion } = &mut event {
            if motion.kind.is_rapid() && motion.feed_mm_min.is_none() {
                motion.feed_mm_min = traverse;
            }
            configured_feed = motion.feed_mm_min;
            let segment =
                dynamics::capped_segment(motion, index, tolerance, &self.summary.settings)?;
            if motion.kind == MoveKind::Tap && segment.scheduled_feed_mm_min != motion.feed_mm_min {
                return Err(format!(
                    "event {index}: the tapping feed is synchronized to the spindle; the machine feed limits would change it"
                ));
            }
            motion.feed_mm_min = segment.scheduled_feed_mm_min;
            Some(segment)
        } else {
            None
        };
        Ok(Scheduled {
            authored: authored.clone(),
            event,
            segment,
            configured_feed,
        })
    }
    pub fn complete(&mut self, scheduled: &Scheduled) -> Result<(), String> {
        self.hash
            .update(serde_json::to_vec(&scheduled.authored).map_err(|e| e.to_string())?);
        self.summary.authored_prefix_sha256 = format!("{:x}", self.hash.clone().finalize());
        self.summary.completed_source_events += 1;
        if let Some(s) = &scheduled.segment {
            self.summary.completed_source_motions += 1;
            self.summary.limited_source_motions += usize::from(
                s.original_feed_mm_min.is_some()
                    && s.original_feed_mm_min != s.scheduled_feed_mm_min,
            );
            self.summary.source_exact_stop_seconds += s.seconds;
        }
        Ok(())
    }
    pub fn record(
        &mut self,
        scheduled: &Scheduled,
        mapping: &Mapping,
        check_index: usize,
        decoded: Option<&crate::motion::Motion>,
        tolerance: f64,
    ) -> Result<Option<FeedAttribution>, String> {
        let time = if let Some(m) = decoded {
            dynamics::decoded_seconds(m, tolerance, &self.summary.settings)?
        } else {
            0.
        };
        self.summary.decoded_motions += usize::from(decoded.is_some());
        self.summary.decoded_exact_stop_seconds += time;
        let completed = mapping.source_fraction_interval.is_none_or(|r| r[1] == 1.);
        if completed {
            self.complete(scheduled)?;
        }
        let Event::Motion { motion } = &scheduled.authored else {
            return Ok(None);
        };
        let segment = scheduled.segment.as_ref().unwrap();
        Ok(Some(FeedAttribution {
            check_index,
            event_index: mapping.event_index,
            source_motion_id: motion.id,
            source_fraction_interval: mapping.source_fraction_interval.unwrap(),
            authored_feed_mm_min: motion.feed_mm_min,
            configured_feed_mm_min: scheduled.configured_feed,
            scheduled_feed_mm_min: segment.scheduled_feed_mm_min,
            decoded_feed_mm_min: decoded.and_then(|m| m.feed_mm_min),
            scheduled_source_event_sha256: hash(&scheduled.event)?,
            completed_source_exact_stop_seconds: completed.then_some(segment.seconds),
            decoded_exact_stop_seconds: time,
        }))
    }
}
