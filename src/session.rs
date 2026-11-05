//! One loaded geometry and one prepared probe set. Cutter radius and feed changes
//! reuse signed distances; a new geometry or probe set explicitly replaces them.
use crate::planar::{
    Boolean, Construction, ConstructionOptions, PlanarRegion, PreparedRegion, RegionQuery,
    RegionSummary,
};
use crate::{Clearance, Fixture, Point, geometry::Region};
use serde::Serialize;
use std::sync::Arc;

#[derive(Default, Serialize)]
pub struct Counters {
    pub geometry_preparations: u64,
    pub clearance_batches: u64,
    pub clearance_points: u64,
    pub classifications: u64,
    pub classification_cache_hits: u64,
    pub planar_preparations: u64,
    pub planar_constructions: u64,
}

#[derive(Default)]
pub struct Session {
    region: Option<Region>,
    clearances: Option<Vec<f64>>,
    classified: Option<(f64, Arc<[Clearance]>)>,
    pub counters: Counters,
    feed_mm_min: f64,
    planar: Option<PreparedRegion>,
    verifier: Option<crate::verification::PreparedScene>,
}

impl Session {
    pub fn prepare_verification(
        &mut self,
        scene: &crate::verification::Scene,
        options: crate::verification::Options,
    ) -> Result<(), String> {
        let prepared = scene.prepare(options)?;
        self.verifier = Some(prepared);
        Ok(())
    }

    pub fn verifier(&self) -> Result<&crate::verification::PreparedScene, String> {
        self.verifier
            .as_ref()
            .ok_or_else(|| "prepare a verification scene first".into())
    }

    pub fn load(&mut self, fixture: &Fixture) -> Result<(), String> {
        // Commit only after validation succeeds: bad input cannot poison the session.
        let region = Region::prepare(fixture)?;
        self.region = Some(region);
        self.clearances = None;
        self.classified = None;
        self.counters.geometry_preparations += 1;
        Ok(())
    }

    pub fn probe(&mut self, points: &[Point]) -> Result<(), String> {
        let region = self
            .region
            .as_ref()
            .ok_or("load geometry before preparing probes")?;
        let distances: Vec<_> = points
            .iter()
            .map(|&p| region.clearance(p))
            .collect::<Result<_, _>>()?;
        self.counters.clearance_batches += 1;
        self.counters.clearance_points += points.len() as u64;
        self.clearances = Some(distances);
        self.classified = None;
        Ok(())
    }

    pub fn classify(&mut self, radius_mm: f64) -> Result<Arc<[Clearance]>, String> {
        let region = self
            .region
            .as_ref()
            .ok_or("load geometry before classifying")?;
        // Validate radius even when the probe batch is empty.
        region.classify(0.0, radius_mm)?;
        if let Some((radius, results)) = &self.classified
            && *radius == radius_mm
        {
            self.counters.classification_cache_hits += 1;
            return Ok(Arc::clone(results));
        }
        let distances = self
            .clearances
            .as_ref()
            .ok_or("prepare probes before classifying")?;
        let results: Vec<_> = distances
            .iter()
            .map(|&d| region.classify(d, radius_mm))
            .collect::<Result<_, _>>()?;
        self.counters.classifications += 1;
        let results: Arc<[Clearance]> = results.into();
        self.classified = Some((radius_mm, Arc::clone(&results)));
        Ok(results)
    }

    pub fn set_feed(&mut self, feed_mm_min: f64) -> Result<(), String> {
        if !feed_mm_min.is_finite() || feed_mm_min <= 0.0 {
            return Err("feed_mm_min must be finite and positive".into());
        }
        self.feed_mm_min = feed_mm_min;
        Ok(())
    }

    pub fn prepare_planar(
        &mut self,
        region: &PlanarRegion,
        options: ConstructionOptions,
    ) -> Result<RegionSummary, String> {
        let prepared = region.prepare(options)?;
        let summary = prepared.summary();
        self.planar = Some(prepared);
        self.counters.planar_preparations += 1;
        Ok(summary)
    }
    pub fn planar(&self) -> Result<&PreparedRegion, String> {
        self.planar
            .as_ref()
            .ok_or("prepare a planar region first".into())
    }
    pub fn offset_planar(&mut self, delta_mm: f64) -> Result<Construction, String> {
        let output = self.planar()?.offset(delta_mm)?;
        self.counters.planar_constructions += 1;
        Ok(output)
    }
    pub fn cutter_planar(
        &mut self,
        radius_mm: f64,
        allowance_mm: f64,
    ) -> Result<Construction, String> {
        let output = self.planar()?.cutter_region(radius_mm, allowance_mm)?;
        self.counters.planar_constructions += 1;
        Ok(output)
    }
    pub fn boolean_planar(
        &mut self,
        other: &PlanarRegion,
        operation: Boolean,
        options: ConstructionOptions,
    ) -> Result<Construction, String> {
        let other = other.prepare(options)?;
        let output = self.planar()?.boolean(&other, operation)?;
        self.counters.planar_constructions += 1;
        Ok(output)
    }
    pub fn query_planar(&self, points: &[Point]) -> Result<Vec<RegionQuery>, String> {
        let planar = self.planar()?;
        points.iter().map(|&p| planar.query(p)).collect()
    }
}
