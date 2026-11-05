//! Original-source component labels for rejecting impossible guide connections.
//! Labels are never a clearance certificate. Boundary/error-band points and
//! ambiguous winding return None; all retained links still need full admission.
use super::*;
use crate::stock::CancelSignal;

impl PreparedRegion {
    pub(crate) fn interior_component(
        &self,
        point: Point,
        work: &mut usize,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Option<usize>, String> {
        if cancel.is_cancelled() {
            return Err("source component query cancelled".into());
        }
        if !point.iter().all(|x| x.is_finite() && x.abs() <= 1e6) {
            return Err("invalid source component query point".into());
        }
        // With fewer than two components there is no possible exclusion.
        if self.definition.components.len() < 2 {
            return Ok(None);
        }
        let point = Vector2::new(point[0] - self.origin[0], point[1] - self.origin[1]);
        let uncertainty = self.definition.boundary_error_mm + self.options.tolerance_mm;
        let mut winding = vec![0i32; self.definition.components.len()];
        for (boundary, &owner) in self
            .shape
            .ccw_plines
            .iter()
            .chain(&self.shape.cw_plines)
            .zip(&self.boundary_owners)
        {
            if cancel.is_cancelled() {
                return Err("source component query cancelled".into());
            }
            *work = work.saturating_add(2 * boundary.polyline.vertex_count());
            if *work > limit {
                return Err("source component query work limit exceeded".into());
            }
            let distance = boundary
                .polyline
                .closest_point(point, self.epsilon)
                .ok_or("empty source component boundary")?
                .distance;
            if distance <= uncertainty {
                return Ok(None);
            }
            winding[owner] += boundary.polyline.winding_number(point);
        }
        let mut inside = winding
            .iter()
            .enumerate()
            .filter(|(_, w)| **w != 0)
            .map(|(i, _)| i);
        let label = inside.next();
        Ok(if inside.next().is_none() { label } else { None })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn label(region: &PreparedRegion, point: Point) -> Option<usize> {
        region
            .interior_component(point, &mut 0, 100_000, &|| false)
            .unwrap()
    }
    #[test]
    fn nested_curved_components_keep_holes_and_uncertainty_separate() {
        for shift in [0., 100_000.] {
            let mut region = PlanarRegion::circle("outer", [shift, shift], 10.).unwrap();
            region.components[0].holes.push(
                PlanarRegion::circle("hole", [shift, shift], 6.)
                    .unwrap()
                    .components
                    .remove(0)
                    .outer,
            );
            region.components.extend(
                PlanarRegion::circle("inner", [shift, shift], 3.)
                    .unwrap()
                    .components,
            );
            region.boundary_error_mm = 0.01;
            let region = region.prepare(ConstructionOptions::default()).unwrap();
            assert_eq!(label(&region, [shift + 8., shift]), Some(0));
            assert_eq!(label(&region, [shift + 2., shift]), Some(1));
            for x in [11., 5., 6., 6.005, 2.995] {
                assert_eq!(label(&region, [shift + x, shift]), None);
            }
        }
    }
    #[test]
    fn touching_linear_components_do_not_supply_a_positive_radius_passage() {
        let square = |x: f64, y: f64| Component {
            outer: Boundary {
                vertices: vec![
                    [x, y, 0.],
                    [x + 2., y, 0.],
                    [x + 2., y + 2., 0.],
                    [x, y + 2., 0.],
                ],
            },
            holes: vec![],
        };
        let region = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "touch".into(),
            components: vec![square(0., 0.), square(2., 2.)],
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
        }
        .prepare(ConstructionOptions::default())
        .unwrap();
        assert_eq!(label(&region, [1., 1.]), Some(0));
        assert_eq!(label(&region, [3., 3.]), Some(1));
        assert_eq!(label(&region, [2., 2.]), None);
        assert_eq!(label(&region, [1., 3.]), None);
        assert!(
            region
                .interior_component([1., 1.], &mut 0, 0, &|| false)
                .unwrap_err()
                .contains("work")
        );
        assert!(
            region
                .interior_component([1., 1.], &mut 0, 1000, &|| true)
                .unwrap_err()
                .contains("cancelled")
        );
    }
}
