//! Conservative ball contact heights for material access. The cached support is
//! only a lower witness; every skipped subtree contributes its whole upper bound.
use super::*;

pub(crate) struct ContactEnclosure<'a> {
    surface: &'a PreparedSurface,
    witness: Option<Support>,
    minimum_center_z: f64,
}
impl<'a> ContactEnclosure<'a> {
    pub(crate) fn new(surface: &'a PreparedSurface, minimum_center_z: f64) -> Self {
        Self {
            surface,
            minimum_center_z,
            witness: None,
        }
    }
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn bounds(
        &mut self,
        xy: [f64; 2],
        radius: f64,
        accuracy: f64,
        work: &mut u64,
        limit: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<[f64; 2]>, String> {
        let surface = self.surface;
        let root = &surface.nodes[0];
        let guard = 4096.
            * f64::EPSILON
            * (1.
                + radius
                + self.minimum_center_z.abs()
                + surface.floor.abs()
                + xy[0].abs()
                + xy[1].abs()
                + root
                    .min
                    .iter()
                    .chain(&root.max)
                    .map(|x| x.abs())
                    .sum::<f64>());
        let charge = |work: &mut u64| -> Result<bool, String> {
            if cancel.is_cancelled() {
                return Err("surface contact enclosure cancelled".into());
            }
            if *work >= limit {
                return Ok(false);
            }
            *work += 1;
            Ok(true)
        };
        let bound = |node: &Node| {
            // Outward-expanded XY and radius avoid a rounded distance rejecting
            // a grazing subtree. Its highest Z bounds every downward prism.
            let d2: f64 = (0..2)
                .map(|k| {
                    (node.min[k] - guard - xy[k])
                        .max(0.)
                        .max(xy[k] - node.max[k] - guard)
                        .powi(2)
                })
                .sum();
            let r = radius + guard;
            if d2 > r * r {
                f64::NEG_INFINITY
            } else {
                let mut upper = node.max[2] + (r * r - d2).max(0.).sqrt() + guard;
                if let Some(plane) = &node.contact_plane
                    && plane.normal[2] > 1e-6
                {
                    // This halfspace encloses ALL vertices and downward prisms
                    // of the node. Offset by the full ball radius, then solve
                    // the halfspace boundary for center Z; no plane fitting or
                    // assumed coplanarity is involved.
                    let plane_upper = plane.origin[2]
                        + (plane.offset + radius * plane.length_upper
                            - plane.normal[0] * (xy[0] - plane.origin[0])
                            - plane.normal[1] * (xy[1] - plane.origin[1])
                            + guard)
                            / plane.normal[2]
                        + guard;
                    upper = upper.min(plane_upper);
                }
                upper
            }
        };
        let mut best = Contact {
            center_z: (surface.floor + radius).max(self.minimum_center_z),
            support: Support::Base,
        };
        if let Some(witness) = self.witness {
            if !charge(work)? {
                return Ok(None);
            }
            match witness {
                Support::Base => (),
                Support::Face { index, .. } => {
                    surface.triangles[index].raise_contact(index, xy, radius, &mut best);
                }
                Support::Point(q) => {
                    let d2 = (xy[0] - q[0]).powi(2) + (xy[1] - q[1]).powi(2);
                    if d2 <= radius * radius {
                        let z = q[2] + (radius * radius - d2).max(0.).sqrt();
                        if z > best.center_z {
                            best = Contact {
                                center_z: z,
                                support: witness,
                            };
                        }
                    }
                }
            }
        }
        let mut excluded_upper = f64::NEG_INFINITY;
        let mut stack = vec![0];
        while let Some(i) = stack.pop() {
            if !charge(work)? {
                return Ok(None);
            }
            let node = &surface.nodes[i];
            let upper = bound(node);
            if upper <= best.center_z + accuracy {
                // Keeping the upper, rather than returning the lower witness,
                // preserves clearance even when refinement stops early.
                excluded_upper = excluded_upper.max(upper);
                continue;
            }
            if let Some([left, right]) = node.children {
                if bound(&surface.nodes[left]) > bound(&surface.nodes[right]) {
                    stack.extend([right, left]);
                } else {
                    stack.extend([left, right]);
                }
            } else {
                for ti in node.range.clone() {
                    if !charge(work)? {
                        return Ok(None);
                    }
                    surface.triangles[ti].raise_contact(ti, xy, radius, &mut best);
                }
            }
        }
        self.witness = Some(best.support);
        Ok(Some([
            best.center_z - guard,
            excluded_upper.max(best.center_z + guard),
        ]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn prepare(shift: Vec3, slope: f64) -> PreparedSurface {
        let p = |x: f64, y: f64| [x + shift[0], y + shift[1], 3. + slope * x + shift[2]];
        let mut triangles = vec![];
        for x in -12..12 {
            for y in -12..12 {
                let (x, y) = (x as f64, y as f64);
                triangles.extend([
                    [p(x, y), p(x + 1., y), p(x + 1., y + 1.)],
                    [p(x, y), p(x + 1., y + 1.), p(x, y + 1.)],
                ]);
            }
        }
        SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: triangles.into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        }
        .prepare(shift[2] - 20., 1e-5, &|| false)
        .unwrap()
    }
    #[test]
    fn cached_upper_encloses_analytic_planes_and_exact_boundary_contacts() {
        for shift in [[0.; 3], [100000., -200000., 1234.]] {
            for slope in [0., 0.3, -0.7] {
                let surface = prepare(shift, slope);
                let mut cache = ContactEnclosure::new(&surface, surface.floor);
                let mut work = 0;
                // Forward/backward jumps and changes of radius must recheck the
                // current query; an old contact height alone is not reusable.
                for j in 0..250 {
                    let x = if j % 2 == 0 {
                        -14. + j as f64 * 0.11
                    } else {
                        14. - j as f64 * 0.11
                    };
                    let y = (j % 11) as f64 * 0.3 - 1.5;
                    let radius = 0.5 + (j % 5) as f64 * 0.35;
                    let xy = [x + shift[0], y + shift[1]];
                    let interval = cache
                        .bounds(xy, radius, 0.005, &mut work, 2_000_000, &|| false)
                        .unwrap()
                        .unwrap();
                    let [lower, upper] = interval;
                    let exact = surface
                        .contact(xy, radius, &mut SurfaceWork::default(), &|| false)
                        .unwrap()
                        .center_z;
                    assert!(lower <= exact + 1e-8);
                    assert!(upper >= exact - 1e-8, "{upper} < {exact}");
                    assert!(
                        upper - exact <= 0.00501,
                        "loose enclosure {upper} vs {exact}"
                    );
                    if x.abs() + radius < 12. {
                        let analytic =
                            shift[2] + 3. + slope * x + radius * (1. + slope * slope).sqrt();
                        assert!(lower <= analytic + 1e-8);
                        assert!(upper >= analytic - 1e-8);
                        assert!(upper - analytic <= 0.00501);
                    }
                }
            }
        }
    }
    #[test]
    fn enclosure_work_and_cancellation_are_bounded_inside_each_query() {
        let surface = prepare([0.; 3], 0.3);
        let mut cache = ContactEnclosure::new(&surface, surface.floor);
        for limit in [1, 2, 10] {
            let mut work = 0;
            assert!(
                cache
                    .bounds([0., 0.], 1., 0.005, &mut work, limit, &|| false)
                    .unwrap()
                    .is_none()
            );
            assert_eq!(work, limit);
        }
        let polls = std::cell::Cell::new(0);
        let cancel = || {
            polls.set(polls.get() + 1);
            polls.get() >= 7
        };
        assert!(
            cache
                .bounds([0., 0.], 1., 0.005, &mut 0, 10000, &cancel)
                .is_err()
        );
        assert_eq!(polls.get(), 7);
        let result = cache
            .bounds([0., 0.], 1., 0.005, &mut 0, 10000, &|| false)
            .unwrap()
            .unwrap();
        let fresh = ContactEnclosure::new(&surface, surface.floor)
            .bounds([0., 0.], 1., 0.005, &mut 0, 10000, &|| false)
            .unwrap()
            .unwrap();
        assert_eq!(result, fresh);
    }
    #[test]
    fn cached_low_support_cannot_hide_a_spike_and_depth_clamp_is_exact() {
        let flat = prepare([0.; 3], 0.);
        let mut triangles: Vec<_> = flat.triangles.iter().map(|t| t.p).collect();
        triangles.push([[3., 0., 7.], [3., 0., 1.], [3., 1., 1.]]);
        let surface = SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: triangles.into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        }
        .prepare(-20., 1e-5, &|| false)
        .unwrap();
        let mut query = ContactEnclosure::new(&surface, surface.floor);
        for radius in [0.5, 1., 2.] {
            query
                .bounds([-5., 0.], radius, 0.005, &mut 0, 10000, &|| false)
                .unwrap()
                .unwrap();
            for fraction in [0., 0.5, 0.999999, 1.000001, -0.5] {
                let dx = radius * fraction;
                let [lo, hi] = query
                    .bounds([3. + dx, 0.], radius, 0.005, &mut 0, 10000, &|| false)
                    .unwrap()
                    .unwrap();
                let analytic = if dx.abs() <= radius {
                    (7. + (radius * radius - dx * dx).max(0.).sqrt()).max(3. + radius)
                } else {
                    3. + radius
                };
                assert!(
                    lo <= analytic + 1e-8 && hi >= analytic - 1e-8,
                    "{lo}..{hi} excludes {analytic}"
                );
                assert!(hi - lo <= 0.00501);
            }
        }
        // A known operational center-height floor is part of the queried max,
        // not evidence that the mesh has a higher point. One root bound suffices.
        let mut query = ContactEnclosure::new(&surface, 30.);
        let mut work = 0;
        let [lo, hi] = query
            .bounds([3., 0.], 2., 0.005, &mut work, 1, &|| false)
            .unwrap()
            .unwrap();
        assert!(lo <= 30. && hi >= 30. && hi - lo < 1e-6);
        assert_eq!(work, 1);
    }
    #[test]
    fn prepared_contact_window_preserves_every_relevant_contact_and_original_surface() {
        for shift in [[0.; 3], [100000., -200000., 1234.]] {
            let surface = prepare(shift, 0.3);
            let before = surface.triangles.len();
            for minimum in [shift[2] + 6., shift[2] + 30.] {
                let window = surface
                    .contact_window(minimum, 2., &|| false)
                    .unwrap()
                    .unwrap();
                assert!(window.triangles.len() < before);
                for radius in [0.25, 1., 2.] {
                    for i in 0..100 {
                        let xy = [
                            shift[0] - 14. + i as f64 * 0.3,
                            shift[1] + (i % 7) as f64 * 0.4,
                        ];
                        let original = surface
                            .contact(xy, radius, &mut SurfaceWork::default(), &|| false)
                            .unwrap()
                            .center_z
                            .max(minimum);
                        let filtered = window
                            .contact(xy, radius, &mut SurfaceWork::default(), &|| false)
                            .unwrap()
                            .center_z
                            .max(minimum);
                        assert!((original - filtered).abs() < 1e-9);
                    }
                }
            }
            assert_eq!(surface.triangles.len(), before);
            assert!(
                surface
                    .contact_window(shift[2] - 20., 2., &|| false)
                    .unwrap()
                    .is_none()
            );
            assert!(surface.contact_window(shift[2] + 6., 2., &|| true).is_err());
        }
    }
}
