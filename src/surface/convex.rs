//! A guarded convex downward solid and its indexed analytic slice support.
use super::*;
use robust::{Coord3D, orient3d};
use std::collections::{BTreeMap, BTreeSet};

fn key(p: Vec3) -> [u64; 3] {
    p.map(|v| if v == 0. { 0 } else { v.to_bits() })
}
fn xy(p: Vec3) -> Coord<f64> {
    Coord { x: p[0], y: p[1] }
}
fn xyz(p: Vec3) -> Coord3D<f64> {
    Coord3D {
        x: p[0],
        y: p[1],
        z: p[2],
    }
}
fn charge(work: &mut SurfaceWork, cancel: &impl CancelSignal) -> Result<(), String> {
    work.nodes += 1;
    work.check(cancel)
}
fn connected(
    graph: &[Vec<usize>],
    work: &mut SurfaceWork,
    cancel: &impl CancelSignal,
) -> Result<bool, String> {
    let mut visited = vec![false; graph.len()];
    let mut stack = vec![0];
    let mut count = 0;
    while let Some(i) = stack.pop() {
        charge(work, cancel)?;
        if visited[i] {
            continue;
        }
        visited[i] = true;
        count += 1;
        stack.extend(graph[i].iter().copied().filter(|&j| !visited[j]));
    }
    Ok(count == graph.len())
}

// A consistently oriented triangulated disk, with injective convex boundary and
// positive projected faces, maps one-to-one onto that boundary's filled polygon.
// Concavity across EVERY interior edge then makes its piecewise affine height
// graph concave throughout the polygon. Its finite downward extrusion is convex.
// No welding, guessed topology or tolerance-based convexification is permitted.
fn certify(
    surface: &PreparedSurface,
    work: &mut SurfaceWork,
    cancel: &impl CancelSignal,
) -> Result<bool, String> {
    let mut points = Vec::new();
    let mut ids = BTreeMap::new();
    let mut xy_ids = BTreeMap::new();
    let mut faces = Vec::with_capacity(surface.triangles.len());
    for triangle in &surface.triangles {
        charge(work, cancel)?;
        let mut p = triangle.p;
        let orientation = orient2d(xy(p[0]), xy(p[1]), xy(p[2]));
        if orientation == 0. {
            return Ok(false);
        }
        if orientation < 0. {
            p.swap(1, 2);
        }
        let mut face = [0; 3];
        for (i, point) in p.into_iter().enumerate() {
            let k = key(point);
            face[i] = *ids.entry(k).or_insert_with(|| {
                points.push(point);
                points.len() - 1
            });
            let previous = xy_ids.entry([k[0], k[1]]).or_insert(face[i]);
            if *previous != face[i] {
                return Ok(false);
            }
        }
        faces.push(face);
    }
    type EdgeUse = (usize, usize, usize, usize);
    let mut edges: BTreeMap<(usize, usize), Vec<EdgeUse>> = BTreeMap::new();
    let mut links = vec![Vec::new(); points.len()];
    for (i, face) in faces.iter().enumerate() {
        for k in 0..3 {
            charge(work, cancel)?;
            let (a, b, c) = (face[k], face[(k + 1) % 3], face[(k + 2) % 3]);
            let uses = edges.entry((a.min(b), a.max(b))).or_default();
            if uses.len() == 2 {
                return Ok(false);
            }
            uses.push((a, b, c, i));
            links[a].push((b, c));
        }
    }
    if points.len() + faces.len() != edges.len() + 1 {
        return Ok(false);
    }
    let mut boundary = BTreeMap::new();
    let mut incoming = BTreeSet::new();
    let mut adjacent = vec![Vec::new(); faces.len()];
    for uses in edges.values() {
        charge(work, cancel)?;
        let &(a, b, c, i) = &uses[0];
        if uses.len() == 1 {
            if boundary.insert(a, b).is_some() || !incoming.insert(b) {
                return Ok(false);
            }
        } else {
            let (d, e, f, j) = uses[1];
            if a != e || b != d {
                return Ok(false);
            }
            if orient3d(
                xyz(points[a]),
                xyz(points[b]),
                xyz(points[c]),
                xyz(points[f]),
            ) < 0.
            {
                return Ok(false);
            }
            adjacent[i].push(j);
            adjacent[j].push(i);
        }
    }
    if boundary.len() < 3 || !connected(&adjacent, work, cancel)? {
        return Ok(false);
    }
    let first = *boundary.first_key_value().unwrap().0;
    let mut ring = Vec::with_capacity(boundary.len());
    let mut current = first;
    loop {
        charge(work, cancel)?;
        if ring.len() >= boundary.len() {
            return Ok(false);
        }
        ring.push(current);
        let Some(&next) = boundary.get(&current) else {
            return Ok(false);
        };
        current = next;
        if current == first {
            break;
        }
    }
    if ring.len() != boundary.len() {
        return Ok(false);
    }
    // Compare the complete boundary with its exact-predicate convex hull. Local
    // turn signs alone would incorrectly admit a self-crossing star polygon.
    let mut sorted = ring.clone();
    sorted.sort_by(|&a, &b| {
        points[a][0]
            .total_cmp(&points[b][0])
            .then(points[a][1].total_cmp(&points[b][1]))
    });
    let chain = |iter: Vec<usize>| {
        let mut out: Vec<usize> = Vec::new();
        for i in iter {
            while out.len() >= 2
                && orient2d(
                    xy(points[out[out.len() - 2]]),
                    xy(points[out[out.len() - 1]]),
                    xy(points[i]),
                ) <= 0.
            {
                out.pop();
            }
            out.push(i);
        }
        out
    };
    let mut hull = chain(sorted.clone());
    sorted.reverse();
    let mut upper = chain(sorted);
    hull.pop();
    upper.pop();
    hull.extend(upper);
    if hull.len() != ring.len() {
        return Ok(false);
    }
    let Some(start) = hull.iter().position(|&i| i == first) else {
        return Ok(false);
    };
    if ring
        .iter()
        .enumerate()
        .any(|(i, &v)| hull[(start + i) % hull.len()] != v)
    {
        return Ok(false);
    }
    // Every vertex link must be a single cycle (interior) or path (boundary).
    // Edge counts/Euler alone would not exclude pinches or nonmanifold vertices.
    for (vertex, pairs) in links.iter().enumerate() {
        charge(work, cancel)?;
        let mut link: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for &(a, b) in pairs {
            charge(work, cancel)?;
            link.entry(a).or_default().push(b);
            link.entry(b).or_default().push(a);
        }
        let ends = link.values().filter(|n| n.len() == 1).count();
        if ends != if boundary.contains_key(&vertex) { 2 } else { 0 }
            || link.values().any(|n| n.is_empty() || n.len() > 2)
        {
            return Ok(false);
        }
        let mut seen = BTreeSet::new();
        let mut stack = vec![*link.first_key_value().unwrap().0];
        while let Some(i) = stack.pop() {
            charge(work, cancel)?;
            if seen.insert(i) {
                stack.extend(link[&i].iter().copied());
            }
        }
        if seen.len() != link.len() {
            return Ok(false);
        }
    }
    work.check(cancel)?;
    Ok(true)
}

impl PreparedSurface {
    pub(crate) fn convex_solid(
        &self,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        work.check(cancel)?;
        if let Some(&value) = self.convex_solid.get() {
            return Ok(value);
        }
        let value = certify(self, work, cancel)?;
        work.check(cancel)?;
        let _ = self.convex_solid.set(value);
        Ok(value)
    }
    pub(crate) fn convex_slice_support(
        &self,
        height: f64,
        radius: f64,
        normal: Point,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Point>, String> {
        if !height.is_finite()
            || height.abs() > 1e6
            || !radius.is_finite()
            || radius <= 0.
            || radius > 1e6
            || height <= self.floor + radius
            || normal.iter().any(|x| !x.is_finite())
            || (normal[0].hypot(normal[1]) - 1.).abs() > 1e-9
        {
            return Err("invalid convex-slice height/radius/unit normal".into());
        }
        if !self.convex_solid(work, cancel)? {
            return Err("source is not a certified convex solid".into());
        }
        let projection = |p: Point| p[0] * normal[0] + p[1] * normal[1];
        let bound = |node: &Node| {
            let reserve = 8192.
                * f64::EPSILON
                * (1.
                    + height.abs()
                    + radius
                    + node
                        .min
                        .iter()
                        .chain(&node.max)
                        .map(|v| v.abs())
                        .sum::<f64>());
            if node.max[2] + radius + reserve < height {
                return f64::NEG_INFINITY;
            }
            let gap = (height - node.max[2]).clamp(0., radius);
            let radial = ((radius - gap) * (radius + gap)).max(0.).sqrt();
            let point = std::array::from_fn(|k| {
                if normal[k] >= 0. {
                    node.max[k]
                } else {
                    node.min[k]
                }
            });
            projection(point) + radial * (normal[0] * normal[0] + normal[1] * normal[1]) + reserve
        };
        let mut best = f64::NEG_INFINITY;
        let mut result = None;
        let mut stack = vec![(0, bound(&self.nodes[0]))];
        while let Some((i, upper)) = stack.pop() {
            charge(work, cancel)?;
            if upper < best || upper == f64::NEG_INFINITY {
                continue;
            }
            let node = &self.nodes[i];
            if let Some([a, b]) = node.children {
                let (x, y) = (bound(&self.nodes[a]), bound(&self.nodes[b]));
                if x < y {
                    stack.extend([(a, x), (b, y)]);
                } else {
                    stack.extend([(b, y), (a, x)]);
                }
            } else {
                for index in node.range.clone() {
                    if let Some(point) =
                        self.slice_support(index, height, radius, normal, work, cancel)?
                    {
                        let value = projection(point);
                        if value > best {
                            best = value;
                            result = Some(point);
                        }
                    }
                }
            }
        }
        work.check(cancel)?;
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    fn prepare(triangles: Vec<[Vec3; 3]>) -> PreparedSurface {
        SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: triangles.into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        }
        .prepare(0., 1e-5, &|| false)
        .unwrap()
    }
    fn fan(center_height: f64) -> Vec<[Vec3; 3]> {
        let ring = [[-2., -2., 1.], [2., -2., 1.], [2., 2., 1.], [-2., 2., 1.]];
        (0..4)
            .map(|i| [[0., 0., center_height], ring[i], ring[(i + 1) % 4]])
            .collect()
    }
    fn certified(triangles: Vec<[Vec3; 3]>) -> bool {
        prepare(triangles)
            .convex_solid(&mut SurfaceWork::default(), &|| false)
            .unwrap()
    }
    #[test]
    fn guard_requires_embedded_convex_disk_and_concave_height() {
        assert!(certified(fan(3.)));
        assert!(certified(fan(1.))); // coplanar interior edges remain admissible
        assert!(!certified(fan(0.5))); // an inward crater cannot be filled
        let mut reversed = fan(3.);
        for triangle in &mut reversed {
            triangle.swap(0, 1);
        }
        assert!(certified(reversed));

        let mut open = fan(3.);
        open.pop();
        assert!(!certified(open));
        let mut duplicate = fan(3.);
        duplicate.push(duplicate[0]);
        assert!(!certified(duplicate));
        let mut disconnected = fan(3.);
        disconnected.extend(
            fan(3.)
                .into_iter()
                .map(|t| t.map(|p| [p[0] + 10., p[1], p[2]])),
        );
        assert!(!certified(disconnected));
        let mut different_z = fan(3.);
        different_z[0][0][2] += 0.01;
        assert!(!certified(different_z));
        assert!(!certified(vec![[[0., 0., 1.], [0., 0., 3.], [1., 0., 1.]]]));

        // The star's same-sign local turns do not certify a simple convex boundary.
        let ring: Vec<_> = (0..5)
            .map(|i| {
                let angle = (2 * i) as f64 * std::f64::consts::TAU / 5.;
                [angle.cos() * 2., angle.sin() * 2., 1.]
            })
            .collect();
        assert!(!certified(
            (0..5)
                .map(|i| [[0., 0., 3.], ring[i], ring[(i + 1) % 5]])
                .collect()
        ));

        // Four rectangles surround a real opening; no convex-hull shortcut may
        // erase the hole even though each individual roof facet is horizontal.
        let mut hole = vec![];
        for [x0, y0, x1, y1] in [
            [0., 0., 4., 1.],
            [0., 3., 4., 4.],
            [0., 1., 1., 3.],
            [3., 1., 4., 3.],
        ] {
            hole.extend([
                [[x0, y0, 1.], [x1, y0, 1.], [x1, y1, 1.]],
                [[x0, y0, 1.], [x1, y1, 1.], [x0, y1, 1.]],
            ]);
        }
        assert!(!certified(hole));
    }

    fn compare_support(surface: &PreparedSurface, heights: &[f64], radius: f64, directions: usize) {
        let mut indexed_work = SurfaceWork::default();
        let mut full_work = SurfaceWork::default();
        assert!(surface.convex_solid(&mut indexed_work, &|| false).unwrap());
        for &height in heights {
            for i in 0..directions {
                let angle = i as f64 * std::f64::consts::TAU / directions as f64;
                let normal = [angle.cos(), angle.sin()];
                let projection = |p: Point| p[0] * normal[0] + p[1] * normal[1];
                let actual = surface
                    .convex_slice_support(height, radius, normal, &mut indexed_work, &|| false)
                    .unwrap();
                let expected = (0..surface.prism_count())
                    .filter_map(|index| {
                        surface
                            .slice_support(index, height, radius, normal, &mut full_work, &|| false)
                            .unwrap()
                    })
                    .map(projection)
                    .max_by(f64::total_cmp);
                match (actual, expected) {
                    (Some(a), Some(b)) => {
                        assert!((projection(a) - b).abs() <= 1e-10 * (1. + b.abs()))
                    }
                    (None, None) => {}
                    _ => panic!("indexed support lost a nonempty prism slice"),
                }
            }
        }
    }
    #[test]
    fn indexed_support_matches_every_original_prism_extremum() {
        compare_support(&prepare(fan(3.)), &[2., 3., 4., 4.01], 1., 64);
        let translated = fan(3.)
            .into_iter()
            .map(|t| t.map(|p| [p[0] + 100_000., p[1] - 100_000., p[2]]))
            .collect();
        compare_support(&prepare(translated), &[2., 3., 4., 4.01], 1., 64);
        let saved: serde_json::Value =
            serde_json::from_str(include_str!("../../fixtures/operations/hemisphere.json"))
                .unwrap();
        let mesh: SurfaceMesh =
            serde_json::from_value(saved["operations"][0]["surface"].clone()).unwrap();
        let surface = mesh.prepare(2., 1e-5, &|| false).unwrap();
        compare_support(&surface, &[4.5, 6., 8.], 2.01, 32);
    }
    #[test]
    fn cancellation_or_exhaustion_does_not_cache_a_partial_certificate() {
        let surface = prepare(fan(3.));
        let calls = Cell::new(0);
        let cancel = || {
            calls.set(calls.get() + 1);
            calls.get() > 8
        };
        assert!(
            surface
                .convex_solid(&mut SurfaceWork::default(), &cancel)
                .is_err()
        );
        assert!(surface.convex_solid.get().is_none());
        let mut exhausted = SurfaceWork {
            nodes: 20_000_000,
            ..Default::default()
        };
        // Work is counted, never capped: a large prior count still completes.
        assert!(surface.convex_solid(&mut exhausted, &|| false).unwrap());
        assert!(
            surface
                .convex_solid(&mut SurfaceWork::default(), &|| false)
                .unwrap()
        );
        assert!(
            surface
                .convex_slice_support(3., 1., [1., 0.], &mut SurfaceWork::default(), &|| true)
                .is_err()
        );
        // Work is counted, never capped.
        assert!(
            surface
                .convex_slice_support(3., 1., [1., 0.], &mut exhausted, &|| false)
                .is_ok()
        );
    }
}
