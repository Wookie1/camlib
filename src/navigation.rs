//! Shared source-admitted guide connectivity/traversal, factored from adaptive.
use crate::{
    Point,
    pocket::{PocketPlan, Stroke},
    stock::CancelSignal,
};
use std::collections::HashMap;
mod pairs;
/// Above this many candidate pairs, pairs are searched lazily (performance
/// threshold, not a limit).
const MAX_PAIRS: usize = 2_000_000;
fn distance(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
struct Groups {
    parent: Vec<usize>,
    count: usize,
}
impl Groups {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            count: n,
        }
    }
    fn root(&mut self, mut a: usize) -> usize {
        while self.parent[a] != a {
            self.parent[a] = self.parent[self.parent[a]];
            a = self.parent[a];
        }
        a
    }
    fn join(&mut self, a: usize, b: usize) -> bool {
        let a = self.root(a);
        let b = self.root(b);
        if a == b {
            false
        } else {
            self.parent[b] = a;
            self.count -= 1;
            true
        }
    }
}
#[derive(Clone, Copy)]
pub(crate) struct Guide {
    pub(crate) stroke: Stroke,
    pub(crate) radius: f64,
    pub(crate) coverage: f64,
    pub(crate) edge: usize,
}
pub(crate) struct GuideGraph {
    pub(crate) tours: Vec<Vec<Guide>>,
    pub(crate) links: usize,
    pub(crate) work: usize,
}
pub(crate) struct GraphOptions {
    pub bridge_radius: f64,
    pub cutter_radius: f64,
    pub coverage: f64,
    pub local_sizing: bool,
    pub starting_work: usize,
}
pub(crate) fn graph(
    guides: &PocketPlan,
    originals: Vec<(Stroke, f64, f64)>,
    options: GraphOptions,
    cancel: &impl CancelSignal,
) -> Result<GuideGraph, String> {
    let GraphOptions {
        bridge_radius,
        cutter_radius,
        coverage,
        local_sizing,
        starting_work,
    } = options;
    let mut nodes = vec![];
    let mut node_radii: Vec<f64> = vec![];
    let mut ids = HashMap::new();
    let mut edges = vec![];
    let mut endpoints = vec![];
    for (s, radius, coverage) in originals {
        if cancel.is_cancelled() {
            return Err("adaptive navigation cancelled".into());
        }
        let mut pair = [0; 2];
        for (k, p) in [s.a, s.b].into_iter().enumerate() {
            let key = p.map(|x| if x == 0. { 0 } else { x.to_bits() });
            pair[k] = *ids.entry(key).or_insert_with(|| {
                nodes.push(p);
                node_radii.push(radius);
                nodes.len() - 1
            });
            node_radii[pair[k]] = node_radii[pair[k]].max(radius);
        }
        edges.push(Guide {
            stroke: s,
            radius,
            coverage,
            edge: edges.len(),
        });
        endpoints.push(pair);
    }
    let mut groups = Groups::new(nodes.len());
    for &[a, b] in &endpoints {
        groups.join(a, b);
    }
    let pairs = nodes.len().saturating_mul(nodes.len().saturating_sub(1)) / 2;
    let mut lazy = if pairs > MAX_PAIRS && groups.count > guides.report().center_components {
        Some(pairs::Pairs::new(&nodes, &node_radii, &mut groups, cancel)?)
    } else {
        None
    };
    let mut candidates = vec![];
    if lazy.is_none() && groups.count > guides.report().center_components {
        candidates = Vec::with_capacity(pairs);
        for a in 0..nodes.len() {
            for b in a + 1..nodes.len() {
                if groups.root(a) != groups.root(b) {
                    candidates.push((
                        node_radii[a].min(node_radii[b]),
                        distance(nodes[a], nodes[b]),
                        a,
                        b,
                    ));
                }
            }
        }
        candidates.sort_unstable_by(|a, b| {
            b.0.total_cmp(&a.0)
                .then(a.1.total_cmp(&b.1))
                .then(a.2.cmp(&b.2))
                .then(a.3.cmp(&b.3))
        });
    }
    let mut candidates = candidates.into_iter();
    let (mut links, mut work) = (0, starting_work);
    loop {
        if groups.count <= guides.report().center_components {
            break;
        }
        let candidate = if let Some(search) = &mut lazy {
            search.next(&mut groups, cancel)?
        } else {
            candidates.next()
        };
        let Some((preferred, _, a, b)) = candidate else {
            break;
        };
        if cancel.is_cancelled() {
            return Err("adaptive navigation cancelled".into());
        }
        if groups.count <= guides.report().center_components {
            break;
        }
        if groups.root(a) == groups.root(b) {
            continue;
        }
        work += guides.link_work_per_query(); // source sign query plus all source edge distances
        if guides.link_geometry_fits(nodes[a], nodes[b]) {
            let mut radius = bridge_radius;
            if preferred > bridge_radius {
                work += guides.link_work_per_query();
                if local_sizing {
                    if let Some((lower, _)) =
                        guides.guide_radius_bounds(Stroke::line(nodes[a], nodes[b]))
                    {
                        radius = (lower - cutter_radius).min(preferred).max(bridge_radius);
                    }
                } else if guides.link_geometry_fits_radius(
                    nodes[a],
                    nodes[b],
                    preferred + cutter_radius,
                ) {
                    radius = preferred;
                }
            }
            groups.join(a, b);
            edges.push(Guide {
                stroke: Stroke::line(nodes[a], nodes[b]),
                radius,
                coverage,
                edge: edges.len(),
            });
            endpoints.push([a, b]);
            links += 1;
        }
    }
    let mut adjacent = vec![vec![]; nodes.len()];
    for (i, &[a, b]) in endpoints.iter().enumerate() {
        adjacent[a].push((i, false, b));
        adjacent[b].push((i, true, a));
    }
    let mut used = vec![false; edges.len()];
    let mut tours = vec![];
    for root in 0..nodes.len() {
        if adjacent[root].iter().all(|(e, _, _)| used[*e]) {
            continue;
        }
        let mut tour = vec![];
        let mut stack = vec![(root, 0, None::<Guide>)];
        while let Some((node, index, _)) = stack.last_mut() {
            if cancel.is_cancelled() {
                return Err("adaptive traversal cancelled".into());
            }
            if *index == adjacent[*node].len() {
                let (_, _, incoming) = stack.pop().unwrap();
                if let Some(s) = incoming {
                    tour.push(Guide {
                        stroke: s.stroke.reversed(),
                        ..s
                    });
                }
                continue;
            }
            let (edge, reverse, next) = adjacent[*node][*index];
            *index += 1;
            if used[edge] {
                continue;
            }
            used[edge] = true;
            let s = if reverse {
                Guide {
                    stroke: edges[edge].stroke.reversed(),
                    ..edges[edge]
                }
            } else {
                edges[edge]
            };
            tour.push(s);
            stack.push((next, 0, Some(s)));
        }
        if !tour.is_empty() {
            tours.push(tour);
        }
    }
    Ok(GuideGraph { tours, links, work })
}
