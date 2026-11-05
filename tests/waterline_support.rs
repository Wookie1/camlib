use camlib::surface::*;
use camlib::waterline::{ContourMethod, ContourOptions, prepare_contours};
use std::sync::atomic::AtomicBool;
fn options(h: f64) -> ContourOptions {
    ContourOptions {
        method: ContourMethod::Support,
        center_z_mm: h,
        radius_mm: 2.005,
        protected_radius_mm: 2.00016,
        normal_error_mm: 0.01,
        tolerance_mm: 1e-5,
        xy_min_mm: [0., 0.],
        xy_max_mm: [36., 24.],
        simplify: true,
    }
}
fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<[[f64; 3]; 3]> {
    vec![
        [[x0, y0, 8.], [x1, y0, 8.], [x1, y1, 8.]],
        [[x0, y0, 8.], [x1, y1, 8.], [x0, y1, 8.]],
    ]
}
fn contains(rings: &[Vec<[f64; 2]>], p: [f64; 2]) -> bool {
    let mut inside = false;
    for ring in rings {
        for (a, b) in ring
            .iter()
            .zip(ring.iter().cycle().skip(1))
            .take(ring.len())
        {
            if (a[1] > p[1]) != (b[1] > p[1])
                && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
            {
                inside = !inside;
            }
        }
    }
    inside
}
fn mesh(triangles: Vec<[[f64; 3]; 3]>) -> SurfaceMesh {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
        triangles: triangles.into(),
    }
}
#[test]
fn convex_slice_support_matches_flat_and_inclined_analytic_extrema() {
    let cancel = AtomicBool::new(false);
    let mut work = SurfaceWork::default();
    let vertices = [[0., 0., 8.], [4., 0., 8.], [0., 4., 8.]];
    let flat = mesh(vec![vertices]).prepare(0., 1e-5, &cancel).unwrap();
    for h in [7f64, 9.] {
        for i in 0..32 {
            let angle = i as f64 * std::f64::consts::TAU / 32.;
            let n = [angle.cos(), angle.sin()];
            let p = flat
                .slice_support(0, h, 2., n, &mut work, &cancel)
                .unwrap()
                .unwrap();
            let r = (4. - (h - 8.).max(0.).powi(2)).sqrt();
            let expected = vertices
                .iter()
                .map(|p| p[0] * n[0] + p[1] * n[1])
                .fold(f64::NEG_INFINITY, f64::max)
                + r;
            assert!((p[0] * n[0] + p[1] * n[1] - expected).abs() < 1e-11);
            assert!(
                (flat
                    .point_distance([p[0], p[1], h], &mut work, &cancel)
                    .unwrap()
                    .distance_mm
                    - 2.)
                    .abs()
                    < 1e-11
            );
        }
    }
    let inclined = mesh(vec![[[0., 0., 1.], [4., 0., 5.], [0., 4., 1.]]])
        .prepare(0., 1e-5, &cancel)
        .unwrap();
    let left = inclined
        .slice_support(0, 4., 2., [-1., 0.], &mut work, &cancel)
        .unwrap()
        .unwrap();
    assert!((left[0] - (3. - 2. * 2f64.sqrt())).abs() < 1e-11);
    let top = inclined
        .slice_support(0, 4., 2., [0., 1.], &mut work, &cancel)
        .unwrap()
        .unwrap();
    assert!((top[1] - (1. + 2. * 2f64.sqrt())).abs() < 1e-11);
}
#[test]
fn convex_slice_support_handles_vertical_degenerate_and_invalid_slices() {
    let cancel = AtomicBool::new(false);
    let mut work = SurfaceWork::default();
    let surface = mesh(vec![[[0., 0., 1.], [0., 0., 5.], [4., 0., 1.]]])
        .prepare(0., 1e-5, &cancel)
        .unwrap();
    let side = surface
        .slice_support(0, 4., 2., [0., 1.], &mut work, &cancel)
        .unwrap()
        .unwrap();
    assert!((side[1] - 2.).abs() < 1e-12);
    for n in [[1., 0.], [0., 1.], [-1., 0.], [0., -1.]] {
        let p = surface
            .slice_support(0, 7., 2., n, &mut work, &cancel)
            .unwrap()
            .unwrap();
        assert_eq!(p, [0., 0.]);
    }
    assert!(
        surface
            .slice_support(0, 7.1, 2., [1., 0.], &mut work, &cancel)
            .unwrap()
            .is_none()
    );
    assert!(
        surface
            .slice_support(1, 4., 2., [1., 0.], &mut work, &cancel)
            .is_err()
    );
    assert!(
        surface
            .slice_support(0, 4., 2., [1., 1.], &mut work, &cancel)
            .is_err()
    );
    assert!(
        surface
            .slice_support(0, 2., 2., [1., 0.], &mut work, &cancel)
            .is_err()
    );
    assert!(
        surface
            .slice_support(0, 4., 2., [1., 0.], &mut work, &AtomicBool::new(true))
            .is_err()
    );
}
#[test]
fn support_union_preserves_nonconvex_frame_hole_and_separate_component() {
    let cancel = AtomicBool::new(false);
    let mut triangles = vec![];
    for bounds in [
        (2., 2., 22., 7.),
        (2., 17., 22., 22.),
        (2., 7., 7., 17.),
        (17., 7., 22., 17.),
        (28., 10., 32., 14.),
    ] {
        triangles.extend(rect(bounds.0, bounds.1, bounds.2, bounds.3));
    }
    let surface = mesh(triangles).prepare(0., 1e-5, &cancel).unwrap();
    let result = prepare_contours(&surface, options(7.), &cancel).unwrap();
    assert_eq!(result.loops.len(), 3);
    assert!(contains(&result.loops, [5., 12.]));
    assert!(contains(&result.loops, [30., 12.]));
    assert!(!contains(&result.loops, [12., 12.]));
    assert!(!contains(&result.loops, [25., 12.]));
    assert_eq!(result.report.method, ContourMethod::Support);
    assert_eq!(result.report.grid_cells, 0);
    assert!(result.report.support_queries > 0 && result.report.certificate_tests > 0);
    assert!(result.report.support_error_mm + result.report.boolean_error_mm <= 0.01);
    assert!(result.report.max_normal_excess_mm <= 0.01);
    assert!(result.report.boolean_error_mm <= 4. * 1e-5 + 1e-12);
    // Every supplied prism's true convex-slice support is retained before any
    // manufacturing-margin shortcut. Test a separate unsimplified construction.
    let mut o = options(9.);
    o.simplify = false;
    let exact = prepare_contours(&surface, o, &cancel).unwrap();
    let mut work = SurfaceWork::default();
    for prism in 0..surface.prism_count() {
        for i in 0..16 {
            let angle = i as f64 * std::f64::consts::TAU / 16.;
            let n = [angle.cos(), angle.sin()];
            let point = surface
                .slice_support(prism, 9., o.radius_mm, n, &mut work, &cancel)
                .unwrap()
                .unwrap();
            assert!(
                contains(&exact.loops, point),
                "support point lost at{point:?}"
            );
        }
    }
}
#[test]
fn support_polygon_bounds_point_line_tight_budget_and_translated_slices() {
    let cancel = AtomicBool::new(false);
    for t in [
        [[0., 0., 1.], [0., 0., 5.], [4., 0., 1.]],
        [[0., 0., 5.], [4., 0., 5.], [0., 0., 1.]],
    ] {
        let surface = mesh(vec![t]).prepare(0., 1e-5, &cancel).unwrap();
        let mut o = options(7.);
        o.radius_mm = 2.;
        o.protected_radius_mm = 1.99;
        o.normal_error_mm = 0.00065;
        o.simplify = false;
        let result = prepare_contours(&surface, o, &cancel).unwrap();
        assert_eq!(result.loops.len(), 1);
        assert!(result.report.max_normal_excess_mm <= o.normal_error_mm);
    }
    let shift = [50_000., -70_000.];
    let triangles = rect(8. + shift[0], 6. + shift[1], 16. + shift[0], 10. + shift[1]);
    let surface = mesh(triangles).prepare(0., 1e-5, &cancel).unwrap();
    let mut o = options(9.);
    o.xy_min_mm = shift;
    o.xy_max_mm = [shift[0] + 24., shift[1] + 16.];
    let result = prepare_contours(&surface, o, &cancel).unwrap();
    assert_eq!(result.loops.len(), 1);
    for ring in &result.loops {
        for p in ring {
            let x = p[0] - shift[0];
            let y = p[1] - shift[1];
            let d = ((8. - x).max(x - 16.).max(0.).powi(2)
                + (6. - y).max(y - 10.).max(0.).powi(2)
                + 1.)
                .sqrt();
            assert!(
                d >= o.protected_radius_mm - 1e-8 && d <= o.radius_mm + o.normal_error_mm + 1e-8
            );
        }
    }
}
struct CancelAfter(std::cell::Cell<usize>);
impl camlib::stock::CancelSignal for CancelAfter {
    fn is_cancelled(&self) -> bool {
        let n = self.0.get();
        self.0.set(n.saturating_sub(1));
        n == 0
    }
}
#[test]
fn support_build_cancellation_is_transactional_for_motion_and_stock_generation() {
    use camlib::{
        pocket::Operation, stock::Stock, stream::StreamManifest, waterline::WaterlinePlan,
    };
    let m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/waterline.json")).unwrap();
    let mut op = match &m.operations[0] {
        Operation::Waterline(p) => p.clone(),
        _ => panic!("fixture"),
    };
    op.contour_method = ContourMethod::Support;
    let cancel = AtomicBool::new(false);
    let mut a = WaterlinePlan::prepare(
        op.clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut reference = WaterlinePlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let tool = m.tools[0].prepare().unwrap();
    // Cancel while the (parallel) level build is in progress: nothing commits.
    assert!(
        a.next_motion_with_stock(&stock, &CancelAfter(std::cell::Cell::new(1)))
            .is_err()
    );
    assert!(a.report().level_reports.is_empty());
    assert_eq!(stock.generation(), 0);
    for _ in 0..3 {
        let motion = a.next_motion_with_stock(&stock, &cancel).unwrap().unwrap();
        assert_eq!(
            motion,
            reference
                .next_motion_with_stock(&stock, &cancel)
                .unwrap()
                .unwrap()
        );
        assert!(
            a.next_motion_with_stock(&stock, &cancel)
                .unwrap_err()
                .contains("commit")
        );
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
}
