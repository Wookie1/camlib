use super::*;

#[test]
fn indexed_mesh_round_trip_preserves_all_coordinate_bits_and_source() {
    let points = [[-0.0, 0., 2.], [1., 0., 2.], [0., 1., 2.], [1., 1., 3.]];
    let triangles = (0..1200)
        .map(|i| {
            if i % 2 == 0 {
                [points[0], points[1], points[2]]
            } else {
                [points[1], points[3], points[2]]
            }
        })
        .collect::<Vec<_>>();
    let mesh = SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: triangles.into(),
        surface_error_mm: 0.003,
        source: serde_json::json!({"adapter":"test"}),
    };
    let wire = serde_json::to_value(&mesh).unwrap();
    assert_eq!(wire["vertices"].as_array().unwrap().len(), 4);
    assert!(wire.get("triangles").is_none());
    let restored: SurfaceMesh = serde_json::from_value(wire).unwrap();
    assert_eq!(restored.source, mesh.source);
    assert_eq!(restored.surface_error_mm, mesh.surface_error_mm);
    for (a, b) in mesh
        .triangles
        .iter()
        .flatten()
        .flatten()
        .zip(restored.triangles.iter().flatten().flatten())
    {
        assert_eq!(a.to_bits(), b.to_bits());
    }
}

#[test]
fn invalid_indexed_mesh_is_rejected_before_geometric_use() {
    let base = serde_json::json!({"version":1,"units":"mm","surface_error_mm":0.,"source":null,
        "vertices":[[0.,0.,1.],[1.,0.,1.],[0.,1.,1.]],"indices":[[0,1,2]]});
    let good: SurfaceMesh = serde_json::from_value(base.clone()).unwrap();
    assert_eq!(good.triangles.len(), 1);
    assert!(
        serde_json::to_value(&good)
            .unwrap()
            .get("triangles")
            .is_some()
    );
    let mut outside = base.clone();
    outside["indices"][0][2] = 3.into();
    assert!(
        serde_json::from_value::<SurfaceMesh>(outside)
            .unwrap_err()
            .to_string()
            .contains("index out of range")
    );
    let mut mixed = base.clone();
    mixed["triangles"] = serde_json::json!([[[0., 0., 1.], [1., 0., 1.], [0., 1., 1.]]]);
    assert!(serde_json::from_value::<SurfaceMesh>(mixed).is_err());
    let mut missing = base.clone();
    missing.as_object_mut().unwrap().remove("indices");
    assert!(serde_json::from_value::<SurfaceMesh>(missing).is_err());
    let mut empty = base;
    empty["indices"] = serde_json::json!([]);
    assert!(serde_json::from_value::<SurfaceMesh>(empty).is_err());
}
