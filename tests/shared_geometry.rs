use camlib::{material::Definition, pocket::Operation, stream::StreamManifest};
use serde_json::{Value, json};
use std::sync::Arc;

fn shared() -> Value {
    let mut m: Value =
        serde_json::from_str(include_str!("../fixtures/operations/parallel.json")).unwrap();
    let mut surface = m["operations"][0]["surface"].take();
    m["geometry"] = json!([{ "triangles": surface["triangles"].take() }]);
    surface.as_object_mut().unwrap().remove("triangles");
    surface["geometry"] = json!(0);
    m["operations"][0]["surface"] = surface.clone();
    m["verification"] = json!({"scene":null,"material":[{"after_operation":null,
        "target":{"version":1,"units":"mm","id":"target","revision":"1",
                  "surface":surface,"base_z_mm":0.},"options":{}}]});
    m
}

#[test]
fn inline_meshes_are_written_as_one_shared_table() {
    let source = include_str!("../fixtures/operations/parallel.json");
    let m: StreamManifest = serde_json::from_str(source).unwrap();
    let written = serde_json::to_value(&m).unwrap();
    assert_eq!(written["geometry"].as_array().unwrap().len(), 1);
    assert!(
        written["operations"][0]["surface"]
            .get("triangles")
            .is_none()
    );
    let bytes = m.canonical_bytes().unwrap();
    let restored: StreamManifest = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(bytes, restored.canonical_bytes().unwrap());
    assert_eq!(m.digest().unwrap(), restored.digest().unwrap());
}

#[test]
fn typed_references_share_facets_and_copy_on_write_preserves_other_uses() {
    let mut m: StreamManifest = serde_json::from_value(shared()).unwrap();
    m.validate().unwrap();
    let before = m.canonical_bytes().unwrap();
    let restored: StreamManifest = serde_json::from_slice(&before).unwrap();
    assert_eq!(before, restored.canonical_bytes().unwrap());
    let Operation::Parallel(op) = &mut m.operations[0] else {
        panic!()
    };
    let Definition::Surface(target) = &m.verification.as_ref().unwrap().material[0].target else {
        panic!()
    };
    assert!(Arc::ptr_eq(
        &op.surface.triangles,
        &target.surface.triangles
    ));
    let old = target.surface.triangles[0][0][0];
    Arc::make_mut(&mut op.surface.triangles)[0][0][0] += 0.1;
    assert_eq!(target.surface.triangles[0][0][0], old);
    assert!(!Arc::ptr_eq(
        &op.surface.triangles,
        &target.surface.triangles
    ));
    let changed = serde_json::to_value(&m).unwrap();
    assert_eq!(changed["geometry"].as_array().unwrap().len(), 2);
    assert_ne!(before, m.canonical_bytes().unwrap());
}

#[test]
fn source_error_coordinates_and_revision_remain_bound() {
    let base = shared();
    let m: StreamManifest = serde_json::from_value(base.clone()).unwrap();
    let identity = m.digest().unwrap();
    for i in 0..5 {
        let mut v = base.clone();
        match i {
            0 => v["operations"][0]["surface"]["surface_error_mm"] = json!(0.012),
            1 => v["operations"][0]["surface"]["source"] = json!({"revision":"2"}),
            2 => v["geometry"][0]["triangles"][0][0][0] = json!(-0.0),
            3 => v["run"]["revision"] = json!("2"),
            _ => v["verification"]["material"][0]["target"]["revision"] = json!("2"),
        }
        let changed: StreamManifest = serde_json::from_value(v).unwrap();
        assert_ne!(identity, changed.digest().unwrap(), "mutation {i}");
    }
}

#[test]
fn invalid_references_cannot_expand_cycle_override_or_hide_unused_geometry() {
    for i in 0..7 {
        let mut v = shared();
        match i {
            0 => v["operations"][0]["surface"]["geometry"] = json!(1),
            1 => v["operations"][0]["surface"]["geometry"] = json!(-1),
            2 => v["operations"][0]["surface"]["triangles"] = json!([]),
            3 => v["geometry"][0] = json!({"geometry":0}),
            4 => {
                v.as_object_mut().unwrap().remove("geometry");
            }
            5 => {
                let unused = v["geometry"][0].clone();
                v["geometry"].as_array_mut().unwrap().push(unused);
            }
            _ => v["geometry"][0]["triangles"] = json!([]),
        }
        assert!(
            serde_json::from_value::<StreamManifest>(v).is_err(),
            "mutation {i}"
        );
    }
    let mut v = shared();
    v["geometry"] = json!(vec![v["geometry"][0].clone(); 130]);
    assert!(serde_json::from_value::<StreamManifest>(v).is_err());
}
