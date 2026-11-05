use camlib::session::Session;
use serde_json::{Value, json};

/// The region/geometry request interface (camlib.Engine): a bad request is
/// refused without disturbing the session's state.
#[test]
fn session_requests_recover_from_bad_requests() {
    let mut session = Session::default();
    let mut request = |v: Value| session.handle(&v.to_string());
    assert!(request(json!({"command":"classify","radius_mm":1})).is_err());
    assert!(
        request(json!({"command":"load","fixture":camlib::fixtures::narrow_channel()})).is_ok()
    );
    assert!(request(json!({"command":"probe","points":[[30,10]]})).is_ok());
    assert_eq!(
        request(json!({"command":"classify","radius_mm":1.9})).unwrap()["results"][0]["verdict"],
        "allowed"
    );
    assert!(request(json!({"command":"set_feed","feed_mm_min":-1})).is_err());
    assert_eq!(
        request(json!({"command":"classify","radius_mm":2.1})).unwrap()["results"][0]["verdict"],
        "blocked"
    );
    assert_eq!(
        request(json!({"command":"stats"})).unwrap()["geometry_preparations"],
        1
    );
    assert!(request(json!({"command":"unknown"})).is_err());
}
