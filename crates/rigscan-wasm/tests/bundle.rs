use rigscan_wasm::scan_bundle;
use serde_json::Value;
use std::collections::BTreeMap;

fn bundle(files: &[(&str, &str)]) -> BTreeMap<String, Vec<u8>> {
    files
        .iter()
        .map(|(p, t)| (p.to_string(), t.as_bytes().to_vec()))
        .collect()
}

fn scan(main: &str, files: &[(&str, &str)]) -> Value {
    scan_bundle(main, bundle(files))
}

fn rules(v: &Value) -> Vec<&str> {
    v["findings"]
        .as_array()
        .unwrap()
        .iter()
        .map(|f| f["rule"].as_str().unwrap())
        .collect()
}

const HEALTHY: &str = r#"<robot name="arm">
  <link name="base"><inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
  <link name="hand"><inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
  <joint name="wrist" type="revolute"><parent link="base"/><child link="hand"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>"#;

const BROKEN: &str = r#"<robot name="arm">
  <link name="base"/>
  <joint name="elbow" type="fixed"><parent link="base"/><child link="ghost"/></joint>
</robot>"#;

#[test]
fn a_healthy_urdf_has_no_findings_and_describes_its_tree() {
    let v = scan("arm.urdf", &[("arm.urdf", HEALTHY)]);
    assert_eq!(v["version"], 1);
    assert_eq!(v["format"], "urdf");
    assert_eq!(v["robot"], "arm");
    assert_eq!(v["findings"].as_array().unwrap().len(), 0);
    assert_eq!(v["verdict"]["healthy"], 3);
    let links: Vec<&str> = v["links"]
        .as_array()
        .unwrap()
        .iter()
        .map(|l| l["name"].as_str().unwrap())
        .collect();
    assert_eq!(links, ["base", "hand"]);
    let j = &v["joints"][0];
    assert_eq!(
        (
            j["name"].as_str(),
            j["kind"].as_str(),
            j["parent"].as_str(),
            j["child"].as_str(),
            j["status"].as_str()
        ),
        (
            Some("wrist"),
            Some("revolute"),
            Some("base"),
            Some("hand"),
            Some("healthy")
        )
    );
    assert_eq!(v["source"], HEALTHY);
}

#[test]
fn a_finding_carries_its_position_element_and_the_joint_status() {
    let v = scan("arm.urdf", &[("arm.urdf", BROKEN)]);
    assert_eq!(rules(&v), ["RS001"]);
    let f = &v["findings"][0];
    assert_eq!(f["severity"], "critical");
    assert_eq!(
        (f["line"].as_u64(), f["column"].as_u64()),
        (Some(3), Some(3))
    );
    assert_eq!(f["element"]["kind"], "joint");
    assert_eq!(f["element"]["name"], "elbow");
    assert_eq!(v["joints"][0]["status"], "critical");
    assert_eq!(v["links"][0]["status"], "healthy");
    assert_eq!(v["verdict"]["critical"], 1);
}

#[test]
fn mesh_files_are_not_checked_because_the_page_cannot_know_what_exists() {
    let urdf = r#"<robot name="t"><link name="a"><visual><geometry><mesh filename="meshes/gone.stl"/></geometry></visual></link></robot>"#;
    let v = scan("t.urdf", &[("t.urdf", urdf)]);
    assert!(!rules(&v).contains(&"RS014"), "{:?}", rules(&v));
}

#[test]
fn an_mjcf_include_is_read_from_the_dropped_files() {
    let main = r#"<mujoco><include file="parts/arm.xml"/><worldbody/></mujoco>"#;
    let part = r#"<mujoco><worldbody><body name="b"><geom size="1"/></body></worldbody></mujoco>"#;
    let v = scan("robot.xml", &[("robot.xml", main), ("parts/arm.xml", part)]);
    assert_eq!(v["format"], "mjcf");
    assert!(!rules(&v).contains(&"RS018"), "{:?}", rules(&v));
    let missing = scan("robot.xml", &[("robot.xml", main)]);
    assert!(rules(&missing).contains(&"RS018"), "{:?}", rules(&missing));
}

#[test]
fn includes_resolve_from_the_file_that_contains_them_and_through_dot_dot() {
    let main = r#"<mujoco><include file="sub/inner.xml"/><worldbody/></mujoco>"#;
    let inner = r#"<mujoco><include file="../shared/leaf.xml"/></mujoco>"#;
    let leaf = r#"<mujoco><worldbody><body name="b"><geom size="1"/></body></worldbody></mujoco>"#;
    let v = scan(
        "robots/r.xml",
        &[
            ("robots/r.xml", &main.replace("sub/", "../robots/sub/")),
            ("robots/sub/inner.xml", inner),
            ("robots/shared/leaf.xml", leaf),
        ],
    );
    assert!(
        !rules(&v).contains(&"RS018"),
        "{:?} {:?}",
        rules(&v),
        v["findings"]
    );
    assert_eq!(v["links"].as_array().unwrap().len(), 2);
}

#[test]
fn a_usd_stage_composes_its_sublayers_from_the_dropped_files() {
    let main = "#usda 1.0\n(\n    subLayers = [@parts/arm.usda@]\n)\n";
    let part = "#usda 1.0\ndef Xform \"a\" (prepend apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\"])\n{\n    float physics:mass = 1\n}\n";
    let v = scan(
        "robot.usda",
        &[("robot.usda", main), ("parts/arm.usda", part)],
    );
    assert_eq!(v["format"], "usd");
    assert!(!rules(&v).contains(&"RS022"), "{:?}", v);
    assert_eq!(v["links"][0]["name"], "/a");
    let missing = scan("robot.usda", &[("robot.usda", main)]);
    assert!(rules(&missing).contains(&"RS022"), "{:?}", missing);
}

#[test]
fn a_file_that_does_not_parse_reports_the_error() {
    let v = scan("bad.urdf", &[("bad.urdf", "<robot><link")]);
    assert_eq!(v["path"], "bad.urdf");
    assert!(v["error"].as_str().unwrap().contains("bad.urdf"), "{v}");
    assert!(v.get("findings").is_none());
}

#[test]
fn unsupported_files_and_an_xml_that_is_not_mjcf_are_errors() {
    let v = scan("notes.txt", &[("notes.txt", "hello")]);
    assert!(v["error"].as_str().unwrap().contains("unsupported"), "{v}");
    let launch = scan("launch.xml", &[("launch.xml", "<launch/>")]);
    assert!(
        launch["error"].as_str().unwrap().contains("unsupported"),
        "{launch}"
    );
}

#[test]
fn a_main_file_that_was_not_dropped_is_an_error() {
    let v = scan("gone.urdf", &[("other.urdf", HEALTHY)]);
    assert!(v["error"].as_str().unwrap().contains("gone.urdf"), "{v}");
}
