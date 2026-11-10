use rigscan_core::{lint_usd, parse_usd, Element, Finding, Severity, UsdLoader};
use std::collections::HashMap;

struct Files(HashMap<String, String>);

impl UsdLoader for Files {
    fn load(&self, path: &str) -> Option<Vec<u8>> {
        self.0.get(path).map(|s| s.as_bytes().to_vec())
    }
}

fn scan(src: &str) -> Vec<Finding> {
    let files = Files(HashMap::from([("robot.usda".to_string(), src.to_string())]));
    let usd = parse_usd("robot.usda", files).unwrap();
    let robot = usd.to_robot().unwrap();
    lint_usd(&usd, &robot)
}

fn only<'a>(f: &'a [Finding], rule: &str) -> Vec<&'a Finding> {
    f.iter().filter(|f| f.rule == rule).collect()
}

const BODY: &str = r#"prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]"#;

fn chain(extra: &str) -> String {
    format!(
        r#"#usda 1.0
def Xform "r" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{{
    def Xform "a" ({BODY}) {{ float physics:mass = 1 }}
    def Xform "b" ({BODY}) {{ float physics:mass = 1 }}
    def PhysicsRevoluteJoint "j"
    {{
        rel physics:body0 = </r/a>
        rel physics:body1 = </r/b>
    }}
{extra}
}}
"#
    )
}

#[test]
fn healthy_robot_has_no_findings() {
    let f = scan(&chain(""));
    assert!(f.is_empty(), "{f:?}");
}

#[test]
fn shared_rules_run_on_the_mapped_robot() {
    let f = scan(
        &chain("").replace(
            "rel physics:body1 = </r/b>",
            "rel physics:body1 = </r/b>\n        float physics:lowerLimit = 10\n        float physics:upperLimit = -10",
        ),
    );
    assert_eq!(only(&f, "RS012").len(), 1, "{f:?}");
}

#[test]
fn joint_from_a_body_to_itself_is_critical() {
    let f = scan(&chain("").replace("</r/b>", "</r/a>"));
    let hits = only(&f, "RS019");
    assert_eq!(hits.len(), 1, "{f:?}");
    assert_eq!(hits[0].severity, Severity::Critical);
    assert_eq!(hits[0].element, Element::Joint(0));
    assert_eq!(hits[0].message, "joint `/r/j` connects `/r/a` to itself");
}

#[test]
fn joint_with_no_bodies_connects_the_world_to_itself() {
    let f = scan(&format!(
        r#"#usda 1.0
def Xform "r" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{{
    def Xform "a" ({BODY}) {{ float physics:mass = 1 }}
    def PhysicsFixedJoint "j" {{}}
}}
"#
    ));
    let hits = only(&f, "RS019");
    assert_eq!(hits.len(), 1, "{f:?}");
    assert_eq!(hits[0].message, "joint `/r/j` connects `world` to itself");
}

#[test]
fn body_with_no_mass_and_no_collider_is_a_warning() {
    let f = scan(&chain(
        r#"    def Xform "c" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}"#,
    ));
    let hits = only(&f, "RS020");
    assert_eq!(hits.len(), 1, "{f:?}");
    assert_eq!(hits[0].severity, Severity::Warning);
    assert_eq!(hits[0].element, Element::Link(2));
    assert_eq!(
        hits[0].message,
        "rigid body `/r/c` has no mass and no collision shape, so its mass cannot be computed"
    );
}

#[test]
fn zero_mass_counts_as_no_mass() {
    let f = scan(&chain(&format!(
        r#"    def Xform "c" ({BODY}) {{ float physics:mass = 0 }}"#
    )));
    assert_eq!(only(&f, "RS020").len(), 1, "{f:?}");
}

#[test]
fn a_collider_under_the_body_supplies_the_mass() {
    let f = scan(&chain(
        r#"    def Xform "c" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "shape" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {}
    }"#,
    ));
    assert!(only(&f, "RS020").is_empty(), "{f:?}");
}

#[test]
fn a_collider_belongs_to_its_nearest_body_only() {
    let f = scan(&chain(
        r#"    def Xform "outer" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Xform "inner" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
        {
            def Cube "shape" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {}
        }
    }"#,
    ));
    let hits = only(&f, "RS020");
    assert_eq!(hits.len(), 1, "{f:?}");
    assert!(hits[0].message.contains("/r/outer"), "{}", hits[0].message);
}

#[test]
fn joints_without_an_articulation_root_are_a_warning() {
    let f = scan(&chain("").replace(
        r#"(prepend apiSchemas = ["PhysicsArticulationRootAPI"])"#,
        "",
    ));
    let hits = only(&f, "RS021");
    assert_eq!(hits.len(), 1, "{f:?}");
    assert_eq!(hits[0].severity, Severity::Warning);
    assert_eq!(hits[0].element, Element::Robot);
}

#[test]
fn a_lone_body_needs_no_articulation_root() {
    let f = scan(&format!(
        r#"#usda 1.0
def Xform "a" ({BODY}) {{ float physics:mass = 1 }}
"#
    ));
    assert!(only(&f, "RS021").is_empty(), "{f:?}");
}

#[test]
fn a_reference_that_cannot_be_found_is_critical() {
    let f = scan("#usda 1.0\ndef \"w\" (references = @./gone.usda@)\n{\n}\n");
    let hits = only(&f, "RS022");
    assert!(!hits.is_empty(), "{f:?}");
    assert_eq!(hits[0].severity, Severity::Critical);
    assert!(hits[0].message.contains("gone.usda"), "{}", hits[0].message);
}

#[test]
fn usd_findings_have_no_position() {
    let f = scan(&chain("").replace("</r/b>", "</r/a>"));
    assert!(f.iter().all(|f| f.span.is_none()), "{f:?}");
}

#[test]
fn a_collider_outside_any_body_is_ignored() {
    let f = scan(&chain(
        r#"    def Cube "ground" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {}"#,
    ));
    assert!(f.is_empty(), "{f:?}");
}

#[test]
fn a_closed_chain_is_not_reported() {
    let f = scan(&chain(
        r#"    def PhysicsRevoluteJoint "k"
    {
        rel physics:body0 = </r/a>
        rel physics:body1 = </r/b>
    }"#,
    ));
    assert!(only(&f, "RS004").is_empty(), "{f:?}");
}

#[test]
fn a_loop_of_joints_is_not_reported() {
    let f = scan(&chain(
        r#"    def PhysicsRevoluteJoint "k"
    {
        rel physics:body0 = </r/b>
        rel physics:body1 = </r/a>
    }"#,
    ));
    assert!(only(&f, "RS005").is_empty(), "{f:?}");
}

#[test]
fn a_free_body_is_not_a_second_root() {
    let f = scan(&chain(
        r#"    def Xform "free" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]) { float physics:mass = 1 }"#,
    ));
    assert!(only(&f, "RS006").is_empty(), "{f:?}");
}

#[test]
fn a_stage_without_rigid_bodies_is_not_reported() {
    let f = scan("#usda 1.0\ndef Xform \"prop\"\n{\n}\n");
    assert!(f.is_empty(), "{f:?}");
}
