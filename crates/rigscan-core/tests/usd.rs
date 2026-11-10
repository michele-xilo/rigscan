use rigscan_core::{lint, parse_usd, Format, JointKind, Robot, Span, UsdLoader};
use std::collections::HashMap;

struct Files(HashMap<String, String>);

impl UsdLoader for Files {
    fn load(&self, path: &str) -> Option<Vec<u8>> {
        self.0.get(path).map(|s| s.as_bytes().to_vec())
    }
}

fn files(list: &[(&str, &str)]) -> Files {
    Files(
        list.iter()
            .map(|(n, t)| (n.to_string(), t.to_string()))
            .collect(),
    )
}

fn robot(src: &str) -> Robot {
    parse_usd("robot.usda", files(&[("robot.usda", src)]))
        .unwrap()
        .to_robot()
        .unwrap()
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

const ARM: &str = r#"#usda 1.0
(
    defaultPrim = "r"
)

def Xform "r" (
    prepend apiSchemas = ["PhysicsArticulationRootAPI"]
)
{
    def Xform "base" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    )
    {
        float physics:mass = 2
        float3 physics:diagonalInertia = (0.1, 0.2, 0.3)
    }
    def Xform "arm" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    )
    {
        float physics:mass = 1
    }
    def PhysicsRevoluteJoint "shoulder"
    {
        rel physics:body0 = </r/base>
        rel physics:body1 = </r/arm>
        point3f physics:localPos0 = (0, 0, 0.5)
        float physics:lowerLimit = -90
        float physics:upperLimit = 45
    }
}
"#;

#[test]
fn rigid_bodies_become_links_and_joints_connect_them() {
    let r = robot(ARM);
    assert_eq!(r.format, Format::Usd);
    let names: Vec<&str> = r.links.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(names, ["/r/base", "/r/arm"]);
    assert_eq!(r.joints.len(), 1);
    let j = &r.joints[0];
    assert_eq!(
        (j.name.as_str(), j.parent.as_str(), j.child.as_str()),
        ("/r/shoulder", "/r/base", "/r/arm")
    );
    assert!(near(j.origin.xyz[2], 0.5));
}

#[test]
fn revolute_limits_convert_from_degrees_to_radians() {
    let j = &robot(ARM).joints[0];
    assert_eq!(j.kind, JointKind::Revolute);
    let l = j.limit.as_ref().unwrap();
    assert!(near(l.lower.unwrap(), -std::f64::consts::FRAC_PI_2));
    assert!(near(l.upper.unwrap(), std::f64::consts::FRAC_PI_4));
}

#[test]
fn mass_and_diagonal_inertia_are_read() {
    let r = robot(ARM);
    let i = r.links[0].inertial.as_ref().unwrap();
    assert!(near(i.mass.unwrap(), 2.0));
    let t = i.inertia.as_ref().unwrap();
    assert!(near(t.ixx, 0.1) && near(t.iyy, 0.2) && near(t.izz, 0.3));
    assert!(near(t.ixy, 0.0));
    // The arm has a mass and no inertia: the missing part stays missing.
    let arm = r.links[1].inertial.as_ref().unwrap();
    assert!(arm.inertia.is_none());
}

#[test]
fn body_without_mass_api_has_no_inertial() {
    let r = robot(
        r#"#usda 1.0
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
"#,
    );
    assert!(r.links[0].inertial.is_none());
}

#[test]
fn zero_mass_means_computed_from_shapes_not_zero() {
    let r = robot(
        r#"#usda 1.0
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
{
    float physics:mass = 0
}
"#,
    );
    assert!(r.links[0]
        .inertial
        .as_ref()
        .is_none_or(|i| i.mass.is_none()));
}

#[test]
fn joint_without_body0_is_anchored_to_world() {
    let r = robot(
        r#"#usda 1.0
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def PhysicsFixedJoint "root"
{
    rel physics:body1 = </b>
}
"#,
    );
    let names: Vec<&str> = r.links.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(names, ["world", "/b"]);
    assert_eq!(r.joints[0].parent, "world");
    assert_eq!(r.joints[0].kind, JointKind::Fixed);
}

#[test]
fn revolute_without_limits_is_continuous_and_prismatic_keeps_its_units() {
    let r = robot(
        r#"#usda 1.0
def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def Xform "c" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def PhysicsRevoluteJoint "spin"
{
    rel physics:body0 = </a>
    rel physics:body1 = </b>
}
def PhysicsPrismaticJoint "slide"
{
    rel physics:body0 = </b>
    rel physics:body1 = </c>
    float physics:lowerLimit = -2
    float physics:upperLimit = 4
}
"#,
    );
    assert_eq!(r.joints[0].kind, JointKind::Continuous);
    assert!(r.joints[0].limit.is_none());
    assert_eq!(r.joints[1].kind, JointKind::Prismatic);
    let l = r.joints[1].limit.as_ref().unwrap();
    assert!(near(l.lower.unwrap(), -2.0) && near(l.upper.unwrap(), 4.0));
}

#[test]
fn other_joint_types_keep_their_name() {
    let r = robot(
        r#"#usda 1.0
def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def PhysicsSphericalJoint "ball"
{
    rel physics:body0 = </a>
    rel physics:body1 = </b>
}
"#,
    );
    assert_eq!(r.joints[0].kind, JointKind::Other("spherical".into()));
}

#[test]
fn joint_target_that_does_not_exist_is_an_undefined_link() {
    let r = robot(
        r#"#usda 1.0
def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def PhysicsFixedJoint "j"
{
    rel physics:body0 = </a>
    rel physics:body1 = </ghost>
}
"#,
    );
    let rules: Vec<&str> = lint(&r).iter().map(|f| f.rule).collect();
    assert!(rules.contains(&"RS001"), "{rules:?}");
}

#[test]
fn static_prim_a_joint_connects_is_a_link() {
    let r = robot(
        r#"#usda 1.0
def Xform "ground" {}
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def PhysicsFixedJoint "j"
{
    rel physics:body0 = </ground>
    rel physics:body1 = </b>
}
"#,
    );
    assert!(r.links.iter().any(|l| l.name == "/ground"));
    assert!(lint(&r).is_empty(), "{:?}", lint(&r));
}

#[test]
fn what_the_model_cannot_hold_is_listed_without_a_position() {
    let r = robot(
        r#"#usda 1.0
def PhysicsScene "scene" {}
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsDriveAPI:angular"]) {}
"#,
    );
    let what: Vec<&str> = r.unmapped.iter().map(|u| u.what.as_str()).collect();
    assert!(what.contains(&"scene"), "{what:?}");
    assert!(what.contains(&"collision"), "{what:?}");
    assert!(what.contains(&"drive"), "{what:?}");
    assert!(r.unmapped.iter().all(|u| u.span == Span::NONE));
}

#[test]
fn model_positions_are_unknown() {
    let r = robot(ARM);
    assert!(r.span.is_none());
    assert!(r.links.iter().all(|l| l.span.is_none()));
    assert!(r.joints.iter().all(|j| j.span.is_none()));
}

#[test]
fn healthy_chain_has_no_findings() {
    let f = lint(&robot(ARM));
    // The arm has no parent besides the base, and the base is the only root.
    assert!(f.is_empty(), "{f:?}");
}

#[test]
fn references_are_composed_through_the_loader() {
    let r = parse_usd(
        "scene.usda",
        files(&[
            (
                "scene.usda",
                "#usda 1.0\ndef \"world\" (references = @./sub/arm.usda@)\n{\n}\n",
            ),
            ("sub/arm.usda", ARM),
        ]),
    )
    .unwrap();
    assert!(
        r.composition_errors.is_empty(),
        "{:?}",
        r.composition_errors
    );
    let robot = r.to_robot().unwrap();
    assert_eq!(robot.links.len(), 2);
    assert_eq!(robot.links[0].name, "/world/base");
}

#[test]
fn missing_reference_is_recorded_not_fatal() {
    let r = parse_usd(
        "scene.usda",
        files(&[(
            "scene.usda",
            "#usda 1.0\ndef \"world\" (references = @./gone.usda@)\n{\n}\n",
        )]),
    )
    .unwrap();
    assert!(!r.composition_errors.is_empty());
}

#[test]
fn malformed_text_is_a_parse_error_with_its_line() {
    let e = parse_usd(
        "robot.usda",
        files(&[(
            "robot.usda",
            "#usda 1.0\ndef Xform \"a\" {\n  float = = \n}\n",
        )]),
    )
    .unwrap_err();
    assert!(e.0.contains("3"), "{}", e.0);
}

#[test]
fn missing_root_file_is_a_parse_error() {
    assert!(parse_usd("nope.usda", files(&[])).is_err());
}

#[test]
fn nested_references_resolve_against_the_layer_that_makes_them() {
    let r = parse_usd(
        "scene.usda",
        files(&[
            (
                "scene.usda",
                "#usda 1.0\ndef \"w\" (references = @sub/a.usda@)\n{\n}\n",
            ),
            (
                "sub/a.usda",
                "#usda 1.0\n(defaultPrim = \"a\")\ndef \"a\" (references = @./../parts/b.usda@)\n{\n}\n",
            ),
            (
                "parts/b.usda",
                "#usda 1.0\n(defaultPrim = \"b\")\ndef Xform \"b\" (prepend apiSchemas = [\"PhysicsRigidBodyAPI\"]) {}\n",
            ),
        ]),
    )
    .unwrap();
    assert!(
        r.composition_errors.is_empty(),
        "{:?}",
        r.composition_errors
    );
    assert_eq!(r.to_robot().unwrap().links[0].name, "/w");
}

#[test]
fn rotations_the_model_cannot_hold_are_listed_unless_identity() {
    let r = robot(
        r#"#usda 1.0
def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysicsMaterialAPI"])
{
    float3 physics:diagonalInertia = (1, 1, 1)
    quatf physics:principalAxes = (0.7071068, 0, 0.7071068, 0)
}
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
{
    float3 physics:diagonalInertia = (1, 1, 1)
    quatd physics:principalAxes = (1, 0, 0, 0)
}
def PhysicsFixedJoint "j"
{
    rel physics:body0 = </a>
    rel physics:body1 = </b>
    quatf physics:localRot0 = (0.7071068, 0.7071068, 0, 0)
    quatf physics:localRot1 = (1, 0, 0, 0)
}
"#,
    );
    let mut what: Vec<(&str, &str)> = r
        .unmapped
        .iter()
        .map(|u| (u.what.as_str(), u.name.as_deref().unwrap()))
        .collect();
    what.sort();
    assert_eq!(
        what,
        [
            ("localRot0", "/j"),
            ("material", "/a"),
            ("principalAxes", "/a")
        ]
    );
}

#[test]
fn double_precision_values_are_read_and_the_generic_joint_is_other() {
    let r = robot(
        r#"#usda 1.0
def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
{
    double physics:mass = 3
    double3 physics:diagonalInertia = (0.5, 0.6, 0.7)
}
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def PhysicsJoint "free"
{
    rel physics:body0 = </a>
    rel physics:body1 = </b>
    point3d physics:localPos0 = (1, 2, 3)
}
"#,
    );
    let i = r.links[0].inertial.as_ref().unwrap();
    assert!(near(i.mass.unwrap(), 3.0));
    assert!(near(i.inertia.as_ref().unwrap().iyy, 0.6));
    assert_eq!(r.joints[0].kind, JointKind::Other("joint".into()));
    assert!(near(r.joints[0].origin.xyz[1], 2.0));
}

#[test]
fn values_of_an_unexpected_type_are_ignored_or_listed() {
    let r = robot(
        r#"#usda 1.0
def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
{
    string physics:mass = "heavy"
    float physics:diagonalInertia = 1
}
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def PhysicsFixedJoint "j"
{
    rel physics:body0 = </a>
    rel physics:body1 = </b>
    float physics:localRot0 = 1
}
"#,
    );
    assert!(r.links[0].inertial.is_none());
    let what: Vec<&str> = r.unmapped.iter().map(|u| u.what.as_str()).collect();
    assert_eq!(what, ["localRot0"]);
}
