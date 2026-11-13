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
    assert_eq!(what, [("material", "/a"), ("principalAxes", "/a")]);
    let half_pi = std::f64::consts::FRAC_PI_2;
    assert!(
        near(r.joints[0].origin.rpy[0], half_pi),
        "{:?}",
        r.joints[0].origin
    );
}

fn rot_joint(rot0: &str, rot1: &str) -> Robot {
    robot(&format!(
        r#"#usda 1.0
def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {{}}
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {{}}
def PhysicsFixedJoint "j"
{{
    rel physics:body0 = </a>
    rel physics:body1 = </b>
    {rot0}
    {rot1}
}}
"#
    ))
}

#[test]
fn local_rot0_becomes_the_joint_origin_rpy() {
    let r = rot_joint(
        "quatd physics:localRot0 = (0.7071067811865476, 0, 0, 0.7071067811865476)",
        "",
    );
    assert!(near(r.joints[0].origin.rpy[2], std::f64::consts::FRAC_PI_2));
    assert!(r.unmapped.is_empty(), "{:?}", r.unmapped);
    let r = rot_joint("quatf physics:localRot0 = (1, 0, 0, 0)", "");
    assert_eq!(r.joints[0].origin.rpy, [0.0; 3]);
}

#[test]
fn a_zero_local_rot0_is_listed_as_lost() {
    let r = rot_joint("quatf physics:localRot0 = (0, 0, 0, 0)", "");
    assert_eq!(r.joints[0].origin.rpy, [0.0; 3]);
    let what: Vec<&str> = r.unmapped.iter().map(|u| u.what.as_str()).collect();
    assert_eq!(what, ["localRot0"]);
}

#[test]
fn a_turned_local_rot1_is_still_listed_as_lost() {
    let r = rot_joint("", "quatf physics:localRot1 = (0.7071068, 0.7071068, 0, 0)");
    let what: Vec<&str> = r.unmapped.iter().map(|u| u.what.as_str()).collect();
    assert_eq!(what, ["localRot1"]);
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

fn axis_robot(joint: &str, extra: &str) -> Robot {
    robot(&format!(
        r#"#usda 1.0
def Xform "r"
{{
    def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {{}}
    def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {{}}
    def {joint} "j"
    {{
        rel physics:body0 = </r/a>
        rel physics:body1 = </r/b>
        {extra}
    }}
}}
"#
    ))
}

#[test]
fn joint_axis_defaults_to_x_and_follows_the_token() {
    assert_eq!(
        axis_robot("PhysicsRevoluteJoint", "").joints[0].axis,
        Some([1.0, 0.0, 0.0])
    );
    assert_eq!(
        axis_robot(
            "PhysicsPrismaticJoint",
            "uniform token physics:axis = \"Y\""
        )
        .joints[0]
            .axis,
        Some([0.0, 1.0, 0.0])
    );
    assert_eq!(
        axis_robot("PhysicsRevoluteJoint", "uniform token physics:axis = \"Z\"").joints[0].axis,
        Some([0.0, 0.0, 1.0])
    );
}

#[test]
fn fixed_and_generic_usd_joints_have_no_axis() {
    assert_eq!(axis_robot("PhysicsFixedJoint", "").joints[0].axis, None);
    assert_eq!(axis_robot("PhysicsJoint", "").joints[0].axis, None);
}

#[test]
fn center_of_mass_is_the_inertial_origin() {
    let r = robot(
        r#"#usda 1.0
def Xform "r"
{
    def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
    {
        float physics:mass = 1
        point3f physics:centerOfMass = (0.1, 0.2, 0.3)
    }
}
"#,
    );
    let o = r.links[0].inertial.as_ref().unwrap().origin;
    assert!(
        near(o.xyz[0], 0.1) && near(o.xyz[1], 0.2) && near(o.xyz[2], 0.3),
        "{o:?}"
    );
}

#[test]
fn usd_joints_have_no_dynamics() {
    assert_eq!(
        axis_robot("PhysicsRevoluteJoint", "").joints[0].dynamics,
        None
    );
}

#[test]
fn an_unauthored_center_of_mass_is_the_body_origin() {
    let r = robot(
        r#"#usda 1.0
def Xform "r"
{
    def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
    {
        float physics:mass = 1
        point3f physics:centerOfMass = (-inf, -inf, -inf)
    }
}
"#,
    );
    assert_eq!(r.links[0].inertial.as_ref().unwrap().origin.xyz, [0.0; 3]);
}

#[test]
fn an_unknown_axis_token_falls_back_to_x() {
    assert_eq!(
        axis_robot("PhysicsRevoluteJoint", "uniform token physics:axis = \"W\"").joints[0].axis,
        Some([1.0, 0.0, 0.0])
    );
}

#[test]
fn a_joint_can_connect_a_prim_that_is_not_a_rigid_body() {
    let r = robot(
        r#"#usda 1.0
def Xform "r"
{
    def Xform "ground" {}
    def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
    def PhysicsFixedJoint "j"
    {
        rel physics:body0 = </r/ground>
        rel physics:body1 = </r/a>
    }
}
"#,
    );
    let ground = r.links.iter().find(|l| l.name == "/r/ground").unwrap();
    assert_eq!(ground.inertial, None);
}

const UNITS: &str = r#"#usda 1.0
(
    metersPerUnit = 0.01
    kilogramsPerUnit = 0.001
    upAxis = "Z"
)
def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
{
    float physics:mass = 2000
    float3 physics:diagonalInertia = (1, 2, 3)
    point3f physics:centerOfMass = (0, 0, 10)
}
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
def PhysicsPrismaticJoint "slide"
{
    rel physics:body0 = </a>
    rel physics:body1 = </b>
    point3f physics:localPos0 = (0, 0, 50)
    float physics:lowerLimit = 0
    float physics:upperLimit = 20
}
def PhysicsRevoluteJoint "turn"
{
    rel physics:body0 = </a>
    rel physics:body1 = </b>
    float physics:lowerLimit = -90
    float physics:upperLimit = 90
}
"#;

#[test]
fn lengths_and_masses_are_converted_from_the_stage_units() {
    let r = robot(UNITS);
    let a = r.links.iter().find(|l| l.name == "/a").unwrap();
    let inertial = a.inertial.as_ref().unwrap();
    assert!(near(inertial.mass.unwrap(), 2.0), "{inertial:?}");
    let i = inertial.inertia.unwrap();
    assert!(
        near(i.ixx, 1e-7) && near(i.iyy, 2e-7) && near(i.izz, 3e-7),
        "{i:?}"
    );
    assert!(near(inertial.origin.xyz[2], 0.1), "{inertial:?}");
    let slide = r.joints.iter().find(|j| j.name == "/slide").unwrap();
    assert!(near(slide.origin.xyz[2], 0.5), "{:?}", slide.origin);
    let limit = slide.limit.as_ref().unwrap();
    assert!(near(limit.upper.unwrap(), 0.2), "{limit:?}");
    let turn = r.joints.iter().find(|j| j.name == "/turn").unwrap();
    let half_pi = std::f64::consts::FRAC_PI_2;
    assert!(near(turn.limit.as_ref().unwrap().upper.unwrap(), half_pi));
}

#[test]
fn a_y_up_stage_is_listed_as_lost_and_a_z_up_stage_is_not() {
    let up = |axis: &str| {
        robot(&format!(
            "#usda 1.0\n(\n    upAxis = \"{axis}\"\n)\ndef Xform \"a\" (prepend apiSchemas = [\"PhysicsRigidBodyAPI\"]) {{}}\n"
        ))
        .unmapped
        .iter()
        .map(|u| u.what.clone())
        .collect::<Vec<_>>()
    };
    assert_eq!(up("Y"), ["upAxis"]);
    assert!(up("Z").is_empty());
}

#[test]
fn a_local_pos1_is_listed_as_lost_unless_zero() {
    let lost = |pos1: &str| {
        robot(&format!(
            r#"#usda 1.0
def Xform "a" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {{}}
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {{}}
def PhysicsFixedJoint "j"
{{
    rel physics:body0 = </a>
    rel physics:body1 = </b>
    point3f physics:localPos1 = {pos1}
}}
"#
        ))
        .unmapped
        .iter()
        .map(|u| u.what.clone())
        .collect::<Vec<_>>()
    };
    assert_eq!(lost("(0, 0, 0.2)"), ["localPos1"]);
    assert!(lost("(0, 0, 0)").is_empty());
}

fn shapes_of(body_children: &str) -> Robot {
    robot(&format!(
        "#usda 1.0\n(\n    metersPerUnit = 1\n)\ndef Xform \"b\" (prepend apiSchemas = [\"PhysicsRigidBodyAPI\"])\n{{\n{body_children}\n}}\n"
    ))
}

fn only_shape(r: &Robot) -> &rigscan_core::Shape {
    let shapes = &r.links.iter().find(|l| l.name == "/b").unwrap().shapes;
    assert_eq!(shapes.len(), 1, "{shapes:?}");
    &shapes[0]
}

#[test]
fn a_cube_under_a_body_is_a_box_at_its_translation() {
    let r = shapes_of(
        r#"def Cube "c" {
    double size = 2
    double3 xformOp:translate = (0, 0, 1)
    uniform token[] xformOpOrder = ["xformOp:translate"]
}"#,
    );
    let s = only_shape(&r);
    assert_eq!(s.geometry, rigscan_core::Geometry::Box { size: [2.0; 3] });
    assert_eq!(s.role, rigscan_core::Role::Visual);
    assert!(near(s.origin.xyz[2], 1.0), "{:?}", s.origin);
}

#[test]
fn a_scale_in_the_prim_stack_stretches_the_geometry() {
    let r = shapes_of(
        r#"def Cube "c" {
    double3 xformOp:scale = (1, 2, 3)
    uniform token[] xformOpOrder = ["xformOp:scale"]
}"#,
    );
    assert_eq!(
        only_shape(&r).geometry,
        rigscan_core::Geometry::Box {
            size: [2.0, 4.0, 6.0]
        }
    );
}

#[test]
fn the_pose_is_taken_through_the_xforms_between_the_body_and_the_shape() {
    let r = shapes_of(
        r#"def Xform "g" {
    float3 xformOp:rotateXYZ = (0, 0, 90)
    uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]
    def Sphere "s" {
        double radius = 0.5
        double3 xformOp:translate = (1, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}"#,
    );
    let s = only_shape(&r);
    assert_eq!(s.geometry, rigscan_core::Geometry::Sphere { radius: 0.5 });
    assert!(
        near(s.origin.xyz[0], 0.0) && near(s.origin.xyz[1], 1.0),
        "{:?}",
        s.origin
    );
}

#[test]
fn a_body_that_is_moved_does_not_move_its_shapes_in_its_own_frame() {
    let r = robot(
        r#"#usda 1.0
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
{
    double3 xformOp:translate = (5, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]
    def Cube "c" {
        double3 xformOp:translate = (0, 0, 1)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"#,
    );
    let s = only_shape(&r);
    assert!(
        near(s.origin.xyz[0], 0.0) && near(s.origin.xyz[2], 1.0),
        "{:?}",
        s.origin
    );
}

#[test]
fn a_cylinder_along_x_is_turned_from_z_onto_x() {
    let r = shapes_of(
        r#"def Cylinder "c" {
    double radius = 0.1
    double height = 2
    uniform token axis = "X"
}"#,
    );
    let s = only_shape(&r);
    assert_eq!(
        s.geometry,
        rigscan_core::Geometry::Cylinder {
            radius: 0.1,
            length: 2.0
        }
    );
    let half_pi = std::f64::consts::FRAC_PI_2;
    assert!(near(s.origin.rpy[1], half_pi), "{:?}", s.origin);
}

#[test]
fn a_collision_prim_is_a_shape_not_a_loss_unless_it_is_hidden() {
    let r = shapes_of(
        r#"def Sphere "both" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {}
def Sphere "only" (prepend apiSchemas = ["PhysicsCollisionAPI"]) { uniform token purpose = "guide" }"#,
    );
    let roles: Vec<_> = r
        .links
        .iter()
        .find(|l| l.name == "/b")
        .unwrap()
        .shapes
        .iter()
        .map(|s| s.role)
        .collect();
    assert_eq!(
        roles,
        [rigscan_core::Role::Both, rigscan_core::Role::Collision]
    );
    assert!(
        r.unmapped.iter().all(|u| u.what != "collision"),
        "{:?}",
        r.unmapped
    );
}

#[test]
fn a_collision_prim_that_is_not_a_primitive_is_still_listed_as_lost() {
    let r = shapes_of(r#"def Mesh "m" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {}"#);
    assert!(r
        .links
        .iter()
        .find(|l| l.name == "/b")
        .unwrap()
        .shapes
        .is_empty());
    assert!(r.unmapped.iter().any(|u| u.what == "collision"));
}

#[test]
fn shape_lengths_follow_the_stage_units() {
    let r = robot(
        r#"#usda 1.0
(
    metersPerUnit = 0.01
)
def Xform "b" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
{
    def Cube "c" {
        double size = 100
        double3 xformOp:translate = (0, 0, 50)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"#,
    );
    let s = only_shape(&r);
    assert_eq!(s.geometry, rigscan_core::Geometry::Box { size: [1.0; 3] });
    assert!(near(s.origin.xyz[2], 0.5), "{:?}", s.origin);
}
