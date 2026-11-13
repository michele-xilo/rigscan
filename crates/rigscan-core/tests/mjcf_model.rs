use rigscan_core::{
    lint, parse_mjcf, Format, Geometry, IncludeLoader, Included, JointKind, Robot, Role, Span,
};

struct NoFiles;

impl IncludeLoader for NoFiles {
    fn load(&self, _file: &str, _from: Option<&str>) -> Option<Included> {
        None
    }
}

fn robot(src: &str) -> Robot {
    parse_mjcf(src, &NoFiles).unwrap().to_robot().unwrap()
}

fn rules(r: &Robot) -> Vec<&'static str> {
    lint(r).iter().map(|f| f.rule).collect()
}

fn link_names(r: &Robot) -> Vec<&str> {
    r.links.iter().map(|l| l.name.as_str()).collect()
}

/// (name, parent, child) of every joint.
fn edges(r: &Robot) -> Vec<(&str, &str, &str)> {
    r.joints
        .iter()
        .map(|j| (j.name.as_str(), j.parent.as_str(), j.child.as_str()))
        .collect()
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

#[test]
fn robot_takes_its_name_and_format_from_the_file() {
    let r = robot("<mujoco model=\"arm\"><worldbody/></mujoco>");
    assert_eq!(r.format, Format::Mjcf);
    assert_eq!(r.name.as_deref(), Some("arm"));
    assert_eq!(link_names(&r), ["world"]);
}

#[test]
fn bodies_become_links_under_a_world_root() {
    let r = robot(
        r#"<mujoco><worldbody>
  <body name="a"><joint name="ja"/>
    <body name="b"><joint name="jb"/></body>
  </body>
</worldbody></mujoco>"#,
    );
    assert_eq!(link_names(&r), ["world", "a", "b"]);
    assert_eq!(edges(&r), [("ja", "world", "a"), ("jb", "a", "b")]);
    assert_eq!(r.links[1].span, Span { line: 2, col: 3 });
    assert_eq!(r.joints[0].span, Span { line: 2, col: 18 });
}

#[test]
fn several_joints_on_one_body_stay_separate_joints_between_the_same_links() {
    let r = robot(
        r#"<mujoco><worldbody>
  <body name="a"><joint name="slide" type="slide"/><joint name="turn"/></body>
</worldbody></mujoco>"#,
    );
    assert_eq!(link_names(&r), ["world", "a"]);
    assert_eq!(edges(&r), [("slide", "world", "a"), ("turn", "world", "a")]);
    assert!(rules(&r).is_empty(), "{:?}", lint(&r));
}

#[test]
fn body_without_a_joint_is_welded_to_its_parent_by_a_fixed_joint() {
    let r = robot("<mujoco><worldbody><body name=\"a\"/></worldbody></mujoco>");
    assert_eq!(edges(&r), [("a", "world", "a")]);
    assert_eq!(r.joints[0].kind, JointKind::Fixed);
    assert!(rules(&r).is_empty());
}

#[test]
fn fixed_joint_name_never_collides_with_a_real_joint() {
    let r = robot(
        r#"<mujoco><worldbody>
  <body name="a"><joint name="b"/><body name="b"/></body>
</worldbody></mujoco>"#,
    );
    let names: Vec<&str> = r.joints.iter().map(|j| j.name.as_str()).collect();
    let unique: std::collections::HashSet<_> = names.iter().collect();
    assert_eq!(unique.len(), names.len(), "{names:?}");
    assert!(rules(&r).is_empty(), "{:?}", lint(&r));
}

#[test]
fn unnamed_bodies_and_joints_get_distinct_names() {
    let r = robot(
        r#"<mujoco><worldbody>
  <body><joint/></body><body><joint/></body>
</worldbody></mujoco>"#,
    );
    assert_eq!(r.links.len(), 3);
    let l: std::collections::HashSet<_> = r.links.iter().map(|l| &l.name).collect();
    let j: std::collections::HashSet<_> = r.joints.iter().map(|j| &j.name).collect();
    assert_eq!((l.len(), j.len()), (3, 2));
    assert!(rules(&r).is_empty(), "{:?}", lint(&r));
}

fn kind_of(joint: &str) -> JointKind {
    let src = format!("<mujoco><worldbody><body name=\"a\">{joint}</body></worldbody></mujoco>");
    robot(&src).joints[0].kind.clone()
}

#[test]
fn joint_types_map_onto_neutral_kinds() {
    assert_eq!(kind_of("<joint range=\"-10 10\"/>"), JointKind::Revolute);
    assert_eq!(kind_of("<joint type=\"hinge\"/>"), JointKind::Continuous);
    assert_eq!(kind_of("<joint type=\"slide\"/>"), JointKind::Prismatic);
    assert_eq!(kind_of("<freejoint/>"), JointKind::Floating);
    assert_eq!(kind_of("<joint type=\"free\"/>"), JointKind::Floating);
    assert_eq!(
        kind_of("<joint type=\"ball\"/>"),
        JointKind::Other("ball".into())
    );
}

#[test]
fn hinge_range_is_read_in_degrees_by_default() {
    let r = robot("<mujoco><worldbody><body name=\"a\"><joint range=\"-90 180\"/></body></worldbody></mujoco>");
    let l = r.joints[0].limit.as_ref().unwrap();
    assert!(near(l.lower.unwrap(), -std::f64::consts::FRAC_PI_2));
    assert!(near(l.upper.unwrap(), std::f64::consts::PI));
}

#[test]
fn compiler_angle_radian_keeps_hinge_range_as_written() {
    let r = robot(
        "<mujoco><compiler angle=\"radian\"/><worldbody><body name=\"a\"><joint range=\"-1 2\"/></body></worldbody></mujoco>",
    );
    let l = r.joints[0].limit.as_ref().unwrap();
    assert_eq!((l.lower, l.upper), (Some(-1.0), Some(2.0)));
}

#[test]
fn slide_range_is_never_converted_and_unlimited_joints_have_no_limit() {
    let r = robot(
        r#"<mujoco><worldbody><body name="a">
  <joint name="s" type="slide" range="0 0.5"/><joint name="u" type="slide"/>
</body></worldbody></mujoco>"#,
    );
    let l = r.joints[0].limit.as_ref().unwrap();
    assert_eq!((l.lower, l.upper), (Some(0.0), Some(0.5)));
    assert!(r.joints[1].limit.is_none());
}

#[test]
fn explicit_limited_false_ignores_the_range() {
    let r = robot("<mujoco><worldbody><body name=\"a\"><joint limited=\"false\" range=\"-1 1\"/></body></worldbody></mujoco>");
    assert!(r.joints[0].limit.is_none());
    assert_eq!(r.joints[0].kind, JointKind::Continuous);
}

#[test]
fn slide_and_hinge_without_limits_are_not_reported_as_missing_limits() {
    let r = robot(
        r#"<mujoco><worldbody><body name="a">
  <joint type="slide"/><joint type="hinge" range="-1 1"/>
</body></worldbody></mujoco>"#,
    );
    assert!(rules(&r).is_empty(), "{:?}", lint(&r));
}

#[test]
fn lower_range_above_upper_is_reported_for_mjcf_too() {
    let r = robot("<mujoco><worldbody><body name=\"a\"><joint type=\"slide\" range=\"1 0\"/></body></worldbody></mujoco>");
    assert_eq!(rules(&r), ["RS012"]);
}

#[test]
fn body_position_becomes_the_joint_origin() {
    let r = robot(
        "<mujoco><worldbody><body name=\"a\" pos=\"1 2 3\"><joint/></body></worldbody></mujoco>",
    );
    assert_eq!(r.joints[0].origin.xyz, [1.0, 2.0, 3.0]);
}

#[test]
fn inertial_maps_mass_and_diagonal_inertia() {
    let r = robot(
        "<mujoco><worldbody><body name=\"a\"><inertial mass=\"2\" pos=\"0 0 0\" diaginertia=\"1 2 3\"/></body></worldbody></mujoco>",
    );
    let i = r.links[1].inertial.as_ref().unwrap();
    assert_eq!(i.mass, Some(2.0));
    let t = i.inertia.unwrap();
    assert_eq!(
        (t.ixx, t.iyy, t.izz, t.ixy, t.ixz, t.iyz),
        (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
    );
}

#[test]
fn inertial_maps_full_inertia_in_mujoco_order() {
    let r = robot(
        "<mujoco><worldbody><body name=\"a\"><inertial mass=\"1\" pos=\"0 0 0\" fullinertia=\"1 2 3 4 5 6\"/></body></worldbody></mujoco>",
    );
    let t = r.links[1].inertial.as_ref().unwrap().inertia.unwrap();
    assert_eq!(
        (t.ixx, t.iyy, t.izz, t.ixy, t.ixz, t.iyz),
        (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    );
}

#[test]
fn body_without_inertial_has_none_because_mujoco_infers_it_from_geoms() {
    let r =
        robot("<mujoco><worldbody><body name=\"a\"><geom size=\"1\"/></body></worldbody></mujoco>");
    assert!(r.links[1].inertial.is_none());
}

#[test]
fn physics_rules_run_on_mjcf_inertials() {
    let r = robot(
        "<mujoco><worldbody><body name=\"a\"><joint/><inertial mass=\"-1\" pos=\"0 0 0\" diaginertia=\"1 1 1\"/></body></worldbody></mujoco>",
    );
    assert_eq!(rules(&r), ["RS008"]);
}

#[test]
fn mesh_geoms_become_mesh_refs_joined_with_meshdir() {
    let r = robot(
        r#"<mujoco>
  <compiler meshdir="assets"/>
  <asset><mesh name="m" file="m.stl"/><mesh file="dir/other.obj"/></asset>
  <worldbody><body name="a">
    <geom type="mesh" mesh="m"/>
    <geom mesh="other"/>
  </body></worldbody>
</mujoco>"#,
    );
    let m: Vec<(&str, u32)> = r.links[1]
        .meshes()
        .map(|(file, span)| (file, span.line))
        .collect();
    assert_eq!(m, [("assets/m.stl", 5), ("assets/dir/other.obj", 6)]);
}

#[test]
fn unmapped_parts_are_kept_with_name_and_span() {
    let r = robot(
        r#"<mujoco>
  <worldbody><body name="a"><joint name="j"/><geom name="g" type="ellipsoid" size="1 1 1"/></body></worldbody>
  <actuator><motor name="m" joint="j"/></actuator>
  <tendon><fixed name="t"/></tendon>
</mujoco>"#,
    );
    let got: Vec<(&str, Option<&str>, u32)> = r
        .unmapped
        .iter()
        .map(|u| (u.what.as_str(), u.name.as_deref(), u.span.line))
        .collect();
    assert!(got.contains(&("geom", Some("g"), 2)), "{got:?}");
    assert!(got.contains(&("motor", Some("m"), 3)), "{got:?}");
    assert!(got.contains(&("fixed", Some("t"), 4)), "{got:?}");
}

#[test]
fn invalid_number_is_an_error_with_its_position() {
    let e = parse_mjcf(
        "<mujoco>\n<worldbody><body name=\"a\" pos=\"1 x 3\"/></worldbody></mujoco>",
        &NoFiles,
    )
    .unwrap()
    .to_robot()
    .unwrap_err();
    assert!(e.0.starts_with("2:"), "{}", e.0);
}

#[test]
fn healthy_robot_has_no_findings() {
    let r = robot(
        r#"<mujoco model="r">
  <default><joint range="-90 90"/></default>
  <worldbody>
    <body name="base" pos="0 0 1">
      <inertial mass="1" pos="0 0 0" diaginertia="0.1 0.1 0.1"/>
      <body name="arm"><joint axis="0 1 0"/><geom type="capsule" size="0.05"/></body>
    </body>
  </worldbody>
</mujoco>"#,
    );
    assert!(rules(&r).is_empty(), "{:?}", lint(&r));
}

#[test]
fn compiler_autolimits_false_needs_limited_true_for_a_range_to_count() {
    let r = robot(
        r#"<mujoco><compiler autolimits="false"/><worldbody><body name="a">
  <joint name="x" type="slide" range="0 1"/><joint name="y" type="slide" range="0 1" limited="true"/>
</body></worldbody></mujoco>"#,
    );
    assert!(r.joints[0].limit.is_none());
    assert!(r.joints[1].limit.is_some());
}

#[test]
fn bodies_inside_a_frame_are_reported_as_unmapped_not_dropped_silently() {
    let r = robot(
        "<mujoco><worldbody><frame name=\"f\"><body name=\"a\"/></frame></worldbody></mujoco>",
    );
    assert!(r.unmapped.iter().any(|u| u.what == "frame"));
}

#[test]
fn wrong_number_count_is_an_error_with_its_position() {
    let e = parse_mjcf(
        "<mujoco>\n<worldbody><body name=\"a\" pos=\"1 2\"/></worldbody></mujoco>",
        &NoFiles,
    )
    .unwrap()
    .to_robot()
    .unwrap_err();
    assert!(
        e.0.starts_with("2:") && e.0.contains("needs 3 numbers, found 2"),
        "{}",
        e.0
    );
}

#[test]
fn mesh_paths_without_meshdir_or_already_absolute_are_kept_as_written() {
    let r = robot(
        r#"<mujoco>
  <compiler meshdir="assets"/>
  <asset><mesh name="abs" file="/opt/m.stl"/></asset>
  <worldbody><geom mesh="abs"/></worldbody>
</mujoco>"#,
    );
    assert_eq!(r.links[0].meshes().next().unwrap().0, "/opt/m.stl");
    let r = robot(
        "<mujoco><asset><mesh name=\"m\" file=\"m.stl\"/></asset><worldbody><geom mesh=\"m\"/></worldbody></mujoco>",
    );
    assert_eq!(r.links[0].meshes().next().unwrap().0, "m.stl");
}

#[test]
fn mesh_asset_without_a_file_gives_geoms_no_mesh_ref() {
    let r = robot(
        "<mujoco><asset><mesh name=\"m\"/></asset><worldbody><geom mesh=\"m\"/></worldbody></mujoco>",
    );
    assert_eq!(r.links[0].meshes().count(), 0);
    assert!(r.unmapped.is_empty());
}

#[test]
fn elements_from_an_included_file_are_reported_at_the_include() {
    struct One;
    impl IncludeLoader for One {
        fn load(&self, file: &str, _from: Option<&str>) -> Option<Included> {
            (file == "arm.xml").then(|| Included {
                path: file.into(),
                text: "<mujoco>\n\n\n<body name=\"a\"><joint name=\"j\"/></body>\n</mujoco>".into(),
            })
        }
    }
    let r = parse_mjcf(
        "<mujoco>\n<worldbody>\n  <include file=\"arm.xml\"/>\n</worldbody>\n</mujoco>",
        &One,
    )
    .unwrap()
    .to_robot()
    .unwrap();
    let at = Span { line: 3, col: 3 };
    assert_eq!(r.links[1].span, at);
    assert_eq!(r.joints[0].span, at);
}

#[test]
fn hinge_and_slide_axis_default_to_z() {
    let r = robot(
        "<mujoco><worldbody><body name=\"a\"><joint name=\"h\"/><body name=\"b\"><joint name=\"s\" type=\"slide\"/></body></body></worldbody></mujoco>",
    );
    assert_eq!(r.joints[0].axis, Some([0.0, 0.0, 1.0]));
    assert_eq!(r.joints[1].axis, Some([0.0, 0.0, 1.0]));
}

#[test]
fn authored_and_default_class_axis_is_kept() {
    let r = robot(
        "<mujoco><default><joint axis=\"1 0 0\"/></default><worldbody><body name=\"a\"><joint name=\"h\"/><body name=\"b\"><joint name=\"s\" axis=\"0 1 0\"/></body></body></worldbody></mujoco>",
    );
    assert_eq!(r.joints[0].axis, Some([1.0, 0.0, 0.0]));
    assert_eq!(r.joints[1].axis, Some([0.0, 1.0, 0.0]));
}

#[test]
fn free_and_fixed_joints_have_no_axis() {
    let r = robot(
        "<mujoco><worldbody><body name=\"a\"><freejoint/></body><body name=\"b\"/></worldbody></mujoco>",
    );
    assert_eq!(r.joints[0].axis, None);
    assert_eq!(r.joints[1].axis, None);
}

#[test]
fn damping_and_frictionloss_become_dynamics() {
    let r = robot(
        "<mujoco><worldbody><body name=\"a\"><joint name=\"h\" damping=\"0.5\" frictionloss=\"0.1\"/><body name=\"b\"><joint name=\"s\" damping=\"2\"/></body><body name=\"c\"><joint name=\"t\"/></body></body></worldbody></mujoco>",
    );
    let d = r.joints[0].dynamics.clone().unwrap();
    assert_eq!((d.damping, d.friction), (Some(0.5), Some(0.1)));
    let d = r.joints[1].dynamics.clone().unwrap();
    assert_eq!((d.damping, d.friction), (Some(2.0), None));
    assert_eq!(r.joints[2].dynamics, None);
}

#[test]
fn inertial_pos_is_the_centre_of_mass() {
    let r = robot(
        "<mujoco><worldbody><body name=\"a\"><inertial pos=\"1 2 3\" mass=\"1\" diaginertia=\"1 1 1\"/></body></worldbody></mujoco>",
    );
    assert_eq!(
        r.links[1].inertial.as_ref().unwrap().origin.xyz,
        [1.0, 2.0, 3.0]
    );
}

fn shapes(geoms: &str) -> Vec<(Role, Geometry)> {
    robot(&format!(
        "<mujoco><worldbody><body name=\"a\">{geoms}</body></worldbody></mujoco>"
    ))
    .links[1]
        .shapes
        .iter()
        .map(|s| (s.role, s.geometry.clone()))
        .collect()
}

#[test]
fn primitive_geoms_become_shapes_with_full_extents() {
    let got = shapes(
        "<geom size=\"0.5\"/>\
         <geom type=\"box\" size=\"1 2 3\"/>\
         <geom type=\"cylinder\" size=\"0.1 2\"/>\
         <geom type=\"capsule\" size=\"0.2 1.5\"/>",
    );
    assert_eq!(
        got,
        [
            (Role::Both, Geometry::Sphere { radius: 0.5 }),
            (
                Role::Both,
                Geometry::Box {
                    size: [2.0, 4.0, 6.0]
                }
            ),
            (
                Role::Both,
                Geometry::Cylinder {
                    radius: 0.1,
                    length: 4.0
                }
            ),
            (
                Role::Both,
                Geometry::Capsule {
                    radius: 0.2,
                    length: 3.0
                }
            ),
        ]
    );
}

#[test]
fn a_geom_with_no_contacts_is_visual_only() {
    let got = shapes(
        "<geom size=\"1\" contype=\"0\" conaffinity=\"0\"/>\
         <geom size=\"1\" contype=\"0\"/>\
         <geom size=\"1\" conaffinity=\"0\"/>",
    );
    let roles: Vec<_> = got.iter().map(|g| g.0).collect();
    assert_eq!(roles, [Role::Visual, Role::Both, Role::Both]);
}

#[test]
fn a_geom_in_a_hidden_group_is_collision_only() {
    let got = shapes(
        "<geom size=\"1\" group=\"3\"/>\
         <geom size=\"1\" group=\"5\"/>\
         <geom size=\"1\" group=\"2\"/>\
         <geom size=\"1\"/>",
    );
    let roles: Vec<_> = got.iter().map(|g| g.0).collect();
    assert_eq!(
        roles,
        [Role::Collision, Role::Collision, Role::Both, Role::Both]
    );
}

#[test]
fn the_group_of_a_geom_is_read_through_its_default_class() {
    let r = robot(
        "<mujoco><default><default class=\"collision\"><geom group=\"3\"/></default></default>\
         <worldbody><body name=\"a\"><geom size=\"1\" class=\"collision\"/></body></worldbody></mujoco>",
    );
    assert_eq!(r.links[1].shapes[0].role, Role::Collision);
}

fn same_point(a: [f64; 3], b: [f64; 3]) {
    assert!(
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9),
        "{a:?} != {b:?}"
    );
}

#[test]
fn a_joint_pos_moves_the_link_frame_to_the_joint() {
    let r = robot(
        r#"<mujoco><worldbody>
  <body name="a" pos="1 0 0">
    <joint name="j" pos="0 0 0.5"/>
    <inertial pos="0 0 0.5" mass="1" diaginertia="1 1 1"/>
    <geom size="1" pos="0 0 0.5"/>
    <body name="b" pos="0 0 1"><joint name="k"/></body>
  </body>
</worldbody></mujoco>"#,
    );
    let joint = |n: &str| r.joints.iter().find(|j| j.name == n).unwrap();
    same_point(joint("j").origin.xyz, [1.0, 0.0, 0.5]);
    same_point(r.links[1].shapes[0].origin.xyz, [0.0; 3]);
    same_point(r.links[1].inertial.as_ref().unwrap().origin.xyz, [0.0; 3]);
    same_point(joint("k").origin.xyz, [0.0, 0.0, 0.5]);
}

#[test]
fn a_joint_pos_is_rotated_by_the_body_orientation() {
    let r = robot(
        r#"<mujoco><worldbody>
  <body name="a" euler="0 0 90"><joint name="j" pos="1 0 0"/><geom size="1" pos="1 0 0"/></body>
</worldbody></mujoco>"#,
    );
    same_point(r.joints[0].origin.xyz, [0.0, 1.0, 0.0]);
    same_point(r.links[1].shapes[0].origin.xyz, [0.0; 3]);
}

#[test]
fn geom_pos_is_the_shape_origin_and_default_class_values_apply() {
    let r = robot(
        "<mujoco><default><geom size=\"2\" pos=\"1 2 3\"/></default><worldbody><body name=\"a\"><geom/></body></worldbody></mujoco>",
    );
    let s = &r.links[1].shapes[0];
    assert_eq!(s.geometry, Geometry::Sphere { radius: 2.0 });
    assert_eq!(s.origin.xyz, [1.0, 2.0, 3.0]);
}

#[test]
fn mesh_geoms_carry_the_asset_file_and_scale() {
    let r = robot(
        r#"<mujoco>
  <compiler meshdir="assets"/>
  <asset><mesh name="m" file="m.stl" scale="1 2 3"/><mesh name="n" file="n.stl"/></asset>
  <worldbody><body name="a">
    <geom type="mesh" mesh="m"/>
    <geom mesh="n"/>
  </body></worldbody>
</mujoco>"#,
    );
    let g: Vec<_> = r.links[1]
        .shapes
        .iter()
        .map(|s| s.geometry.clone())
        .collect();
    assert_eq!(
        g,
        [
            Geometry::Mesh {
                filename: "assets/m.stl".into(),
                scale: [1.0, 2.0, 3.0]
            },
            Geometry::Mesh {
                filename: "assets/n.stl".into(),
                scale: [1.0; 3]
            },
        ]
    );
}

#[test]
fn geoms_the_model_cannot_hold_stay_unmapped() {
    let r = robot(
        "<mujoco><worldbody><body name=\"a\">\
         <geom name=\"e\" type=\"ellipsoid\" size=\"1 2 3\"/>\
         <geom name=\"p\" type=\"plane\" size=\"1 1 1\"/>\
         <geom name=\"f\" type=\"capsule\" size=\"1\" fromto=\"0 0 0 1 0 0\"/>\
         <geom name=\"s\" type=\"box\" size=\"1 2\"/>\
         </body></worldbody></mujoco>",
    );
    assert!(r.links[1].shapes.is_empty());
    let lost: Vec<_> = r
        .unmapped
        .iter()
        .map(|u| (u.what.as_str(), u.name.as_deref()))
        .collect();
    assert_eq!(
        lost,
        [
            ("geom", Some("e")),
            ("geom", Some("p")),
            ("geom", Some("f")),
            ("geom", Some("s"))
        ]
    );
}

#[test]
fn invalid_geom_numbers_are_parse_errors_naming_the_position() {
    for geom in ["<geom size=\"x\"/>", "<geom size=\"1\" pos=\"1 2\"/>"] {
        let src = format!("<mujoco><worldbody>{geom}</worldbody></mujoco>");
        let err = parse_mjcf(&src, &NoFiles)
            .unwrap()
            .to_robot()
            .unwrap_err()
            .0;
        assert!(err.starts_with("1:"), "{geom}: {err}");
    }
}

#[test]
fn a_bad_mesh_scale_is_a_parse_error() {
    let src = "<mujoco><asset><mesh name=\"m\" file=\"m.stl\" scale=\"1 2\"/></asset><worldbody/></mujoco>";
    let err = parse_mjcf(src, &NoFiles).unwrap().to_robot().unwrap_err().0;
    assert!(err.starts_with("1:") && err.contains("scale"), "{err}");
}

fn body_rpy(attrs: &str, compiler: &str) -> [f64; 3] {
    let src = format!(
        "<mujoco>{compiler}<worldbody><body name=\"a\" {attrs}><joint name=\"j\"/></body></worldbody></mujoco>"
    );
    robot(&src).joints[0].origin.rpy
}

fn assert_rpy(got: [f64; 3], want: [f64; 3]) {
    for i in 0..3 {
        assert!((got[i] - want[i]).abs() < 1e-6, "{got:?} != {want:?}");
    }
}

const HALF_PI: f64 = std::f64::consts::FRAC_PI_2;

#[test]
fn body_orientation_becomes_the_joint_rpy() {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    assert_rpy(body_rpy("quat=\"1 0 0 0\"", ""), [0.0; 3]);
    assert_rpy(
        body_rpy(&format!("quat=\"{s} 0 0 {s}\""), ""),
        [0.0, 0.0, HALF_PI],
    );
    assert_rpy(body_rpy("axisangle=\"1 0 0 90\"", ""), [HALF_PI, 0.0, 0.0]);
    assert_rpy(body_rpy("euler=\"0 0 90\"", ""), [0.0, 0.0, HALF_PI]);
    assert_rpy(body_rpy("xyaxes=\"0 1 0 -1 0 0\"", ""), [0.0, 0.0, HALF_PI]);
    assert_rpy(body_rpy("zaxis=\"1 0 0\"", ""), [0.0, HALF_PI, 0.0]);
}

#[test]
fn a_quaternion_is_normalised_and_a_rotation_past_gimbal_lock_keeps_its_matrix() {
    assert_rpy(body_rpy("quat=\"2 0 0 0\"", ""), [0.0; 3]);
    assert_rpy(
        body_rpy("zaxis=\"0 0 -1\"", ""),
        [std::f64::consts::PI, 0.0, 0.0],
    );
}

#[test]
fn euler_follows_the_compiler_sequence_and_angle_unit() {
    assert_rpy(body_rpy("euler=\"90 0 90\"", ""), [0.0, -HALF_PI, HALF_PI]);
    assert_rpy(
        body_rpy("euler=\"90 0 90\"", "<compiler eulerseq=\"XYZ\"/>"),
        [HALF_PI, 0.0, HALF_PI],
    );
    assert_rpy(
        body_rpy(
            "euler=\"0 0 1.5707963267948966\"",
            "<compiler angle=\"radian\"/>",
        ),
        [0.0, 0.0, HALF_PI],
    );
}

#[test]
fn geom_and_inertial_orientation_become_their_origin_rpy() {
    let r = robot(
        "<mujoco><worldbody><body name=\"a\"><inertial pos=\"0 0 0\" mass=\"1\" diaginertia=\"1 1 1\" euler=\"0 0 90\"/>\
         <geom size=\"1\" axisangle=\"0 1 0 90\"/></body></worldbody></mujoco>",
    );
    assert_rpy(
        r.links[1].inertial.as_ref().unwrap().origin.rpy,
        [0.0, 0.0, HALF_PI],
    );
    assert_rpy(r.links[1].shapes[0].origin.rpy, [0.0, HALF_PI, 0.0]);
    assert!(r.unmapped.is_empty(), "{:?}", r.unmapped);
}

#[test]
fn invalid_orientation_is_a_parse_error() {
    for attrs in [
        "quat=\"0 0 0 0\"",
        "quat=\"1 0 0\"",
        "axisangle=\"0 0 0 90\"",
        "euler=\"0 0\"",
        "xyaxes=\"1 0 0 2 0 0\"",
        "zaxis=\"0 0 0\"",
    ] {
        let src = format!(
            "<mujoco><worldbody><body name=\"a\" {attrs}><joint name=\"j\"/></body></worldbody></mujoco>"
        );
        let err = parse_mjcf(&src, &NoFiles).unwrap().to_robot();
        assert!(err.is_err(), "{attrs}");
    }
    let bad_seq = "<mujoco><compiler eulerseq=\"xxq\"/><worldbody><body name=\"a\" euler=\"1 2 3\"><joint name=\"j\"/></body></worldbody></mujoco>";
    assert!(parse_mjcf(bad_seq, &NoFiles).unwrap().to_robot().is_err());
}
