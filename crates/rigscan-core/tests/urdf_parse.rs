use rigscan_core::{parse_urdf, Format, Span};

const URDF: &str = r#"<robot name="demo">
  <link name="base"/>
  <link name="arm"/>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
  </joint>
</robot>"#;

#[test]
fn parses_links_and_joints_with_spans() {
    let robot = parse_urdf(URDF).unwrap();

    let names: Vec<_> = robot.links.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(names, ["base", "arm"]);
    assert_eq!(robot.links[1].span, Span { line: 3, col: 3 });

    let j = &robot.joints[0];
    assert_eq!(
        (j.name.as_str(), j.parent.as_str(), j.child.as_str()),
        ("j1", "base", "arm")
    );
    assert_eq!(j.span, Span { line: 4, col: 3 });
}

#[test]
fn reads_format_and_robot_name() {
    let robot = parse_urdf(URDF).unwrap();

    assert_eq!(robot.format, Format::Urdf);
    assert_eq!(robot.name.as_deref(), Some("demo"));
}

const WITH_TRANSMISSION: &str = r#"<robot name="demo">
  <link name="base"/>
  <link name="arm"/>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
  <transmission name="t1">
    <joint name="j1"><hardwareInterface>EffortJointInterface</hardwareInterface></joint>
  </transmission>
</robot>"#;

#[test]
fn transmission_joint_reference_is_not_a_joint() {
    let robot = parse_urdf(WITH_TRANSMISSION).unwrap();

    assert_eq!(robot.joints.len(), 1);
    assert_eq!(robot.joints[0].parent, "base");
}

#[test]
fn transmission_does_not_cause_findings() {
    let robot = parse_urdf(WITH_TRANSMISSION).unwrap();

    assert!(rigscan_core::lint(&robot).is_empty());
}

#[test]
fn document_without_robot_element_is_an_error() {
    for src in ["", "   \n", "<html><body/></html>"] {
        assert!(parse_urdf(src).is_err(), "accepted {src:?}");
    }
}

#[test]
fn span_columns_count_characters_not_bytes() {
    let src = "<robot name=\"r\">\n<link name=\"é\"/><link name=\"b\"/>\n</robot>";
    let robot = parse_urdf(src).unwrap();

    // `<link name="é"/>` is 16 characters but 17 bytes.
    assert_eq!(robot.links[0].span, Span { line: 2, col: 1 });
    assert_eq!(robot.links[1].span, Span { line: 2, col: 17 });
}

#[test]
fn unclosed_element_at_end_of_input_is_a_parse_error() {
    let err = parse_urdf("<robot name=\"t\"><link name=\"a\"/>")
        .unwrap_err()
        .0;

    assert!(err.contains("unclosed"), "{err}");
}

fn lost(src: &str) -> Vec<(String, Option<String>, u32, u32)> {
    parse_urdf(src)
        .unwrap()
        .unmapped
        .into_iter()
        .map(|u| (u.what, u.name, u.span.line, u.span.col))
        .collect()
}

fn entry(
    what: &str,
    name: Option<&str>,
    line: u32,
    col: u32,
) -> (String, Option<String>, u32, u32) {
    (what.to_string(), name.map(str::to_string), line, col)
}

#[test]
fn a_robot_of_mapped_elements_loses_nothing() {
    let r = parse_urdf(
        r#"<robot name="r">
  <link name="a"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
  <link name="b"/>
  <joint name="j" type="revolute">
    <parent link="a"/><child link="b"/><origin xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>"#,
    )
    .unwrap();
    assert!(r.unmapped.is_empty(), "{:?}", r.unmapped);
}

#[test]
fn robot_level_elements_are_recorded_without_a_name() {
    let got = lost(
        "<robot name=\"r\">\n  <link name=\"a\"/>\n  <material name=\"m\"/>\n  <transmission name=\"t\"><joint name=\"a\"/></transmission>\n  <gazebo/>\n</robot>",
    );
    assert_eq!(
        got,
        [
            entry("material", None, 3, 3),
            entry("transmission", None, 4, 3),
            entry("gazebo", None, 5, 3),
        ]
    );
}

#[test]
fn link_visual_and_collision_are_recorded_under_the_link() {
    let got = lost(
        "<robot name=\"r\">\n  <link name=\"a\">\n    <visual><geometry><box size=\"1 1 1\"/></geometry></visual>\n    <collision><geometry><mesh filename=\"a.stl\"/></geometry></collision>\n  </link>\n</robot>",
    );
    assert_eq!(
        got,
        [
            entry("visual", Some("a"), 3, 5),
            entry("collision", Some("a"), 4, 5),
        ]
    );
}

#[test]
fn joint_children_the_model_cannot_hold_are_recorded_under_the_joint() {
    let got = lost(
        "<robot name=\"r\">\n  <link name=\"a\"/><link name=\"b\"/>\n  <joint name=\"j\" type=\"revolute\">\n    <parent link=\"a\"/><child link=\"b\"/>\n    <axis xyz=\"0 0 1\"/>\n    <dynamics damping=\"1\"/>\n    <safety_controller/>\n  </joint>\n</robot>",
    );
    assert_eq!(
        got,
        [
            entry("axis", Some("j"), 5, 5),
            entry("dynamics", Some("j"), 6, 5),
            entry("safety_controller", Some("j"), 7, 5),
        ]
    );
}

#[test]
fn an_inertial_origin_is_recorded_under_the_link() {
    let got = lost(
        "<robot name=\"r\">\n  <link name=\"a\">\n    <inertial><origin xyz=\"0 0 1\"/><mass value=\"1\"/></inertial>\n  </link>\n</robot>",
    );
    assert_eq!(got, [entry("inertial/origin", Some("a"), 3, 15)]);
}

#[test]
fn a_self_closing_joint_still_names_its_unmapped_children() {
    let got = lost(
        "<robot name=\"r\">\n  <link name=\"a\"/><link name=\"b\"/>\n  <joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/><axis/></joint>\n</robot>",
    );
    assert_eq!(got, [entry("axis", Some("j"), 3, 67)]);
}
