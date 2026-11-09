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
