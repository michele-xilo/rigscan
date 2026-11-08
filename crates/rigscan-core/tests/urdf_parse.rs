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
