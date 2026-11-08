use rigscan_core::{lint, parse_urdf, Severity, Span};

#[test]
fn rs001_joint_references_missing_link() {
    let src = r#"<robot name="demo">
  <link name="base"/>
  <joint name="j1" type="fixed">
    <parent link="base"/>
    <child link="ghost"/>
  </joint>
</robot>"#;
    let findings = lint(&parse_urdf(src).unwrap());

    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].rule, "RS001");
    assert_eq!(findings[0].severity, Severity::Critical);
    assert_eq!(findings[0].span, Span { line: 3, col: 3 });
    assert!(findings[0].message.contains("ghost"));
}

#[test]
fn healthy_robot_has_no_findings() {
    let src = r#"<robot name="demo">
  <link name="base"/><link name="arm"/>
  <joint name="j1" type="fixed"><parent link="base"/><child link="arm"/></joint>
</robot>"#;
    assert!(lint(&parse_urdf(src).unwrap()).is_empty());
}
