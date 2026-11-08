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

fn run(body: &str) -> Vec<rigscan_core::Finding> {
    lint(&parse_urdf(&format!("<robot name=\"t\">\n{body}\n</robot>")).unwrap())
}

fn only(findings: &[rigscan_core::Finding], rule: &str) -> Vec<rigscan_core::Finding> {
    findings.iter().filter(|f| f.rule == rule).cloned().collect()
}

fn joint(name: &str, parent: &str, child: &str) -> String {
    format!(
        "<joint name=\"{name}\" type=\"fixed\"><parent link=\"{parent}\"/><child link=\"{child}\"/></joint>"
    )
}

#[test]
fn rs002_duplicate_link_names() {
    let f = only(&run("<link name=\"a\"/>\n<link name=\"a\"/>"), "RS002");
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].severity, Severity::Critical);
    assert_eq!(f[0].span, Span { line: 3, col: 1 });
}

#[test]
fn rs003_duplicate_joint_names() {
    let body = format!(
        "<link name=\"a\"/><link name=\"b\"/><link name=\"c\"/>\n{}\n{}",
        joint("j", "a", "b"),
        joint("j", "b", "c")
    );
    let f = only(&run(&body), "RS003");
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].span, Span { line: 4, col: 1 });
}

#[test]
fn rs004_link_with_two_parents() {
    let body = format!(
        "<link name=\"a\"/><link name=\"b\"/><link name=\"c\"/>\n{}\n{}",
        joint("j1", "a", "c"),
        joint("j2", "b", "c")
    );
    let f = only(&run(&body), "RS004");
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].severity, Severity::Critical);
    assert_eq!(f[0].span, Span { line: 4, col: 1 });
}

#[test]
fn rs005_kinematic_loop() {
    let body = format!(
        "<link name=\"a\"/><link name=\"b\"/>\n{}\n{}",
        joint("j1", "a", "b"),
        joint("j2", "b", "a")
    );
    let f = only(&run(&body), "RS005");
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].severity, Severity::Critical);
}

#[test]
fn rs006_multiple_root_links() {
    let body = format!(
        "<link name=\"a\"/><link name=\"b\"/>\n<link name=\"c\"/>\n{}",
        joint("j1", "a", "b")
    );
    let f = only(&run(&body), "RS006");
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].severity, Severity::Critical);
    assert_eq!(f[0].span, Span { line: 3, col: 1 });
}

#[test]
fn branching_tree_has_no_findings() {
    let body = format!(
        "<link name=\"a\"/><link name=\"b\"/><link name=\"c\"/>\n{}\n{}",
        joint("j1", "a", "b"),
        joint("j2", "a", "c")
    );
    assert!(run(&body).is_empty());
}
