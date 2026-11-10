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
    findings
        .iter()
        .filter(|f| f.rule == rule)
        .cloned()
        .collect()
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

#[test]
fn self_closing_joint_is_reported_with_its_span() {
    let f = run("<link name=\"a\"/>\n<joint name=\"j\" type=\"fixed\"/>");

    assert_eq!(f.len(), 2, "{f:?}");
    assert!(f
        .iter()
        .all(|x| x.rule == "RS001" && x.span == Span { line: 3, col: 1 }));
    assert!(f[0].message.contains("no parent link"), "{}", f[0].message);
    assert!(f[1].message.contains("no child link"), "{}", f[1].message);
}

#[test]
fn self_closing_joint_does_not_swallow_the_next_joint() {
    let f = run("<link name=\"a\"/><link name=\"b\"/>\n<joint name=\"bad\"/>\n<joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"ghost\"/></joint>");

    assert!(
        f.iter()
            .any(|x| x.message.contains("`j`") && x.message.contains("ghost")),
        "{f:?}"
    );
    assert!(f.iter().any(|x| x.message.contains("`bad`")), "{f:?}");
}

#[test]
fn joint_without_parent_element_is_reported() {
    let f = run("<link name=\"a\"/><link name=\"b\"/>\n<joint name=\"j\" type=\"fixed\"><child link=\"b\"/></joint>");

    assert_eq!(f.len(), 1, "{f:?}");
    assert!(f[0].message.contains("no parent link"), "{}", f[0].message);
}

#[test]
fn rs007_robot_without_links_is_reported_at_the_robot_element() {
    let f = run("");

    assert_eq!(f.len(), 1, "{f:?}");
    assert_eq!(f[0].rule, "RS007");
    assert_eq!(f[0].severity, Severity::Critical);
    assert_eq!(f[0].span, Span { line: 1, col: 1 });
    assert!(f[0].message.contains("no links"), "{}", f[0].message);
}

#[test]
fn rs007_span_points_at_the_robot_element_not_the_file_start() {
    let src = "<?xml version=\"1.0\"?>\n<!-- generated -->\n  <robot name=\"t\"/>";

    let f = lint(&parse_urdf(src).unwrap());

    assert_eq!(f.len(), 1, "{f:?}");
    assert_eq!(f[0].rule, "RS007");
    assert_eq!(f[0].span, Span { line: 3, col: 3 });
}

#[test]
fn rs007_does_not_fire_for_a_robot_with_a_link() {
    assert!(only(&run("<link name=\"a\"/>"), "RS007").is_empty());
}

fn two_joints_same_parent(format: rigscan_core::Format) -> Vec<&'static str> {
    use rigscan_core::{Joint, JointKind, Link, Origin, Robot, Span};
    let span = Span { line: 1, col: 1 };
    let link = |n: &str| Link {
        name: n.into(),
        inertial: None,
        meshes: Vec::new(),
        span,
    };
    let joint = |n: &str, p: &str| Joint {
        name: n.into(),
        parent: p.into(),
        child: "c".into(),
        kind: JointKind::Fixed,
        origin: Origin::default(),
        limit: None,
        mimic: None,
        axis: None,
        dynamics: None,
        span,
    };
    let mut r = Robot::new(format);
    r.links = vec![link("p"), link("q"), link("c")];
    r.joints = vec![joint("j1", "p"), joint("j2", "p")];
    lint(&r).iter().map(|f| f.rule).collect()
}

#[test]
fn rs004_flags_repeated_joints_between_the_same_links_in_urdf() {
    assert!(two_joints_same_parent(rigscan_core::Format::Urdf).contains(&"RS004"));
}

#[test]
fn rs004_allows_repeated_joints_between_the_same_links_in_mjcf() {
    assert!(!two_joints_same_parent(rigscan_core::Format::Mjcf).contains(&"RS004"));
}

#[test]
fn rs004_flags_a_body_reached_from_two_parents_in_mjcf() {
    use rigscan_core::{Format, Joint, JointKind, Link, Origin, Robot, Span};
    let span = Span { line: 1, col: 1 };
    let link = |n: &str| Link {
        name: n.into(),
        inertial: None,
        meshes: Vec::new(),
        span,
    };
    let joint = |n: &str, p: &str| Joint {
        name: n.into(),
        parent: p.into(),
        child: "c".into(),
        kind: JointKind::Fixed,
        origin: Origin::default(),
        limit: None,
        mimic: None,
        axis: None,
        dynamics: None,
        span,
    };
    let mut r = Robot::new(Format::Mjcf);
    r.links = vec![link("p"), link("q"), link("c")];
    r.joints = vec![joint("j1", "p"), joint("j2", "q")];
    assert!(lint(&r).iter().any(|f| f.rule == "RS004"));
}
