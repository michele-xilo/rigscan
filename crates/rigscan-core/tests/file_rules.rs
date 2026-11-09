use rigscan_core::{
    lint, lint_with, parse_urdf, Element, Finding, Resolution, Resolver, Severity, Span,
};

/// Reports `Found` for the listed references, `Unknown` for `package://other/`, `Missing` otherwise.
struct Files(&'static [&'static str]);

impl Resolver for Files {
    fn resolve(&self, uri: &str) -> Resolution {
        if self.0.contains(&uri) {
            Resolution::Found
        } else if uri.starts_with("package://other/") {
            Resolution::Unknown
        } else {
            Resolution::Missing
        }
    }
}

fn findings(src: &str, rule: &str, resolver: &dyn Resolver) -> Vec<Finding> {
    lint_with(&parse_urdf(src).unwrap(), resolver)
        .into_iter()
        .filter(|f| f.rule == rule)
        .collect()
}

/// Line and column of the first `needle`, counting characters, so tests do not hand-count.
fn at(src: &str, needle: &str) -> Span {
    let byte = src.find(needle).unwrap();
    let before = &src[..byte];
    Span {
        line: before.matches('\n').count() as u32 + 1,
        col: before.rsplit('\n').next().unwrap().chars().count() as u32 + 1,
    }
}

fn link_with_meshes(visual: &str, collision: &str) -> String {
    format!(
        "<robot name=\"t\">\n  <link name=\"a\">\n    <visual><geometry><mesh filename=\"{visual}\"/></geometry></visual>\n    <collision><geometry><mesh filename=\"{collision}\"/></geometry></collision>\n  </link>\n</robot>"
    )
}

#[test]
fn rs014_missing_mesh_is_critical_at_the_mesh_element() {
    let src = link_with_meshes("meshes/a.stl", "meshes/a.stl");

    let f = findings(&src, "RS014", &Files(&[]));

    assert_eq!(f.len(), 2, "{f:?}");
    assert_eq!(f[0].severity, Severity::Critical);
    assert_eq!(f[0].element, Element::Link(0));
    assert_eq!(f[0].span, at(&src, "<mesh"));
    assert!(f[0].message.contains("meshes/a.stl"), "{}", f[0].message);
}

#[test]
fn rs014_reports_each_missing_mesh_separately() {
    let src = link_with_meshes("meshes/v.stl", "meshes/c.stl");

    let f = findings(&src, "RS014", &Files(&["meshes/v.stl"]));

    assert_eq!(f.len(), 1, "{f:?}");
    assert!(f[0].message.contains("meshes/c.stl"), "{}", f[0].message);
}

#[test]
fn rs014_existing_meshes_are_clean() {
    let src = link_with_meshes("meshes/v.stl", "meshes/c.stl");

    assert!(findings(&src, "RS014", &Files(&["meshes/v.stl", "meshes/c.stl"])).is_empty());
}

#[test]
fn rs014_unknown_resolution_is_not_a_finding() {
    let src = link_with_meshes("package://other/a.stl", "package://other/b.stl");

    assert!(findings(&src, "RS014", &Files(&[])).is_empty());
}

#[test]
fn lint_without_a_resolver_does_not_check_meshes() {
    let robot = parse_urdf(&link_with_meshes("meshes/a.stl", "meshes/b.stl")).unwrap();

    assert!(lint(&robot).iter().all(|f| f.rule != "RS014"));
}

fn mimic_robot(mimic: &str) -> String {
    format!(
        "<robot name=\"t\">\n  <link name=\"a\"/>\n  <link name=\"b\"/>\n  <link name=\"c\"/>\n  <joint name=\"j1\" type=\"revolute\">\n    <parent link=\"a\"/><child link=\"b\"/>\n    <limit lower=\"0\" upper=\"1\" effort=\"1\" velocity=\"1\"/>\n  </joint>\n  <joint name=\"j2\" type=\"revolute\">\n    <parent link=\"b\"/><child link=\"c\"/>\n    <limit lower=\"0\" upper=\"1\" effort=\"1\" velocity=\"1\"/>\n    {mimic}\n  </joint>\n</robot>"
    )
}

#[test]
fn rs015_mimic_of_undefined_joint_is_critical_at_the_mimic_element() {
    let src = mimic_robot("<mimic joint=\"ghost\"/>");

    let f = findings(&src, "RS015", &Files(&[]));

    assert_eq!(f.len(), 1, "{f:?}");
    assert_eq!(f[0].severity, Severity::Critical);
    assert_eq!(f[0].element, Element::Joint(1));
    assert_eq!(f[0].span, at(&src, "<mimic"));
    assert!(f[0].message.contains("ghost"), "{}", f[0].message);
}

#[test]
fn rs015_mimic_of_itself_is_critical() {
    let src = mimic_robot("<mimic joint=\"j2\"/>");

    let f = findings(&src, "RS015", &Files(&[]));

    assert_eq!(f.len(), 1, "{f:?}");
    assert!(f[0].message.contains("itself"), "{}", f[0].message);
}

#[test]
fn rs015_mimic_of_another_joint_is_clean() {
    let src = mimic_robot("<mimic joint=\"j1\" multiplier=\"2\"/>");

    assert!(findings(&src, "RS015", &Files(&[])).is_empty());
}
