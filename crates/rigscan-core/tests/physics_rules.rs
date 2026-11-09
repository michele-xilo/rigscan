use rigscan_core::{lint, parse_urdf, Element, Finding, Severity, Span};

fn findings(src: &str, rule: &str) -> Vec<Finding> {
    lint(&parse_urdf(src).unwrap())
        .into_iter()
        .filter(|f| f.rule == rule)
        .collect()
}

fn all(src: &str) -> Vec<Finding> {
    lint(&parse_urdf(src).unwrap())
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

fn link_with(inertial: &str) -> String {
    format!("<robot name=\"t\">\n  <link name=\"a\">\n    <inertial>{inertial}</inertial>\n  </link>\n</robot>")
}

fn tensor(ixx: f64, ixy: f64, iyy: f64, izz: f64) -> String {
    format!("<mass value=\"1\"/><inertia ixx=\"{ixx}\" ixy=\"{ixy}\" ixz=\"0\" iyy=\"{iyy}\" iyz=\"0\" izz=\"{izz}\"/>")
}

#[test]
fn rs008_negative_mass_is_critical_at_the_inertial_element() {
    let src = link_with("<mass value=\"-1\"/>");

    let f = findings(&src, "RS008");

    assert_eq!(f.len(), 1, "{f:?}");
    assert_eq!(f[0].severity, Severity::Critical);
    assert_eq!(f[0].element, Element::Link(0));
    assert_eq!(f[0].span, at(&src, "<inertial>"));
    assert!(
        f[0].message.contains("negative mass") && f[0].message.contains("`a`"),
        "{}",
        f[0].message
    );
}

#[test]
fn rs008_does_not_fire_for_positive_or_zero_mass() {
    assert!(findings(&link_with("<mass value=\"2\"/>"), "RS008").is_empty());
    assert!(findings(&link_with("<mass value=\"0\"/>"), "RS008").is_empty());
}

const MOVABLE: &str = "<robot name=\"t\">
  <link name=\"a\"/>
  <link name=\"b\"><inertial><mass value=\"0\"/></inertial></link>
  <joint name=\"j\" type=\"JOINT\"><parent link=\"a\"/><child link=\"b\"/>LIMIT</joint>
</robot>";

fn movable(kind: &str) -> String {
    MOVABLE.replace("JOINT", kind).replace(
        "LIMIT",
        "<limit lower=\"0\" upper=\"1\" effort=\"1\" velocity=\"1\"/>",
    )
}

#[test]
fn rs009_zero_mass_under_a_movable_joint_is_a_warning() {
    for kind in ["revolute", "continuous", "prismatic"] {
        let src = movable(kind);

        let f = findings(&src, "RS009");

        assert_eq!(f.len(), 1, "{kind}: {f:?}");
        assert_eq!(f[0].severity, Severity::Warning);
        assert_eq!(f[0].element, Element::Link(1));
        assert_eq!(f[0].span, at(&src, "<inertial>"));
        assert!(
            f[0].message.contains("zero mass") && f[0].message.contains("`j`"),
            "{}",
            f[0].message
        );
    }
}

#[test]
fn rs009_ignores_massless_frames_on_fixed_joints_and_roots() {
    assert!(findings(&movable("fixed"), "RS009").is_empty());
    let root = link_with("<mass value=\"0\"/>");
    assert!(findings(&root, "RS009").is_empty());
}

#[test]
fn rs010_negative_principal_moment_is_critical() {
    let src = link_with(&tensor(1.0, 0.0, 1.0, -1.0));

    let f = all(&src);

    assert_eq!(f.len(), 1, "{f:?}");
    assert_eq!(f[0].rule, "RS010");
    assert_eq!(f[0].severity, Severity::Critical);
    assert_eq!(f[0].element, Element::Link(0));
    assert_eq!(f[0].span, at(&src, "<inertial>"));
}

#[test]
fn rs010_uses_the_whole_tensor_not_just_the_diagonal() {
    // Positive diagonal, but the off-diagonal term makes one eigenvalue negative (-1).
    let src = link_with(&tensor(1.0, 2.0, 1.0, 1.0));

    assert_eq!(findings(&src, "RS010").len(), 1);
}

#[test]
fn rs010_ignores_a_tensor_that_is_numerically_zero() {
    let src = link_with(&tensor(1e-20, 0.0, 1e-20, -1e-20));

    assert!(all(&src).is_empty());
}

#[test]
fn rs011_triangle_inequality_violation_is_a_warning() {
    let src = link_with(&tensor(1.0, 0.0, 1.0, 3.0));

    let f = all(&src);

    assert_eq!(f.len(), 1, "{f:?}");
    assert_eq!(f[0].rule, "RS011");
    assert_eq!(f[0].severity, Severity::Warning);
    assert_eq!(f[0].element, Element::Link(0));
    assert_eq!(f[0].span, at(&src, "<inertial>"));
}

#[test]
fn rs011_uses_principal_moments_not_the_diagonal() {
    // Diagonal 1, 1, 1 passes, but the principal moments are 0.1, 1 and 1.9.
    let src = link_with(&tensor(1.0, 0.9, 1.0, 1.0));

    assert_eq!(findings(&src, "RS011").len(), 1);
}

#[test]
fn rs011_accepts_a_valid_body_and_the_flat_plate_boundary() {
    assert!(all(&link_with(&tensor(1.0, 0.0, 1.0, 1.0))).is_empty());
    assert!(all(&link_with(&tensor(1.0, 0.0, 1.0, 2.0))).is_empty());
}

fn joint_with(kind: &str, limit: &str) -> String {
    format!("<robot name=\"t\">\n  <link name=\"a\"/><link name=\"b\"/>\n  <joint name=\"j\" type=\"{kind}\"><parent link=\"a\"/><child link=\"b\"/>{limit}</joint>\n</robot>")
}

#[test]
fn rs012_lower_above_upper_is_critical_at_the_joint() {
    let src = joint_with(
        "revolute",
        "<limit lower=\"1\" upper=\"0\" effort=\"1\" velocity=\"1\"/>",
    );

    let f = all(&src);

    assert_eq!(f.len(), 1, "{f:?}");
    assert_eq!(f[0].rule, "RS012");
    assert_eq!(f[0].severity, Severity::Critical);
    assert_eq!(f[0].element, Element::Joint(0));
    assert_eq!(f[0].span, at(&src, "<joint"));
}

#[test]
fn rs012_accepts_ordered_and_equal_bounds() {
    for (lo, up) in [("0", "1"), ("1", "1")] {
        let limit = format!("<limit lower=\"{lo}\" upper=\"{up}\" effort=\"1\" velocity=\"1\"/>");
        assert!(all(&joint_with("revolute", &limit)).is_empty());
    }
}

#[test]
fn rs013_revolute_and_prismatic_joints_need_a_limit() {
    for kind in ["revolute", "prismatic"] {
        let src = joint_with(kind, "");

        let f = all(&src);

        assert_eq!(f.len(), 1, "{kind}: {f:?}");
        assert_eq!(f[0].rule, "RS013");
        assert_eq!(f[0].severity, Severity::Critical);
        assert_eq!(f[0].element, Element::Joint(0));
        assert_eq!(f[0].span, at(&src, "<joint"));
        assert!(f[0].message.contains("no limit"), "{}", f[0].message);
    }
}

#[test]
fn rs013_limit_without_effort_or_velocity_is_reported() {
    let src = joint_with("revolute", "<limit lower=\"0\" upper=\"1\"/>");

    let f = all(&src);

    assert_eq!(f.len(), 1, "{f:?}");
    assert_eq!(f[0].rule, "RS013");
    assert!(
        f[0].message.contains("effort") && f[0].message.contains("velocity"),
        "{}",
        f[0].message
    );
}

#[test]
fn rs013_does_not_require_limits_on_continuous_or_fixed_joints() {
    for kind in ["continuous", "fixed"] {
        assert!(all(&joint_with(kind, "")).is_empty(), "{kind}");
    }
}

#[test]
fn healthy_prismatic_joint_with_full_limit_has_no_findings() {
    let src = joint_with(
        "prismatic",
        "<limit lower=\"0\" upper=\"1\" effort=\"5\" velocity=\"2\"/>",
    );

    assert!(all(&src).is_empty());
}
