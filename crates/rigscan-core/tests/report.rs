use rigscan_core::{
    lint, parse_urdf, report, Element, Finding, Report, Robot, Severity, Span, Status, Verdict,
};

fn parsed(src: &str) -> Robot {
    parse_urdf(src).unwrap()
}

fn status_of(r: &Report, e: Element) -> Status {
    r.elements.iter().find(|x| x.element == e).unwrap().status
}

fn manual(severity: Severity, element: Element) -> Finding {
    Finding {
        rule: "TEST",
        severity,
        message: String::new(),
        span: Span { line: 1, col: 1 },
        element,
    }
}

const HEALTHY: &str = r#"<robot name="t">
  <link name="a"/><link name="b"/>
  <joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>
</robot>"#;

#[test]
fn healthy_robot_lists_every_link_and_joint_as_healthy() {
    let robot = parsed(HEALTHY);

    let r = report(&robot, &lint(&robot));

    let names: Vec<&str> = r.elements.iter().map(|e| e.name).collect();
    assert_eq!(names, ["a", "b", "j"]);
    assert!(r.elements.iter().all(|e| e.status == Status::Healthy));
    assert_eq!(
        r.verdict,
        Verdict {
            healthy: 3,
            warnings: 0,
            critical: 0
        }
    );
}

#[test]
fn finding_marks_its_joint_critical_and_leaves_the_rest_healthy() {
    let robot = parsed(
        r#"<robot name="t"><link name="a"/>
  <joint name="j" type="fixed"><parent link="a"/><child link="ghost"/></joint></robot>"#,
    );

    let r = report(&robot, &lint(&robot));

    assert_eq!(status_of(&r, Element::Joint(0)), Status::Critical);
    assert_eq!(status_of(&r, Element::Link(0)), Status::Healthy);
    assert_eq!(
        r.verdict,
        Verdict {
            healthy: 1,
            warnings: 0,
            critical: 1
        }
    );
}

#[test]
fn duplicate_link_marks_only_the_second_definition() {
    let robot = parsed(r#"<robot name="t"><link name="a"/><link name="a"/></robot>"#);

    let r = report(&robot, &lint(&robot));

    assert_eq!(status_of(&r, Element::Link(0)), Status::Healthy);
    assert_eq!(status_of(&r, Element::Link(1)), Status::Critical);
}

#[test]
fn robot_without_links_lists_only_the_robot_element() {
    let robot = parsed(r#"<robot name="empty"/>"#);

    let r = report(&robot, &lint(&robot));

    assert_eq!(r.elements.len(), 1);
    assert_eq!(r.elements[0].element, Element::Robot);
    assert_eq!(r.elements[0].name, "empty");
    assert_eq!(r.elements[0].status, Status::Critical);
    assert_eq!(
        r.verdict,
        Verdict {
            healthy: 0,
            warnings: 0,
            critical: 1
        }
    );
}

#[test]
fn warning_only_element_is_a_warning() {
    let robot = parsed(HEALTHY);
    let findings = [manual(Severity::Warning, Element::Link(0))];

    let r = report(&robot, &findings);

    assert_eq!(status_of(&r, Element::Link(0)), Status::Warning);
    assert_eq!(
        r.verdict,
        Verdict {
            healthy: 2,
            warnings: 1,
            critical: 0
        }
    );
}

#[test]
fn worst_severity_wins_whatever_the_order() {
    let robot = parsed(HEALTHY);
    for order in [
        [Severity::Warning, Severity::Critical],
        [Severity::Critical, Severity::Warning],
    ] {
        let findings = order.map(|s| manual(s, Element::Link(0)));

        let r = report(&robot, &findings);

        assert_eq!(status_of(&r, Element::Link(0)), Status::Critical);
        assert_eq!(r.verdict.critical, 1);
        assert_eq!(r.verdict.warnings, 0);
    }
}
