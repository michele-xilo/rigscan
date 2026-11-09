//! Lint rules. Each rule reads a [`Robot`] and reports findings with source spans.

use crate::files::Resolver;
use crate::model::{Element, Format, Joint, Link, Robot, Span};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    Warning,
    Critical,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Finding {
    pub rule: &'static str,
    pub severity: Severity,
    pub message: String,
    pub span: Span,
    pub element: Element,
}

/// Run all lint rules over a parsed robot. Mesh files are not looked up: see [`lint_with`].
pub fn lint(robot: &Robot) -> Vec<Finding> {
    lint_with(robot, &crate::files::Unresolved)
}

/// Like [`lint`], and also checks that the files the robot references exist, asking `resolver`.
pub fn lint_with(robot: &Robot, resolver: &dyn Resolver) -> Vec<Finding> {
    let mut findings = Vec::new();
    // RS001: joint references a link that is not defined.
    let link_names: std::collections::HashSet<&str> =
        robot.links.iter().map(|l| l.name.as_str()).collect();
    for (i, j) in robot.joints.iter().enumerate() {
        for (role, name) in [("parent", &j.parent), ("child", &j.child)] {
            if !link_names.contains(name.as_str()) {
                let message = if name.is_empty() {
                    format!("joint `{}` has no {role} link", j.name)
                } else {
                    format!("joint `{}` {role} link `{name}` is not defined", j.name)
                };
                findings.push(Finding {
                    rule: "RS001",
                    severity: Severity::Critical,
                    message,
                    span: j.span,
                    element: Element::Joint(i),
                });
            }
        }
    }
    // RS007: a robot without links has nothing to check, so every other rule passes vacuously.
    if robot.links.is_empty() {
        findings.push(critical(
            "RS007",
            "robot has no links".into(),
            robot.span,
            Element::Robot,
        ));
    }
    duplicate_names(robot, &mut findings);
    crate::physics::physics(robot, &mut findings);
    crate::files::files(robot, resolver, &mut findings);
    tree_shape(robot, &mut findings);
    findings
}

pub(crate) fn critical(
    rule: &'static str,
    message: String,
    span: Span,
    element: Element,
) -> Finding {
    Finding {
        rule,
        severity: Severity::Critical,
        message,
        span,
        element,
    }
}

pub(crate) fn warning(
    rule: &'static str,
    message: String,
    span: Span,
    element: Element,
) -> Finding {
    Finding {
        severity: Severity::Warning,
        ..critical(rule, message, span, element)
    }
}

/// RS002 / RS003: names must be unique within links and within joints.
fn duplicate_names(robot: &Robot, out: &mut Vec<Finding>) {
    use std::collections::HashSet;
    let mut seen = HashSet::new();
    for (i, l) in robot.links.iter().enumerate() {
        if !seen.insert(l.name.as_str()) {
            out.push(critical(
                "RS002",
                format!("link `{}` is defined more than once", l.name),
                l.span,
                Element::Link(i),
            ));
        }
    }
    seen.clear();
    for (i, j) in robot.joints.iter().enumerate() {
        if !seen.insert(j.name.as_str()) {
            out.push(critical(
                "RS003",
                format!("joint `{}` is defined more than once", j.name),
                j.span,
                Element::Joint(i),
            ));
        }
    }
}

/// RS004 (two parents), RS005 (loop), RS006 (several roots).
fn tree_shape(robot: &Robot, out: &mut Vec<Finding>) {
    use std::collections::HashMap;
    // child link -> first joint that parents it
    let mut parent_joint: HashMap<&str, (usize, &Joint)> = HashMap::new();
    for (i, j) in robot.joints.iter().enumerate() {
        match parent_joint.get(j.child.as_str()) {
            // MJCF declares joints inside a body, so a body with several joints has them all
            // between the same two links. A second parent is still wrong.
            Some((_, first)) if robot.format == Format::Mjcf && first.parent == j.parent => {}
            Some(_) => out.push(critical(
                "RS004",
                format!("link `{}` has more than one parent joint", j.child),
                j.span,
                Element::Joint(i),
            )),
            None => {
                parent_joint.insert(&j.child, (i, j));
            }
        }
    }

    // Walk up from each link; revisiting a link on the current path is a loop.
    let mut state: HashMap<&str, u8> = HashMap::new(); // 1 = on path, 2 = done
    for l in &robot.links {
        let mut path = Vec::new();
        let mut cur = l.name.as_str();
        loop {
            match state.get(cur) {
                Some(2) => break,
                Some(1) => {
                    let (ji, j) = parent_joint[cur];
                    out.push(critical(
                        "RS005",
                        format!("kinematic loop through link `{cur}`"),
                        j.span,
                        Element::Joint(ji),
                    ));
                    break;
                }
                _ => {}
            }
            state.insert(cur, 1);
            path.push(cur);
            match parent_joint.get(cur) {
                Some((_, j)) => cur = j.parent.as_str(),
                None => break,
            }
        }
        for n in path {
            state.insert(n, 2);
        }
    }

    // Every link that is nobody's child is a root; a tree has exactly one.
    let roots: Vec<(usize, &Link)> = robot
        .links
        .iter()
        .enumerate()
        .filter(|(_, l)| !parent_joint.contains_key(l.name.as_str()))
        .collect();
    for &(i, extra) in roots.iter().skip(1) {
        out.push(critical(
            "RS006",
            format!(
                "link `{}` is an extra root; a robot must have one root link",
                extra.name
            ),
            extra.span,
            Element::Link(i),
        ));
    }
}
