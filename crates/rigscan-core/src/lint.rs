//! Lint rules. Each rule reads a [`Robot`] and reports findings with source spans.

use crate::model::{Joint, Link, Robot, Span};

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
}

/// Run all lint rules over a parsed robot.
pub fn lint(robot: &Robot) -> Vec<Finding> {
    let mut findings = Vec::new();
    // RS001: joint references a link that is not defined.
    let link_names: std::collections::HashSet<&str> =
        robot.links.iter().map(|l| l.name.as_str()).collect();
    for j in &robot.joints {
        for (role, name) in [("parent", &j.parent), ("child", &j.child)] {
            if !link_names.contains(name.as_str()) {
                findings.push(Finding {
                    rule: "RS001",
                    severity: Severity::Critical,
                    message: format!("joint `{}` {role} link `{name}` is not defined", j.name),
                    span: j.span,
                });
            }
        }
    }
    duplicate_names(robot, &mut findings);
    tree_shape(robot, &mut findings);
    findings
}

fn critical(rule: &'static str, message: String, span: Span) -> Finding {
    Finding {
        rule,
        severity: Severity::Critical,
        message,
        span,
    }
}

/// RS002 / RS003: names must be unique within links and within joints.
fn duplicate_names(robot: &Robot, out: &mut Vec<Finding>) {
    use std::collections::HashSet;
    let mut seen = HashSet::new();
    for l in &robot.links {
        if !seen.insert(l.name.as_str()) {
            out.push(critical(
                "RS002",
                format!("link `{}` is defined more than once", l.name),
                l.span,
            ));
        }
    }
    seen.clear();
    for j in &robot.joints {
        if !seen.insert(j.name.as_str()) {
            out.push(critical(
                "RS003",
                format!("joint `{}` is defined more than once", j.name),
                j.span,
            ));
        }
    }
}

/// RS004 (two parents), RS005 (loop), RS006 (several roots).
fn tree_shape(robot: &Robot, out: &mut Vec<Finding>) {
    use std::collections::HashMap;
    // child link -> first joint that parents it
    let mut parent_joint: HashMap<&str, &Joint> = HashMap::new();
    for j in &robot.joints {
        if parent_joint.contains_key(j.child.as_str()) {
            out.push(critical(
                "RS004",
                format!("link `{}` has more than one parent joint", j.child),
                j.span,
            ));
        } else {
            parent_joint.insert(&j.child, j);
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
                    let j = parent_joint[cur];
                    out.push(critical(
                        "RS005",
                        format!("kinematic loop through link `{cur}`"),
                        j.span,
                    ));
                    break;
                }
                _ => {}
            }
            state.insert(cur, 1);
            path.push(cur);
            match parent_joint.get(cur) {
                Some(j) => cur = j.parent.as_str(),
                None => break,
            }
        }
        for n in path {
            state.insert(n, 2);
        }
    }

    // Every link that is nobody's child is a root; a tree has exactly one.
    let roots: Vec<&Link> = robot
        .links
        .iter()
        .filter(|l| !parent_joint.contains_key(l.name.as_str()))
        .collect();
    for extra in roots.iter().skip(1) {
        out.push(critical(
            "RS006",
            format!(
                "link `{}` is an extra root; a robot must have one root link",
                extra.name
            ),
            extra.span,
        ));
    }
}
