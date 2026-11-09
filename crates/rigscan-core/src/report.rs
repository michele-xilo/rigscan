//! Per-element health derived from lint findings.

use crate::lint::{Finding, Severity};
use crate::model::{Element, Robot, Span};

/// Ordered so that the worst status of an element is the maximum of its findings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Status {
    Healthy,
    Warning,
    Critical,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ElementStatus<'a> {
    pub element: Element,
    pub name: &'a str,
    pub span: Span,
    pub status: Status,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Verdict {
    pub healthy: usize,
    pub warnings: usize,
    pub critical: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Report<'a> {
    pub elements: Vec<ElementStatus<'a>>,
    pub verdict: Verdict,
}

/// The robot element is listed only when it has a finding, so the verdict counts links and joints.
/// Findings must come from linting this same robot: their element indices are not checked.
pub fn report<'a>(robot: &'a Robot, findings: &[Finding]) -> Report<'a> {
    let mut robot_status = Status::Healthy;
    let mut links = vec![Status::Healthy; robot.links.len()];
    let mut joints = vec![Status::Healthy; robot.joints.len()];
    for f in findings {
        let status = match f.severity {
            Severity::Warning => Status::Warning,
            Severity::Critical => Status::Critical,
        };
        let slot = match f.element {
            Element::Robot => &mut robot_status,
            Element::Link(i) => &mut links[i],
            Element::Joint(i) => &mut joints[i],
        };
        *slot = (*slot).max(status);
    }

    let mut elements = Vec::with_capacity(1 + links.len() + joints.len());
    if robot_status != Status::Healthy {
        elements.push(ElementStatus {
            element: Element::Robot,
            name: robot.name.as_deref().unwrap_or(""),
            span: robot.span,
            status: robot_status,
        });
    }
    for (i, (l, status)) in robot.links.iter().zip(links).enumerate() {
        elements.push(ElementStatus {
            element: Element::Link(i),
            name: &l.name,
            span: l.span,
            status,
        });
    }
    for (i, (j, status)) in robot.joints.iter().zip(joints).enumerate() {
        elements.push(ElementStatus {
            element: Element::Joint(i),
            name: &j.name,
            span: j.span,
            status,
        });
    }

    let mut verdict = Verdict::default();
    for e in &elements {
        match e.status {
            Status::Healthy => verdict.healthy += 1,
            Status::Warning => verdict.warnings += 1,
            Status::Critical => verdict.critical += 1,
        }
    }
    Report { elements, verdict }
}
