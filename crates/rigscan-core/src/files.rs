//! Rules about things the robot refers to: mesh files and mimic joints.

use crate::lint::{critical, Finding};
use crate::model::{Element, Robot};

/// What a [`Resolver`] found for a reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Resolution {
    Found,
    Missing,
    /// The resolver cannot tell, for example a `package://` it cannot locate. Never a finding.
    Unknown,
}

/// Answers whether a file reference exists. The core does no file access, so the caller supplies this.
pub trait Resolver {
    /// `uri` is the reference as written in the robot file.
    fn resolve(&self, uri: &str) -> Resolution;
}

pub(crate) struct Unresolved;

impl Resolver for Unresolved {
    fn resolve(&self, _uri: &str) -> Resolution {
        Resolution::Unknown
    }
}

pub(crate) fn files(robot: &Robot, resolver: &dyn Resolver, out: &mut Vec<Finding>) {
    missing_meshes(robot, resolver, out);
    bad_mimics(robot, out);
}

/// RS014: a referenced mesh file does not exist.
fn missing_meshes(robot: &Robot, resolver: &dyn Resolver, out: &mut Vec<Finding>) {
    for (i, link) in robot.links.iter().enumerate() {
        for mesh in &link.meshes {
            if resolver.resolve(&mesh.filename) == Resolution::Missing {
                out.push(critical(
                    "RS014",
                    format!(
                        "link `{}` mesh file `{}` was not found",
                        link.name, mesh.filename
                    ),
                    mesh.span,
                    Element::Link(i),
                ));
            }
        }
    }
}

/// RS015: a mimic must follow another joint that exists.
fn bad_mimics(robot: &Robot, out: &mut Vec<Finding>) {
    if robot.joints.iter().all(|j| j.mimic.is_none()) {
        return;
    }
    let names: std::collections::HashSet<&str> =
        robot.joints.iter().map(|j| j.name.as_str()).collect();
    for (i, j) in robot.joints.iter().enumerate() {
        let Some(m) = &j.mimic else { continue };
        let message = if m.joint == j.name {
            format!("joint `{}` mimics itself", j.name)
        } else if !names.contains(m.joint.as_str()) {
            format!(
                "joint `{}` mimics joint `{}`, which is not defined",
                j.name, m.joint
            )
        } else {
            continue;
        };
        out.push(critical("RS015", message, m.span, Element::Joint(i)));
    }
}
