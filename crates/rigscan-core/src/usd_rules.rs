//! Rules that only make sense for USD.

use crate::lint::{critical, lint, warning, Finding};
use crate::model::{Element, Robot, Span};
use crate::usd::Usd;

/// The shared rules on `robot` (mapped from `usd`) plus the USD-specific ones.
pub fn lint_usd(usd: &Usd, robot: &Robot) -> Vec<Finding> {
    let mut out = lint(robot);
    self_joints(robot, &mut out);
    massless_bodies(usd, robot, &mut out);
    missing_articulation_root(usd, &mut out);
    for e in &usd.composition_errors {
        out.push(critical(
            "RS022",
            format!("a layer could not be composed: {e}"),
            Span::NONE,
            Element::Robot,
        ));
    }
    out
}

/// RS019: both ends of a joint are the same link, or both are the world.
fn self_joints(robot: &Robot, out: &mut Vec<Finding>) {
    for (i, j) in robot.joints.iter().enumerate() {
        if j.parent == j.child {
            out.push(critical(
                "RS019",
                format!("joint `{}` connects `{}` to itself", j.name, j.parent),
                Span::NONE,
                Element::Joint(i),
            ));
        }
    }
}

/// RS020: nothing says what a rigid body weighs. The links are `robot.links` offset by the root link
/// `world`, which `to_robot` adds first when a joint needs it.
fn massless_bodies(usd: &Usd, robot: &Robot, out: &mut Vec<Finding>) {
    let offset = robot.links.len() - usd.bodies.len();
    for (i, b) in usd.bodies.iter().enumerate() {
        if b.rigid && b.mass.is_none() && !b.collision {
            out.push(warning(
                "RS020",
                format!(
                    "rigid body `{}` has no mass and no collision shape, so its mass cannot be computed",
                    b.path
                ),
                Span::NONE,
                Element::Link(offset + i),
            ));
        }
    }
}

/// RS021: joints with nothing marking the chain as an articulation.
fn missing_articulation_root(usd: &Usd, out: &mut Vec<Finding>) {
    if !usd.joints.is_empty() && usd.articulation_roots.is_empty() {
        out.push(warning(
            "RS021",
            "the stage has joints but no prim with PhysicsArticulationRootAPI".into(),
            Span::NONE,
            Element::Robot,
        ));
    }
}
