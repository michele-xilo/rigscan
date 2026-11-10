//! Rules that only make sense for MJCF: names used in the file that are not defined.

use crate::files::Resolver;
use crate::lint::{critical, lint_with, Finding};
use crate::mjcf::{Mjcf, MjcfNode, Reference};
use crate::mjcf_map::mesh_assets;
use crate::model::{Element, Robot};
use std::collections::HashSet;

/// The shared rules on `robot` (mapped from `mjcf`) plus the MJCF-specific ones.
pub fn lint_mjcf(mjcf: &Mjcf, robot: &Robot, resolver: &dyn Resolver) -> Vec<Finding> {
    let mut out = lint_with(robot, resolver);
    // RS016
    refs(
        &mjcf.unknown_classes,
        "RS016",
        "default class",
        "is not defined",
        &mut out,
    );
    // RS017
    let assets: HashSet<&str> = mesh_assets(&mjcf.root).map(|(n, _)| n).collect();
    undefined_meshes(&mjcf.root, &assets, &mut out);
    // RS018
    refs(
        &mjcf.unresolved_includes,
        "RS018",
        "include file",
        "was not found",
        &mut out,
    );
    out
}

fn refs(list: &[Reference], rule: &'static str, what: &str, state: &str, out: &mut Vec<Finding>) {
    for r in list {
        out.push(critical(
            rule,
            format!("{what} `{}` {state}", r.name),
            r.anchor,
            Element::Robot,
        ));
    }
}

/// Class defaults are not elements of the robot, so a `geom` inside `<default>` is skipped; one that
/// takes its `mesh` from a class has it filled in by then.
fn undefined_meshes(node: &MjcfNode, assets: &HashSet<&str>, out: &mut Vec<Finding>) {
    for c in &node.children {
        if c.tag == "default" {
            continue;
        }
        if c.tag == "geom" {
            if let Some(name) = c.attr("mesh").filter(|n| !assets.contains(n)) {
                out.push(critical(
                    "RS017",
                    format!("mesh asset `{name}` is not defined"),
                    c.anchor,
                    Element::Robot,
                ));
            }
        }
        undefined_meshes(c, assets, out);
    }
}
