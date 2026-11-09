//! Physics rules: mass, inertia and joint limits (RS008 to RS013).

use crate::lint::{critical, warning, Finding};
use crate::model::{Element, Inertia, Joint, JointKind, Robot};
use std::collections::HashMap;

// Measured on 69 real robots: every tensor flagged below is wrong by more than 1%, so these only
// need to absorb rounding in the eigenvalue solve. Tensors with every moment under the floor are
// numerical zeros (288 of them in that corpus) and say nothing about the robot.
const NEGLIGIBLE: f64 = 1e-12;
const SIGN_TOLERANCE: f64 = 1e-9;
const TRIANGLE_TOLERANCE: f64 = 1e-6;

pub fn physics(robot: &Robot, out: &mut Vec<Finding>) {
    links(robot, out);
    for (i, j) in robot.joints.iter().enumerate() {
        limits(i, j, out);
    }
}

fn is_movable(kind: &JointKind) -> bool {
    matches!(
        kind,
        JointKind::Revolute
            | JointKind::Continuous
            | JointKind::Prismatic
            | JointKind::Floating
            | JointKind::Planar
    )
}

fn links(robot: &Robot, out: &mut Vec<Finding>) {
    // Built only when a zero mass turns up: massless frames are rare enough that most robots skip it.
    let mut parent_joint: Option<HashMap<&str, &Joint>> = None;
    for (i, l) in robot.links.iter().enumerate() {
        let Some(inertial) = &l.inertial else {
            continue;
        };
        let element = Element::Link(i);
        match inertial.mass {
            Some(m) if m < 0.0 => out.push(critical(
                "RS008",
                format!("link `{}` has negative mass {m}", l.name),
                inertial.span,
                element,
            )),
            Some(0.0) => {
                let map = parent_joint.get_or_insert_with(|| {
                    robot.joints.iter().map(|j| (j.child.as_str(), j)).collect()
                });
                // A massless frame on a fixed joint is a common idiom; only a moving body needs mass.
                if let Some(j) = map.get(l.name.as_str()).filter(|j| is_movable(&j.kind)) {
                    out.push(warning(
                        "RS009",
                        format!(
                            "link `{}` has zero mass but is moved by joint `{}`",
                            l.name, j.name
                        ),
                        inertial.span,
                        element,
                    ));
                }
            }
            _ => {}
        }
        if let Some(inertia) = &inertial.inertia {
            inertia_shape(&l.name, inertia, inertial.span, element, out);
        }
    }
}

fn inertia_shape(
    name: &str,
    inertia: &Inertia,
    span: crate::model::Span,
    element: Element,
    out: &mut Vec<Finding>,
) {
    let w = principal_moments(inertia);
    let largest = w[0].abs().max(w[2].abs());
    if largest < NEGLIGIBLE {
        return;
    }
    if w[0] < -SIGN_TOLERANCE * largest {
        out.push(critical(
            "RS010",
            format!(
                "link `{name}` has a negative principal moment of inertia {:e}",
                w[0]
            ),
            span,
            element,
        ));
    } else if w[0] + w[1] < w[2] * (1.0 - TRIANGLE_TOLERANCE) {
        out.push(warning(
            "RS011",
            format!(
                "link `{name}` principal moments {:e}, {:e}, {:e} break the triangle inequality",
                w[0], w[1], w[2]
            ),
            span,
            element,
        ));
    }
}

/// Eigenvalues of the symmetric inertia tensor in ascending order (closed form for 3x3).
fn principal_moments(t: &Inertia) -> [f64; 3] {
    let off = t.ixy * t.ixy + t.ixz * t.ixz + t.iyz * t.iyz;
    if off == 0.0 {
        let mut d = [t.ixx, t.iyy, t.izz];
        d.sort_by(f64::total_cmp);
        return d;
    }
    let q = (t.ixx + t.iyy + t.izz) / 3.0;
    let p = (((t.ixx - q).powi(2) + (t.iyy - q).powi(2) + (t.izz - q).powi(2) + 2.0 * off) / 6.0)
        .sqrt();
    let (a, b, c) = ((t.ixx - q) / p, (t.iyy - q) / p, (t.izz - q) / p);
    let (d, e, f) = (t.ixy / p, t.ixz / p, t.iyz / p);
    let det = a * (b * c - f * f) - d * (d * c - f * e) + e * (d * f - b * e);
    let phi = (det / 2.0).clamp(-1.0, 1.0).acos() / 3.0;
    let high = q + 2.0 * p * phi.cos();
    let low = q + 2.0 * p * (phi + 2.0 * std::f64::consts::FRAC_PI_3).cos();
    [low, 3.0 * q - high - low, high]
}

fn limits(i: usize, j: &Joint, out: &mut Vec<Finding>) {
    let kind = match j.kind {
        JointKind::Revolute => "revolute",
        JointKind::Prismatic => "prismatic",
        _ => return,
    };
    let element = Element::Joint(i);
    let Some(limit) = &j.limit else {
        out.push(critical(
            "RS013",
            format!("{kind} joint `{}` has no limit", j.name),
            j.span,
            element,
        ));
        return;
    };
    let missing: Vec<&str> = [("effort", limit.effort), ("velocity", limit.velocity)]
        .into_iter()
        .filter(|(_, v)| v.is_none())
        .map(|(n, _)| n)
        .collect();
    if !missing.is_empty() {
        let names: Vec<String> = missing.iter().map(|n| format!("`{n}`")).collect();
        out.push(critical(
            "RS013",
            format!(
                "{kind} joint `{}` limit is missing {}",
                j.name,
                names.join(" and ")
            ),
            j.span,
            element,
        ));
    }
    if let (Some(lo), Some(up)) = (limit.lower, limit.upper) {
        if lo > up {
            out.push(critical(
                "RS012",
                format!(
                    "joint `{}` lower limit {lo} is above upper limit {up}",
                    j.name
                ),
                j.span,
                element,
            ));
        }
    }
}
