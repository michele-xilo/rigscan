//! Writes a [`Robot`] as URDF.

use crate::model::{
    Converted, Geometry, Inertial, Joint, JointKind, Link, Origin, Robot, Role, Shape, Span,
    Unmapped,
};
use std::fmt::Write;

/// The robot as URDF text, with what URDF cannot hold listed in `lost`.
///
/// Numbers are written with Rust's shortest form that reads back as the same value, so a robot
/// read from the text is equal to the one written. A shape for both roles is written twice.
pub fn write_urdf(robot: &Robot) -> Converted {
    let mut out = String::from("<?xml version=\"1.0\"?>\n<robot");
    if let Some(name) = &robot.name {
        attr(&mut out, "name", name);
    }
    out.push_str(">\n");
    let mut lost = robot.unmapped.clone();
    for link in &robot.links {
        write_link(&mut out, link, &mut lost);
    }
    for joint in &robot.joints {
        write_joint(&mut out, joint, &mut lost);
    }
    out.push_str("</robot>\n");
    Converted { text: out, lost }
}

fn lose(lost: &mut Vec<Unmapped>, what: &str, name: &str) {
    lost.push(Unmapped {
        what: what.to_string(),
        name: Some(name.to_string()),
        span: Span::NONE,
    });
}

fn write_link(out: &mut String, link: &Link, lost: &mut Vec<Unmapped>) {
    out.push_str("  <link");
    attr(out, "name", &link.name);
    let shapes: Vec<&Shape> = link
        .shapes
        .iter()
        .filter(|s| match s.geometry {
            Geometry::Capsule { .. } => {
                lose(lost, "capsule", &link.name);
                false
            }
            _ => true,
        })
        .collect();
    if link.inertial.is_none() && shapes.is_empty() {
        out.push_str("/>\n");
        return;
    }
    out.push_str(">\n");
    if let Some(i) = &link.inertial {
        write_inertial(out, i);
    }
    for s in shapes {
        match s.role {
            Role::Visual => write_shape(out, "visual", s),
            Role::Collision => write_shape(out, "collision", s),
            Role::Both => {
                write_shape(out, "visual", s);
                write_shape(out, "collision", s);
            }
        }
    }
    out.push_str("  </link>\n");
}

fn write_inertial(out: &mut String, i: &Inertial) {
    out.push_str("    <inertial>\n");
    origin(out, "      ", &i.origin);
    if let Some(m) = i.mass {
        out.push_str("      <mass");
        number(out, "value", m);
        out.push_str("/>\n");
    }
    if let Some(t) = &i.inertia {
        out.push_str("      <inertia");
        for (key, v) in [
            ("ixx", t.ixx),
            ("ixy", t.ixy),
            ("ixz", t.ixz),
            ("iyy", t.iyy),
            ("iyz", t.iyz),
            ("izz", t.izz),
        ] {
            number(out, key, v);
        }
        out.push_str("/>\n");
    }
    out.push_str("    </inertial>\n");
}

fn write_shape(out: &mut String, tag: &str, s: &Shape) {
    let _ = writeln!(out, "    <{tag}>");
    origin(out, "      ", &s.origin);
    out.push_str("      <geometry>\n        ");
    match &s.geometry {
        Geometry::Box { size } => {
            out.push_str("<box");
            vector(out, "size", *size);
        }
        Geometry::Sphere { radius } => {
            out.push_str("<sphere");
            number(out, "radius", *radius);
        }
        Geometry::Cylinder { radius, length } => {
            out.push_str("<cylinder");
            number(out, "radius", *radius);
            number(out, "length", *length);
        }
        Geometry::Mesh { filename, scale } => {
            out.push_str("<mesh");
            attr(out, "filename", filename);
            if *scale != [1.0; 3] {
                vector(out, "scale", *scale);
            }
        }
        // Capsules are filtered out before a shape is written.
        Geometry::Capsule { .. } => {}
    }
    out.push_str("/>\n      </geometry>\n");
    let _ = writeln!(out, "    </{tag}>");
}

fn write_joint(out: &mut String, j: &Joint, lost: &mut Vec<Unmapped>) {
    let kind = match &j.kind {
        JointKind::Revolute => "revolute",
        JointKind::Continuous => "continuous",
        JointKind::Prismatic => "prismatic",
        JointKind::Fixed => "fixed",
        JointKind::Floating => "floating",
        JointKind::Planar => "planar",
        JointKind::Other(t) => {
            lose(lost, &format!("joint/{t}"), &j.name);
            t
        }
    };
    out.push_str("  <joint");
    attr(out, "name", &j.name);
    attr(out, "type", kind);
    out.push_str(">\n    <parent");
    attr(out, "link", &j.parent);
    out.push_str("/>\n    <child");
    attr(out, "link", &j.child);
    out.push_str("/>\n");
    origin(out, "    ", &j.origin);
    if let Some(a) = j.axis {
        out.push_str("    <axis");
        vector(out, "xyz", a);
        out.push_str("/>\n");
    }
    if let Some(l) = &j.limit {
        out.push_str("    <limit");
        optional(out, "lower", l.lower);
        optional(out, "upper", l.upper);
        optional(out, "effort", l.effort);
        optional(out, "velocity", l.velocity);
        out.push_str("/>\n");
    }
    if let Some(d) = &j.dynamics {
        out.push_str("    <dynamics");
        optional(out, "damping", d.damping);
        optional(out, "friction", d.friction);
        out.push_str("/>\n");
    }
    if let Some(m) = &j.mimic {
        out.push_str("    <mimic");
        attr(out, "joint", &m.joint);
        optional(out, "multiplier", m.multiplier);
        optional(out, "offset", m.offset);
        out.push_str("/>\n");
    }
    out.push_str("  </joint>\n");
}

fn origin(out: &mut String, indent: &str, o: &Origin) {
    out.push_str(indent);
    out.push_str("<origin");
    vector(out, "xyz", o.xyz);
    vector(out, "rpy", o.rpy);
    out.push_str("/>\n");
}

fn attr(out: &mut String, key: &str, value: &str) {
    let _ = write!(out, " {key}=\"");
    for c in value.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            c => out.push(c),
        }
    }
    out.push('"');
}

fn number(out: &mut String, key: &str, v: f64) {
    let _ = write!(out, " {key}=\"{v}\"");
}

fn optional(out: &mut String, key: &str, v: Option<f64>) {
    if let Some(v) = v {
        number(out, key, v);
    }
}

fn vector(out: &mut String, key: &str, v: [f64; 3]) {
    let _ = write!(out, " {key}=\"{} {} {}\"", v[0], v[1], v[2]);
}
