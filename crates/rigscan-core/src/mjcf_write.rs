//! Writes a [`Robot`] as MJCF.

use crate::model::{
    Converted, Geometry, Inertial, Joint, JointKind, Link, Origin, Robot, Role, Shape, Span,
    Unmapped,
};
use crate::urdf_write::{attr, optional, vector};
use std::collections::{HashMap, HashSet};
use std::fmt::Write;

/// The robot as MJCF text, with what MJCF cannot hold listed in `lost`.
///
/// Angles are written in radians with `eulerseq="XYZ"`, which is the order of a URDF `rpy`, so an
/// orientation is written and read back without rounding. A link is a body nested in its parent's
/// body and its joint's origin is the body pose: the link frame is the joint frame, so no joint
/// `pos` is needed. A link with no parent is welded to the world.
pub fn write_mjcf(robot: &Robot) -> Converted {
    let mut w = Writer {
        robot,
        out: String::new(),
        lost: robot.unmapped.clone(),
        children: HashMap::new(),
        meshes: HashMap::new(),
        placed: HashSet::new(),
    };
    w.run();
    Converted {
        text: w.out,
        lost: w.lost,
    }
}

struct Writer<'a> {
    robot: &'a Robot,
    out: String,
    lost: Vec<Unmapped>,
    /// Parent link to the links below it, each with the joints that connect them.
    children: HashMap<&'a str, Vec<(&'a str, Vec<&'a Joint>)>>,
    /// A mesh file and its scale, to the name of the asset written for it.
    meshes: HashMap<(&'a str, [u64; 3]), usize>,
    placed: HashSet<&'a str>,
}

impl<'a> Writer<'a> {
    fn run(&mut self) {
        let robot = self.robot;
        self.out.push_str("<mujoco");
        if let Some(name) = &robot.name {
            attr(&mut self.out, "model", name);
        }
        self.out
            .push_str(">\n  <compiler angle=\"radian\" eulerseq=\"XYZ\"/>\n");
        self.assets();
        self.group_joints();
        self.out.push_str("  <worldbody>\n");
        let child_links: HashSet<&str> = robot.joints.iter().map(|j| j.child.as_str()).collect();
        for link in robot
            .links
            .iter()
            .filter(|l| !child_links.contains(l.name.as_str()))
        {
            if !self.placed.insert(&link.name) {
                continue;
            }
            if link.name == "world" {
                self.geoms(link, 2);
                self.below(link, 2);
            } else {
                self.body(link, &[], 2);
            }
        }
        self.out.push_str("  </worldbody>\n</mujoco>\n");
        for link in &robot.links {
            if !self.placed.contains(link.name.as_str()) {
                self.lose("link", &link.name);
            }
        }
    }

    fn lose(&mut self, what: &str, name: &str) {
        self.lost.push(Unmapped {
            what: what.to_string(),
            name: Some(name.to_string()),
            span: Span::NONE,
        });
    }

    /// One asset per mesh file and scale, numbered in sorted order: the numbers do not depend on
    /// the order of the links, which MJCF changes by nesting them.
    fn assets(&mut self) {
        let robot = self.robot;
        let mut files: Vec<(&str, [u64; 3])> = robot
            .links
            .iter()
            .flat_map(|l| &l.shapes)
            .filter_map(|s| match &s.geometry {
                Geometry::Mesh { filename, scale } => {
                    Some((filename.as_str(), scale.map(f64::to_bits)))
                }
                _ => None,
            })
            .collect();
        files.sort_unstable();
        files.dedup();
        if files.is_empty() {
            return;
        }
        self.out.push_str("  <asset>\n");
        for (i, key) in files.iter().enumerate() {
            let _ = write!(self.out, "    <mesh name=\"mesh{i}\"");
            attr(&mut self.out, "file", key.0);
            let scale = key.1.map(f64::from_bits);
            if scale != [1.0; 3] {
                vector(&mut self.out, "scale", scale);
            }
            self.out.push_str("/>\n");
            self.meshes.insert(*key, i);
        }
        self.out.push_str("  </asset>\n");
    }

    fn group_joints(&mut self) {
        let robot = self.robot;
        for j in &robot.joints {
            let below = self.children.entry(j.parent.as_str()).or_default();
            match below.iter_mut().find(|(child, _)| *child == j.child) {
                Some((_, joints)) => joints.push(j),
                None => below.push((j.child.as_str(), vec![j])),
            }
        }
    }

    /// The bodies of the links below `link`, at `depth` levels of indentation.
    fn below(&mut self, link: &Link, depth: usize) {
        let robot = self.robot;
        let Some(children) = self.children.get(link.name.as_str()).cloned() else {
            return;
        };
        for (child, joints) in children {
            let Some(next) = robot.links.iter().find(|l| l.name == child) else {
                // A joint to a link that is not defined has no body to write.
                for j in joints {
                    self.lose("joint/undefined", &j.name);
                }
                continue;
            };
            if !self.placed.insert(&next.name) {
                // MJCF is a tree: a second parent cannot be written.
                for j in joints {
                    self.lose("joint/tree", &j.name);
                }
                continue;
            }
            self.body(next, &joints, depth);
        }
    }

    fn body(&mut self, link: &Link, joints: &[&Joint], depth: usize) {
        let pad = "  ".repeat(depth);
        let _ = write!(self.out, "{pad}<body");
        attr(&mut self.out, "name", &link.name);
        if let Some(first) = joints.first() {
            for later in &joints[1..] {
                if later.origin != first.origin {
                    self.lose("joint/origin", &later.name);
                }
            }
            pose(&mut self.out, &first.origin);
        }
        let below = self.children.contains_key(link.name.as_str());
        if link.inertial.is_none() && link.shapes.is_empty() && joints.is_empty() && !below {
            self.out.push_str("/>\n");
            return;
        }
        self.out.push_str(">\n");
        if let Some(i) = &link.inertial {
            inertial(&mut self.out, &pad, i);
        }
        let top = joints.first().is_some_and(|j| j.parent == "world");
        for j in joints {
            self.joint(j, top, &pad);
        }
        self.geoms(link, depth + 1);
        self.below(link, depth + 1);
        let _ = writeln!(self.out, "{pad}</body>");
    }

    fn joint(&mut self, j: &Joint, top: bool, pad: &str) {
        if j.mimic.is_some() {
            self.lose("mimic", &j.name);
        }
        if matches!(j.kind, JointKind::Fixed | JointKind::Floating) {
            // Neither has a place for a bound or for damping and friction.
            let limited = j.limit.as_ref().is_some_and(|l| {
                [l.lower, l.upper, l.effort, l.velocity]
                    .iter()
                    .any(Option::is_some)
            });
            if limited {
                self.lose("limit", &j.name);
            }
            if j.dynamics
                .as_ref()
                .is_some_and(|d| d.damping.is_some() || d.friction.is_some())
            {
                self.lose("dynamics", &j.name);
            }
        }
        let (kind, bounded) = match &j.kind {
            JointKind::Revolute => ("hinge", true),
            JointKind::Continuous => ("hinge", false),
            JointKind::Prismatic => ("slide", true),
            JointKind::Fixed => {
                if j.name != j.child {
                    self.lose("joint/name", &j.name);
                }
                return;
            }
            JointKind::Floating if top => {
                let _ = write!(self.out, "{pad}  <freejoint");
                attr(&mut self.out, "name", &j.name);
                self.out.push_str("/>\n");
                return;
            }
            JointKind::Floating => {
                self.lose("joint/floating", &j.name);
                return;
            }
            JointKind::Planar => {
                self.lose("joint/planar", &j.name);
                return;
            }
            JointKind::Other(t) => {
                self.lose(&format!("joint/{t}"), &j.name);
                return;
            }
        };
        let _ = write!(self.out, "{pad}  <joint");
        attr(&mut self.out, "name", &j.name);
        attr(&mut self.out, "type", kind);
        if let Some(a) = j.axis {
            vector(&mut self.out, "axis", a);
        }
        let limit = j.limit.as_ref();
        if limit.is_some_and(|l| l.effort.is_some()) {
            self.lose("limit/effort", &j.name);
        }
        if limit.is_some_and(|l| l.velocity.is_some()) {
            self.lose("limit/velocity", &j.name);
        }
        let bounds = limit.map(|l| (l.lower, l.upper));
        match (bounded, bounds) {
            (true, Some((Some(lo), Some(hi)))) => {
                let _ = write!(self.out, " range=\"{lo} {hi}\"");
            }
            // A hinge with no range is read back as continuous, so a revolute joint without both
            // bounds changes type; a prismatic one only loses the bound it has.
            (true, None) if j.kind == JointKind::Prismatic => {}
            (true, _) => self.lose("limit", &j.name),
            (false, Some((None, None))) | (false, None) => {}
            (false, Some(_)) => self.lose("limit", &j.name),
        }
        if let Some(d) = &j.dynamics {
            optional(&mut self.out, "damping", d.damping);
            optional(&mut self.out, "frictionloss", d.friction);
        }
        self.out.push_str("/>\n");
    }

    fn geoms(&mut self, link: &Link, depth: usize) {
        let pad = "  ".repeat(depth);
        for s in &link.shapes {
            let _ = write!(self.out, "{pad}<geom");
            match &s.geometry {
                Geometry::Box { size } => {
                    let _ = write!(
                        self.out,
                        " type=\"box\" size=\"{} {} {}\"",
                        size[0] / 2.0,
                        size[1] / 2.0,
                        size[2] / 2.0
                    );
                }
                Geometry::Sphere { radius } => {
                    let _ = write!(self.out, " type=\"sphere\" size=\"{radius}\"");
                }
                Geometry::Cylinder { radius, length } => {
                    let _ = write!(
                        self.out,
                        " type=\"cylinder\" size=\"{radius} {}\"",
                        length / 2.0
                    );
                }
                Geometry::Capsule { radius, length } => {
                    let _ = write!(
                        self.out,
                        " type=\"capsule\" size=\"{radius} {}\"",
                        length / 2.0
                    );
                }
                Geometry::Mesh { filename, scale } => {
                    let i = self.meshes[&(filename.as_str(), scale.map(f64::to_bits))];
                    let _ = write!(self.out, " type=\"mesh\" mesh=\"mesh{i}\"");
                }
            }
            pose(&mut self.out, &s.origin);
            role(&mut self.out, s);
            self.out.push_str("/>\n");
        }
    }
}

/// MuJoCo collides geoms by `contype` and `conaffinity`, and hides group 3 and up: a look-only geom
/// collides with nothing, a collision-only one is hidden, and one for both is left alone.
fn role(out: &mut String, s: &Shape) {
    match s.role {
        Role::Visual => out.push_str(" contype=\"0\" conaffinity=\"0\""),
        Role::Collision => out.push_str(" group=\"3\""),
        Role::Both => {}
    }
}

/// A zero pose is the default and is left out.
fn pose(out: &mut String, o: &Origin) {
    if o.xyz != [0.0; 3] {
        vector(out, "pos", o.xyz);
    }
    if o.rpy != [0.0; 3] {
        vector(out, "euler", o.rpy);
    }
}

fn inertial(out: &mut String, pad: &str, i: &Inertial) {
    let _ = write!(out, "{pad}  <inertial");
    pose(out, &i.origin);
    optional(out, "mass", i.mass);
    if let Some(t) = &i.inertia {
        if t.ixy == 0.0 && t.ixz == 0.0 && t.iyz == 0.0 {
            vector(out, "diaginertia", [t.ixx, t.iyy, t.izz]);
        } else {
            let _ = write!(
                out,
                " fullinertia=\"{} {} {} {} {} {}\"",
                t.ixx, t.iyy, t.izz, t.ixy, t.ixz, t.iyz
            );
        }
    }
    out.push_str("/>\n");
}
