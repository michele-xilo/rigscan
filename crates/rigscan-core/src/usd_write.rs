//! Writes a [`Robot`] as USD (`.usda`) with UsdPhysics.

use crate::model::{
    Converted, Geometry, Inertial, Joint, JointKind, Link, Origin, Robot, Role, Shape, Span,
    Unmapped,
};
use crate::rotation::{align_x, apply, from_rpy, mul, symmetric_eigen, to_quat, transpose, Mat};
use std::collections::{HashMap, HashSet};
use std::fmt::Write;

const IDENTITY: Mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
const WORLD: &str = "world";

/// The robot as `.usda` text, with what USD cannot hold listed in `lost`.
///
/// Every link is a rigid body placed in the world by the joints above it, all under one
/// articulation root, and every joint is a prim of its own, so a closed chain is written as it is.
/// A root link with no joint is held to the world by a fixed joint, because a USD body is free
/// unless something holds it. Schema attributes are written as `float`, as UsdPhysics declares
/// them, so they keep about seven digits; poses and sizes of shapes are `double`.
pub fn write_usd(robot: &Robot) -> Converted {
    let mut w = Writer {
        out: String::new(),
        lost: robot.unmapped.clone(),
        used: HashSet::new(),
        used_joints: HashSet::new(),
    };
    w.robot(robot);
    Converted {
        text: w.out,
        lost: w.lost,
    }
}

struct Writer {
    out: String,
    lost: Vec<Unmapped>,
    /// Prim names taken under the root: links and joints share one namespace.
    used: HashSet<String>,
    /// Joint names, kept apart from links because joints sit in a scope of their own.
    used_joints: HashSet<String>,
}

/// A rotation and a translation: `p' = r * p + t`.
#[derive(Clone, Copy)]
struct Pose {
    r: Mat,
    t: [f64; 3],
}

impl Pose {
    const IDENTITY: Pose = Pose {
        r: IDENTITY,
        t: [0.0; 3],
    };

    fn of(o: &Origin) -> Pose {
        Pose {
            r: from_rpy(o.rpy),
            t: o.xyz,
        }
    }

    fn then(&self, inner: &Pose) -> Pose {
        let moved = apply(&self.r, inner.t);
        Pose {
            r: mul(&self.r, &inner.r),
            t: [0, 1, 2].map(|i| self.t[i] + moved[i]),
        }
    }
}

impl Writer {
    fn lose(&mut self, what: &str, name: &str) {
        self.lost.push(Unmapped {
            what: what.to_string(),
            name: Some(name.to_string()),
            span: Span::NONE,
        });
    }

    /// A prim name for `wanted`: an identifier not used yet. A name that had to change is reported
    /// when it is the source's own; the root weld's name is ours.
    fn prim_name(&mut self, wanted: &str, report: bool, joint: bool) -> String {
        let base = identifier(wanted);
        let mut name = base.clone();
        let mut n = 2;
        let used = if joint {
            &mut self.used_joints
        } else {
            &mut self.used
        };
        while !used.insert(name.clone()) {
            name = format!("{base}_{n}");
            n += 1;
        }
        if report && name != wanted {
            self.lose("name", wanted);
        }
        name
    }

    fn robot(&mut self, robot: &Robot) {
        let root = identifier(robot.name.as_deref().unwrap_or("robot"));
        if let Some(name) = &robot.name {
            if *name != root {
                self.lose("name", name);
            }
        }
        let _ = write!(
            self.out,
            "#usda 1.0\n(\n    defaultPrim = \"{root}\"\n    metersPerUnit = 1\n    kilogramsPerUnit = 1\n    upAxis = \"Z\"\n)\n\n\
             def Xform \"{root}\" (\n    prepend apiSchemas = [\"PhysicsArticulationRootAPI\"]\n)\n{{\n"
        );
        // The world is not a body: a link of that name with nothing in it stands for it.
        let is_world = |l: &Link| l.name == WORLD && l.inertial.is_none() && l.shapes.is_empty();
        let bodies: Vec<&Link> = robot.links.iter().filter(|l| !is_world(l)).collect();
        let mut names: HashMap<&str, String> = HashMap::new();
        for l in &bodies {
            let name = self.prim_name(&l.name, true, false);
            names.insert(&l.name, name);
        }
        let poses = poses(robot, &bodies);
        for l in &bodies {
            self.body(&names[l.name.as_str()], l, poses[l.name.as_str()]);
        }
        let before = std::mem::take(&mut self.out);
        self.joints(robot, &root, &bodies, &names);
        let joints = std::mem::replace(&mut self.out, before);
        if !joints.is_empty() {
            // Joints get a scope of their own: an MJCF weld is named after its body.
            let scope = self.prim_name("joints", false, false);
            let _ = writeln!(self.out, "    def Scope \"{scope}\"\n    {{");
            for line in joints.lines() {
                let _ = writeln!(self.out, "    {line}");
            }
            self.out.push_str("    }\n");
        }
        self.out.push_str("}\n");
    }

    fn body(&mut self, name: &str, link: &Link, pose: Pose) {
        let mass = link
            .inertial
            .as_ref()
            .and_then(|i| self.mass_data(i, &link.name));
        let apis = if mass.is_some() {
            "\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\""
        } else {
            "\"PhysicsRigidBodyAPI\""
        };
        let _ = writeln!(
            self.out,
            "    def Xform \"{name}\" (\n        prepend apiSchemas = [{apis}]\n    )\n    {{"
        );
        self.xform("        ", &pose, None);
        if let Some(m) = mass {
            self.out.push_str(&m);
        }
        for (i, s) in link.shapes.iter().enumerate() {
            self.shape(i, s, &link.name);
        }
        self.out.push_str("    }\n");
    }

    /// The mass attributes of a link, as text; `None` when there is nothing USD can read.
    fn mass_data(&mut self, i: &Inertial, link: &str) -> Option<String> {
        let mass = i.mass;
        // USD reads a mass of zero as "compute it from the shapes", which is another value.
        if mass.is_some_and(|m| !(m > 0.0 && m.is_finite())) {
            self.lose("inertial", link);
            return None;
        }
        let mut s = String::new();
        if let Some(m) = mass {
            let _ = writeln!(s, "        float physics:mass = {}", f32s(m));
        }
        if i.origin.xyz != [0.0; 3] {
            let _ = writeln!(
                s,
                "        point3f physics:centerOfMass = {}",
                vec32(i.origin.xyz)
            );
        }
        if let Some(t) = &i.inertia {
            let r = from_rpy(i.origin.rpy);
            let m = [
                [t.ixx, t.ixy, t.ixz],
                [t.ixy, t.iyy, t.iyz],
                [t.ixz, t.iyz, t.izz],
            ];
            // The tensor in the link frame, then its principal frame: USD stores it that way.
            let (d, axes) = symmetric_eigen(mul(&r, &mul(&m, &transpose(&r))));
            if d.iter().any(|x| *x != 0.0) {
                let _ = writeln!(s, "        float3 physics:diagonalInertia = {}", vec32(d));
                if axes != IDENTITY {
                    let q = to_quat(&axes);
                    let _ = writeln!(
                        s,
                        "        quatf physics:principalAxes = ({}, {}, {}, {})",
                        f32s(q[0]),
                        f32s(q[1]),
                        f32s(q[2]),
                        f32s(q[3])
                    );
                }
            }
        }
        (!s.is_empty()).then_some(s)
    }

    fn xform(&mut self, pad: &str, pose: &Pose, scale: Option<[f64; 3]>) {
        let mut order = Vec::new();
        if pose.t != [0.0; 3] {
            let _ = writeln!(self.out, "{pad}double3 xformOp:translate = {}", vec(pose.t));
            order.push("\"xformOp:translate\"");
        }
        if pose.r != IDENTITY {
            let q = to_quat(&pose.r);
            let _ = writeln!(
                self.out,
                "{pad}quatd xformOp:orient = ({}, {}, {}, {})",
                q[0], q[1], q[2], q[3]
            );
            order.push("\"xformOp:orient\"");
        }
        if let Some(s) = scale {
            let _ = writeln!(self.out, "{pad}double3 xformOp:scale = {}", vec(s));
            order.push("\"xformOp:scale\"");
        }
        if !order.is_empty() {
            let _ = writeln!(
                self.out,
                "{pad}uniform token[] xformOpOrder = [{}]",
                order.join(", ")
            );
        }
    }

    fn shape(&mut self, index: usize, shape: &Shape, link: &str) {
        let (ty, attrs, scale) = match &shape.geometry {
            Geometry::Box { size } => ("Cube", "double size = 1\n".to_string(), Some(*size)),
            Geometry::Sphere { radius } => ("Sphere", format!("double radius = {radius}\n"), None),
            Geometry::Cylinder { radius, length } => (
                "Cylinder",
                format!("double radius = {radius}\ndouble height = {length}\nuniform token axis = \"Z\"\n"),
                None,
            ),
            Geometry::Capsule { radius, length } => (
                "Capsule",
                format!("double radius = {radius}\ndouble height = {length}\nuniform token axis = \"Z\"\n"),
                None,
            ),
            // A USD mesh holds its points, and the model keeps only the name of a file.
            Geometry::Mesh { .. } => {
                self.lose("mesh", link);
                return;
            }
        };
        let api = match shape.role {
            Role::Visual => String::new(),
            _ => " (\n            prepend apiSchemas = [\"PhysicsCollisionAPI\"]\n        )"
                .to_string(),
        };
        let _ = writeln!(
            self.out,
            "        def {ty} \"shape_{index}\"{api}\n        {{"
        );
        for line in attrs.lines() {
            let _ = writeln!(self.out, "            {line}");
        }
        if shape.role == Role::Collision {
            self.out
                .push_str("            uniform token purpose = \"guide\"\n");
        }
        self.xform("            ", &Pose::of(&shape.origin), scale);
        self.out.push_str("        }\n");
    }

    fn joints(
        &mut self,
        robot: &Robot,
        root: &str,
        bodies: &[&Link],
        names: &HashMap<&str, String>,
    ) {
        let known: HashSet<&str> = names.keys().copied().collect();
        let mut held: HashSet<&str> = HashSet::new();
        for j in &robot.joints {
            held.insert(j.child.as_str());
        }
        // A root link has no joint above it, and USD leaves a body free unless a joint holds it.
        for l in bodies {
            if !held.contains(l.name.as_str()) {
                let name = self.prim_name("rootJoint", false, true);
                let _ = write!(
                    self.out,
                    "    def PhysicsFixedJoint \"{name}\"\n    {{\n        rel physics:body1 = </{root}/{}>\n    }}\n",
                    names[l.name.as_str()]
                );
            }
        }
        for j in &robot.joints {
            let parent = if j.parent == WORLD && !known.contains(WORLD) {
                Some(None)
            } else {
                known.get(j.parent.as_str()).map(|p| Some(*p))
            };
            let child = known.get(j.child.as_str()).copied();
            let (Some(parent), Some(child)) = (parent, child) else {
                self.lose("joint/undefined", &j.name);
                continue;
            };
            self.joint(j, root, parent.map(|p| names[p].as_str()), &names[child]);
        }
    }

    fn joint(&mut self, j: &Joint, root: &str, parent: Option<&str>, child: &str) {
        let (ty, moves) = match &j.kind {
            JointKind::Revolute | JointKind::Continuous => ("PhysicsRevoluteJoint", true),
            JointKind::Prismatic => ("PhysicsPrismaticJoint", true),
            JointKind::Fixed => ("PhysicsFixedJoint", false),
            // A floating joint takes no degree of freedom away: leaving it out is the same body.
            JointKind::Floating => {
                if parent.is_some() {
                    self.lose("joint/floating", &j.name);
                }
                return;
            }
            JointKind::Planar => {
                self.lose("joint/planar", &j.name);
                ("PhysicsFixedJoint", false)
            }
            JointKind::Other(t) if t == "ball" || t == "spherical" => {
                ("PhysicsSphericalJoint", false)
            }
            JointKind::Other(t) => {
                self.lose(&format!("joint/{t}"), &j.name);
                ("PhysicsFixedJoint", false)
            }
        };
        let (token, r1) = if moves {
            axis_frame(j.axis.unwrap_or([1.0, 0.0, 0.0]))
        } else {
            ("X", IDENTITY)
        };
        let drive = match (&j.kind, j.limit.as_ref().and_then(|l| l.effort)) {
            (JointKind::Prismatic, Some(e)) if e > 0.0 => Some(("linear", e)),
            (JointKind::Revolute | JointKind::Continuous, Some(e)) if e > 0.0 => {
                Some(("angular", e))
            }
            _ => None,
        };
        if let Some(l) = &j.limit {
            if l.effort.is_some() && drive.is_none() {
                self.lose("limit/effort", &j.name);
            }
            if l.velocity.is_some() {
                self.lose("limit/velocity", &j.name);
            }
        }
        if j.dynamics
            .as_ref()
            .is_some_and(|d| d.damping.is_some() || d.friction.is_some())
        {
            self.lose("dynamics", &j.name);
        }
        if j.mimic.is_some() {
            self.lose("mimic", &j.name);
        }
        let name = self.prim_name(&j.name, true, true);
        let api = drive.map_or(String::new(), |(kind, _)| {
            format!(" (\n        prepend apiSchemas = [\"PhysicsDriveAPI:{kind}\"]\n    )")
        });
        let _ = writeln!(self.out, "    def {ty} \"{name}\"{api}\n    {{");
        if let Some(p) = parent {
            let _ = writeln!(self.out, "        rel physics:body0 = </{root}/{p}>");
        }
        let _ = writeln!(self.out, "        rel physics:body1 = </{root}/{child}>");
        if moves {
            let _ = writeln!(self.out, "        token physics:axis = \"{token}\"");
        }
        let _ = writeln!(
            self.out,
            "        point3f physics:localPos0 = {}",
            vec32(j.origin.xyz)
        );
        self.quat32("physics:localRot0", &mul(&from_rpy(j.origin.rpy), &r1));
        if r1 != IDENTITY {
            self.quat32("physics:localRot1", &r1);
        }
        self.limits(j, ty);
        if let Some((kind, effort)) = drive {
            let _ = writeln!(
                self.out,
                "        float drive:{kind}:physics:maxForce = {}",
                f32s(effort)
            );
        }
        self.out.push_str("    }\n");
    }

    fn quat32(&mut self, name: &str, r: &Mat) {
        if *r == IDENTITY {
            return;
        }
        let q = to_quat(r);
        let _ = writeln!(
            self.out,
            "        quatf {name} = ({}, {}, {}, {})",
            f32s(q[0]),
            f32s(q[1]),
            f32s(q[2]),
            f32s(q[3])
        );
    }

    /// Revolute limits are degrees in USD and prismatic ones are lengths.
    fn limits(&mut self, j: &Joint, ty: &str) {
        let Some(l) = &j.limit else {
            if j.kind == JointKind::Revolute {
                self.lose("limit", &j.name);
            }
            return;
        };
        if !matches!(j.kind, JointKind::Revolute | JointKind::Prismatic) {
            if l.lower.is_some() || l.upper.is_some() {
                self.lose("limit", &j.name);
            }
            return;
        }
        let scale = if ty == "PhysicsRevoluteJoint" {
            180.0 / std::f64::consts::PI
        } else {
            1.0
        };
        if l.lower.is_none() && l.upper.is_none() && j.kind == JointKind::Revolute {
            self.lose("limit", &j.name);
        }
        for (name, v) in [("lowerLimit", l.lower), ("upperLimit", l.upper)] {
            if let Some(v) = v {
                let _ = writeln!(
                    self.out,
                    "        float physics:{name} = {}",
                    f32s(v * scale)
                );
            }
        }
    }
}

/// The pose of every link in the world: the joints above it, from the first joint into each link.
fn poses<'a>(robot: &'a Robot, bodies: &[&'a Link]) -> HashMap<&'a str, Pose> {
    let first: HashMap<&str, &Joint> = robot
        .joints
        .iter()
        .rev()
        .map(|j| (j.child.as_str(), j))
        .collect();
    let mut poses: HashMap<&str, Pose> = HashMap::new();
    for l in bodies {
        let mut chain = Vec::new();
        let mut at = l.name.as_str();
        while !poses.contains_key(at) && chain.len() <= robot.links.len() {
            let Some(j) = first.get(at) else { break };
            chain.push(*j);
            at = j.parent.as_str();
        }
        // Applied from the top of the chain down.
        let mut pose = poses.get(at).copied().unwrap_or(Pose::IDENTITY);
        for j in chain.iter().rev() {
            pose = pose.then(&Pose::of(&j.origin));
            poses.insert(j.child.as_str(), pose);
        }
        poses.entry(l.name.as_str()).or_insert(pose);
    }
    poses
}

/// The axis token and the frame that turns it onto `axis`. An axis along x, y or z is the token
/// itself; any other is x in a turned frame.
fn axis_frame(axis: [f64; 3]) -> (&'static str, Mat) {
    let n = axis.iter().map(|v| v * v).sum::<f64>().sqrt();
    if n < 1e-12 {
        return ("X", IDENTITY);
    }
    let u = axis.map(|v| v / n);
    for (k, token) in ["X", "Y", "Z"].into_iter().enumerate() {
        if u[k] > 1.0 - 1e-12 {
            return (token, IDENTITY);
        }
    }
    ("X", align_x(u))
}

/// A name USD accepts for a prim: letters, digits and underscores, not starting with a digit.
fn identifier(name: &str) -> String {
    let mut s: String = name
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect();
    if s.is_empty() || s.starts_with(|c: char| c.is_ascii_digit()) {
        s.insert(0, '_');
    }
    s
}

/// The shortest text that reads back as the same `f32`.
fn f32s(x: f64) -> String {
    format!("{}", x as f32)
}

fn vec32(v: [f64; 3]) -> String {
    format!("({}, {}, {})", f32s(v[0]), f32s(v[1]), f32s(v[2]))
}

fn vec(v: [f64; 3]) -> String {
    format!("({}, {}, {})", v[0], v[1], v[2])
}
