//! Maps the resolved MJCF tree onto the format-neutral [`Robot`].

use crate::mjcf::{Mjcf, MjcfNode};
use crate::model::{
    Dynamics, Format, Geometry, Inertia, Inertial, Joint, JointKind, Limit, Link, Origin,
    ParseError, Robot, Role, Shape, Unmapped,
};
use crate::rotation::{about, from_axis_angle, from_quat, from_xyaxes, from_zaxis, mul, rpy, Mat};
use std::collections::{HashMap, HashSet};

/// Sections whose children the neutral model has no place for.
const UNMAPPED_SECTIONS: [&str; 6] = [
    "actuator", "tendon", "equality", "contact", "sensor", "keyframe",
];

/// Elements that can hold bodies but are not expanded here, so what is inside is lost.
const UNEXPANDED: [&str; 5] = ["frame", "replicate", "attach", "composite", "flexcomp"];

impl Mjcf {
    /// Map the resolved tree onto the format-neutral model.
    ///
    /// Every `<joint>` becomes one [`Joint`] from the parent body's link to the body's own, so a
    /// body with several joints keeps them all. A body without a joint is welded by a fixed joint.
    pub fn to_robot(&self) -> Result<Robot, ParseError> {
        let mut m = Mapper::new(self)?;
        m.run(&self.root)?;
        Ok(m.robot)
    }
}

struct Mapper {
    robot: Robot,
    /// Radians per unit of a hinge angle.
    angle: f64,
    /// Order of the three `euler` angles; lowercase turns about the moving axes, uppercase the fixed ones.
    eulerseq: String,
    /// MuJoCo's default: a `range` limits the joint unless `limited="false"`.
    autolimits: bool,
    meshdir: String,
    /// Asset mesh name to the file it is read from and its scale.
    meshes: HashMap<String, (String, [f64; 3])>,
    /// Names in use, so a generated name never collides with a real one.
    link_names: HashSet<String>,
    joint_names: HashSet<String>,
}

impl Mapper {
    fn new(mjcf: &Mjcf) -> Result<Self, ParseError> {
        let root = &mjcf.root;
        let mut robot = Robot::new(Format::Mjcf);
        robot.name = root.attr("model").map(str::to_string);
        robot.span = root.anchor;
        let mut m = Mapper {
            robot,
            angle: std::f64::consts::PI / 180.0,
            eulerseq: "xyz".to_string(),
            autolimits: true,
            meshdir: String::new(),
            meshes: HashMap::new(),
            link_names: HashSet::from(["world".to_string()]),
            joint_names: HashSet::new(),
        };
        for compiler in children(root, "compiler") {
            if compiler.attr("angle") == Some("radian") {
                m.angle = 1.0;
            }
            if compiler.attr("autolimits") == Some("false") {
                m.autolimits = false;
            }
            if let Some(seq) = compiler.attr("eulerseq") {
                m.eulerseq = seq.to_string();
            }
            if let Some(dir) = compiler.attr("meshdir") {
                m.meshdir = dir.to_string();
            }
        }
        for (name, file, node) in mesh_assets(root) {
            if let Some(file) = file {
                let scale = numbers(node, "scale", 3)?.map_or([1.0; 3], |v| [v[0], v[1], v[2]]);
                m.meshes.insert(name.to_string(), (file.to_string(), scale));
            }
        }
        for world in children(root, "worldbody") {
            m.reserve_names(world);
        }
        Ok(m)
    }

    fn reserve_names(&mut self, node: &MjcfNode) {
        for c in &node.children {
            let taken = match c.tag.as_str() {
                "body" => &mut self.link_names,
                "joint" | "freejoint" => &mut self.joint_names,
                _ => continue,
            };
            if let Some(n) = c.attr("name") {
                taken.insert(n.to_string());
            }
            self.reserve_names(c);
        }
    }

    fn run(&mut self, root: &MjcfNode) -> Result<(), ParseError> {
        for section in UNMAPPED_SECTIONS {
            for c in children(root, section).flat_map(|s| &s.children) {
                self.lose(&c.tag, c.attr("name"), c);
            }
        }
        for world in children(root, "worldbody") {
            if self.robot.links.is_empty() {
                self.robot.links.push(Link {
                    name: "world".into(),
                    inertial: None,
                    shapes: Vec::new(),
                    span: world.anchor,
                });
            }
            self.contents(world, "world", 0, [0.0; 3])?;
        }
        Ok(())
    }

    /// Bodies and geoms inside a body (or the worldbody) whose link is `parent` at index `link`.
    /// `offset` is where the link frame sits in the body frame: the position of the body's joint.
    fn contents(
        &mut self,
        node: &MjcfNode,
        parent: &str,
        link: usize,
        offset: [f64; 3],
    ) -> Result<(), ParseError> {
        for c in &node.children {
            match c.tag.as_str() {
                "body" => self.body(c, parent, offset)?,
                "geom" => self.geom(c, link, offset)?,
                t if UNEXPANDED.contains(&t) => self.lose(t, c.attr("name"), c),
                _ => {}
            }
        }
        Ok(())
    }

    fn body(&mut self, node: &MjcfNode, parent: &str, offset: [f64; 3]) -> Result<(), ParseError> {
        let name = match node.attr("name") {
            Some(n) => n.to_string(),
            None => unique(
                &format!("body{}", self.robot.links.len()),
                &mut self.link_names,
            ),
        };
        let link = self.robot.links.len();
        // URDF puts the link frame on its joint; MJCF keeps it on the body and moves the joint by `pos`.
        let joints: Vec<&MjcfNode> = node
            .children
            .iter()
            .filter(|c| c.tag == "joint" || c.tag == "freejoint")
            .collect();
        let anchor = match joints.first() {
            Some(j) => numbers(j, "pos", 3)?.map_or([0.0; 3], |v| [v[0], v[1], v[2]]),
            None => [0.0; 3],
        };
        let inertial = match children(node, "inertial").next() {
            Some(i) => {
                let mut inertial = self.inertial(i)?;
                inertial.origin.xyz = sub(inertial.origin.xyz, anchor);
                Some(inertial)
            }
            None => None,
        };
        self.robot.links.push(Link {
            name: name.clone(),
            inertial,
            shapes: Vec::new(),
            span: node.anchor,
        });
        let pos = numbers(node, "pos", 3)?.map_or([0.0; 3], |v| [v[0], v[1], v[2]]);
        let rotation = self.rotation(node)?;
        let origin = Origin {
            xyz: add(sub(pos, offset), turn(&rotation, anchor)),
            rpy: self.orientation(node)?,
        };
        for j in &joints {
            // Only the first joint places the link frame; a later one elsewhere cannot be held.
            if numbers(j, "pos", 3)?.is_some_and(|v| v != anchor) {
                self.lose("joint/pos", j.attr("name"), j);
            }
            let joint = self.joint(j, parent, &name, origin)?;
            self.robot.joints.push(joint);
        }
        if joints.is_empty() {
            let joint_name = unique(&name, &mut self.joint_names);
            self.robot.joints.push(Joint {
                name: joint_name,
                parent: parent.to_string(),
                child: name.clone(),
                kind: JointKind::Fixed,
                origin,
                limit: None,
                mimic: None,
                axis: None,
                dynamics: None,
                span: node.anchor,
            });
        }
        self.contents(node, &name, link, anchor)
    }

    fn joint(
        &mut self,
        node: &MjcfNode,
        parent: &str,
        child: &str,
        origin: Origin,
    ) -> Result<Joint, ParseError> {
        let ty = if node.tag == "freejoint" {
            "free"
        } else {
            node.attr("type").unwrap_or("hinge")
        };
        let limited = match node.attr("limited") {
            Some("false") => false,
            Some("true") => true,
            _ => self.autolimits,
        };
        let range = match (limited, ty) {
            (true, "hinge" | "slide") => numbers(node, "range", 2)?,
            _ => None,
        };
        let scale = if ty == "hinge" { self.angle } else { 1.0 };
        let kind = match ty {
            "hinge" if range.is_some() => JointKind::Revolute,
            "hinge" => JointKind::Continuous,
            "slide" => JointKind::Prismatic,
            "free" => JointKind::Floating,
            other => JointKind::Other(other.to_string()),
        };
        let axis = match ty {
            "hinge" | "slide" => {
                Some(numbers(node, "axis", 3)?.map_or([0.0, 0.0, 1.0], |v| [v[0], v[1], v[2]]))
            }
            _ => None,
        };
        let damping = numbers(node, "damping", 1)?.map(|v| v[0]);
        let friction = numbers(node, "frictionloss", 1)?.map(|v| v[0]);
        let dynamics =
            (damping.is_some() || friction.is_some()).then_some(Dynamics { damping, friction });
        let name = match node.attr("name") {
            Some(n) => n.to_string(),
            None => unique(
                &format!("joint{}", self.robot.joints.len()),
                &mut self.joint_names,
            ),
        };
        Ok(Joint {
            name,
            parent: parent.to_string(),
            child: child.to_string(),
            kind,
            origin,
            limit: range.map(|r| Limit {
                lower: Some(r[0] * scale),
                upper: Some(r[1] * scale),
                effort: None,
                velocity: None,
            }),
            mimic: None,
            axis,
            dynamics,
            span: node.anchor,
        })
    }

    fn inertial(&self, node: &MjcfNode) -> Result<Inertial, ParseError> {
        let inertia = if let Some(d) = numbers(node, "diaginertia", 3)? {
            Some(Inertia {
                ixx: d[0],
                iyy: d[1],
                izz: d[2],
                ixy: 0.0,
                ixz: 0.0,
                iyz: 0.0,
            })
        } else {
            // MuJoCo lists the full tensor as M11 M22 M33 M12 M13 M23.
            numbers(node, "fullinertia", 6)?.map(|f| Inertia {
                ixx: f[0],
                iyy: f[1],
                izz: f[2],
                ixy: f[3],
                ixz: f[4],
                iyz: f[5],
            })
        };
        let mass = numbers(node, "mass", 1)?.map(|v| v[0]);
        Ok(Inertial {
            mass,
            inertia,
            origin: Origin {
                xyz: numbers(node, "pos", 3)?.map_or([0.0; 3], |v| [v[0], v[1], v[2]]),
                rpy: self.orientation(node)?,
            },
            span: node.anchor,
        })
    }

    fn orientation(&self, node: &MjcfNode) -> Result<[f64; 3], ParseError> {
        // Fixed axes x, y, z are the rpy convention, so the angles are already the answer; going
        // through a matrix would add rounding and lose roll and yaw at a pitch of 90 degrees.
        if self.eulerseq == "XYZ" && node.attr("quat").is_none() && node.attr("axisangle").is_none()
        {
            if let Some(e) = numbers(node, "euler", 3)? {
                return Ok([e[0] * self.angle, e[1] * self.angle, e[2] * self.angle]);
            }
        }
        Ok(rpy(&self.rotation(node)?))
    }

    /// The rotation of whichever orientation attribute `node` carries, in MuJoCo's priority order.
    fn rotation(&self, node: &MjcfNode) -> Result<Mat, ParseError> {
        let bad = |key: &str| position_error(node, format!("attribute `{key}` has no direction"));
        let matrix = if let Some(q) = numbers(node, "quat", 4)? {
            from_quat([q[0], q[1], q[2], q[3]]).ok_or_else(|| bad("quat"))?
        } else if let Some(a) = numbers(node, "axisangle", 4)? {
            from_axis_angle([a[0], a[1], a[2]], a[3] * self.angle)
                .ok_or_else(|| bad("axisangle"))?
        } else if let Some(e) = numbers(node, "euler", 3)? {
            self.euler(node, &e)?
        } else if let Some(v) = numbers(node, "xyaxes", 6)? {
            from_xyaxes([v[0], v[1], v[2]], [v[3], v[4], v[5]]).ok_or_else(|| bad("xyaxes"))?
        } else if let Some(z) = numbers(node, "zaxis", 3)? {
            from_zaxis([z[0], z[1], z[2]]).ok_or_else(|| bad("zaxis"))?
        } else {
            return Ok(about(0, 0.0));
        };
        Ok(matrix)
    }

    fn euler(&self, node: &MjcfNode, angles: &[f64]) -> Result<Mat, ParseError> {
        let seq: Vec<char> = self.eulerseq.chars().collect();
        let valid = seq.len() == 3 && seq.iter().all(|c| "xyzXYZ".contains(*c));
        if !valid {
            let msg = format!(
                "compiler `eulerseq` must be three of x, y, z, found `{}`",
                self.eulerseq
            );
            return Err(position_error(node, msg));
        }
        let mut r = about(0, 0.0);
        for (c, angle) in seq.iter().zip(angles) {
            let index = c.to_ascii_lowercase() as usize - 'x' as usize;
            let turn = about(index, angle * self.angle);
            r = if c.is_ascii_lowercase() {
                mul(&r, &turn)
            } else {
                mul(&turn, &r)
            };
        }
        Ok(r)
    }

    fn geom(&mut self, node: &MjcfNode, link: usize, offset: [f64; 3]) -> Result<(), ParseError> {
        let name = node.attr("name");
        let geometry = match node.attr("mesh") {
            Some(m) => match self.meshes.get(m) {
                Some((file, scale)) => Some(Geometry::Mesh {
                    filename: join(&self.meshdir, file),
                    scale: *scale,
                }),
                // RS017 reports a mesh geom naming an unknown asset.
                None => return Ok(()),
            },
            None => primitive(node)?,
        };
        let segment = if geometry.is_none() {
            segment(node)?
        } else {
            None
        };
        let geometry = geometry.or_else(|| segment.as_ref().map(|s| s.0.clone()));
        let Some(geometry) = geometry else {
            self.lose("geom", name, node);
            return Ok(());
        };
        // MuJoCo collides two geoms only when contype/conaffinity say so; both 0 is look-only.
        // Groups 3 and up are hidden by default, the convention for collision-only geometry.
        let hidden = node
            .attr("group")
            .and_then(|g| g.parse::<i32>().ok())
            .is_some_and(|g| g >= 3);
        let role = if node.attr("contype") == Some("0") && node.attr("conaffinity") == Some("0") {
            Role::Visual
        } else if hidden {
            Role::Collision
        } else {
            Role::Both
        };
        // MuJoCo ignores pos and the orientation attributes on a geom with fromto.
        let (xyz, rpy) = match &segment {
            Some((_, middle, rotation)) => (*middle, rpy(rotation)),
            None => (
                numbers(node, "pos", 3)?.map_or([0.0; 3], |v| [v[0], v[1], v[2]]),
                self.orientation(node)?,
            ),
        };
        let xyz = sub(xyz, offset);
        self.robot.links[link].shapes.push(Shape {
            role,
            geometry,
            origin: Origin { xyz, rpy },
            span: node.anchor,
        });
        Ok(())
    }

    fn lose(&mut self, what: &str, name: Option<&str>, node: &MjcfNode) {
        self.robot.unmapped.push(Unmapped {
            what: what.to_string(),
            name: name.map(str::to_string),
            span: node.anchor,
        });
    }
}

pub(crate) fn children<'a>(node: &'a MjcfNode, tag: &'a str) -> impl Iterator<Item = &'a MjcfNode> {
    node.children.iter().filter(move |c| c.tag == tag)
}

/// Every mesh asset as (name, file). A mesh built from inline vertices has a name and no file;
/// one with neither has no name to refer to it by and is left out.
pub(crate) fn mesh_assets(
    root: &MjcfNode,
) -> impl Iterator<Item = (&str, Option<&str>, &MjcfNode)> {
    children(root, "asset")
        .flat_map(|a| children(a, "mesh"))
        .filter_map(|m| {
            let file = m.attr("file");
            let name = m.attr("name").or_else(|| file.map(stem))?;
            Some((name, file, m))
        })
}

/// The file name without directory and extension, which MuJoCo uses as a mesh's default name.
fn stem(file: &str) -> &str {
    let name = file.rsplit(['/', '\\']).next().unwrap_or(file);
    name.rsplit_once('.').map_or(name, |(s, _)| s)
}

fn join(dir: &str, file: &str) -> String {
    if dir.is_empty() || file.starts_with('/') {
        file.to_string()
    } else {
        format!("{}/{file}", dir.trim_end_matches('/'))
    }
}

/// `base`, or `base_2`, `base_3`... when taken; the result is added to `taken`.
fn unique(base: &str, taken: &mut HashSet<String>) -> String {
    let mut name = base.to_string();
    let mut n = 1;
    while taken.contains(&name) {
        n += 1;
        name = format!("{base}_{n}");
    }
    taken.insert(name.clone());
    name
}

/// A primitive geom's shape, or `None` when the model cannot hold it (`fromto`, ellipsoid, plane,
/// heightfield, sdf, or a size with too few numbers). MuJoCo sizes are half extents.
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn turn(r: &Mat, v: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|i| r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2])
}

/// A capsule or cylinder given by its end points: the shape, the middle of the segment and the
/// rotation that turns Z onto it. Other types with `fromto` are not held.
fn segment(node: &MjcfNode) -> Result<Option<(Geometry, [f64; 3], Mat)>, ParseError> {
    let Some(ends) = numbers(node, "fromto", 6)? else {
        return Ok(None);
    };
    let size = list(node, "size")?.unwrap_or_default();
    let (from, to) = ([ends[0], ends[1], ends[2]], [ends[3], ends[4], ends[5]]);
    let along = sub(to, from);
    let length = along.iter().map(|c| c * c).sum::<f64>().sqrt();
    let geometry = match (node.attr("type"), size.first()) {
        (Some("capsule"), Some(&radius)) => Geometry::Capsule { radius, length },
        (Some("cylinder"), Some(&radius)) => Geometry::Cylinder { radius, length },
        _ => return Ok(None),
    };
    let rotation = from_zaxis(along)
        .ok_or_else(|| position_error(node, "attribute `fromto` has no direction".to_string()))?;
    let middle = add(from, along.map(|c| c / 2.0));
    Ok(Some((geometry, middle, rotation)))
}

fn primitive(node: &MjcfNode) -> Result<Option<Geometry>, ParseError> {
    if node.attr("fromto").is_some() {
        return Ok(None);
    }
    let size = list(node, "size")?.unwrap_or_default();
    Ok(
        match (node.attr("type").unwrap_or("sphere"), size.as_slice()) {
            ("sphere", [r, ..]) => Some(Geometry::Sphere { radius: *r }),
            ("box", [x, y, z, ..]) => Some(Geometry::Box {
                size: [2.0 * x, 2.0 * y, 2.0 * z],
            }),
            ("cylinder", [r, h, ..]) => Some(Geometry::Cylinder {
                radius: *r,
                length: 2.0 * h,
            }),
            ("capsule", [r, h, ..]) => Some(Geometry::Capsule {
                radius: *r,
                length: 2.0 * h,
            }),
            _ => None,
        },
    )
}

fn position_error(node: &MjcfNode, msg: String) -> ParseError {
    let file = node
        .file
        .as_deref()
        .map(|f| format!("{f}:"))
        .unwrap_or_default();
    ParseError(format!("{file}{}:{}: {msg}", node.span.line, node.span.col))
}

/// The attribute as finite numbers of any count, or an error that names the element's position.
fn list(node: &MjcfNode, key: &str) -> Result<Option<Vec<f64>>, ParseError> {
    let Some(v) = node.attr(key) else {
        return Ok(None);
    };
    let nums: Vec<f64> = v
        .split_whitespace()
        .map(|t| t.parse::<f64>().ok().filter(|n| n.is_finite()))
        .collect::<Option<_>>()
        .ok_or_else(|| {
            position_error(
                node,
                format!("invalid number in `{v}` in attribute `{key}`"),
            )
        })?;
    Ok(Some(nums))
}

/// The attribute as `count` finite numbers, or an error that names the element's position.
fn numbers(node: &MjcfNode, key: &str, count: usize) -> Result<Option<Vec<f64>>, ParseError> {
    let Some(nums) = list(node, key)? else {
        return Ok(None);
    };
    if nums.len() != count {
        return Err(position_error(
            node,
            format!(
                "attribute `{key}` needs {count} numbers, found {}",
                nums.len()
            ),
        ));
    }
    Ok(Some(nums))
}
