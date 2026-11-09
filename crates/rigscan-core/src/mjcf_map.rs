//! Maps the resolved MJCF tree onto the format-neutral [`Robot`].

use crate::mjcf::{Mjcf, MjcfNode};
use crate::model::{
    Format, Inertia, Inertial, Joint, JointKind, Limit, Link, MeshRef, Origin, ParseError, Robot,
    Unmapped,
};
use std::collections::{HashMap, HashSet};

/// Sections whose children the neutral model has no place for.
const UNMAPPED_SECTIONS: [&str; 6] = [
    "actuator", "tendon", "equality", "contact", "sensor", "keyframe",
];

/// Elements that can hold bodies but are not expanded here, so what is inside is lost.
const UNEXPANDED: [&str; 5] = ["frame", "replicate", "attach", "composite", "flexcomp"];

/// Ways MJCF can give a body's orientation. Rotation is not converted, so each is reported.
const ORIENTATION: [&str; 5] = ["quat", "axisangle", "euler", "xyaxes", "zaxis"];

impl Mjcf {
    /// Map the resolved tree onto the format-neutral model.
    ///
    /// Every `<joint>` becomes one [`Joint`] from the parent body's link to the body's own, so a
    /// body with several joints keeps them all. A body without a joint is welded by a fixed joint.
    pub fn to_robot(&self) -> Result<Robot, ParseError> {
        let mut m = Mapper::new(self);
        m.run(&self.root)?;
        Ok(m.robot)
    }
}

struct Mapper {
    robot: Robot,
    /// Radians per unit of a hinge angle.
    angle: f64,
    /// MuJoCo's default: a `range` limits the joint unless `limited="false"`.
    autolimits: bool,
    meshdir: String,
    /// Asset mesh name to the file it is read from.
    meshes: HashMap<String, String>,
    /// Names in use, so a generated name never collides with a real one.
    link_names: HashSet<String>,
    joint_names: HashSet<String>,
}

impl Mapper {
    fn new(mjcf: &Mjcf) -> Self {
        let root = &mjcf.root;
        let mut robot = Robot::new(Format::Mjcf);
        robot.name = root.attr("model").map(str::to_string);
        robot.span = root.anchor;
        let mut m = Mapper {
            robot,
            angle: std::f64::consts::PI / 180.0,
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
            if let Some(dir) = compiler.attr("meshdir") {
                m.meshdir = dir.to_string();
            }
        }
        for mesh in children(root, "asset").flat_map(|a| children(a, "mesh")) {
            if let Some(file) = mesh.attr("file") {
                let name = mesh.attr("name").unwrap_or_else(|| stem(file));
                m.meshes.insert(name.to_string(), file.to_string());
            }
        }
        for world in children(root, "worldbody") {
            m.reserve_names(world);
        }
        m
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
                    meshes: Vec::new(),
                    span: world.anchor,
                });
            }
            self.contents(world, "world", 0)?;
        }
        Ok(())
    }

    /// Bodies and geoms inside a body (or the worldbody) whose link is `parent` at index `link`.
    fn contents(&mut self, node: &MjcfNode, parent: &str, link: usize) -> Result<(), ParseError> {
        for c in &node.children {
            match c.tag.as_str() {
                "body" => self.body(c, parent)?,
                "geom" => self.geom(c, link),
                t if UNEXPANDED.contains(&t) => self.lose(t, c.attr("name"), c),
                _ => {}
            }
        }
        Ok(())
    }

    fn body(&mut self, node: &MjcfNode, parent: &str) -> Result<(), ParseError> {
        let name = match node.attr("name") {
            Some(n) => n.to_string(),
            None => unique(
                &format!("body{}", self.robot.links.len()),
                &mut self.link_names,
            ),
        };
        let link = self.robot.links.len();
        let inertial = match children(node, "inertial").next() {
            Some(i) => Some(inertial(i)?),
            None => None,
        };
        self.robot.links.push(Link {
            name: name.clone(),
            inertial,
            meshes: Vec::new(),
            span: node.anchor,
        });
        for key in ORIENTATION {
            if node.attr(key).is_some() {
                self.lose(key, Some(&name), node);
            }
        }

        let origin = Origin {
            xyz: match numbers(node, "pos", 3)? {
                Some(v) => [v[0], v[1], v[2]],
                None => [0.0; 3],
            },
            ..Origin::default()
        };
        let mut any = false;
        for j in node
            .children
            .iter()
            .filter(|c| c.tag == "joint" || c.tag == "freejoint")
        {
            any = true;
            let joint = self.joint(j, parent, &name, origin)?;
            self.robot.joints.push(joint);
        }
        if !any {
            let joint_name = unique(&name, &mut self.joint_names);
            self.robot.joints.push(Joint {
                name: joint_name,
                parent: parent.to_string(),
                child: name.clone(),
                kind: JointKind::Fixed,
                origin,
                limit: None,
                mimic: None,
                span: node.anchor,
            });
        }
        self.contents(node, &name, link)
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
            span: node.anchor,
        })
    }

    fn geom(&mut self, node: &MjcfNode, link: usize) {
        let file = node.attr("mesh").and_then(|m| self.meshes.get(m));
        match file {
            Some(file) => {
                let filename = join(&self.meshdir, file);
                self.robot.links[link].meshes.push(MeshRef {
                    filename,
                    span: node.anchor,
                });
            }
            // A mesh geom naming an unknown asset is left for the MJCF rules to report.
            None if node.attr("mesh").is_some() => {}
            None => self.lose("geom", node.attr("name"), node),
        }
    }

    fn lose(&mut self, what: &str, name: Option<&str>, node: &MjcfNode) {
        self.robot.unmapped.push(Unmapped {
            what: what.to_string(),
            name: name.map(str::to_string),
            span: node.anchor,
        });
    }
}

fn children<'a>(node: &'a MjcfNode, tag: &'a str) -> impl Iterator<Item = &'a MjcfNode> {
    node.children.iter().filter(move |c| c.tag == tag)
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

fn inertial(node: &MjcfNode) -> Result<Inertial, ParseError> {
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
        span: node.anchor,
    })
}

/// The attribute as `count` finite numbers, or an error that names the element's position.
fn numbers(node: &MjcfNode, key: &str, count: usize) -> Result<Option<Vec<f64>>, ParseError> {
    let Some(v) = node.attr(key) else {
        return Ok(None);
    };
    let at = |msg: String| {
        let file = node
            .file
            .as_deref()
            .map(|f| format!("{f}:"))
            .unwrap_or_default();
        ParseError(format!("{file}{}:{}: {msg}", node.span.line, node.span.col))
    };
    let nums: Vec<f64> = v
        .split_whitespace()
        .map(|t| t.parse::<f64>().ok().filter(|n| n.is_finite()))
        .collect::<Option<_>>()
        .ok_or_else(|| at(format!("invalid number in `{v}` in attribute `{key}`")))?;
    if nums.len() != count {
        return Err(at(format!(
            "attribute `{key}` needs {count} numbers, found {}",
            nums.len()
        )));
    }
    Ok(Some(nums))
}
