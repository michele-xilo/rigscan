//! Maps the UsdPhysics data onto the format-neutral [`Robot`].

use crate::model::{
    Format, Geometry, Inertia, Inertial, Joint, JointKind, Limit, Link, Origin, ParseError, Robot,
    Shape, Span,
};
use crate::rotation::{from_quat, rpy};
use crate::usd::Usd;

/// The link a joint with no `body0` or `body1` is attached to.
const WORLD: &str = "world";

impl Usd {
    /// Map the bodies and joints onto the neutral model. A body is a link named by its prim path,
    /// and a joint connects `body0` to `body1`; a missing end is the world. Nothing is added to
    /// the structure beyond that root link.
    pub fn to_robot(&self) -> Result<Robot, ParseError> {
        let mut robot = Robot::new(Format::Usd);
        robot.span = Span::NONE;
        robot.unmapped = self.unmapped.clone();
        if self
            .joints
            .iter()
            .any(|j| j.body0.is_none() || j.body1.is_none())
        {
            robot.links.push(link(WORLD, None));
        }
        let (m, kg) = (self.meters_per_unit, self.kilograms_per_unit);
        for b in &self.bodies {
            let inertial = (b.mass.is_some() || b.inertia.is_some()).then(|| Inertial {
                mass: b.mass.map(|v| v * kg),
                inertia: b.inertia.map(|d| Inertia {
                    ixx: d[0] * kg * m * m,
                    iyy: d[1] * kg * m * m,
                    izz: d[2] * kg * m * m,
                    ixy: 0.0,
                    ixz: 0.0,
                    iyz: 0.0,
                }),
                origin: Origin {
                    xyz: b.center_of_mass.unwrap_or([0.0; 3]).map(|v| v * m),
                    ..Origin::default()
                },
                span: Span::NONE,
            });
            robot.links.push(link(&b.path, inertial));
        }
        for sh in &self.shapes {
            if let Some(link) = robot.links.iter_mut().find(|l| l.name == sh.body) {
                link.shapes.push(Shape {
                    role: sh.role,
                    geometry: scaled(&sh.geometry, m),
                    origin: Origin {
                        xyz: sh.origin.xyz.map(|v| v * m),
                        rpy: sh.origin.rpy,
                    },
                    span: Span::NONE,
                });
            }
        }
        for j in &self.joints {
            robot.joints.push(joint(j, m));
        }
        Ok(robot)
    }
}

/// Primitive sizes from stage units to metres.
fn scaled(g: &Geometry, m: f64) -> Geometry {
    match g {
        Geometry::Box { size } => Geometry::Box {
            size: size.map(|v| v * m),
        },
        Geometry::Sphere { radius } => Geometry::Sphere { radius: radius * m },
        Geometry::Cylinder { radius, length } => Geometry::Cylinder {
            radius: radius * m,
            length: length * m,
        },
        Geometry::Capsule { radius, length } => Geometry::Capsule {
            radius: radius * m,
            length: length * m,
        },
        mesh @ Geometry::Mesh { .. } => mesh.clone(),
    }
}

fn link(name: &str, inertial: Option<Inertial>) -> Link {
    Link {
        name: name.to_string(),
        inertial,
        shapes: Vec::new(),
        span: Span::NONE,
    }
}

fn joint(j: &crate::usd::UsdJoint, meters: f64) -> Joint {
    let revolute = j.type_name == "PhysicsRevoluteJoint";
    let limited = j.lower.is_some() || j.upper.is_some();
    // USD authors revolute limits in degrees and prismatic ones in stage units.
    let scale = if revolute {
        std::f64::consts::PI / 180.0
    } else {
        meters
    };
    let kind = match j.type_name.as_str() {
        "PhysicsRevoluteJoint" if limited => JointKind::Revolute,
        "PhysicsRevoluteJoint" => JointKind::Continuous,
        "PhysicsPrismaticJoint" => JointKind::Prismatic,
        "PhysicsFixedJoint" => JointKind::Fixed,
        other => JointKind::Other(other_name(other)),
    };
    let axis = matches!(
        kind,
        JointKind::Revolute | JointKind::Continuous | JointKind::Prismatic
    )
    .then_some(j.axis);
    let limit =
        (limited && matches!(kind, JointKind::Revolute | JointKind::Prismatic)).then(|| Limit {
            lower: j.lower.map(|v| v * scale),
            upper: j.upper.map(|v| v * scale),
            effort: None,
            velocity: None,
        });
    Joint {
        name: j.path.clone(),
        parent: j.body0.clone().unwrap_or_else(|| WORLD.into()),
        child: j.body1.clone().unwrap_or_else(|| WORLD.into()),
        kind,
        origin: Origin {
            xyz: j.local_pos0.map(|v| v * meters),
            rpy: joint_rpy(j.local_rot0),
        },
        limit,
        mimic: None,
        axis,
        dynamics: None,
        span: Span::NONE,
    }
}

/// Most joints carry no rotation, so identity skips the trigonometry.
fn joint_rpy(q: [f64; 4]) -> [f64; 3] {
    if q == [1.0, 0.0, 0.0, 0.0] {
        return [0.0; 3];
    }
    from_quat(q).map_or([0.0; 3], |m| rpy(&m))
}

/// `PhysicsSphericalJoint` to `spherical`; the generic `PhysicsJoint` to `joint`.
fn other_name(type_name: &str) -> String {
    let core = type_name.strip_prefix("Physics").unwrap_or(type_name);
    let core = core.strip_suffix("Joint").unwrap_or(core);
    if core.is_empty() {
        "joint".into()
    } else {
        core.to_ascii_lowercase()
    }
}
