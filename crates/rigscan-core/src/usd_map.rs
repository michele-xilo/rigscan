//! Maps the UsdPhysics data onto the format-neutral [`Robot`].

use crate::model::{
    Format, Inertia, Inertial, Joint, JointKind, Limit, Link, Origin, ParseError, Robot, Shape,
    Span,
};
use crate::rotation::{apply, from_quat, mul, rpy, transpose, Mat};
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
                inertia: b.inertia.map(|d| tensor(d, b.principal_axes, kg * m * m)),
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
                    geometry: sh.geometry.clone(),
                    origin: sh.origin,
                    span: Span::NONE,
                });
            }
        }
        for j in &self.joints {
            robot.joints.push(joint(j, m, kg));
        }
        Ok(robot)
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

fn joint(j: &crate::usd::UsdJoint, meters: f64, kilograms: f64) -> Joint {
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
    let moves = matches!(
        kind,
        JointKind::Revolute | JointKind::Continuous | JointKind::Prismatic
    );
    // With no `body1` the second frame is in the world, and the body is held to it: the world is
    // the parent and `body0` the child, so each frame goes to the other side.
    let flip = j.body1.is_none() && j.body0.is_some();
    let (parent_side, child_side) = if flip {
        ((j.local_pos1, j.local_rot1), (j.local_pos0, j.local_rot0))
    } else {
        ((j.local_pos0, j.local_rot0), (j.local_pos1, j.local_rot1))
    };
    // The axis token is in the joint frame, which is the child's frame in the child body.
    let r_child = from_quat(child_side.1).unwrap_or(IDENTITY);
    let axis = moves.then(|| apply(&r_child, j.axis));
    // A drive's torque or force carries the same unit as the joint's effort.
    let effort = j
        .max_force
        .map(|f| f * kilograms * meters * if revolute { meters } else { 1.0 });
    let limit = (moves && (limited || effort.is_some())).then(|| Limit {
        lower: j.lower.map(|v| v * scale),
        upper: j.upper.map(|v| v * scale),
        effort,
        velocity: None,
    });
    // The child frame sits where the parent's frame is, offset back by its own: T_p * T_c^-1.
    let r_parent = from_quat(parent_side.1).unwrap_or(IDENTITY);
    let turn = mul(&r_parent, &transpose(&r_child));
    let back = apply(&turn, child_side.0);
    Joint {
        name: j.path.clone(),
        parent: if flip { None } else { j.body0.clone() }.unwrap_or_else(|| WORLD.into()),
        child: if flip {
            j.body0.clone()
        } else {
            j.body1.clone()
        }
        .unwrap_or_else(|| WORLD.into()),
        kind,
        origin: Origin {
            xyz: std::array::from_fn(|i| (parent_side.0[i] - back[i]) * meters),
            rpy: joint_rpy(&turn),
        },
        limit,
        mimic: None,
        axis,
        dynamics: None,
        span: Span::NONE,
    }
}

/// Most joints carry no rotation, so identity skips the trigonometry.
fn joint_rpy(r: &Mat) -> [f64; 3] {
    if *r == IDENTITY {
        return [0.0; 3];
    }
    rpy(r)
}

const IDENTITY: Mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// The full tensor in the body frame of a tensor that is diagonal in the principal frame.
fn tensor(d: [f64; 3], axes: [f64; 4], scale: f64) -> Inertia {
    let r = from_quat(axes).unwrap_or(IDENTITY);
    let at = |i: usize, j: usize| (0..3).map(|k| r[i][k] * d[k] * r[j][k]).sum::<f64>() * scale;
    Inertia {
        ixx: at(0, 0),
        iyy: at(1, 1),
        izz: at(2, 2),
        ixy: at(0, 1),
        ixz: at(0, 2),
        iyz: at(1, 2),
    }
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
