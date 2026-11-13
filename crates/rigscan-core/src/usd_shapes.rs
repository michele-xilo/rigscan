//! Primitive shapes of a stage: gprims placed in the frame of the rigid body above them.

use crate::model::{Geometry, Origin, Role, Span, Unmapped};
use crate::rotation::{about, from_quat, mul, rpy, Mat};
use crate::usd::{number, quat, vec3, Usd, UsdShape};
use openusd::sdf::Value;
use openusd::usd::{Prim, Stage};
use std::f64::consts::FRAC_PI_2;

const IDENTITY: Mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// A rotation and scale, then a translation, acting on column vectors: `p' = lin * p + t`.
#[derive(Clone, Copy)]
struct Affine {
    lin: Mat,
    t: [f64; 3],
}

impl Affine {
    const IDENTITY: Affine = Affine {
        lin: IDENTITY,
        t: [0.0; 3],
    };

    /// `self` applied after `inner`.
    fn after(&self, inner: &Affine) -> Affine {
        let moved = apply(&self.lin, inner.t);
        Affine {
            lin: mul(&self.lin, &inner.lin),
            t: [
                moved[0] + self.t[0],
                moved[1] + self.t[1],
                moved[2] + self.t[2],
            ],
        }
    }

    fn inverse(&self) -> Option<Affine> {
        let lin = invert(&self.lin)?;
        let back = apply(&lin, self.t);
        Some(Affine {
            lin,
            t: back.map(|v| -v),
        })
    }
}

fn apply(m: &Mat, v: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|i| m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2])
}

fn invert(m: &Mat) -> Option<Mat> {
    let c = |a: usize, b: usize, c: usize, d: usize| m[a][b] * m[c][d] - m[a][d] * m[c][b];
    let cof = [
        [c(1, 1, 2, 2), -c(1, 0, 2, 2), c(1, 0, 2, 1)],
        [-c(0, 1, 2, 2), c(0, 0, 2, 2), -c(0, 0, 2, 1)],
        [c(0, 1, 1, 2), -c(0, 0, 1, 2), c(0, 0, 1, 1)],
    ];
    let det: f64 = (0..3).map(|j| m[0][j] * cof[0][j]).sum();
    (det.abs() > 1e-12).then(|| {
        let mut out = [[0.0; 3]; 3];
        for (i, row) in out.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = cof[j][i] / det;
            }
        }
        out
    })
}

/// The shapes of every `Cube`, `Sphere`, `Cylinder` and `Capsule` under a body. A shape whose
/// pose or size the model cannot hold, and a collision prim that is not a shape, are listed as lost.
pub(crate) fn read(stage: &Stage, paths: &[String], usd: &mut Usd) {
    let meters = usd.meters_per_unit;
    let prims = paths
        .iter()
        .filter_map(|p| Some((p, stage.prim(p.as_str()).ok()?)));
    for (path, prim) in prims {
        let ty = prim
            .type_name()
            .ok()
            .flatten()
            .map(|t| t.to_string())
            .unwrap_or_default();
        if !matches!(ty.as_str(), "Cube" | "Sphere" | "Cylinder" | "Capsule") {
            continue;
        }
        let collision = prim
            .api_schemas()
            .is_ok_and(|a| a.iter().any(|x| x.as_str() == "PhysicsCollisionAPI"));
        let shape =
            body_of(usd, path).and_then(|body| shape(stage, &prim, &ty, &body, collision, meters));
        match shape {
            Some(s) => usd.shapes.push(s),
            None if collision => usd.unmapped.push(Unmapped {
                what: "collision".to_string(),
                name: Some(path.clone()),
                span: Span::NONE,
            }),
            None => {}
        }
    }
}

/// The nearest rigid body or joint target at or above the prim.
fn body_of(usd: &Usd, path: &str) -> Option<String> {
    let mut p = path;
    loop {
        if usd.bodies.iter().any(|b| b.path == p) {
            return Some(p.to_string());
        }
        match p.rfind('/') {
            Some(at) if at > 0 => p = &p[..at],
            _ => return None,
        }
    }
}

fn shape(
    stage: &Stage,
    prim: &Prim,
    ty: &str,
    body: &str,
    collision: bool,
    meters: f64,
) -> Option<UsdShape> {
    let stage_body = stage.prim(body).ok()?;
    let relative = world(prim)?;
    let relative = world(&stage_body)?.inverse()?.after(&relative);
    let (scale, rotation) = decompose(&relative.lin)?;
    let (geometry, turn) = geometry(prim, ty, scale.map(|s| s * meters))?;
    let hidden = is_hidden(prim);
    let role = match (collision, hidden) {
        (true, true) => Role::Collision,
        (true, false) => Role::Both,
        (false, false) => Role::Visual,
        (false, true) => return None,
    };
    Some(UsdShape {
        body: body.to_string(),
        geometry,
        role,
        origin: Origin {
            xyz: relative.t.map(|v| v * meters),
            rpy: rpy(&mul(&rotation, &turn)),
        },
    })
}

/// Guides and proxies are not drawn, and neither is an invisible prim.
fn is_hidden(prim: &Prim) -> bool {
    let token = |name: &str| match prim.attribute(name).get::<Value>() {
        Ok(Some(Value::Token(t))) => Some(t.to_string()),
        _ => None,
    };
    matches!(token("purpose").as_deref(), Some("guide" | "proxy"))
        || token("visibility").as_deref() == Some("invisible")
}

/// Per-axis scale and the rotation left when it is taken out.
fn decompose(lin: &Mat) -> Option<([f64; 3], Mat)> {
    let scale = [0, 1, 2].map(|j| (0..3).map(|i| lin[i][j] * lin[i][j]).sum::<f64>().sqrt());
    if scale.iter().any(|s| *s < 1e-9) {
        return None;
    }
    let mut r = *lin;
    for row in &mut r {
        for (cell, s) in row.iter_mut().zip(scale) {
            *cell /= s;
        }
    }
    Some((scale, r))
}

/// The geometry, with `scale` taking stage units to metres, and the rotation from the shape's own axis onto the prim's Z.
fn geometry(prim: &Prim, ty: &str, scale: [f64; 3]) -> Option<(Geometry, Mat)> {
    let get = |name: &str, default: f64| match prim.attribute(name).get::<Value>() {
        Ok(Some(v)) => number(v),
        _ => Some(default),
    };
    let same = |a: f64, b: f64| (a - b).abs() <= 1e-6 * a.abs().max(b.abs());
    match ty {
        "Cube" => {
            let size = get("size", 2.0)?;
            Some((
                Geometry::Box {
                    size: scale.map(|s| size * s),
                },
                IDENTITY,
            ))
        }
        "Sphere" => {
            let radius = get("radius", 1.0)?;
            (same(scale[0], scale[1]) && same(scale[1], scale[2])).then(|| {
                (
                    Geometry::Sphere {
                        radius: radius * scale[0],
                    },
                    IDENTITY,
                )
            })
        }
        _ => {
            let capsule = ty == "Capsule";
            let radius = get("radius", if capsule { 0.5 } else { 1.0 })?;
            let height = get("height", if capsule { 1.0 } else { 2.0 })?;
            let axis = match prim.attribute("axis").get::<Value>() {
                Ok(Some(Value::Token(t))) => match t.as_str() {
                    "X" => 0,
                    "Y" => 1,
                    _ => 2,
                },
                _ => 2,
            };
            let across: Vec<f64> = (0..3).filter(|i| *i != axis).map(|i| scale[i]).collect();
            if !same(across[0], across[1]) {
                return None;
            }
            let (radius, length) = (radius * across[0], height * scale[axis]);
            let geometry = if capsule {
                Geometry::Capsule { radius, length }
            } else {
                Geometry::Cylinder { radius, length }
            };
            // Z onto X is a quarter turn about Y; Z onto Y is a quarter turn back about X.
            let turn = match axis {
                0 => about(1, FRAC_PI_2),
                1 => about(0, -FRAC_PI_2),
                _ => IDENTITY,
            };
            Some((geometry, turn))
        }
    }
}

/// The prim's transform in the stage, through every parent that is not cut off by a reset.
fn world(prim: &Prim) -> Option<Affine> {
    let mut chain = vec![prim.clone()];
    while let Some(p) = chain.last().and_then(Prim::parent) {
        if p.path().as_str() == "/" {
            break;
        }
        chain.push(p);
    }
    let mut world = Affine::IDENTITY;
    for p in chain.iter().rev() {
        let (local, reset) = local(p)?;
        world = if reset { local } else { world.after(&local) };
    }
    Some(world)
}

/// The prim's own transform, and whether it resets the stack above it. The first operation in
/// `xformOpOrder` is the outermost, so it is applied last to a point.
fn local(prim: &Prim) -> Option<(Affine, bool)> {
    let order = match prim.attribute("xformOpOrder").get::<Value>() {
        Ok(Some(Value::TokenVec(t))) => t,
        _ => return Some((Affine::IDENTITY, false)),
    };
    let mut local = Affine::IDENTITY;
    let mut reset = false;
    for token in order {
        let token = token.to_string();
        if token == "!resetXformStack!" {
            reset = true;
            continue;
        }
        let (name, inverted) = match token.strip_prefix("!invert!") {
            Some(n) => (n, true),
            None => (token.as_str(), false),
        };
        let value = prim.attribute(name).get::<Value>().ok().flatten()?;
        let op = operation(name.split(':').nth(1)?, &value)?;
        let op = if inverted { op.inverse()? } else { op };
        local = local.after(&op);
    }
    Some((local, reset))
}

fn operation(kind: &str, value: &Value) -> Option<Affine> {
    let linear = |lin: Mat| Affine { lin, t: [0.0; 3] };
    match kind {
        "translate" => Some(Affine {
            lin: IDENTITY,
            t: vec3(value.clone())?,
        }),
        "scale" => {
            let s = vec3(value.clone())?;
            Some(linear([
                [s[0], 0.0, 0.0],
                [0.0, s[1], 0.0],
                [0.0, 0.0, s[2]],
            ]))
        }
        "orient" => from_quat(quat(value)?).map(linear),
        "transform" => {
            let Value::Matrix4d(m) = value else {
                return None;
            };
            // USD stores matrices for row vectors: the translation is the last row.
            let m = m.0;
            Some(Affine {
                lin: [0, 1, 2].map(|i| [m[i], m[4 + i], m[8 + i]]),
                t: [m[12], m[13], m[14]],
            })
        }
        "rotateX" | "rotateY" | "rotateZ" => {
            let index = (kind.as_bytes()[6] - b'X') as usize;
            Some(linear(about(index, degrees(value)?)))
        }
        k if k.starts_with("rotate") && k.len() == 9 => {
            let angles = vec3(value.clone())?;
            let order = &k.as_bytes()[6..];
            // rotateXZY turns about X first, then Z, then Y; the angles are still given as x, y, z.
            let mut lin = IDENTITY;
            for axis in order {
                let index = (axis - b'X') as usize;
                lin = mul(&about(index, angles[index].to_radians()), &lin);
            }
            Some(linear(lin))
        }
        _ => None,
    }
}

fn degrees(value: &Value) -> Option<f64> {
    number(value.clone()).map(f64::to_radians)
}
