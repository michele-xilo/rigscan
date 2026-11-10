//! USD reader: UsdPhysics bodies and joints read from a composed stage.

use crate::model::{ParseError, Span, Unmapped};
use openusd::ar::{self, ResolvedPath};
use openusd::sdf::Value;
use openusd::usd::{Prim, PrimPredicate, Stage};
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::io;
use std::rc::Rc;
use std::time::SystemTime;

/// Supplies the bytes of a layer. The core does no file access, so the caller provides this.
pub trait UsdLoader {
    /// `path` is a layer path as the stage resolves it; `None` means it does not exist.
    fn load(&self, path: &str) -> Option<Vec<u8>>;
}

/// A prim a joint can connect: a rigid body, or a static prim such as a ground collider.
#[derive(Debug, Clone, PartialEq)]
pub struct UsdBody {
    pub path: String,
    pub rigid: bool,
    pub mass: Option<f64>,
    pub inertia: Option<[f64; 3]>,
    /// Centre of mass in the body frame; `None` when USD is left to compute it from the shapes.
    pub center_of_mass: Option<[f64; 3]>,
    /// A prim with `PhysicsCollisionAPI` is this prim or under it, with no rigid body in between.
    pub collision: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UsdJoint {
    pub path: String,
    /// The schema type name, for example `PhysicsRevoluteJoint`.
    pub type_name: String,
    /// `None` when the relationship has no target: the joint is anchored to the world.
    pub body0: Option<String>,
    pub body1: Option<String>,
    pub local_pos0: [f64; 3],
    /// Unit vector of `physics:axis` (`X`, `Y` or `Z`, default `X`); only revolute and prismatic joints use it.
    pub axis: [f64; 3],
    /// As authored: degrees for a revolute joint, stage units for a prismatic one.
    pub lower: Option<f64>,
    pub upper: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Usd {
    pub bodies: Vec<UsdBody>,
    pub joints: Vec<UsdJoint>,
    pub unmapped: Vec<Unmapped>,
    /// Prims with `PhysicsArticulationRootAPI`.
    pub articulation_roots: Vec<String>,
    /// References, payloads and sublayers the loader could not supply, and other composition problems.
    pub composition_errors: Vec<String>,
}

/// Open the stage at `root`, composing every layer it pulls in through `loader`, and read the
/// UsdPhysics bodies and joints.
pub fn parse_usd(root: &str, loader: impl UsdLoader + 'static) -> Result<Usd, ParseError> {
    let resolver = LoaderResolver {
        loader: Box::new(loader),
        found: RefCell::default(),
    };
    let stage = Stage::builder()
        .resolver(resolver)
        .open(root)
        .map_err(|e| ParseError(e.to_string()))?;
    read(&stage).map_err(|e| ParseError(e.to_string()))
}

/// Lets the stage read layers through the caller's loader instead of the file system.
struct LoaderResolver {
    loader: Box<dyn UsdLoader>,
    /// Each layer is asked for once: `resolve` and `open_asset` both need it.
    found: RefCell<HashMap<String, Option<Rc<Vec<u8>>>>>,
}

impl LoaderResolver {
    fn fetch(&self, path: &str) -> Option<Rc<Vec<u8>>> {
        self.found
            .borrow_mut()
            .entry(path.to_string())
            .or_insert_with(|| self.loader.load(path).map(Rc::new))
            .clone()
    }
}

impl ar::Resolver for LoaderResolver {
    fn create_identifier(&self, asset_path: &str, anchor: Option<&ResolvedPath>) -> String {
        let base = anchor
            .and_then(|a| a.to_str())
            .and_then(|a| a.rsplit_once('/'));
        match base {
            Some((dir, _)) if !asset_path.starts_with('/') && !asset_path.contains("://") => {
                normalize(&format!("{dir}/{asset_path}"))
            }
            _ => normalize(asset_path),
        }
    }

    fn resolve(&self, asset_path: &str) -> Option<ResolvedPath> {
        self.fetch(asset_path)?;
        Some(ResolvedPath::new(asset_path))
    }

    fn resolve_for_new_asset(&self, asset_path: &str) -> Option<ResolvedPath> {
        Some(ResolvedPath::new(asset_path))
    }

    fn open_asset(&self, resolved_path: &ResolvedPath) -> io::Result<Box<dyn ar::Asset>> {
        let path = resolved_path.to_string_lossy();
        match self.fetch(&path) {
            Some(bytes) => Ok(Box::new(io::Cursor::new(bytes.as_ref().clone()))),
            None => Err(io::ErrorKind::NotFound.into()),
        }
    }

    /// The default reads the file system, which the core never touches.
    fn get_modification_timestamp(&self, _: &str, _: &ResolvedPath) -> Option<SystemTime> {
        None
    }

    fn identity(&self) -> String {
        "rigscan-loader".into()
    }
}

/// `a/./b/../c` to `a/c`, so two spellings of one layer share an identifier.
fn normalize(path: &str) -> String {
    let mut out: Vec<&str> = Vec::new();
    for part in path.split('/') {
        match part {
            "." => {}
            ".." if out.last().is_some_and(|l| !l.is_empty() && *l != "..") => {
                out.pop();
            }
            p => out.push(p),
        }
    }
    out.join("/")
}

fn read(stage: &Stage) -> openusd::Result<Usd> {
    let mut paths = Vec::new();
    stage.traverse(PrimPredicate::DEFAULT, |p| paths.push(p.clone()))?;
    let mut usd = Usd {
        bodies: Vec::new(),
        joints: Vec::new(),
        unmapped: Vec::new(),
        articulation_roots: Vec::new(),
        composition_errors: Vec::new(),
    };
    let mut targets = Vec::new();
    let mut colliders = Vec::new();
    let prims = paths
        .iter()
        .filter_map(|p| Some((p, stage.prim(p.as_str()).ok()?)));
    for (p, prim) in prims {
        let sinks = (&mut targets, &mut colliders);
        prim_data(stage, &prim, p.as_str(), &mut usd, sinks)?;
    }
    mark_colliders(&mut usd.bodies, &colliders);
    // A joint may connect a prim that is not a rigid body, such as a ground collider; it is still
    // a part of the structure as written. A target that does not exist is left out, so it reads
    // as an undefined link.
    let mut known: HashSet<String> = usd.bodies.iter().map(|b| b.path.clone()).collect();
    for t in targets {
        if known.insert(t.clone()) {
            usd.bodies.push(UsdBody {
                path: t,
                rigid: false,
                mass: None,
                inertia: None,
                center_of_mass: None,
                collision: false,
            });
        }
    }
    usd.composition_errors = stage
        .composition_errors()
        .iter()
        .map(ToString::to_string)
        .collect();
    Ok(usd)
}

/// A collision shape belongs to the nearest rigid body at or above it.
fn mark_colliders(bodies: &mut [UsdBody], colliders: &[String]) {
    let index: HashMap<String, usize> = bodies
        .iter()
        .enumerate()
        .map(|(i, b)| (b.path.clone(), i))
        .collect();
    for c in colliders {
        let mut p = c.as_str();
        loop {
            if let Some(&i) = index.get(p) {
                bodies[i].collision = true;
                break;
            }
            match p.rfind('/') {
                Some(at) if at > 0 => p = &p[..at],
                _ => break,
            }
        }
    }
}

/// `sinks` collects the prims a joint targets and the prims with a collision shape.
fn prim_data(
    stage: &Stage,
    prim: &Prim,
    path: &str,
    usd: &mut Usd,
    (targets, colliders): (&mut Vec<String>, &mut Vec<String>),
) -> openusd::Result<()> {
    let ty = prim.type_name()?.map(|t| t.to_string()).unwrap_or_default();
    let apis: Vec<String> = prim
        .api_schemas()?
        .iter()
        .map(ToString::to_string)
        .collect();
    let has = |a: &str| apis.iter().any(|x| x == a);
    let attr = |n: &str| prim.attribute(n).get::<Value>().ok().flatten();

    if has("PhysicsRigidBodyAPI") {
        let (mass, inertia, center_of_mass) = if has("PhysicsMassAPI") {
            (
                attr("physics:mass").and_then(number).filter(|m| *m > 0.0),
                // Zero means "compute from the shapes", which is not a known value.
                attr("physics:diagonalInertia")
                    .and_then(vec3)
                    .filter(|v| v.iter().any(|x| *x > 0.0)),
                // The default is (-inf, -inf, -inf), meaning "compute from the shapes".
                attr("physics:centerOfMass")
                    .and_then(vec3)
                    .filter(|v| v.iter().all(|x| x.is_finite())),
            )
        } else {
            (None, None, None)
        };
        usd.bodies.push(UsdBody {
            path: path.to_string(),
            rigid: true,
            mass,
            inertia,
            center_of_mass,
            collision: false,
        });
    }
    if ty.starts_with("Physics") && ty.ends_with("Joint") {
        let target = |name: &str| -> openusd::Result<Option<String>> {
            Ok(prim
                .relationship(name)
                .targets()?
                .first()
                .map(|t| t.as_str().to_string()))
        };
        let (body0, body1) = (target("physics:body0")?, target("physics:body1")?);
        for t in [&body0, &body1].into_iter().flatten() {
            let defined = stage
                .prim(t.as_str())
                .is_ok_and(|p| p.is_defined().unwrap_or(false));
            if defined {
                targets.push(t.clone());
            }
        }
        usd.joints.push(UsdJoint {
            path: path.to_string(),
            type_name: ty.clone(),
            body0,
            body1,
            local_pos0: attr("physics:localPos0").and_then(vec3).unwrap_or([0.0; 3]),
            axis: attr("physics:axis")
                .and_then(axis)
                .unwrap_or([1.0, 0.0, 0.0]),
            lower: attr("physics:lowerLimit")
                .and_then(number)
                .filter(|v| v.is_finite()),
            upper: attr("physics:upperLimit")
                .and_then(number)
                .filter(|v| v.is_finite()),
        });
        for rot in ["physics:localRot0", "physics:localRot1"] {
            if attr(rot).is_some_and(|q| !is_identity(&q)) {
                lose(usd, &rot["physics:".len()..], path);
            }
        }
    }
    if ty == "PhysicsScene" {
        lose(usd, "scene", path);
    }
    if has("PhysicsArticulationRootAPI") {
        usd.articulation_roots.push(path.to_string());
    }
    if has("PhysicsCollisionAPI") {
        colliders.push(path.to_string());
        lose(usd, "collision", path);
    }
    if apis.iter().any(|a| a.starts_with("PhysicsDriveAPI")) {
        lose(usd, "drive", path);
    }
    if has("PhysicsMaterialAPI") {
        lose(usd, "material", path);
    }
    if has("PhysicsMassAPI") && attr("physics:principalAxes").is_some_and(|q| !is_identity(&q)) {
        lose(usd, "principalAxes", path);
    }
    Ok(())
}

fn lose(usd: &mut Usd, what: &str, path: &str) {
    usd.unmapped.push(Unmapped {
        what: what.to_string(),
        name: Some(path.to_string()),
        span: Span::NONE,
    });
}

fn number(v: Value) -> Option<f64> {
    match v {
        Value::Float(x) => Some(f64::from(x)),
        Value::Double(x) => Some(x),
        _ => None,
    }
}

fn axis(v: Value) -> Option<[f64; 3]> {
    let Value::Token(t) = v else { return None };
    match t.as_str() {
        "X" => Some([1.0, 0.0, 0.0]),
        "Y" => Some([0.0, 1.0, 0.0]),
        "Z" => Some([0.0, 0.0, 1.0]),
        _ => None,
    }
}

fn vec3(v: Value) -> Option<[f64; 3]> {
    match v {
        Value::Vec3f(v) => Some([f64::from(v.x), f64::from(v.y), f64::from(v.z)]),
        Value::Vec3d(v) => Some([v.x, v.y, v.z]),
        _ => None,
    }
}

/// Anything but a quaternion is not a rotation we recognise, and counts as not identity.
fn is_identity(v: &Value) -> bool {
    let near = |a: f64, b: f64| (a - b).abs() < 1e-6;
    match v {
        Value::Quatf(q) => {
            near(q.w.into(), 1.0)
                && near(q.x.into(), 0.0)
                && near(q.y.into(), 0.0)
                && near(q.z.into(), 0.0)
        }
        Value::Quatd(q) => near(q.w, 1.0) && near(q.x, 0.0) && near(q.y, 0.0) && near(q.z, 0.0),
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openusd::ar::Resolver;

    struct Nothing;

    impl UsdLoader for Nothing {
        fn load(&self, _: &str) -> Option<Vec<u8>> {
            None
        }
    }

    fn resolver() -> LoaderResolver {
        LoaderResolver {
            loader: Box::new(Nothing),
            found: RefCell::default(),
        }
    }

    #[test]
    fn identifiers_are_anchored_to_the_referencing_layer_unless_absolute() {
        let r = resolver();
        let anchor = ResolvedPath::new("a/b/root.usda");
        assert_eq!(
            r.create_identifier("./c/../x.usda", Some(&anchor)),
            "a/b/x.usda"
        );
        assert_eq!(
            r.create_identifier("/abs/x.usda", Some(&anchor)),
            "/abs/x.usda"
        );
        assert_eq!(
            r.create_identifier("http://h/x.usda", Some(&anchor)),
            "http://h/x.usda"
        );
        assert_eq!(r.create_identifier("x.usda", None), "x.usda");
    }

    #[test]
    fn missing_layer_does_not_resolve_or_open() {
        let r = resolver();
        assert!(r.resolve("gone.usda").is_none());
        assert!(r.open_asset(&ResolvedPath::new("gone.usda")).is_err());
    }

    #[test]
    fn the_file_system_is_never_consulted() {
        let r = resolver();
        assert!(r.resolve_for_new_asset("x.usda").is_some());
        assert!(r
            .get_modification_timestamp("x.usda", &ResolvedPath::new("/etc/hosts"))
            .is_none());
    }
}
