//! Rigscan in the browser: scan a set of dropped files and describe the result as JSON.
//!
//! The page cannot read the disk, so every file a robot refers to has to be in the dropped set.

use rigscan_core::{
    lint_mjcf, lint_usd, lint_with, parse_mjcf, parse_urdf, parse_usd, report, Element, Finding,
    Format, IncludeLoader, Included, JointKind, ParseError, Resolution, Resolver, Robot, Severity,
    Span, Status, UsdLoader,
};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::rc::Rc;

type Files = Rc<BTreeMap<String, Vec<u8>>>;

/// Scan `main` inside a set of named files and describe the result.
///
/// Always returns an object: the result, or `{ "path", "error" }` when the file cannot be scanned.
pub fn scan_bundle(main: &str, files: BTreeMap<String, Vec<u8>>) -> Value {
    let main = normalize(main);
    match scan(&main, Rc::new(files)) {
        Ok(v) => v,
        Err(error) => json!({ "path": main, "error": error }),
    }
}

fn scan(main: &str, files: Files) -> Result<Value, String> {
    let bytes = files
        .get(main)
        .ok_or_else(|| format!("{main}: file was not provided"))?;
    let format =
        detect_format(main, bytes).ok_or_else(|| format!("{main}: unsupported file type"))?;
    let at = |e: ParseError| format!("{main}: {}", e.0);
    let source = match format {
        Format::Usd => String::new(),
        _ => String::from_utf8_lossy(bytes).into_owned(),
    };
    let (robot, findings) = match format {
        Format::Urdf => {
            let robot = parse_urdf(&source).map_err(at)?;
            let findings = lint_with(&robot, &Unchecked);
            (robot, findings)
        }
        Format::Mjcf => {
            let loader = BundleLoader {
                files,
                main: main.to_string(),
            };
            let mjcf = parse_mjcf(&source, &loader).map_err(at)?;
            let robot = mjcf.to_robot().map_err(at)?;
            let findings = lint_mjcf(&mjcf, &robot, &Unchecked);
            (robot, findings)
        }
        Format::Usd => {
            let usd = parse_usd(main, BundleUsdLoader(files)).map_err(at)?;
            let robot = usd.to_robot().map_err(at)?;
            let findings = lint_usd(&usd, &robot);
            (robot, findings)
        }
    };
    Ok(describe(main, format, &source, &robot, &findings))
}

fn describe(
    path: &str,
    format: Format,
    source: &str,
    robot: &Robot,
    findings: &[Finding],
) -> Value {
    let report = report(robot, findings);
    let mut links = vec![Status::Healthy; robot.links.len()];
    let mut joints = vec![Status::Healthy; robot.joints.len()];
    for e in &report.elements {
        match e.element {
            Element::Link(i) => links[i] = e.status,
            Element::Joint(i) => joints[i] = e.status,
            Element::Robot => {}
        }
    }
    json!({
        "version": 1,
        "path": path,
        "format": match format { Format::Urdf => "urdf", Format::Mjcf => "mjcf", Format::Usd => "usd" },
        "robot": robot.name,
        "verdict": {
            "healthy": report.verdict.healthy,
            "warnings": report.verdict.warnings,
            "critical": report.verdict.critical,
        },
        "links": robot.links.iter().zip(&links).map(|(l, s)| {
            let (line, column) = position(l.span);
            json!({ "name": l.name, "status": status(*s), "line": line, "column": column })
        }).collect::<Vec<_>>(),
        "joints": robot.joints.iter().zip(&joints).map(|(j, s)| {
            let (line, column) = position(j.span);
            json!({
                "name": j.name,
                "kind": kind(&j.kind),
                "parent": j.parent,
                "child": j.child,
                "origin": { "xyz": j.origin.xyz, "rpy": j.origin.rpy },
                "axis": j.axis,
                "status": status(*s),
                "line": line,
                "column": column,
            })
        }).collect::<Vec<_>>(),
        "findings": findings.iter().map(|f| {
            let (line, column) = position(f.span);
            json!({
                "rule": f.rule,
                "severity": match f.severity { Severity::Warning => "warning", Severity::Critical => "critical" },
                "message": f.message,
                "line": line,
                "column": column,
                "element": element(robot, f.element),
            })
        }).collect::<Vec<_>>(),
        "unmapped": robot.unmapped.iter().map(|u| {
            let (line, column) = position(u.span);
            json!({ "what": u.what, "name": u.name, "line": line, "column": column })
        }).collect::<Vec<_>>(),
        "source": source,
    })
}

fn element(robot: &Robot, e: Element) -> Value {
    match e {
        Element::Robot => json!({ "kind": "robot", "name": robot.name }),
        Element::Link(i) => json!({ "kind": "link", "name": robot.links[i].name }),
        Element::Joint(i) => json!({ "kind": "joint", "name": robot.joints[i].name }),
    }
}

fn status(s: Status) -> &'static str {
    match s {
        Status::Healthy => "healthy",
        Status::Warning => "warning",
        Status::Critical => "critical",
    }
}

fn kind(k: &JointKind) -> String {
    match k {
        JointKind::Revolute => "revolute".into(),
        JointKind::Continuous => "continuous".into(),
        JointKind::Prismatic => "prismatic".into(),
        JointKind::Fixed => "fixed".into(),
        JointKind::Floating => "floating".into(),
        JointKind::Planar => "planar".into(),
        JointKind::Other(name) => name.clone(),
    }
}

/// USD has no source positions; its spans are `Span::NONE` (line 0).
fn position(span: Span) -> (Value, Value) {
    if span.line == 0 {
        (Value::Null, Value::Null)
    } else {
        (json!(span.line), json!(span.col))
    }
}

fn detect_format(path: &str, bytes: &[u8]) -> Option<Format> {
    let ext = path.rsplit_once('.')?.1.to_ascii_lowercase();
    match ext.as_str() {
        "urdf" => Some(Format::Urdf),
        "usda" | "usdc" | "usd" => Some(Format::Usd),
        // `.xml` is also ROS launch files and package manifests.
        "xml" if bytes.windows(7).take(16 * 1024).any(|w| w == b"<mujoco") => Some(Format::Mjcf),
        _ => None,
    }
}

/// A `/`-separated path with `.` and `..` resolved, so the same file has one name however it is reached.
fn normalize(path: &str) -> String {
    let mut parts: Vec<&str> = Vec::new();
    for part in path.split(['/', '\\']) {
        match part {
            "" | "." => {}
            ".." => {
                parts.pop();
            }
            p => parts.push(p),
        }
    }
    parts.join("/")
}

fn dir_of(path: &str) -> &str {
    path.rsplit_once('/').map_or("", |(dir, _)| dir)
}

struct BundleLoader {
    files: Files,
    main: String,
}

impl IncludeLoader for BundleLoader {
    /// MuJoCo reads an include relative to the file that contains it.
    fn load(&self, file: &str, from: Option<&str>) -> Option<Included> {
        let base = from.map_or_else(|| dir_of(&self.main), dir_of);
        let path = normalize(&format!("{base}/{file}"));
        let bytes = self.files.get(&path)?;
        Some(Included {
            path,
            text: String::from_utf8_lossy(bytes).into_owned(),
        })
    }
}

struct BundleUsdLoader(Files);

impl UsdLoader for BundleUsdLoader {
    fn load(&self, path: &str) -> Option<Vec<u8>> {
        self.0.get(&normalize(path)).cloned()
    }
}

/// Mesh files are never reported missing: only the dropped files are visible, so absence proves nothing.
struct Unchecked;

impl Resolver for Unchecked {
    fn resolve(&self, _uri: &str) -> Resolution {
        Resolution::Unknown
    }
}

/// Scan from JavaScript. `paths` and `files` are parallel arrays of file names and `Uint8Array`
/// contents; `main` names the file to scan. Returns the result as a JSON string.
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen::prelude::wasm_bindgen]
pub fn scan_files(main: &str, paths: &js_sys::Array, files: &js_sys::Array) -> String {
    let bundle = paths
        .iter()
        .zip(files.iter())
        .filter_map(|(p, f)| {
            let path = normalize(&p.as_string()?);
            Some((path, js_sys::Uint8Array::new(&f).to_vec()))
        })
        .collect();
    scan_bundle(main, bundle).to_string()
}
