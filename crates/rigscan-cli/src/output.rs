//! Rendering of scan results: findings only, per-element report, or JSON.

use rigscan_core::{
    report, Element, ElementStatus, Finding, Format, Report, Robot, Severity, Status, Verdict,
};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::io::{self, Write};
use std::path::Path;

pub struct Scanned {
    pub robot: Robot,
    pub format: Format,
    pub findings: Vec<Finding>,
}

fn severity(s: Severity) -> &'static str {
    match s {
        Severity::Warning => "warning",
        Severity::Critical => "critical",
    }
}

fn status(s: Status) -> &'static str {
    match s {
        Status::Healthy => "healthy",
        Status::Warning => "warning",
        Status::Critical => "critical",
    }
}

fn format_name(f: Format) -> &'static str {
    match f {
        Format::Urdf => "urdf",
    }
}

fn kind(e: Element) -> &'static str {
    match e {
        Element::Robot => "robot",
        Element::Link(_) => "link",
        Element::Joint(_) => "joint",
    }
}

fn element_name(robot: &Robot, e: Element) -> &str {
    match e {
        Element::Robot => robot.name.as_deref().unwrap_or(""),
        Element::Link(i) => &robot.links[i].name,
        Element::Joint(i) => &robot.joints[i].name,
    }
}

pub fn format_finding(path: &Path, f: &Finding) -> String {
    format!(
        "{}:{}:{}: {} {}: {}",
        path.display(),
        f.span.line,
        f.span.col,
        severity(f.severity),
        f.rule,
        f.message
    )
}

fn plural(n: usize, one: &str, many: &str) -> String {
    format!("{n} {}", if n == 1 { one } else { many })
}

fn verdict_line(v: &Verdict) -> String {
    format!(
        "{} healthy, {}, {} critical",
        v.healthy,
        plural(v.warnings, "warning", "warnings"),
        v.critical
    )
}

pub fn write_text(out: &mut impl Write, path: &Path, s: &Scanned) -> io::Result<()> {
    for f in &s.findings {
        writeln!(out, "{}", format_finding(path, f))?;
    }
    Ok(())
}

pub fn write_report(out: &mut impl Write, path: &Path, s: &Scanned) -> io::Result<()> {
    let r = report(&s.robot, &s.findings);
    writeln!(out, "{}: {}", path.display(), verdict_line(&r.verdict))?;
    let mut by_element: HashMap<Element, Vec<&Finding>> = HashMap::new();
    for f in &s.findings {
        by_element.entry(f.element).or_default().push(f);
    }
    for e in &r.elements {
        writeln!(
            out,
            "  {:<8} {:<5} {}",
            status(e.status),
            kind(e.element),
            e.name
        )?;
        for f in by_element.get(&e.element).into_iter().flatten() {
            writeln!(out, "    {}", format_finding(path, f))?;
        }
    }
    Ok(())
}

fn element_json(e: &ElementStatus) -> Value {
    json!({
        "kind": kind(e.element),
        "name": e.name,
        "status": status(e.status),
        "line": e.span.line,
        "column": e.span.col,
    })
}

fn finding_json(robot: &Robot, f: &Finding) -> Value {
    json!({
        "rule": f.rule,
        "severity": severity(f.severity),
        "message": f.message,
        "line": f.span.line,
        "column": f.span.col,
        "element": { "kind": kind(f.element), "name": element_name(robot, f.element) },
    })
}

fn file_json(path: &Path, s: &Scanned, r: &Report) -> Value {
    json!({
        "path": path.display().to_string(),
        "format": format_name(s.format),
        "robot": s.robot.name,
        "verdict": {
            "healthy": r.verdict.healthy,
            "warnings": r.verdict.warnings,
            "critical": r.verdict.critical,
        },
        "elements": r.elements.iter().map(element_json).collect::<Vec<_>>(),
        "findings": s.findings.iter().map(|f| finding_json(&s.robot, f)).collect::<Vec<_>>(),
    })
}

/// Bump `version` for any change that is not purely additive.
pub fn json_document(results: &[(&Path, &Result<Scanned, String>)]) -> Value {
    let mut total = Verdict::default();
    let mut errors = 0;
    let mut files = Vec::with_capacity(results.len());
    for (path, result) in results {
        match result {
            Ok(s) => {
                let r = report(&s.robot, &s.findings);
                total.healthy += r.verdict.healthy;
                total.warnings += r.verdict.warnings;
                total.critical += r.verdict.critical;
                files.push(file_json(path, s, &r));
            }
            Err(msg) => {
                errors += 1;
                files.push(json!({ "path": path.display().to_string(), "error": msg }));
            }
        }
    }
    json!({
        "version": 1,
        "files": files,
        "summary": {
            "healthy": total.healthy,
            "warnings": total.warnings,
            "critical": total.critical,
            "errors": errors,
        },
    })
}
