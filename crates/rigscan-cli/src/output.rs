//! Rendering of scan results: findings only, per-element report, or JSON.

use rigscan_core::{
    loss_info, report, Element, ElementStatus, Finding, Format, LossKind, Report, Robot, Severity,
    Span, Status, Unmapped, Verdict,
};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

pub struct Scanned {
    pub src: String,
    pub robot: Robot,
    pub format: Format,
    pub findings: Vec<Finding>,
    /// Canonical paths of the files this one includes.
    pub included: Vec<PathBuf>,
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
        Format::Mjcf => "mjcf",
        Format::Usd => "usd",
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

const RED: &str = "31";
const AMBER: &str = "33";
const TEAL: &str = "36";

pub fn paint(color: bool, code: &str, text: &str) -> String {
    if color {
        format!("\x1b[{code}m{text}\x1b[0m")
    } else {
        text.to_string()
    }
}

fn severity_code(s: Severity) -> &'static str {
    match s {
        Severity::Warning => AMBER,
        Severity::Critical => RED,
    }
}

fn status_code(s: Status) -> &'static str {
    match s {
        Status::Healthy => TEAL,
        Status::Warning => AMBER,
        Status::Critical => RED,
    }
}

pub fn healthy(color: bool, text: &str) -> String {
    paint(color, TEAL, text)
}

pub fn problem(color: bool, text: &str) -> String {
    paint(color, RED, text)
}

/// `path:line:col`, or just the path when the format has no source positions.
fn location(path: &Path, span: Span) -> String {
    if span.is_none() {
        path.display().to_string()
    } else {
        format!("{}:{}:{}", path.display(), span.line, span.col)
    }
}

/// JSON `null` for a position that is not known.
fn position(span: Span) -> (Value, Value) {
    if span.is_none() {
        (Value::Null, Value::Null)
    } else {
        (json!(span.line), json!(span.col))
    }
}

pub fn format_finding(path: &Path, f: &Finding, color: bool) -> String {
    let word = severity(f.severity);
    format!(
        "{}: {} {}: {}",
        location(path, f.span),
        paint(color, severity_code(f.severity), word),
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

pub fn write_text(out: &mut impl Write, path: &Path, s: &Scanned, color: bool) -> io::Result<()> {
    for f in &s.findings {
        writeln!(out, "{}", format_finding(path, f, color))?;
    }
    Ok(())
}

fn line_starts(src: &str) -> Vec<usize> {
    std::iter::once(0)
        .chain(src.match_indices('\n').map(|(i, _)| i + 1))
        .collect()
}

fn line_text<'a>(src: &'a str, starts: &[usize], line: u32) -> &'a str {
    let Some(&from) = starts.get(line as usize - 1) else {
        return "";
    };
    let to = starts.get(line as usize).copied().unwrap_or(src.len());
    src[from..to].trim_end_matches(['\n', '\r'])
}

/// Source line under each finding with a caret at its column. Tabs are copied into the padding so
/// the caret lines up whatever the tab width, and columns count characters, not bytes.
pub fn write_full(out: &mut impl Write, path: &Path, s: &Scanned, color: bool) -> io::Result<()> {
    if s.findings.is_empty() {
        return Ok(());
    }
    let starts = line_starts(&s.src);
    for f in &s.findings {
        if f.span.is_none() {
            let head = format!("{}[{}]", severity(f.severity), f.rule);
            let head = paint(color, severity_code(f.severity), &head);
            writeln!(out, "{head}: {}", f.message)?;
            writeln!(out, "--> {}", path.display())?;
            writeln!(out)?;
            continue;
        }
        let text = line_text(&s.src, &starts, f.span.line);
        let pad = " ".repeat(f.span.line.to_string().len());
        let code = severity_code(f.severity);
        let head = format!("{}[{}]", severity(f.severity), f.rule);
        let lead: String = text
            .chars()
            .take(f.span.col as usize - 1)
            .map(|c| if c == '\t' { '\t' } else { ' ' })
            .collect();
        writeln!(out, "{}: {}", paint(color, code, &head), f.message)?;
        writeln!(
            out,
            "{pad}--> {}:{}:{}",
            path.display(),
            f.span.line,
            f.span.col
        )?;
        writeln!(out, "{pad} |")?;
        writeln!(out, "{} | {text}", f.span.line)?;
        writeln!(out, "{pad} | {lead}{}", paint(color, code, "^"))?;
        writeln!(out)?;
    }
    Ok(())
}

/// Findings per rule across all files, most frequent first.
pub fn write_statistics(out: &mut impl Write, scanned: &[&Scanned], color: bool) -> io::Result<()> {
    // A rule has one severity, so it is kept with the count rather than in the key.
    let mut counts: HashMap<&str, (usize, Severity)> = HashMap::new();
    for f in scanned.iter().flat_map(|s| &s.findings) {
        counts.entry(f.rule).or_insert((0, f.severity)).0 += 1;
    }
    let mut rows: Vec<_> = counts.into_iter().collect();
    rows.sort_by(|a, b| b.1 .0.cmp(&a.1 .0).then(a.0.cmp(b.0)));
    for (rule, (n, sev)) in rows {
        let word = paint(color, severity_code(sev), severity(sev));
        writeln!(out, "{n:>5}  {rule}  {word}")?;
    }
    Ok(())
}

pub fn write_report(out: &mut impl Write, path: &Path, s: &Scanned, color: bool) -> io::Result<()> {
    let r = report(&s.robot, &s.findings);
    writeln!(out, "{}: {}", path.display(), verdict_line(&r.verdict))?;
    let mut by_element: HashMap<Element, Vec<&Finding>> = HashMap::new();
    for f in &s.findings {
        by_element.entry(f.element).or_default().push(f);
    }
    for e in &r.elements {
        let word = status(e.status);
        writeln!(
            out,
            "  {}{} {:<5} {}",
            paint(color, status_code(e.status), word),
            " ".repeat(8 - word.len()),
            kind(e.element),
            e.name
        )?;
        for f in by_element.get(&e.element).into_iter().flatten() {
            writeln!(out, "    {}", format_finding(path, f, color))?;
        }
    }
    Ok(())
}

fn element_json(e: &ElementStatus) -> Value {
    let (line, column) = position(e.span);
    json!({
        "kind": kind(e.element),
        "name": e.name,
        "status": status(e.status),
        "line": line,
        "column": column,
    })
}

fn finding_json(robot: &Robot, f: &Finding) -> Value {
    let (line, column) = position(f.span);
    json!({
        "rule": f.rule,
        "severity": severity(f.severity),
        "message": f.message,
        "line": line,
        "column": column,
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

/// Kind and meaning of a loss id; an id from a reader is an element or attribute the neutral
/// model has no field for.
fn describe(id: &str) -> (&'static str, &'static str) {
    match loss_info(id) {
        Some(i) => (
            match i.kind {
                LossKind::Dropped => "dropped",
                LossKind::Changed => "changed",
                LossKind::Invented => "invented",
            },
            i.summary,
        ),
        None => (
            "dropped",
            "The neutral model has no field for it; it was not carried over.",
        ),
    }
}

/// The losses grouped by id in the order first met, each with the names it hit.
fn group_losses(lost: &[Unmapped]) -> Vec<(&str, Vec<&Unmapped>)> {
    let mut groups: Vec<(&str, Vec<&Unmapped>)> = Vec::new();
    for u in lost {
        match groups.iter_mut().find(|(id, _)| *id == u.what) {
            Some((_, items)) => items.push(u),
            None => groups.push((&u.what, vec![u])),
        }
    }
    groups
}

/// One line per kind of loss with the names it hit, its meaning under it, and a total.
pub fn loss_text(lost: &[Unmapped]) -> String {
    let groups = group_losses(lost);
    let mut out = String::new();
    for (id, items) in &groups {
        let names: Vec<String> = items
            .iter()
            .map(|u| {
                let name = u.name.as_deref().unwrap_or("-");
                if u.span.is_none() {
                    name.to_string()
                } else {
                    format!("{name} ({}:{})", u.span.line, u.span.col)
                }
            })
            .collect();
        out.push_str(&format!(
            "lost {id}: {}\n  {}\n",
            names.join(", "),
            describe(id).1
        ));
    }
    if !groups.is_empty() {
        out.push_str(&format!(
            "{}, {}\n",
            plural(groups.len(), "kind of loss", "kinds of loss"),
            plural(lost.len(), "element", "elements")
        ));
    }
    out
}

/// The loss report document, version 1.
pub fn loss_json(input: &Path, from: Format, to: Format, lost: &[Unmapped]) -> Value {
    let entries: Vec<Value> = lost
        .iter()
        .map(|u| {
            let (kind, summary) = describe(&u.what);
            let (line, column) = position(u.span);
            json!({
                "id": u.what,
                "kind": kind,
                "summary": summary,
                "name": u.name,
                "line": line,
                "column": column,
            })
        })
        .collect();
    let counts: serde_json::Map<String, Value> = group_losses(lost)
        .into_iter()
        .map(|(id, items)| (id.to_string(), json!(items.len())))
        .collect();
    json!({
        "version": 1,
        "input": input.display().to_string(),
        "from": format_name(from),
        "to": format_name(to),
        "lost": entries,
        "counts": counts,
    })
}
