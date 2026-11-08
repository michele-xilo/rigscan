//! Rigscan core: parse robot descriptions with source spans.

/// 1-based line and column in the source text.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Span {
    pub line: u32,
    pub col: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Link {
    pub name: String,
    pub span: Span,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Joint {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub span: Span,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Urdf {
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
}

#[derive(Debug)]
pub struct ParseError(pub String);

/// Parse a URDF document, keeping the source span of every link and joint.
pub fn parse_urdf(src: &str) -> Result<Urdf, ParseError> {
    use quick_xml::events::{BytesStart, Event};
    use quick_xml::Reader;

    fn attr(e: &BytesStart, key: &str) -> Result<Option<String>, ParseError> {
        match e.try_get_attribute(key) {
            Ok(Some(a)) => a
                .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                .map(|v| Some(v.into_owned()))
                .map_err(|err| ParseError(err.to_string())),
            Ok(None) => Ok(None),
            Err(err) => Err(ParseError(err.to_string())),
        }
    }

    fn span_at(src: &str, offset: usize) -> Span {
        let before = &src[..offset];
        let line = before.matches('\n').count() as u32 + 1;
        let col = before.rsplit('\n').next().map_or(0, |l| l.chars().count()) as u32 + 1;
        Span { line, col }
    }

    let mut reader = Reader::from_str(src);
    let mut urdf = Urdf::default();
    let mut joint: Option<Joint> = None;

    loop {
        // Text between tags is its own event, so this is the offset of the next '<'.
        let start = reader.buffer_position() as usize;
        let event = reader.read_event().map_err(|e| ParseError(e.to_string()))?;
        match event {
            Event::Start(e) | Event::Empty(e) => match e.name().as_ref() {
                "link" => urdf.links.push(Link {
                    name: attr(&e, "name")?.unwrap_or_default(),
                    span: span_at(src, start),
                }),
                "joint" => {
                    joint = Some(Joint {
                        name: attr(&e, "name")?.unwrap_or_default(),
                        parent: String::new(),
                        child: String::new(),
                        span: span_at(src, start),
                    })
                }
                "parent" => {
                    if let Some(j) = joint.as_mut() {
                        j.parent = attr(&e, "link")?.unwrap_or_default();
                    }
                }
                "child" => {
                    if let Some(j) = joint.as_mut() {
                        j.child = attr(&e, "link")?.unwrap_or_default();
                    }
                }
                _ => {}
            },
            Event::End(e) if e.name().as_ref() == "joint" => {
                urdf.joints.extend(joint.take());
            }
            Event::Eof => break,
            _ => {}
        }
    }
    Ok(urdf)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    Warning,
    Critical,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Finding {
    pub rule: &'static str,
    pub severity: Severity,
    pub message: String,
    pub span: Span,
}

/// Run all lint rules over a parsed robot.
pub fn lint(robot: &Urdf) -> Vec<Finding> {
    let mut findings = Vec::new();
    // RS001: joint references a link that is not defined.
    for j in &robot.joints {
        for (role, name) in [("parent", &j.parent), ("child", &j.child)] {
            if !robot.links.iter().any(|l| &l.name == name) {
                findings.push(Finding {
                    rule: "RS001",
                    severity: Severity::Critical,
                    message: format!("joint `{}` {role} link `{name}` is not defined", j.name),
                    span: j.span,
                });
            }
        }
    }
    duplicate_names(robot, &mut findings);
    tree_shape(robot, &mut findings);
    findings
}

fn critical(rule: &'static str, message: String, span: Span) -> Finding {
    Finding { rule, severity: Severity::Critical, message, span }
}

/// RS002 / RS003: names must be unique within links and within joints.
fn duplicate_names(robot: &Urdf, out: &mut Vec<Finding>) {
    use std::collections::HashSet;
    let mut seen = HashSet::new();
    for l in &robot.links {
        if !seen.insert(l.name.as_str()) {
            out.push(critical("RS002", format!("link `{}` is defined more than once", l.name), l.span));
        }
    }
    seen.clear();
    for j in &robot.joints {
        if !seen.insert(j.name.as_str()) {
            out.push(critical("RS003", format!("joint `{}` is defined more than once", j.name), j.span));
        }
    }
}

/// RS004 (two parents), RS005 (loop), RS006 (several roots).
fn tree_shape(robot: &Urdf, out: &mut Vec<Finding>) {
    use std::collections::HashMap;
    // child link -> first joint that parents it
    let mut parent_joint: HashMap<&str, &Joint> = HashMap::new();
    for j in &robot.joints {
        if parent_joint.contains_key(j.child.as_str()) {
            out.push(critical("RS004", format!("link `{}` has more than one parent joint", j.child), j.span));
        } else {
            parent_joint.insert(&j.child, j);
        }
    }

    // Walk up from each link; revisiting a link on the current path is a loop.
    let mut state: HashMap<&str, u8> = HashMap::new(); // 1 = on path, 2 = done
    for l in &robot.links {
        let mut path = Vec::new();
        let mut cur = l.name.as_str();
        loop {
            match state.get(cur) {
                Some(2) => break,
                Some(1) => {
                    let j = parent_joint[cur];
                    out.push(critical("RS005", format!("kinematic loop through link `{cur}`"), j.span));
                    break;
                }
                _ => {}
            }
            state.insert(cur, 1);
            path.push(cur);
            match parent_joint.get(cur) {
                Some(j) => cur = j.parent.as_str(),
                None => break,
            }
        }
        for n in path {
            state.insert(n, 2);
        }
    }

    // Every link that is nobody's child is a root; a tree has exactly one.
    let roots: Vec<&Link> = robot.links.iter().filter(|l| !parent_joint.contains_key(l.name.as_str())).collect();
    for extra in roots.iter().skip(1) {
        out.push(critical("RS006", format!("link `{}` is an extra root; a robot must have one root link", extra.name), extra.span));
    }
}
