//! MJCF parser: an element tree with spans, `include` spliced in and `default` classes applied.

use crate::model::{ParseError, Span};
use crate::urdf::Spans;
use quick_xml::events::{BytesStart, Event};
use quick_xml::Reader;
use std::collections::HashMap;

/// One element of the resolved tree.
#[derive(Debug, Clone, PartialEq)]
pub struct MjcfNode {
    pub tag: String,
    pub attrs: Vec<(String, String)>,
    pub children: Vec<MjcfNode>,
    pub span: Span,
    /// The included file this element was read from; `None` for the main file.
    pub file: Option<String>,
}

impl MjcfNode {
    pub fn attr(&self, key: &str) -> Option<&str> {
        self.attrs
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.as_str())
    }
}

/// A name used in the file that could not be resolved.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reference {
    pub name: String,
    pub span: Span,
    pub file: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Mjcf {
    pub root: MjcfNode,
    pub unresolved_includes: Vec<Reference>,
    pub unknown_classes: Vec<Reference>,
}

/// Supplies the text of an included file. The core does no file access, so the caller provides this.
pub trait IncludeLoader {
    /// `file` is the path as written in the `include` element.
    fn load(&self, file: &str) -> Option<String>;
}

/// Include nesting beyond this is treated as a cycle.
const MAX_INCLUDE_DEPTH: usize = 16;

/// The tree passes recurse once per level, so unbounded nesting would overflow the stack.
/// Counted across includes. Sized for a 2 MiB thread stack, the default for the CLI's worker threads;
/// real robots nest tens of bodies deep.
const MAX_NESTING: usize = 256;

/// Parse an MJCF document, splicing in `include`d files and applying `default` classes.
pub fn parse_mjcf(src: &str, loader: &dyn IncludeLoader) -> Result<Mjcf, ParseError> {
    let mut root = read_tree(src, None, 0)?;
    if root.tag != "mujoco" {
        return Err(ParseError("no <mujoco> element found".into()));
    }
    let mut unresolved_includes = Vec::new();
    expand_includes(&mut root, loader, 0, 1, &mut unresolved_includes)?;
    let classes = collect_classes(&root);
    let mut unknown_classes = Vec::new();
    apply_defaults(&mut root, "main", &classes, &mut unknown_classes);
    Ok(Mjcf {
        root,
        unresolved_includes,
        unknown_classes,
    })
}

/// `base` is the nesting depth of the position the text is read into.
fn read_tree(src: &str, file: Option<&str>, base: usize) -> Result<MjcfNode, ParseError> {
    let mut reader = Reader::from_str(src);
    let mut spans = Spans::new(src);
    let mut stack: Vec<MjcfNode> = Vec::new();
    let mut root: Option<MjcfNode> = None;
    loop {
        // Text between tags is its own event, so this is the offset of the next '<'.
        let start = reader.buffer_position() as usize;
        match reader.read_event().map_err(|e| ParseError(e.to_string()))? {
            Event::Start(e) => {
                if base + stack.len() >= MAX_NESTING {
                    let at = spans.at(start);
                    return Err(ParseError(format!(
                        "{}:{}: elements nested more than {MAX_NESTING} levels deep",
                        at.line, at.col
                    )));
                }
                stack.push(node(&e, spans.at(start), file)?)
            }
            Event::Empty(e) => attach(&mut stack, &mut root, node(&e, spans.at(start), file)?)?,
            Event::End(_) => {
                let n = stack
                    .pop()
                    .ok_or_else(|| ParseError("unexpected closing tag".into()))?;
                attach(&mut stack, &mut root, n)?;
            }
            Event::Eof => break,
            _ => {}
        }
    }
    if !stack.is_empty() {
        return Err(ParseError("unclosed element at end of input".into()));
    }
    root.ok_or_else(|| ParseError("no root element found".into()))
}

fn attach(
    stack: &mut [MjcfNode],
    root: &mut Option<MjcfNode>,
    n: MjcfNode,
) -> Result<(), ParseError> {
    match stack.last_mut() {
        Some(parent) => parent.children.push(n),
        None if root.is_none() => *root = Some(n),
        None => return Err(ParseError("more than one root element".into())),
    }
    Ok(())
}

fn node(e: &BytesStart, span: Span, file: Option<&str>) -> Result<MjcfNode, ParseError> {
    let mut attrs = Vec::new();
    for a in e.attributes() {
        let a = a.map_err(|err| ParseError(err.to_string()))?;
        let value = a
            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
            .map_err(|err| ParseError(err.to_string()))?;
        attrs.push((a.key.as_ref().to_string(), value.into_owned()));
    }
    Ok(MjcfNode {
        tag: e.name().as_ref().to_string(),
        attrs,
        children: Vec::new(),
        span,
        file: file.map(str::to_string),
    })
}

fn expand_includes(
    node: &mut MjcfNode,
    loader: &dyn IncludeLoader,
    depth: usize,
    level: usize,
    unresolved: &mut Vec<Reference>,
) -> Result<(), ParseError> {
    let mut children = Vec::with_capacity(node.children.len());
    for mut child in std::mem::take(&mut node.children) {
        if child.tag != "include" {
            expand_includes(&mut child, loader, depth, level + 1, unresolved)?;
            children.push(child);
            continue;
        }
        let name = child.attr("file").ok_or_else(|| {
            ParseError(format!(
                "{}:{}: <include> needs a `file` attribute",
                child.span.line, child.span.col
            ))
        })?;
        let Some(text) = loader.load(name) else {
            unresolved.push(Reference {
                name: name.to_string(),
                span: child.span,
                file: child.file.clone(),
            });
            continue;
        };
        if depth >= MAX_INCLUDE_DEPTH {
            return Err(ParseError(format!(
                "include of `{name}` nests more than {MAX_INCLUDE_DEPTH} levels deep (cycle?)"
            )));
        }
        let mut included = read_tree(&text, Some(name), level)
            .map_err(|ParseError(m)| ParseError(format!("{name}: {m}")))?;
        if included.tag != "mujoco" {
            return Err(ParseError(format!("{name}: no <mujoco> element found")));
        }
        expand_includes(&mut included, loader, depth + 1, level + 1, unresolved)?;
        children.extend(included.children);
    }
    node.children = children;
    Ok(())
}

/// One `<default>` class: its parent and the attributes it sets per tag.
struct Class {
    parent: Option<String>,
    tags: HashMap<String, Vec<(String, String)>>,
}

type Classes = HashMap<String, Class>;

fn collect_classes(root: &MjcfNode) -> Classes {
    let mut classes = Classes::new();
    for d in root.children.iter().filter(|c| c.tag == "default") {
        collect_class(d, "main", None, &mut classes);
    }
    classes
}

fn collect_class(d: &MjcfNode, name: &str, parent: Option<&str>, classes: &mut Classes) {
    classes.entry(name.to_string()).or_insert_with(|| Class {
        parent: parent.map(str::to_string),
        tags: HashMap::new(),
    });
    for child in &d.children {
        if child.tag == "default" {
            if let Some(sub) = child.attr("class") {
                collect_class(child, sub, Some(name), classes);
            }
        } else {
            let tags = &mut classes.get_mut(name).expect("inserted above").tags;
            tags.insert(child.tag.clone(), child.attrs.clone());
        }
    }
}

/// Fills attributes an element does not set from its class, then that class's parents.
fn apply_defaults(
    node: &mut MjcfNode,
    inherited: &str,
    classes: &Classes,
    unknown: &mut Vec<Reference>,
) {
    for child in &mut node.children {
        if child.tag == "default" {
            continue;
        }
        for key in ["class", "childclass"] {
            if let Some(name) = child.attr(key) {
                if !classes.contains_key(name) {
                    unknown.push(Reference {
                        name: name.to_string(),
                        span: child.span,
                        file: child.file.clone(),
                    });
                }
            }
        }
        let class = child.attr("class").unwrap_or(inherited).to_string();
        let next = child.attr("childclass").unwrap_or(inherited).to_string();
        let mut current = Some(class.as_str());
        while let Some(c) = current.and_then(|c| classes.get(c)) {
            if let Some(attrs) = c.tags.get(&child.tag) {
                for (k, v) in attrs {
                    if child.attr(k).is_none() {
                        child.attrs.push((k.clone(), v.clone()));
                    }
                }
            }
            current = c.parent.as_deref();
        }
        apply_defaults(child, &next, classes, unknown);
    }
}
