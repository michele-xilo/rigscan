//! URDF parser.

use crate::model::{
    Dynamics, Format, Inertia, Inertial, Joint, JointKind, Limit, Link, MeshRef, Mimic, Origin,
    ParseError, Robot, Span, Unmapped,
};
use quick_xml::events::BytesStart;
use std::borrow::Cow;

/// Parse a URDF document, keeping the source span of every link and joint.
pub fn parse_urdf(src: &str) -> Result<Robot, ParseError> {
    use quick_xml::events::Event;
    use quick_xml::Reader;

    let mut reader = Reader::from_str(src);
    let mut spans = Spans::new(src);
    let mut robot = Robot::new(Format::Urdf);
    let mut joint: Option<Joint> = None;

    // Elements are matched by their parent, so a <joint> inside <transmission>
    // or a <link> inside <gazebo>, which only reference the real ones, is ignored.
    let mut stack: Vec<Ctx> = Vec::new();
    let mut seen_robot = false;

    loop {
        // Text between tags is its own event, so this is the offset of the next '<'.
        let start = reader.buffer_position() as usize;
        let event = reader.read_event().map_err(|e| ParseError(e.to_string()))?;
        let (e, opens) = match event {
            Event::Start(e) => (e, true),
            Event::Empty(e) => (e, false),
            Event::End(_) => {
                if stack.pop() == Some(Ctx::Joint) {
                    robot.joints.extend(joint.take().map(with_default_axis));
                }
                continue;
            }
            Event::Eof => break,
            _ => continue,
        };
        let qname = e.name();
        let name = qname.as_ref();
        let parent = stack.last().copied();
        match (parent, name) {
            (None, "robot") => {
                seen_robot = true;
                robot.span = spans.at(start);
                robot.name = attr(&e, "name")?
            }
            (Some(Ctx::Robot), "link") => robot.links.push(Link {
                name: attr(&e, "name")?.unwrap_or_default(),
                inertial: None,
                meshes: Vec::new(),
                span: spans.at(start),
            }),
            (Some(Ctx::Robot), "joint") => {
                joint = Some(Joint {
                    name: attr(&e, "name")?.unwrap_or_default(),
                    parent: String::new(),
                    child: String::new(),
                    kind: joint_kind(attr_ref(&e, "type")?),
                    origin: Default::default(),
                    limit: None,
                    mimic: None,
                    axis: None,
                    dynamics: None,
                    span: spans.at(start),
                })
            }
            (Some(Ctx::Link), "inertial") => {
                if let Some(l) = robot.links.last_mut() {
                    l.inertial = Some(Inertial {
                        mass: None,
                        inertia: None,
                        origin: Origin::default(),
                        span: spans.at(start),
                    });
                }
            }
            (Some(Ctx::Inertial), "origin") => {
                let xyz = at(vec3(&e, "xyz"), &mut spans, start)?;
                let rpy = at(vec3(&e, "rpy"), &mut spans, start)?;
                if let Some(i) = robot.links.last_mut().and_then(|l| l.inertial.as_mut()) {
                    i.origin.xyz = xyz.unwrap_or(i.origin.xyz);
                    i.origin.rpy = rpy.unwrap_or(i.origin.rpy);
                }
            }
            (Some(Ctx::Inertial), "mass") => {
                let mass = at(number(&e, "value"), &mut spans, start)?;
                if let Some(i) = robot.links.last_mut().and_then(|l| l.inertial.as_mut()) {
                    i.mass = mass;
                }
            }
            (Some(Ctx::Inertial), "inertia") => {
                let inertia = Inertia {
                    ixx: at(required(&e, "ixx"), &mut spans, start)?,
                    ixy: at(required(&e, "ixy"), &mut spans, start)?,
                    ixz: at(required(&e, "ixz"), &mut spans, start)?,
                    iyy: at(required(&e, "iyy"), &mut spans, start)?,
                    iyz: at(required(&e, "iyz"), &mut spans, start)?,
                    izz: at(required(&e, "izz"), &mut spans, start)?,
                };
                if let Some(i) = robot.links.last_mut().and_then(|l| l.inertial.as_mut()) {
                    i.inertia = Some(inertia);
                }
            }
            (Some(Ctx::Geometry), "mesh") => {
                let filename = at(required_str(&e, "filename"), &mut spans, start)?;
                let span = spans.at(start);
                if let Some(l) = robot.links.last_mut() {
                    l.meshes.push(MeshRef { filename, span });
                }
            }
            (Some(Ctx::Joint), "parent") => {
                if let Some(j) = joint.as_mut() {
                    j.parent = attr(&e, "link")?.unwrap_or_default();
                }
            }
            (Some(Ctx::Joint), "child") => {
                if let Some(j) = joint.as_mut() {
                    j.child = attr(&e, "link")?.unwrap_or_default();
                }
            }
            (Some(Ctx::Joint), "origin") => {
                if let Some(j) = joint.as_mut() {
                    if let Some(v) = at(vec3(&e, "xyz"), &mut spans, start)? {
                        j.origin.xyz = v;
                    }
                    if let Some(v) = at(vec3(&e, "rpy"), &mut spans, start)? {
                        j.origin.rpy = v;
                    }
                }
            }
            (Some(Ctx::Joint), "axis") => {
                if let Some(j) = joint.as_mut() {
                    j.axis = Some(at(vec3(&e, "xyz"), &mut spans, start)?.unwrap_or(DEFAULT_AXIS));
                }
            }
            (Some(Ctx::Joint), "dynamics") => {
                if let Some(j) = joint.as_mut() {
                    j.dynamics = Some(Dynamics {
                        damping: at(number(&e, "damping"), &mut spans, start)?,
                        friction: at(number(&e, "friction"), &mut spans, start)?,
                    });
                }
            }
            (Some(Ctx::Joint), "limit") => {
                if let Some(j) = joint.as_mut() {
                    j.limit = Some(Limit {
                        lower: at(number(&e, "lower"), &mut spans, start)?,
                        upper: at(number(&e, "upper"), &mut spans, start)?,
                        effort: at(number(&e, "effort"), &mut spans, start)?,
                        velocity: at(number(&e, "velocity"), &mut spans, start)?,
                    });
                }
            }
            (Some(Ctx::Joint), "mimic") => {
                let span = spans.at(start);
                if let Some(j) = joint.as_mut() {
                    j.mimic = Some(Mimic {
                        span,
                        joint: attr(&e, "joint")?.unwrap_or_default(),
                        multiplier: at(number(&e, "multiplier"), &mut spans, start)?,
                        offset: at(number(&e, "offset"), &mut spans, start)?,
                    });
                }
            }
            (Some(ctx @ (Ctx::Robot | Ctx::Link | Ctx::Joint | Ctx::Inertial)), _) => {
                let owner = match ctx {
                    Ctx::Link | Ctx::Inertial => robot.links.last().map(|l| l.name.clone()),
                    Ctx::Joint => joint.as_ref().map(|j| j.name.clone()),
                    _ => None,
                };
                let what = if ctx == Ctx::Inertial {
                    format!("inertial/{name}")
                } else {
                    name.to_string()
                };
                robot.unmapped.push(Unmapped {
                    what,
                    name: owner,
                    span: spans.at(start),
                });
            }
            _ => {}
        }
        if opens {
            stack.push(child_ctx(parent, name));
        } else if name == "joint" && parent == Some(Ctx::Robot) {
            // No End event follows a self-closing joint, so it is kept here
            // for lint to report instead of being replaced by the next one.
            robot.joints.extend(joint.take().map(with_default_axis));
        }
    }
    if !stack.is_empty() {
        return Err(ParseError("unclosed element at end of input".into()));
    }
    if !seen_robot {
        return Err(ParseError("no <robot> element found".into()));
    }
    Ok(robot)
}

/// The attribute value, borrowed from the source unless it needed unescaping.
/// The axis URDF gives a joint that does not author one.
const DEFAULT_AXIS: [f64; 3] = [1.0, 0.0, 0.0];

/// A moving joint always has an axis; a fixed or floating one has none even if one is written.
fn with_default_axis(mut j: Joint) -> Joint {
    match j.kind {
        JointKind::Revolute | JointKind::Continuous | JointKind::Prismatic => {
            j.axis.get_or_insert(DEFAULT_AXIS);
        }
        JointKind::Fixed | JointKind::Floating => j.axis = None,
        _ => {}
    }
    j
}

fn attr_ref<'a>(e: &'a BytesStart, key: &str) -> Result<Option<Cow<'a, str>>, ParseError> {
    match e.try_get_attribute(key) {
        Ok(Some(a)) => a
            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
            .map(Some)
            .map_err(|err| ParseError(err.to_string())),
        Ok(None) => Ok(None),
        Err(err) => Err(ParseError(err.to_string())),
    }
}

fn attr(e: &BytesStart, key: &str) -> Result<Option<String>, ParseError> {
    Ok(attr_ref(e, key)?.map(Cow::into_owned))
}

fn number(e: &BytesStart, key: &str) -> Result<Option<f64>, ParseError> {
    match attr_ref(e, key)? {
        None => Ok(None),
        Some(v) => v
            .trim()
            .parse::<f64>()
            .ok()
            // Rust accepts nan and inf; they would slip past every later comparison.
            .filter(|n| n.is_finite())
            .map(Some)
            .ok_or_else(|| ParseError(format!("invalid number `{v}` in attribute `{key}`"))),
    }
}

fn required(e: &BytesStart, key: &str) -> Result<f64, ParseError> {
    number(e, key)?.ok_or_else(|| ParseError(format!("missing attribute `{key}`")))
}

fn required_str(e: &BytesStart, key: &str) -> Result<String, ParseError> {
    attr(e, key)?.ok_or_else(|| ParseError(format!("missing attribute `{key}`")))
}

fn vec3(e: &BytesStart, key: &str) -> Result<Option<[f64; 3]>, ParseError> {
    let Some(v) = attr_ref(e, key)? else {
        return Ok(None);
    };
    let nums = v
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|_| ParseError(format!("invalid number in `{v}` in attribute `{key}`")))?;
    match nums[..] {
        [x, y, z] => Ok(Some([x, y, z])),
        _ => Err(ParseError(format!(
            "attribute `{key}` needs 3 numbers, found {}",
            nums.len()
        ))),
    }
}

fn joint_kind(ty: Option<Cow<str>>) -> JointKind {
    match ty.as_deref() {
        Some("revolute") => JointKind::Revolute,
        Some("continuous") => JointKind::Continuous,
        Some("prismatic") => JointKind::Prismatic,
        Some("fixed") => JointKind::Fixed,
        Some("floating") => JointKind::Floating,
        Some("planar") => JointKind::Planar,
        Some(other) => JointKind::Other(other.to_string()),
        None => JointKind::Other(String::new()),
    }
}

/// Prefix an attribute error with the line and column of its element.
fn at<T>(r: Result<T, ParseError>, spans: &mut Spans, start: usize) -> Result<T, ParseError> {
    r.map_err(|ParseError(msg)| {
        let s = spans.at(start);
        ParseError(format!("{}:{}: {msg}", s.line, s.col))
    })
}

/// Which kind of element we are inside; everything the parser does not read
/// is `Other`, so its children are never mistaken for top-level ones.
#[derive(Clone, Copy, PartialEq)]
enum Ctx {
    Robot,
    Link,
    Joint,
    Inertial,
    Shape,
    Geometry,
    Other,
}

fn child_ctx(parent: Option<Ctx>, name: &str) -> Ctx {
    match (parent, name) {
        (None, "robot") => Ctx::Robot,
        (Some(Ctx::Robot), "link") => Ctx::Link,
        (Some(Ctx::Robot), "joint") => Ctx::Joint,
        (Some(Ctx::Link), "inertial") => Ctx::Inertial,
        (Some(Ctx::Link), "visual" | "collision") => Ctx::Shape,
        (Some(Ctx::Shape), "geometry") => Ctx::Geometry,
        _ => Ctx::Other,
    }
}

/// Turns byte offsets into line and column. Offsets arrive in increasing
/// order, so each call only scans the text since the previous one; rescanning
/// from the start of the file made parsing quadratic.
pub(crate) struct Spans<'a> {
    src: &'a str,
    offset: usize,
    line: u32,
    col: u32,
}

impl<'a> Spans<'a> {
    pub(crate) fn new(src: &'a str) -> Self {
        Spans {
            src,
            offset: 0,
            line: 1,
            col: 1,
        }
    }

    pub(crate) fn at(&mut self, offset: usize) -> Span {
        if offset < self.offset {
            *self = Spans::new(self.src);
        }
        let skipped = &self.src[self.offset..offset];
        match skipped.rfind('\n') {
            Some(i) => {
                self.line += skipped.matches('\n').count() as u32;
                self.col = 1 + skipped[i + 1..].chars().count() as u32;
            }
            None => self.col += skipped.chars().count() as u32,
        }
        self.offset = offset;
        Span {
            line: self.line,
            col: self.col,
        }
    }
}
