//! URDF parser.

use crate::model::{Format, Joint, Link, ParseError, Robot, Span};

/// Parse a URDF document, keeping the source span of every link and joint.
pub fn parse_urdf(src: &str) -> Result<Robot, ParseError> {
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

    let mut reader = Reader::from_str(src);
    let mut spans = Spans::new(src);
    let mut robot = Robot::new(Format::Urdf);
    let mut joint: Option<Joint> = None;

    // Only direct children of <robot> are links and joints; <transmission> and
    // <gazebo> contain <joint>/<link> elements that merely reference them.
    let mut depth = 0u32;

    loop {
        // Text between tags is its own event, so this is the offset of the next '<'.
        let start = reader.buffer_position() as usize;
        let event = reader.read_event().map_err(|e| ParseError(e.to_string()))?;
        let (e, opens) = match event {
            Event::Start(e) => (e, true),
            Event::Empty(e) => (e, false),
            Event::End(e) => {
                depth = depth.saturating_sub(1);
                if depth == 1 && e.name().as_ref() == "joint" {
                    robot.joints.extend(joint.take());
                }
                continue;
            }
            Event::Eof => break,
            _ => continue,
        };
        match (e.name().as_ref(), depth) {
            ("robot", 0) => robot.name = attr(&e, "name")?,
            ("link", 1) => robot.links.push(Link {
                name: attr(&e, "name")?.unwrap_or_default(),
                span: spans.at(start),
            }),
            ("joint", 1) => {
                joint = Some(Joint {
                    name: attr(&e, "name")?.unwrap_or_default(),
                    parent: String::new(),
                    child: String::new(),
                    span: spans.at(start),
                })
            }
            ("parent", 2) => {
                if let Some(j) = joint.as_mut() {
                    j.parent = attr(&e, "link")?.unwrap_or_default();
                }
            }
            ("child", 2) => {
                if let Some(j) = joint.as_mut() {
                    j.child = attr(&e, "link")?.unwrap_or_default();
                }
            }
            _ => {}
        }
        if opens {
            depth += 1;
        }
    }
    Ok(robot)
}

/// Turns byte offsets into line and column. Offsets arrive in increasing
/// order, so each call only scans the text since the previous one; rescanning
/// from the start of the file made parsing quadratic.
struct Spans<'a> {
    src: &'a str,
    offset: usize,
    line: u32,
    col: u32,
}

impl<'a> Spans<'a> {
    fn new(src: &'a str) -> Self {
        Spans {
            src,
            offset: 0,
            line: 1,
            col: 1,
        }
    }

    fn at(&mut self, offset: usize) -> Span {
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
