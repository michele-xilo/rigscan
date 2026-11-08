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

    fn span_at(src: &str, offset: usize) -> Span {
        let before = &src[..offset];
        let line = before.matches('\n').count() as u32 + 1;
        let col = before.rsplit('\n').next().map_or(0, |l| l.chars().count()) as u32 + 1;
        Span { line, col }
    }

    let mut reader = Reader::from_str(src);
    let mut robot = Robot::new(Format::Urdf);
    let mut joint: Option<Joint> = None;

    // Only direct children of <robot> are links and joints; <transmission> and
    // <gazebo> contain <joint>/<link> elements that merely reference them.
    let mut depth = 0u32;
    let mut seen_robot = false;

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
            ("robot", 0) => {
                seen_robot = true;
                robot.name = attr(&e, "name")?
            }
            ("link", 1) => robot.links.push(Link {
                name: attr(&e, "name")?.unwrap_or_default(),
                span: span_at(src, start),
            }),
            ("joint", 1) => {
                joint = Some(Joint {
                    name: attr(&e, "name")?.unwrap_or_default(),
                    parent: String::new(),
                    child: String::new(),
                    span: span_at(src, start),
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
    if !seen_robot {
        return Err(ParseError("no <robot> element found".into()));
    }
    Ok(robot)
}
