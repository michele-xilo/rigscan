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

    loop {
        // Text between tags is its own event, so this is the offset of the next '<'.
        let start = reader.buffer_position() as usize;
        let event = reader.read_event().map_err(|e| ParseError(e.to_string()))?;
        match event {
            Event::Start(e) | Event::Empty(e) => match e.name().as_ref() {
                "robot" => robot.name = attr(&e, "name")?,
                "link" => robot.links.push(Link {
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
                robot.joints.extend(joint.take());
            }
            Event::Eof => break,
            _ => {}
        }
    }
    Ok(robot)
}
