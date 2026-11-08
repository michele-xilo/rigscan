//! Format-neutral robot model shared by every parser and rule.

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

/// The file format a [`Robot`] was read from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Format {
    Urdf,
}

/// A robot description, independent of the format it was read from.
///
/// Rules read only this type, so one rule serves every format.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Robot {
    pub format: Format,
    pub name: Option<String>,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
}

impl Robot {
    pub fn new(format: Format) -> Self {
        Robot {
            format,
            name: None,
            links: Vec::new(),
            joints: Vec::new(),
        }
    }
}

#[derive(Debug)]
pub struct ParseError(pub String);
