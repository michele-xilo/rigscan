//! Format-neutral robot model shared by every parser and rule.

/// 1-based line and column in the source text; line 0 means the position is unknown ([`Span::NONE`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Span {
    pub line: u32,
    pub col: u32,
}

impl Span {
    /// Where nothing is known: USD has no source positions for its elements.
    pub const NONE: Span = Span { line: 0, col: 0 };

    pub fn is_none(&self) -> bool {
        *self == Span::NONE
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Link {
    pub name: String,
    pub inertial: Option<Inertial>,
    /// Mesh files referenced by the link's `<visual>` and `<collision>`
    /// geometry, in document order.
    pub meshes: Vec<MeshRef>,
    pub span: Span,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeshRef {
    pub filename: String,
    pub span: Span,
}

/// The `<inertial>` element. `mass` is optional so a rule can tell a missing
/// mass from a zero one.
#[derive(Debug, Clone, PartialEq)]
pub struct Inertial {
    pub mass: Option<f64>,
    pub inertia: Option<Inertia>,
    pub span: Span,
}

/// Upper triangle of the symmetric inertia tensor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inertia {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Joint {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub kind: JointKind,
    pub origin: Origin,
    pub limit: Option<Limit>,
    pub mimic: Option<Mimic>,
    pub span: Span,
}

/// A joint's `type` attribute. A misspelled or missing type is kept as
/// `Other` so a rule can report it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
    Other(String),
}

/// Pose of a joint frame relative to its parent link. Missing attributes
/// default to zero, as in the URDF specification.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Origin {
    pub xyz: [f64; 3],
    pub rpy: [f64; 3],
}

/// The `<limit>` element. Every field is optional so a rule can tell a
/// missing value from a zero.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Limit {
    pub lower: Option<f64>,
    pub upper: Option<f64>,
    pub effort: Option<f64>,
    pub velocity: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Mimic {
    pub joint: String,
    pub multiplier: Option<f64>,
    pub offset: Option<f64>,
    pub span: Span,
}

/// The file format a [`Robot`] was read from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Format {
    Urdf,
    Mjcf,
    Usd,
}

/// A robot description, independent of the format it was read from.
///
/// Rules read only this type, so one rule serves every format.
#[derive(Debug, Clone, PartialEq)]
pub struct Robot {
    pub format: Format,
    pub name: Option<String>,
    /// The robot element, or the start of the file when the format has none.
    pub span: Span,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
    /// Parts of the source the model cannot hold, kept so a conversion can report them as lost.
    pub unmapped: Vec<Unmapped>,
}

/// A source element or attribute the neutral model has no field for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Unmapped {
    /// The element tag, or the attribute name when only an attribute is lost.
    pub what: String,
    /// The element's `name`, or the name of the body the attribute was on.
    pub name: Option<String>,
    pub span: Span,
}

impl Robot {
    pub fn new(format: Format) -> Self {
        Robot {
            format,
            name: None,
            span: Span { line: 1, col: 1 },
            links: Vec::new(),
            joints: Vec::new(),
            unmapped: Vec::new(),
        }
    }
}

/// The part of a robot a finding is about. Links and joints are identified by position, so
/// duplicate names still point at the right one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Element {
    Robot,
    Link(usize),
    Joint(usize),
}

#[derive(Debug)]
pub struct ParseError(pub String);
