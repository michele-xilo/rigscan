//! Rigscan core: parse robot descriptions with source spans and lint them.

mod lint;
mod model;
mod urdf;

pub use lint::{lint, Finding, Severity};
pub use model::{
    Format, Inertia, Inertial, Joint, JointKind, Limit, Link, MeshRef, Mimic, Origin, ParseError,
    Robot, Span,
};
pub use urdf::parse_urdf;
