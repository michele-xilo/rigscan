//! Rigscan core: parse robot descriptions with source spans and lint them.

mod files;
mod lint;
mod model;
mod physics;
mod report;
mod urdf;

pub use files::{Resolution, Resolver};
pub use lint::{lint, lint_with, Finding, Severity};
pub use model::{
    Element, Format, Inertia, Inertial, Joint, JointKind, Limit, Link, MeshRef, Mimic, Origin,
    ParseError, Robot, Span,
};
pub use report::{report, ElementStatus, Report, Status, Verdict};
pub use urdf::parse_urdf;
