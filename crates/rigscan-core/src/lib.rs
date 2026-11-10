//! Rigscan core: parse robot descriptions with source spans and lint them.

mod files;
mod lint;
mod mjcf;
mod mjcf_map;
mod mjcf_rules;
mod model;
mod physics;
mod report;
mod urdf;

pub use files::{Resolution, Resolver};
pub use lint::{lint, lint_with, Finding, Severity};
pub use mjcf::{parse_mjcf, IncludeLoader, Included, Mjcf, MjcfNode, Reference};
pub use mjcf_rules::lint_mjcf;
pub use model::{
    Element, Format, Inertia, Inertial, Joint, JointKind, Limit, Link, MeshRef, Mimic, Origin,
    ParseError, Robot, Span, Unmapped,
};
pub use report::{report, ElementStatus, Report, Status, Verdict};
pub use urdf::parse_urdf;
