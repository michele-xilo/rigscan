//! Rigscan core: parse robot descriptions with source spans and lint them.

mod files;
mod lint;
mod mjcf;
mod mjcf_map;
mod mjcf_rules;
mod model;
mod physics;
mod report;
mod rotation;
mod urdf;
mod usd;
mod usd_map;
mod usd_rules;

pub use files::{Resolution, Resolver};
pub use lint::{lint, lint_with, Finding, Severity};
pub use mjcf::{parse_mjcf, IncludeLoader, Included, Mjcf, MjcfNode, Reference};
pub use mjcf_rules::lint_mjcf;
pub use model::{
    Dynamics, Element, Format, Geometry, Inertia, Inertial, Joint, JointKind, Limit, Link, Mimic,
    Origin, ParseError, Robot, Role, Shape, Span, Unmapped,
};
pub use report::{report, ElementStatus, Report, Status, Verdict};
pub use urdf::parse_urdf;
pub use usd::{parse_usd, Usd, UsdBody, UsdJoint, UsdLoader};
pub use usd_rules::lint_usd;
