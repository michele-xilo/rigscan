//! What each kind of conversion loss means.

use LossKind::{Changed, Dropped, Invented};

/// How a loss changed the robot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LossKind {
    /// Source data the target cannot hold.
    Dropped,
    /// Data kept but rewritten into another form.
    Changed,
    /// A value the source did not have, written because the target requires one.
    Invented,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LossInfo {
    pub kind: LossKind,
    pub summary: &'static str,
}

/// The description of a loss id a writer emits, or `None` for an id from a reader (an element or
/// attribute the neutral model has no field for).
pub fn loss_info(id: &str) -> Option<LossInfo> {
    // A joint type the target has no name for is `joint/<type>`, whatever the type is.
    let id = if id.starts_with("joint/") && !TABLE.iter().any(|(k, _, _)| *k == id) {
        "joint/<type>"
    } else {
        id
    };
    let (_, kind, summary) = TABLE.iter().find(|(k, _, _)| *k == id)?;
    Some(LossInfo {
        kind: *kind,
        summary,
    })
}

const TABLE: &[(&str, LossKind, &str)] = &[
    ("capsule", Dropped, "URDF has no capsule shape; the shape was left out."),
    ("dynamics", Dropped, "MJCF holds damping and friction only on a moving joint; they were left out."),
    ("inertial", Dropped, "The target needs both a mass and an inertia tensor; an inertial without both was left out."),
    ("inertial/frame", Changed, "MuJoCo takes a full inertia tensor only in the body frame; the rotated frame was folded into the tensor."),
    ("joint/<type>", Dropped, "The target has no joint of this type; it was written as a fixed joint or left out."),
    ("joint/floating", Dropped, "MJCF holds a floating joint only on the root; this one was left out."),
    ("joint/multiple", Dropped, "URDF allows one joint into a link; the others were not written."),
    ("joint/name", Dropped, "A fixed joint becomes a weld in MJCF, and a weld has no name."),
    ("joint/origin", Dropped, "Joints into one MJCF body share its pose; this joint's different origin was not kept."),
    ("joint/planar", Dropped, "MJCF has no planar joint; it was left out."),
    ("joint/tree", Dropped, "MJCF is a tree; a joint into a link that already has a parent was not written."),
    ("joint/undefined", Dropped, "The joint's child link is not defined, so there is no body to put it on."),
    ("limit", Dropped, "MJCF could not keep this limit (a hinge needs both bounds, a fixed or floating joint takes none); it was left out."),
    ("limit/effort", Dropped, "MJCF has no effort limit on a joint; it was left out."),
    ("limit/effort/default", Invented, "URDF requires an effort on a limit and the source has none; 0 was written."),
    ("limit/velocity", Dropped, "MJCF has no velocity limit on a joint; it was left out."),
    ("limit/velocity/default", Invented, "URDF requires a velocity on a limit and the source has none; 0 was written."),
    ("link", Dropped, "No joint chain from a root reaches this link, so it has no place in the MJCF body tree."),
    ("mimic", Dropped, "MJCF is written without mimic coupling; it was left out."),
];

#[cfg(test)]
mod tests {
    use super::*;

    /// Every id a writer passes to `lose`, read from the source so a new loss cannot be added
    /// without a description. A format string such as `joint/{t}` is checked as `joint/x`.
    fn writer_ids() -> Vec<String> {
        let mut ids = Vec::new();
        for src in [include_str!("urdf_write.rs"), include_str!("mjcf_write.rs")] {
            let code = src.split("#[cfg(test)]").next().unwrap_or(src);
            for call in code.split("lose(").skip(1) {
                let Some(start) = call.find('"') else {
                    continue;
                };
                // A call without a literal id (the definitions) has other code before the quote.
                if call[..start].contains(')') || call[..start].contains('{') {
                    continue;
                }
                let rest = &call[start + 1..];
                let id = &rest[..rest.find('"').unwrap_or(0)];
                ids.push(match id.find('{') {
                    Some(i) => format!("{}x", &id[..i]),
                    None => id.to_string(),
                });
            }
        }
        // The URDF writer picks the id of a made-up limit value from a table, not a `lose` call.
        for part in include_str!("urdf_write.rs").split("\"limit/").skip(1) {
            if let Some(end) = part.find('"') {
                ids.push(format!("limit/{}", &part[..end]));
            }
        }
        ids.sort();
        ids.dedup();
        ids
    }

    #[test]
    fn every_loss_a_writer_can_report_is_described() {
        let ids = writer_ids();
        assert!(ids.len() >= 19, "found too few ids: {ids:?}");
        let missing: Vec<_> = ids.iter().filter(|i| loss_info(i).is_none()).collect();
        assert!(missing.is_empty(), "no description for {missing:?}");
    }

    #[test]
    fn the_loss_document_lists_every_loss() {
        let doc = include_str!("../../../docs/losses.md");
        let missing: Vec<_> = TABLE
            .iter()
            .filter(|(id, _, _)| !doc.contains(&format!("`{id}`")))
            .map(|(id, _, _)| id)
            .collect();
        assert!(missing.is_empty(), "docs/losses.md lacks {missing:?}");
    }

    #[test]
    fn a_joint_type_without_its_own_entry_is_described_as_joint_type() {
        assert_eq!(loss_info("joint/ball"), loss_info("joint/<type>"));
        assert_eq!(loss_info("joint/planar").unwrap().kind, Dropped);
        assert!(loss_info("visual/material").is_none());
    }
}
