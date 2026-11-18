# Conversion losses

`rigscan convert` lists what the target format could not hold, one line per kind of loss. Each kind has an id, and the
id is also what the JSON loss report (`--loss-report`) carries.

| Kind | Meaning |
|---|---|
| dropped | Source data the target cannot hold; it is not in the output. |
| changed | The data is in the output, rewritten into another form. |
| invented | A value the source did not have, written because the target requires one. |

Ids written by the converters:

| Id | Kind | What happened |
|---|---|---|
| `capsule` | dropped | URDF has no capsule shape; the shape was left out. |
| `dynamics` | dropped | MJCF holds damping and friction only on a moving joint; they were left out. |
| `inertial` | dropped | The target needs both a mass and an inertia tensor; an inertial without both was left out. |
| `inertial/frame` | changed | MuJoCo takes a full inertia tensor only in the body frame; the rotated frame was folded into the tensor. |
| `joint/<type>` | dropped | The target has no joint of this type; it was written as a fixed joint or left out. |
| `joint/floating` | dropped | MJCF holds a floating joint only on the root; this one was left out. |
| `joint/multiple` | dropped | URDF allows one joint into a link; the others were not written. |
| `joint/name` | dropped | A fixed joint becomes a weld in MJCF, and a weld has no name. |
| `joint/origin` | dropped | Joints into one MJCF body share its pose; this joint's different origin was not kept. |
| `joint/planar` | dropped | MJCF has no planar joint; it was left out. |
| `joint/tree` | dropped | MJCF is a tree; a joint into a link that already has a parent was not written. |
| `joint/undefined` | dropped | The joint's child link is not defined, so there is no body to put it on. |
| `limit` | dropped | MJCF could not keep this limit (a hinge needs both bounds, a fixed or floating joint takes none); it was left out. |
| `limit/effort` | dropped | MJCF has no effort limit on a joint; it was left out. |
| `limit/effort/default` | invented | URDF requires an effort on a limit and the source has none; 0 was written. |
| `limit/velocity` | dropped | MJCF has no velocity limit on a joint; it was left out. |
| `limit/velocity/default` | invented | URDF requires a velocity on a limit and the source has none; 0 was written. |
| `link` | dropped | No joint chain from a root reaches this link, so it has no place in the MJCF body tree. |
| `mimic` | dropped | MJCF is written without mimic coupling; it was left out. |

`joint/<type>` stands for the type that has no counterpart (`joint/ball`, for example).

An id that is not in this table comes from reading the source: an element or attribute the neutral model has no field
for, such as `visual/material`. It is reported as dropped, with the position in the source where it was found.
