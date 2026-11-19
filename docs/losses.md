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
| `dynamics` | dropped | The target holds damping and friction only in a form this joint cannot use (MJCF on a moving joint only, USD not at all); they were left out. |
| `inertial` | dropped | The target needs both a mass and an inertia tensor; an inertial without both was left out. |
| `inertial/frame` | changed | MuJoCo takes a full inertia tensor only in the body frame; the rotated frame was folded into the tensor. |
| `joint/<type>` | dropped | The target has no joint of this type; it was written as a fixed joint or left out. |
| `joint/floating` | dropped | The target holds a floating joint only on the root (MJCF) or not at all (USD); this one was left out. |
| `joint/multiple` | dropped | URDF allows one joint into a link; the others were not written. |
| `joint/name` | dropped | A fixed joint becomes a weld in MJCF, and a weld has no name. |
| `joint/origin` | dropped | Joints into one MJCF body share its pose; this joint's different origin was not kept. |
| `joint/planar` | dropped | The target has no planar joint; it was written as a fixed joint or left out. |
| `joint/tree` | dropped | MJCF is a tree; a joint into a link that already has a parent was not written. |
| `joint/undefined` | dropped | The joint's child link is not defined, so there is no body to put it on. |
| `limit` | dropped | The target could not keep this limit (an MJCF or USD hinge needs both bounds, a fixed or floating joint takes none); it was left out. |
| `limit/effort` | dropped | The target has no effort limit on this joint (MJCF has none, USD holds one only as a positive drive force); it was left out. |
| `limit/effort/default` | invented | URDF requires an effort on a limit and the source has none; 0 was written. |
| `limit/velocity` | dropped | The target has no velocity limit on a joint (MJCF and USD have none); it was left out. |
| `limit/velocity/default` | invented | URDF requires a velocity on a limit and the source has none; 0 was written. |
| `name` | changed | A USD prim name must be an identifier (letters, digits, underscores, not starting with a digit); it was rewritten. |
| `link` | dropped | No joint chain from a root reaches this link, so it has no place in the MJCF body tree. |
| `mesh` | dropped | USD holds a mesh as points and the model keeps only a file name; the shape was left out. |
| `mimic` | dropped | MJCF and USD are written without mimic coupling; it was left out. |

`joint/<type>` stands for the type that has no counterpart (`joint/ball`, for example).

An id that is not in this table comes from reading the source: an element or attribute the neutral model has no field
for, such as `visual/material`. It is reported as dropped, with the position in the source where it was found.
