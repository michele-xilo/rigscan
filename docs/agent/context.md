# Context

## What it is

Rigscan: a health check for robot descriptions. A fast cross-format linter plus
(planned) URDF <-> MJCF conversion that reports what each conversion lost.
Rust core, targets: CLI, WebAssembly (browser playground), Python bindings.
Metaphor: the file is a patient, a scan produces a report (healthy / warning /
critical per link and joint).

## Wedge

- Linting has essentially no adoption today: `check_urdf` only checks parsing;
  `urdf_validator` (2 stars, ~140 PyPI downloads/month) is URDF/xacro only and
  checks robot-level physics (COM, torque, reach), not file hygiene.
- URDF -> MJCF has one maintained-looking tool (`urdf2mjcf`, ~21k PyPI
  downloads/month, no pushes since 2025-09).
- No tool reports what a conversion drops (tendons, equality constraints,
  contact parameters, ...).
- USD conversion is owned by NVIDIA/Newton (`urdf-usd-converter`,
  `mujoco-usd-converter`, ~110-118k downloads/month, pushed weekly). Do not
  compete on USD conversion directly.

Market figures were pulled on 2026-11-08 from the GitHub API, crates.io and
pypistats. Download counts include CI and mirrors; treat as rough scale.

## Scope

v0.1 (target: about 14 weeks, assumes 1-2 people part-time):

- URDF and MJCF parsers that keep source line/column spans.
- One internal model: kinematic tree, inertials, joints and limits, geoms,
  basic actuators. Features a target format cannot express are kept as
  annotations; they drive the loss report.
- About 25 lint rules, stable IDs `RS001`..., two groups: structure and
  physics sanity.
- URDF <-> MJCF conversion with a loss report.
- Output: text, JSON, SARIF.
- Python wheel (PyO3/maturin), WASM build, static browser playground that
  draws the kinematic tree as a 2D diagram coloured by status.

Out of v0.1: xacro (v0.3; users pre-expand), USD writer (v0.2, `.usda` with
UsdPhysics only), SDF, mesh conversion, simulation.

## Milestones

1. Weeks 1-2: workspace, URDF parser with spans, model, first rules, CLI.
   Done when it lints ~20 real robots from `robot_descriptions` without crashes.
2. Weeks 3-5: MJCF (includes, defaults, classes resolved). Done when MuJoCo
   Menagerie models load.
3. Weeks 6-8: reach ~25 rules, JSON and SARIF, a fixture per rule.
   **Week 8 is the go/no-go review**: do the rules catch things people care
   about?
4. Weeks 9-11: URDF <-> MJCF conversion with loss report; converted models
   must load in MuJoCo.
5. Weeks 12-13: PyO3 wheel, WASM build, playground.
6. Week 14: docs, GitHub Action, launch posts (ROS Discourse, MuJoCo, Isaac).

## Status (update when it changes)

- Done: workspace, `rigscan-core` with a format-neutral `Robot` (#1), URDF parser (links, joints, spans, robot name, joint type/origin/limit/mimic, inertials, mesh refs; #5),
  rules RS001-RS022 (structure, mass and inertia, joint limits, mesh files, mimic joints, MJCF names, USD physics), README, Apache-2.0 LICENSE.
- Builds for `wasm32-unknown-unknown` (compile check only, not run in a
  browser).
- CLI: `rigscan scan <files>` (#2): `path:line:col: severity RULE: message`, exit 0/1/2. No report layer yet (#3).
- MJCF: `parse_mjcf` gives a resolved element tree (#10) and `Mjcf::to_robot` maps it onto `Robot` (#11). `rigscan scan` reads MJCF (#60) and runs `lint_mjcf`: the shared rules plus RS016-RS018.
- USD: `parse_usd` composes a stage and `Usd::to_robot` maps UsdPhysics onto `Robot` (#15); `rigscan scan` reads `.usda`, `.usdc` and `.usd`; runs `lint_usd`: the shared rules plus RS019-RS022.
- Not done: conversion, bindings, playground.
- A self-closing `<joint .../>` or a joint missing `<parent>`/`<child>` is kept
  with empty endpoints and reported by RS001 ("has no parent link").

## Model shape (grows with parsers; do not add fields speculatively)

`Robot { format, name, span, links, joints }`. `Finding { rule, severity, message, span, element }`, where
`Element` is `Robot`, `Link(index)` or `Joint(index)` (positions, so duplicate names stay distinct).
`report(&Robot, &[Finding])` gives a `Status` per link and joint (worst finding wins) and a `Verdict`; the
robot element is listed only when it has a finding. The CLI renders `text`, `full`, `report` and `json` (version 1). `rigscan scan` walks directories with the
`ignore` crate (`.gitignore` honoured even outside a git repository), scans in parallel with rayon and collects
in input order so output is deterministic. The summary line is on stderr to keep stdout equal to the findings.

- `Link { name, inertial, meshes, span }`: `Inertial { mass: Option, inertia: Option<Inertia> }`,
  `MeshRef { filename, span }` from `<visual>` and `<collision>` geometry (#5).
- `Joint { name, parent, child, kind, origin, limit, mimic, span }`:
  `JointKind` keeps an unknown or missing `type` as `Other(String)` so a rule
  can report it; `Origin` defaults to zero; `Limit` and `Mimic` fields are
  `Option` so a rule can tell missing from zero (#5).
- Parse errors: an invalid number, a wrong-length vector, a missing required
  attribute (`inertia` fields, `mesh filename`) is a `ParseError` prefixed with
  `line:col:`. A scan stops at the first one. If users want all problems
  reported at once, make these findings instead.
- Not yet: geoms with primitives, actuators, axis, link origins (#11, viewer),
  USD data (#15), loss annotations (#18).

Known risk for #11: an MJCF body can have several joints to its parent, or
none. The model has one parent/child edge per joint, so RS004 would flag
those. Decide then: chain through synthetic links, or give the child link a
joint list.

`span` is required on every element. #15 will probably make it optional,
because `.usdc` has no line numbers.

## MJCF tree

`parse_mjcf(src, &dyn IncludeLoader) -> Mjcf { root, unresolved_includes, unknown_classes }`. `MjcfNode { tag, attrs,
children, span, file }`; `file` is the include the element came from, `None` for the main file. An `<include>` is
replaced by the children of the included `<mujoco>` root; the loader receives the path as written, so the core
does no file access. An include the loader cannot supply is recorded, not an error. An element takes its class
from its own `class`, else the nearest `childclass`, else `main`; attributes it does not set are filled from that
class's default for its tag, then from parent classes. Nesting is capped at 256 levels (counted across includes) and
include depth at 16, because the passes recurse and CLI worker threads have 2 MiB stacks.

## MJCF to the neutral model

The robot's structure is kept as written; no links are invented for joints. `<worldbody>` is the root link `world`.
Each `<joint>` is one `Joint` from the parent body's link to the body's own, so a body with several joints has
several joints between the same two links, in document order. A body with no joint gets a fixed joint named after
the body (made unique against real joint names), as MuJoCo welds it. Unnamed bodies and joints get generated names.
Kinds: `hinge` is `Revolute` with a `range` and `Continuous` without; `slide` is `Prismatic`; `free` is `Floating`;
`ball` and anything else is `Other`. Hinge `range` is converted from degrees to radians unless
`<compiler angle="radian">`; `autolimits` and `limited` decide whether `range` counts. Body `pos` is the joint
origin. A body without `<inertial>` has no inertial because MuJoCo infers it from geoms. Mesh geoms become
`MeshRef`s joined with `meshdir`.

`Robot.unmapped` lists what the model cannot hold, with name and span, for the loss report (#18): actuators,
tendons, equality constraints, contacts, sensors, keyframes, primitive geoms, body orientation (`quat`, `euler`,
`axisangle`, `xyaxes`, `zaxis`; rotation is not converted, so `rpy` stays zero) and anything inside `frame`,
`replicate`, `attach`, `composite` or `flexcomp`, which are not expanded.

RS004 accepts several joints between the same two links for MJCF and still flags a link reached from two different
parents. RS013 applies to URDF only: MJCF joints may be unlimited and have no effort or velocity.

## MJCF includes and spans

`IncludeLoader::load(file, from)` returns `Included { path, text }`; `from` is the `path` of the file containing the
`<include>` (`None` for the main file), because MuJoCo reads an include relative to the file that contains it. The
CLI's `FsLoader` joins `file` to the directory of `from`. `Span` has no file, so every `MjcfNode` and `Reference`
also has an `anchor`: its own `span` in the main file, and the outermost `<include>` for anything read from an
included file. The mapper uses the anchor, so a finding on an included element prints the scanned file's path and
the include's line. `.xml` is taken as MJCF when the first 16 KiB contain `<mujoco`; a directory scan skips other
`.xml` files (ROS launch files, `package.xml`). A directory scan also drops a file that another scanned file includes: `FsLoader` records the files it read and
`scan` removes those results afterwards, counting only includes from files that scanned without error. A file named on the
command line is always scanned.

## USD to the neutral model

`parse_usd(root, loader)` opens the stage with `openusd` (pinned `=0.8.0`) through an `ar::Resolver` that reads layers
from a caller-supplied `UsdLoader` (bytes, because `.usdc` is binary), so the core does no file access and
references, payloads and sublayers compose. It returns a plain `Usd { bodies, joints, unmapped, composition_errors }`;
a reference the loader cannot supply is a `composition_errors` entry, not an error. `Usd::to_robot` makes a link of
each prim with `PhysicsRigidBodyAPI`, named by its prim path, and of each prim a joint connects that is not a rigid
body (a ground collider); a joint target that does not exist is not added, so RS001 reports it. Each `Physics*Joint`
is one `Joint` from `body0` to `body1`; an empty end is the root link `world`, added only when a joint needs it.
`PhysicsRevoluteJoint` is `Revolute` with a limit and `Continuous` without (limits are degrees in USD and become
radians); prismatic limits stay as authored; fixed is `Fixed`; any other type is `Other` named after the type
(`spherical`, `distance`, `joint`). `localPos0` is the joint origin, in stage units (`metersPerUnit` is not
applied). A mass or diagonal inertia of zero means "computed from shapes" in USD and stays `None`.

USD has no source positions, so every span is `Span::NONE` (line 0): the CLI prints the file path alone in `text`,
`full` and `report`, and JSON `line` and `column` are `null`. `Robot.unmapped` lists scenes, collision shapes, drives,
materials, `principalAxes` and a non-identity `localRot0`/`localRot1`. The CLI's `FsUsdLoader` records the layers
read, and a directory scan drops a file that another scanned file pulled in, like MJCF includes.

`openusd` is MIT except `core_schemas.rs`, under the Tomorrow Open Source Technology License 1.0 (Apache-2.0 with a
different trademarks section); a release has to carry the notice in the crate's `third-party/`. Opening and reading a
synthetic 1000-body chain takes about 43 ms (4.7 to 7.3 MiB/s), most of it `openusd`'s composition, so USD is
slower than URDF and MJCF.

`lint_usd(&Usd, &Robot)` is the USD entry: `lint` plus RS019-RS022. `Usd` records `articulation_roots` and, per body,
whether a collision shape sits at or under it (a collider belongs to its nearest rigid body), because a collider
supplies the mass PhysX computes; a missing `PhysicsMassAPI` alone is not a finding. Findings carry `Span::NONE`.

## Rule index

| ID | Severity | Meaning |
|---|---|---|
| RS001 | critical | joint references an undefined parent/child link, or has none |
| RS002 | critical | duplicate link name |
| RS003 | critical | duplicate joint name |
| RS004 | critical | link has more than one parent joint (flagged on the extra joint); in MJCF, joints from the same parent do not count |
| RS005 | critical | kinematic loop |
| RS006 | critical | more than one root link (flagged on each extra root) |
| RS007 | critical | robot has no links (flagged on the robot element) |
| RS008 | critical | negative mass (flagged on the `<inertial>` element) |
| RS009 | warning | zero mass on a link whose parent joint moves; massless frames on fixed joints and roots are fine |
| RS010 | critical | negative principal moment, from the full tensor; tensors with every moment under 1e-12 are ignored |
| RS011 | warning | principal moments break A + B >= C (a flat plate, A + B == C, is valid) |
| RS012 | critical | revolute/prismatic `lower` above `upper` |
| RS013 | critical | URDF only: revolute/prismatic joint without `<limit>`, or a limit without `effort` or `velocity` |
| RS014 | critical | mesh file not found (flagged on the `<mesh>` element); only when the caller passes a `Resolver` to `lint_with`, and never when it answers `Unknown` |
| RS015 | critical | `<mimic>` of an undefined joint or of the joint itself (flagged on the `<mimic>` element) |
| RS016 | critical | MJCF: `class` or `childclass` names a default class that is not defined |
| RS017 | critical | MJCF: mesh geom names an asset that is not defined; a mesh asset is named by `name`, else its file stem, and one with inline vertices still counts |
| RS018 | critical | MJCF: `<include>` file does not exist |
| RS019 | critical | USD: a joint connects a link to itself, or the world to itself (flagged on the joint) |
| RS020 | warning | USD: rigid body with no mass (or zero) and no collision shape at or under it (flagged on the link) |
| RS021 | warning | USD: the stage has joints but no prim with `PhysicsArticulationRootAPI` (flagged on the robot) |
| RS022 | critical | USD: a reference, payload or sublayer could not be composed, from `Usd.composition_errors` (flagged on the robot) |

`lint_mjcf(&Mjcf, &Robot, &dyn Resolver)` is the MJCF entry: `lint_with` on the mapped robot plus RS016-RS018.
Those findings are on the `Robot` element at the anchor span, so one inside an included file prints the include's
line. Probes for MJCF rules are `.xml` files in `scripts/compare/probes/`; only MuJoCo is run on them. Probes for USD rules are `.usda`; none of the compared tools read USD.

`lint` never touches files; `lint_with` asks a `Resolver` whether a reference exists. The CLI resolver treats
`package://name/...` as the nearest ancestor directory called `name`. The corpus holds URDFs only, so a
CLI scan of it reports RS014 for every mesh; the core corpus test uses `lint` and is unaffected.

Planned next: units/scale sanity.

## End-to-end corpus

- `scripts/fetch-corpus.sh` downloads 69 URDFs from Gepetto/example-robot-data at a pinned commit into
  gitignored `corpus/`. The file list is `scripts/corpus-urdf.txt`.
- `crates/rigscan-core/tests/e2e_corpus.rs`: lint every file (must be clean), perf report, then 200 seeded
  corruptions (8 modes x 25), each must be detected. Modes: RS001-RS006, invalid number, truncated XML.
- Real findings in upstream files (empty robot, invalid inertias, zero mass on moving links) are listed with
  counts in `KNOWN_FINDINGS`; the test fails if one stops being reported or a new one appears.
- `scripts/fetch-mjcf-corpus.sh` downloads the 268 `.xml` files of MuJoCo Menagerie (no meshes) at a pinned commit into
  `corpus/mujoco_menagerie`. `crates/rigscan-core/tests/e2e_mjcf_corpus.rs` parses every file with includes read from disk,
  then maps and lints each model (a file that no other file includes) with `lint_mjcf` and no mesh resolver. Of the 268
  files, 127 are models and 141 are include-only fragments, which are not linted alone. The two known findings are
  keyframe-only files that nothing includes (RS007).
- The USD corpus waits for its reader.
