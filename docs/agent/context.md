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
  rules RS001-RS006 (structural), README, Apache-2.0 LICENSE.
- Builds for `wasm32-unknown-unknown` (compile check only, not run in a
  browser).
- CLI: `rigscan scan <files>` (#2): `path:line:col: severity RULE: message`, exit 0/1/2. No report layer yet (#3).
- Not done: inertials/limits/mesh parsing, physics rules, MJCF, conversion,
  bindings, playground.
- A self-closing `<joint .../>` or a joint missing `<parent>`/`<child>` is kept
  with empty endpoints and reported by RS001 ("has no parent link").

## Model shape (grows with parsers; do not add fields speculatively)

`Robot { format, name, links, joints }`.

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

## Rule index

| ID | Severity | Meaning |
|---|---|---|
| RS001 | critical | joint references an undefined parent/child link, or has none |
| RS002 | critical | duplicate link name |
| RS003 | critical | duplicate joint name |
| RS004 | critical | link has more than one parent joint (flagged on the extra joint) |
| RS005 | critical | kinematic loop |
| RS006 | critical | more than one root link (flagged on each extra root) |

Planned next: mass <= 0, inertia triangle inequality, inverted joint limits,
unresolved mesh paths, mimic joints with bad references, units/scale sanity.
