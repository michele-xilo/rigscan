# Usage

## Scan

```bash
rigscan scan robot.urdf
# robot.urdf:5:5: critical RS001: joint `j1` child link `ghost` is not defined
```

`rigscan scan` takes files and directories (default: the current directory). Directories are searched recursively
for `.urdf`, `.usd`, `.usda`, `.usdc` and `.xml` files with a `<mujoco>` root, skipping anything in `.gitignore` and
any MJCF or USD file that another scanned file includes or references (a fragment, not a robot on its own). A file
named on the command line is always scanned. Files are scanned in parallel and the output is sorted, so it is the
same on every run.

Findings go to stdout and a summary (`Found 3 findings in 2 files.` or `All checks passed!`) to stderr.

| Exit code | Meaning |
|---|---|
| 0 | no critical findings |
| 1 | at least one critical finding |
| 2 | a file could not be scanned (unreadable, or it does not parse) |

## Output

`--output-format` picks the output:

- `text` (default): one line per finding, `file:line:col: severity RULE: message`.
- `full`: each finding with its source line and a caret under the column.
- `report`: a verdict line per file, then the status of every link and joint with its findings underneath. There is
  no numeric score.
- `json`: one document with `version`, `files` (verdict, element statuses and findings with line and column, or an
  `error` for a file that could not be scanned) and a `summary`.

```bash
rigscan scan --output-format report robot.urdf
# robot.urdf: 1 healthy, 0 warnings, 1 critical
#   healthy  link  base
#   critical joint j1
#     robot.urdf:3:3: critical RS001: joint `j1` child link `ghost` is not defined
```

Other options: `--statistics` counts findings per rule, `-q`/`--quiet` prints only the summary and only when
something is wrong, and `--color auto|always|never` controls colour (teal healthy, amber warning, red critical;
`auto` honours `NO_COLOR`).

USD has no source positions, so findings name the file only and JSON `line` and `column` are `null`.

## Formats

| Format | Reads | Lints |
|---|---|---|
| URDF | links, joints, types, origins, axes, limits, dynamics, mimic, inertials, geometry (box, sphere, cylinder, mesh) | RS001-RS015 |
| MJCF | element tree with `include`s and `default` classes applied; bodies, joints, geoms (primitives and meshes), inertials, orientation | RS001-RS018 |
| USD (UsdPhysics) | rigid bodies, joints, mass, centre of mass, joint axes and limits, composed from references, payloads and sublayers; `.usda` and `.usdc` | RS008-RS012, RS019-RS022 |

What a model cannot hold (actuators, tendons, materials, USD collision shapes and so on) is kept in the robot's
`unmapped` list, which the conversion loss report will use.

## Library

```rust
use rigscan_core::{lint, parse_urdf};

let robot = parse_urdf(&std::fs::read_to_string("robot.urdf")?)?;
for f in lint(&robot) {
    println!("{} line {}: {}", f.rule, f.span.line, f.message);
}
```

`parse_mjcf` and `parse_usd` take a loader that fetches `include`d and referenced files, so the core does no
filesystem work and compiles for `wasm32-unknown-unknown`. `lint_mjcf` and `lint_usd` add the format's own rules.
