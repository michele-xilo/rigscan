# Rigscan

[![codecov](https://codecov.io/gh/michele-xilo/rigscan/branch/main/graph/badge.svg)](https://codecov.io/gh/michele-xilo/rigscan)

A health check for robot descriptions.

Rigscan scans robot description files, reports what is wrong with them, and
(planned) converts between formats while telling you what each conversion lost.
The core is written in Rust and is designed to run as a CLI, in the browser via
WebAssembly, and from Python.

> **Status: pre-alpha.** URDF parsing, seven structural lint rules and a `scan`
> command exist. There is no MJCF or USD support, no conversion, and nothing is
> published yet. The API will change.

## What works today

- Parse URDF links and joints, keeping the line and column of each element,
  along with joint types, origins, limits and mimic, link inertials, and the
  mesh files a link references. Invalid numbers are reported with their
  line and column.
- Seven structural lint rules, all critical:
  - `RS001` a joint references a parent or child link that is not defined, or has none
  - `RS002` duplicate link names
  - `RS003` duplicate joint names
  - `RS004` a link with more than one parent joint
  - `RS005` a kinematic loop
  - `RS006` more than one root link
  - `RS007` a robot with no links
- `rigscan-core` compiles for `wasm32-unknown-unknown`.

```bash
cargo run -p rigscan-cli -- scan robot.urdf
# robot.urdf:5:5: critical RS001: joint `j1` child link `ghost` is not defined
```

`rigscan scan` takes one or more files and exits 0 when clean, 1 when there
are critical findings, and 2 when a file cannot be scanned. File type is
detected from the extension (`.urdf` for now).

`--output-format` picks the output:

- `text` (default): one line per finding, as above.
- `report`: a verdict line per file, then the status of every link and joint
  with its findings underneath. There is no numeric score.
- `json`: one document with `version`, `files` (verdict, element statuses and
  findings with line and column, or an `error` for a file that could not be
  scanned) and a `summary`.

```bash
cargo run -p rigscan-cli -- scan --output-format report robot.urdf
# robot.urdf: 1 healthy, 0 warnings, 1 critical
#   healthy  link  base
#   critical joint j1
#     robot.urdf:3:3: critical RS001: joint `j1` child link `ghost` is not defined
```

As a library:

```rust
use rigscan_core::{lint, parse_urdf};

let robot = parse_urdf(&std::fs::read_to_string("robot.urdf")?)?;
for f in lint(&robot) {
    println!("{} line {}: {}", f.rule, f.span.line, f.message);
}
```

## Planned

- `rigscan scan robot.urdf` with a per-link and per-joint health report, and
  JSON and SARIF output.
- MJCF parsing, and URDF to MJCF and MJCF to URDF conversion with a report of
  dropped features.
- Around 25 lint rules covering structure (missing links, loops, duplicate
  names, unresolved mesh paths) and physics sanity (mass, inertia, joint limits).
- Python wheel and a browser playground.
- Later: a USD (`.usda`, UsdPhysics) writer and xacro support.

## Development

```bash
cargo test
cargo build -p rigscan-core --target wasm32-unknown-unknown
```

Bug fixes start with a failing test.

End-to-end tests run on 69 real robots (Unitree, Franka Panda, SO-101 and more)
fetched from [example-robot-data](https://github.com/Gepetto/example-robot-data)
into the gitignored `corpus/` directory. They lint every file, report
parse and lint speed, then corrupt the files with a seeded generator (8 failure
modes, equal counts) and check that every corruption is detected:

```bash
scripts/fetch-corpus.sh
cargo test --release -p rigscan-core --test e2e_corpus -- --nocapture
```

Without the corpus the tests skip and say so; `RIGSCAN_REQUIRE_CORPUS=1` makes
that a failure. `RIGSCAN_SEED=<n>` reruns the corruptions with another seed.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) and the [style guide](STYLE_GUIDE.md).

## License

Apache-2.0. See [LICENSE](LICENSE).
