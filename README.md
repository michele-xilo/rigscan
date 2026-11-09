# Rigscan

[![build](https://github.com/michele-xilo/rigscan/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/michele-xilo/rigscan/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/michele-xilo/rigscan/branch/main/graph/badge.svg)](https://codecov.io/gh/michele-xilo/rigscan)

A health check for robot descriptions.

Rigscan scans robot description files, reports what is wrong with them, and
(planned) converts between formats while telling you what each conversion lost.
The core is written in Rust and is designed to run as a CLI, in the browser via
WebAssembly, and from Python.

> **Status: pre-alpha.** URDF parsing, thirteen lint rules and a `scan`
> command exist. There is no MJCF or USD support, no conversion, and nothing is
> published yet. The API will change.

## What works today

- Parse URDF links and joints, keeping the line and column of each element,
  along with joint types, origins, limits and mimic, link inertials, and the
  mesh files a link references. Invalid numbers are reported with their
  line and column.
- Lint rules (critical unless noted):
  - `RS001` a joint references a parent or child link that is not defined, or has none
  - `RS002` duplicate link names
  - `RS003` duplicate joint names
  - `RS004` a link with more than one parent joint
  - `RS005` a kinematic loop
  - `RS006` more than one root link
  - `RS007` a robot with no links
  - `RS008` negative link mass
  - `RS009` zero mass on a link moved by a joint (warning; massless frames on
    fixed joints are fine)
  - `RS010` negative principal moment of inertia
  - `RS011` principal moments break the triangle inequality (warning)
  - `RS012` joint `lower` limit above `upper`
  - `RS013` revolute or prismatic joint without a limit, or a limit missing
    `effort` or `velocity`
- `rigscan-core` compiles for `wasm32-unknown-unknown`.

```bash
cargo run -p rigscan-cli -- scan robot.urdf
# robot.urdf:5:5: critical RS001: joint `j1` child link `ghost` is not defined
```

`rigscan scan` takes files and directories (default: the current directory).
Directories are searched recursively for supported files (`.urdf` for now),
skipping anything in `.gitignore`; a file named on the command line is always
scanned. Files are scanned in parallel and the output is sorted, so it is the
same on every run. Findings go to stdout and a summary (`Found 3 findings in
2 files.` or `All checks passed!`) to stderr. It exits 0 when clean, 1 when there
are critical findings, and 2 when a file cannot be scanned.

`--output-format` picks the output:

- `text` (default): one line per finding, as above.
- `full`: each finding with its source line and a caret under the column.
- `report`: a verdict line per file, then the status of every link and joint
  with its findings underneath. There is no numeric score.
- `json`: one document with `version`, `files` (verdict, element statuses and
  findings with line and column, or an `error` for a file that could not be
  scanned) and a `summary`.

Other options: `--statistics` counts findings per rule, `-q`/`--quiet` prints
only the summary and only when something is wrong, and `--color
auto|always|never` controls colour (teal healthy, amber warning, red critical;
`auto` honours `NO_COLOR`).

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

## Comparison with other tools

Each check has a minimal probe URDF in `scripts/compare/probes/`. The same probes are fed to Rigscan and to
three popular tools that read URDF: MuJoCo, Pinocchio (which uses urdfdom) and yourdfpy. A tool
"flags" a probe when it rejects the file or logs an error while loading it. These tools are loaders, not
linters, so a `no` means the tool accepts that kind of broken robot, not that it is worse at its job. Checks
a tool flags that Rigscan does not are the target for new rules.

<!-- compare:coverage:start -->
| Check | Rigscan | MuJoCo 4.0.0 | Pinocchio 4.1.0 | yourdfpy 0.0.60 |
|---|:-:|:-:|:-:|:-:|
| `RS001` Joint references an undefined link | yes | yes | yes | no |
| `RS002` Duplicate link name | yes | yes | yes | no |
| `RS003` Duplicate joint name | yes | no | yes | no |
| `RS004` Link with two parent joints | yes | yes | yes | no |
| `RS005` Kinematic loop | yes | yes | yes | no |
| `RS006` More than one root link | yes | no | yes | no |
| `RS007` Robot has no links | yes | no | yes | no |
| `RS008` Negative mass | yes | yes | no | no |
| `RS009` Zero mass on a moving link | yes | yes | no | no |
| `RS010` Negative principal moment | yes | yes | no | no |
| `RS011` Triangle inequality of moments | yes | yes | no | no |
| `RS012` Joint lower limit above upper | yes | no | no | no |
| `RS013` Joint limit missing or incomplete | yes | no | yes | yes |
| Malformed input (bad number, truncated XML) | yes | yes | yes | yes |
<!-- compare:coverage:end -->

Whole-file speed on rigs from the corpus:

<!-- compare:speed:start -->
| Rig | Size | Rigscan CLI | MuJoCo | Pinocchio | yourdfpy |
|---|--:|--:|--:|--:|--:|
| `double_pendulum` | 4 KiB | 4.1 | reject | 0.1 | 0.3 |
| `ur5_robot` | 12 KiB | 4.4 | reject | 0.2 | 0.6 |
| `panda` | 15 KiB | 4.6 | reject | 0.3 | 0.8 |
| `go2` | 24 KiB | 4.7 | reject | 0.5 | 1.6 |
| `talos_full_v2` | 107 KiB | 5.3 | reject | 1.7 | 4.8 |

Milliseconds, best of 20 (rigs: best of 5), warm. Corpus: Gepetto/example-robot-data `78b6101a6387`. Rigscan `rigscan-cli 0.0.1`, MuJoCo 4.0.0, Pinocchio 4.1.0, yourdfpy 0.0.60, Python 3.12.7, Apple M2 Pro, Darwin 25.5.0. Rigscan is timed as a whole process (startup included); the Python tools are timed in-process (interpreter startup and imports excluded), so the comparison favours them. A tool that rejects or logs an error on a rig is shown by that outcome instead of a time; MuJoCo cannot resolve `package://` mesh paths.
<!-- compare:speed:end -->

<details><summary>Time to verdict per probe</summary>

<!-- compare:speed-probes:start -->
| Probe | Rigscan CLI | MuJoCo | Pinocchio | yourdfpy |
|---|--:|--:|--:|--:|
| `RS001_undefined_child` | 4.6 | 0.0 | 0.1 | 0.1 |
| `RS002_duplicate_link` | 4.2 | 0.0 | 0.1 | 0.1 |
| `RS003_duplicate_joint` | 4.6 | 0.2 | 0.1 | 0.1 |
| `RS004_two_parents` | 4.3 | 0.0 | 0.1 | 0.1 |
| `RS005_loop` | 4.6 | 0.0 | 0.1 | 0.1 |
| `RS006_two_roots` | 4.2 | 0.2 | 0.1 | 0.1 |
| `RS007_no_links` | 4.5 | 0.2 | 0.1 | 0.0 |
| `RS008_negative_mass` | 3.8 | 0.1 | 0.0 | 0.1 |
| `RS009_zero_mass_moving` | 4.1 | 0.1 | 0.0 | 0.1 |
| `RS010_negative_inertia` | 4.2 | 0.1 | 0.0 | 0.1 |
| `RS011_triangle` | 4.0 | 0.1 | 0.0 | 0.1 |
| `RS012_lower_gt_upper` | 4.1 | 0.2 | 0.0 | 0.1 |
| `RS013_limit_no_effort` | 4.4 | 0.2 | 0.2 | 0.1 |
| `RS013_revolute_no_limit` | 4.1 | 0.2 | 0.2 | 0.1 |
| `healthy` | 4.6 | 0.2 | 0.0 | 0.1 |
| `parse_invalid_number` | 4.6 | 0.1 | 0.1 | 0.1 |
| `parse_truncated` | 4.4 | 0.0 | 0.1 | 0.2 |

Milliseconds, best of 20 (rigs: best of 5), warm. Corpus: Gepetto/example-robot-data `78b6101a6387`. Rigscan `rigscan-cli 0.0.1`, MuJoCo 4.0.0, Pinocchio 4.1.0, yourdfpy 0.0.60, Python 3.12.7, Apple M2 Pro, Darwin 25.5.0. Rigscan is timed as a whole process (startup included); the Python tools are timed in-process (interpreter startup and imports excluded), so the comparison favours them. A tool that rejects or logs an error on a rig is shown by that outcome instead of a time; MuJoCo cannot resolve `package://` mesh paths.
<!-- compare:speed-probes:end -->

</details>

The outputs behind these tables are committed in `scripts/compare/goldens.json`. Adding a rule means adding
its probe and rerunning the script (`scripts/compare/compare.py`); CI runs `compare.py --check`.

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
