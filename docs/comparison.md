# Comparison with other tools

Each check has a minimal probe file in `scripts/compare/probes/`: a URDF, an MJCF `.xml` for the MJCF rules, or a
`.usda` for the USD rules. The same probes are fed to Rigscan and to three popular tools that read URDF: MuJoCo,
Pinocchio (which uses urdfdom) and yourdfpy. Only MuJoCo reads MJCF, and none of the three reads USD, so they show
the minus sign for those checks. A tool "flags" a probe when it rejects the file or logs an error while loading it.

These tools are loaders, not linters, so a cross means the tool accepts that kind of broken robot, not that it is
worse at its job. A check a tool flags that Rigscan does not is the target for a new rule.

## Which broken robots each tool catches

<!-- compare:coverage:start -->
| Check | Rigscan | MuJoCo 4.0.0 | Pinocchio 4.1.0 | yourdfpy 0.0.60 |
|---|:-:|:-:|:-:|:-:|
| `RS001` Joint references an undefined link | ✅ | ✅ | ✅ | ❌ |
| `RS002` Duplicate link name | ✅ | ✅ | ✅ | ❌ |
| `RS003` Duplicate joint name | ✅ | ❌ | ✅ | ❌ |
| `RS004` Link with two parent joints | ✅ | ✅ | ✅ | ❌ |
| `RS005` Kinematic loop | ✅ | ✅ | ✅ | ❌ |
| `RS006` More than one root link | ✅ | ❌ | ✅ | ❌ |
| `RS007` Robot has no links | ✅ | ❌ | ✅ | ❌ |
| `RS008` Negative mass | ✅ | ✅ | ❌ | ❌ |
| `RS009` Zero mass on a moving link | ✅ | ✅ | ❌ | ❌ |
| `RS010` Negative principal moment | ✅ | ✅ | ❌ | ❌ |
| `RS011` Triangle inequality of moments | ✅ | ✅ | ❌ | ❌ |
| `RS012` Joint lower limit above upper | ✅ | ❌ | ❌ | ❌ |
| `RS013` Joint limit missing or incomplete | ✅ | ❌ | ✅ | ✅ |
| `RS014` Mesh file not found | ✅ | ❌ | ❌ | ❌ |
| `RS015` Mimic of an undefined joint | ✅ | ✅ | ❌ | ❌ |
| `RS016` MJCF: unknown default class | ✅ | ✅ | ➖ | ➖ |
| `RS017` MJCF: mesh geom names an undefined asset | ✅ | ✅ | ➖ | ➖ |
| `RS018` MJCF: include file not found | ✅ | ✅ | ➖ | ➖ |
| `RS019` USD: joint connects a link to itself | ✅ | ➖ | ➖ | ➖ |
| `RS020` USD: rigid body with no mass and no collider | ✅ | ➖ | ➖ | ➖ |
| `RS021` USD: joints without an articulation root | ✅ | ➖ | ➖ | ➖ |
| `RS022` USD: layer could not be composed | ✅ | ➖ | ➖ | ➖ |
| Malformed input (bad number, truncated XML) | ✅ | ✅ | ✅ | ✅ |

✅ flagged  🟡 flagged on some probes  ❌ not flagged  ➖ the tool cannot read that format
<!-- compare:coverage:end -->

## How fast each tool loads a robot

<!-- compare:speed:start -->
| Rig | Size | Rigscan | Pinocchio | yourdfpy |
|---|--:|--:|--:|--:|
| `double_pendulum` | 4 KiB | 0.015 🏆 | 0.079 | 0.233 |
| `ur5_robot` | 12 KiB | 0.047 🏆 | 0.187 | 0.538 |
| `panda` | 15 KiB | 0.063 🏆 | 0.264 | 0.779 |
| `go2` | 24 KiB | 0.128 🏆 | 0.449 | 1.491 |
| `talos_full_v2` | 107 KiB | 0.488 🏆 | 1.622 | 4.285 |

Milliseconds to load one file with the process already running. Rigscan: criterion median of parse plus lint (`cargo bench -p rigscan-core --bench core -- corpus`). Python tools: best of 20 in-process calls, imports excluded; yourdfpy time includes its validate(). Corpus: Gepetto/example-robot-data `78b6101a6387`. Rigscan commit 59d878f, Pinocchio 4.1.0, yourdfpy 0.0.60, Python 3.12.7, Apple M2 Pro, Darwin 25.5.0. A tool that rejects or logs an error on a rig shows that outcome instead of a time. MuJoCo is left out: it loads meshes and cannot resolve `package://` paths.
<!-- compare:speed:end -->

The trophy marks the fastest tool on that rig.

## Reproducing

The outputs behind these tables are committed in `scripts/compare/goldens.json`. Adding a rule means adding its
probe and rerunning the script; CI runs `compare.py --check` and fails when a table or a probe is stale.

```bash
cargo build --release -p rigscan-cli
scripts/fetch-corpus.sh
cargo bench -p rigscan-core --bench core -- corpus
uv run --project scripts/compare python scripts/compare/compare.py --bench
```

Without `--bench` the script refreshes the check coverage and reuses the stored speed measurements.
