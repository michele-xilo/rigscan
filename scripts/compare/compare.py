#!/usr/bin/env python3
"""Run popular URDF tools next to Rigscan on the same inputs.

    uv run --project scripts/compare python scripts/compare/compare.py          # refresh goldens.json, the coverage table and the README summary
    uv run --project scripts/compare python scripts/compare/compare.py --bench  # also re-measure speed (run the corpus bench first)
    python3 scripts/compare/compare.py --check                                  # CI: no tools needed

Needs `cargo build --release -p rigscan-cli` and, for --bench, the corpus (see README).
"""
import argparse
import json
import os
import platform
import re
import subprocess
import sys
import tempfile
import time
from importlib import metadata
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
PROBES = HERE / "probes"
GOLDENS = HERE / "goldens.json"
README = ROOT / "README.md"
COMPARISON = ROOT / "docs/comparison.md"
RIGSCAN = ROOT / "target/release/rigscan"
ROBOTS = ROOT / "corpus/example-robot-data/robots"
RIGS = ["double_pendulum", "ur5_robot", "panda", "go2", "talos_full_v2"]
TOOLS = ["mujoco", "pinocchio", "yourdfpy"]
# MuJoCo compiles meshes and cannot resolve package:// paths, so it has no comparable load time on corpus rigs.
SPEED_TOOLS = ["pinocchio", "yourdfpy"]
PACKAGES = {"mujoco": "mujoco", "pinocchio": "pin", "yourdfpy": "yourdfpy"}
REPS = 20
# Which probe formats each tool can load; a probe it cannot read is marked n/a. None of the three reads USD.
READS = {"mujoco": {".urdf", ".xml"}, "pinocchio": {".urdf"}, "yourdfpy": {".urdf"}}
# Written as escapes so the source holds no emoji: tick, cross, a yellow circle, a minus and a trophy.
TICK, CROSS, PARTLY, NA, TROPHY = "\u2705", "\u274c", "\U0001f7e1", "\u2796", "\U0001f3c6"
SYMBOL = {"yes": TICK, "no": CROSS, "partly": PARTLY, "-": NA}
LEGEND = f"{TICK} flagged  {PARTLY} flagged on some probes  {CROSS} not flagged  {NA} the tool cannot read that format"

# One short title per rule; --check fails when a rule in core has no title or no probe.
TITLES = {
    "RS001": "Joint references an undefined link",
    "RS002": "Duplicate link name",
    "RS003": "Duplicate joint name",
    "RS004": "Link with two parent joints",
    "RS005": "Kinematic loop",
    "RS006": "More than one root link",
    "RS007": "Robot has no links",
    "RS008": "Negative mass",
    "RS009": "Zero mass on a moving link",
    "RS010": "Negative principal moment",
    "RS011": "Triangle inequality of moments",
    "RS012": "Joint lower limit above upper",
    "RS013": "Joint limit missing or incomplete",
    "RS014": "Mesh file not found",
    "RS015": "Mimic of an undefined joint",
    "RS016": "MJCF: unknown default class",
    "RS017": "MJCF: mesh geom names an undefined asset",
    "RS018": "MJCF: include file not found",
    "RS019": "USD: joint connects a link to itself",
    "RS020": "USD: rigid body with no mass and no collider",
    "RS021": "USD: joints without an articulation root",
    "RS022": "USD: layer could not be composed",
    "parse": "Malformed input (bad number, truncated XML)",
}


def check_key(probe):
    return probe.split("_")[0] if probe.startswith("RS") else ("parse" if probe.startswith("parse") else None)


def markers(name):
    return f"<!-- compare:{name}:start -->", f"<!-- compare:{name}:end -->"


# ---- tools -----------------------------------------------------------------


def load_mujoco(path):
    import mujoco

    mujoco.MjModel.from_xml_path(path)


def load_pinocchio(path):
    import pinocchio

    pinocchio.buildModelFromUrdf(path)


def load_yourdfpy(path):
    import yourdfpy

    urdf = yourdfpy.URDF.load(
        path,
        build_scene_graph=False,
        load_meshes=False,
        build_collision_scene_graph=False,
        load_collision_meshes=False,
    )
    # load() only parses; validate() is the tool's own health check.
    if not urdf.validate():
        raise ValueError("; ".join(str(e) for e in urdf.errors))


LOADERS = {"mujoco": load_mujoco, "pinocchio": load_pinocchio, "yourdfpy": load_yourdfpy}


def first_line(text, prefer="rror"):
    lines = [l.strip() for l in text.splitlines() if l.strip()]
    pick = next((l for l in lines if prefer in l), lines[0] if lines else "")
    return pick[:120]


def attempt(tool, path):
    """Returns (outcome, message, seconds). `logged` = accepted but wrote to stderr.

    stderr is captured at the file descriptor because pinocchio's urdfdom logs from C++.
    """
    with tempfile.TemporaryFile("w+") as cap:
        sys.stderr.flush()
        saved = os.dup(2)
        os.dup2(cap.fileno(), 2)
        start = time.perf_counter()
        try:
            LOADERS[tool](str(path))
            outcome, message = "ok", ""
        except Exception as e:  # any failure is the tool rejecting the file
            outcome, message = "reject", str(e).replace(str(path), Path(path).name) or type(e).__name__
        finally:
            seconds = time.perf_counter() - start
            sys.stderr.flush()
            os.dup2(saved, 2)
            os.close(saved)
        cap.seek(0)
        logged = cap.read()
    if outcome == "ok" and logged.strip():
        outcome, message = "logged", logged.replace(str(path), Path(path).name)
    return outcome, first_line(message), seconds


def best_of(fn, reps=REPS):
    return min(fn() for _ in range(reps))


def rigscan_run(path):
    """Returns (sorted rule ids or ['error'], seconds) for one file."""
    start = time.perf_counter()
    p = subprocess.run(
        [RIGSCAN, "scan", "--output-format", "json", "--quiet", str(path)],
        capture_output=True,
        text=True,
    )
    seconds = time.perf_counter() - start
    if p.returncode == 2:
        return ["error"], seconds
    doc = json.loads(p.stdout)
    return sorted({f["rule"] for file in doc["files"] for f in file.get("findings", [])}), seconds


# ---- goldens ---------------------------------------------------------------


def probes():
    return sorted(p for ext in (".urdf", ".xml", ".usda") for p in PROBES.glob(f"*{ext}"))


def build_goldens():
    out = {
        "versions": {t: metadata.version(PACKAGES[t]) for t in TOOLS},
        "rigscan": subprocess.run([RIGSCAN, "--version"], capture_output=True, text=True).stdout.strip(),
        "probes": {},
    }
    for path in probes():
        rules, _ = rigscan_run(path)
        entry = {"rigscan": rules}
        for tool in TOOLS:
            if path.suffix not in READS[tool]:
                entry[tool] = {"outcome": "n/a", "message": ""}
                continue
            outcome, message, _ = attempt(tool, path)
            entry[tool] = {"outcome": outcome, "message": message}
        out["probes"][path.stem] = entry
    return out


def coverage_cells(goldens):
    """Per check, whether each of Rigscan and the other tools flags at least one of its probes."""
    checks = {}
    for probe, entry in goldens["probes"].items():
        key = check_key(probe)
        if key:
            checks.setdefault(key, []).append(entry)
    cells = {}
    for key in sorted(checks, key=lambda k: (k == "parse", k)):
        entries = checks[key]
        row = ["yes" if all(key in e["rigscan"] or (key == "parse" and e["rigscan"] == ["error"]) for e in entries) else "no"]
        for tool in TOOLS:
            if all(e[tool]["outcome"] == "n/a" for e in entries):
                row.append("-")
                continue
            flagged = [e[tool]["outcome"] not in ("ok", "n/a") for e in entries]
            row.append("yes" if all(flagged) else "partly" if any(flagged) else "no")
        cells[key] = row
    return cells


def coverage_table(goldens):
    v = goldens["versions"]
    head = f"| Check | Rigscan | MuJoCo {v['mujoco']} | Pinocchio {v['pinocchio']} | yourdfpy {v['yourdfpy']} |"
    rows = [head, "|---|:-:|:-:|:-:|:-:|"]
    for key, row in coverage_cells(goldens).items():
        label = f"`{key}` {TITLES[key]}" if key != "parse" else TITLES[key]
        rows.append(f"| {label} | " + " | ".join(SYMBOL[c] for c in row) + " |")
    return "\n".join(rows) + f"\n\n{LEGEND}"


def splice(text, name, body, where="README.md"):
    start, end = markers(name)
    pattern = re.compile(re.escape(start) + r".*?" + re.escape(end), re.S)
    if not pattern.search(text):
        sys.exit(f"{where} is missing the {start} ... {end} markers")
    return pattern.sub(lambda _: f"{start}\n{body}\n{end}", text)


# ---- speed -----------------------------------------------------------------


def ms(seconds):
    return f"{seconds * 1000:.1f}"


def cpu():
    out = subprocess.run(["sysctl", "-n", "machdep.cpu.brand_string"], capture_output=True, text=True).stdout.strip()
    return out or platform.processor() or platform.machine()


def corpus_commit():
    src = ROBOTS.parent / "SOURCE"
    return src.read_text().strip().splitlines()[-1] if src.exists() else "unknown"


def criterion_ms(rig):
    """Median parse plus lint time from `cargo bench -p rigscan-core --bench core -- corpus`."""
    est = ROOT / "target/criterion/corpus" / rig / "new/estimates.json"
    if not est.exists():
        sys.exit(f"missing {est}; run: cargo bench -p rigscan-core --bench core -- corpus")
    return json.loads(est.read_text())["median"]["point_estimate"] / 1e6


def measure_speed(goldens):
    """Load time per rig for Rigscan and the tools that can read corpus rigs, with the note that explains how."""
    rigs = []
    for rig in RIGS:
        path = next(ROBOTS.rglob(f"{rig}.urdf"))
        entry = {"rig": rig, "kib": round(path.stat().st_size / 1024), "rigscan": criterion_ms(rig)}
        for tool in SPEED_TOOLS:
            outcome, _, _ = attempt(tool, path)
            entry[tool] = {"outcome": outcome}
            if outcome == "ok":
                entry[tool]["ms"] = best_of(lambda: attempt(tool, path)[2]) * 1000
        rigs.append(entry)
    v = goldens["versions"]
    commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    note = (
        f"Milliseconds to load one file with the process already running. Rigscan: criterion median of "
        f"parse plus lint (`cargo bench -p rigscan-core --bench core -- corpus`). Python tools: best of {REPS} "
        "in-process calls, imports excluded; yourdfpy time includes its validate(). "
        f"Corpus: Gepetto/example-robot-data `{corpus_commit()[:12]}`. "
        f"Rigscan commit {commit}, Pinocchio {v['pinocchio']}, yourdfpy {v['yourdfpy']}, "
        f"Python {platform.python_version()}, {cpu()}, {platform.system()} {platform.release()}. "
        "A tool that rejects or logs an error on a rig shows that outcome instead of a time. "
        "MuJoCo is left out: it loads meshes and cannot resolve `package://` paths."
    )
    return {"rigs": rigs, "note": note}


def times(entry):
    """Millisecond load time per tool name, for the tools that produced one."""
    out = {"rigscan": entry["rigscan"]}
    out.update({t: entry[t]["ms"] for t in SPEED_TOOLS if "ms" in entry[t]})
    return out


def speed_table(goldens):
    speed = goldens["speed"]
    rows = ["| Rig | Size | Rigscan | Pinocchio | yourdfpy |", "|---|--:|--:|--:|--:|"]
    for entry in speed["rigs"]:
        t = times(entry)
        best = min(t.values())
        cells = []
        for tool in ["rigscan", *SPEED_TOOLS]:
            if tool == "rigscan" or "ms" in entry[tool]:
                trophy = f" {TROPHY}" if t[tool] == best else ""
                cells.append(f"{t[tool]:.3f}{trophy}")
            else:
                cells.append(f"{CROSS} {entry[tool]['outcome']}")
        rows.append(f"| `{entry['rig']}` | {entry['kib']} KiB | " + " | ".join(cells) + " |")
    return "\n".join(rows) + f"\n\n{speed['note']}"


def summary_table(goldens):
    """The short table for the README: formats read, checks caught and load time on the largest rig."""
    v = goldens["versions"]
    cells = coverage_cells(goldens)
    rules = [k for k in cells if k != "parse"]
    caught = [sum(cells[k][i] == "yes" for k in rules) for i in range(4)]
    big = goldens["speed"]["rigs"][-1]
    t = times(big)
    best = max(caught)
    head = f"| | Rigscan | MuJoCo {v['mujoco']} | Pinocchio {v['pinocchio']} | yourdfpy {v['yourdfpy']} |"
    rows = [head, "|---|:-:|:-:|:-:|:-:|"]
    for label, suffixes in [("Reads URDF", {".urdf"}), ("Reads MJCF", {".xml"}), ("Reads USD (UsdPhysics)", {".usda"})]:
        rows.append(f"| {label} | {TICK} | " + " | ".join(TICK if suffixes <= READS[tool] else CROSS for tool in TOOLS) + " |")
    caught_cells = [f"{n} of {len(rules)}" + (f" {TROPHY}" if n == best else "") for n in caught]
    rows.append("| Broken-robot checks flagged | " + " | ".join(caught_cells) + " |")
    load = [f"{t['rigscan']:.2f} ms {TROPHY}", NA] + [f"{t[x]:.2f} ms" if x in t else CROSS for x in SPEED_TOOLS]
    rows.append(f"| Load time, `{big['rig']}` ({big['kib']} KiB) | " + " | ".join(load) + " |")
    slower = ["1x", NA] + [f"{t[x] / t['rigscan']:.0f}x slower" if x in t else NA for x in SPEED_TOOLS]
    rows.append("| Time compared with Rigscan | " + " | ".join(slower) + " |")
    legend = f"{TICK} yes  {CROSS} no  {TROPHY} best  {NA} not measured. The tools are loaders, not linters; see [docs/comparison.md](docs/comparison.md) for each check and each rig."
    return "\n".join(rows) + f"\n\n{legend}"


# ---- modes -----------------------------------------------------------------


def check():
    problems = []
    goldens = json.loads(GOLDENS.read_text())
    names = {p.stem for p in probes()}
    if names != set(goldens["probes"]):
        problems.append("probes/ and goldens.json disagree; rerun compare.py")
    rules = set()
    for src in (ROOT / "crates/rigscan-core/src").glob("*.rs"):
        rules |= set(re.findall(r'"(RS\d{3})"', src.read_text()))
    for rule in sorted(rules):
        if not any(n.startswith(rule + "_") for n in names):
            problems.append(f"{rule} has no probe in scripts/compare/probes/")
        if rule not in TITLES:
            problems.append(f"{rule} has no entry in TITLES")
    for n in sorted(names):
        entry = goldens["probes"].get(n, {})
        key = check_key(n)
        if key is None and n != "healthy":
            problems.append(f"probe {n} is not named RSnnn_*, parse_* or healthy")
        if key and key.startswith("RS") and key not in entry.get("rigscan", []):
            problems.append(f"Rigscan golden for {n} does not include {key}")
    if "speed" not in goldens:
        problems.append("goldens.json has no speed measurements; rerun compare.py --bench")
    else:
        text = COMPARISON.read_text()
        stale = splice(splice(text, "coverage", coverage_table(goldens), "docs/comparison.md"), "speed", speed_table(goldens), "docs/comparison.md")
        if stale != text:
            problems.append("docs/comparison.md tables are stale; rerun compare.py")
        text = README.read_text()
        if splice(text, "summary", summary_table(goldens)) != text:
            problems.append("README summary table is stale; rerun compare.py")
    for p in problems:
        print(p, file=sys.stderr)
    return 1 if problems else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="verify only; needs no tools")
    ap.add_argument("--bench", action="store_true", help="also refresh the speed tables")
    args = ap.parse_args()
    if args.check:
        sys.exit(check())
    os.chdir(tempfile.gettempdir())  # MuJoCo writes MUJOCO_LOG.TXT into the working directory
    if not RIGSCAN.exists():
        sys.exit("build the CLI first: cargo build --release -p rigscan-cli")
    previous = json.loads(GOLDENS.read_text()) if GOLDENS.exists() else {}
    goldens = build_goldens()
    if args.bench:
        goldens["speed"] = measure_speed(goldens)
    elif "speed" in previous:
        goldens["speed"] = previous["speed"]
    else:
        sys.exit("goldens.json has no speed measurements yet; run with --bench")
    GOLDENS.write_text(json.dumps(goldens, indent=2, sort_keys=True) + "\n")
    text = splice(COMPARISON.read_text(), "coverage", coverage_table(goldens), "docs/comparison.md")
    COMPARISON.write_text(splice(text, "speed", speed_table(goldens), "docs/comparison.md"))
    README.write_text(splice(README.read_text(), "summary", summary_table(goldens)))


if __name__ == "__main__":
    main()
