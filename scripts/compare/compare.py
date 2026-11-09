#!/usr/bin/env python3
"""Run popular URDF tools next to Rigscan on the same inputs.

    uv run --project scripts/compare python scripts/compare/compare.py          # refresh goldens.json and the README coverage table
    uv run --project scripts/compare python scripts/compare/compare.py --bench  # also refresh the README speed tables
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
RIGSCAN = ROOT / "target/release/rigscan"
ROBOTS = ROOT / "corpus/example-robot-data/robots"
RIGS = ["double_pendulum", "ur5_robot", "panda", "go2", "talos_full_v2"]
TOOLS = ["mujoco", "pinocchio", "yourdfpy"]
PACKAGES = {"mujoco": "mujoco", "pinocchio": "pin", "yourdfpy": "yourdfpy"}
REPS = 20

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
    return sorted(PROBES.glob("*.urdf"))


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
            outcome, message, _ = attempt(tool, path)
            entry[tool] = {"outcome": outcome, "message": message}
        out["probes"][path.stem] = entry
    return out


def coverage_table(goldens):
    """Per check, whether each tool flags at least one of its probes."""
    checks = {}
    for probe, entry in goldens["probes"].items():
        key = check_key(probe)
        if key:
            checks.setdefault(key, []).append(entry)
    v = goldens["versions"]
    head = f"| Check | Rigscan | MuJoCo {v['mujoco']} | Pinocchio {v['pinocchio']} | yourdfpy {v['yourdfpy']} |"
    rows = [head, "|---|:-:|:-:|:-:|:-:|"]
    for key in sorted(checks, key=lambda k: (k == "parse", k)):
        entries = checks[key]
        cells = [
            "yes" if all(key in e["rigscan"] or (key == "parse" and e["rigscan"] == ["error"]) for e in entries) else "no"
        ]
        for tool in TOOLS:
            flagged = [e[tool]["outcome"] != "ok" for e in entries]
            cells.append("yes" if all(flagged) else "partly" if any(flagged) else "no")
        label = f"`{key}` {TITLES[key]}" if key != "parse" else TITLES[key]
        rows.append(f"| {label} | " + " | ".join(cells) + " |")
    return "\n".join(rows)


def splice(text, name, body):
    start, end = markers(name)
    pattern = re.compile(re.escape(start) + r".*?" + re.escape(end), re.S)
    if not pattern.search(text):
        sys.exit(f"README.md is missing the {start} ... {end} markers")
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


def speed_tables(goldens):
    rig_paths = [next(ROBOTS.rglob(f"{r}.urdf")) for r in RIGS]
    header = "| Rig | Size | Rigscan CLI | MuJoCo | Pinocchio | yourdfpy |\n|---|--:|--:|--:|--:|--:|"
    rows = []
    for path in rig_paths:
        cells = [f"{best_of(lambda: rigscan_run(path)[1], 5) * 1000:.1f}"]
        for tool in TOOLS:
            outcome, _, _ = attempt(tool, path)
            cells.append(ms(best_of(lambda: attempt(tool, path)[2], 5)) if outcome == "ok" else outcome)
        rows.append(f"| `{path.stem}` | {path.stat().st_size / 1024:.0f} KiB | " + " | ".join(cells) + " |")
    whole = "\n".join([header, *rows])

    header = "| Probe | Rigscan CLI | MuJoCo | Pinocchio | yourdfpy |\n|---|--:|--:|--:|--:|"
    rows = []
    for path in probes():
        cells = [ms(best_of(lambda: rigscan_run(path)[1]))]
        for tool in TOOLS:
            cells.append(ms(best_of(lambda: attempt(tool, path)[2])))
        rows.append(f"| `{path.stem}` | " + " | ".join(cells) + " |")
    per_probe = "\n".join([header, *rows])

    v = goldens["versions"]
    note = (
        f"Milliseconds, best of {REPS} (rigs: best of 5), warm. "
        f"Corpus: Gepetto/example-robot-data `{corpus_commit()[:12]}`. "
        f"Rigscan `{goldens['rigscan']}`, MuJoCo {v['mujoco']}, Pinocchio {v['pinocchio']}, yourdfpy {v['yourdfpy']}, "
        f"Python {platform.python_version()}, {cpu()}, {platform.system()} {platform.release()}. "
        "Rigscan is timed as a whole process (startup included); the Python tools are timed in-process "
        "(interpreter startup and imports excluded), so the comparison favours them. "
        "A tool that rejects or logs an error on a rig is shown by that outcome instead of a time; "
        "MuJoCo cannot resolve `package://` mesh paths."
    )
    return whole, per_probe, note


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
    start, end = markers("coverage")
    text = README.read_text()
    if splice(text, "coverage", coverage_table(goldens)) != text:
        problems.append("README coverage table is stale; rerun compare.py")
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
    goldens = build_goldens()
    GOLDENS.write_text(json.dumps(goldens, indent=2, sort_keys=True) + "\n")
    text = splice(README.read_text(), "coverage", coverage_table(goldens))
    if args.bench:
        whole, per_probe, note = speed_tables(goldens)
        text = splice(text, "speed", f"{whole}\n\n{note}")
        text = splice(text, "speed-probes", f"{per_probe}\n\n{note}")
    README.write_text(text)


if __name__ == "__main__":
    main()
