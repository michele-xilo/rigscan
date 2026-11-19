#!/usr/bin/env python3
"""Loads every converted corpus file in MuJoCo and fails on anything but a known cause.

URDF corpus -> MJCF, MuJoCo Menagerie -> URDF and the Newton USD assets -> MJCF and URDF, through
the built `rigscan` binary. The corpora hold no mesh files, so a load that fails only because a file cannot be opened is expected, as is an
input the converter refuses for its critical findings. Every other failure is a bug in a writer.

    python3 scripts/verify/mujoco_load.py [--require] [--bin target/debug/rigscan]

Without --require the check is skipped, with a reason, when `mujoco` or a corpus is missing.
"""
import argparse
import collections
import glob
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUNS = [
    ("example-robot-data", "corpus/example-robot-data/**/*.urdf", "mjcf", ".xml"),
    ("mujoco_menagerie", "corpus/mujoco_menagerie/*/*.xml", "urdf", ".urdf"),
    ("newton_assets", "corpus/newton_assets/**/*.usd*", "mjcf", ".xml"),
    ("newton_assets", "corpus/newton_assets/**/*.usd*", "urdf", ".urdf"),
]
# MuJoCo messages that are the source's doing, not a writer's: a referenced file is not on disk
# (the corpora hold no meshes), or a moving body has no inertia because MuJoCo derives it from
# geoms or from `attach`, which Rigscan does not read. The converted model is faithful to what
# Rigscan read, and the missing data is reported on stderr.
EXPECTED = ("Error opening file", "mjMINVAL")


def skip(reason, require):
    print(f"{'error' if require else 'skipped'}: {reason}")
    return 1 if require else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--require", action="store_true")
    ap.add_argument("--bin", default=str(ROOT / "target/debug/rigscan"))
    args = ap.parse_args()
    try:
        import mujoco
    except ImportError:
        return skip("the mujoco package is not installed", args.require)

    unexpected = collections.defaultdict(list)
    with tempfile.TemporaryDirectory() as tmp:
        for name, pattern, target, suffix in RUNS:
            files = sorted(glob.glob(str(ROOT / pattern), recursive=True))
            if not files:
                return skip(f"corpus/{name} is not present", args.require)
            out = str(Path(tmp) / f"model{suffix}")
            counts = collections.Counter()
            for f in files:
                r = subprocess.run(
                    [args.bin, "convert", f, "--to", target, "-o", out],
                    capture_output=True, text=True,
                )
                if r.returncode == 2:
                    if "not converted" in r.stderr:
                        counts["refused by the converter"] += 1
                    else:
                        counts["unreadable input"] += 1
                        unexpected[f"{name}: unreadable input"].append(f)
                    continue
                try:
                    mujoco.MjModel.from_xml_path(out)
                    counts["loads"] += 1
                except Exception as e:  # MuJoCo raises ValueError for every load error
                    msg = str(e).splitlines()[0]
                    if any(x in msg for x in EXPECTED):
                        counts["needs a missing file or inertia"] += 1
                    else:
                        counts["fails"] += 1
                        key = msg.replace(f, "<file>")
                        unexpected[f"{name}: {key[:100]}"].append(f)
            print(f"{name} -> {target}: " + ", ".join(f"{v} {k}" for k, v in counts.items()))

    for cause, fs in sorted(unexpected.items(), key=lambda kv: -len(kv[1])):
        print(f"UNEXPECTED {len(fs):3} {cause}   e.g. {Path(fs[0]).relative_to(ROOT)}")
    print(f"mujoco {mujoco.__version__}")
    return 1 if unexpected else 0


sys.exit(main())
