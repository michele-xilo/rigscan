#!/usr/bin/env python3
"""Compares a model with its conversion, both loaded in MuJoCo.

Each fixture in scripts/verify/fixtures is converted with the built `rigscan` binary and the two
models must agree on the number of moving bodies, the number of joints, the mass of the moving
bodies and the type and axis of every joint. Bodies welded to the world are left out: MuJoCo's URDF
reader gives them no mass, whatever the file says. The fixtures hold no meshes because the corpora
cannot be loaded without them.

    python3 scripts/verify/mujoco_compare.py [--require] [--bin target/debug/rigscan]

Without --require the check is skipped, with a reason, when `mujoco` is missing.
"""
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = ROOT / "scripts/verify/fixtures"
RUNS = [("arm.xml", "urdf", ".urdf"), ("arm.urdf", "mjcf", ".xml")]


def summary(mujoco, path):
    m = mujoco.MjModel.from_xml_path(str(path))
    moving = m.body_dofnum > 0
    joints = {}
    for i in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
        joints[name] = (int(m.jnt_type[i]), [round(float(x), 9) for x in m.jnt_axis[i]])
    return {
        "moving bodies": int(moving.sum()),
        "joints": m.njnt,
        "moving mass": round(float(m.body_mass[moving].sum()), 9),
        "joint type and axis": joints,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--require", action="store_true")
    ap.add_argument("--bin", default=str(ROOT / "target/debug/rigscan"))
    args = ap.parse_args()
    try:
        import mujoco
    except ImportError:
        print(f"{'error' if args.require else 'skipped'}: the mujoco package is not installed")
        return 1 if args.require else 0

    bad = 0
    with tempfile.TemporaryDirectory() as tmp:
        for name, target, suffix in RUNS:
            src = FIXTURES / name
            out = Path(tmp) / f"model{suffix}"
            # exit 1 means something was lost and reported; only a refusal (2) is a failure here
            r = subprocess.run([args.bin, "convert", str(src), "--to", target, "-o", str(out)],
                               capture_output=True, text=True)
            if r.returncode == 2:
                print(f"{name} -> {target}: refused: {r.stderr.strip()}")
                bad += 1
                continue
            a, b = summary(mujoco, src), summary(mujoco, out)
            diffs = [f"{k}: {a[k]} became {b[k]}" for k in a if a[k] != b[k]]
            print(f"{name} -> {target}: " + ("same" if not diffs else "DIFFERENT"))
            for d in diffs:
                print(f"  {d}")
            bad += bool(diffs)
    print(f"mujoco {mujoco.__version__}")
    return 1 if bad else 0


sys.exit(main())
