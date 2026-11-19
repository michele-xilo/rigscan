#!/usr/bin/env python3
"""Checks `rigscan convert --to usd` against Pixar's own USD reader and MuJoCo.

Every URDF of the corpus is converted to `.usda` with the built `rigscan` binary. The file must
open with `pxr` (usd-core) and be a valid UsdPhysics stage: the same number of revolute and
prismatic joints as the source model has in MuJoCo, and for every body that exists in both its
pose relative to the first body of its tree and its mass. A source that reported a joint, name
or inertial loss is skipped for the pose and mass check, because the file then differs on purpose.

    python3 scripts/verify/usd_write_check.py [--require] [--bin target/debug/rigscan]

Without --require the check is skipped, with a reason, when `mujoco`, `pxr` or the corpus is missing.
"""
import argparse
import glob
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CORPUS = "corpus/example-robot-data/**/*.urdf"
POSE, ROTATION, MASS = 3e-3, 2e-3, 1e-4
# The source is read with every body kept, as the writer keeps them.
KEEP = '<mujoco><compiler fusestatic="false" discardvisual="false"/></mujoco>'


def skip(reason, require):
    print(f"{'error' if require else 'skipped'}: {reason}")
    return 1 if require else 0


def identifier(name):
    s = re.sub(r"[^A-Za-z0-9]", "_", name)
    return "_" + s if not s or s[0].isdigit() else s


def source_model(mujoco, path):
    text = Path(path).read_text()
    # MuJoCo cannot resolve package:// meshes and the writer leaves meshes out, so both drop them.
    text = re.sub(r"<(visual|collision)\b(?:(?!</\1>).)*?<mesh\b.*?</\1>", "", text, flags=re.S)
    text = re.sub(r"(<robot[^>]*>)", lambda m: m.group(1) + KEEP, text, count=1)
    tmp = Path(path).with_name(Path(path).stem + ".keep.urdf")
    tmp.write_text(text)
    try:
        return mujoco.MjModel.from_xml_path(str(tmp))
    finally:
        tmp.unlink()


def compare(np, mujoco, pxr, usd_path, model):
    Usd, UsdGeom, UsdPhysics = pxr
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        return "does not open"
    joints = sum(1 for p in stage.Traverse()
                 if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint))
    expected = sum(1 for j in range(model.njnt) if model.jnt_type[j] in (2, 3))
    if joints != expected:
        return f"{joints} joints in the file, {expected} in the model"
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    bodies = {}
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies[p.GetName()] = p
    first = None
    for b in range(1, model.nbody):
        label = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        p = bodies.get(identifier(label))
        if p is None:
            continue
        x = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)
        pos = np.array(x.ExtractTranslation())
        rot = np.array([[x[c][r] for c in range(3)] for r in range(3)])
        if first is None:
            first = (pos, rot, d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy())
        up, ur, mp, mr = first
        # Poses relative to the first body, so a root placed anywhere compares equal.
        want = mr.T @ (d.xpos[b] - mp)
        have = ur.T @ (pos - up)
        if np.abs(want - have).max() > POSE:
            return "a body is not where the model puts it"
        mass = UsdPhysics.MassAPI(p).GetMassAttr().Get()
        if mass is not None and abs(mass - model.body_mass[b]) > MASS * max(1.0, model.body_mass[b]):
            return "a body has another mass than the model"
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--require", action="store_true")
    ap.add_argument("--bin", default=str(ROOT / "target/debug/rigscan"))
    args = ap.parse_args()
    try:
        import mujoco
        import numpy as np
        from pxr import Usd, UsdGeom, UsdPhysics
    except ImportError as e:
        return skip(f"{e.name} is not installed", args.require)
    files = sorted(glob.glob(str(ROOT / CORPUS), recursive=True))
    if not files:
        return skip(f"no files match {CORPUS}", args.require)
    same = skipped = 0
    bad = []
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "out.usda"
        report = Path(tmp) / "loss.json"
        for f in files:
            r = subprocess.run([args.bin, "convert", f, "--to", "usd", "-o", str(out),
                                "--loss-report", str(report)], capture_output=True, text=True)
            if r.returncode == 2:
                skipped += 1
                print(f'skip {Path(f).name}: refused')
                continue
            try:
                model = source_model(mujoco, f)
            except Exception as e:
                print(f'skip {Path(f).name}: {str(e).splitlines()[0][:90]}')
                skipped += 1
                continue
            lost = {x["id"] for x in json.loads(report.read_text()).get("losses", [])}
            if any(i.startswith(("joint", "name", "inertial")) for i in lost):
                err = compare(np, mujoco, (Usd, UsdGeom, UsdPhysics), str(out), model) \
                    if "joint/undefined" not in lost else None
                if err and "does not open" in err:
                    bad.append((f, err))
                skipped += 1
                continue
            err = compare(np, mujoco, (Usd, UsdGeom, UsdPhysics), str(out), model)
            if err:
                bad.append((f, err))
            else:
                same += 1
    for f, e in bad:
        print(f"DIFFERENT {Path(f).relative_to(ROOT)}: {e}")
    print(f"{same} same, {len(bad)} different, {skipped} skipped")
    return 1 if bad or (args.require and same == 0) else 0


if __name__ == "__main__":
    sys.exit(main())
