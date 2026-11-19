#!/usr/bin/env python3
"""Checks `rigscan convert` of the USD corpus against Pixar's own USD reader.

Each file is converted to MJCF with the built `rigscan` binary and loaded in MuJoCo. The model is
compared with what `pxr` (usd-core) reads from the file: the number of revolute and prismatic
joints, and for every rigid body its pose relative to the first body of its tree, its mass and its
full inertia tensor in the body frame. A conversion that reported a joint, inertial or link loss,
or a Y-up stage, is skipped: the model is then known to differ, and the loss is reported.

    python3 scripts/verify/usd_compare.py [--require] [--bin target/debug/rigscan]

Without --require the check is skipped, with a reason, when `mujoco`, `pxr` or the corpus is missing.
"""
import argparse
import collections
import glob
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CORPUS = "corpus/newton_assets/**/*.usd*"
# Layers of a composed asset that hold only the physics prims: they carry no poses, so the
# bodies of the standalone file all sit at the origin.
FRAGMENT = "Payload/Physics.usda"
# A tree of bodies has no pose in the neutral model, so each tree is compared to its own first body.
POSE, ROTATION, MASS, TENSOR = 3e-3, 2e-3, 1e-4, 1e-3


def skip(reason, require):
    print(f"{'error' if require else 'skipped'}: {reason}")
    return 1 if require else 0


def rotation(np, X):
    r = np.array([[X[a][b] for b in range(3)] for a in range(3)]).T
    return r / np.linalg.norm(r, axis=0)


def compare(np, mujoco, pxr, path, model_path):
    Usd, UsdGeom, UsdPhysics, Gf = pxr
    stage = Usd.Stage.Open(path)
    # An absent metersPerUnit is read as metres, as the converter does; pxr's fallback is 0.01.
    unit = UsdGeom.GetStageMetersPerUnit(stage) if stage.HasAuthoredMetadata("metersPerUnit") else 1.0
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    joints = sum(1 for p in stage.Traverse()
                 if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint))
    if joints != m.njnt:
        return f"{joints} joints in the file, {m.njnt} in the model"
    refs, worst = {}, collections.defaultdict(float)
    for p in stage.Traverse():
        if not p.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, str(p.GetPath()))
        if i < 0:
            continue
        X = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)
        pos, rot = np.array(X.ExtractTranslation()) * unit, rotation(np, X)
        ref = refs.setdefault(int(m.body_rootid[i]),
                              (pos, d.xpos[i].copy(), rot, d.xmat[i].reshape(3, 3).copy()))
        worst["pose"] = max(worst["pose"], np.abs((pos - ref[0]) - (d.xpos[i] - ref[1])).max())
        worst["rotation"] = max(
            worst["rotation"], np.abs(ref[2].T @ rot - ref[3].T @ d.xmat[i].reshape(3, 3)).max())
        mass = p.GetAttribute("physics:mass").Get()
        if mass and mass > 0:
            worst["mass"] = max(worst["mass"], abs(mass - m.body_mass[i]) / mass)
        dia = p.GetAttribute("physics:diagonalInertia").Get()
        if mass and dia and any(dia):
            q = p.GetAttribute("physics:principalAxes").Get()
            Q = (np.array(Gf.Matrix3d(Gf.Quatd(q.GetReal(), Gf.Vec3d(*q.GetImaginary())))).T
                 if q else np.eye(3))
            want = Q @ np.diag(list(dia)) @ Q.T
            Qm = np.zeros(9)
            mujoco.mju_quat2Mat(Qm, m.body_iquat[i])
            Qm = Qm.reshape(3, 3)
            got = Qm @ np.diag(m.body_inertia[i]) @ Qm.T
            worst["tensor"] = max(worst["tensor"], np.abs(want - got).max() / np.abs(want).max())
    over = [f"{k} {worst[k]:.2g}" for k, tol in
            (("pose", POSE), ("rotation", ROTATION), ("mass", MASS), ("tensor", TENSOR))
            if worst[k] > tol]
    return ", ".join(over) or None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--require", action="store_true")
    ap.add_argument("--bin", default=str(ROOT / "target/debug/rigscan"))
    args = ap.parse_args()
    try:
        import mujoco
        import numpy as np
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
    except ImportError as e:
        return skip(f"{e.name} is not installed (pip install mujoco usd-core)", args.require)
    files = sorted(glob.glob(str(ROOT / CORPUS), recursive=True))
    if not files:
        return skip("corpus/newton_assets is not present", args.require)

    counts, wrong = collections.Counter(), []
    with tempfile.TemporaryDirectory() as tmp:
        out, report = f"{tmp}/model.xml", f"{tmp}/loss.json"
        for f in files:
            if f.endswith(FRAGMENT):
                counts["layer without poses, skipped"] += 1
                continue
            r = subprocess.run([args.bin, "convert", f, "--to", "mjcf", "-o", out,
                                "--loss-report", report], capture_output=True, text=True)
            if r.returncode == 2:
                counts["refused by the converter"] += 1
                continue
            lost = {x["id"] for x in json.load(open(report))["lost"]}
            if any(i.startswith("joint/") and i != "joint/name" for i in lost) \
                    or lost & {"inertial", "link", "upAxis"}:
                counts["reported a loss that changes the model, skipped"] += 1
                continue
            try:
                diff = compare(np, mujoco, (Usd, UsdGeom, UsdPhysics, Gf), f, out)
            except Exception as e:  # MuJoCo raises ValueError for every load error
                diff = f"does not load: {str(e).splitlines()[0]}"
            counts["same" if diff is None else "different"] += 1
            if diff:
                wrong.append((f, diff))
    print(", ".join(f"{v} {k}" for k, v in counts.items()))
    for f, diff in wrong:
        print(f"DIFFERENT {Path(f).relative_to(ROOT)}: {diff}")
    print(f"mujoco {mujoco.__version__}")
    return 1 if wrong else 0


sys.exit(main())
