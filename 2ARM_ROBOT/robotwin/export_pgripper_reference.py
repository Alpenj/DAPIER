"""Freeze the current MuJoCo PGripper geometry and expert waypoints for SAPIEN."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np


def export(source, model_path, output):
    sys.path.insert(0, str(source))
    from replay_recorded_episode import build_tabletop_spec
    from mobile_dual_so101 import apply_control_as_pose
    from pgripper import home_action
    from pgripper_handover import run

    output.mkdir(parents=True, exist_ok=False)
    profile = json.loads((source / "tabletop_replay.json").read_text())
    model = build_tabletop_spec(model_path, profile, grippers="both").compile()
    data = mujoco.MjData(model)
    home = np.asarray(home_action(model, np.tile(np.deg2rad([0, -35, 55, 35, 0, 0]), 2)))
    apply_control_as_pose(model, data, home)
    meshes = output / "meshes"
    meshes.mkdir()
    used_meshes = set()
    bodies = []
    for bid in range(1, model.nbody):
        b = model.body(bid)
        if not b.name.startswith(("left_", "right_")):
            continue
        item = dict(name=b.name, parent=model.body(int(b.parentid[0])).name,
                    pos=b.pos.tolist(), quat=b.quat.tolist(), mass=float(b.mass[0]),
                    ipos=b.ipos.tolist(), iquat=b.iquat.tolist(), inertia=b.inertia.tolist(), geoms=[])
        assert b.jntnum[0] <= 1
        if b.jntnum[0]:
            j = model.joint(int(b.jntadr[0]))
            d = int(j.dofadr[0])
            item["joint"] = dict(name=j.name, type=int(j.type[0]), pos=j.pos.tolist(), axis=j.axis.tolist(),
                limits=j.range.tolist(), ref=float(model.qpos0[j.qposadr[0]]), damping=float(model.dof_damping[d]),
                friction=float(model.dof_frictionloss[d]), armature=float(model.dof_armature[d]))
        for gid in range(int(b.geomadr[0]), int(b.geomadr[0] + b.geomnum[0])):
            g = model.geom(gid)
            mid = int(g.dataid[0])
            item["geoms"].append(dict(name=g.name, type=int(g.type[0]), pos=g.pos.tolist(), quat=g.quat.tolist(),
                size=g.size.tolist(), rgba=g.rgba.tolist(), collision=bool(g.contype[0] or g.conaffinity[0]),
                friction=g.friction.tolist(), mesh=f"meshes/{mid}.obj" if mid >= 0 else None))
            if mid >= 0:
                used_meshes.add(mid)
        bodies.append(item)
    for mid in used_meshes:
        va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
        with (meshes / f"{mid}.obj").open("w") as f:
            for v in model.mesh_vert[va:va+vn]:
                f.write("v " + " ".join(map(str, v)) + "\n")
            for face in model.mesh_face[fa:fa+fn]:
                f.write("f " + " ".join(str(int(i)+1) for i in face) + "\n")
    joints = [model.joint(int(j)).name for j in model.actuator_trnid[:, 0]]
    drives = []
    for i, name in enumerate(joints):
        drives.append(dict(name=name, kp=float(model.actuator_gainprm[i, 0]),
            kv=float(-model.actuator_biasprm[i, 2]), force_limit=float(model.actuator_forcerange[i, 1])))
    couplings = []
    for i in range(model.neq):
        assert model.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT
        follower, driver = [model.joint(int(j)) for j in (model.eq_obj1id[i], model.eq_obj2id[i])]
        factor = float(model.eq_data[i, 1])
        offset = float(model.qpos0[follower.qposadr[0]] - factor * model.qpos0[driver.qposadr[0]])
        couplings.append(dict(follower=follower.name, driver=driver.name, factor=factor, offset=offset))
    cameras = [dict(name=c.name, body=model.body(int(c.bodyid[0])).name,
                   pos=c.pos.tolist(), quat=c.quat.tolist(), fovy=float(c.fovy[0]))
               for c in [model.camera(n) for n in ("top_h201_reference", "left_wrist_rgb", "right_wrist_rgb")]]
    # Freeze several configurations, including both aperture endpoints, for independent FK checks.
    samples = []
    rng = np.random.default_rng(60909)
    for i in range(8):
        action = home.copy()
        if i:
            action = rng.uniform(model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        if i < 3:
            action[[5, 11]] = (2.2028, 0, 1.1)[i]
        apply_control_as_pose(model, data, action)
        samples.append(dict(action=action.tolist(), poses={b["name"]: np.r_[data.body(b["name"]).xpos,
            data.body(b["name"]).xquat].tolist() for b in bodies}))
    source_files = [source / n for n in ("pgripper.py", "pgripper_handover.py", "physics_ik.py",
        "replay_recorded_episode.py", "tabletop_replay.json", "assets/norma_pgripper/provenance.json")]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}
    reference = dict(profile=profile, bodies=bodies, couplings=couplings, cameras=cameras, joints=joints,
        drives=drives, home=home.tolist(), ctrlrange=model.actuator_ctrlrange.tolist(), fk_samples=samples,
        source_hashes=hashes, mujoco_version=mujoco.__version__, grippers="both", action_units="rad; opening-positive gripper")
    (output / "reference.json").write_text(json.dumps(reference, indent=2))
    args = argparse.Namespace(model=model_path, output=output / "mujoco_baseline", donor="left",
        disable_recipient_contact=False, handover_only=False, viewer=False, render=False)
    assert run(args) == 0, "MuJoCo reference task failed"
    assert hashes == {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}, "Reference changed during export"
    print("Reference exported:", output, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export(args.source.resolve(), args.model.resolve(), args.output.resolve())
