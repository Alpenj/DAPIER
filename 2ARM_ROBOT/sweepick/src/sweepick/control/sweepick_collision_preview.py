"""Collision / self-collision result for ONE trajectory file written by python -m sweepick.control.sweepick_trajectory_executor.

Geometry only: forward kinematics + MuJoCo's collision detection on the assembled dual scene. No dynamics, no policy.
VERDICT = the commanded (nominal) path, checked at EVERY commanded step, with no penetration allowance: a contact pair
that does not exist at the start pose, or a start-pose pair that gets any deeper, is a contact.
The joint mapping is a CANDIDATE, so this is a result in the model's frame: a PROVISIONAL gate of model-dependent moves,
not a physical statement about the real cell.
DIAGNOSTIC (never a verdict): the same check with joints displaced from the commanded path by a chosen number of ticks.
That number has no physical basis (it came from 'twice the largest lag seen'); it only shows how close the path runs.

usage: python -m sweepick.control.sweepick_collision_preview TRAJECTORY.json CANDIDATE_Q12_CONFIG.json CALIBRATION_DIR ASSEMBLED.json OUT.json [SCENE.mjb] [DIAG_TICKS]"""
import hashlib, itertools, json, sys
from pathlib import Path
import numpy as np, mujoco
from sweepick.control import sweepick_joint_command_mapping as rc
from sweepick.control import sweepick_trajectory_profile as tt
from sweepick.control import sweepick_joint_kinematics as sweepick_kin

FLOAT_EPS = 1e-9          # floating-point comparison only; not a penetration allowance


def main(traj_file, cand_file, cal_dir, model_file, out_file, scene_mjb=None, diag_ticks=30):
    """scene_mjb: the compiled assembled scene (same model, saved with mj_saveModel). Without it the scene is built from
    integration_scenes (model host only)."""
    doc = json.loads(Path(traj_file).read_text())
    side, traj, other = doc["side"], doc["trajectory"], doc.get("other_arm_ticks")
    mapping = rc.load_mapping(cand_file, cal_dir, model_file)
    msha = hashlib.sha256(Path(cand_file).read_bytes()).hexdigest()
    ident = tt.identity(side, traj, msha)
    out = dict(schema="tjj.collision-result.v3", identity=ident, identity_in_file=doc.get("identity"), side=side, mapping_sha256=msha, mapping_status="CANDIDATE (verified=false)", other_arm_ticks=other,
               scene="integration_scenes.task_env('desk', grippers='both')" + (" (compiled .mjb)" if scene_mjb else ""), scene_sha256=hashlib.sha256(Path(scene_mjb).read_bytes()).hexdigest() if scene_mjb else None, calibration_dir=str(cal_dir),
               scope=["robot self-collision", "the other arm at other_arm_ticks", "desk and fixed scene parts of the SIM model"], not_covered=["real objects on the table, cables, the hub, anything absent from the SIM scene", "errors of the candidate joint mapping", "the SIM scene's free block (ignored)", "any position off the commanded path (see tube_diagnostic)"],
               kind="geometric result in the MODEL frame through a candidate mapping: provisional, not a physical limit")
    if ident != doc.get("identity") or msha != doc.get("mapping_sha256"):
        out.update(verdict="INVALID_INPUT", reason="the identity / mapping of the file does not match what this host computes")
        Path(out_file).write_text(json.dumps(out, indent=1)); print(json.dumps(out)); return
    if other is None:
        out.update(verdict="INVALID_INPUT", reason="the other arm's position is not given: arm-to-arm contact cannot be judged")
        Path(out_file).write_text(json.dumps(out, indent=1)); print(json.dumps(out)); return
    if scene_mjb:
        m = mujoco.MjModel.from_binary_path(str(scene_mjb))
    else:
        import integration_scenes
        m = integration_scenes.task_env("desk", grippers="both").model
    d = mujoco.MjData(m)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    gname = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}"
    bname = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or f"body{m.geom_bodyid[g]}"
    free = {b for b in range(m.nbody) if m.body_jntnum[b] and m.jnt_type[m.body_jntadr[b]] == mujoco.mjtJoint.mjJNT_FREE}

    def set_arm(s, ticks):
        sweepick_kin.set_arm_q(m, d, s, [rc.tick_to_model(n, mapping[s][n], ticks[n]) for n in rc.JOINTS])

    def contacts(ticks):
        d.qpos[:] = m.qpos0
        set_arm(side, ticks); set_arm("left" if side == "right" else "right", other)
        mujoco.mj_forward(m, d)
        res = {}
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist < 0 and m.geom_bodyid[c.geom1] not in free and m.geom_bodyid[c.geom2] not in free:
                key = tuple(sorted((f"{bname(c.geom1)}:{gname(c.geom1)}", f"{bname(c.geom2)}:{gname(c.geom2)}")))
                res[key] = min(res.get(key, 0.0), float(c.dist))
        return res

    start = {n: traj["start"][n] for n in rc.JOINTS}
    base = contacts(start)

    def worse(found):
        return {k: d for k, d in found.items() if k not in base or d < base[k] - FLOAT_EPS}

    first, pairs = None, {}
    for k in range(traj["steps"]):                                             # every commanded step
        bad = worse(contacts({n: traj["goals"][n][k] for n in rc.JOINTS}))
        for key, dist in bad.items():
            if first is None:
                first = dict(step=k, time_s=round((k + 1) * traj["dt"], 3), pair=list(key), depth_m=dist, new_pair=key not in base)
            pairs[key] = min(pairs.get(key, 0.0), dist)
    diag_ticks = float(diag_ticks)
    tfirst, tpairs, evals, last = None, {}, 0, None
    for k in range(traj["steps"]):
        nominal = {n: traj["goals"][n][k] for n in rc.JOINTS}
        if last is not None and k != traj["steps"] - 1 and max(abs(nominal[n] - last[n]) for n in traj["moving"]) < diag_ticks / 4:      # diagnostic density only
            continue
        last = nominal
        for off in itertools.product((-diag_ticks, 0.0, diag_ticks), repeat=len(rc.JOINTS)):
            if not any(off):
                continue
            t = {n: float(np.clip(nominal[n] + o, mapping[side][n]["lo"], mapping[side][n]["hi"])) for n, o in zip(rc.JOINTS, off)}
            evals += 1
            for key, dist in worse(contacts(t)).items():
                if tfirst is None:
                    tfirst = dict(step=k, pair=list(key), depth_m=dist, offset_ticks={n: o for n, o in zip(rc.JOINTS, off) if o})
                tpairs[key] = min(tpairs.get(key, 0.0), dist)
    out.update(nominal_verdict="CLEAR" if not pairs else "CONTACT", verdict="CLEAR" if not pairs else "CONTACT", first_nominal_contact=first, nominal_contact_pairs={" | ".join(k): v for k, v in sorted(pairs.items(), key=lambda x: x[1])[:12]},
               nominal_steps_checked=traj["steps"], penetration_allowance_m=0.0, baseline_contacts_at_start={" | ".join(k): v for k, v in base.items()},
               tube_diagnostic=dict(role="DIAGNOSTIC only: never a verdict and never a gate", displacement_ticks=diag_ticks, basis="no physical basis; shows how close the commanded path runs to a contact", evaluations=evals, contacts=len(tpairs), first=tfirst,
                                    pairs={" | ".join(k): v for k, v in sorted(tpairs.items(), key=lambda x: x[1])[:8]}), duration_s=traj["duration_s"], travel_deg=traj["travel_deg"])
    Path(out_file).write_text(json.dumps(out, indent=1))
    print(json.dumps({k: out[k] for k in ("nominal_verdict", "identity", "first_nominal_contact", "nominal_steps_checked", "travel_deg")} | dict(tube=dict(contacts=len(tpairs), first=tfirst)), default=str))


if __name__ == "__main__":
    main(*sys.argv[1:8])
