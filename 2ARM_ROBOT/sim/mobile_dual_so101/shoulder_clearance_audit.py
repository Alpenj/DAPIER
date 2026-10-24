#!/usr/bin/env python3
"""Offline shoulder/table kinematic audit. Never a physics/control trajectory."""
import argparse
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np

from collision_diagnostics import describe_pair
from integration_scenes import task_env, task_provenance


def shoulder_dependency(model, data, side):
    mesh = model.mesh(side + "_rotation_pitch_so101_v1").id
    geoms = [g for g in range(model.ngeom)
             if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
             and model.geom_dataid[g] == mesh and model.geom_contype[g]]
    if len(geoms) != 1:
        raise ValueError("expected one active shoulder rotation_pitch mesh")
    geom = geoms[0]
    body = int(model.geom_bodyid[geom])
    chain, joints = [], []
    ancestor = body
    while ancestor:
        chain.append(model.body(ancestor).name)
        joints.extend(range(int(model.body_jntadr[ancestor]),
                            int(model.body_jntadr[ancestor]+model.body_jntnum[ancestor])))
        ancestor = int(model.body_parentid[ancestor])
    if len(joints) != 1 or model.jnt_type[joints[0]] != mujoco.mjtJoint.mjJNT_HINGE:
        raise ValueError("model changed: audit requires a new multi-joint sweep design")
    joint = joints[0]
    if not model.jnt_limited[joint]:
        raise ValueError("unbounded joint: cannot claim full-range coverage")
    limits = model.jnt_range[joint].copy()
    actuators = [i for i in range(model.nu) if model.actuator_trnid[i, 0] == joint]
    # Audit the full physical joint range, including the tiny rounded ctrl-range
    # fringe. This is FK only; command validity is recorded, never overridden.
    control_ranges = [model.actuator_ctrlrange[i].tolist() for i in actuators]
    for i in actuators:
        if model.actuator_ctrllimited[i]:
            limits[0] = max(limits[0], model.actuator_ctrlrange[i, 0])
            limits[1] = min(limits[1], model.actuator_ctrlrange[i, 1])
    if limits[0] >= limits[1]:
        raise ValueError("empty joint/control intersection")
    return dict(side=side, geom_id=geom, geom_name=model.geom(geom).name or None,
        mesh_asset=model.mesh(mesh).name, body=model.body(body).name,
        parent_body=model.body(int(model.body_parentid[body])).name,
        ancestor_bodies=chain+["world"], influencing_joint_ids=joints,
        joint_id=joint, joint_name=model.joint(joint).name,
        qpos_address=int(model.jnt_qposadr[joint]),
        local_axis=model.jnt_axis[joint].tolist(), world_axis=data.xaxis[joint].tolist(),
        world_anchor_m=data.xanchor[joint].tolist(),
        joint_range_rad=model.jnt_range[joint].tolist(),
        actuator_ctrlranges_rad=control_ranges, command_range_rad=limits.tolist(),
        effective_range_rad=model.jnt_range[joint].tolist())


def audit_sweep(env, *, sample_step_deg=.5, observer=None, preview_data=None):
    if not math.isfinite(sample_step_deg) or not 0 < sample_step_deg <= .5:
        raise ValueError("sample step must be finite and in (0, 0.5] degrees")
    model = env.model
    kind = mujoco.mjtState.mjSTATE_INTEGRATION
    initial = np.empty(mujoco.mj_stateSize(model, kind))
    mujoco.mj_getState(model, env.data, initial, kind)
    data = preview_data if preview_data is not None else mujoco.MjData(model)
    mujoco.mj_setState(model, data, initial, kind)
    mujoco.mj_forward(model, data)
    home = data.qpos.copy()
    table = model.geom("table").id
    result = dict(scene_id=env.config.scene_id, mode="KINEMATIC SWEEP / NOT PHYSICS EXECUTION",
                  physics_steps=0, required_clearance_m=env.config.required_clearance_m,
                  sample_step_deg=sample_step_deg, home_qpos=home.tolist(), sides={})
    for side in ("left", "right"):
        data.qpos[:] = home
        mujoco.mj_forward(model, data)
        dep = shoulder_dependency(model, data, side)
        g, j, adr = dep["geom_id"], dep["joint_id"], dep["qpos_address"]
        reference_pos, reference_mat = data.geom_xpos[g].copy(), data.geom_xmat[g].copy()
        other_checks = []
        for other in range(model.njnt):
            if other == j:
                continue
            data.qpos[:] = home
            a = int(model.jnt_qposadr[other])
            if model.jnt_type[other] == mujoco.mjtJoint.mjJNT_FREE:
                data.qpos[a:a+3] += [.011, .013, .017]
            elif model.jnt_limited[other]:
                lo, hi = model.jnt_range[other]
                data.qpos[a] = lo if abs(home[a]-lo) > abs(home[a]-hi) else hi
            else:
                data.qpos[a] += .01
            mujoco.mj_forward(model, data)
            error = max(float(np.max(np.abs(data.geom_xpos[g]-reference_pos))),
                        float(np.max(np.abs(data.geom_xmat[g]-reference_mat))))
            other_checks.append(dict(joint=model.joint(other).name, transform_max_abs_change=error))
            if error > 1e-12:
                raise ValueError("non-ancestor joint unexpectedly changed shoulder transform")
        dep["non_dependency_fk_checks"] = other_checks
        lo, hi = dep["effective_range_rad"]
        intervals = math.ceil((hi-lo)/math.radians(sample_step_deg))
        samples = []
        for index, angle in enumerate(np.linspace(lo, hi, intervals+1)):
            data.qpos[:] = home
            data.qpos[adr] = angle
            data.qvel[:] = 0
            # Explicit isolated FK preview only: never mj_step or control execution.
            mujoco.mj_forward(model, data)
            diagnostic = describe_pair(model, data, g, table, env.config.required_clearance_m)
            evidence = diagnostic["independent_geometry"]
            hull = evidence["compiled_hull"]
            row = dict(sample=index, angle_rad=float(angle), angle_deg=math.degrees(angle),
                native_signed_distance_m=diagnostic["native_signed_distance_m"],
                guard_distance_m=diagnostic["final_distance_m"],
                mesh_box_certificate_m=diagnostic["certified_mesh_box_separation_lower_bound_m"],
                independent_geometry=evidence, contacts=diagnostic["pair_contacts"],
                minimum_world_z_m=float(hull["lowest_vertex_world_m"][2]),
                penetration=any(c["distance_m"] < 0 for c in diagnostic["pair_contacts"]),
                independent_positive_separation=hull["gap_m"] > 0,
                min_z_gap_m=hull["gap_m"])
            samples.append(row)
            if observer:
                observer(model, data, dep, row, intervals+1)
        # Rigorous interpolation allowance: |dz/dtheta| <= r*|n x axis|.
        # This supplements sampling; it is NOT a change to the runtime guard.
        mesh = int(model.geom_dataid[g])
        start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
        vertices = model.mesh_vert[start:start+count].astype(float)
        world = vertices @ data.geom_xmat[g].reshape(3,3).T + data.geom_xpos[g]
        radius = float(np.max(np.linalg.norm(world-data.xanchor[j], axis=1)))
        normal = data.geom_xmat[table].reshape(3,3)[:,2].copy()
        normal /= np.linalg.norm(normal)
        axis = np.array(dep["world_axis"])
        axis /= np.linalg.norm(axis)
        cross = float(np.linalg.norm(np.cross(axis, normal)))
        allowance = radius*cross*(hi-lo)/intervals/2 + 1e-12
        minimum_gap = min(row["min_z_gap_m"] for row in samples)
        exact = [row["independent_geometry"]["compiled_hull"]["exact_positive_distance_m"]
                 for row in samples]
        result["sides"][side] = dict(dependency=dep, samples=samples,
            summary=dict(sample_count=len(samples),
                native_zero_count=sum(row["native_signed_distance_m"] == 0 for row in samples),
                native_min_m=min(row["native_signed_distance_m"] for row in samples),
                min_z_gap_m=minimum_gap, max_z_gap_m=max(row["min_z_gap_m"] for row in samples),
                exact_witness_count=sum(value is not None for value in exact),
                minimum_exact_witness_distance_m=min((x for x in exact if x is not None), default=None),
                contact_sample_count=sum(bool(row["contacts"]) for row in samples),
                penetration_sample_count=sum(row["penetration"] for row in samples),
                axis_cross_table_normal=cross, maximum_vertex_radius_m=radius,
                between_samples_gap_allowance_m=allowance,
                continuous_separation_lower_bound_m=minimum_gap-allowance,
                minimum_guard_distance_m=min(row["guard_distance_m"] for row in samples)))
    after = np.empty_like(initial)
    mujoco.mj_getState(model, env.data, after, kind)
    result["source_simulation_state_unchanged"] = bool(np.array_equal(initial, after))
    if not result["source_simulation_state_unchanged"]:
        raise RuntimeError("audit changed source simulation state")
    result["policy_status"] = "NOMINAL_NEAR_SUPPORT_CANDIDATE; REAL_ASSEMBLY_UNVERIFIED; NO_POLICY_CHANGE"
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-step-deg", type=float, default=.5)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("use a new output file; preserve prior evidence")
    env = task_env("desk")
    provenance = task_provenance(env)
    import hashlib
    provenance["audit_source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    def save(report):
        report["provenance"] = provenance
        args.output.write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps({side:r["summary"] for side,r in report["sides"].items()}, indent=2), flush=True)
    if not args.viewer:
        save(audit_sweep(env, sample_step_deg=args.sample_step_deg))
        return
    from mujoco.viewer import launch_passive
    # Window observes the audit's exact preview data, not a second sweep.
    display_data = mujoco.MjData(env.model)
    original_rgba = env.model.geom_rgba.copy()
    last = time.monotonic()
    last_overlay = ""
    with launch_passive(env.model, display_data, show_left_ui=False, show_right_ui=False) as viewer:
        def observe(model, data, dep, row, count):
            nonlocal last, last_overlay
            if not viewer.is_running():
                raise RuntimeError("viewer closed: sweep incomplete")
            assert data is display_data  # exact FK state measured by the audit
            g, table = dep["geom_id"], model.geom("table").id
            with viewer.lock():
                model.geom_rgba[:] = original_rgba
                model.geom_rgba[g] = [1., .02, .02, 1.]
                model.geom_rgba[table] = [1., .85, .03, 1.]
                viewer.opt.geomgroup[model.geom_group[g]] = 1
            lines = [
                "KINEMATIC SWEEP / NOT PHYSICS EXECUTION",
                "integration_desk | SHOULDER/TABLE ONLY",
                f"{dep['side']} {dep['joint_name']}",
                f"{row['sample']+1}/{count} | angle {row['angle_deg']:.2f} deg",
                f"Range: {np.degrees(dep['effective_range_rad']).round(2)} deg",
                f"RED geom {g} / YELLOW table {table}",
                f"native: {row['native_signed_distance_m']*1000:.6f} mm",
                f"guard: {row['guard_distance_m']*1000:.6f} mm",
                f"independent Z gap: {row['min_z_gap_m']*1000:.6f} mm",
                f"contacts: {len(row['contacts'])} / penetration: {row['penetration']}",
                "Required: 30 mm (UNCHANGED)",
                "qpos preview only | SIM time 0 | NO mj_step",
            ]
            last_overlay = "\n".join(lines)
            if len(last_overlay) >= 500:
                raise RuntimeError("sweep overlay exceeds renderer text capacity")
            viewer.set_texts((mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                              last_overlay, ""))
            viewer.sync()
            if row["sample"] % 40 == 0 or row["sample"]+1 == count:
                print(json.dumps({k:row[k] for k in ("sample","angle_deg",
                    "native_signed_distance_m","guard_distance_m","min_z_gap_m","penetration")}),
                    flush=True)
            time.sleep(max(0., 1/20-(time.monotonic()-last)))
            last = time.monotonic()
        try:
            report = audit_sweep(env, sample_step_deg=args.sample_step_deg,
                                 observer=observe, preview_data=display_data)
            save(report)
            viewer.set_texts([(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               last_overlay, ""),
                (mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                "SWEEP COMPLETE | KINEMATIC PREVIEW\n"
                + "\n".join(f"{s}: minimum {r['summary']['min_z_gap_m']*1000:.6f} mm"
                            for s,r in report["sides"].items())
                + "\n30 mm policy UNCHANGED | NOT task motion\n"
                  "Assembly uncertainty UNVERIFIED | close to exit", "")])
            while viewer.is_running():
                viewer.sync()
                time.sleep(.05)
        finally:
            env.model.geom_rgba[:] = original_rgba


if __name__ == "__main__":
    main()
