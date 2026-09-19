#!/usr/bin/env python3
"""One copied PRE-CLOSE NoSlip 0/5 comparison; no live teacher or hardware."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import types

import mujoco
import numpy as np

from close_contact_diagnostic import restore
from dynamic_preflight import integration_state
from pgripper import jaw_gap_m
from waypoint_block_teacher import WaypointBlockTeacher

MODE = "DIAGNOSTIC COPY / NOT LIVE TASK SUCCESS"
PADS = ("left_pgripper_pad_1", "left_pgripper_pad_2")


def contact_metrics(contacts, com, rotation):
    """Normalize stored native contacts; never reconstruct forces with mj_forward."""
    com, rotation = np.asarray(com), np.asarray(rotation).reshape(3, 3)
    normalized = []
    for contact in contacts:
        names = contact["names"]
        pads = [name for name in names if name in PADS]
        if "red_block_geom" not in names or not pads:
            continue
        frame = np.asarray(contact["frame"]).reshape(3, 3)
        wrench = np.asarray(contact["wrench_contact"])
        point = np.asarray(contact["position_world_m"])
        if not all(np.isfinite(x).all() for x in (frame, wrench, point, com, rotation)):
            raise ValueError("non-finite native contact evidence")
        # MuJoCo contact axes are rows; force points from geom1 toward geom2.
        # Keep r×f separate from the contact's own torsional/rolling moment.
        sign = 1 if names[1] == "red_block_geom" else -1
        force, moment = sign*frame.T@wrench[:3], sign*frame.T@wrench[3:]
        lever = point-com
        normalized.append(dict(pad=pads[0], position_world_m=point.tolist(),
            position_block_com_m=(rotation.T@lever).tolist(),
            inward_normal_world=(sign*frame[0]).tolist(), normal_force_N=float(wrench[0]),
            tangential_force_contact_N=wrench[1:3].tolist(), force_world_N=force.tolist(),
            force_block_N=(rotation.T@force).tolist(),
            tau_r_cross_f_world_Nm=np.cross(lever,force).tolist(),
            intrinsic_contact_moment_world_Nm=moment.tolist(), distance_m=contact["distance_m"]))
    total = lambda key: np.sum([c[key] for c in normalized], axis=0) if normalized else np.zeros(3)
    net_force, tau_force, tau_intrinsic = (total(k) for k in
        ("force_world_N", "tau_r_cross_f_world_Nm", "intrinsic_contact_moment_world_Nm"))
    groups = {}
    for pad in PADS:
        cs = [c for c in normalized if c["pad"] == pad]
        normal = sum(c["normal_force_N"] for c in cs)
        centroid = (np.sum([np.array(c["position_world_m"])*c["normal_force_N"] for c in cs],axis=0)/normal).tolist() if normal > 0 else None
        groups[pad] = dict(contact_count=len(cs), summed_normal_force_N=normal,
                           normal_weighted_centroid_world_m=centroid)
    first = [next((c for c in normalized if c["pad"] == pad), None) for pad in PADS]
    world_height = abs(first[0]["position_world_m"][2]-first[1]["position_world_m"][2]) if all(first) else None
    block_height = abs(first[0]["position_block_com_m"][2]-first[1]["position_block_com_m"][2]) if all(first) else None
    return dict(metric_revision="native-all-contacts-v1", block_COM_world_m=com.tolist(),
        block_rotation_world=rotation.tolist(), contacts=normalized, pads=groups,
        first_native_height_world_m=world_height, first_native_height_block_m=block_height,
        F_net_world_N=net_force.tolist(), F_net_block_N=(rotation.T@net_force).tolist(),
        tau_COM_r_cross_f_world_Nm=tau_force.tolist(),
        tau_COM_r_cross_f_block_Nm=(rotation.T@tau_force).tolist(),
        intrinsic_contact_moment_world_Nm=tau_intrinsic.tolist(),
        full_constraint_tau_COM_world_Nm=(tau_force+tau_intrinsic).tolist(),
        geometric_surface_overlap=None,
        geometric_note="Native representative points/centroids are not a continuous pressure patch.")


def native_contacts(model, data):
    result=[]
    for i,c in enumerate(data.contact):
        if model.geom("red_block_geom").id not in (c.geom1,c.geom2):
            continue
        wrench=np.zeros(6);mujoco.mj_contactForce(model,data,i,wrench)
        result.append(dict(names=[model.geom(int(c.geom1)).name,model.geom(int(c.geom2)).name],
            position_world_m=c.pos.tolist(),frame=c.frame.reshape(3,3).tolist(),
            wrench_contact=wrench.tolist(),distance_m=float(c.dist),friction=c.friction.tolist(),condim=int(c.dim)))
    return result


def options(model):
    return {n:np.asarray(getattr(model.opt,n)).tolist() for n in dir(model.opt)
            if not n.startswith("_") and not callable(getattr(model.opt,n))}


def array_hashes(model):
    return {n:hashlib.sha256(getattr(model,n).tobytes()).hexdigest() for n in dir(model)
            if not n.startswith("_") and isinstance(getattr(model,n),np.ndarray)}


def run(source_path, continuation_path, model_path, donor_path, output):
    if output.exists():
        raise FileExistsError("Preserve the one controlled A/B: output already exists")
    source=json.loads(source_path.read_text());cont=json.loads(continuation_path.read_text())
    donor=json.loads(donor_path.read_text());state=np.asarray(source["approach_terminal_state"])
    candidate=source["candidate_kinematic"]
    close_stages=int(cont["close_stages"][-1]["stage"])
    assert close_stages==48  # Frozen historical A2 schedule, not an adaptive new search.
    output.mkdir(parents=True)
    result=dict(mode=MODE,hardware_execution=False,live_success=False,
        source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in (source_path,continuation_path,model_path,donor_path,Path(__file__))},
        preclose_state=state.tolist(),state_spec="mjSTATE_INTEGRATION",close_stages=close_stages,
        protocol="same frozen 48 CLOSE stages + 50 CONFIRM steps; only noslip_iterations changes",
        force_timing="integrated qpos + mj_forward after mj_step, as existing arm teacher",
        cases={})
    expected_arrays=None;expected_options=None;command_hashes=[]
    for setting in (0,5):
        teacher=WaypointBlockTeacher(donor["candidate"])
        teacher.env.physics_observer=None;teacher.env.reset(seed=0)
        m=mujoco.MjModel.from_binary_path(str(model_path));arrays=array_hashes(m)
        if expected_arrays is None:expected_arrays=arrays;expected_options=options(m)
        assert arrays==expected_arrays and options(m)==expected_options
        m.opt.noslip_iterations=setting;d=mujoco.MjData(m)
        teacher.m=m;teacher.d=d;teacher.env.model=m;teacher.env.data=d
        restore(m,d,state);np.testing.assert_array_equal(integration_state(m,d),state)
        reference=d.ctrl.copy();teacher.site=m.site("left_cube_grasp").id
        teacher.approach=np.asarray(candidate["target_TCP"])[:3,0]
        teacher.closing=np.asarray(candidate["target_TCP"])[:3,2]
        teacher.waypoint_xyz=np.asarray(candidate["target_TCP"])[:3,3]
        teacher.env.settle_info=donor["settle"];teacher.env._block_hold_s=0
        teacher.close_arm_reference=reference.copy();teacher.gripper_hold_reference=(reference[5],reference[11])
        teacher.close_start_tcp=d.site_xpos[teacher.site].copy();teacher.close_contact_origin=None
        teacher.step_telemetry=[];teacher.phase="CLOSE";teacher.env.collision_phase="CLOSE"
        block=m.body("red_block").id;initial_com=d.xipos[block].copy();initial_quat=d.xquat[block].copy()
        case=dict(initial_state_exact=True,initial_ctrl=reference.tolist(),options=options(m),
                  array_hashes=arrays,rows_file=f"noslip{setting}.jsonl",stages=[],failure=None)
        digest=hashlib.sha256();count=0;last_row=None;stage=0
        stream=(output/case["rows_file"]).open("w")
        def record(self):
            nonlocal count,last_row
            contacts=native_contacts(m,d);velocity=np.zeros(6)
            mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,block,velocity,0)
            metrics=contact_metrics(contacts,d.xipos[block],d.xmat[block])
            row=dict(step=count+1,stage=stage,phase=teacher.phase,time_s=float(d.time),
                ctrl=d.ctrl.tolist(),raw_qpos=d.qpos.tolist(),raw_qvel=d.qvel.tolist(),
                jaw_opening_m=jaw_gap_m(m,d,"left"),block_com_world_m=d.xipos[block].tolist(),
                block_rotation=d.xmat[block].reshape(3,3).tolist(),block_quaternion=d.xquat[block].tolist(),
                block_velocity_world=velocity.tolist(),block_translation_m=(d.xipos[block]-initial_com).tolist(),
                block_rotation_since_start_rad=float(2*np.arccos(np.clip(abs(d.xquat[block]@initial_quat),0,1))),
                tcp_position_world_m=d.site_xpos[teacher.site].tolist(),
                tcp_rotation=d.site_xmat[teacher.site].reshape(3,3).tolist(),
                contacts=contacts,normalized=metrics,warnings=d.warning.number.tolist())
            stream.write(json.dumps(row,allow_nan=False)+"\n");count+=1;last_row=row
            digest.update(np.asarray(d.ctrl,dtype=np.float64).tobytes())
        teacher.record_step=types.MethodType(record,teacher)
        opened=reference[5];closed=float(m.actuator_ctrlrange[5,0])
        schedule=np.linspace(opened,closed,max(2,math.ceil((opened-closed)/.01)+1))[1:]
        case["initial_contacts"]=contact_metrics(native_contacts(m,d),d.xipos[block],d.xmat[block])
        np.testing.assert_array_equal(integration_state(m,d),state)
        try:
            for stage,opening in enumerate(schedule[:close_stages],1):
                target=reference.copy();target[5]=opening
                teacher.move(target,duration=.1)
                case["stages"].append(dict(stage=stage,time_s=float(d.time),
                    command=target.tolist(),metrics=last_row["normalized"]))
                if stage%8==0 or stage>=46:
                    forces={p:v["summed_normal_force_N"] for p,v in last_row["normalized"]["pads"].items()}
                    print(json.dumps(dict(mode=MODE,noslip=setting,stage=stage,step=count,forces=forces)),flush=True)
            teacher.close_arm_reference=None;teacher.phase="GRASP_CONFIRM";teacher.env.collision_phase="GRASP_CONFIRM"
            for _ in range(50):
                teacher.env.apply_action(tuple(d.ctrl),physics_steps=1)
                teacher.record_step();checked=teacher.inspect_runtime()
                if not all(v>0 for v in checked["finger_force_N"].values()):
                    raise ValueError("bilateral contact did not persist through confirmation")
        except (ValueError,RuntimeError) as error:
            case["failure"]=dict(phase=teacher.phase,stage=stage,time_s=float(d.time),reason=str(error))
        finally:
            stream.close()
        case.update(physics_steps=count,final_state=integration_state(m,d).tolist(),last_row=last_row,
                    command_sha256=digest.hexdigest(),final_options=options(m))
        assert array_hashes(m)==expected_arrays and options(m)==case["options"]
        command_hashes.append(case["command_sha256"]);result["cases"][str(setting)]=case
        (output/"summary.json").write_text(json.dumps(result,indent=2)+"\n")
        print("CASE_DONE",setting,case["failure"],flush=True)
    result["identical_commands"]=command_hashes[0]==command_hashes[1]
    result["identical_compiled_arrays"]=True
    result["option_differences"]={k:[result["cases"]["0"]["options"][k],result["cases"]["5"]["options"][k]]
        for k in expected_options if result["cases"]["0"]["options"][k]!=result["cases"]["5"]["options"][k]}
    assert result["option_differences"]=={"noslip_iterations":[0,5]}
    (output/"summary.json").write_text(json.dumps(result,indent=2)+"\n")
    return result


def replay(directory, model_path):
    """Saved physics display in one window; no additional experiment/mj_step."""
    import time
    import mujoco.viewer
    cases={}
    for setting in (0,5):
        with (directory/f"noslip{setting}.jsonl").open() as stream:
            rows=[json.loads(line) for line in stream]
        cases[setting]=[r for r in rows if r["stage"]>=46][::10]+[rows[-1]]
    m=mujoco.MjModel.from_binary_path(str(model_path));d=mujoco.MjData(m)
    with mujoco.viewer.launch_passive(m,d,show_left_ui=False,show_right_ui=False) as viewer:
        viewer.cam.lookat[:]=[.2,0,.04];viewer.cam.distance=.38
        viewer.cam.azimuth=115;viewer.cam.elevation=-15
        start=time.monotonic()
        while viewer.is_running():
            elapsed=time.monotonic()-start;setting=(0,5)[int(elapsed/8)%2]
            rows=cases[setting];row=rows[min(len(rows)-1,int(elapsed%8/6*len(rows)))]
            metric=row["normalized"]
            with viewer.lock():
                d.qpos[:]=row["raw_qpos"];d.qvel[:]=row["raw_qvel"];d.ctrl[:]=row["ctrl"]
                mujoco.mj_forward(m,d);viewer.user_scn.ngeom=0
                for c in metric["contacts"]:
                    g=viewer.user_scn.geoms[viewer.user_scn.ngeom]
                    color=[1,.1,.1,1] if c["pad"]==PADS[0] else [1,1,.1,1]
                    mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.full(3,.002),np.array(c["position_world_m"]),np.eye(3).ravel(),np.array(color))
                    viewer.user_scn.ngeom+=1
            height=metric["first_native_height_world_m"]
            label="missing opposing contact" if height is None else f"{height*1000:.4f} mm"
            lines=[MODE,"RECORDED ctrl + mj_step / NOT NEW EXECUTION",f"NoSlip={setting} | {row['phase']} stage {row['stage']} | t={row['time_s']:.3f}",
                   f"Native contact height: {label}",
                   "Fn: "+str([round(metric["pads"][p]["summed_normal_force_N"],6) for p in PADS]),
                   "F_net N: "+str(np.round(metric["F_net_world_N"],6)),
                   "tau_COM r x f Nm: "+str(np.round(metric["tau_COM_r_cross_f_world_Nm"],6)),
                   "Same saved PRE-CLOSE state / same command prefix", "NoSlip5: unilateral at CONFIRM; runtime unchanged"]
            viewer.set_texts([(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,"\n".join(lines),"")])
            viewer.sync();time.sleep(.04)

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay",type=Path)
    parser.add_argument("--model",type=Path,required=True)
    for name in ("source","continuation","donor","output"):
        parser.add_argument("--"+name,type=Path)
    args=parser.parse_args()
    if args.replay:replay(args.replay,args.model)
    elif all((args.source,args.continuation,args.donor,args.output)):
        run(args.source,args.continuation,args.model,args.donor,args.output)
    else:parser.error("experiment requires --source --continuation --donor --output")
