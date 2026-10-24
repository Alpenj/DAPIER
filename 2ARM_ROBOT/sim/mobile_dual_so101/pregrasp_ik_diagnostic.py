#!/usr/bin/env python3
"""Bounded SIM-only IK decomposition; preview is never physics execution."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
from queue import SimpleQueue
import time
from types import SimpleNamespace
import mujoco
import numpy as np
from center_block_teacher import CenterBlockTeacher
from physics_ik import solve_bimanual_position_ik, axis_direction_task
from mobile_dual_so101 import apply_control_as_pose
from collision_guard import check_bimanual_path
from grasp_debug import markers

def diagnose(source, axis_ab=False):
    saved=json.loads(Path(source).read_text())
    t=CenterBlockTeacher("desk")
    if saved["provenance"]["model_sha256"] != t.report["provenance"]["model_sha256"]:
        raise ValueError("saved settled model hash differs")
    m,d=t.m,t.d
    state=saved["trace"][-1]
    d.qpos[:]=state["qpos"];d.qvel[:]=state["qvel"];d.ctrl[:]=state["target_q"]
    d.time=state["time_s"];mujoco.mj_forward(m,d)
    before=d.qpos.copy()
    inp=saved["teacher_input"];target=np.array(inp["pregrasp_xyz"])
    desired=np.array(inp["approach_axis_world"]);closing=np.array(inp["closing_axis_world"])
    site=t.site
    joints=m.actuator_trnid[:5,0].astype(int);dofs=m.jnt_dofadr[joints]
    ranges=m.jnt_range[joints]
    home=np.array(inp["measured_q"])
    seeds=[("settled_HOME",home.copy()),("previous_failed",np.array(saved["plans"][0]["ik"]["action_rad"]))]
    for name,delta in [("HOME_elbow_plus",[0,-.15,.2,-.15,0]),
                       ("HOME_elbow_minus",[0,.15,-.2,.15,0]),
                       ("elbow_alternative",[.4,-.5,.8,.5,0])]:
        q=home.copy();q[:5]+=delta
        if np.any(q[:5]<ranges[:,0]) or np.any(q[:5]>ranges[:,1]):
            raise ValueError("diagnostic seed outside limits; never clamp")
        seeds.append((name,q))
    def describe_site(i):
        return dict(name=m.site(i).name,parent=m.body(int(m.site_bodyid[i])).name,
            local_position=m.site_pos[i].tolist(),local_quaternion=m.site_quat[i].tolist(),
            world_position=d.site_xpos[i].tolist(),world_rotation=d.site_xmat[i].reshape(3,3).tolist())
    pads=[]
    for g in t.fingers.values():
        mesh=int(m.geom_dataid[g]);start=int(m.mesh_vertadr[mesh])
        v=m.mesh_vert[start:start+int(m.mesh_vertnum[mesh])]
        world=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
        pads.append(dict(name=m.geom(g).name,parent=m.body(int(m.geom_bodyid[g])).name,
            type=int(m.geom_type[g]),mesh=m.mesh(mesh).name,
            world_min=world.min(0).tolist(),world_max=world.max(0).tolist()))
    inner=[describe_site(m.site(f"left_pgripper_pad_{i}_inner").id) for i in (1,2)]
    pinch=np.mean([x["world_position"] for x in inner],axis=0)
    # Opposed pad inner sites are generated from compiled CAD-tip bounds; both jaws move.
    geometry=dict(tcp=describe_site(site),gripperframe=describe_site(m.site("left_gripperframe").id),
        jaw_contact_surfaces=pads,inner_face_sites=inner,pinch_center_world=pinch.tolist(),
        tcp_minus_pinch_world=(d.site_xpos[site]-pinch).tolist(),
        opening_axis_world=((np.array(inner[1]["world_position"])-inner[0]["world_position"])/
            np.linalg.norm(np.array(inner[1]["world_position"])-inner[0]["world_position"])).tolist(),
        stock_offset_used=False,position_source="PGripper CAD opposing inner-face midpoint; orientation inherited gripperframe")
    results=[]
    for name,seed in seeds:
        for formulation in (("raw_rotation", "axis_direction") if axis_ab else ("position_only", "raw_rotation")):
            with_axis = formulation != "position_only"
            trace=[]
            def observe(iteration,q,p):
                jp=np.zeros((3,m.nv));jr=jp.copy();mujoco.mj_jacSite(m,p,jp,jr,site)
                axis=p.site_xmat[site].reshape(3,3)[:,0]
                J=jp[:,dofs]
                if with_axis:
                    Jaxis = (axis_direction_task(axis,desired,jr[:,dofs])[0]
                             if formulation == "axis_direction" else jr[:,dofs])
                    J=np.vstack((J,.05*Jaxis))
                singular=np.linalg.svd(J,compute_uv=False)
                margins=np.minimum(q[:5]-ranges[:,0],ranges[:,1]-q[:5])
                trace.append(dict(iteration=iteration,q=q.tolist(),fk=p.site_xpos[site].tolist(),
                    actual_axis=axis.tolist(),position_error_m=float(np.linalg.norm(target-p.site_xpos[site])),
                    approach_error_rad=float(np.arccos(np.clip(axis@desired,-1,1))),
                    lower_margin=(q[:5]-ranges[:,0]).tolist(),upper_margin=(ranges[:,1]-q[:5]).tolist(),
                    limiting_joint=m.joint(int(joints[np.argmin(margins)])).name,
                    minimum_margin=float(margins.min()),jacobian_rank=int(np.linalg.matrix_rank(J)),
                    singular_values=singular.tolist()))
            ik=solve_bimanual_position_ik(m,seed,{"left":target},site_names={"left":m.site(site).name},
                tool_axis_targets={"left":desired} if with_axis else None,
                max_iterations=300,iteration_observer=observe,
                axis_formulation=formulation if with_axis else "raw_rotation")
            final=trace[-1]
            preview=mujoco.MjData(m);apply_control_as_pose(m,preview,ik.action_rad)
            closing_error=float(np.arccos(np.clip(preview.site_xmat[site].reshape(3,3)[:,2]@closing,-1,1)))
            guard=check_bimanual_path(m,home,ik.action_rad,required_clearance_m=.03)
            near=next((row for row in trace if row["minimum_margin"]<=1e-5),None)
            eligible=bool(ik.converged and final["position_error_m"]<=.0005 and
                final["approach_error_rad"]<=math.radians(2) and final["minimum_margin"]>=0 and
                closing_error<=math.radians(15) and guard.safe)
            row=dict(seed=name,formulation=formulation,objective="position+approach" if with_axis else "position-only",
                seed_q=seed.tolist(),ik=asdict(ik),final=final,trace=trace,
                first_limit_approach=near,closing_error_rad=closing_error,
                guard=guard.as_report(),eligible=eligible)
            results.append(row)
            print(json.dumps({k:row[k] for k in ("seed","objective","final","eligible")}),flush=True)
    if not np.array_equal(d.qpos,before):raise AssertionError("diagnostic changed settled state")
    return t,dict(mode="KINEMATIC IK DIAGNOSTIC / NOT PHYSICS",source=str(source),
        provenance=t.report["provenance"],target=target.tolist(),desired_axis=desired.tolist(),
        closing_axis=closing.tolist(),grasp_xyz=inp["grasp_xyz"],geometry=geometry,results=results)

def show(t,report,index=0):
    from mujoco import viewer
    keys=SimpleQueue()
    p=mujoco.MjData(t.m)
    state_kind=mujoco.mjtState.mjSTATE_INTEGRATION
    state=np.empty(mujoco.mj_stateSize(t.m,state_kind))
    mujoco.mj_getState(t.m,t.d,state,state_kind)
    with viewer.launch_passive(t.m,p,key_callback=keys.put) as v:
        v.cam.lookat[:]=[.2,0,.15];v.cam.distance=.9;v.cam.azimuth=130;v.cam.elevation=-30
        while v.is_running():
            while not keys.empty():
                key=keys.get()
                if key in (ord("N"),ord("n")):index=(index+1)%len(report["results"])
                if key in (ord("B"),ord("b")):index=(index-1)%len(report["results"])
            row=report["results"][index];last=row["final"]
            mujoco.mj_setState(t.m,p,state,state_kind)
            apply_control_as_pose(t.m,p,row["ik"]["action_rad"])
            proxy=SimpleNamespace(position=t.d.xpos[t.block_body],preview_data=p,teacher=t)
            candidate=dict(pregrasp_xyz=row.get("target",report["target"]),grasp_xyz=report["grasp_xyz"],
                approach_axis_world=row.get("desired_axis",report["desired_axis"]),closing_axis_world=report["closing_axis"])
            with v.lock():
                markers(v.user_scn,proxy,candidate)
                for candidate in report.get("candidates", []):
                    g=v.user_scn.geoms[v.user_scn.ngeom]
                    origin=np.array(report["grasp_xyz"])
                    color=([0,1,0,.7] if candidate["best"]["eligible"] else
                           [1,.6,0,.7] if candidate["best"]["kinematic_pass"] and candidate["best"]["closing_pass"]
                           else [1,0,0,.35])
                    mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_ARROW,np.ones(3)*.002,
                        origin,np.eye(3).ravel(),np.array(color,dtype=np.float32))
                    mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_ARROW,.001,origin,
                        origin-.08*np.array(candidate["axis"]))
                    v.user_scn.ngeom+=1
                for start,end,color,label in [
                    (p.site_xpos[t.site],np.array(row.get("target",report["target"])),[1,1,1,1],"position error"),
                    (p.site_xpos[t.site],p.site_xpos[t.site]+.05*np.array(last["actual_axis"]),[0,1,0,1],"actual approach")]:
                    g=v.user_scn.geoms[v.user_scn.ngeom]
                    mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_ARROW,np.ones(3)*.003,
                        np.array(start),np.eye(3).ravel(),np.array(color,dtype=np.float32))
                    mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_ARROW,.002,np.array(start),np.array(end))
                    g.label=label;v.user_scn.ngeom+=1
            text=(f"KINEMATIC IK DIAGNOSTIC / NOT PHYSICS\nN/B: next/previous candidate {index}\n"
                f"{row['seed']} | {row.get('formulation', row['objective'])}\nIteration {last['iteration']} | eligible {row['eligible']}\n"
                f"Position {last['position_error_m']*1000:.4f} mm | approach {math.degrees(last['approach_error_rad']):.3f} deg\n"
                f"Limit: {last['limiting_joint']} | margin {last['minimum_margin']:.6f} rad\n"
                f"Wrist flex {last['q'][3]:.6f} | upper margin {last['upper_margin'][3]:.3e}\n"
                f"Converged {row['ik']['converged']} | rank {last['jacobian_rank']} | "
                f"smin {min(last['singular_values']):.3e} | path {row['guard']['safe']}")
            if "pregrasp_error" in row:
                text += f"\nPRE {row['pregrasp_error']*1000:.3f} / GRASP {row['grasp_error']*1000:.3f} mm"
            v.set_texts([(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,text,"")])
            v.sync();time.sleep(.05)

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--source",required=True)
    parser.add_argument("--output",required=True);parser.add_argument("--viewer",action="store_true")
    parser.add_argument("--axis-ab",action="store_true")
    args=parser.parse_args()
    if Path(args.output).exists():raise FileExistsError(args.output)
    t,r=diagnose(args.source,axis_ab=args.axis_ab);Path(args.output).write_text(json.dumps(r,indent=2))
    if args.viewer:show(t,r)
if __name__=="__main__":main()
