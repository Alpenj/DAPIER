#!/usr/bin/env python3
"""Bounded paired PREGRASP/GRASP tilt audit, SIM diagnostic only."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import mujoco
import numpy as np
from center_block_teacher import CenterBlockTeacher
from physics_ik import solve_bimanual_position_ik, axis_direction_task
from mobile_dual_so101 import apply_control_as_pose
from collision_guard import check_bimanual_path, minimum_protected_clearance, general_support_clearance
from pregrasp_ik_diagnostic import show

def search(source,output):
    saved=json.loads(Path(source).read_text());t=CenterBlockTeacher("desk");m,d=t.m,t.d
    if saved["provenance"]["model_sha256"] != t.report["provenance"]["model_sha256"]:
        raise ValueError("model differs from settled source")
    state=saved["trace"][-1]
    d.qpos[:]=state["qpos"];d.qvel[:]=state["qvel"];d.ctrl[:]=state["target_q"]
    d.time=state["time_s"];mujoco.mj_forward(m,d)
    original=d.qpos.copy()
    inp=saved["teacher_input"];pre=np.array(inp["pregrasp_xyz"]);grasp=np.array(inp["grasp_xyz"])
    down=np.array(inp["approach_axis_world"]);home=np.array(inp["measured_q"])
    faces=np.array([d.xmat[t.block_body].reshape(3,3)[:,i]*sign for i in (0,1) for sign in (-1,1)])
    local=[]
    for g in t.fingers.values():
        mesh=int(m.geom_dataid[g]);a=int(m.mesh_vertadr[mesh])
        v=m.mesh_vert[a:a+int(m.mesh_vertnum[mesh])]
        world=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
        local.extend((world-d.site_xpos[t.site])@d.site_xmat[t.site].reshape(3,3))
    local=np.array(local)
    # Conservative open-pad envelope for every roll/azimuth: axial*cos + radial*sin.
    # This bounds physical non-penetration only; the unchanged 30 mm guard is separate.
    axial=float(np.max(np.abs(local[:,0])));radial=float(np.max(np.linalg.norm(local[:,1:],axis=1)))
    table_top=float(d.geom_xpos[t.floor,2]+m.geom_size[t.floor,2])
    height=float(grasp[2]-table_top)
    lo,hi=0.,math.atan2(radial,axial)
    for _ in range(60):
        mid=(lo+hi)/2
        if axial*math.cos(mid)+radial*math.sin(mid)<=height:lo=mid
        else:hi=mid
    max_tilt=math.degrees(lo)
    joints=m.actuator_trnid[:5,0].astype(int);dofs=m.jnt_dofadr[joints];ranges=m.jnt_range[joints]
    qseeds=[home,np.array(saved["plans"][0]["ik"]["action_rad"])]
    candidates=[]
    def endpoint(xyz,axis,seed):
        # 3 position + 2 axis DoF; no roll goal is added to the 5DoF IK.
        ik=solve_bimanual_position_ik(m,seed,{"left":xyz},site_names={"left":m.site(t.site).name},
            tool_axis_targets={"left":axis},axis_formulation="axis_direction",max_iterations=300)
        p=mujoco.MjData(m);p.qpos[:]=d.qpos;apply_control_as_pose(m,p,ik.action_rad)
        rot=p.site_xmat[t.site].reshape(3,3)
        jp=np.zeros((3,m.nv));jw=jp.copy();mujoco.mj_jacSite(m,p,jp,jw,t.site)
        J=np.vstack((jp[:,dofs],.05*axis_direction_task(rot[:,0],axis,jw[:,dofs])[0]))
        sv=np.linalg.svd(J,compute_uv=False);q=np.array(ik.action_rad)
        margins=np.minimum(q[:5]-ranges[:,0],ranges[:,1]-q[:5])
        close_errors=np.arccos(np.clip(faces@rot[:,2],-1,1));face=int(np.argmin(close_errors))
        guard=check_bimanual_path(m,q,q,required_clearance_m=.03)
        floor=general_support_clearance(m,p,t.arms,t.floor)
        block=minimum_protected_clearance(m,p,[(g,t.block) for g in t.arms if g not in t.fingers.values()])[0]
        fingers=minimum_protected_clearance(m,p,[(g,t.block) for g in t.fingers.values()])[0]
        return dict(ik=asdict(ik),q=q.tolist(),xyz=xyz.tolist(),
            position_error_m=float(np.linalg.norm(p.site_xpos[t.site]-xyz)),
            approach_error_rad=float(np.arccos(np.clip(rot[:,0]@axis,-1,1))),
            actual_axis=rot[:,0].tolist(),fk=p.site_xpos[t.site].tolist(),
            lower_margin=(q[:5]-ranges[:,0]).tolist(),upper_margin=(ranges[:,1]-q[:5]).tolist(),
            joint_margins=[dict(joint=m.joint(int(m.actuator_trnid[a,0])).name,
                lower=float(q[a]-m.jnt_range[int(m.actuator_trnid[a,0]),0]),
                upper=float(m.jnt_range[int(m.actuator_trnid[a,0]),1]-q[a])) for a in range(m.nu)],
            limiting_joint=m.joint(int(joints[np.argmin(margins)])).name,minimum_margin=float(margins.min()),
            jacobian_rank=int(np.linalg.matrix_rank(J)),singular_values=sv.tolist(),
            closing_axis=rot[:,2].tolist(),closing_face=face,closing_error_rad=float(close_errors[face]),
            endpoint_guard=guard.as_report(),support_gap_m=floor,nonfinger_block_gap_m=block,
            finger_block_gap_m=fingers)
    def evaluate(tilt,azimuth,stage):
        if any(abs(c["tilt"]-tilt)<1e-8 and abs(c["azimuth"]-azimuth)<1e-8 for c in candidates):return
        theta,phi=np.radians([tilt,azimuth])
        axis=down*math.cos(theta)+np.array([math.cos(phi),math.sin(phi),0.])*math.sin(theta)
        branches=[]
        for seed_id,seed in enumerate(qseeds):
            a=endpoint(pre,axis,seed);b=endpoint(grasp,axis,a["q"])
            approach=check_bimanual_path(m,a["q"],b["q"],required_clearance_m=.03)
            home_path=check_bimanual_path(m,home,a["q"],required_clearance_m=.03)
            kinematic=all(x["ik"]["converged"] and x["position_error_m"]<=.0005 and
                x["approach_error_rad"]<=math.radians(2) and x["minimum_margin"]>=0 for x in (a,b))
            closing=all(x["closing_error_rad"]<=math.radians(15) for x in (a,b)) and a["closing_face"]==b["closing_face"]
            safe=all(x["endpoint_guard"]["safe"] and x["support_gap_m"]>=.03 and
                x["nonfinger_block_gap_m"]>=.03 and x["finger_block_gap_m"]>=0 for x in (a,b))
            # Additional block-clearance sampling is needed only if the general path passed.
            block_path=True
            if approach.safe and home_path.safe:
                p=mujoco.MjData(m);p.qpos[:]=d.qpos
                for start,end in ((home,a["q"]),(a["q"],b["q"])):
                    start,end=np.array(start),np.array(end)
                    n=max(1,math.ceil(np.max(np.abs(end-start))/.025))
                    for f in np.linspace(0,1,n+1):
                        apply_control_as_pose(m,p,start+(end-start)*f)
                        for g in t.arms:
                            gap=minimum_protected_clearance(m,p,[(g,t.block)])[0]
                            if gap<(0. if g in t.fingers.values() else .03):block_path=False
            branches.append(dict(seed_id=seed_id,pregrasp=a,grasp=b,approach_guard=approach.as_report(),
                home_guard=home_path.as_report(),block_path_checked=bool(approach.safe and home_path.safe),
                block_path_pass=block_path,kinematic_pass=kinematic,closing_pass=closing,
                eligible=bool(kinematic and closing and safe and approach.safe and home_path.safe and block_path)))
        best=min(branches,key=lambda b:(not b["eligible"],not b["kinematic_pass"],
            sum(x["position_error_m"]/.0005+x["approach_error_rad"]/math.radians(2) for x in (b["pregrasp"],b["grasp"]))))
        candidates.append(dict(tilt=tilt,azimuth=azimuth,stage=stage,axis=axis.tolist(),branches=branches,best=best))
        print(json.dumps(dict(candidate=len(candidates)-1,tilt=tilt,azimuth=azimuth,
            pre_mm=best["pregrasp"]["position_error_m"]*1000,grasp_mm=best["grasp"]["position_error_m"]*1000,
            kinematic=best["kinematic_pass"],eligible=best["eligible"])),flush=True)
    angles=sorted(set([0.,max_tilt/3,2*max_tilt/3,max_tilt]))
    for angle in angles:
        for azimuth in ([0.] if angle==0 else range(0,360,45)):evaluate(angle,float(azimuth),"coarse")
    # Refine the best measured residual neighborhood; no claim of a global optimum.
    best=min(candidates,key=lambda c:(not c["best"]["eligible"],
        sum(x["position_error_m"]/.0005+x["approach_error_rad"]/math.radians(2)
            for x in (c["best"]["pregrasp"],c["best"]["grasp"]))))
    for tilt in (max(0.,best["tilt"]-max_tilt/6),best["tilt"],min(max_tilt,best["tilt"]+max_tilt/6)):
        for az in ((best["azimuth"]-22.5)%360,best["azimuth"],(best["azimuth"]+22.5)%360):
            evaluate(tilt,az,"refine")
    selected=sorted((c for c in candidates if c["best"]["eligible"]),
        key=lambda c:(c["tilt"],-min(c["best"][p]["minimum_margin"] for p in ("pregrasp","grasp")),
            -min(c["best"][p]["support_gap_m"] for p in ("pregrasp","grasp"))))
    results=[]
    for i,c in enumerate(candidates):
        for phase in ("pregrasp","grasp"):
            e=c["best"][phase]
            results.append(dict(seed=f"{i} {phase} tilt={c['tilt']:.2f} az={c['azimuth']:.1f}",
                formulation="axis_direction",objective=phase,ik=e["ik"],eligible=c["best"]["eligible"],
                final=dict(e,iteration=e["ik"]["iterations"]),guard=c["best"]["approach_guard"],
                target=e["xyz"],desired_axis=c["axis"],pregrasp_error=c["best"]["pregrasp"]["position_error_m"],
                grasp_error=c["best"]["grasp"]["position_error_m"]))
    report=dict(mode="KINEMATIC IK DIAGNOSTIC / NOT PHYSICS",provenance=t.report["provenance"],
        target=pre.tolist(),grasp_xyz=grasp.tolist(),desired_axis=down.tolist(),closing_axis=faces[0].tolist(),
        geometry_bound=dict(axial_m=axial,radial_m=radial,available_height_m=height,max_tilt_deg=max_tilt,
            description="sufficient all-roll open-pad nonpenetration envelope, NOT global feasible cone; general30mm unchanged"),
        candidates=candidates,selected=selected[0] if selected else None,results=results)
    assert np.array_equal(original,d.qpos)
    Path(output).write_text(json.dumps(report,indent=2))
    return t,report

def main():
    p=argparse.ArgumentParser();p.add_argument("--source",required=True);p.add_argument("--output",required=True)
    p.add_argument("--viewer",action="store_true");a=p.parse_args()
    if Path(a.output).exists():raise FileExistsError(a.output)
    t,r=search(a.source,a.output)
    if a.viewer:
        review=next((i for i,c in enumerate(r["candidates"])
                     if c["best"]["kinematic_pass"] and c["best"]["closing_pass"]),0)
        show(t,r,index=2*review+1)
if __name__=="__main__":main()
