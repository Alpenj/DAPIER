#!/usr/bin/env python3
"""SIM-only settled block grasp inspection. P previews IK; never executes a path."""
import argparse
import copy
from dataclasses import asdict
import json
import math
from pathlib import Path
from queue import SimpleQueue
import time
import mujoco
import numpy as np
from center_block_teacher import CenterBlockTeacher
from mobile_dual_so101 import actuator_targets_from_qpos, apply_control_as_pose
from physics_ik import solve_bimanual_position_ik
from collision_guard import check_bimanual_path, minimum_protected_clearance

FACE_NAMES = ("+block X", "-block X", "+block Y", "-block Y")
CLOSING_TOLERANCE_RAD = math.radians(15)  # Existing teacher criterion.
STATE = mujoco.mjtState.mjSTATE_INTEGRATION


class GraspDebug:
    def __init__(self):
        self.teacher = CenterBlockTeacher()
        self.env = self.teacher.env
        self.env.reset(seed=0)
        self.settle = self.env.settle()
        self.model, self.data = self.env.model, self.env.data
        self.saved = np.empty(mujoco.mj_stateSize(self.model, STATE))
        mujoco.mj_getState(self.model, self.data, self.saved, STATE)
        # Viewer may edit its model/options/ctrl: isolate BOTH model and data.
        self.preview_model = copy.copy(self.model)
        self.preview_data = mujoco.MjData(self.preview_model)
        self.restore_preview()
        body = self.model.body("shoe").id
        self.position = self.data.xpos[body].copy()
        self.rotation = self.data.xmat[body].reshape(3,3).copy()
        self.home = actuator_targets_from_qpos(self.model, self.data.qpos)
        self.cache = {}

    def assert_unchanged(self):
        state = np.empty_like(self.saved)
        mujoco.mj_getState(self.model, self.data, state, STATE)
        if not np.array_equal(state,self.saved):
            raise AssertionError("settled simulation state was modified")

    def restore_preview(self):
        mujoco.mj_setState(self.preview_model,self.preview_data,self.saved,STATE)
        mujoco.mj_forward(self.preview_model,self.preview_data)

    def preview(self, candidate, *, requested=False):
        self.restore_preview()
        if requested:
            # May display a FAILED IK result for inspection; this is not execution.
            apply_control_as_pose(self.preview_model,self.preview_data,candidate["q_target"])
        self.assert_unchanged()

    def candidate(self, index, tilt_deg=0.0, tilt_sign=1):
        if index not in range(4) or not 0 <= tilt_deg <= 45 or tilt_sign not in (-1,1):
            raise ValueError("candidate 0..3; geometric tilt 0..45 degrees, sign +/-1")
        key=(index,float(tilt_deg),tilt_sign)
        if key in self.cache:
            return self.cache[key]
        closing=self.rotation[:,index//2]*(1 if index%2==0 else -1)
        down=-self.rotation[:,2]
        shoulder=self.data.xpos[self.model.body("left_shoulder").id]
        toward=shoulder-self.position
        toward-=closing*np.dot(toward,closing)
        toward-=down*np.dot(toward,down)
        toward/=np.linalg.norm(toward)
        direction=-toward*tilt_sign  # approach towards block, away from shoulder.
        angle=math.radians(tilt_deg)
        approach=down*math.cos(angle)+direction*math.sin(angle)
        grasp=self.position-closing*.020-approach*.020
        pregrasp=grasp-approach*.060
        ik=solve_bimanual_position_ik(self.model,self.home,{"left":pregrasp},
             tool_axis_targets={"left":approach},max_iterations=300)
        p=mujoco.MjData(self.model)
        mujoco.mj_setState(self.model,p,self.saved,STATE)
        apply_control_as_pose(self.model,p,ik.action_rad)
        rot=p.site_xmat[self.teacher.site].reshape(3,3)
        error=float(np.arccos(np.clip(rot[:,2]@closing,-1,1)))
        margins=[]
        for a,q in enumerate(ik.action_rad):
            j=int(self.model.actuator_trnid[a,0])
            low,high=self.model.jnt_range[j]
            margins.append(dict(joint=self.model.joint(j).name,q_rad=q,
                lower_rad=float(low),upper_rad=float(high),
                lower_margin_rad=float(q-low),upper_margin_rad=float(high-q)))
        limits=all(min(r["lower_margin_rad"],r["upper_margin_rad"])>=0 for r in margins)
        guard=check_bimanual_path(self.model,self.home,ik.action_rad,
                                 required_clearance_m=self.env.config.required_clearance_m)
        path_min=math.inf
        floor_min=math.inf
        block_min=math.inf
        count=max(1,math.ceil(np.max(np.abs(np.asarray(ik.action_rad)-self.home))/.025))
        for t in np.linspace(0,1,count+1):
            apply_control_as_pose(self.model,p,np.asarray(self.home)+t*(np.asarray(ik.action_rad)-self.home))
            path_min=min(path_min,minimum_protected_clearance(self.model,p)[0])
            floor_min=min(floor_min,*(float(mujoco.mj_geomDistance(
                self.model,p,g,self.teacher.floor,2.,None))for g in self.teacher.arms))
            block_min=min(block_min,minimum_protected_clearance(
                self.model,p,[(g,self.teacher.block) for g in self.teacher.arms])[0])
        orientation=ik.tool_axis_error_rad_by_side["left"]<=math.radians(2) and error<=CLOSING_TOLERANCE_RAD
        eligible=bool(ik.converged and limits and guard.safe and orientation
                      and ik.residual_m_by_side["left"]<=.0005
                      and min(floor_min,block_min)>=self.env.config.required_clearance_m)
        row=dict(index=index,face=FACE_NAMES[index],tilt_deg=tilt_deg,tilt_sign=tilt_sign,
            tilt_direction_world=direction.tolist(),pregrasp_xyz=pregrasp.tolist(),grasp_xyz=grasp.tolist(),
            approach_axis_world=approach.tolist(),closing_axis_world=closing.tolist(),
            q_target=list(ik.action_rad),ik=asdict(ik),closing_axis_error_rad=error,
            joint_limit_margins=margins,joint_limits_pass=limits,
            minimum_joint_limit_margin_rad=min(min(r["lower_margin_rad"],r["upper_margin_rad"]) for r in margins),
            path_guard=guard.as_report(),minimum_path_clearance_m=path_min,
            arm_floor_path_clearance_m=floor_min,arm_block_path_clearance_m=block_min,
            path_samples=count+1,eligible=eligible,preview_only=True,
            actual_motion_executed=False,hardware_execution=False)
        self.cache[key]=row
        self.assert_unchanged()
        return row


def markers(scene, debug, candidate):
    scene.ngeom=0
    def marker(kind,pos,color,label,end=None):
        geom=scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom,kind,np.array([.006]*3),np.asarray(pos),
                           np.eye(3).ravel(),np.asarray(color,dtype=np.float32))
        if end is not None:
            mujoco.mjv_connector(geom,kind,.003,np.asarray(pos),np.asarray(end))
        geom.label=label
        scene.ngeom+=1
    marker(mujoco.mjtGeom.mjGEOM_SPHERE,debug.position,[1,.5,0,.8],"settled block")
    marker(mujoco.mjtGeom.mjGEOM_SPHERE,debug.preview_data.site_xpos[debug.teacher.site],
           [0,1,0,1],"left gripper site")
    pre=np.asarray(candidate["pregrasp_xyz"])
    grasp=np.asarray(candidate["grasp_xyz"])
    marker(mujoco.mjtGeom.mjGEOM_SPHERE,pre,[0,.8,1,1],"PREGRASP")
    marker(mujoco.mjtGeom.mjGEOM_SPHERE,grasp,[1,0,1,1],"GRASP")
    marker(mujoco.mjtGeom.mjGEOM_ARROW,pre,[1,.2,.2,1],"approach +X",
           pre+.05*np.asarray(candidate["approach_axis_world"]))
    marker(mujoco.mjtGeom.mjGEOM_ARROW,grasp,[1,1,0,1],"finger gap +Z",
           grasp+.05*np.asarray(candidate["closing_axis_world"]))

def run_viewer(debug,index=0,tilt_deg=0.0,*,smoke_seconds=None):
    from mujoco import viewer as mj_viewer
    keys=SimpleQueue()
    candidate=debug.candidate(index,tilt_deg)
    print(json.dumps(candidate,indent=2),flush=True)
    preview_requested=False
    started=time.monotonic()
    try:
        with mj_viewer.launch_passive(debug.preview_model,debug.preview_data,key_callback=keys.put) as viewer:
            viewer.cam.lookat[:]=[-.13,.16,.19]
            viewer.cam.distance=.75
            viewer.cam.azimuth=135
            viewer.cam.elevation=-25
            while viewer.is_running():
                if smoke_seconds is not None and time.monotonic()-started>=smoke_seconds:
                    break
                changed=False
                quit_requested=False
                while not keys.empty():
                    key=keys.get()
                    if key in (ord("Q"),ord("q")):
                        quit_requested=True
                    elif ord("0")<=key<=ord("3"):
                        index=key-ord("0")
                        preview_requested=False
                        changed=True
                    elif key in (262,263):
                        index=(index+(1 if key==262 else -1))%4
                        preview_requested=False
                        changed=True
                    elif key in (ord("P"),ord("p")):
                        preview_requested=True
                        changed=True
                    elif key in (ord("H"),ord("h")):
                        preview_requested=False
                        changed=True
                if quit_requested:
                    break
                if changed:
                    candidate=debug.candidate(index,tilt_deg)
                    print(json.dumps(dict(candidate=candidate,ik_preview=preview_requested),indent=2),flush=True)
                with viewer.lock():
                    debug.preview(candidate,requested=preview_requested)
                    markers(viewer.user_scn,debug,candidate)
                viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    f"SIM DEBUG | {candidate['face']} | tilt {tilt_deg:g} deg",
                    f"IK {candidate['ik']['converged']} | eligible {candidate['eligible']}\n"
                    f"{'IK PREVIEW ONLY' if preview_requested else 'SETTLED HOME - NO MOTION'}\n"
                    "0..3 / arrows: candidate | P: preview | H: home | Q: quit\n"
                    "cyan pregrasp / magenta grasp / red approach / yellow finger gap")])
                viewer.sync()
                time.sleep(.03)
    finally:
        debug.restore_preview()
        debug.assert_unchanged()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer",action="store_true")
    parser.add_argument("--candidate",type=int,choices=range(4),default=0)
    parser.add_argument("--tilt-deg",type=float,default=0)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--viewer-smoke-seconds",type=float,help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.output and args.output.exists():
        parser.error("output exists; preserve prior report")
    debug=GraspDebug()
    report=debug.candidate(args.candidate,args.tilt_deg)
    if args.output:
        args.output.write_text(json.dumps(report,indent=2)+"\n")
    if args.viewer:
        run_viewer(debug,args.candidate,args.tilt_deg,smoke_seconds=args.viewer_smoke_seconds)
    else:
        print(json.dumps(report,indent=2))
    debug.assert_unchanged()


if __name__=="__main__":
    main()
