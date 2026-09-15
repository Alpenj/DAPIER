#!/usr/bin/env python3
"""One center block teacher, SIM only; failures are recorded, never demonstrations."""
import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import mujoco
import numpy as np
from shoe_task import ShoeTaskEnv, MOBILE_BLOCK_CONFIG
from mobile_dual_so101 import actuator_targets_from_qpos, apply_control_as_pose
from physics_ik import solve_bimanual_position_ik, plan_septic_joint_trajectory
from collision_guard import check_bimanual_path, minimum_protected_clearance

PHASES = ("RESET","SETTLE","PREGRASP","APPROACH","CLOSE","GRASP_CONFIRM","LIFT","HOLD","SUCCESS","FAILURE")


class CenterBlockTeacher:
    def __init__(self):
        self.env = ShoeTaskEnv(replace(MOBILE_BLOCK_CONFIG, shoe_xy_range_m=0, shoe_yaw_range_rad=0))
        self.m, self.d = self.env.model, self.env.data
        self.phase = "RESET"
        self.report = dict(target_object="block", scene="mobile tower", success=False,
                           hardware_execution=False, transitions=[], plans=[], trace=[])
        self.block = self.m.geom("block_geom").id
        self.floor = self.m.geom("floor").id
        self.site = self.m.site("left_gripperframe").id
        self.fingers = {}
        self.arms = []
        for g in range(self.m.ngeom):
            if not self.m.geom_contype[g] or not self.m.geom_conaffinity[g]:
                continue
            if self.m.body(int(self.m.geom_bodyid[g])).name.startswith(("left_","right_")):
                self.arms.append(g)
            if self.m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                name = self.m.mesh(int(self.m.geom_dataid[g])).name
                for finger, mesh in (("fixed","left_wrist_roll_follower_so101_v1"),
                                     ("moving","left_moving_jaw_so101_v1")):
                    if name == mesh:
                        self.fingers[finger] = g
        if set(self.fingers) != {"fixed","moving"}:
            raise ValueError("missing physical left finger collision meshes")

    def transition(self, phase):
        if phase not in PHASES:
            raise ValueError("unknown phase")
        self.phase = phase
        self.report["transitions"].append(dict(phase=phase,time_s=float(self.d.time)))

    def snapshot(self):
        mujoco.mj_forward(self.m,self.d)
        force = dict(fixed=0., moving=0.)
        contacts = []
        for i,c in enumerate(self.d.contact):
            if self.block not in (c.geom1,c.geom2):
                continue
            wrench=np.zeros(6)
            mujoco.mj_contactForce(self.m,self.d,i,wrench)
            other=int(c.geom2 if c.geom1==self.block else c.geom1)
            for name,g in self.fingers.items():
                if g==other:
                    force[name]+=max(0.,float(wrench[0]))
            contacts.append(dict(other_geom=other,normal_force_N=float(wrench[0]),distance_m=float(c.dist)))
        gap,a,b=minimum_protected_clearance(self.m,self.d)
        floor_gap=min(float(mujoco.mj_geomDistance(self.m,self.d,g,self.floor,2.,None)) for g in self.arms)
        dof=int(self.m.joint("shoe_free").dofadr[0])
        return dict(phase=self.phase,time_s=float(self.d.time),target_q=self.d.ctrl.tolist(),
                    measured_q=list(actuator_targets_from_qpos(self.m,self.d.qpos)),
                    qpos=self.d.qpos.tolist(),qvel=self.d.qvel.tolist(),
                    minimum_clearance_m=gap,closest_pair=[a,b],arm_floor_clearance_m=floor_gap,
                    block_velocity=self.d.qvel[dof:dof+6].tolist(),finger_force_N=force,
                    contacts=contacts,metrics=self.env.metrics())

    def inspect_runtime(self):
        row=self.snapshot()
        if not np.isfinite(self.d.qpos).all() or not np.isfinite(self.d.qvel).all():
            raise ValueError("non-finite physics state")
        if any(w.number for w in self.d.warning):
            raise ValueError("MuJoCo physics warning")
        for j in range(self.m.njnt):
            if self.m.jnt_limited[j]:
                q=self.d.qpos[self.m.jnt_qposadr[j]]
                if not self.m.jnt_range[j,0] <= q <= self.m.jnt_range[j,1]:
                    raise ValueError("measured joint limit violation")
        if row["arm_floor_clearance_m"] < self.env.config.required_clearance_m:
            raise ValueError("arm-floor clearance violated")
        nonfinger_gap=minimum_protected_clearance(self.m,self.d,
            [(g,self.block) for g in self.arms if g not in self.fingers.values()])[0]
        if nonfinger_gap < self.env.config.required_clearance_m:
            raise ValueError("non-finger arm-block clearance violated")
        metrics=row["metrics"]
        if metrics["attachment_active"] or metrics["maximum_block_penetration_m"] > .001:
            raise ValueError("block attachment or excess penetration")
        forbidden=[c for c in row["contacts"] if c["normal_force_N"]>0
                   and c["other_geom"] not in (*self.fingers.values(),self.floor)]
        if forbidden:
            raise ValueError("forbidden non-finger block contact")
        if self.phase in ("LIFT","HOLD") and not all(v>0 for v in row["finger_force_N"].values()):
            raise ValueError("bilateral finger contact lost during lift/hold")
        return row

    def move(self, target, duration=.8):
        current=actuator_targets_from_qpos(self.m,self.d.qpos)
        assessment=check_bimanual_path(self.m,current,target,
                                       required_clearance_m=self.env.config.required_clearance_m)
        self.report["plans"].append(dict(phase=self.phase,target_q=list(target),
                                        full_path_guard=assessment.as_report()))
        if not assessment.safe:
            raise ValueError("full interpolated path rejected: "+assessment.reason)
        # Additional native plane check on the same joint interpolation; no sphere/plane bound.
        p=mujoco.MjData(self.m)
        p.qpos[:]=self.d.qpos
        count=max(1,math.ceil(np.max(np.abs(np.asarray(target)-current))/.025))
        for fraction in np.linspace(0,1,count+1):
            apply_control_as_pose(self.m,p,np.asarray(current)+(np.asarray(target)-current)*fraction)
            floor_gap=min(float(mujoco.mj_geomDistance(self.m,p,g,self.floor,2.,None)) for g in self.arms)
            if floor_gap < self.env.config.required_clearance_m:
                raise ValueError(f"interpolated arm-floor clearance {floor_gap} at {fraction}")
            for g in self.arms:
                gap=minimum_protected_clearance(self.m,p,[(g,self.block)])[0]
                required=(0.0 if g in self.fingers.values() else self.env.config.required_clearance_m)
                if self.phase in ("CLOSE","GRASP_CONFIRM","LIFT","HOLD") and g in self.fingers.values():
                    required=-.001
                if gap < required:
                    raise ValueError(f"interpolated arm-block clearance {gap} for geom {g}")
        trajectory=plan_septic_joint_trajectory(self.m,current,target,minimum_duration_s=duration)
        steps=math.ceil(trajectory.duration_s/self.m.opt.timestep)
        for step in range(steps):
            q,_,_,_=trajectory.sample((step+1)*self.m.opt.timestep)
            self.env.apply_action(q,physics_steps=1)
            row=self.inspect_runtime()
            if step%33==0 or step==steps-1:
                self.report["trace"].append(row)

    def solve(self, xyz):
        initial=actuator_targets_from_qpos(self.m,self.d.qpos)
        result=solve_bimanual_position_ik(self.m,initial,{"left":xyz},
                    tool_axis_targets={"left":self.approach},max_iterations=300)
        p=mujoco.MjData(self.m)
        apply_control_as_pose(self.m,p,result.action_rad)
        rot=p.site_xmat[self.site].reshape(3,3)
        closing_error=float(np.arccos(np.clip(rot[:,2]@self.closing,-1,1)))
        limits=all(not self.m.jnt_limited[j] or
            self.m.jnt_range[j,0] <= p.qpos[self.m.jnt_qposadr[j]] <= self.m.jnt_range[j,1]
            for j in range(self.m.njnt))
        candidate_path=check_bimanual_path(self.m,initial,result.action_rad,
                            required_clearance_m=self.env.config.required_clearance_m)
        self.report["plans"].append(dict(phase=self.phase,xyz=list(xyz),ik=asdict(result),
             approach_axis_world=rot[:,0].tolist(),closing_axis_world=rot[:,2].tolist(),
             closing_error_rad=closing_error,joint_limits=bool(limits),
             full_candidate_path_guard=candidate_path.as_report()))
        if not result.converged:
            raise ValueError("position/approach IK did not converge")
        if not limits:
            raise ValueError("IK endpoint violates joint limits")
        if closing_error > math.radians(15):
            raise ValueError("endpoint closing axis unsuitable for settled block face")
        return result.action_rad

    def run(self):
        try:
            self.transition("RESET")
            _,self.report["reset"]=self.env.reset(seed=0)
            self.report["trace"].append(self.snapshot())
            self.transition("SETTLE")
            self.report["settle"]=self.env.settle()
            self.report["trace"].append(self.snapshot())
            body=self.m.body("shoe").id
            position=self.d.xpos[body].copy()
            rotation=self.d.xmat[body].reshape(3,3).copy()
            site_rotation=self.d.site_xmat[self.site].reshape(3,3).copy()
            # Select a settled cube face aligned with the current finger plane.
            axes=[rotation[:,i]*sign for i in (0,1) for sign in (-1,1)]
            self.closing=max(axes,key=lambda a:float(a@site_rotation[:,2]))
            self.approach=-rotation[:,2]
            # Fixed distal surface is local Z=0. Cube center lies +20 mm across it.
            # Keep the fingertips above the unchanged 30 mm floor clearance.
            grasp=position-self.closing*.020-self.approach*.020
            pregrasp=grasp-self.approach*.060
            self.report["teacher_input"]=dict(block_position_m=position.tolist(),
                block_quaternion_wxyz=self.d.xquat[body].tolist(),
                block_bottom_height_m=self.env.settle_info["block_bottom_height_m"],
                measured_q=list(actuator_targets_from_qpos(self.m,self.d.qpos)),
                pregrasp_xyz=pregrasp.tolist(),grasp_xyz=grasp.tolist(),
                approach_axis_world=self.approach.tolist(),closing_axis_world=self.closing.tolist())
            self.report["geometry"]=dict(site="left_gripperframe",
                site_parent=self.m.body(int(self.m.site_bodyid[self.site])).name,
                site_local_pos=self.m.site_pos[self.site].tolist(),
                site_local_quat=self.m.site_quat[self.site].tolist(),
                approach_axis_local=[1,0,0],opening_axis_local=[0,0,1],
                closing_motion_local=[0,0,-1],site_world_rotation=site_rotation.tolist(),
                finger_geoms={name:dict(id=g,body=self.m.body(int(self.m.geom_bodyid[g])).name,
                  mesh=self.m.mesh(int(self.m.geom_dataid[g])).name)for name,g in self.fingers.items()})
            self.transition("PREGRASP")
            target=list(self.solve(pregrasp))
            target[5]=.6
            self.move(target)
            self.transition("APPROACH")
            for distance in np.linspace(.055,0,12):
                self.move(self.solve(grasp-self.approach*distance),duration=.3)
            self.transition("CLOSE")
            for opening in np.arange(.59,-.1745,-.01):
                target=list(actuator_targets_from_qpos(self.m,self.d.qpos))
                target[5]=float(opening)
                self.move(target,duration=.1)
                if all(v>0 for v in self.snapshot()["finger_force_N"].values()):
                    break
            self.transition("GRASP_CONFIRM")
            if not all(v>0 for v in self.snapshot()["finger_force_N"].values()):
                raise ValueError("bilateral positive finger contact not established; no lift")
            for _ in range(50):
                self.env.apply_action(tuple(self.d.ctrl),physics_steps=1)
                row=self.inspect_runtime()
                if not all(v>0 for v in row["finger_force_N"].values()):
                    raise ValueError("bilateral contact did not persist through confirmation")
            self.report["trace"].append(row)
            self.transition("LIFT")
            for height in (.005,.010,.020,.035):
                self.move(self.solve(grasp-self.approach*height),duration=.5)
            self.transition("HOLD")
            target=tuple(self.d.ctrl)
            for step in range(math.ceil(3.2/self.m.opt.timestep)):
                self.env.apply_action(target,physics_steps=1)
                row=self.inspect_runtime()
                if step%33==0 or row["metrics"]["success"]:
                    self.report["trace"].append(row)
                if row["metrics"]["success"]:
                    self.report["success"]=True
                    self.transition("SUCCESS")
                    break
            if not self.report["success"]:
                raise ValueError("continuous 3-second supported lift success condition not reached")
        except (ValueError,RuntimeError) as error:
            self.report["failure"]=dict(phase=self.phase,reason=str(error))
            self.transition("FAILURE")
            self.report["trace"].append(self.snapshot())
        self.report["final_phase"]=self.phase
        self.report["maximum_block_bottom_height_m"]=max(
            row["metrics"]["block_bottom_height_m"] for row in self.report["trace"])
        self.report["hold_duration_s"]=self.env.metrics()["continuous_hold_s"]
        reference=self.env.settle_info["block_bottom_height_m"] if self.env.settle_info else 0.
        self.report["maximum_lift_above_settled_bottom_m"]=max(
            [0.]+[row["metrics"]["block_bottom_height_m"]-reference
                  for row in self.report["trace"] if row["phase"] not in ("RESET","SETTLE")])
        return self.report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        raise ValueError("use a new output file; preserve prior evidence")
    report=CenterBlockTeacher().run()
    args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items()if k not in ("trace","plans")},indent=2))
    return 0 if report["success"] else 1


if __name__=="__main__":
    raise SystemExit(main())
