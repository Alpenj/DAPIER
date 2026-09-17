#!/usr/bin/env python3
"""One center block teacher, SIM only; failures are recorded, never demonstrations."""
import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import mujoco
import numpy as np
from shoe_task import (ShoeTaskEnv, MOBILE_BLOCK_CONFIG, target_body_name, measured_joint_state,
                       target_joint_name, target_geom_name, block_finger_geoms)
from mobile_dual_so101 import actuator_targets_from_qpos, apply_control_as_pose
from physics_ik import solve_bimanual_position_ik, plan_septic_joint_trajectory
from collision_guard import (check_bimanual_path, minimum_protected_clearance,
    structural_near_support_status, general_support_clearance, NEAR_SUPPORT_SCOPE,
    task_clearance_status)

PHASES = ("HOME","RESET","SETTLE","PREGRASP","APPROACH","CLOSE","GRASP_CONFIRM","LIFT","HOLD","SUCCESS","FAILURE")


class CenterBlockTeacher:
    def __init__(self, scene="desk"):
        if scene == "legacy_tower":
            self.env = ShoeTaskEnv(replace(MOBILE_BLOCK_CONFIG, shoe_xy_range_m=0, shoe_yaw_range_rad=0))
        else:
            from integration_scenes import task_env
            self.env = task_env(scene)
        self.m, self.d = self.env.model, self.env.data
        mujoco.mj_forward(self.m, self.d)
        self.observer = None
        self.phase = "HOME" if scene != "legacy_tower" else "RESET"
        self.report = dict(target_object="block", scene=scene, scene_id=self.env.config.scene_id,
                           success=False, hardware_execution=False,
                           transitions=[], plans=[], trace=[])
        if scene != "legacy_tower":
            from integration_scenes import task_provenance
            self.report["provenance"] = task_provenance(self.env)
        self.env.measured_state_trace = []
        self.report["measured_gripper_steps"] = self.env.measured_state_trace
        self.block = self.m.geom(target_geom_name(self.m)).id
        self.block_body = self.m.body(target_body_name(self.m)).id
        self.floor = self.env.support_geom  # legacy attribute: support may be a table.
        self.site = self.m.site(
            "left_cube_grasp" if scene == "desk" else "left_gripperframe").id
        self.fingers = block_finger_geoms(self.m)
        self.arms = [g for g in range(self.m.ngeom)
                     if self.m.geom_contype[g] and self.m.geom_conaffinity[g]
                     and self.m.body(int(self.m.geom_bodyid[g])).name.startswith(("left_", "right_"))]
        self.report["geometry"] = dict(
            support=self.m.geom(self.floor).name, ik_site=self.m.site(self.site).name,
            gripper_frame="left_gripperframe",
            initial_approach_axis_world=self.d.site_xmat[self.site].reshape(3,3)[:,0].tolist(),
            initial_jaw_separation_axis_world=self.d.site_xmat[self.site].reshape(3,3)[:,2].tolist(),
            finger_geoms={name: dict(id=g, name=self.m.geom(g).name,
                body=self.m.body(int(self.m.geom_bodyid[g])).name,
                geom_type=int(self.m.geom_type[g]), local_pos=self.m.geom_pos[g].tolist(),
                local_quat=self.m.geom_quat[g].tolist(), size=self.m.geom_size[g].tolist())
                for name, g in self.fingers.items()})

    def targets(self, position, closing, approach):
        # Desk cube_grasp already locates the cube center relative to its actual
        # fixed pad. Do not reuse the legacy mesh fingertip offset.
        grasp = (position.copy() if self.env.config.scene_id == "integration_desk"
                 else position - closing * .020 - approach * .020)
        return grasp - approach * .060, grasp

    def transition(self, phase):
        if phase not in PHASES:
            raise ValueError("unknown phase")
        self.phase = phase
        self.report["transitions"].append(dict(phase=phase,time_s=float(self.d.time)))
        if self.observer is not None:
            self.observer(phase)

    def snapshot(self):
        mujoco.mj_forward(self.m,self.d)
        force = dict.fromkeys(self.fingers, 0.)
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
            contacts.append(dict(other_geom=other,other_geom_name=self.m.geom(other).name,
                                 other_body=self.m.body(int(self.m.geom_bodyid[other])).name,normal_force_N=float(wrench[0]),distance_m=float(c.dist)))
        gap,a,b=minimum_protected_clearance(self.m,self.d)
        floor_gap=minimum_protected_clearance(self.m,self.d,[(g,self.floor) for g in self.arms])[0]
        dof=int(self.m.joint(target_joint_name(self.m)).dofadr[0])
        return dict(phase=self.phase,time_s=float(self.d.time),target_q=self.d.ctrl.tolist(),
                    measured_q=list(actuator_targets_from_qpos(self.m,self.d.qpos)),
                    qpos=self.d.qpos.tolist(),qvel=self.d.qvel.tolist(),
                    minimum_clearance_m=gap,closest_pair=[a,b],
                    closest_body_pair=[self.m.body(int(self.m.geom_bodyid[g])).name for g in (a,b)],
                    arm_floor_clearance_m=floor_gap,support_geom=self.m.geom(self.floor).name,
                    block_velocity=self.d.qvel[dof:dof+6].tolist(),finger_force_N=force,
                    contacts=contacts,
                    task_policy=(task_clearance_status(self.m,self.d,self.env.collision_phase)
                                 if self.env.collision_phase is not None else None),
                    protected_contacts=[dict(
                        geom_ids=[int(c.geom1), int(c.geom2)], distance_m=float(c.dist),
                        bodies=[self.m.body(int(self.m.geom_bodyid[g])).name
                                for g in (c.geom1,c.geom2)])
                        for c in self.d.contact if self.block not in (c.geom1,c.geom2)],
                    metrics=self.env.metrics(),
                    structural_near_support=structural_near_support_status(self.m,self.d))

    def inspect_runtime(self):
        row=self.snapshot()
        measured=actuator_targets_from_qpos(self.m,self.d.qpos)
        guard=check_bimanual_path(self.m,measured,measured,
                                 required_clearance_m=self.env.config.required_clearance_m,
                                 task_phase=self.env.collision_phase,reference_data=self.d)
        if not guard.safe:
            raise ValueError("measured physics state rejected: "+guard.reason)
        if not np.isfinite(self.d.qpos).all() or not np.isfinite(self.d.qvel).all():
            raise ValueError("non-finite physics state")
        if any(w.number for w in self.d.warning):
            raise ValueError("MuJoCo physics warning")
        measured_joint_state(self.m,self.d,integration_desk=self.env.config.scene_id=="integration_desk")
        if self.env.collision_phase is not None:
            status=task_clearance_status(self.m,self.d,self.env.collision_phase)
            if not status["safe"]:
                raise ValueError("measured general/task-specific clearance rejected")
        else:
            if general_support_clearance(self.m,self.d,self.arms,self.floor) < self.env.config.required_clearance_m:
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
        if (self.phase.startswith("LIFT") or self.phase == "HOLD") and not all(v>0 for v in row["finger_force_N"].values()):
            raise ValueError("bilateral finger contact lost during lift/hold")
        return row

    def move(self, target, duration=.8):
        current=actuator_targets_from_qpos(self.m,self.d.qpos)
        assessment=check_bimanual_path(self.m,current,target,
                                       required_clearance_m=self.env.config.required_clearance_m,
                                       task_phase=self.env.collision_phase,reference_data=self.d)
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
            if self.env.collision_phase is not None:
                if not task_clearance_status(self.m,p,self.env.collision_phase)["safe"]:
                    raise ValueError(f"interpolated task/general clearance rejected at {fraction}")
                continue
            floor_gap=general_support_clearance(self.m,p,self.arms,self.floor)
            if floor_gap < self.env.config.required_clearance_m:
                raise ValueError(f"interpolated arm-floor clearance {floor_gap} at {fraction}")
            for g in self.arms:
                gap=minimum_protected_clearance(self.m,p,[(g,self.block)])[0]
                required=(0.0 if g in self.fingers.values() else self.env.config.required_clearance_m)
                if self.phase in ("CLOSE","GRASP_CONFIRM","LIFT","HOLD") and g in self.fingers.values():
                    required=-.001
                if gap < required:
                    raise ValueError(f"interpolated arm-block clearance {gap} for geom {g}")
        command_start=np.asarray(current).copy()
        if getattr(self,"gripper_hold_reference",None) is not None:
            # A hold command starts from the preceding command, not measured solver error.
            # Measured state above still defines IK/path and runtime safety evidence.
            command_start[[5,11]]=self.d.ctrl[[5,11]]
        if self.phase=="CLOSE" and getattr(self,"close_arm_reference",None) is not None:
            # Measured state defines the physical/collision start, not the next
            # arm hold reference during gripper-only CLOSE.
            arm=[i for i in range(self.m.nu) if i not in (5,11)]
            command_start[arm]=np.asarray(self.close_arm_reference)[arm]
        trajectory=plan_septic_joint_trajectory(self.m,command_start,target,minimum_duration_s=duration)
        steps=math.ceil(trajectory.duration_s/self.m.opt.timestep)
        for step in range(steps):
            q,_,_,_=trajectory.sample((step+1)*self.m.opt.timestep)
            try:
                self.env.apply_action(q,physics_steps=1)
            finally:
                if hasattr(self,"record_step"):self.record_step()
            row=self.inspect_runtime()
            if step%33==0 or step==steps-1:
                self.report["trace"].append(row)

    def solve(self, xyz):
        initial=actuator_targets_from_qpos(self.m,self.d.qpos)
        result=solve_bimanual_position_ik(self.m,initial,{"left":xyz},
                    site_names={"left":self.m.site(self.site).name},
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
            if self.env.config.scene_id != "legacy_tower":
                self.transition("HOME")
                q = actuator_targets_from_qpos(self.m, self.d.qpos)
                guard = check_bimanual_path(self.m, q, q,
                    required_clearance_m=self.env.config.required_clearance_m)
                limits = all(not self.m.jnt_limited[j] or
                    self.m.jnt_range[j,0] <= self.d.qpos[self.m.jnt_qposadr[j]] <= self.m.jnt_range[j,1]
                    for j in range(self.m.njnt))
                self.report["home"] = dict(candidate="approved viewer model_default; newly checked",
                    joint_limits=bool(limits), target_q=list(q),
                    ctrl_error_rad=float(np.max(np.abs(self.d.ctrl-q))), guard=guard.as_report(),
                    structural_near_support=structural_near_support_status(self.m,self.d),
                    policy_scope=NEAR_SUPPORT_SCOPE)
                self.report["trace"].append(self.snapshot())
                if not limits or not guard.safe:
                    if not guard.safe:
                        from collision_diagnostics import describe_pair
                        self.report["home"]["pair_diagnostic"] = describe_pair(
                            self.m, self.d, guard.first_geom_id, guard.second_geom_id,
                            self.env.config.required_clearance_m)
                        print(json.dumps(self.report["home"]["pair_diagnostic"], indent=2), flush=True)
                    raise ValueError("HOME static validation failed: " + guard.reason)
            self.transition("RESET")
            _,self.report["reset"]=self.env.reset(seed=0)
            self.report["trace"].append(self.snapshot())
            self.transition("SETTLE")
            self.report["settle"]=self.env.settle()
            self.report["trace"].append(self.snapshot())
            body=self.block_body
            position=self.d.xpos[body].copy()
            rotation=self.d.xmat[body].reshape(3,3).copy()
            site_rotation=self.d.site_xmat[self.site].reshape(3,3).copy()
            # Select a settled cube face aligned with the current finger plane.
            axes=[rotation[:,i]*sign for i in (0,1) for sign in (-1,1)]
            self.closing=max(axes,key=lambda a:float(a@site_rotation[:,2]))
            self.approach=-rotation[:,2]
            pregrasp,grasp=self.targets(position,self.closing,self.approach)
            self.report["teacher_input"]=dict(block_position_m=position.tolist(),
                block_quaternion_wxyz=self.d.xquat[body].tolist(),
                block_bottom_height_m=self.env.settle_info["block_bottom_height_m"],
                measured_q=list(actuator_targets_from_qpos(self.m,self.d.qpos)),
                pregrasp_xyz=pregrasp.tolist(),grasp_xyz=grasp.tolist(),
                approach_axis_world=self.approach.tolist(),closing_axis_world=self.closing.tolist())
            self.report["geometry"].update(dict(site=self.m.site(self.site).name,
                site_parent=self.m.body(int(self.m.site_bodyid[self.site])).name,
                site_local_pos=self.m.site_pos[self.site].tolist(),
                site_local_quat=self.m.site_quat[self.site].tolist(),
                approach_axis_local=[1,0,0],opening_axis_local=[0,0,1],
                closing_motion_local=({"jaw_1":[0,0,1],"jaw_2":[0,0,-1]}
                    if "jaw_1" in self.fingers else [0,0,-1]),
                site_world_rotation=site_rotation.tolist(),
                finger_geoms={name:dict(id=g,body=self.m.body(int(self.m.geom_bodyid[g])).name,
                  geom_name=self.m.geom(g).name, geom_type=int(self.m.geom_type[g]),
                  mesh=(self.m.mesh(int(self.m.geom_dataid[g])).name
                        if self.m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH else None))for name,g in self.fingers.items()}))
            self.transition("PREGRASP")
            target=list(self.solve(pregrasp))
            is_parallel = "jaw_1" in self.fingers
            target[5]=float(self.m.actuator_ctrlrange[5,1]) if is_parallel else .6
            self.move(target)
            self.transition("APPROACH")
            for distance in np.linspace(.055,0,12):
                self.move(self.solve(grasp-self.approach*distance),duration=.3)
            self.finish_task(grasp)
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



    def finish_task(self, grasp):
        self.close_arm_reference=self.d.ctrl.copy()
        self.close_start_tcp=self.d.site_xpos[self.site].copy()
        self.close_contact_origin=None
        self.transition("CLOSE")
        # Opening-positive actuator radians; never reuse stock close=-.1745
        # on PGripper (range 0..2.2028). Use both physical jaw contacts.
        closed=float(self.m.actuator_ctrlrange[5,0])
        opened=float(self.d.ctrl[5])
        closings=np.linspace(opened,closed,max(2,math.ceil((opened-closed)/.01)+1))[1:]
        for opening in closings:
            # Measured contact response is physical evidence, not automatically
            # the next arm command reference. Hold the verified command across CLOSE.
            self.close_previous_command=self.d.ctrl.copy()
            self.close_previous_measured=np.asarray(actuator_targets_from_qpos(self.m,self.d.qpos))
            target=list(self.close_arm_reference if self.close_arm_reference is not None
                        else actuator_targets_from_qpos(self.m,self.d.qpos))
            target[5]=float(opening)
            preflight=None
            if hasattr(self,"step_telemetry"):
                from dynamic_preflight import full_state_preflight,integration_state
                preflight=full_state_preflight(self,[dict(eligible=True,phase="CLOSE",
                    xyz=np.asarray(grasp).tolist(),ik=dict(action_rad=target),duration_s=.1)])
                self.report.setdefault("close_preflights",[]).append(preflight)
                if not preflight["passed"]:
                    raise ValueError("CLOSE copied-state preflight: "+preflight["failure"]["reason"])
            self.move(target,duration=.1)
            if preflight is not None:
                identical=np.array_equal(integration_state(self.m,self.d),preflight["final_state"])
                self.report.setdefault("close_live_comparisons",[]).append(dict(bitwise_equal=bool(identical)))
                if not identical:raise ValueError("CLOSE preflight/live state mismatch")
            if all(v>0 for v in self.snapshot()["finger_force_N"].values()):
                break
        self.close_arm_reference=None
        self.transition("GRASP_CONFIRM")
        if not all(v>0 for v in self.snapshot()["finger_force_N"].values()):
            raise ValueError("bilateral positive finger contact not established; no lift")
        for _ in range(50):
            self.env.apply_action(tuple(self.d.ctrl),physics_steps=1)
            row=self.inspect_runtime()
            if not all(v>0 for v in row["finger_force_N"].values()):
                raise ValueError("bilateral contact did not persist through confirmation")
        self.report["trace"].append(row)
        if hasattr(self,"lift_targets"):
            for phase,height in self.lift_targets:
                self.transition(phase)
                self.move(self.solve(grasp+np.array([0.,0.,height])),duration=.5)
        else:
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



def run_with_viewer(teacher, output, *, hold_final=True, display_data=None, viewer_setup=None):
    """Observe the SAME model/data and controller as headless run; no preview."""
    import time
    from mujoco.viewer import launch_passive
    state_kind = mujoco.mjtState.mjSTATE_INTEGRATION
    before = np.empty(mujoco.mj_stateSize(teacher.m, state_kind))
    after = np.empty_like(before)
    print("SIM PHYSICS: ctrl + mj_step; qpos initialization only at RESET. "
          "This is NOT an IK teleport preview.", flush=True)
    viewed_data = teacher.d if display_data is None else display_data
    with launch_passive(teacher.m, viewed_data, show_left_ui=False, show_right_ui=False) as viewer:
        viewer.cam.lookat[:] = [.12, 0, .15]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.35, 135, -25
        last_phase = None
        last_wall = time.monotonic()
        last_sim = float(teacher.d.time)
        last_draw = 0.
        last_draw_sim = -float("inf")
        original_rgba = teacher.m.geom_rgba.copy()
        original_groups = viewer.opt.geomgroup.copy()
        if viewer_setup is not None:
            viewer_setup(viewer)

        def sync_readonly():
            if hasattr(teacher,"record_step") and teacher.env.collision_phase is not None:
                teacher.record_step()
            failure = teacher.report.get("failure")
            q = actuator_targets_from_qpos(teacher.m, teacher.d.qpos)
            fmt = lambda values: " ".join(f"{v:.3f}" for v in values)
            gap, a, b = minimum_protected_clearance(teacher.m, teacher.d)
            near = structural_near_support_status(teacher.m, teacher.d)
            forces = dict.fromkeys(teacher.fingers, 0.)
            block_contacts = []
            for i, c in enumerate(teacher.d.contact):
                if teacher.block in (c.geom1, c.geom2):
                    other = int(c.geom2 if c.geom1 == teacher.block else c.geom1)
                    block_contacts.append(teacher.m.geom(other).name or str(other))
                    for name, g in teacher.fingers.items():
                        if other == g:
                            wrench = np.zeros(6)
                            mujoco.mj_contactForce(teacher.m, teacher.d, i, wrench)
                            forces[name] += max(0., float(wrench[0]))
            metrics = teacher.env.metrics()
            home = teacher.report.get("home", {}).get("guard", {}).get("safe")
            top = [
                "SIM PHYSICS / LIVE TASK STATE | ctrl + mj_step",
                f"Phase: {teacher.phase} | SIM time: {teacher.d.time:.4f} s",
                f"HOME: {'PASS' if home else 'PENDING/FAIL'} | RESET: "
                f"{'PASS' if 'reset' in teacher.report else 'not completed'}",
                f"Target L: {fmt(teacher.d.ctrl[:6])}", f"Target R: {fmt(teacher.d.ctrl[6:])}",
                f"Measured L: {fmt(q[:6])}", f"Measured R: {fmt(q[6:])}",
            ]
            pair_name = lambda g: teacher.m.geom(g).name or teacher.m.body(int(teacher.m.geom_bodyid[g])).name
            lift = ("N/A before SETTLE" if not teacher.env.settle_info else
                    f"{(metrics['block_bottom_height_m']-teacher.env.settle_info['block_bottom_height_m'])*1000:.3f} mm")
            state = [f"Live minimum: {gap*1000:.4f} mm",
                     f"Closest: {a}:{pair_name(a)} <-> {b}:{pair_name(b)}",
                     f"Block contacts: {','.join(block_contacts) or 'none'}",
                     f"Finger normal force N: {forces}",
                     f"Lift: {lift} | HOLD: {teacher.env._block_hold_s:.3f} s",
                     f"Success: {teacher.report['success']}"]
            if failure:
                state.extend([f"STOPPED at {failure['phase']}", failure["reason"]])
            policy = []
            if near:
                policy = ["STRUCTURAL NEAR-SUPPORT",
                          "SIM ONLY / INTEGRATION DESK / HARDWARE UNVERIFIED",
                          "Exact shoulder/table installation relationship",
                          "30 mm exempt ONLY for these two tested pairs"]
                for row in near:
                    policy.extend([
                        f"{row['side']} {row['pair']}: {row['clearance_m']*1000:.6f} mm",
                        f"nominal {row['nominal_compiled_clearance_m']*1000:.6f} mm | "
                        f"contacts {len(row['contacts'])} | penetration {row['penetration']}",
                    ])
                policy.append("Other arm/table: GENERAL OBSTACLE / 30 mm")
                with viewer.lock():
                    for row in near:
                        g, table = row["pair"]
                        teacher.m.geom_rgba[g] = [1., .03, .03, 1.]
                        teacher.m.geom_rgba[table] = [1., .85, .05, 1.]
                        viewer.opt.geomgroup[teacher.m.geom_group[g]] = 1
            box_panel = []
            if teacher.report["scene_id"] == "integration_desk":
                housing = teacher.m.geom("left_pgripper_housing").id
                details = []
                minimum_protected_clearance(teacher.m, teacher.d,
                    [(housing, teacher.floor)], diagnostics=details)
                row = details[0]
                box_panel = ["BOX-BOX: left_pgripper_housing / table",
                    f"Native: {row['native_signed_distance_m']*1000:.3f} mm",
                    f"SAT certified lower bound: {row['certified_box_box_separation_lower_bound_m']*1000:.3f} mm",
                    f"Final: {row['final_distance_m']*1000:.3f} / required 30 mm"]
            if teacher.env.collision_phase is not None:
                status=task_clearance_status(teacher.m,teacher.d,teacher.env.collision_phase)
                box_panel=[f"GENERAL: {status['general_clearance_m']*1000:.3f} / 30 mm"]
                for row in status["pairs"]:
                    if row["classification"] != "INTENDED_FINGER_CONTACT":
                        box_panel.append(f"{row['names'][0].replace('left_pgripper_', '')}/{row['names'][1]}: {row['clearance_m']*1000:.3f} mm")
                if getattr(teacher,"waypoint_xyz",None) is not None:
                    error=np.linalg.norm(teacher.d.site_xpos[teacher.site]-teacher.waypoint_xyz)
                    axis=teacher.d.site_xmat[teacher.site].reshape(3,3)[:,0]
                    angle=math.degrees(math.acos(float(np.clip(axis@teacher.approach,-1,1))))
                    box_panel.append(f"TCP err {error*1000:.3f} mm | axis err {angle:.3f} deg")
            for a in (5, 11):
                j = int(teacher.m.actuator_trnid[a, 0])
                measured = float(teacher.d.qpos[int(teacher.m.jnt_qposadr[j])])
                excess = max(0., measured-float(teacher.m.jnt_range[j,1]),
                             float(teacher.m.jnt_range[j,0])-measured)
                box_panel.append(f"{teacher.m.actuator(a).name}: cmd {teacher.d.ctrl[a]:.7f} "
                                 f"meas {measured:.10f} excess {excess:.2e}")
            if getattr(teacher,"step_telemetry",[]):
                telemetry=teacher.step_telemetry[-1]
                box_panel=[f"Target/FK {telemetry['target_fk_clearance_m']*1000:.6f} mm",
                    f"Measured {telemetry['measured_clearance_m']*1000:.6f} mm",
                    f"Tracking {telemetry['maximum_tracking_error_rad']:.7f} rad",
                    "Pair "+" / ".join(telemetry["closest_pair_names"])]
                if "measured_approach_error_rad" in telemetry:
                    box_panel.append(f"Axis cmd/FK {math.degrees(telemetry['command_fk_approach_error_rad']):.5f} / measured {math.degrees(telemetry['measured_approach_error_rad']):.5f} deg")
                box_panel += [f"{name}: {value['qpos']:.8f}" for name,value in telemetry["passive_jaw_state"].items()]
                selected=teacher.report.get("staging_search",{}).get("selected")
                box_panel.append("Dynamic preflight: "+("PASS" if selected and selected.get("dynamic_preflight",{}).get("passed") else "PENDING"))
            if teacher.phase=="CLOSE" and getattr(teacher,"step_telemetry",[]):
                r=teacher.step_telemetry[-1]
                if "planned_tcp_error_m" in r:
                    box_panel=[r["arm_reference_source"],
                        f"TCP planned/measured: {r['planned_tcp_error_m']*1000:.6f} / {r['executed_tcp_error_m']*1000:.6f} mm",
                        f"TCP drift: {(r['cumulative_tcp_drift_m'] or 0)*1000:.6f} mm",
                        f"Arm reference drift: {r['arm_reference_drift_rad']} rad",
                        f"Arm tracking: {r['maximum_arm_tracking_error_rad']:.8f} rad",
                        f"Clearance: {r['measured_clearance_m']*1000:.6f} mm"]
            if getattr(teacher,"step_telemetry",[]) and "contact_state" in teacher.step_telemetry[-1]:
                r=teacher.step_telemetry[-1]
                state=[f"Contact {r['contact_state']} | finger forces {r['finger_force_N']} N",
                    "Opposite gap mm: "+str({k:round(v*1000,4) for k,v in r['opposite_finger_gap_m'].items()}),
                    "Block displacement mm: "+str([round(v*1000,4) for v in r['block_translation_since_contact_m']]),
                    f"Block yaw {math.degrees(r['block_yaw_rad']):.4f} deg",
                    f"Lift {lift} | HOLD {teacher.env._block_hold_s:.3f} s",
                    "Failure: "+(failure["reason"] if failure else "none")]
            if getattr(teacher,"step_telemetry",[]) and teacher.step_telemetry[-1].get("grippers"):
                policy=["GRIPPER STATE / raw, never clamped"]
                for g in teacher.step_telemetry[-1]["grippers"]:
                    policy.extend([f"{g['side']}: cmd {g['command']:.10f} meas {g['raw_qpos']:.10f}",
                        f"excess {g['upper_excess_rad']:.2e} / tol {g['numerical_tolerance_rad']:.1e}",
                        f"cmd margin {g['command_upper_margin_rad']:.2e} rad | jaw {g['jaw_opening_m']*1000:.5f} mm"])
            texts = [(mujoco.mjtFont.mjFONT_NORMAL, grid, "\n".join(lines), "")
                     for grid, lines in ((mujoco.mjtGridPos.mjGRID_TOPLEFT, top),
                                         (mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, state),
                                         (mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT, policy),
                                         (mujoco.mjtGridPos.mjGRID_TOPRIGHT, box_panel)) if lines]
            if any(len(t[2]) >= 500 for t in texts):
                raise RuntimeError("viewer task text exceeds renderer capacity")
            if hasattr(teacher,"draw_waypoint_markers"):
                teacher.draw_waypoint_markers(viewer)
            viewer.set_texts(texts)
            mujoco.mj_getState(teacher.m, teacher.d, before, state_kind)
            if viewed_data is not teacher.d:
                # The GUI observes a snapshot; its inputs cannot change live physics.
                mujoco.mj_setState(teacher.m, viewed_data, before, state_kind)
                mujoco.mj_forward(teacher.m, viewed_data)
            viewer.sync()
            mujoco.mj_getState(teacher.m, teacher.d, after, state_kind)
            if not np.array_equal(before, after):
                raise RuntimeError("viewer modified simulation inputs/state; execution stopped")

        def observe(phase):
            nonlocal last_phase, last_wall, last_sim, last_draw, last_draw_sim
            if hasattr(teacher,"record_step"):
                teacher.record_step()
            phase_changed = phase != last_phase
            if phase_changed:
                print(f"[SIM PHYSICS] phase={phase} t={teacher.d.time:.6f}s", flush=True)
                last_phase = phase
                print(json.dumps(dict(phase=phase, time_s=float(teacher.d.time),
                    target_q=teacher.d.ctrl.tolist(),
                    measured_q=list(actuator_targets_from_qpos(teacher.m, teacher.d.qpos)),
                    clearance_m=minimum_protected_clearance(teacher.m, teacher.d)[0],
                    contact_count=int(teacher.d.ncon))), flush=True)
            if not viewer.is_running():
                if phase != "FAILURE":
                    raise RuntimeError("viewer closed; teacher stopped")
                return
            now = time.monotonic()
            delta = float(teacher.d.time) - last_sim
            if delta > 0:
                time.sleep(max(0., min(delta, delta - (now-last_wall))))
            last_wall, last_sim = time.monotonic(), float(teacher.d.time)
            if phase_changed or (last_wall-last_draw >= 1/30 and teacher.d.time-last_draw_sim >= .05):
                sync_readonly()
                last_draw = time.monotonic()
                last_draw_sim = float(teacher.d.time)

        teacher.observer = observe
        teacher.env.physics_observer = lambda: observe(teacher.phase)
        try:
            report = teacher.run()
            output.write_text(json.dumps(report, indent=2)+"\n")
            print(json.dumps(dict(final_phase=report["final_phase"],
                                  failure=report.get("failure"), output=str(output))), flush=True)
            if hold_final:
                print("Final state held with physics stopped; close the viewer to exit.", flush=True)
            while hold_final and viewer.is_running():
                sync_readonly()
                time.sleep(.03)
        finally:
            teacher.m.geom_rgba[:] = original_rgba
            viewer.opt.geomgroup[:] = original_groups
            teacher.observer = None
            teacher.env.physics_observer = None
    return report

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--scene",choices=("desk","mobile","legacy_tower"),default="desk")
    parser.add_argument("--viewer",action="store_true",help="observe actual ctrl + mj_step, not pose preview")
    args=parser.parse_args()
    if args.output.exists():
        raise ValueError("use a new output file; preserve prior evidence")
    teacher=CenterBlockTeacher(scene=args.scene)
    report=run_with_viewer(teacher,args.output) if args.viewer else teacher.run()
    args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items()if k not in ("trace","plans")},indent=2))
    return 0 if report["success"] else 1


if __name__=="__main__":
    raise SystemExit(main())
