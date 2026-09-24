#!/usr/bin/env python3
"""SIM-only center block waypoints with explicit phase-scoped collision policy."""
import argparse
from dataclasses import asdict, replace
import json
import hashlib
import math
from pathlib import Path
import numpy as np
import mujoco
from center_block_teacher import CenterBlockTeacher,run_with_viewer
from mobile_dual_so101 import actuator_targets_from_qpos
from physics_ik import solve_bimanual_position_ik
from collision_guard import check_bimanual_path,task_clearance_status,NEAR_SUPPORT_SCOPE
from shoe_task import measured_joint_state

class WaypointBlockTeacher(CenterBlockTeacher):
    def __init__(self, candidate, staging_reference=None, approach_reference=None):
        super().__init__("desk")
        self.approach=np.asarray(candidate["axis"],float)
        self.report["candidate"]=candidate
        self.report["task_policy_scope"]=NEAR_SUPPORT_SCOPE
        for name in ("waypoint_block_teacher.py","integration_manipulation_pairs.json","dynamic_preflight.py"):
            self.report["provenance"]["source_sha256"][name]=hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
        self.lift_targets=(("LIFT_5MM",.005),("LIFT_15MM",.015),("LIFT_30MM",.035))
        self.step_telemetry=[]
        self.report["execution_telemetry"]=self.step_telemetry
        self.command_preview=None
        self.dynamic_observer=None
        self.last_telemetry_print=None
        self.execution_mode="SIM PHYSICS / LIVE TASK STATE"
        self.staging_reference=staging_reference
        self.axis_reserve=None
        if approach_reference is not None:
            for key in ("scene_id","model_sha256","asset_sha256","gripper_revision"):
                if approach_reference["provenance"][key]!=self.report["provenance"][key]:
                    raise ValueError("approach evidence geometry changed: "+key)
            from dynamic_preflight import measured_axis_reserve
            self.axis_reserve=measured_axis_reserve(self.m,self.site,approach_reference)
            self.report["axis_planning_reserve"]=self.axis_reserve
        self.gripper_hold_reference=None
        self.previous_ik=None
        self.waypoint_xyz=None
        self.planning_observer=None
        self.execute_plan=True
        self.waypoint_markers={}
        self.env.physics_observer=self.record_step
    def record_step(self):
        from dynamic_preflight import record_step
        record_step(self)

    def transition(self,phase):
        self.phase=phase
        # Separate large translation, alignment and proximity admission; a timer
        # never authorizes contact. General clearance still guards unrelated links.
        if phase not in ("FAILURE","SUCCESS"):
            self.env.collision_phase=(None if phase in ("HOME","RESET","SETTLE") else phase)
        self.report["transitions"].append(dict(phase=phase,time_s=float(self.d.time)))
        if self.observer is not None:self.observer(phase)
    def draw_waypoint_markers(self, viewer):
        with viewer.lock():
            viewer.user_scn.ngeom=0
            colors={"HOME":[1,1,1,1],"SAFE_STAGE":[0,1,1,1],"ALIGN_HIGH":[0,1,1,1],
                    "PREGRASP":[1,.5,0,1],"GRASP":[1,0,1,1]}
            for name,point in self.waypoint_markers.items():
                if name=="ALIGN_HIGH":continue  # Same XYZ as SAFE_STAGE, distinct orientation task.
                g=viewer.user_scn.geoms[viewer.user_scn.ngeom]
                mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.ones(3)*.004,
                    np.asarray(point),np.eye(3).ravel(),np.asarray(colors[name],np.float32))
                g.label="SAFE_STAGE / ALIGN_HIGH" if name=="SAFE_STAGE" else name
                viewer.user_scn.ngeom+=1

    def evaluate_waypoint(self, xyz, seed, phase, *, axis_tolerance_rad=math.radians(2)):
        """Plan only in private data; every segment starts at its predecessor q."""
        from mobile_dual_so101 import apply_control_as_pose
        xyz=np.asarray(xyz,float)
        alignment=phase!="SAFE_STAGE"
        ik=solve_bimanual_position_ik(self.m,seed,{"left":xyz},
            site_names={"left":self.m.site(self.site).name},
            tool_axis_targets={"left":self.approach} if alignment else None,
            axis_formulation="axis_direction",tool_axis_tolerance_rad=axis_tolerance_rad,max_iterations=300)
        if self.gripper_hold_reference is not None:
            command=list(ik.action_rad)
            command[5],command[11]=self.gripper_hold_reference
            ik=replace(ik,action_rad=tuple(command))
        p=mujoco.MjData(self.m);p.qpos[:]=self.d.qpos
        apply_control_as_pose(self.m,p,ik.action_rad)
        rot=p.site_xmat[self.site].reshape(3,3)
        approach=float(np.arccos(np.clip(rot[:,0]@self.approach,-1,1)))
        closing=float(np.arccos(np.clip(rot[:,2]@self.closing,-1,1)))
        error=float(np.linalg.norm(p.site_xpos[self.site]-xyz))
        margins=[dict(name=self.m.joint(int(self.m.actuator_trnid[a,0])).name,
            lower=float(q-self.m.jnt_range[int(self.m.actuator_trnid[a,0]),0]),
            upper=float(self.m.jnt_range[int(self.m.actuator_trnid[a,0]),1]-q))
            for a,q in enumerate(ik.action_rad)]
        valid=all(min(r["lower"],r["upper"])>=0 for r in margins)
        guard=check_bimanual_path(self.m,seed,ik.action_rad,
            task_phase=phase,reference_data=self.d,max_joint_step_rad=.025)
        status=task_clearance_status(self.m,p,phase)
        from collision_guard import minimum_protected_clearance
        gaps=dict(pad_block=minimum_protected_clearance(self.m,p,[(g,self.block) for g in self.fingers.values()])[0],
            pad_table=minimum_protected_clearance(self.m,p,[(g,self.floor) for g in self.fingers.values()])[0],
            housing_block=minimum_protected_clearance(self.m,p,[(self.m.geom("left_pgripper_housing").id,self.block)])[0])
        eligible=bool(ik.converged and error<=.0005 and valid and guard.safe and status["safe"]
            and (not alignment or (approach<=math.radians(2) and closing<=math.radians(15))))
        path_min=math.inf
        if guard.safe:
            for fraction in np.linspace(0.,1.,guard.checked_samples):
                apply_control_as_pose(self.m,p,np.asarray(seed)+(np.asarray(ik.action_rad)-seed)*fraction)
                path_min=min(path_min,task_clearance_status(self.m,p,phase)["general_clearance_m"])
        row=dict(phase=phase,xyz=xyz.tolist(),seed_q=list(seed),ik=asdict(ik),
            position_error_m=error,approach_error_rad=approach,closing_error_rad=closing,
            joint_margins=margins,guard=guard.as_report(),task_policy=status,
            geometric_clearances_m=gaps,path_general_minimum_m=path_min if guard.safe else None,eligible=eligible)
        return row

    def configure_task_open(self):
        from pgripper import jaw_gap_m
        from mobile_dual_so101 import apply_control_as_pose
        p=mujoco.MjData(self.m);p.qpos[:]=self.d.qpos
        segments=self.staging_reference["staging_search"]["selected"]["segments"]
        widths=[]
        for start,end in zip(segments[1:],segments[2:]):
            a=np.asarray(start["ik"]["action_rad"]);b=np.asarray(end["ik"]["action_rad"])
            samples=max(2,math.ceil(float(np.max(np.abs(b-a)))/.025)+1)
            for fraction in np.linspace(0,1,samples):
                apply_control_as_pose(self.m,p,a+(b-a)*fraction)
                axis=p.site_xmat[self.site].reshape(3,3)[:,2]
                rotation=p.geom_xmat[self.block].reshape(3,3)
                widths.append(float(2*np.sum(self.m.geom_size[self.block]*np.abs(rotation.T@axis))))
        q=np.asarray(segments[-1]["ik"]["action_rad"]).copy()
        low,high=self.m.actuator_ctrlrange[5]
        q[5]=low;apply_control_as_pose(self.m,p,q);gap_low=jaw_gap_m(self.m,p,"left")
        q[5]=high;apply_control_as_pose(self.m,p,q);gap_high=jaw_gap_m(self.m,p,"left")
        # Reserve the existing 0.5 mm TCP tolerance on both sides of the block.
        # Divide remaining opening slack equally between fit and command-boundary reserve.
        required=max(widths)+2*.0005
        if gap_high<=required or gap_high<=gap_low:
            raise ValueError("no task opening fits block plus existing TCP tolerance")
        chosen=(required+gap_high)/2
        reference=float(low+(high-low)*(chosen-gap_low)/(gap_high-gap_low))
        self.gripper_hold_reference=(reference,reference)
        self.report["task_open"]=dict(reference_rad=reference,source="compiled jaw/block geometry; explicit hold reference",
            maximum_projected_block_width_m=max(widths),tcp_tolerance_m=.0005,required_opening_m=required,
            maximum_opening_m=gap_high,selected_opening_m=chosen,command_upper_margin_rad=float(high-reference),
            residual_fit_margin_m=chosen-required,scope="SIM_ONLY / fixed sampled approach; not hardware robustness")
        print(json.dumps(dict(task_open=self.report["task_open"])),flush=True)

    def staging_plan(self, pregrasp, grasp, *, require_dynamic=False):
        from mobile_dual_so101 import apply_control_as_pose
        # TCP-only waypoints do not bound the full gripper. Search retreat using
        # swept-geometry gates; align the tool before admitting target proximity.
        home=np.asarray(actuator_targets_from_qpos(self.m,self.d.qpos))
        if self.gripper_hold_reference is None:home[[5,11]]=self.d.ctrl[[5,11]]
        p=mujoco.MjData(self.m);p.qpos[:]=self.d.qpos
        apply_control_as_pose(self.m,p,self.report["candidate"]["best"]["pregrasp"]["q"])
        gripper=[g for g in self.arms if self.m.geom(g).name.startswith("left_pgripper")]
        radius=max(float(np.linalg.norm(p.geom_xpos[g]-p.site_xpos[self.site])+self.m.geom_rbound[g]) for g in gripper)
        bound=radius+float(np.linalg.norm(self.m.geom_size[self.block]))+self.env.config.required_clearance_m
        direction=-self.approach
        audit=dict(direction=direction.tolist(),gripper_tcp_bound_m=radius,
            block_radius_m=float(np.linalg.norm(self.m.geom_size[self.block])),
            search_bound_m=bound,resolution_m=.0005,trials=[],selected=None,
            limitation="bounded sampled 1D retreat; not a global minimum proof")
        audit["dynamic_scope"]=["SAFE_STAGE","ALIGN_HIGH"] if require_dynamic else []
        self.report["staging_search"]=audit
        self.waypoint_markers={"HOME":self.d.site_xpos[self.site].tolist(),
            "PREGRASP":list(pregrasp),"GRASP":list(grasp)}
        def trial(offset):
            stage=pregrasp+direction*offset
            self.waypoint_markers.update(SAFE_STAGE=stage.tolist(),ALIGN_HIGH=stage.tolist())
            sequence=[];seed=home.copy()
            targets=[("SAFE_STAGE",stage),("ALIGN_HIGH",stage),("PREGRASP_NEAR",pregrasp)]
            for phase,xyz in targets:
                row=self.evaluate_waypoint(xyz,seed,phase);sequence.append(row)
                if self.planning_observer:self.planning_observer(row)
                if not row["eligible"]:break
                seed=np.array(row["ik"]["action_rad"])
            if len(sequence)==3 and sequence[-1]["eligible"]:
                apply_control_as_pose(self.m,p,seed)
                minimum_z=math.inf
                for g in self.fingers.values():
                    mesh=int(self.m.geom_dataid[g]);a=int(self.m.mesh_vertadr[mesh])
                    vertices=self.m.mesh_vert[a:a+int(self.m.mesh_vertnum[mesh])]
                    world=vertices@p.geom_xmat[g].reshape(3,3).T+p.geom_xpos[g]
                    minimum_z=min(minimum_z,float(world[:,2].min()))
                top=float(p.geom_xpos[self.floor,2]+self.m.geom_size[self.floor,2])
                coarse=np.asarray(grasp).copy()
                coarse[2]=max(grasp[2],top+self.env.config.required_clearance_m+
                    float(p.site_xpos[self.site,2]-minimum_z))
                for phase,xyz in (("APPROACH_COARSE",coarse),("APPROACH_FINE",grasp)):
                    row=self.evaluate_waypoint(xyz,seed,phase);sequence.append(row)
                    if self.planning_observer:self.planning_observer(row)
                    if not row["eligible"]:break
                    seed=np.array(row["ik"]["action_rad"])
            result=dict(offset_m=float(offset),segments=sequence,
                        eligible=len(sequence)==5 and all(r["eligible"] for r in sequence))
            result["kinematic_pass"]=result["eligible"]
            if require_dynamic and result["kinematic_pass"]:
                from dynamic_preflight import full_state_preflight
                result["dynamic_preflight"]=full_state_preflight(self,sequence[:2])
                result["eligible"]=result["dynamic_preflight"]["passed"]
            audit["trials"].append(result)
            print(json.dumps(dict(mode="KINEMATIC WAYPOINT DIAGNOSTIC / NOT PHYSICS",
                offset_m=offset,last_phase=sequence[-1]["phase"],eligible=result["eligible"],
                general_clearance_m=sequence[-1]["task_policy"]["general_clearance_m"])),flush=True)
            return result
        if self.staging_reference is not None:
            saved=self.staging_reference
            for key in ("scene_id","model_sha256","asset_sha256","gripper_revision"):
                if saved["provenance"][key]!=self.report["provenance"][key]:
                    raise ValueError("saved staging geometry changed: "+key)
            audit["search_repeated"]=False
            result=trial(saved["staging_search"]["selected"]["offset_m"])
            audit["selected"]=result
            if not result["eligible"]:raise ValueError("fixed retreat with task open target failed validation")
            return result["segments"]
        previous=0.;best=None
        for offset in np.linspace(0.,bound,25):
            result=trial(float(offset))
            if result["eligible"]:
                best=result;break
            previous=float(offset)
        if best is not None:
            lo,hi=previous,best["offset_m"]
            while hi-lo>audit["resolution_m"]:
                mid=(lo+hi)/2;result=trial(mid)
                if result["eligible"]:hi=mid;best=result
                else:lo=mid
            audit["selected"]=best
            audit["last_rejected_offset_m"]=lo
            # Restore the selected markers/preview after the last refinement failure.
            stage=pregrasp+direction*best["offset_m"]
            self.waypoint_markers.update(SAFE_STAGE=stage.tolist(),ALIGN_HIGH=stage.tolist())
            if self.planning_observer:
                for row in best["segments"]:self.planning_observer(row)
            return best["segments"]
        raise ValueError("no complete staging/approach chain passed within geometry-derived retreat bound")

    def solve(self,xyz,*,position_tolerance_m=.0005):
        # Reserve part of the unchanged 0.5 mm measured endpoint gate for
        # tracking error when a caller requests a more precise command pose.
        if not math.isfinite(position_tolerance_m) or not 0 < position_tolerance_m <= .0005:
            raise ValueError("command position tolerance must be positive and at most 0.5 mm")
        xyz=np.asarray(xyz,float);self.waypoint_xyz=xyz
        seed=actuator_targets_from_qpos(self.m,self.d.qpos)
        seed=list(seed)
        # Carry the actual closed command into lift IK; the previous arm branch stays fixed.
        seed[5],seed[11]=float(self.d.ctrl[5]),float(self.d.ctrl[11])
        alignment=self.phase!="SAFE_STAGE"
        ik=solve_bimanual_position_ik(self.m,seed,{"left":xyz},
            site_names={"left":self.m.site(self.site).name},
            tool_axis_targets={"left":self.approach} if alignment else None,
            axis_formulation="axis_direction",max_iterations=300,tolerance_m=position_tolerance_m)
        # Roll is not an IK target: position3 + axis2 fit the arm's five DoF.
        # Closing-face alignment remains an independent acceptance gate.
        p=mujoco.MjData(self.m);p.qpos[:]=self.d.qpos
        from mobile_dual_so101 import apply_control_as_pose
        apply_control_as_pose(self.m,p,ik.action_rad)
        closing=float(np.arccos(np.clip(p.site_xmat[self.site].reshape(3,3)[:,2]@self.closing,-1,1)))
        margins=measured_joint_state(self.m,p,integration_desk=True)
        axis=p.site_xmat[self.site].reshape(3,3)[:,0]
        approach_error=float(np.arccos(np.clip(axis@self.approach,-1,1)))
        guard=check_bimanual_path(self.m,actuator_targets_from_qpos(self.m,self.d.qpos),ik.action_rad,
            task_phase=self.env.collision_phase,reference_data=self.d)
        status=task_clearance_status(self.m,p,self.env.collision_phase)
        self.report["plans"].append(dict(phase=self.phase,xyz=xyz.tolist(),seed_q=list(seed),ik=asdict(ik),
            closing_error_rad=closing,approach_error_rad=approach_error,
            measured_joint_validation=margins,
            joint_margins=[dict(name=self.m.joint(int(self.m.actuator_trnid[a,0])).name,
                lower=float(q-self.m.jnt_range[int(self.m.actuator_trnid[a,0]),0]),
                upper=float(self.m.jnt_range[int(self.m.actuator_trnid[a,0]),1]-q))
                for a,q in enumerate(ik.action_rad)],
            guard=guard.as_report(),task_policy=status))
        if not ik.converged:raise ValueError("waypoint position/axis IK did not converge")
        if alignment and closing>math.radians(15):raise ValueError("waypoint closing-axis criterion failed")
        if not guard.safe:
            raise ValueError(f"path rejected: {self.m.geom(guard.first_geom_id).name} / "
                             f"{self.m.geom(guard.second_geom_id).name}; "
                             f"{guard.minimum_clearance_m*1000:.3f} < {guard.required_clearance_m*1000:.3f} mm")
        if not status["safe"]:raise ValueError("waypoint endpoint task policy failed")
        self.previous_ik=ik.action_rad
        return ik.action_rad
    def move(self,target,duration=.8):
        if self.gripper_hold_reference is not None:
            target=list(target)
            target[11]=self.gripper_hold_reference[1]
            if self.phase not in ("CLOSE","GRASP_CONFIRM","LIFT","LIFT_5MM","LIFT_15MM","LIFT_30MM","HOLD"):
                target[5]=self.gripper_hold_reference[0]
        super().move(target,duration)
        if self.waypoint_xyz is not None:
            error=float(np.linalg.norm(self.d.site_xpos[self.site]-self.waypoint_xyz))
            axis=self.d.site_xmat[self.site].reshape(3,3)
            approach=float(np.arccos(np.clip(axis[:,0]@self.approach,-1,1)))
            closing=float(np.arccos(np.clip(axis[:,2]@self.closing,-1,1)))
            self.report.setdefault("waypoint_gates",[]).append(dict(phase=self.phase,time_s=float(self.d.time),
                position_error_m=error,approach_error_rad=approach,closing_error_rad=closing,
                measured_q=list(actuator_targets_from_qpos(self.m,self.d.qpos)),
                status=task_clearance_status(self.m,self.d,self.env.collision_phase)))
            if error>.0005:raise ValueError("measured waypoint position tolerance failed")
            if self.phase!="SAFE_STAGE" and (approach>math.radians(2) or closing>math.radians(15)):
                raise ValueError("measured waypoint orientation tolerance failed")
    def run(self):
        try:
            self.transition("HOME")
            home=actuator_targets_from_qpos(self.m,self.d.qpos)
            guard=check_bimanual_path(self.m,home,home)
            self.report["home"]=dict(guard=guard.as_report())
            self.report["trace"].append(self.snapshot())
            if not guard.safe:raise ValueError("HOME guard failed")
            self.transition("RESET");_,self.report["reset"]=self.env.reset(seed=0)
            self.transition("SETTLE");self.report["settle"]=self.env.settle()
            self.report["trace"].append(self.snapshot())
            grasp=self.d.xpos[self.block_body].copy()
            rotation=self.d.xmat[self.block_body].reshape(3,3)
            self.closing=rotation[:,0]  # Proven +block X face of the audited tilt candidate.
            high=grasp+rotation[:,2]*.060  # Existing pregrasp stand-off, not a new height.
            self.report["teacher_input"]=dict(grasp_xyz=grasp.tolist(),pregrasp_xyz=high.tolist(),
                settled_quaternion=self.d.xquat[self.block_body].tolist(),approach=self.approach.tolist())
            if self.staging_reference is not None:
                self.configure_task_open()
            self.transition("STAGING_DIAGNOSTIC")
            sequence=self.staging_plan(high,grasp,require_dynamic=True)
            self.report["preflight_complete"]=True
            for row in sequence if self.execute_plan else []:
                self.transition(row["phase"])
                endpoint_preflight=None
                if row["phase"]=="PREGRASP_NEAR" and self.axis_reserve is not None:
                    # Acceptance stays at 2 degrees; observed tracking deviation sets only the IK target.
                    # Replan from measured state and authorize this segment only after copied physics.
                    seed=actuator_targets_from_qpos(self.m,self.d.qpos)
                    row=self.evaluate_waypoint(row["xyz"],seed,row["phase"],
                        axis_tolerance_rad=self.axis_reserve["internal_tolerance_rad"])
                    if self.planning_observer:self.planning_observer(row)
                    print(json.dumps(dict(mode="KINEMATIC ENDPOINT CANDIDATE / NOT LIVE",
                        planned_approach_error_deg=math.degrees(row["approach_error_rad"]),
                        internal_tolerance_deg=math.degrees(self.axis_reserve["internal_tolerance_rad"]),
                        acceptance_deg=2.,eligible=row["eligible"])),flush=True)
                    from dynamic_preflight import full_state_preflight
                    endpoint_preflight=full_state_preflight(self,[row])
                    self.report["endpoint_candidate"]=row
                    self.report["endpoint_preflight"]=endpoint_preflight
                    if not endpoint_preflight["passed"]:
                        raise ValueError("PREGRASP endpoint candidate preflight failed: "+str(endpoint_preflight["failure"]))
                self.waypoint_xyz=np.asarray(row["xyz"])
                target=row["ik"]["action_rad"]
                self.report["plans"].append(row)
                self.move(target, duration=row.get("duration_s", .8))
                self.previous_ik=target
                if endpoint_preflight is not None:
                    from dynamic_preflight import integration_state
                    actual=integration_state(self.m,self.d)
                    predicted=np.asarray(endpoint_preflight["final_state"])
                    self.report["endpoint_preflight_live_comparison"]=dict(
                        bitwise_equal=bool(np.array_equal(predicted,actual)),
                        maximum_state_difference=float(np.max(np.abs(predicted-actual))))
                preflight=self.report["staging_search"]["selected"]["dynamic_preflight"]
                if row["phase"]==preflight["scope"][-1]:
                    from dynamic_preflight import integration_state
                    predicted=np.asarray(preflight["final_state"])
                    actual=integration_state(self.m,self.d)
                    self.report["preflight_live_comparison"]=dict(scope=preflight["scope"],
                        comparison="final mjSTATE_INTEGRATION only", bitwise_equal=bool(np.array_equal(predicted,actual)),
                        maximum_state_difference=float(np.max(np.abs(predicted-actual))))
            if self.execute_plan:self.finish_task(grasp)
            else:self.transition("STAGING_READY")
        except (ValueError,RuntimeError) as error:
            self.report["failure"]=dict(phase=self.phase,reason=str(error))
            self.transition("FAILURE");self.report["trace"].append(self.snapshot())
        self.report["final_phase"]=self.phase
        self.report["hold_duration_s"]=self.env._block_hold_s
        self.report["maximum_lift_above_settled_bottom_m"]=max([0.]+[
            row["metrics"]["block_bottom_height_m"]-(self.env.settle_info["block_bottom_height_m"] if self.env.settle_info else 0.)
            for row in self.report["trace"]])
        return self.report

def run_staging_viewers(teacher, output):
    """Diagnostic qpos lives in private data; physics viewer observes teacher.d."""
    import time
    from mobile_dual_so101 import apply_control_as_pose
    preview=mujoco.MjData(teacher.m)
    def setup(diagnostic):
        diagnostic.cam.lookat[:]=[.2,0,.15]
        diagnostic.cam.distance,diagnostic.cam.azimuth,diagnostic.cam.elevation=.9,130,-30
        def observe(row):
            if not diagnostic.is_running():raise RuntimeError("waypoint diagnostic viewer closed")
            preview.qpos[:]=teacher.d.qpos
            apply_control_as_pose(teacher.m,preview,row["ik"]["action_rad"])
            teacher.draw_waypoint_markers(diagnostic)
            limit=min(row["joint_margins"][:5],key=lambda m:min(m["lower"],m["upper"]))
            gaps=row["geometric_clearances_m"]
            text=(f"KINEMATIC WAYPOINT DIAGNOSTIC / NOT PHYSICS\n"
                f"{row['phase']} | {'PASS' if row['eligible'] else 'FAIL'}\n"
                f"Position {row['position_error_m']*1000:.4f} mm | axis {math.degrees(row['approach_error_rad']):.3f} deg\n"
                f"General {row['task_policy']['general_clearance_m']*1000:.3f} / 30 mm\n"
                f"Pad/block {gaps['pad_block']*1000:.3f} | pad/table {gaps['pad_table']*1000:.3f} mm\n"
                f"Housing/block {gaps['housing_block']*1000:.3f} mm\n"
                f"Limit {limit['name']}: {min(limit['lower'],limit['upper']):.6f} rad\n"
                f"Full segment {'PASS' if row['guard']['safe'] else 'FAIL'}")
            diagnostic.set_texts([(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,text,"")])
            diagnostic.sync();time.sleep(.05)
        teacher.planning_observer=observe
        dynamic_draw={"time":None,"phase":None}
        def dynamic_observe(clone,row):
            if clone.execution_mode!="DYNAMIC PREFLIGHT / COPIED STATE / NOT LIVE TASK":return
            changed=dynamic_draw["phase"]!=clone.phase
            final="preflight_result" in row
            if not (changed or final or dynamic_draw["time"] is None or row["time_s"]-dynamic_draw["time"]>=.05):return
            if not diagnostic.is_running():raise RuntimeError("dynamic preflight viewer closed")
            from dynamic_preflight import integration_state
            mujoco.mj_setState(clone.m,preview,integration_state(clone.m,clone.d),mujoco.mjtState.mjSTATE_INTEGRATION)
            mujoco.mj_forward(clone.m,preview)
            teacher.draw_waypoint_markers(diagnostic)
            fmt=lambda q:" ".join(f"{x:.4f}" for x in q)
            left=(f"DYNAMIC PREFLIGHT / COPIED STATE / NOT LIVE TASK\n"
                  f"{clone.phase} | SIM {row['time_s']:.3f} s | {row.get('preflight_result','RUNNING')}\n"
                  f"Target L {fmt(row['target_q'][:6])}\nMeasured L {fmt(row['measured_actuator_q'][:6])}\n"
                  f"Target R {fmt(row['target_q'][6:])}\nMeasured R {fmt(row['measured_actuator_q'][6:])}")
            right=(f"Target/FK {row['target_fk_clearance_m']*1000:.6f} mm\n"
                   f"Measured {row['measured_clearance_m']*1000:.6f} mm\n"
                   f"Difference {row['clearance_difference_m']*1000:.6f} mm\n"
                   f"Tracking {row['maximum_tracking_error_rad']:.7f} rad\n"
                   f"Pair {' / '.join(row['closest_pair_names'])}\n"
                   +"\n".join(f"{name}: {value['qpos']:.8f}" for name,value in row["passive_jaw_state"].items()))
            if clone.phase=="CLOSE" and "planned_tcp_error_m" in row:
                right=(f"{row['arm_reference_source']}\n"
                    f"TCP planned/measured {row['planned_tcp_error_m']*1000:.6f} / {row['executed_tcp_error_m']*1000:.6f} mm\n"
                    f"TCP drift {(row['cumulative_tcp_drift_m'] or 0)*1000:.6f} mm\n"
                    f"Arm reference drift {row['arm_reference_drift_rad']} rad\n"
                    f"Tracking {row['maximum_arm_tracking_error_rad']:.8f} rad\n"
                    f"Clearance {row['measured_clearance_m']*1000:.6f} mm\n"
                    f"Finger forces {row['finger_force_N']}")
            grip=["GRIPPER / explicit task reference" if clone.gripper_hold_reference else "GRIPPER / legacy baseline"]
            for g in row.get("grippers",[]):
                grip.extend([f"{g['side']}: cmd margin {g['command_upper_margin_rad']:.2e} rad",
                    f"excess {g['upper_excess_rad']:.2e} / tol {g['numerical_tolerance_rad']:.1e}",
                    f"jaw opening {g['jaw_opening_m']*1000:.5f} mm"])
            diagnostic.set_texts([(mujoco.mjtFont.mjFONT_NORMAL,pos,text,"") for pos,text in
                ((mujoco.mjtGridPos.mjGRID_TOPLEFT,left),(mujoco.mjtGridPos.mjGRID_TOPRIGHT,right),
                 (mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,"\n".join(grip)))])
            diagnostic.sync()
            dynamic_draw.update(time=row["time_s"],phase=clone.phase)
        teacher.dynamic_observer=dynamic_observe
    # One display window avoids simultaneous passive-viewer GLX initialization.
    # Switching its snapshot never changes live or copied preflight physics.
    return run_with_viewer(teacher,output,display_data=preview,viewer_setup=setup)

def main():
    p=argparse.ArgumentParser();p.add_argument("--candidate-report",required=True)
    p.add_argument("--staging-report",required=True,help="reuse a validated retreat; do not repeat search")
    p.add_argument("--approach-reference",help="observed endpoint tracking report for an interior IK goal")
    p.add_argument("--output",required=True);p.add_argument("--viewer",action="store_true")
    p.add_argument("--plan-only",action="store_true")
    a=p.parse_args()
    if Path(a.output).exists():raise FileExistsError(a.output)
    saved=json.loads(Path(a.candidate_report).read_text())
    candidates=saved["candidates"]
    candidate=min((c for c in candidates if c["best"]["kinematic_pass"] and c["best"]["closing_pass"]),
                  key=lambda c:c["tilt"])
    t=WaypointBlockTeacher(candidate,json.loads(Path(a.staging_report).read_text()),
        json.loads(Path(a.approach_reference).read_text()) if a.approach_reference else None)
    for key in ("model_sha256","asset_sha256","scene_id","gripper_revision"):
        if saved["provenance"][key]!=t.report["provenance"][key]:
            raise ValueError("candidate scene provenance changed: "+key)
    t.execute_plan=not a.plan_only
    report=run_staging_viewers(t,Path(a.output)) if a.viewer else t.run()
    Path(a.output).write_text(json.dumps(report,indent=2))
if __name__=="__main__":main()
