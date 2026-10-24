"""SIM-only execution preflight; command FK is telemetry, never safety evidence."""
import copy
import hashlib
import numpy as np
import mujoco
from mobile_dual_so101 import actuator_targets_from_qpos, apply_control_as_pose
from collision_guard import minimum_protected_clearance, task_clearance_status, protected_geom_pairs

def integration_state(model, data):
    kind=mujoco.mjtState.mjSTATE_INTEGRATION
    state=np.empty(mujoco.mj_stateSize(model,kind))
    mujoco.mj_getState(model,data,state,kind)
    return state

def contact_telemetry(t,row,origin,reference,previous_command,previous_measured,policy):
    m,d=t.m,t.d;contacts=[]
    protected={tuple(sorted(p)) for p in protected_geom_pairs(m)}
    for i,c in enumerate(d.contact):
        wrench=np.zeros(6);mujoco.mj_contactForce(m,d,i,wrench)
        contacts.append(dict(geom_ids=[int(c.geom1),int(c.geom2)],
            names=[m.geom(int(g)).name for g in (c.geom1,c.geom2)],
            position_m=c.pos.tolist(),normal_world_geom1_to_geom2=c.frame[:3].tolist(),
            distance_m=float(c.dist),penetration_m=max(0.,-float(c.dist)),
            normal_force_N=float(wrench[0]),
            protected=tuple(sorted((int(c.geom1),int(c.geom2)))) in protected))
    forces=row["finger_force_N"]
    gaps={name:minimum_protected_clearance(m,d,[(g,t.block)])[0] for name,g in t.fingers.items()}
    q=np.array(row["target_q"]);arm=[i for i in range(m.nu) if i not in (5,11)]
    rot=d.xmat[t.block_body].reshape(3,3);quat=d.xquat[t.block_body].copy()
    velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,t.block_body,velocity,0)
    row.update(policy=policy,
        arm_reference_source="APPROACH_FINE explicit command" if policy=="fixed" else "previous measured arm q",
        previous_command_q=previous_command.tolist(),previous_measured_q=previous_measured.tolist(),
        arm_reference_drift_rad=float(np.max(np.abs((q-reference)[arm]))),
        measured_closing_error_rad=float(np.arccos(np.clip(d.site_xmat[t.site].reshape(3,3)[:,2]@t.closing,-1,1))),
        contacts=contacts,
        finger_contacts={name:any(g in c["geom_ids"] and t.block in c["geom_ids"] for c in contacts) for name,g in t.fingers.items()},
        contact_state="bilateral" if all(v>0 for v in forces.values()) else "unilateral" if any(v>0 for v in forces.values()) else "none",
        finger_block_gap_m=gaps,opposite_finger_gap_m={name:gaps[name] for name,v in forces.items() if v<=0},
        non_target_protected_contacts=[c for c in contacts if c["protected"] and t.block not in c["geom_ids"]],
        block_position_m=d.xpos[t.block_body].tolist(),block_quaternion=quat.tolist(),
        block_yaw_rad=float(np.arctan2(rot[1,0],rot[0,0])),
        block_linear_velocity_world_m_s=velocity[3:].tolist(),block_angular_velocity_world_rad_s=velocity[:3].tolist(),
        block_translation_since_contact_m=(d.xpos[t.block_body]-origin["position"]).tolist(),
        block_rotation_since_contact_rad=float(2*np.arccos(np.clip(abs(quat@origin["quaternion"]),0,1))))


def record_step(teacher):
    if teacher.step_telemetry and teacher.step_telemetry[-1]["time_s"]==float(teacher.d.time):
        return
    m,d=teacher.m,teacher.d
    status=task_clearance_status(m,d,teacher.env.collision_phase)
    pair=status["closest_general_pair"]
    if teacher.command_preview is None:teacher.command_preview=mujoco.MjData(m)
    preview=teacher.command_preview
    preview.qpos[:]=d.qpos
    apply_control_as_pose(m,preview,d.ctrl)
    command_gap=minimum_protected_clearance(m,preview,[tuple(pair)])[0]
    measured=np.asarray(actuator_targets_from_qpos(m,d.qpos))
    actuated=set(int(j) for j in m.actuator_trnid[:,0])
    passive={m.joint(j).name:dict(qpos=float(d.qpos[m.jnt_qposadr[j]]),
        qvel=float(d.qvel[m.jnt_dofadr[j]])) for j in range(m.njnt)
        if j not in actuated and m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_SLIDE,mujoco.mjtJoint.mjJNT_HINGE)}
    from pgripper import jaw_gap_m
    from shoe_task import INTEGRATION_GRIPPER_BOUNDARY_TOLERANCE_RAD
    grippers=[]
    for side,a in (("left",5),("right",11)):
        j=int(m.actuator_trnid[a,0]);value=float(d.qpos[m.jnt_qposadr[j]])
        low,high=m.jnt_range[j]
        equalities=[i for i in range(m.neq) if m.eq_type[i]==mujoco.mjtEq.mjEQ_JOINT and m.eq_obj2id[i]==j]
        grippers.append(dict(side=side,command=float(d.ctrl[a]),raw_qpos=value,
            qvel=float(d.qvel[m.jnt_dofadr[j]]),joint_range=[float(low),float(high)],
            ctrlrange=m.actuator_ctrlrange[a].tolist(),tracking_error=value-float(d.ctrl[a]),
            upper_excess_rad=max(0.,value-high),numerical_tolerance_rad=INTEGRATION_GRIPPER_BOUNDARY_TOLERANCE_RAD,
            command_upper_margin_rad=float(m.actuator_ctrlrange[a,1]-d.ctrl[a]),jaw_opening_m=jaw_gap_m(m,d,side),
            command_source=("explicit task open/close reference; prior command trajectory start"
                if getattr(teacher,"gripper_hold_reference",None) is not None else "legacy measured trajectory start"),
            constraints=[dict(type=int(d.efc_type[k]),id=int(d.efc_id[k]),position=float(d.efc_pos[k]),force=float(d.efc_force[k]))
                for k in range(d.nefc) if (d.efc_type[k]==mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT and d.efc_id[k]==j)
                or (d.efc_type[k]==mujoco.mjtConstraint.mjCNSTR_EQUALITY and d.efc_id[k] in equalities)]))
    measured_axis=d.site_xmat[teacher.site].reshape(3,3)[:,0]
    command_axis=preview.site_xmat[teacher.site].reshape(3,3)[:,0]
    angle=lambda u:float(np.arccos(np.clip(u@teacher.approach,-1.,1.)))
    row=dict(grippers=grippers,warnings=d.warning.number.tolist(),phase=teacher.phase,
        tracking_error_rad=(measured-d.ctrl).tolist(),actuator_force=d.actuator_force.tolist(),
        actuator_force_saturated=[bool(m.actuator_forcelimited[a] and
            (d.actuator_force[a]<=m.actuator_forcerange[a,0] or d.actuator_force[a]>=m.actuator_forcerange[a,1]))
            for a in range(m.nu)],
        desired_approach_axis=teacher.approach.tolist(),measured_approach_axis=measured_axis.tolist(),
        command_fk_approach_axis=command_axis.tolist(),measured_approach_error_rad=angle(measured_axis),
        command_fk_approach_error_rad=angle(command_axis),time_s=float(d.time),target_q=d.ctrl.tolist(),
        measured_actuator_q=measured.tolist(),raw_qpos=d.qpos.tolist(),raw_qvel=d.qvel.tolist(),
        target_fk_clearance_m=command_gap,measured_clearance_m=status["general_clearance_m"],
        clearance_difference_m=command_gap-status["general_clearance_m"],
        maximum_tracking_error_rad=float(np.max(np.abs(d.ctrl-measured))),
        passive_jaw_state=passive,closest_pair=pair,
        closest_pair_names=[m.geom(g).name for g in pair],policy_safe=status["safe"],
        task_pairs=status["pairs"],command_fk_is_safety_evidence=False)
    if getattr(teacher,"waypoint_xyz",None) is not None:
        arm=[i for i in range(m.nu) if i not in (5,11)]
        reference=getattr(teacher,"close_arm_reference",None)
        snap=teacher.snapshot()
        row.update(desired_tcp_position=teacher.waypoint_xyz.tolist(),
            commanded_tcp_position=preview.site_xpos[teacher.site].tolist(),
            measured_tcp_position=d.site_xpos[teacher.site].tolist(),
            planned_tcp_error_m=float(np.linalg.norm(preview.site_xpos[teacher.site]-teacher.waypoint_xyz)),
            executed_tcp_error_m=float(np.linalg.norm(d.site_xpos[teacher.site]-teacher.waypoint_xyz)),
            arm_reference_source=("APPROACH_FINE terminal command" if teacher.phase=="CLOSE" and reference is not None else "existing phase command"),
            maximum_arm_tracking_error_rad=float(np.max(np.abs((measured-d.ctrl)[arm]))),
            arm_reference_drift_rad=(float(np.max(np.abs((d.ctrl-reference)[arm]))) if reference is not None else None),
            cumulative_tcp_drift_m=(float(np.linalg.norm(d.site_xpos[teacher.site]-teacher.close_start_tcp)) if hasattr(teacher,"close_start_tcp") else None),
            finger_force_N=snap["finger_force_N"])
    if teacher.phase=="CLOSE" and hasattr(teacher,"close_previous_command"):
        if teacher.close_contact_origin is None and any(v>0 for v in row["finger_force_N"].values()):
            teacher.close_contact_origin=dict(position=d.xpos[teacher.block_body].copy(),quaternion=d.xquat[teacher.block_body].copy())
        origin=teacher.close_contact_origin or dict(position=d.xpos[teacher.block_body].copy(),quaternion=d.xquat[teacher.block_body].copy())
        contact_telemetry(teacher,row,origin,teacher.close_arm_reference,
            teacher.close_previous_command,teacher.close_previous_measured,"fixed")
    teacher.step_telemetry.append(row)
    previous=teacher.last_telemetry_print
    if previous is None or previous[0]!=teacher.phase or d.time-previous[1]>=1.:
        import json
        print(json.dumps(dict(mode=teacher.execution_mode,phase=teacher.phase,time_s=float(d.time),
            measured_clearance_m=row["measured_clearance_m"],maximum_tracking_error_rad=row["maximum_tracking_error_rad"],grippers=[{k:g[k] for k in ("side","command","raw_qpos","upper_excess_rad","jaw_opening_m")} for g in grippers])),flush=True)
        teacher.last_telemetry_print=(teacher.phase,float(d.time))
    if teacher.dynamic_observer:teacher.dynamic_observer(teacher,row)

def full_state_preflight(teacher, segments):
    # Kinematic clearance is insufficient when tracking and passive linkage motion consume the margin.
    # Execute a copied full-state physics preflight before authorizing tight-clearance segments.
    before=integration_state(teacher.m,teacher.d)
    clone=copy.copy(teacher);clone.env=copy.copy(teacher.env)
    clone.d=copy.copy(teacher.d);clone.env.data=clone.d
    clone.env.settle_info=copy.deepcopy(teacher.env.settle_info)
    clone.env.measured_state_trace=[]
    clone.env.physics_observer=None
    clone.observer=None
    clone.report=dict(success=False,transitions=[],plans=[],trace=[],waypoint_gates=[])
    clone.step_telemetry=[];clone.command_preview=None;clone.last_telemetry_print=None
    clone.execution_mode="DYNAMIC PREFLIGHT / COPIED STATE / NOT LIVE TASK"
    clone.previous_ik=None
    start=integration_state(clone.m,clone.d)
    if not np.array_equal(start,before):raise AssertionError("preflight initial state differs")
    failure=None
    try:
        for row in segments:
            if not row["eligible"]:raise ValueError("kinematic prerequisite failed")
            clone.transition(row["phase"]);clone.waypoint_xyz=np.asarray(row["xyz"])
            clone.move(row["ik"]["action_rad"],duration=row.get("duration_s",.8))
            clone.previous_ik=row["ik"]["action_rad"]
    except (ValueError,RuntimeError) as error:
        failure=dict(phase=clone.phase,reason=str(error))
    finally:
        if not np.array_equal(before,integration_state(teacher.m,teacher.d)):
            raise AssertionError("preflight contaminated live state")
    result=dict(passed=failure is None,failure=failure,
        scope=[row["phase"] for row in segments],
        initial_state=start.tolist(),initial_state_sha256=hashlib.sha256(start.tobytes()).hexdigest(),
        final_state=integration_state(clone.m,clone.d).tolist(),
        live_state_unchanged=True,telemetry=clone.step_telemetry,
        waypoint_gates=clone.report["waypoint_gates"],
        minimum_measured_clearance_m=min((r["measured_clearance_m"] for r in clone.step_telemetry),default=None))
    if clone.step_telemetry and teacher.dynamic_observer:
        teacher.dynamic_observer(clone,dict(clone.step_telemetry[-1],preflight_result="PASS" if result["passed"] else "FAIL"))
    return result

def clearance_sensitivity(model, data, pair=(36,74)):
    """Raw-coordinate finite differences, not a hardware margin or feasible perturbation controller."""
    base=minimum_protected_clearance(model,data,[pair])[0]
    rows=[]
    for j in range(model.njnt):
        name=model.joint(j).name
        if not name.startswith("left_") or model.jnt_type[j] not in (mujoco.mjtJoint.mjJNT_HINGE,mujoco.mjtJoint.mjJNT_SLIDE):continue
        address=int(model.jnt_qposadr[j]);value=float(data.qpos[address])
        derivatives=[]
        for eps in (1e-5,5e-6):
            lo,hi=model.jnt_range[j]
            offsets=[x for x in (-eps,eps) if not model.jnt_limited[j] or lo<=value+x<=hi]
            if not offsets:continue
            values=[]
            for offset in offsets:
                p=copy.copy(data);p.qpos[address]+=offset;mujoco.mj_forward(model,p)
                values.append(minimum_protected_clearance(model,p,[pair])[0])
            derivative=((values[-1]-values[0])/(offsets[-1]-offsets[0]) if len(offsets)==2
                        else (values[0]-base)/offsets[0])
            derivatives.append(dict(step=eps,derivative=derivative,offsets=offsets))
        rows.append(dict(joint=name,qpos=value,coordinate="m" if model.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE else "rad",derivatives=derivatives))
    return dict(pair=list(pair),base_clearance_m=base,rows=rows,
        purpose="engineering evidence only; raw-coordinate partials hold other passive coordinates fixed")


def measured_axis_reserve(model, site, report):
    """Observed last-100-ms command/actual axis deviation; not a universal bound."""
    samples=[r for r in report["execution_telemetry"] if r["phase"]=="PREGRASP_NEAR"]
    if not samples:raise ValueError("no PREGRASP endpoint evidence")
    measured=mujoco.MjData(model);command=mujoco.MjData(model)
    deviations=[]
    for row in samples:
        if row["time_s"]<samples[-1]["time_s"]-.1-1e-10:continue
        measured.qpos[:]=row["raw_qpos"];mujoco.mj_forward(model,measured)
        apply_control_as_pose(model,command,row["target_q"])
        u=measured.site_xmat[site].reshape(3,3)[:,0]
        v=command.site_xmat[site].reshape(3,3)[:,0]
        deviations.append(float(np.arccos(np.clip(u@v,-1.,1.))))
    reserve=max(deviations)
    if not np.isfinite(reserve) or not 0<reserve<np.deg2rad(2):
        raise ValueError("observed axis deviation cannot define an interior planning target")
    return dict(observed_axis_deviation_rad=reserve,internal_tolerance_rad=float(np.deg2rad(2)-reserve),
                acceptance_rad=float(np.deg2rad(2)),samples=len(deviations),
                source="last 100 ms of saved command/full-state endpoint; candidate preflight required")
