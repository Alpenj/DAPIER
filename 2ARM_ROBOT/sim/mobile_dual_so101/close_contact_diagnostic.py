"""Copied-state CLOSE A/B; never authorizes live task execution."""
import argparse,json,math
from pathlib import Path
import numpy as np
import mujoco
from waypoint_block_teacher import WaypointBlockTeacher
from dynamic_preflight import integration_state,full_state_preflight,contact_telemetry
from mobile_dual_so101 import actuator_targets_from_qpos
from collision_guard import minimum_protected_clearance,protected_geom_pairs
MODE="DIAGNOSTIC COPY / NOT LIVE TASK SUCCESS"

def restore(m,d,state):
    kind=mujoco.mjtState.mjSTATE_INTEGRATION
    mujoco.mj_setState(m,d,np.asarray(state),kind);mujoco.mj_forward(m,d)
    mujoco.mj_setState(m,d,np.asarray(state),kind)


def run(report_path,output):
    saved=json.loads(Path(report_path).read_text());t=WaypointBlockTeacher(saved["candidate"]);t.env.reset(seed=0)
    if t.report["provenance"]["model_sha256"]!=saved["provenance"]["model_sha256"]:raise ValueError("compiled geometry changed")
    t.env.settle_info=saved["settle"];t.gripper_hold_reference=(saved["task_open"]["reference_rad"],)*2
    t.closing=np.array([1.,0,0]);t.waypoint_xyz=np.array(saved["teacher_input"]["grasp_xyz"])
    t.phase="CLOSE";t.env.collision_phase="CLOSE"
    start=saved["close_preflights"][9]["initial_state"]
    reference=np.array(saved["close_preflights"][0]["telemetry"][-1]["target_q"])
    onset=next(r for r in saved["execution_telemetry"] if r["phase"]=="CLOSE" and any(v>0 for v in r["finger_force_N"].values()))
    p=mujoco.MjData(t.m);p.qpos[:]=onset["raw_qpos"];mujoco.mj_forward(t.m,p)
    origin=dict(position=p.xpos[t.block_body].copy(),quaternion=p.xquat[t.block_body].copy())
    opened=saved["task_open"]["reference_rad"];closed=float(t.m.actuator_ctrlrange[5,0])
    schedule=np.linspace(opened,closed,max(2,math.ceil((opened-closed)/.01)+1))[1:]
    result=dict(mode=MODE,live_success=False,candidate=saved["candidate"],provenance=t.report["provenance"],
        start_state=start,contact_onset_time_s=onset["time_s"],baseline_q_proof=[],policies={})
    arm=[i for i in range(t.m.nu) if i not in (5,11)]
    for index in (9,10):
        prev=saved["close_preflights"][index-1]["telemetry"][-1];current=saved["close_preflights"][index]["telemetry"][-1]
        result["baseline_q_proof"].append(dict(stage=index+1,
            target_equals_previous_measured=bool(np.array_equal(np.array(current["target_q"])[arm],np.array(prev["measured_actuator_q"])[arm])),
            previous_command=prev["target_q"],previous_measured=prev["measured_actuator_q"],command=current["target_q"],measured=current["measured_actuator_q"]))
    for policy in ("baseline","fixed"):
        restore(t.m,t.d,start);t.close_start_tcp=t.d.site_xpos[t.site].copy()
        stages=[];result["policies"][policy]=stages
        for stage,opening in enumerate(schedule[9:],10):
            previous_command=t.d.ctrl.copy();previous_measured=np.array(actuator_targets_from_qpos(t.m,t.d.qpos))
            # Measured contact response is physical evidence, not automatically
            # the next arm command reference. Keep command/state roles explicit.
            t.close_arm_reference=reference.copy() if policy=="fixed" else None
            target=reference.copy() if policy=="fixed" else previous_measured.copy();target[5]=opening
            t.dynamic_observer=lambda clone,row:contact_telemetry(clone,row,origin,reference,previous_command,previous_measured,policy)
            before=integration_state(t.m,t.d)
            pref=full_state_preflight(t,[dict(eligible=True,phase="CLOSE",xyz=t.waypoint_xyz.tolist(),ik=dict(action_rad=target.tolist()),duration_s=.1)])
            np.testing.assert_array_equal(before,integration_state(t.m,t.d));pref["stage"]=stage
            if policy=="baseline" and stage<=11:
                pref["matches_saved_full_state"]=bool(np.array_equal(pref["final_state"],saved["close_preflights"][stage-1]["final_state"]))
            stages.append(pref);last=pref["telemetry"][-1]
            print(json.dumps(dict(mode=MODE,policy=policy,stage=stage,passed=pref["passed"],failure=pref["failure"],
                sim_time_s=last["time_s"],tcp_mm=last["executed_tcp_error_m"]*1000,planned_mm=last["planned_tcp_error_m"]*1000,
                forces=last["finger_force_N"],opposite_gap_mm={k:v*1000 for k,v in last["opposite_finger_gap_m"].items()},
                block_translation_mm=(np.array(last["block_translation_since_contact_m"])*1000).tolist())),flush=True)
            Path(output).write_text(json.dumps(result,indent=2))
            if not pref["passed"] or last["contact_state"]=="bilateral":break
            restore(t.m,t.d,pref["final_state"])
    return result

def replay(path):
    import time
    import mujoco.viewer
    r=json.loads(Path(path).read_text());t=WaypointBlockTeacher(r["candidate"]);m=t.m;d=mujoco.MjData(m)
    rows=[x for policy in ("baseline","fixed") for stage in r["policies"][policy] for x in stage["telemetry"]]
    with mujoco.viewer.launch_passive(m,d) as viewer:
        viewer.cam.lookat[:]=[.2,0,.08];viewer.cam.distance=.55
        for row in rows:
            if not viewer.is_running():return
            d.qpos[:]=row["raw_qpos"];d.qvel[:]=row["raw_qvel"];d.time=row["time_s"];mujoco.mj_forward(m,d)
            text=(f"{MODE} / RECORDED PHYSICS REPLAY\n{row['policy']} CLOSE t={d.time:.3f}\n"
                f"TCP planned/measured {row['planned_tcp_error_m']*1000:.4f}/{row['executed_tcp_error_m']*1000:.4f} mm\n"
                f"Reference drift {row['arm_reference_drift_rad']:.7f} rad\n"
                f"Contact {row['contact_state']} / force {row['finger_force_N']}\n"
                f"Opposite gap {row['opposite_finger_gap_m']} m\n"
                f"Block yaw {math.degrees(row['block_yaw_rad']):.3f} deg")
            viewer.set_texts([(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,text,"")])
            viewer.sync();time.sleep(.01)
        while viewer.is_running():viewer.sync();time.sleep(.05)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--report");p.add_argument("--output");p.add_argument("--replay");a=p.parse_args()
    if a.replay:replay(a.replay)
    else:
        if Path(a.output).exists():raise FileExistsError(a.output)
        run(a.report,a.output)
