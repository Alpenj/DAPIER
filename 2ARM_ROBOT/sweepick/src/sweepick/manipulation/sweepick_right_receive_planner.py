"""Static RECEIVE preview using the existing teacher; no simulator stepping."""
import hashlib
from pathlib import Path

import numpy as np

from sweepick.integration.sweepick_receive_runtime import configure_runtime, field_module


def runtime():
    configure_runtime()
    import mujoco
    import integration_scenes
    from sim_data_factory.teacher import HandoverTeacher, TeacherFailure
    return mujoco, integration_scenes, HandoverTeacher, TeacherFailure, field_module("kin").set_q12


def build_planning_scene(state):
    mj, scenes, teacher_type, failure_type, set_q12 = runtime()
    scene = state["scene"]
    actual_source = hashlib.sha256(Path(scenes.__file__).read_bytes()).hexdigest()
    if scene["source_sha256"] != actual_source:
        raise ValueError("scene source SHA differs from the RECEIVE scene builder")
    spec = scenes.build_scene("desk", grippers="both")
    table = spec.geom("table")
    task_from_model = np.asarray(scene["T_task_from_model"])
    task_table = task_from_model @ np.asarray(scene["T_model_table"])
    if not np.isclose(task_table[2, 3], scene["table_top_z_m"]):
        raise ValueError("table top height differs from its task-frame transform")
    table.pos[:] = task_table[:3, 3] - task_table[:3, 2] * table.size[2]
    quat = np.empty(4)
    mj.mju_mat2Quat(quat, np.ascontiguousarray(task_table[:3, :3]).ravel())
    table.quat[:] = quat
    spec.geom("red_block_geom").size[:] = state["object_half_extents_m"]
    model = spec.compile()
    data = mj.MjData(model)
    measured = np.r_[state["left_measured_q"], state["right_measured_q"]]
    set_q12(model, data, measured)
    mj.mj_forward(model, data)
    for side in ("left", "right"):
        expected = task_from_model @ np.asarray(scene[f"T_model_{side}_base"])
        actual = np.eye(4)
        body = data.body(side + "_base")
        actual[:3, 3], actual[:3, :3] = body.xpos, body.xmat.reshape(3, 3)
        if not np.allclose(actual, expected):
            raise ValueError(f"{side} base transform differs; this desk scene cannot represent that assembly")
    pose = task_from_model @ np.asarray(state["object_pose_model"])
    teacher = teacher_type(model)
    address = teacher.plan.cm.block_qpos
    quat = np.empty(4)
    mj.mju_mat2Quat(quat, np.ascontiguousarray(pose[:3, :3]).ravel())
    data.qpos[address:address + 7] = np.r_[pose[:3, 3], quat]
    mj.mj_forward(model, data)
    return mj, model, data, teacher, failure_type


def gear(teacher, canonical):
    result = np.array(canonical, dtype=float, copy=True)
    result[[5, 11]] = teacher.g_a + teacher.g_b * result[[5, 11]]
    return result


def plan_receive(state):
    out = dict(status="NOT_READY", missing=list(state.get("missing", [])),
               real_execution_ready=False, motor_commands_sent=0, mj_step_calls=0,
               classification="PROVISIONAL model preview; no new physical HARD criteria",
               waypoints=[], left_gripper_hold_preserved=False, left_arm_frozen=False,
               geometric_start_q12=None, right_last_applied_q=None)
    if state.get("status") != "INPUT_VALID":
        out["invalid"] = list(state.get("invalid", []))
        return out
    try:
        mj, model, data, teacher, failure_type = build_planning_scene(state)
    except (ValueError, KeyError, RuntimeError, FileNotFoundError) as exc:
        out.update(status="NOT_READY", failure=dict(stage="scene", cause=str(exc)))
        return out
    measured = np.r_[state["left_measured_q"], state["right_measured_q"]]
    seed = np.r_[state["left_hold_q"], state["right_measured_q"]]
    hold = float(state["left_hold_q"][5])
    out["geometric_start_q12"] = measured.tolist()
    out["planning_seed_provenance"] = dict(left="last_applied hold command", right="measured geometry, not applied command")
    carry = ("left", teacher.site_block_rel(data, "left"))
    current = gear(teacher, measured)
    base = data.qpos.copy()
    private_segments = []

    def waypoint(label, goal, trajectory=None):
        nonlocal current, base
        goal = np.asarray(goal, dtype=float)
        if not np.isclose(teacher.to_canonical(goal)[5], hold):
            raise ValueError("RECEIVE planner changed the left Jaw hold command")
        hit = teacher.plan.path_collisions(base, current, goal, carry=carry, allow_pad_block=("left",))
        if hit is not None:
            raise ValueError(f"{label}: collision preview {hit}")
        block = teacher.plan.carried_block(base, goal, carry)
        data.qpos[:] = teacher.plan.pose(base, goal, block).qpos
        mj.mj_forward(model, data)
        base, current = data.qpos.copy(), goal.copy()
        teacher.cmd = goal.copy()
        out["waypoints"].append(dict(label=label, q12=teacher.to_canonical(goal).tolist()))
        private_segments.append(trajectory)

    try:
        waypoint("LEFT_HOLD_SEED", gear(teacher, seed))
        opened = current.copy()
        opened[11] = teacher.gear_open
        waypoint("RIGHT_OPEN", opened)
        teacher._plan_transfer(data)
        first = teacher.segment
        queued = list(teacher.queue)
        waypoint("CARRY_" + teacher.info["transfer_order"][0].upper(), first["goal"], first["traj"])
        for side, goal in queued:
            teacher.start_segment(goal, [side])
            waypoint("CARRY_" + side.upper(), teacher.segment["goal"], teacher.segment["traj"])
        teacher.queue = []
        teacher._plan_right_grasp(data)
        first = teacher.segment
        queued = list(teacher.queue)
        waypoint("RIGHT_APPROACH", first["goal"], first["traj"])
        for side, goal in queued:
            teacher.start_segment(goal, [side])
            waypoint("RIGHT_INSERT", teacher.segment["goal"], teacher.segment["traj"])
    except (failure_type, ValueError) as exc:
        out.update(status="PLAN_REJECTED", failure=dict(stage="receive", cause=str(exc)))
        out["waypoints"] = []
        return out
    centre, rotation, _ = teacher.block_pose(data)
    predicted_pose = np.eye(4)
    predicted_pose[:3, 3], predicted_pose[:3, :3] = centre, rotation
    out.update(status="PLAN_PREVIEW", left_gripper_hold_preserved=True,
               planner_info=teacher.info,
               reused=["HandoverTeacher._plan_transfer", "HandoverTeacher._plan_right_grasp", "Planner.path_collisions"],
               scene=dict(state["scene"], compiled_table_top_z_m=state["scene"]["table_top_z_m"]),
               left_hold_q=list(state["left_hold_q"]),
               right_insert_q12=teacher.to_canonical(current).tolist(),
               predicted_right_object_rel=teacher.site_block_rel(data, "right").tolist(),
               predicted_carried_object_pose_task=predicted_pose.tolist(),
               object_pose_model=state["object_pose_model"],
               object_pose_uncertainty=state["object_pose_uncertainty"],
               pose_provenance="predicted carried geometry; actual RECEIVE feedback required downstream",
               _teacher=teacher, _model=model, _data=data, _carry=carry, _trajectories=private_segments)
    return out
