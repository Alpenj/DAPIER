#!/usr/bin/env python3
"""Train-rollout diagnostic for the current checkpoint (SIM only; no training, no new data).

Question: does the current model fail the first grasp even from initial conditions it was trained on,
and where does it first depart?

  protocol  fixes the parents, the execution contract and the reporting definitions before any rollout
  analyze   (1) Train-parent rollouts re-aggregated with r5_reconcile, kept apart from Val tables;
            (2) seed 5000: deployment-mode predictions on the TEACHER's observations vs the teacher's labels
                (approach .. lift phases), and (3) how the student's own closed loop departs
  video     teacher demonstration (reference) and student rollout of seed 5000, same initial condition

A teacher action at time t is the label of the TEACHER's observation at t. Once the student's state differs,
that action is not treated as the student's ground truth: (2) and (3) are reported separately.
"""
import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import r5_reconcile as rr

FACTORY_ROOT = Path("/home/robot/DAPIER/.local-artifacts/sim-data-factory-20261002")
TRAINING = FACTORY_ROOT / "training/arm-output-20261002-01-bounded-candidate"
CHECKPOINT = TRAINING / "step-005000"
DATASET = FACTORY_ROOT / "datasets/handover-g000-g001"
MODEL_KEY = "arm_bounded"
TRAIN_SEEDS = (5000, 5002, 5008)
DETAIL_SEED = 5000
WINDOW_PHASES = ("left_approach", "left_descend", "left_grasp", "left_lift")   # the teacher's own phase labels
KS = (0, 24, 49)                      # chunk indices; at 50 Hz the targets are 0, 0.48 and 0.98 s ahead
DT, HORIZON, CHUNK = 0.02, 2000, 50
LEFT_ARM, RIGHT_ARM = (0, 1, 2, 3, 4), (6, 7, 8, 9, 10)
LEFT_GRIPPER, RIGHT_GRIPPER = 5, 11
MOVING_EPS_RAD = 1e-3                 # numerical guard for direction/magnitude ratios; not a criterion
ARM_MARKS_RAD = (0.02, 0.05, 0.10)    # descriptive marks (0.02 = checker joint_limit_tol_rad reused as a bin)
TCP_MARKS_M = (0.01, 0.02, 0.05)      # descriptive marks for the student-teacher TCP separation
GROUPS = dict(left_arm=LEFT_ARM, right_arm=RIGHT_ARM, left_gripper=(LEFT_GRIPPER,), right_gripper=(RIGHT_GRIPPER,))
# Supplementary measurements added after the rollouts were seen (inference only, same teacher observations).
OTHER_CHECKPOINTS = (
    ("bounded-01 step-004000 (팔 linear, Train 69에서 4,000 update; 대조 실험의 시작점)", "training/handover-g000-g001-bounded-01/step-004000"),
    ("R5 Model A step-005000 (bounded-01 + 5,000 update, 팔 linear; 학습 집합 ID 단위 미확인)", "training/handover-r5-orig-continued/step-005000"),
    ("ARM_LINEAR_CONTROL step-005000 (팔 head 재초기화 후 5,000 update, linear)", "training/arm-output-20261002-01-linear-control/step-005000"),
)
GAIN_PERTURBATION = 0.05     # rad (arm) / opening fraction (gripper) added to one measured-state input
OFFSET_SEARCH_STEPS = 250    # how far ahead the teacher command closest to the k=0 prediction is searched


def load(path):
    return json.loads(Path(path).read_text())


def file_sha(path):
    with Path(path).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def run_dir(run_id):
    return FACTORY_ROOT / "evaluation" / run_id


def train_dir(run_id):
    return run_dir(run_id) / "train" / MODEL_KEY


def manifest_row(seed):
    episodes = load(DATASET / "manifest.json")["episodes"]
    matches = [(i, e) for i, e in enumerate(episodes) if e["seed"] == seed]
    if len(matches) != 1 or matches[0][1]["split"] != "train":
        raise ValueError(f"seed {seed} is not a unique Train-split episode")
    return matches[0]


def teacher_episode(seed):
    """Stored Train demonstration: commands, measured states, teacher phase labels, checker rows, events."""
    index, row = manifest_row(seed)
    path = Path(row["path"])
    with np.load(path / "episode.npz", allow_pickle=False) as a:
        data = dict(q_cmd=a["q_cmd"].astype(np.float64), q_state=a["q_state"].astype(np.float64),
                    integration_state=a["integration_state"].copy(), sim_time=a["sim_time"].copy())
    metadata = load(path / "metadata.json")
    phases = [d.get("phase") for d in load(path / "diagnostics.json")]
    rows = load(path / "task_check_rows.json")
    if len(phases) != len(data["q_cmd"]) or len(rows) != len(data["q_state"]):
        raise ValueError(f"{path}: phase/row counts do not match the episode")
    spans = {}
    for k, phase in enumerate(phases):
        spans.setdefault(phase, [k, k])[1] = k
    return dict(index=index, row=row, path=path, metadata=metadata, phases=phases, phase_spans=spans, rows=rows,
                events=metadata["task_check"]["events"], **data)


def gripper_timeline(teacher):
    """The teacher's actual left-gripper command and contact timeline (not a single plotting line)."""
    left = teacher["q_cmd"][:, LEFT_GRIPPER]
    plateau = float(left.max())
    top = int(left.argmax())
    onset = next(i for i in range(top, len(left)) if left[i] < plateau - 1e-6)
    lift_step = int(round(teacher["events"]["pick_lift"] / DT))
    hold = float(left[lift_step])
    middle = (plateau + hold) / 2
    forces = np.array([r["pad_force_N"]["left"] for r in teacher["rows"]])
    check = teacher["metadata"]["task_check"]
    grip_force = (check.get("config") or teacher["metadata"]["checker_config"])["grip_force_min_N"]
    held = np.nonzero(forces.min(1) >= grip_force)[0]
    return dict(first_command=float(left[0]), open_plateau=plateau, open_reached_step=top, close_onset_step=onset,
                hold_level=hold, hold_reached_step=next(i for i in range(onset, len(left)) if abs(left[i] - hold) < 2e-3),
                middle_level=middle, middle_crossed_step=next(i for i in range(onset, len(left)) if left[i] <= middle),
                first_pad_contact_step=int(np.nonzero(forces.max(1) > 0)[0][0]),
                both_pads_grip_force_first_step=int(held[0]), both_pads_grip_force_last_step=int(held[-1]),
                grip_force_min_N=grip_force, note="levels and steps are read from this demonstration's own commands and checker rows")


def protocol(args):
    out = run_dir(args.run_id)
    out.mkdir(parents=True, exist_ok=False)
    parents = []
    for seed in TRAIN_SEEDS:
        index, row = manifest_row(seed)
        meta = load(Path(row["path"]) / "metadata.json")
        parents.append(dict(seed=seed, manifest_index=index, split=row["split"], generation=row["generation"],
                            parent_physics_group=row["parent_physics_group"], source_episode=row["path"], transitions=row["transitions"],
                            reset=dict(q0=meta["requested_initial_state"]["q"], block_xy_yaw=meta["sampling"]["block_xy_yaw"]),
                            teacher_events=meta["task_check"]["events"]))
    teacher = teacher_episode(DETAIL_SEED)
    window_end = teacher["phase_spans"][WINDOW_PHASES[-1]][1]
    inputs = load(TRAINING / "training_inputs.json")
    record = dict(
        run_id=args.run_id, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        scope="Train-parent diagnostic in local SIM. Not a Val/Test result and not a generalization estimate. No training, no new data, no hardware.",
        checkpoint=dict(path=str(CHECKPOINT), sha256={p.name: file_sha(p) for p in sorted(CHECKPOINT.iterdir()) if p.is_file()},
                        normalization_sha256=file_sha(TRAINING / "normalization.npz"),
                        trained_on_these_parents=all(p["manifest_index"] in [r["manifest_index"] for r in inputs["train"]["rows"]] for p in parents)),
        parents=parents,
        why_these_seeds="the Train cases already used in R4/R5; fixed here before running and not replaced by cases that happen to succeed",
        execution=dict(runs_per_parent=1, max_new_rollouts=3, execution_prefix=1, execution_aggregation="off", group="train",
                       reset="each parent's own q0 and block pose passed explicitly (no reset=None fallback)",
                       horizon="40 s / 2000 transitions or the first abort under the normal contract",
                       not_done="no teacher action, no state restore, no pregrasp start, no right-arm substitution, no object attachment",
                       policy_inputs="wrist RGB + measured canonical state, as in every earlier evaluation"),
        detail_seed=dict(
            seed=DETAIL_SEED, source_episode=str(teacher["path"]),
            window=dict(teacher_phases=list(WINDOW_PHASES), steps=[0, window_end], sim_time_s=[0.0, round(window_end * DT, 2)],
                        phase_spans={p: teacher["phase_spans"][p] for p in WINDOW_PHASES}),
            teacher_events=teacher["events"], teacher_left_gripper=gripper_timeline(teacher),
            teacher_observation_comparison=dict(
                loader="arm_output_trial.TrialEpisodes (the loader this checkpoint was trained with): normalized state + two wrist RGB; labels q_cmd",
                prediction="policy.predict_canonical_chunk (eval mode, VAE latent zeros) = the function the evaluator calls",
                compared="q_pred[t,k] against q_cmd_teacher[t+k]; chunk indices " + str(list(KS)) + " (0, 0.48, 0.98 s ahead); padding excluded",
                groups="left arm, right arm (rad) and left/right gripper (opening fraction) separately",
                baselines="persistence = keep the currently measured position; its error equals |q_cmd_teacher[t+k] - q_state[t]| (the motion the teacher asks for)",
                direction="cosine between predicted and teacher displacement from the measured position; magnitude ratio; only where the teacher "
                          f"displacement norm exceeds {MOVING_EPS_RAD} rad (numerical guard)",
                gripper="the teacher's real command timeline and pad-contact interval of this demonstration"),
            closed_loop=dict(
                rule="after the student's state differs, the teacher action at the same time is NOT the student's label",
                reported="identity of the first observation; student-teacher state separation over time (descriptive marks "
                         f"{list(ARM_MARKS_RAD)} rad, {list(TCP_MARKS_M)} m); first fault or stall with pose and contact pair; "
                         "geometry at equal horizontal distance to the block")),
        questions=["grasp/lift 0 even from the Train initial conditions?",
                   "on the teacher's own observations, are direction / closing / amount of motion predicted?",
                   "followed on teacher observations but lost in the student's own closed loop?",
                   "any Train parent that reaches grasp/lift from its original initial condition?"],
        reused_evidence=dict(
            r5_positive_control=dict(file=str(FACTORY_ROOT / "evaluation/r5_positive_control_report.json"),
                                     shows="state restore, observation and first-action consistency at seed 5000 step 431; teacher command replay "
                                           "reproduces reach/left_grasp/pick_lift. Not repeated here."),
            earlier_models=dict(files=[str(FACTORY_ROOT / "evaluation/10A-B-C25-1001/evaluation.json"),
                                       str(FACTORY_ROOT / "evaluation/09-overfit1001-final/evaluation.json")],
                                shows="other checkpoints / other data: stored results, read only. They are not results of the current model.")),
        stop_rule="diagnosis only: no retraining, loss or model change is started; an implementation defect is repaired only if it is reproduced",
        next_change="exactly one proposal, tied to what this run measures")
    (out / "protocol.json").write_text(json.dumps(record, indent=2))
    print(json.dumps(dict(run_id=record["run_id"], fixed=record["fixed_before_execution"], parents=[(p["seed"], p["manifest_index"], p["split"]) for p in parents],
                          window=record["detail_seed"]["window"], teacher_left_gripper=record["detail_seed"]["teacher_left_gripper"]), indent=2))
    print(f"[written] {out / 'protocol.json'}")


# ----------------------------------------------------------------------------------------------
# Kinematics from stored states (read-only replay of positions, no stepping)
# ----------------------------------------------------------------------------------------------

def kinematics(states):
    """Left/right grasp-site and block positions, lowest pad height, for each stored integration state."""
    import arm_output_trial  # noqa: F401  SIM environment before the scene is built
    import mujoco
    from integration_scenes import task_env
    model = task_env("desk", grippers="both").model
    data = mujoco.MjData(model)
    spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    site = {s: model.site(f"{s}_cube_grasp").id for s in ("left", "right")}
    pads = {s: [model.geom(f"{s}_pgripper_pad_{f}").id for f in (1, 2)] for s in ("left", "right")}
    block = model.body("red_block").id
    table = model.geom("table").id
    table_top = float(model.geom_pos[table][2] + model.geom_size[table][2])
    out = dict(left_tcp=[], right_tcp=[], block=[], left_pad_low=[], right_pad_low=[])
    for state in states:
        mujoco.mj_setState(model, data, state, spec)
        mujoco.mj_forward(model, data)
        out["left_tcp"].append(data.site_xpos[site["left"]].copy())
        out["right_tcp"].append(data.site_xpos[site["right"]].copy())
        out["block"].append(data.xpos[block].copy())
        for s in ("left", "right"):
            out[f"{s}_pad_low"].append(min(float(data.geom_xpos[g][2]) for g in pads[s]) - table_top)
    result = {k: np.array(v) for k, v in out.items()}
    result["table_top_z"] = table_top
    return result


# ----------------------------------------------------------------------------------------------
# Teacher-observation comparison
# ----------------------------------------------------------------------------------------------

def horizon_metrics(prediction, q_cmd, q_state, steps, k, channels):
    """q_pred[t,k] vs q_cmd[t+k] for observations `steps`; persistence = q_state[t]. Padding is excluded."""
    steps = np.asarray(steps)
    channels = list(channels)
    valid = steps + k < len(q_cmd)
    t = steps[valid]
    target, now = q_cmd[t + k][:, channels], q_state[t][:, channels]
    predicted = prediction[t, k][:, channels]   # the prediction array is indexed by the absolute observation step
    error, needed = np.abs(predicted - target), np.abs(target - now)
    out = dict(observations=int(valid.sum()), padded_excluded=int((~valid).sum()),
               model_mae=float(error.mean()), model_max=float(error.max()),
               persistence_mae=float(needed.mean()), persistence_max=float(needed.max()),
               model_over_persistence=float(error.mean() / needed.mean()) if needed.mean() > 0 else rr.UNAVAILABLE)
    if len(channels) > 1:
        d_pred, d_teacher = predicted - now, target - now
        n_pred, n_teacher = np.linalg.norm(d_pred, axis=1), np.linalg.norm(d_teacher, axis=1)
        moving = (n_teacher > MOVING_EPS_RAD) & (n_pred > 0)
        if moving.any():
            cosine = (d_pred * d_teacher).sum(1)[moving] / (n_pred[moving] * n_teacher[moving])
            out.update(moving_observations=int(moving.sum()), direction_cosine_mean=float(cosine.mean()),
                       direction_cosine_p10=float(np.percentile(cosine, 10)), share_direction_cosine_below_0=float((cosine < 0).mean()),
                       displacement_magnitude_ratio_median=float(np.median(n_pred[moving] / n_teacher[moving])))
        else:
            out.update(moving_observations=0)
    return out


def first_exceeding(series, marks):
    return {str(m): (int(np.nonzero(series > m)[0][0]) if (series > m).any() else None) for m in marks}


def teacher_observation_analysis(teacher, prediction, window_end):
    q_cmd, q_state = teacher["q_cmd"], teacher["q_state"]
    result = dict(observations=int(window_end + 1), phases={}, whole_window={})
    spans = dict(whole_window=[0, window_end], **{p: teacher["phase_spans"][p] for p in WINDOW_PHASES})
    for name, (a, b) in spans.items():
        steps = np.arange(a, b + 1)
        block = {f"k{k}": {group: horizon_metrics(prediction, q_cmd, q_state, steps, k, channels) for group, channels in GROUPS.items()} for k in KS}
        block["steps"], block["sim_time_s"] = [int(a), int(b)], [round(a * DT, 2), round(b * DT, 2)]
        if name == "whole_window":
            result["whole_window"] = block
        else:
            result["phases"][name] = block
    steps = np.arange(window_end + 1)
    left_error = np.abs(prediction[steps, 0][:, list(LEFT_ARM)] - q_cmd[steps][:, list(LEFT_ARM)]).max(1)
    right_error = np.abs(prediction[steps, 0][:, list(RIGHT_ARM)] - q_cmd[steps][:, list(RIGHT_ARM)]).max(1)
    needed_left = np.abs(q_cmd[steps][:, list(LEFT_ARM)] - q_state[steps][:, list(LEFT_ARM)]).max(1)
    worst = int(left_error.argmax())
    result["first_command_k0"] = dict(
        note="per observation: largest joint error of the executed-index (k=0) command against the teacher's command",
        left_arm_at_step0_rad=float(left_error[0]), right_arm_at_step0_rad=float(right_error[0]),
        left_arm_first_step_exceeding_rad=first_exceeding(left_error, ARM_MARKS_RAD),
        right_arm_first_step_exceeding_rad=first_exceeding(right_error, ARM_MARKS_RAD),
        left_arm_max_rad=float(left_error.max()), left_arm_max_step=worst, left_arm_max_phase=teacher["phases"][worst],
        left_arm_share_of_steps_where_error_exceeds_teacher_lead=float((left_error > needed_left).mean()),
        left_arm_mean_rad=float(left_error.mean()), teacher_left_lead_mean_rad=float(needed_left.mean()))
    # Left gripper: against the teacher's real open/close command timeline and contact interval
    g = gripper_timeline(teacher)
    pred_left, teacher_left = prediction[steps, 0, LEFT_GRIPPER], q_cmd[steps, LEFT_GRIPPER]
    search_from = max(0, g["close_onset_step"] - 100)
    crossed = [int(t) for t in steps[search_from:] if pred_left[t] <= g["middle_level"]]
    at = lambda step: dict(step=int(step), sim_time_s=round(step * DT, 2), teacher=float(teacher_left[step]), predicted=float(pred_left[step]))
    hold = np.arange(g["both_pads_grip_force_first_step"], window_end + 1)
    closing = np.arange(g["close_onset_step"], g["hold_reached_step"] + 1)
    opening = np.arange(0, g["open_reached_step"] + 1)
    ahead = {}
    for lead in (49, 24, 0):
        t = g["close_onset_step"] - lead
        ahead[f"observed_{lead}_steps_before_close_onset"] = dict(
            observation_step=int(t), predicted_for_onset_plus=[dict(chunk_index=k, target_step=int(t + k), teacher=float(q_cmd[t + k, LEFT_GRIPPER]),
                                                                    predicted=float(prediction[t, k, LEFT_GRIPPER])) for k in KS])
    result["left_gripper"] = dict(
        teacher_timeline=g,
        at_open_reached=at(g["open_reached_step"]), at_close_onset=at(g["close_onset_step"]), at_middle_crossing=at(g["middle_crossed_step"]),
        at_first_pad_contact=at(g["first_pad_contact_step"]), at_grip_force=at(g["both_pads_grip_force_first_step"]),
        at_hold_reached=at(g["hold_reached_step"]), at_window_end=at(window_end),
        predicted_middle_crossing_step=crossed[0] if crossed else None,
        predicted_middle_crossing_lag_steps=(crossed[0] - g["middle_crossed_step"]) if crossed else None,
        opening_interval=dict(steps=[0, g["open_reached_step"]], mae=float(np.abs(pred_left[opening] - teacher_left[opening]).mean()),
                              max=float(np.abs(pred_left[opening] - teacher_left[opening]).max())),
        closing_interval=dict(steps=[g["close_onset_step"], g["hold_reached_step"]], mae=float(np.abs(pred_left[closing] - teacher_left[closing]).mean()),
                              max=float(np.abs(pred_left[closing] - teacher_left[closing]).max()),
                              predicted_min=float(pred_left[closing].min()), teacher_min=float(teacher_left[closing].min())),
        contact_hold_interval=dict(steps=[g["both_pads_grip_force_first_step"], window_end], mae=float(np.abs(pred_left[hold] - teacher_left[hold]).mean()),
                                   max=float(np.abs(pred_left[hold] - teacher_left[hold]).max()),
                                   predicted_range=[float(pred_left[hold].min()), float(pred_left[hold].max())],
                                   teacher_range=[float(teacher_left[hold].min()), float(teacher_left[hold].max())]),
        lookahead=ahead)
    result["series"] = dict(left_arm_k0_max_error_rad=left_error.tolist(), right_arm_k0_max_error_rad=right_error.tolist(),
                            teacher_left_lead_max_rad=needed_left.tolist(), predicted_left_gripper_k0=pred_left.tolist(),
                            teacher_left_gripper=teacher_left.tolist())
    return result


def temporal_offset(teacher, prediction, window_end):
    """Which later teacher command does the executed-index prediction resemble most (left arm)?
    j* = argmin_j mean|q_cmd_teacher[t+j] - q_pred[t,0]|. A description of the error, not a label."""
    q_cmd, left = teacher["q_cmd"], list(LEFT_ARM)
    best, at_zero, at_best = [], [], []
    for t in range(window_end + 1):
        index = np.minimum(t + np.arange(OFFSET_SEARCH_STEPS + 1), len(q_cmd) - 1)
        residual = np.abs(q_cmd[index][:, left] - prediction[t, 0][left]).mean(1)
        best.append(int(residual.argmin()))
        at_zero.append(float(residual[0]))
        at_best.append(float(residual.min()))
    best, at_zero, at_best = np.array(best), np.array(at_zero), np.array(at_best)
    spans = dict(whole_window=[0, window_end], **{p: teacher["phase_spans"][p] for p in WINDOW_PHASES})
    return dict(search_steps=OFFSET_SEARCH_STEPS, first_observation=dict(best_lead_steps=int(best[0]), mae_at_same_step=at_zero[0], mae_at_best_lead=at_best[0]),
                phases={name: dict(best_lead_steps_median=float(np.median(best[a:b + 1])), best_lead_steps_p10=float(np.percentile(best[a:b + 1], 10)),
                                   best_lead_steps_p90=float(np.percentile(best[a:b + 1], 90)), mae_at_same_step=float(at_zero[a:b + 1].mean()),
                                   mae_at_best_lead=float(at_best[a:b + 1].mean())) for name, (a, b) in spans.items()})


def anchoring(teacher, prediction, window_end):
    """How far the executed-index command is from the measured position, against the teacher's own lead."""
    steps = np.arange(window_end + 1)
    out = {}
    for name, channels in (("left_arm", list(LEFT_ARM)), ("right_arm", list(RIGHT_ARM))):
        model = np.abs(prediction[steps, 0][:, channels] - teacher["q_state"][steps][:, channels])
        lead = np.abs(teacher["q_cmd"][steps][:, channels] - teacher["q_state"][steps][:, channels])
        out[name] = dict(model_command_minus_measured_mae_rad=float(model.mean()), teacher_command_minus_measured_mae_rad=float(lead.mean()),
                         ratio=float(model.mean() / lead.mean()) if lead.mean() > 0 else rr.UNAVAILABLE)
    return out


def state_gain(policy, loader, teacher):
    """d q_pred[t,0,j] / d q_state_j: one measured-state input is shifted, the images stay the same."""
    import torch
    import arm_output_trial as trial
    steps = sorted({0, 50} | {(a + b) // 2 for a, b in (teacher["phase_spans"][p] for p in WINDOW_PHASES)})
    std = loader.stats["state"]["std"]
    rows = []
    for t in steps:
        item = trial.split_meta(loader[t])[0]
        base = {k: v[None].cuda() for k, v in item.items() if k.startswith("observation.")}
        with torch.inference_mode():
            reference = policy.predict_canonical_chunk(base).double().cpu().numpy()[0, 0]
        gains = []
        for j in range(12):
            shifted = {k: v.clone() for k, v in base.items()}
            shifted["observation.state"][0, j] += GAIN_PERTURBATION / float(std[j])
            with torch.inference_mode():
                gains.append(float((policy.predict_canonical_chunk(shifted).double().cpu().numpy()[0, 0][j] - reference[j]) / GAIN_PERTURBATION))
        rows.append(dict(step=int(t), sim_time_s=round(t * DT, 2), teacher_phase=teacher["phases"][t], gain=gains))
    table = np.array([r["gain"] for r in rows])
    return dict(perturbation=GAIN_PERTURBATION, rows=rows, mean_gain=table.mean(0).tolist(),
                left_arm_mean=float(table[:, list(LEFT_ARM)].mean()), right_arm_mean=float(table[:, list(RIGHT_ARM)].mean()),
                left_arm_range=[float(table[:, list(LEFT_ARM)].min()), float(table[:, list(LEFT_ARM)].max())],
                right_arm_range=[float(table[:, list(RIGHT_ARM)].min()), float(table[:, list(RIGHT_ARM)].max())],
                note="1 would mean the command moves one-for-one with the measured joint; 0 would mean the state input is not used")


def checkpoint_row(label, prediction, teacher, window_end):
    steps = np.arange(window_end + 1)
    row = dict(checkpoint=label)
    for k in KS:
        for name, channels in (("left_arm", list(LEFT_ARM)), ("right_arm", list(RIGHT_ARM))):
            row[f"k{k}_{name}_mae_rad"] = float(np.abs(prediction[steps, k][:, channels] - teacher["q_cmd"][steps + k][:, channels]).mean())
        row[f"k{k}_left_arm_persistence_mae_rad"] = float(np.abs(teacher["q_cmd"][steps + k][:, list(LEFT_ARM)] - teacher["q_state"][steps][:, list(LEFT_ARM)]).mean())
    row["step0_left_arm_max_error_rad"] = float(np.abs(prediction[0, 0][list(LEFT_ARM)] - teacher["q_cmd"][0][list(LEFT_ARM)]).max())
    return row


# ----------------------------------------------------------------------------------------------
# Student closed loop
# ----------------------------------------------------------------------------------------------

def student_case(record, teacher_seed_episode):
    """Stored student rollout + its relation to the teacher demonstration that started from the same condition."""
    case = Path(record["source_dir"])
    with np.load(case / "episode.npz", allow_pickle=False) as a:
        student = dict(q_cmd=a["q_cmd"].astype(np.float64), q_state=a["q_state"].astype(np.float64), integration_state=a["integration_state"].copy())
    with np.load(case / "predicted_chunks.npz", allow_pickle=False) as a:
        student["chunks"] = a["q_cmd"].astype(np.float64)
    trace = load(case / "action_trace.json")
    student["issued"] = np.array([row["q_cmd"] for row in trace], dtype=np.float64)
    frames = {}
    for side in ("left", "right"):
        with np.load(case / f"cameras/{side}_wrist_rgb.npz", allow_pickle=False) as c, \
                np.load(teacher_seed_episode["path"] / f"cameras/{side}_wrist_rgb.npz", allow_pickle=False) as t:
            frames[side] = int((c["rgb"][0] != t["rgb"][0]).sum())
    teacher = teacher_seed_episode
    same_state = bool(np.array_equal(student["integration_state"][0], teacher["integration_state"][0]))
    student["start"] = dict(
        integration_state_equal_to_teacher_episode=same_state,
        integration_state_max_abs_diff=float(np.abs(student["integration_state"][0] - teacher["integration_state"][0]).max()),
        q_state_max_abs_diff=float(np.abs(student["q_state"][0] - teacher["q_state"][0]).max()),
        first_frame_differing_pixel_values=frames)
    return student


def closed_loop_analysis(record, student, teacher, prediction, window_end):
    """Seed 5000: how the student's own closed loop separates from the demonstration (description, not a label error)."""
    ks, kt = kinematics(student["integration_state"]), kinematics(teacher["integration_state"][:max(window_end + 2, len(student["integration_state"]))])
    n = min(len(student["q_state"]), len(teacher["q_state"]), len(kt["left_tcp"]))
    arm = list(LEFT_ARM + RIGHT_ARM)
    left_dev = np.abs(student["q_state"][:n][:, list(LEFT_ARM)] - teacher["q_state"][:n][:, list(LEFT_ARM)]).max(1)
    right_dev = np.abs(student["q_state"][:n][:, list(RIGHT_ARM)] - teacher["q_state"][:n][:, list(RIGHT_ARM)]).max(1)
    tcp_sep = np.linalg.norm(ks["left_tcp"][:n] - kt["left_tcp"][:n], axis=1)
    horizontal = lambda k: np.linalg.norm((k["left_tcp"] - k["block"])[:, :2], axis=1)
    distance = lambda k: np.linalg.norm(k["left_tcp"] - k["block"], axis=1)
    hs, ht, ds, dt_ = horizontal(ks), horizontal(kt), distance(ks), distance(kt)
    first_cmd = dict(
        student_first_chunk_vs_teacher_observation_prediction_max_abs_diff=(float(np.abs(student["chunks"][0] - prediction[0]).max())
                                                                              if prediction is not None else rr.UNAVAILABLE),
        left_arm_error_vs_teacher_command_rad=float(np.abs(student["issued"][0][list(LEFT_ARM)] - teacher["q_cmd"][0][list(LEFT_ARM)]).max()),
        right_arm_error_vs_teacher_command_rad=float(np.abs(student["issued"][0][list(RIGHT_ARM)] - teacher["q_cmd"][0][list(RIGHT_ARM)]).max()),
        left_gripper_student=float(student["issued"][0][LEFT_GRIPPER]), left_gripper_teacher=float(teacher["q_cmd"][0][LEFT_GRIPPER]),
        note="the first observation is the same for both, so this is the teacher-observation error at t=0")
    end = len(student["integration_state"]) - 1
    # teacher state with the same horizontal TCP-block distance as the student's last state, searched on the teacher's way to the block
    reach_step = int(round(teacher["events"]["reach"] / DT))
    candidates = np.arange(0, min(reach_step, len(ht) - 1) + 1)
    match = int(candidates[np.abs(ht[candidates] - hs[end]).argmin()])
    samples = sorted(set([0, 5, 10, 25, 50, 75, 100, 150, 200, 300, 400, 500, n - 1]) & set(range(n)))
    fault = record["first_fault"]
    out = dict(
        first_command=first_cmd, common_steps=int(n),
        separation=dict(
            note="student state minus teacher state at the same time index; a description of how the runs part, not an error against a label",
            left_arm_joint_first_step_exceeding_rad=first_exceeding(left_dev, ARM_MARKS_RAD),
            right_arm_joint_first_step_exceeding_rad=first_exceeding(right_dev, ARM_MARKS_RAD),
            left_tcp_first_step_exceeding_m=first_exceeding(tcp_sep, TCP_MARKS_M),
            samples=[dict(step=int(t), sim_time_s=round(t * DT, 2), teacher_phase=teacher["phases"][min(t, len(teacher["phases"]) - 1)],
                          left_arm_joint_max_rad=float(left_dev[t]), right_arm_joint_max_rad=float(right_dev[t]), left_tcp_separation_m=float(tcp_sep[t]),
                          student_left_tcp_block_m=float(ds[t]), teacher_left_tcp_block_m=float(dt_[t]),
                          student_left_tcp_height_m=float(ks["left_tcp"][t][2] - ks["table_top_z"]), teacher_left_tcp_height_m=float(kt["left_tcp"][t][2] - kt["table_top_z"]),
                          student_left_gripper_cmd=float(student["issued"][min(t, len(student["issued"]) - 1)][LEFT_GRIPPER]),
                          teacher_left_gripper_cmd=float(teacher["q_cmd"][t][LEFT_GRIPPER])) for t in samples]),
        student_end=dict(
            termination=record["termination"], completed_transitions=record["completed_transitions"], sim_time_s=record["sim_elapsed_s"],
            first_fault=None if fault is None else dict(sim_time_s=fault["sim_time"], reason=fault["reason"], pairs=fault["pairs"]),
            events_before_fault=record["events_valid_prefix"], first_unmet_task_condition=(record["first_unmet_task_condition"] or {}).get("event"),
            left_tcp_block_distance_m=float(ds[end]), left_tcp_block_horizontal_m=float(hs[end]),
            left_tcp_height_above_table_m=float(ks["left_tcp"][end][2] - ks["table_top_z"]), left_pad_lowest_height_m=float(ks["left_pad_low"][end]),
            right_pad_lowest_height_m=float(ks["right_pad_low"][end]),
            left_tcp_block_min_distance_m=float(ds.min()), left_tcp_block_min_distance_step=int(ds.argmin()),
            block_displacement_m=float(np.linalg.norm(ks["block"][end] - ks["block"][0])),
            left_gripper_command_last=float(student["issued"][-1][LEFT_GRIPPER]), left_gripper_command_min=float(student["issued"][:, LEFT_GRIPPER].min()),
            left_gripper_command_max=float(student["issued"][:, LEFT_GRIPPER].max()), q_state_last=student["q_state"][end].tolist()),
        teacher_at_same_horizontal_distance=dict(
            note="teacher state on its way to the block whose horizontal TCP-block distance is closest to the student's last state; geometry only",
            teacher_step=match, teacher_sim_time_s=round(match * DT, 2), teacher_phase=teacher["phases"][match],
            horizontal_distance_m=dict(student=float(hs[end]), teacher=float(ht[match])),
            left_tcp_height_above_table_m=dict(student=float(ks["left_tcp"][end][2] - ks["table_top_z"]), teacher=float(kt["left_tcp"][match][2] - kt["table_top_z"])),
            left_pad_lowest_height_m=dict(student=float(ks["left_pad_low"][end]), teacher=float(kt["left_pad_low"][match])),
            left_gripper_command=dict(student=float(student["issued"][-1][LEFT_GRIPPER]), teacher=float(teacher["q_cmd"][match][LEFT_GRIPPER]))),
        teacher_reference=dict(
            reach_step=reach_step, at_reach=dict(left_tcp_block_m=float(dt_[reach_step]), left_tcp_block_horizontal_m=float(ht[reach_step]),
                                                 left_tcp_height_above_table_m=float(kt["left_tcp"][reach_step][2] - kt["table_top_z"]),
                                                 left_pad_lowest_height_m=float(kt["left_pad_low"][reach_step])),
            descend_start=dict(step=teacher["phase_spans"]["left_descend"][0],
                               left_tcp_block_horizontal_m=float(ht[teacher["phase_spans"]["left_descend"][0]]),
                               left_tcp_height_above_table_m=float(kt["left_tcp"][teacher["phase_spans"]["left_descend"][0]][2] - kt["table_top_z"]))),
        series=dict(student_left_tcp_block_m=ds.tolist(), teacher_left_tcp_block_m=dt_[:window_end + 1].tolist(),
                    student_left_tcp_height_m=(ks["left_tcp"][:, 2] - ks["table_top_z"]).tolist(),
                    teacher_left_tcp_height_m=(kt["left_tcp"][:window_end + 1, 2] - kt["table_top_z"]).tolist(),
                    student_left_tcp_horizontal_m=hs.tolist(), teacher_left_tcp_horizontal_m=ht[:window_end + 1].tolist(),
                    left_tcp_separation_m=tcp_sep.tolist(), left_arm_joint_separation_rad=left_dev.tolist()))
    return out


def gather(run_id):
    base = run_dir(run_id)
    data = dict(run_id=run_id, run_dir=str(base), protocol=load(base / "protocol.json"))
    val = rr.reconcile_val(train_dir(run_id), "arm_bounded on Train parents (aggregation OFF)", horizon_transitions=HORIZON)
    cases = {}
    for record in val["cases"]:
        if record["status"] != "ok":
            raise rr.AggregationError(f"{record['case']}: {record.get('error')}")
        identity = load(Path(record["source_dir"]) / "run_identity.json")
        seed = identity["source_episode"]["seed"]
        if (identity["source_episode"]["split"] != "train" or identity["execution"]["group"] != "train"
                or identity["execution"]["execution_aggregation"] != "off" or record["execution_aggregation"] != "off"
                or identity["checkpoint_sha256"] != data["protocol"]["checkpoint"]["sha256"]):
            raise rr.AggregationError(f"{record['case']} is not a Train / aggregation-off run of the protocol checkpoint")
        teacher = teacher_episode(seed)
        student = student_case(record, teacher)
        record["identity"] = dict(source_episode=identity["source_episode"], reset=identity["reset"], execution=identity["execution"],
                                  backend=identity["backend"], source_sha256=identity["source_sha256"])
        record["start"] = student["start"]
        record["teacher_events_same_initial_condition"] = teacher["events"]
        cases[seed] = (record, student, teacher)
    if sorted(cases) != sorted(TRAIN_SEEDS):
        raise rr.AggregationError(f"expected Train seeds {TRAIN_SEEDS}, found {sorted(cases)}")
    data["train"] = dict(label=val["model_name"], source_dir=val["source_dir"], summary=val["summary"], cases=[cases[s][0] for s in TRAIN_SEEDS])
    # Seed 5000: predictions on the teacher's observations, through the loader the checkpoint was trained with
    import torch
    import arm_output_trial as trial
    from dapier_act_policy import ACTPolicy
    record, student, teacher = cases[DETAIL_SEED]
    window_end = data["protocol"]["detail_seed"]["window"]["steps"][1]
    with np.load(TRAINING / "normalization.npz", allow_pickle=False) as a:
        stats = {k: {s: a[f"{k}_{s}"].copy() for s in ("mean", "std")} for k in ("state", "action")}
    trial.apply_reproducibility_settings()
    loader = trial.TrialEpisodes(DATASET, [teacher["index"]], required_split="train", stats=stats)
    policy = ACTPolicy.from_pretrained(CHECKPOINT, local_files_only=True, strict=True).cuda().eval()
    policy.check_normalization(stats)
    prediction = trial.canonical_chunks(policy, loader, np.arange(window_end + 1))
    gain = state_gain(policy, loader, teacher)
    del policy
    torch.cuda.empty_cache()
    others = [checkpoint_row("현재 모델: ARM_BOUNDED_CANDIDATE step-005000", prediction, teacher, window_end)]
    for label, relative in OTHER_CHECKPOINTS:   # stored checkpoints, inference only; nothing is trained
        other = ACTPolicy.from_pretrained(FACTORY_ROOT / relative, local_files_only=True, strict=True).cuda().eval()
        other.check_normalization(stats)
        others.append(checkpoint_row(label, trial.canonical_chunks(other, loader, np.arange(window_end + 1)), teacher, window_end))
        del other
        torch.cuda.empty_cache()
    labels = np.stack([loader.target_rad(int(t))[0] for t in range(window_end + 1)])
    if not np.allclose(labels[:, 0], teacher["q_cmd"][:window_end + 1], rtol=0, atol=1e-6):
        raise rr.AggregationError("loader labels are not the teacher commands at the observation index")
    data["seed_detail"] = dict(
        seed=DETAIL_SEED, source_episode=str(teacher["path"]), window_end_step=int(window_end),
        loader_check=dict(loader="arm_output_trial.TrialEpisodes", observations=int(len(prediction)), prediction_dtype=str(prediction.dtype),
                          label_at_chunk_index_0_equals_teacher_command_at_same_step_max_abs_diff=float(np.abs(labels[:, 0] - teacher["q_cmd"][:window_end + 1]).max()),
                          label_at_chunk_index_24_equals_teacher_command_24_steps_later_max_abs_diff=float(np.abs(labels[:, 24] - teacher["q_cmd"][24:window_end + 25]).max()),
                          note="labels are float32 in the loader; differences are float32 rounding"),
        teacher_observation=teacher_observation_analysis(teacher, prediction, window_end),
        closed_loop=closed_loop_analysis(record, student, teacher, prediction, window_end),
        supplementary=dict(
            note="added after the rollouts were seen; inference on stored checkpoints and stored observations only",
            anchoring=anchoring(teacher, prediction, window_end), temporal_offset=temporal_offset(teacher, prediction, window_end),
            state_gain=gain, stored_checkpoints_same_observations=others))
    for seed in TRAIN_SEEDS:   # the same closed-loop description for every Train parent
        record, student, teacher = cases[seed]
        end = teacher["phase_spans"][WINDOW_PHASES[-1]][1]
        loop = closed_loop_analysis(record, student, teacher, prediction if seed == DETAIL_SEED else None, end)
        record["closed_loop_summary"] = dict(first_command=loop["first_command"], separation={k: v for k, v in loop["separation"].items() if k != "samples"},
                                             student_end=loop["student_end"], teacher_at_same_horizontal_distance=loop["teacher_at_same_horizontal_distance"],
                                             teacher_reference=loop["teacher_reference"])
    return data


# ----------------------------------------------------------------------------------------------
# Markdown
# ----------------------------------------------------------------------------------------------

def fault_text(record):
    fault = record["first_fault"]
    if fault is None:
        return "-"
    if fault["reason"] == "student_action_rejected":
        joints = (record.get("rejected_command") or {}).get("joints_outside_limits") or []
        return f"{fault['sim_time']:.3f}s 범위 거부: " + ", ".join(f"{j['joint']} {j['command']:.5f}" for j in joints)
    pairs = "; ".join("–".join(p) for p in fault["pairs"]) if isinstance(fault["pairs"], list) else str(fault["pairs"])
    return f"{fault['sim_time']:.3f}s 접촉: {pairs}"


def fmt_events(events):
    return ", ".join(f"{k}@{v:.2f}s" for k, v in events.items()) or "없음"


def markdown(data):
    d = data["seed_detail"]
    obs, loop = d["teacher_observation"], d["closed_loop"]
    out = [f"# Train 원래 초기조건 진단 (run-id: {data['run_id']})", "",
           "**Train 진단이다. Val/Test 성공률이나 일반화 성능이 아니며 Val 표와 합치지 않는다.**",
           f"checkpoint `{data['protocol']['checkpoint']['path']}`, C1, ensemble OFF, 교사 개입·pregrasp reset 없음. 시간은 SIM 시간.", ""]

    out += ["## 1. Train 3건 — 학습에 쓴 parent의 원래 초기조건에서 정책만으로 실행", "",
            "| seed | manifest index / split / parent | 원본 episode | 시작 상태 = 교사 episode 시작 상태 | 완료 transition | checker.valid | 종료 / 첫 fault | fault 이전 이벤트 | grasp / lift / handover | 첫 미달 조건 | 같은 초기조건에서 교사의 이벤트 |",
            "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in data["train"]["cases"]:
        src, ev = r["identity"]["source_episode"], r["events_valid_prefix"]
        chain = " / ".join("예" if e in ev else "아니오" for e in ("left_grasp", "pick_lift", "handover"))
        out.append(f"| {src['seed']} | {src['manifest_index']} / {src['split']} / {src['parent_physics_group']} | `{Path(src['path']).name}` | "
                   f"{r['start']['integration_state_equal_to_teacher_episode']} (최대 차 {r['start']['integration_state_max_abs_diff']:.1e}) | "
                   f"{r['completed_transitions']} | {r['checker_valid']} | {r['termination']} / {fault_text(r)} | {fmt_events(ev)} | {chain} | "
                   f"{(r['first_unmet_task_condition'] or {}).get('event', '-')} | {fmt_events(r['teacher_events_same_initial_condition'])} |")
    s = data["train"]["summary"]
    out += ["", f"합계(Train 3건): checker.valid {s['checker_valid']['count']}/3, reach {s['events_valid_prefix']['reach']['count']}/3, "
            f"left_grasp {s['events_valid_prefix']['left_grasp']['count']}/3, pick_lift {s['events_valid_prefix']['pick_lift']['count']}/3, "
            f"handover {s['events_valid_prefix']['handover']['count']}/3, terminal pick_lift 성공 {s['terminal_pick_lift_success']['count']}/3, "
            f"terminal handover 성공 {s['terminal_handover_success']['count']}/3.", "",
            "reset: 각 seed의 `requested_initial_state.q`와 `sampling.block_xy_yaw`를 명시적으로 전달했다.", "",
            "| seed | q0 (12, canonical) | block x, y, yaw |", "|---|---|---|"]
    for r in data["train"]["cases"]:
        reset = r["identity"]["reset"]
        out.append(f"| {reset['seed']} | {', '.join(f'{v:.4f}' for v in reset['q0'])} | {', '.join(f'{v:.4f}' for v in reset['block_xy_yaw'])} |")
    out += ["", rr.markdown_table(dict(cases=data["train"]["cases"]))]
    out += ["", "세 건의 첫 이탈과 마지막 자세 (같은 초기 관측이므로 첫 명령만 교사 명령과 직접 비교한다):", "",
            "| seed | 첫 명령과 교사 첫 명령의 차 (좌팔 / 우팔, rad) | 좌 TCP가 교사와 5 cm 벌어진 시각 (s) | 마지막: 블록까지 수평 거리 / 좌 TCP 높이 / 좌 패드 최저점 (cm) | 교사가 같은 수평 거리였을 때 좌 TCP 높이 (cm) | 교사의 하강 시작 시각 / reach 시각 (s) |",
            "|---|---|---|---|---|---|"]
    for r in data["train"]["cases"]:
        c = r["closed_loop_summary"]
        five = c["separation"]["left_tcp_first_step_exceeding_m"]["0.05"]
        e, same, ref = c["student_end"], c["teacher_at_same_horizontal_distance"], c["teacher_reference"]
        out.append(f"| {r['identity']['source_episode']['seed']} | {c['first_command']['left_arm_error_vs_teacher_command_rad']:.3f} / {c['first_command']['right_arm_error_vs_teacher_command_rad']:.3f} | "
                   f"{(f'{five * DT:.2f}' if five is not None else '없음')} | {e['left_tcp_block_horizontal_m'] * 100:.1f} / {e['left_tcp_height_above_table_m'] * 100:.1f} / {e['left_pad_lowest_height_m'] * 100:.1f} | "
                   f"{same['left_tcp_height_above_table_m']['teacher'] * 100:.1f} (교사 {same['teacher_sim_time_s']:.2f} s, {same['teacher_phase']}) | "
                   f"{ref['descend_start']['step'] * DT:.2f} / {ref['reach_step'] * DT:.2f} |")

    g = obs["left_gripper"]["teacher_timeline"]
    out += ["", f"## 2. Seed {d['seed']} — 교사 관측에서의 배포 예측 대 교사 라벨 (모방 오차, 폐루프와 별개)", "",
            f"교사 시연 `{Path(d['source_episode']).name}`의 0–{d['window_end_step'] * DT:.2f} s(교사 단계 {' → '.join(WINDOW_PHASES)}), 관측 {obs['observations']}개. "
            "학습에 쓴 로더가 주는 입력으로 평가기와 같은 함수(eval, z=0)를 호출했다. 교사 정보는 정답 비교에만 썼다.", "",
            "persistence = 현재 실측 위치를 그대로 내는 기준. 그 오차는 교사가 그 시점에 요구한 이동량과 같다.", "",
            "| 교사 단계 (SIM s) | k (앞선 시간) | 좌팔 MAE 모델 / persistence (rad) | 좌팔 방향 cosine 평균 (음수 비율) | 좌팔 이동량 비 (예측/교사, 중앙값) | 우팔 MAE 모델 / persistence (rad) | 좌 그리퍼 MAE 모델 / persistence | 우 그리퍼 MAE 모델 / persistence |",
            "|---|---|---|---|---|---|---|---|"]
    def line(name, block):
        rows = []
        for k in KS:
            b = block[f"k{k}"]
            la, ra, lg, rg = b["left_arm"], b["right_arm"], b["left_gripper"], b["right_gripper"]
            cosine = f"{la['direction_cosine_mean']:.2f} ({la['share_direction_cosine_below_0']:.0%})" if la.get("moving_observations") else "교사 이동 없음"
            ratio = f"{la['displacement_magnitude_ratio_median']:.2f}" if la.get("moving_observations") else "-"
            rows.append(f"| {name} ({block['sim_time_s'][0]:.2f}–{block['sim_time_s'][1]:.2f}) | {k} ({k * DT:.2f} s) | {la['model_mae']:.4f} / {la['persistence_mae']:.4f} | "
                        f"{cosine} | {ratio} | {ra['model_mae']:.4f} / {ra['persistence_mae']:.4f} | {lg['model_mae']:.4f} / {lg['persistence_mae']:.4f} | "
                        f"{rg['model_mae']:.4f} / {rg['persistence_mae']:.4f} |")
        return rows
    out += line("전체 구간", obs["whole_window"])
    for name in WINDOW_PHASES:
        out += line(name, obs["phases"][name])
    f = obs["first_command_k0"]
    out += ["", "실행에 쓰이는 k=0 명령(관측별 좌팔 최대 관절 오차):",
            f"- 첫 관측(t=0): 좌팔 {f['left_arm_at_step0_rad']:.4f} rad, 우팔 {f['right_arm_at_step0_rad']:.4f} rad.",
            f"- 좌팔 오차가 처음 넘는 스텝: " + ", ".join(f"{m} rad → {('스텝 ' + str(v) + f' ({v * DT:.2f} s)') if v is not None else '없음'}" for m, v in f["left_arm_first_step_exceeding_rad"].items()) + ".",
            f"- 최대 {f['left_arm_max_rad']:.4f} rad (스텝 {f['left_arm_max_step']}, {f['left_arm_max_step'] * DT:.2f} s, 교사 단계 {f['left_arm_max_phase']}). 평균 {f['left_arm_mean_rad']:.4f} rad, "
            f"교사 명령이 현재 위치보다 앞선 양(평균) {f['teacher_left_lead_mean_rad']:.4f} rad. 오차가 그 앞선 양보다 큰 관측 비율 {f['left_arm_share_of_steps_where_error_exceeds_teacher_lead']:.0%}."]
    lg = obs["left_gripper"]
    cell = lambda x: f"{x['sim_time_s']:.2f} s: 교사 {x['teacher']:.3f}, 예측 {x['predicted']:.3f}"
    out += ["", "좌 그리퍼 — 이 시연의 실제 교사 명령·접촉 구간과 비교 (opening fraction, 1 = 개방):", "",
            "| 교사 시점 | 교사 명령 / 교사 관측에서의 k=0 예측 |", "|---|---|",
            f"| 시작 명령 | 0.00 s: 교사 {g['first_command']:.3f}, 예측 {obs['series']['predicted_left_gripper_k0'][0]:.3f} |",
            f"| 완전 개방 도달 (스텝 {g['open_reached_step']}) | {cell(lg['at_open_reached'])} |",
            f"| 닫힘 시작 (스텝 {g['close_onset_step']}) | {cell(lg['at_close_onset'])} |",
            f"| 개방–파지 중간값 {g['middle_level']:.3f} 통과 (스텝 {g['middle_crossed_step']}) | {cell(lg['at_middle_crossing'])} |",
            f"| 첫 패드 접촉 (스텝 {g['first_pad_contact_step']}) | {cell(lg['at_first_pad_contact'])} |",
            f"| 양 패드 {g['grip_force_min_N']} N 이상 시작 (스텝 {g['both_pads_grip_force_first_step']}) | {cell(lg['at_grip_force'])} |",
            f"| 파지 유지 수준 {g['hold_level']:.3f} 도달 (스텝 {g['hold_reached_step']}) | {cell(lg['at_hold_reached'])} |",
            f"| 구간 끝 (스텝 {d['window_end_step']}) | {cell(lg['at_window_end'])} |", "",
            f"- 예측이 같은 중간값을 통과한 스텝: {lg['predicted_middle_crossing_step']} (교사 대비 {lg['predicted_middle_crossing_lag_steps']:+d} 스텝)." if lg["predicted_middle_crossing_step"] is not None
            else "- 예측은 구간 안에서 개방–파지 중간값 아래로 내려가지 않았다.",
            f"- 개방 구간(스텝 {lg['opening_interval']['steps'][0]}–{lg['opening_interval']['steps'][1]}): 평균 오차 {lg['opening_interval']['mae']:.4f}, 최대 {lg['opening_interval']['max']:.4f}.",
            f"- 닫힘 구간(스텝 {lg['closing_interval']['steps'][0]}–{lg['closing_interval']['steps'][1]}): 평균 오차 {lg['closing_interval']['mae']:.4f}, 최대 {lg['closing_interval']['max']:.4f}; "
            f"최솟값 교사 {lg['closing_interval']['teacher_min']:.3f}, 예측 {lg['closing_interval']['predicted_min']:.3f}.",
            f"- 접촉 유지 구간(스텝 {lg['contact_hold_interval']['steps'][0]}–{lg['contact_hold_interval']['steps'][1]}): 평균 오차 {lg['contact_hold_interval']['mae']:.4f}, 최대 {lg['contact_hold_interval']['max']:.4f}; "
            f"범위 교사 {lg['contact_hold_interval']['teacher_range'][0]:.3f}–{lg['contact_hold_interval']['teacher_range'][1]:.3f}, 예측 {lg['contact_hold_interval']['predicted_range'][0]:.3f}–{lg['contact_hold_interval']['predicted_range'][1]:.3f}.",
            "", "닫힘 시작 이전 관측에서 미리 예측한 값 (chunk index k → 목표 스텝):", "", "| 관측 시점 | k=0 | k=24 | k=49 |", "|---|---|---|---|"]
    for name, item in lg["lookahead"].items():
        cells = " | ".join(f"스텝 {p['target_step']}: 교사 {p['teacher']:.3f}, 예측 {p['predicted']:.3f}" for p in item["predicted_for_onset_plus"])
        out.append(f"| 닫힘 시작 {name.split('_')[1]} 스텝 전 (스텝 {item['observation_step']}) | {cells} |")

    fc, end, same = loop["first_command"], loop["student_end"], loop["teacher_at_same_horizontal_distance"]
    start = data["train"]["cases"][TRAIN_SEEDS.index(DETAIL_SEED)]["start"]
    out += ["", f"## 3. Seed {d['seed']} — 학생 자체 상태에서의 폐루프 (교사 라벨 오차가 아니라 갈라짐의 기술)", "",
            f"- 시작: 학생의 초기 상태는 교사 episode의 초기 상태와 {'같다' if start['integration_state_equal_to_teacher_episode'] else '다르다'}"
            f"(최대 차 {start['integration_state_max_abs_diff']:.1e}); 첫 손목 프레임에서 다른 화소값 좌 {start['first_frame_differing_pixel_values']['left']}개, 우 {start['first_frame_differing_pixel_values']['right']}개.",
            f"- 첫 명령: 학생의 첫 chunk와 교사 관측 t=0에서의 예측의 최대 차 {fc['student_first_chunk_vs_teacher_observation_prediction_max_abs_diff']:.1e}. "
            f"교사 첫 명령과의 차이 좌팔 {fc['left_arm_error_vs_teacher_command_rad']:.4f} rad, 우팔 {fc['right_arm_error_vs_teacher_command_rad']:.4f} rad; "
            f"좌 그리퍼 학생 {fc['left_gripper_student']:.3f}, 교사 {fc['left_gripper_teacher']:.3f}.",
            "- 같은 시각의 학생 상태와 교사 상태가 처음 벌어진 스텝: 좌팔 관절 "
            + ", ".join(f"{m} rad → {('스텝 ' + str(v) + f' ({v * DT:.2f} s)') if v is not None else '없음'}" for m, v in loop["separation"]["left_arm_joint_first_step_exceeding_rad"].items())
            + "; 좌 TCP " + ", ".join(f"{float(m) * 100:.0f} cm → {('스텝 ' + str(v) + f' ({v * DT:.2f} s)') if v is not None else '없음'}" for m, v in loop["separation"]["left_tcp_first_step_exceeding_m"].items()) + ".", "",
            "| SIM s (교사 단계) | 좌팔 관절 차 최대 (rad) | 좌 TCP 간 거리 (cm) | 좌 TCP–블록 거리 학생 / 교사 (cm) | 좌 TCP 높이 학생 / 교사 (cm) | 좌 그리퍼 명령 학생 / 교사 |", "|---|---|---|---|---|---|"]
    for row in loop["separation"]["samples"]:
        out.append(f"| {row['sim_time_s']:.2f} ({row['teacher_phase']}) | {row['left_arm_joint_max_rad']:.4f} | {row['left_tcp_separation_m'] * 100:.2f} | "
                   f"{row['student_left_tcp_block_m'] * 100:.1f} / {row['teacher_left_tcp_block_m'] * 100:.1f} | "
                   f"{row['student_left_tcp_height_m'] * 100:.1f} / {row['teacher_left_tcp_height_m'] * 100:.1f} | "
                   f"{row['student_left_gripper_cmd']:.3f} / {row['teacher_left_gripper_cmd']:.3f} |")
    fault = end["first_fault"]
    ref = loop["teacher_reference"]
    out += ["", "학생의 첫 fault 또는 정체:",
            f"- 종료: {end['termination']}, {end['completed_transitions']} transition, SIM {end['sim_time_s']} s"
            + (f"; 첫 fault {fault['sim_time_s']:.3f} s, {fault['reason']}, 대상 {fault['pairs']}" if fault else "; fault 없음") + ".",
            f"- fault 이전 이벤트: {fmt_events(end['events_before_fault'])}; 첫 미달 조건: {end['first_unmet_task_condition']}.",
            f"- 마지막 상태: 좌 TCP–블록 거리 {end['left_tcp_block_distance_m'] * 100:.1f} cm(수평 {end['left_tcp_block_horizontal_m'] * 100:.1f} cm), 좌 TCP 높이 {end['left_tcp_height_above_table_m'] * 100:.1f} cm, "
            f"좌 패드 최저점 높이 {end['left_pad_lowest_height_m'] * 100:.1f} cm, 우 패드 최저점 높이 {end['right_pad_lowest_height_m'] * 100:.1f} cm, 블록 이동 {end['block_displacement_m'] * 1000:.1f} mm.",
            f"- 좌 TCP가 블록에 가장 가까웠던 때: {end['left_tcp_block_min_distance_m'] * 100:.1f} cm (스텝 {end['left_tcp_block_min_distance_step']}, {end['left_tcp_block_min_distance_step'] * DT:.2f} s).",
            f"- 좌 그리퍼 명령: 최소 {end['left_gripper_command_min']:.3f}, 최대 {end['left_gripper_command_max']:.3f}, 마지막 {end['left_gripper_command_last']:.3f}.", "",
            "같은 수평 거리에서의 자세(기하 비교, 교사 행동을 학생의 정답으로 쓰지 않음):",
            f"- 학생 마지막 상태의 블록까지 수평 거리 {same['horizontal_distance_m']['student'] * 100:.1f} cm. 교사가 블록으로 가는 길에 같은 수평 거리였던 때는 스텝 {same['teacher_step']} "
            f"({same['teacher_sim_time_s']:.2f} s, {same['teacher_phase']}, 수평 {same['horizontal_distance_m']['teacher'] * 100:.1f} cm).",
            f"- 그때 좌 TCP 높이: 학생 {same['left_tcp_height_above_table_m']['student'] * 100:.1f} cm, 교사 {same['left_tcp_height_above_table_m']['teacher'] * 100:.1f} cm. "
            f"좌 패드 최저점: 학생 {same['left_pad_lowest_height_m']['student'] * 100:.1f} cm, 교사 {same['left_pad_lowest_height_m']['teacher'] * 100:.1f} cm. "
            f"좌 그리퍼 명령: 학생 {same['left_gripper_command']['student']:.3f}, 교사 {same['left_gripper_command']['teacher']:.3f}.",
            f"- 교사 기준: 하강 시작(스텝 {ref['descend_start']['step']}) 때 수평 {ref['descend_start']['left_tcp_block_horizontal_m'] * 100:.1f} cm·높이 {ref['descend_start']['left_tcp_height_above_table_m'] * 100:.1f} cm, "
            f"reach(스텝 {ref['reach_step']}) 때 수평 {ref['at_reach']['left_tcp_block_horizontal_m'] * 100:.1f} cm·높이 {ref['at_reach']['left_tcp_height_above_table_m'] * 100:.1f} cm·패드 최저점 {ref['at_reach']['left_pad_lowest_height_m'] * 100:.1f} cm."]
    sup = d["supplementary"]
    anchor, offset, gain = sup["anchoring"], sup["temporal_offset"], sup["state_gain"]
    out += ["", f"## 4. 보조 측정 — 실행 결과를 본 뒤 추가 (저장된 checkpoint·관측에 대한 추론만, 새 학습 없음)", "",
            "실행 인덱스(k=0) 명령이 실측 위치에서 떨어진 정도 (전체 구간 평균, 관절당):",
            f"- 좌팔: 모델 {anchor['left_arm']['model_command_minus_measured_mae_rad']:.4f} rad, 교사 {anchor['left_arm']['teacher_command_minus_measured_mae_rad']:.4f} rad ({anchor['left_arm']['ratio']:.0f}배).",
            f"- 우팔: 모델 {anchor['right_arm']['model_command_minus_measured_mae_rad']:.4f} rad, 교사 {anchor['right_arm']['teacher_command_minus_measured_mae_rad']:.5f} rad (교사는 이 구간에서 우팔을 거의 움직이지 않는다).", "",
            f"k=0 예측과 가장 가까운 교사 명령이 몇 스텝 뒤의 것인지 (좌팔, {offset['search_steps']}스텝 앞까지 탐색):", "",
            "| 구간 | 가장 가까운 교사 명령의 앞선 스텝 (중앙값, 10–90%) | 같은 스텝 교사 명령과의 MAE (rad) | 가장 가까운 교사 명령과의 MAE (rad) |", "|---|---|---|---|",
            f"| 첫 관측 (t=0) | {offset['first_observation']['best_lead_steps']} ({offset['first_observation']['best_lead_steps'] * DT:.2f} s) | {offset['first_observation']['mae_at_same_step']:.4f} | {offset['first_observation']['mae_at_best_lead']:.4f} |"]
    for name, row in offset["phases"].items():
        out.append(f"| {name} | {row['best_lead_steps_median']:.0f} ({row['best_lead_steps_median'] * DT:.2f} s), {row['best_lead_steps_p10']:.0f}–{row['best_lead_steps_p90']:.0f} | "
                   f"{row['mae_at_same_step']:.4f} | {row['mae_at_best_lead']:.4f} |")
    out += ["", f"실측 상태 입력 하나를 {gain['perturbation']} 만큼 바꿨을 때 같은 채널 k=0 명령의 변화 비율 (영상은 그대로; 1이면 실측값을 그대로 따라감, 0이면 상태 입력 미사용):", "",
            "| 관측 (교사 단계) | 좌팔 5축 | 좌 그리퍼 | 우팔 5축 | 우 그리퍼 |", "|---|---|---|---|---|"]
    for row in gain["rows"]:
        g_ = row["gain"]
        out.append(f"| {row['sim_time_s']:.2f} s ({row['teacher_phase']}) | {', '.join(f'{g_[j]:.2f}' for j in LEFT_ARM)} | {g_[LEFT_GRIPPER]:.2f} | "
                   f"{', '.join(f'{g_[j]:.2f}' for j in RIGHT_ARM)} | {g_[RIGHT_GRIPPER]:.2f} |")
    out += ["", f"좌팔 평균 {gain['left_arm_mean']:.2f} (범위 {gain['left_arm_range'][0]:.2f}–{gain['left_arm_range'][1]:.2f}), 우팔 평균 {gain['right_arm_mean']:.2f} "
            f"(범위 {gain['right_arm_range'][0]:.2f}–{gain['right_arm_range'][1]:.2f}). 상태 입력은 연결돼 있지만 1:1로 따라가지 않는다.", "",
            "같은 교사 관측에서 기존 checkpoint들의 좌팔 오차 (rad, 전체 구간 평균; persistence는 k=0 "
            f"{sup['stored_checkpoints_same_observations'][0]['k0_left_arm_persistence_mae_rad']:.4f}, k=24 {sup['stored_checkpoints_same_observations'][0]['k24_left_arm_persistence_mae_rad']:.4f}, "
            f"k=49 {sup['stored_checkpoints_same_observations'][0]['k49_left_arm_persistence_mae_rad']:.4f}):", "",
            "| checkpoint | k=0 좌팔 / 우팔 | k=24 좌팔 | k=49 좌팔 | t=0 좌팔 최대 관절 오차 |", "|---|---|---|---|---|"]
    for row in sup["stored_checkpoints_same_observations"]:
        out.append(f"| {row['checkpoint']} | {row['k0_left_arm_mae_rad']:.4f} / {row['k0_right_arm_mae_rad']:.4f} | {row['k24_left_arm_mae_rad']:.4f} | "
                   f"{row['k49_left_arm_mae_rad']:.4f} | {row['step0_left_arm_max_error_rad']:.4f} |")
    out += ["", "## 확인에 쓴 입력 규격", "",
            f"- 로더 `{d['loader_check']['loader']}`: chunk index 0의 라벨 = 같은 스텝의 교사 명령(최대 차 {d['loader_check']['label_at_chunk_index_0_equals_teacher_command_at_same_step_max_abs_diff']:.1e}), "
            f"index 24의 라벨 = 24스텝 뒤 교사 명령(최대 차 {d['loader_check']['label_at_chunk_index_24_equals_teacher_command_24_steps_later_max_abs_diff']:.1e}). float32 반올림 수준.",
            f"- 예측 dtype {d['loader_check']['prediction_dtype']}, 관측 {d['loader_check']['observations']}개, 구간 안에 padding 없음."]
    return "\n".join(out) + "\n"


def analyze(args):
    data = gather(args.run_id)
    target = Path(args.output_dir) if args.output_dir else Path(data["run_dir"])
    target.mkdir(parents=True, exist_ok=True)
    json_path, md_path = target / "analysis.json", target / "analysis.md"
    if json_path.exists() or md_path.exists():
        raise FileExistsError(f"{json_path} / {md_path} exist; nothing is overwritten")
    json_path.write_text(json.dumps(data, indent=2, default=str))
    text = markdown(data)
    md_path.write_text(text)
    print(text)
    print(f"[written] {json_path}\n[written] {md_path}")


# ----------------------------------------------------------------------------------------------
# Video: teacher demonstration (reference) and student rollout from the same initial condition
# ----------------------------------------------------------------------------------------------

def video(args):
    import cv2
    import mujoco
    import arm_output_trial  # noqa: F401
    from integration_scenes import task_env
    base = run_dir(args.run_id)
    out_dir = base / "videos"
    out_dir.mkdir(parents=True, exist_ok=True)
    if (out_dir / f"train_s{DETAIL_SEED}_teacher_vs_student.mp4").exists() or (out_dir / "index.json").exists():
        raise FileExistsError(f"{out_dir} already holds the video; nothing is overwritten")
    val = rr.reconcile_val(train_dir(args.run_id), "train", horizon_transitions=HORIZON)
    record = next(r for r in val["cases"] if load(Path(r["source_dir"]) / "run_identity.json")["source_episode"]["seed"] == DETAIL_SEED)
    teacher = teacher_episode(DETAIL_SEED)
    case = Path(record["source_dir"])
    with np.load(case / "episode.npz", allow_pickle=False) as a:
        student_states, student_cmd = a["integration_state"].copy(), a["q_cmd"].copy()
    trace = load(case / "scoring_trace.json")
    window_end = load(base / "protocol.json")["detail_seed"]["window"]["steps"][1]
    wrist = {}
    for name, path in (("student", case), ("teacher", teacher["path"])):
        for side in ("left", "right"):
            with np.load(path / f"cameras/{side}_wrist_rgb.npz", allow_pickle=False) as cam:
                wrist[name, side] = cam["rgb"]
    runs = [dict(label="TEACHER demo: reference, not the student's target", states=teacher["integration_state"],
                 commands=teacher["q_cmd"], rows=teacher["rows"], wrist={s: wrist["teacher", s] for s in ("left", "right")}, teacher=True),
            dict(label="STUDENT arm_bounded, policy only (C1, ensemble OFF)", states=student_states, commands=student_cmd, rows=trace["contacts"],
                 wrist={s: wrist["student", s] for s in ("left", "right")}, teacher=False)]
    frames = min(max(len(student_states), window_end + 51), HORIZON + 1)
    model = task_env("desk", grippers="both").model
    mj_data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=360, width=480)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth, camera.elevation, camera.distance = 180, -35, 0.8
    camera.lookat[:] = (0.15, 0.0, 0.08)
    spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    site, block = model.site("left_cube_grasp").id, model.body("red_block").id
    width, row_height, header = 720, 360, 30
    out_path = out_dir / f"train_s{DETAIL_SEED}_teacher_vs_student.mp4"
    temp = out_path.with_suffix(".raw.mp4")
    writer = cv2.VideoWriter(str(temp), cv2.VideoWriter_fourcc(*"mp4v"), 25.0, (width, header + 2 * row_height))
    text = lambda img, s, xy, color=(255, 255, 255), scale=0.42: cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
    try:
        for k in range(0, frames, 2):
            canvas = np.zeros((header + 2 * row_height, width, 3), np.uint8)
            text(canvas, f"Train seed {DETAIL_SEED}  SIM t={k * DT:5.2f}s  same initial condition (Train diagnostic, not Val)", (8, 20), (0, 255, 255), 0.5)
            for row, run in enumerate(runs):
                n = len(run["states"])
                j = min(k, n - 1)
                mujoco.mj_setState(model, mj_data, run["states"][j], spec)
                mujoco.mj_forward(model, mj_data)
                renderer.update_scene(mj_data, camera=camera)
                panel = np.zeros((row_height, width, 3), np.uint8)
                panel[:, :480] = renderer.render()
                w = min(j, len(run["wrist"]["left"]) - 1)
                panel[:180, 480:] = cv2.resize(run["wrist"]["left"][w], (240, 180))
                panel[180:, 480:] = cv2.resize(run["wrist"]["right"][w], (240, 180))
                panel = cv2.cvtColor(panel, cv2.COLOR_RGB2BGR)
                contact = run["rows"][min(j, len(run["rows"]) - 1)]
                distance = float(np.linalg.norm(mj_data.site_xpos[site] - mj_data.xpos[block]))
                command = run["commands"][min(j, len(run["commands"]) - 1)][LEFT_GRIPPER]
                cv2.rectangle(panel, (0, 0), (480, 22), (20, 20, 20), -1)
                text(panel, run["label"], (6, 16), (0, 255, 255), 0.4)
                cv2.rectangle(panel, (0, row_height - 40), (480, row_height), (20, 20, 20), -1)
                left = contact["pad_force_N"]["left"]
                text(panel, f"L tcp-block {distance * 100:5.1f}cm  L pads {left[0]:6.2f}/{left[1]:6.2f} N  L grip cmd {command:.3f}", (6, row_height - 24), (200, 255, 200))
                text(panel, f"block bottom clr {contact['block_bottom_clearance_m'] * 1000:5.1f} mm  table support {contact['table_support_N']:6.2f} N",
                     (6, row_height - 8), (200, 255, 200))
                text(panel, "left wrist", (486, 14))
                text(panel, "right wrist", (486, 194))
                if run["teacher"]:
                    phase = teacher["phases"][min(j, len(teacher["phases"]) - 1)]
                    cv2.rectangle(panel, (0, 22), (480, 42), (60, 40, 20), -1)
                    shown = [e for e, when in teacher["events"].items() if k * DT >= when]
                    text(panel, f"teacher phase: {phase}" + (f"   events: {', '.join(shown)}" if shown else ""), (6, 37), (255, 220, 160), 0.42)
                else:
                    shown = [f"{e} @ {when:.2f}s" for e, when in record["events_valid_prefix"].items() if k * DT >= when]
                    if shown:
                        cv2.rectangle(panel, (0, 22), (480, 42), (20, 60, 20), -1)
                        text(panel, "event: " + ", ".join(shown), (6, 37), (0, 255, 0), 0.45)
                    if k >= n - 1:
                        cv2.rectangle(panel, (0, 44), (480, 84), (0, 0, 120), -1)
                        reason = record["termination"] if record["termination"] != "horizon_completed" else "horizon completed (not a task success)"
                        text(panel, f"ENDED at SIM {record['sim_elapsed_s']:.2f}s after {record['completed_transitions']} transitions", (8, 60), (255, 255, 255), 0.45)
                        text(panel, reason, (8, 78), (255, 255, 255), 0.45)
                canvas[header + row * row_height: header + (row + 1) * row_height] = panel
            writer.write(canvas)
    finally:
        writer.release()
        renderer.close()
    subprocess.run(["ffmpeg", "-y", "-i", str(temp), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22", str(out_path)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    temp.unlink()
    index = [dict(file=str(out_path), seed=DETAIL_SEED, frames=len(range(0, frames, 2)), fps=25,
                  layout="top: stored teacher demonstration (reference only); bottom: student rollout; left: overview re-rendered from stored states, "
                         "right: stored wrist frames; playback time equals SIM time",
                  length_rule="until the later of the student's end and 1 s after the teacher's left_lift phase, capped at the 40 s horizon")]
    (out_dir / "index.json").write_text(json.dumps(index, indent=2))
    print(f"[written] {out_path}")


def trace(args):
    """Per-step CSV of seed 5000 from the stored analysis: teacher-observation prediction and closed-loop separation."""
    base = run_dir(args.run_id)
    data = load(base / "analysis.json")
    out_path = base / f"seed{DETAIL_SEED}_trace.csv"
    if out_path.exists():
        raise FileExistsError(f"{out_path} exists; nothing is overwritten")
    teacher = teacher_episode(DETAIL_SEED)
    obs, loop = data["seed_detail"]["teacher_observation"]["series"], data["seed_detail"]["closed_loop"]["series"]
    student_steps = len(loop["student_left_tcp_block_m"])
    columns = ["step", "sim_time_s", "teacher_phase", "teacher_left_gripper_cmd", "predicted_left_gripper_k0_on_teacher_obs",
               "left_arm_k0_max_joint_error_rad_on_teacher_obs", "teacher_left_arm_lead_max_joint_rad", "right_arm_k0_max_joint_error_rad_on_teacher_obs",
               "teacher_left_tcp_block_m", "teacher_left_tcp_height_m", "student_left_tcp_block_m", "student_left_tcp_height_m",
               "student_teacher_left_tcp_separation_m", "student_teacher_left_arm_joint_separation_rad"]
    lines = [",".join(columns)]
    for t in range(data["seed_detail"]["window_end_step"] + 1):
        own = t < student_steps
        row = [t, f"{t * DT:.2f}", teacher["phases"][t], f"{obs['teacher_left_gripper'][t]:.6f}", f"{obs['predicted_left_gripper_k0'][t]:.6f}",
               f"{obs['left_arm_k0_max_error_rad'][t]:.6f}", f"{obs['teacher_left_lead_max_rad'][t]:.6f}", f"{obs['right_arm_k0_max_error_rad'][t]:.6f}",
               f"{loop['teacher_left_tcp_block_m'][t]:.6f}", f"{loop['teacher_left_tcp_height_m'][t]:.6f}",
               f"{loop['student_left_tcp_block_m'][t]:.6f}" if own else "", f"{loop['student_left_tcp_height_m'][t]:.6f}" if own else "",
               f"{loop['left_tcp_separation_m'][t]:.6f}" if own else "", f"{loop['left_arm_joint_separation_rad'][t]:.6f}" if own else ""]
        lines.append(",".join(str(x) for x in row))
    out_path.write_text("\n".join(lines) + "\n")
    print(f"[written] {out_path} ({len(lines) - 1} rows; student columns are empty after its last state)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("protocol", "analyze", "video", "trace"):
        p = sub.add_parser(name)
        p.add_argument("--run-id", required=True)
        if name == "analyze":
            p.add_argument("--output-dir", help="write the tables somewhere else (preview)")
    args = parser.parse_args()
    dict(protocol=protocol, analyze=analyze, video=video, trace=trace)[args.command](args)


if __name__ == "__main__":
    main()
