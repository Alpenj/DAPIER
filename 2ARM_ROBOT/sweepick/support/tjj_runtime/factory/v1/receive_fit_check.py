#!/usr/bin/env python3
"""RECEIVE R2: prediction error on the teacher's own observations versus closed-loop departure (local SIM only; no
training, no teacher run, no hardware). RECEIVE_A / RECEIVE_B, START and RELEASE_B are read-only.

  offline   deployment-mode predictions (eval, latent zero, wrist RGB + q_state only) on the stored observations of the two
            actual-entry corrections (5000: 668, 5008: 659); executed horizon k = 0..24, canonical arm rad and gripper
            opening fraction; loader path against the runner path on three fixed observations per correction
  closed    the stored R2 original-reset runs (5000 B failed, 5008 B succeeded; A for comparison) next to the teacher
            reference from the same entry step: executed commands per C25 boundary, both arms, grippers, block in the left hand
  run       TEACHER-STATE CONDITIONAL DIAGNOSTIC: the teacher's first right_approach state of 5000 and of 5008, RECEIVE_A and
            RECEIVE_B once each (4 suffixes), repaired gate, RELEASE_B, absolute SIM 40 s
  video     seed 5000: TEACHER REFERENCE and the two STUDENT CONDITIONAL rows aligned at the hand-over state; seed 5008 stored
            original-reset preview

After the student's world differs from the teacher's, the teacher command of the same time is a reference trajectory only.
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
import train_rollout_diagnostic as td   # noqa: E402
import live_release_check as lr   # noqa: E402
import receive_candidate as rc   # noqa: E402
import receive_boundary_repair as rb   # noqa: E402
import receive_entry_r2 as r2   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "receive-r2-fit-vs-rollout-20261005-01"
R2 = FACTORY_ROOT / "evaluation" / r2.RUN
R2_RUNS = R2 / "runs" / "165a177645af"
CORRECTIONS = {5000: "s5000_entry_0475", 5008: "s5008_entry_0375"}
MODELS = ("A", "B")
EXPECTED_SHA = dict(START="19368b679b4a97213698b449243ab8443b851ba6b25aecd713558b63c74f9dc7", RELEASE_B="5a001b39fe271251f3218eabb2781079c4e0c5d218169d6dca41c18917a3402c",
                    A="0e10aad8083879f10de4da2248f4e9f9230dbbf1226df91ffa73dbeb0cf7f0e6", B="b45d60d8fd762e5be420199547c1194114dc5ef963dbd674c5765945877e7774")
GROUPS = dict(left_arm=list(td.LEFT_ARM), right_arm=list(td.RIGHT_ARM), left_gripper=[td.LEFT_GRIPPER], right_gripper=[td.RIGHT_GRIPPER])
PHASES = ("transfer_approach", "right_approach", "right_align", "right_grasp")
K = rc.PREFIX                                               # the executed horizon: k = 0..24
ORDER = {5000: ("A", "B"), 5008: ("B", "A")}                # fixed before any run
LABEL = "TEACHER-STATE CONDITIONAL DIAGNOSTIC"
MOVING = 0.02                                               # analysis only: an arm group "moves" if the teacher asks > 0.02 rad over the 25 steps
SOURCES = r2.SOURCES + ("factory/v1/receive_fit_check.py",)


def load(path):
    return json.loads(Path(path).read_text())


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checkpoints():
    paths = dict(START=rc.MODELS["START"], RELEASE_B=rc.MODELS["B"], A=r2.receive_models()["A"], B=r2.receive_models()["B"])
    got = {k: sha(p / "model.safetensors") for k, p in paths.items()}
    if got != EXPECTED_SHA:
        raise RuntimeError(f"a checkpoint differs from the R2 evaluation identity: {got}")
    return paths


def correction(seed):
    """One actual-entry correction: arrays, the receiving view [0, end) as the repaired loader serves it, teacher phases."""
    entry = next(r for r in load(R2 / "teacher_collection.json")["results"] if r["id"] == CORRECTIONS[seed])
    path = Path(entry["path"])
    for file, digest in entry["files"].items():
        if sha(path / file) != digest:
            raise ValueError(f"correction file changed: {path / file}")
    with np.load(path / "episode.npz", allow_pickle=False) as a:
        data = dict(states=a["integration_state"].copy(), q=a["q_state"].astype(np.float64), cmd=a["q_cmd"].astype(np.float64))
    phases = [d.get("phase") for d in load(path / "diagnostics.json")]
    end = entry["receiving"]["receive"][1]
    e = dict(id=entry["id"], seed=seed, path=str(path), source_step=entry["source_step"], suffix_transitions=entry["transitions"], receive=[0, end], files=entry["files"])
    view = rc.segment_view(rc.stats(), R2, r2.correction_segments([e], 200000))
    return dict(entry=entry, path=path, offset=entry["source_step"], end=end, phases=phases, view=view, rows=load(path / "task_check_rows.json"), **data)


def load_model(name):
    from dapier_act_policy import ACTPolicy
    path = checkpoints()[name]
    policy = ACTPolicy.from_pretrained(path, local_files_only=True, strict=True).cuda().eval()
    policy.check_normalization(rc.stats())
    return policy


# ----------------------------------------------------------------------------------------------
# offline: the teacher's observations, deployment mode
# ----------------------------------------------------------------------------------------------

def predict(policy, view, batch_size=64):
    import torch
    from torch.utils.data import default_collate
    out = []
    for lo in range(0, len(view), batch_size):
        items = [view[i] for i in range(lo, min(lo + batch_size, len(view)))]
        batch = default_collate(items)
        batch = {k: v.cuda() for k, v in batch.items() if k.startswith("observation.")}          # wrist RGB x2, q_state, the measured-q anchor: nothing else
        with torch.inference_mode():
            out.append(policy.predict_canonical_chunk(batch).double().cpu().numpy())
    return np.concatenate(out)


def group_stats(pred, target, anchor, valid, cols):
    """Over the valid (observation, k <= 24) pairs of one phase and one channel group."""
    err = (pred - target)[..., cols]
    hold = (anchor[:, None, :] - target)[..., cols]                    # persistence: keep the measured pose
    m = valid[..., None] & np.ones_like(err, dtype=bool)
    per_k = [float(np.abs(err[:, k][valid[:, k]]).mean()) if valid[:, k].any() else None for k in range(K)]
    bias = [float(err[..., j][valid].mean()) for j in range(len(cols))]
    out = dict(valid_targets=int(valid.sum()), mae=float(np.abs(err[m]).mean()), persistence_mae=float(np.abs(hold[m]).mean()),
               mae_k0=per_k[0], mae_k12=per_k[12], mae_k24=per_k[24], bias_per_channel=[round(b, 5) for b in bias], max_abs_error=float(np.abs(err[m]).max()))
    full = valid[:, K - 1]                                             # observations whose whole executed horizon is inside the phase view
    want = (target[:, K - 1] - anchor)[:, cols][full]
    got = (pred[:, K - 1] - anchor)[:, cols][full]
    norm = np.linalg.norm(want, axis=1)
    moving = norm > MOVING if len(cols) > 1 else np.abs(want[:, 0]) > MOVING
    out.update(teacher_displacement_k24_mean=float(norm.mean()) if len(norm) else None, observations_moving=int(moving.sum()))
    if moving.any():
        ratio = (got[moving] * want[moving]).sum(1) / (want[moving] ** 2).sum(1)      # 1 = the commanded 25-step move the teacher asked for, 0 = none, < 0 = the other way
        out.update(progress_ratio_median=float(np.median(ratio)), progress_ratio_p10=float(np.percentile(ratio, 10)), progress_below_half=int((ratio < 0.5).sum()))
    return out


def runner_prediction(policy, ctx, state, rgb):
    """The executor's own path on one stored observation: Student.next_command (policy_observation, anchor, canonical chunk)."""
    import mujoco
    from sim_data_factory import core
    from sim_data_factory.student_eval import Student

    class Stored:
        current = {f"{s}_wrist_rgb": dict(rgb=rgb[s]) for s in ("left", "right")}

    m, d = ctx["model"], ctx["data"]
    mujoco.mj_setState(m, d, state, int(mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_forward(m, d)
    student = Student(policy, rc.stats(), Stored(), rc.PREFIX, None, "project")
    command, _ = student.next_command(dict(model=m, data=d, control_dt=td.DT))
    return student.chunks[0]["actions"].astype(np.float64), student.chunks[0]["anchor"], core.encode_state(m, d), np.asarray(command, dtype=np.float64), student.trace[-1]


def offline(args):
    import frozen_transfer_check as ft
    from sim_data_factory.reset_sampler import make_task_episode
    target = run_dir() / "offline.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    run_dir().mkdir(parents=True, exist_ok=True)
    checkpoints()
    st = rc.stats()
    data = dict(run_id=RUN, mode="eval, latent zero (predict_canonical_chunk), inputs = two wrist RGB + q_state (+ the same q as the measured-relative anchor)",
                units="arm rad, gripper opening fraction; executed horizon k = 0..24", moving_threshold_rad=MOVING, corrections={}, tables={}, correspondence={})
    preds, fixed = {}, {}
    for seed in CORRECTIONS:
        c = correction(seed)
        view, n = c["view"], c["end"]
        key = next(iter(view.episodes))
        ep = view.episodes[key]
        assert len(view) == n and np.array_equal(ep["action"], c["cmd"][:n].astype(np.float32)) and np.array_equal(ep["state"], c["q"][:n].astype(np.float32))
        steps = np.arange(n)[:, None] + np.arange(K)[None]
        target_c = c["cmd"][np.minimum(steps, n - 1)]                  # q_cmd[t + k] of the same demonstration, canonical
        valid_all = steps < n
        phase = np.array(c["phases"][:n])
        first = {p: int(np.argmax(phase == p)) for p in PHASES if (phase == p).any()}
        fixed[seed] = dict(start=0, first_right_approach=first["right_approach"], first_right_grasp=first["right_grasp"])
        data["corrections"][str(seed)] = dict(id=c["entry"]["id"], observations=n, phase_start_step=c["offset"], valid_targets_k0_24=int(valid_all.sum()),
                                              phases={p: dict(observations=int((phase == p).sum()), first_local=first.get(p), first_abs_s=None if p not in first else round((c["offset"] + first[p]) * td.DT, 2))
                                                      for p in PHASES}, fixed_observations=fixed[seed])
        preds[seed] = dict(c=c, target=target_c, valid=valid_all, phase=phase, anchor=c["q"][:n])
    ctxs = {}
    for name in MODELS:
        policy = load_model(name)
        for seed, p in preds.items():
            c = p["c"]
            pred = predict(policy, c["view"])[:, :K]
            table = {}
            for ph in ("ALL",) + PHASES:
                sel = np.ones(len(pred), bool) if ph == "ALL" else p["phase"] == ph
                if not sel.any():
                    continue
                table[ph] = dict(observations=int(sel.sum()), valid_targets=int(p["valid"][sel].sum()),
                                 groups={g: group_stats(pred[sel], p["target"][sel], p["anchor"][sel], p["valid"][sel], cols) for g, cols in GROUPS.items()})
            data["tables"][f"{name} s{seed}"] = table
            np.savez_compressed(run_dir() / f"offline_pred_{name}_s{seed}.npz", pred_k0_24=pred.astype(np.float32), target_k0_24=p["target"].astype(np.float32), valid=p["valid"],
                                anchor=p["anchor"].astype(np.float32), phase=p["phase"])
            # loader path against the runner path on the three fixed observations
            if seed not in ctxs:
                meta = ft.demonstration(seed)["metadata"]
                ctxs[seed] = make_task_episode(seed, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
            key = next(iter(c["view"].episodes))
            ep = c["view"].episodes[key]
            checks = {}
            for label, t in fixed[seed].items():
                item = c["view"][t]
                rgb = {s: ep["rgb"][s][t] for s in ("left", "right")}
                chunk, anchor, q_live, command, trace = runner_prediction(policy, ctxs[seed], c["states"][t], rgb)
                arm = list(rc.ARM)
                back = item["action"].numpy().astype(np.float64) * st["action"]["std"] + st["action"]["mean"]       # loader target -> canonical
                back[:, arm] = item["action"].numpy().astype(np.float64)[:, arm] * st["action"]["std"][arm] + ep["state"][t][arm]
                pad = item["action_is_pad"].numpy()
                steps = np.minimum(t + np.arange(50), c["end"] - 1)
                checks[label] = dict(local_index=t, abs_s=round((c["offset"] + t) * td.DT, 2),
                                     runner_q_equals_stored_q=bool(np.array_equal(q_live.astype(np.float32), ep["state"][t])),
                                     runner_anchor_equals_loader_anchor=bool(np.array_equal(anchor, item["observation.measured_q"].numpy())),
                                     runner_normalized_state_max_abs_diff=float(np.abs(np.array(trace["normalized_state"]) - item["observation.state"].numpy()).max()),
                                     chunk_runner_vs_loader_max_abs_diff_k0_24=float(np.abs(chunk[:K] - pred[t]).max()),
                                     chunk_runner_vs_loader_max_abs_diff_all_k=float(np.abs(chunk - predict(policy, [c["view"][t]])[0]).max()),
                                     executed_command_equals_chunk_k0=bool(np.array_equal(command, chunk[0])), projected_at_k0=bool(any(trace["projection_mask"])),
                                     loader_target_back_to_canonical_max_abs_diff=float(np.abs(back - c["cmd"][steps])[~pad].max()), padded_targets=int(pad.sum()),
                                     prediction_error_k0_24=dict(left_arm=float(np.abs(chunk[:K] - preds[seed]["target"][t])[preds[seed]["valid"][t]][:, GROUPS["left_arm"]].max()),
                                                                 right_arm=float(np.abs(chunk[:K] - preds[seed]["target"][t])[preds[seed]["valid"][t]][:, GROUPS["right_arm"]].max())))
            data["correspondence"][f"{name} s{seed}"] = checks
        del policy
    target.write_text(json.dumps(data, indent=2))
    for k, table in data["tables"].items():
        for ph, t in table.items():
            g = t["groups"]
            print(k, ph, t["observations"], t["valid_targets"], {n: (round(v["mae"], 4), round(v["persistence_mae"], 4), None if "progress_ratio_median" not in v else round(v["progress_ratio_median"], 2)) for n, v in g.items()})
    print(json.dumps(data["correspondence"], indent=0)[:2500])
    print(f"[written] {target}")


# ----------------------------------------------------------------------------------------------
# closed: the stored original-reset runs next to the teacher reference
# ----------------------------------------------------------------------------------------------

def closed(args):
    import mujoco
    from integration_scenes import task_env
    from sim_data_factory.task_success import ContactModel
    target = run_dir() / "closed_loop.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    run_dir().mkdir(parents=True, exist_ok=True)
    m = task_env("desk", grippers="both").model
    d, cm = mujoco.MjData(m), ContactModel(m)
    spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    site = {s: m.site(f"{s}_cube_grasp") for s in ("left", "right")}
    site_def = {s: dict(site=site[s].name, body=m.body(int(site[s].bodyid[0])).name, pos_in_body=np.round(site[s].pos, 5).tolist(),
                        note="a fixed point of this body; it does not move with the jaw joint unless the body is the jaw") for s in site}

    def series(states, rows, lo, hi, stride=25):
        out, ref = [], None
        for k in range(lo, hi, stride):
            mujoco.mj_setState(m, d, states[k], spec)
            mujoco.mj_forward(m, d)
            f = lr.frames(m, d, cm)
            s = site["left"].id
            rot = d.site_xmat[s].reshape(3, 3).T @ d.xmat[m.body("red_block").id].reshape(3, 3)
            pos = np.array(f["block_in_left_site_mm"])
            if ref is None:
                ref = (pos, rot)
            ang = float(np.degrees(np.arccos(np.clip((np.trace(ref[1].T @ rot) - 1) / 2, -1, 1))))
            q = np.array(f["q_state"])
            out.append(dict(step=k, t=round(float(states[k][0]), 2), block_in_left_mm=pos.round(2).tolist(), block_shift_mm=round(float(np.linalg.norm(pos - ref[0])), 2), block_rot_deg=round(ang, 2),
                            left_pad_N=[round(x, 2) for x in f["pad_force_N"]["left"]], right_pad_N=[round(x, 2) for x in f["pad_force_N"]["right"]],
                            right_block_distance_mm=round(1000 * rows[k]["gripper_block_distance_m"]["right"], 1), left_gripper=round(float(q[td.LEFT_GRIPPER]), 3),
                            right_gripper=round(float(q[td.RIGHT_GRIPPER]), 3), q=q.round(4).tolist()))
        return out

    def onset(s, key, level):
        return next((x["t"] for x in s if x[key] > level), None)

    data = dict(run_id=RUN, site_definition=site_def, time_alignment="student absolute step = teacher phase_start_step + teacher local index; both clocks are the episode's own sim time",
                note="teacher rows are a reference trajectory: once the student's state differs, the teacher command of the same time is not the student's only correct action", runs={})
    for seed in CORRECTIONS:
        c = correction(seed)
        E = c["offset"]
        t_series = series(c["states"], c["rows"], 0, len(c["rows"]))
        for name in MODELS:
            p = R2_RUNS / "full" / f"s{seed}" / name
            result = load(p / "result.json")
            if result["identity_sha256"][:12] != "165a177645af" or result["switches"][0]["step"] != E:
                raise RuntimeError(f"{p}: not the stored R2 run entering RECEIVE at step {E}")
            with np.load(p / "episode.npz", allow_pickle=False) as a:
                states, q, cmd = a["integration_state"].copy(), a["q_state"].astype(np.float64), a["q_cmd"].astype(np.float64)
            rows = load(p / "task_check_rows.json")
            trace = load(p / "action_trace.json")
            with np.load(p / "predicted_chunks.npz", allow_pickle=False) as z:
                chunk_time, chunks = z["time"], z["actions"].astype(np.float64)
            gate = result["gate_s"]
            last = len(cmd) if gate is None else int(round(gate / td.DT))
            entry_diff = np.abs(q[E] - c["q"][0])
            boundaries = []
            for i, b in enumerate(range(E, min(last, E + c["end"]) - K + 1, K)):
                if trace[b]["policy"] != "RECEIVE" or trace[b]["selected_chunk_index"] != 0:
                    break
                j = int(np.argmin(np.abs(chunk_time - b * td.DT)))
                raw = chunks[j][:K]                                             # the chunk this boundary generated; only k = 0..24 was consumed
                applied = cmd[b:b + K]
                teach = c["cmd"][b - E:b - E + K]
                dq = q[b] - c["q"][b - E]
                row = dict(boundary=i, step=b, t=round(b * td.DT, 2), teacher_phase=c["phases"][b - E], chunk_matches_stored_raw=bool(np.allclose(raw[0], trace[b]["q_raw"], atol=1e-6)),
                           state_minus_teacher=dict(left_arm=round(float(np.abs(dq[GROUPS["left_arm"]]).max()), 4), right_arm=round(float(np.abs(dq[GROUPS["right_arm"]]).max()), 4),
                                                    left_gripper=round(float(dq[td.LEFT_GRIPPER]), 3), right_gripper=round(float(dq[td.RIGHT_GRIPPER]), 3)))
                for g, cols in GROUPS.items():
                    want = teach[-1][cols] - c["q"][b - E][cols]
                    got = applied[-1][cols] - q[b][cols]
                    row[g] = dict(command_minus_teacher_max=round(float(np.abs(applied - teach)[:, cols].max()), 4), student_move=round(float(np.linalg.norm(got)), 4),
                                  teacher_move=round(float(np.linalg.norm(want)), 4),
                                  progress_ratio=None if np.linalg.norm(want) <= MOVING else round(float(got @ want / (want @ want)), 2),
                                  student_cmd_end=[round(float(x), 3) for x in applied[-1][cols]] if len(cols) == 1 else None,
                                  teacher_cmd_end=[round(float(x), 3) for x in teach[-1][cols]] if len(cols) == 1 else None)
                boundaries.append(row)
            s_series = series(states, rows, E, len(rows))
            first_cmd = next((r for r in boundaries if any(r[g]["command_minus_teacher_max"] > 0.05 for g in ("left_arm", "right_arm")) or
                              any(abs(r[g]["command_minus_teacher_max"]) > 0.1 for g in ("left_gripper", "right_gripper"))), None)
            stall = next((r for r in boundaries if r["right_arm"]["progress_ratio"] is not None and r["right_arm"]["progress_ratio"] < 0.5), None)
            l_stall = next((r for r in boundaries if r["left_arm"]["progress_ratio"] is not None and r["left_arm"]["progress_ratio"] < 0.5), None)
            data["runs"][f"s{seed} {name}"] = dict(
                path=str(p), task_success=result["task_success"], semantic_ready_s=result["phases"]["semantic_ready_s"], gate_s=gate, order=result["phases"]["order"],
                entry_state_minus_teacher_start=dict(arm_max_rad=round(float(entry_diff[list(rc.ARM)].max()), 5), grippers=[round(float(entry_diff[td.LEFT_GRIPPER]), 4), round(float(entry_diff[td.RIGHT_GRIPPER]), 4)],
                                                     identical=bool(np.array_equal(states[E], c["states"][0]))),
                boundaries_compared=len(boundaries),
                first_boundary_command_departs=None if first_cmd is None else {k: first_cmd[k] for k in ("boundary", "t", "teacher_phase", "state_minus_teacher", "left_arm", "right_arm", "left_gripper", "right_gripper")},
                first_boundary_right_arm_progress_below_half=None if stall is None else {k: stall[k] for k in ("boundary", "t", "teacher_phase", "state_minus_teacher", "right_arm")},
                first_boundary_left_arm_progress_below_half=None if l_stall is None else {k: l_stall[k] for k in ("boundary", "t", "teacher_phase", "state_minus_teacher", "left_arm")},
                block_in_left_hand=dict(student_shift_gt_5mm_s=onset(s_series, "block_shift_mm", 5), student_shift_gt_15mm_s=onset(s_series, "block_shift_mm", 15),
                                        student_rot_gt_10deg_s=onset(s_series, "block_rot_deg", 10), student_rot_gt_30deg_s=onset(s_series, "block_rot_deg", 30),
                                        teacher_shift_gt_5mm_s=onset(t_series, "block_shift_mm", 5), teacher_rot_gt_10deg_s=onset(t_series, "block_rot_deg", 10),
                                        student_max=[max(x["block_shift_mm"] for x in s_series), max(x["block_rot_deg"] for x in s_series)],
                                        teacher_max_until_readiness=[max(x["block_shift_mm"] for x in t_series if x["step"] <= c["end"]), max(x["block_rot_deg"] for x in t_series if x["step"] <= c["end"])],
                                        right_pad_contact_before_student_shift=bool(any(max(x["right_pad_N"]) > 0 for x in s_series if x["t"] <= (onset(s_series, "block_shift_mm", 5) or 0)))),
                boundaries=boundaries, student_series=[{k: v for k, v in x.items() if k != "q"} for x in s_series],
                right_arm_q_series=[[x["t"]] + [x["q"][j] for j in GROUPS["right_arm"]] for x in s_series], left_arm_q_series=[[x["t"]] + [x["q"][j] for j in GROUPS["left_arm"]] for x in s_series])
        data.setdefault("teacher", {})[f"s{seed}"] = dict(phase_start_step=E, readiness_local=c["end"], series=[{k: v for k, v in x.items() if k != "q"} for x in t_series],
                                                           right_arm_q_series=[[x["t"]] + [x["q"][j] for j in GROUPS["right_arm"]] for x in t_series],
                                                           left_arm_q_series=[[x["t"]] + [x["q"][j] for j in GROUPS["left_arm"]] for x in t_series])
    target.write_text(json.dumps(data, indent=2))
    print(json.dumps(site_def))
    for k, v in data["runs"].items():
        print("\n", k, "success", v["task_success"], "entry", v["entry_state_minus_teacher_start"], "boundaries", v["boundaries_compared"])
        print("  first command departure", json.dumps(v["first_boundary_command_departs"]))
        print("  first right-arm stall", json.dumps(v["first_boundary_right_arm_progress_below_half"]))
        print("  first left-arm stall", json.dumps(v["first_boundary_left_arm_progress_below_half"]))
        print("  block in left hand", json.dumps(v["block_in_left_hand"]))
    print(f"[written] {target}")


# ----------------------------------------------------------------------------------------------
# run: four suffixes from the teacher's first right_approach state
# ----------------------------------------------------------------------------------------------

def selection():
    """The teacher's first right_approach state of each correction, with the whole physical history before it."""
    out = {}
    for seed in CORRECTIONS:
        c = correction(seed)
        local = next(i for i, p in enumerate(c["phases"]) if p == "right_approach")
        g = rc.stored_state(r2.entry_case(seed), c["offset"])            # the student prefix up to the entry state
        states = np.concatenate([g["states"], c["states"][1:local + 1]])
        q = np.concatenate([g["q"], c["q"][1:local + 1]])
        rgb = {s: np.load(c["path"] / f"cameras/{s}_wrist_rgb.npz")["rgb"][local].copy() for s in ("left", "right")}
        out[seed] = dict(local=local, step=c["offset"] + local, states=states, q=q, rgb=rgb,
                         record=dict(seed=seed, correction=c["entry"]["id"], teacher_phase="right_approach (first state)", local_index=local, phase_start_step=c["offset"],
                                     abs_step=c["offset"] + local, sim_time_s=round(float(c["states"][local][0]), 2), state_sha256=hashlib.sha256(np.ascontiguousarray(c["states"][local]).tobytes()).hexdigest(),
                                     rgb_sha256={s: hashlib.sha256(rgb[s].tobytes()).hexdigest() for s in rgb}, history_states=len(states),
                                     prefix="student START up to the entry state, then the teacher's transfer_approach; none of it is a student result of this diagnostic"))
    return out


def identity(sel):
    ck = checkpoints()
    body = dict(run_id=RUN, models={k: {f: sha(p / f) for f in ("model.safetensors", "config.json")} | dict(normalization=sha(p.parent / "normalization.npz")) for k, p in ck.items()},
                states={str(s): v["record"]["state_sha256"] for s, v in sel.items()}, rgb={str(s): v["record"]["rgb_sha256"] for s, v in sel.items()},
                sources={p: sha(FACTORY_ROOT / p) for p in SOURCES}, contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                execution=dict(prefix=rc.PREFIX, arm_bound_policy="project", ensemble="off", horizon_steps=rc.HORIZON, torch_seed=4100))
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def case_dir(ident, seed, model):
    return run_dir() / "runs" / ident["sha256"][:12] / f"s{seed}" / model


def complete(out, ident, case):
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (r.get("identity_sha256") == ident["sha256"] and r.get("case") == list(case) and meta.get("complete") and r.get("boundary_contract") == "repaired"
          and all((out / f).exists() for f in ("episode.npz", "action_trace.json", "task_check_rows.json")))
    return r if ok else None


def one(args):
    sel = selection()
    ident = identity(sel)
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    out = case_dir(ident, args.seed, args.model)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    s = sel[args.seed]
    rc.MODELS["RECEIVE"] = checkpoints()[args.model]
    result = rc.execute(out, args.seed, ("RECEIVE", "B"), dict(states=s["states"], q=s["q"], rgb=s["rgb"], step=s["step"]),
                        f"{LABEL}: s{args.seed} teacher right_approach state, RECEIVE_{args.model}", contract="repaired")
    rows = load(out / "task_check_rows.json")
    t0 = s["step"] * td.DT
    result.update(kind="teacher-state", classification=LABEL + " (" + rc.LABEL + " switches)", case=[args.seed, args.model], receive_model=args.model, identity_sha256=ident["sha256"],
                  chain=f"RECEIVE_{args.model} -> RELEASE_B", handover_event=result["handover_s"] is not None and result["handover_s"] > t0,
                  events_before_restore={k: v for k, v in result["events"].items() if v <= t0}, new_events={k: v for k, v in result["events"].items() if v > t0},
                  contract_checks=rb.contract_checks(result), phases=r2.phase_details(result, rows, t0), start_state=s["record"])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(seed=args.seed, model=args.model, order=result["phases"]["order"], termination=result["termination"], run_valid=result["run_valid"],
                          handover_event=result["handover_event"], task_success=result["task_success"], final_support=result["final_support"]), default=str))


def table_row(r):
    q = r["receiving"] or {}
    fault = r["first_fault"]
    return dict(seed=r["seed"], receive=r["receive_model"], start_s=r["start_state"]["sim_time_s"], semantic_ready_s=r["phases"]["semantic_ready_s"], release_b_switch_s=r["gate_s"],
                left_release_s=r["left_release_while_right_holds_first_s"], left_retreat_s=r["left_clear_by_donor_clearance_first_s"],
                handover_event_s=r["handover_s"] if r["handover_event"] else None, first_fault_s=None if fault is None else round(fault["time"], 2), fault_stage=r["fault_stage"],
                fault_pairs=r["phases"]["fault_pairs"], termination=r["termination"], run_valid=r["run_valid"], final_support=r["final_support"],
                receive_ready=r["phases"]["semantic_ready_s"] is not None, conditional_success=r["task_success"], order=r["phases"]["order"],
                projection={k: dict(steps=v["projected_steps"], of=v["dispatched"], longest_s=v["longest_s"], max_raw_excess_rad=round(v["max_raw_excess_rad"], 4), per_axis=v["per_axis"])
                            for k, v in r["projection"].items()},
                right_before_two_pad_hold=q.get("right_before_two_pad_hold"), block_in_left_site=q.get("block_in_left_site"), switches=r["contract_checks"]["switches"])


def run(args):
    for name in ("offline.json", "closed_loop.json"):
        if not (run_dir() / name).exists():
            raise FileNotFoundError(f"{name} is missing: run `offline` and `closed` first")
    sel = selection()
    ident = identity(sel)
    jobs = [(s, mdl) for s in CORRECTIONS for mdl in ORDER[s]]
    (run_dir() / "logs").mkdir(exist_ok=True)
    proto = run_dir() / "selection.json"
    if not proto.exists():
        proto.write_text(json.dumps(dict(run_id=RUN, label=LABEL, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"), states=[v["record"] for v in sel.values()], identity=ident,
                                         cases=[list(j) for j in jobs], budget="four suffixes, one each, whatever the results; no other state, no repeat",
                                         rule="the first state the teacher labelled right_approach in each stored correction; a time is never a runtime condition",
                                         training=False, teacher_run=False, val_test=False, hardware_access=False), indent=2))
    status = []
    for seed, mdl in jobs:
        out, case = case_dir(ident, seed, mdl), (seed, mdl)
        if complete(out, ident, case) is not None:
            status.append(dict(case=list(case), status="completed (reused: same identity, complete files)"))
            print(f"[reused under identity {ident['sha256'][:12]}] {seed} RECEIVE_{mdl}")
            continue
        if out.exists():
            status.append(dict(case=list(case), status="NOT RUN: an incomplete folder of this identity exists; neither reused nor overwritten", path=str(out)))
            continue
        done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--seed", str(seed), "--model", mdl, "--identity", ident["sha256"]], capture_output=True, text=True)
        (run_dir() / "logs" / f"{ident['sha256'][:12]}_s{seed}_{mdl}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        ok = done.returncode == 0 and complete(out, ident, case) is not None
        status.append(dict(case=list(case), returncode=done.returncode, status="completed" if ok else "EXECUTION ERROR (not a task result)", **({} if ok else dict(stderr_tail=done.stderr[-600:]))))
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(line[-1][:900] if ok and line else f"[EXECUTION ERROR, not a task failure] {seed} RECEIVE_{mdl}: returncode {done.returncode}: {done.stderr[-300:]}", flush=True)
    results = [r for seed, mdl in jobs if (r := complete(case_dir(ident, seed, mdl), ident, (seed, mdl))) is not None]
    (run_dir() / "results.json").write_text(json.dumps(dict(
        run_id=RUN, note=f"{LABEL}: the prefix contains the student's START segment and the teacher's transfer; not an original-reset student result; switches read SIM ground truth; one run per case",
        identity_sha256=ident["sha256"], planned_cases=len(jobs), completed_cases=len(results), execution_status=status, table=[table_row(r) for r in results], results=results), indent=2, default=str))
    for row in (table_row(r) for r in results):
        print(json.dumps({k: row[k] for k in ("seed", "receive", "start_s", "semantic_ready_s", "release_b_switch_s", "handover_event_s", "first_fault_s", "fault_pairs", "final_support",
                                              "receive_ready", "conditional_success")}, default=str))
    print(f"[written] {run_dir() / 'results.json'}: {len(results)} of {len(jobs)} cases completed, {sum(not s['status'].startswith('completed') for s in status)} execution error(s) / not run; "
          f"identity {ident['sha256'][:12]}")


def video(args):
    import recovery_trial as rt
    sel = selection()
    ident = identity(sel)
    out = run_dir() / "videos" / f"seed5000_teacher_reference_vs_student_conditional_{ident['sha256'][:12]}.mp4"
    if not out.exists():
        c = correction(5000)
        local = sel[5000]["local"]
        ref = rt.teacher_row("TEACHER REFERENCE (stored correction, from its first right_approach state)", c["path"])
        ref.update(states=ref["states"][local:], commands=ref["commands"][local:], contacts=None if ref["contacts"] is None else ref["contacts"][local:],
                   wrist={s: v[local:] for s, v in ref["wrist"].items()}, start=float(c["states"][local][0]),
                   events={k: v for k, v in c["entry"]["events"].items() if v >= float(c["states"][local][0])})
        rows = [ref]
        for mdl in MODELS:
            r = complete(case_dir(ident, 5000, mdl), ident, (5000, mdl))
            if r is None:
                print(f"[unavailable] seed 5000 RECEIVE_{mdl}: no completed case under this identity; the row is not replaced")
                continue
            row = rt.teacher_row(f"STUDENT CONDITIONAL: RECEIVE_{mdl} -> RELEASE_B (same start state)", Path(r["path"]))
            ev = dict(r["new_events"])
            ev.update({f"SWITCH->{s['to']}": s["sim_time_s"] for s in r["switches"]})
            row["events"] = dict(sorted(ev.items(), key=lambda kv: kv[1]))
            row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}" + ("" if r["gate_reached"] else " (RELEASE_B NOT REACHED)"),
                               "valid handover in this conditional run" if r["task_success"] else f"no valid handover; final support: {r['final_support']}")
            rows.append(row)
        rt.render(rows, out, f"{LABEL} / seed 5000 (time from the hand-over state)", 2000, False, right_gripper=True)
        print(f"[written] {out}")
    else:
        print(f"[exists] {out}")
    preview = run_dir() / "videos" / "seed5008_stored_R2_original_reset_RECEIVE_B_preview.mp4"
    if not preview.exists():
        p = R2_RUNS / "full/s5008/B"
        r = load(p / "result.json")
        row = rt.teacher_row(f"STORED R2 RUN (no new rollout): {r['chain']} original reset", p)
        ev = dict(r["events"])
        ev.update({f"SWITCH->{s['to']}": s["sim_time_s"] for s in r["switches"]})
        row["events"] = dict(sorted(ev.items(), key=lambda kv: kv[1]))
        row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}", "VALID HANDOVER, right hand holds" if r["task_success"] else "no valid handover")
        rt.render([row], preview, "STORED R2 seed 5008 original reset (SIM-ORACLE-GATED THREE-POLICY CANDIDATE)", 1500, False, right_gripper=True)
        print(f"[written] {preview}")
    else:
        print(f"[exists] {preview}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("offline", "closed", "run", "video"):
        sub.add_parser(name)
    o = sub.add_parser("one")
    o.add_argument("--seed", type=int, choices=tuple(CORRECTIONS), required=True)
    o.add_argument("--model", choices=MODELS, required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    dict(offline=offline, closed=closed, run=run, one=one, video=video)[args.command](args)


if __name__ == "__main__":
    main()
