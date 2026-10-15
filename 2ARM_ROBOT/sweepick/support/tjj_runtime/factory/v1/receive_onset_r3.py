#!/usr/bin/env python3
"""RECEIVE R3: one teacher correction from the student's own stalled transfer-onset state, and a replay-matched comparison
of two copies of RECEIVE_R2_B (local SIM / learning only; no hardware). START, RELEASE_B, RECEIVE_R2_B / R2_A / R1 are
read-only. No new policy, gate, timer, input or controller change.

  teacher   the existing re-entered HandoverTeacher, once, from the stored state of R2 full/s5000/B at step 650
  protocol  RECEIVE_R2_B's own 5,000 x 8 plan as the replay; E replaces every second of the 3,333 seed-5000 entry-correction
            positions (1,667) by the new correction: 834 from its first min(50, N) observations, 833 from all of it
  train     --condition c | e : RECEIVE_R2_B weights + its AdamW state, 5,000 updates each
              c = RECEIVE_REPLAY_CONTROL   e = RECEIVE_ONSET_RECOVERY
  fixed     deployment-mode predictions of R2_B / C / E on stored teacher observations (no physics)
  run       original reset: 5000 and 5008 twice per condition, 5002 once (10); the two teacher right_approach states once
            per condition (4, conditional); START -> {C | E} -> RELEASE_B under the repaired contract
  summary   this run's own tables (used by `act-sim results`)
  video     seed 5000 C / E and seed 5008 C / E, original reset, first run of each

Switches read SIM ground truth: SIM-ORACLE-GATED THREE-POLICY CANDIDATE. Teacher, conditional and original-reset results
are never added together.
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
import receive_candidate as rc   # noqa: E402
import receive_boundary_repair as rb   # noqa: E402
import receive_entry_r2 as r2   # noqa: E402
import receive_fit_check as fc   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "receive-onset-r3-20261005-01"
R2B_TRAINING = r2.training_dir("b")
R2B = R2B_TRAINING / r2.CHECKPOINT
R2B_SHA = "b45d60d8fd762e5be420199547c1194114dc5ef963dbd674c5765945877e7774"
SOURCE_CASE = fc.R2_RUNS / "full" / "s5000" / "B"
SOURCE_STEP = 650                                     # stored C25 boundary at 13.0 s; selects material only, never a runtime condition
NAMES = dict(c="RECEIVE_REPLAY_CONTROL", e="RECEIVE_ONSET_RECOVERY")
CHECKPOINT = "step-005000"
UPDATES, BATCH, SEED = 5000, 8, 4100
REPLACED, FROM_START, START_WINDOW = 1667, 834, 50
FULL_ORDER = {5000: ("C", "E", "E", "C"), 5008: ("E", "C", "C", "E"), 5002: ("E", "C")}      # fixed before any run
COND_ORDER = {5000: ("C", "E"), 5008: ("E", "C")}
LABEL = rc.LABEL
SOURCES = fc.SOURCES + ("factory/v1/receive_onset_r3.py",)
DEPART = (0.2, 1.0)                                   # analysis markers only: left-arm joint change from the RECEIVE entry state (rad)


def load(path):
    return json.loads(Path(path).read_text())


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def training_dir(condition):
    return FACTORY_ROOT / "training" / f"{RUN}-{condition}"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def models():
    return dict(R2B=R2B, C=training_dir("c") / CHECKPOINT, E=training_dir("e") / CHECKPOINT)


def frozen():
    out = dict(START=sha(rc.MODELS["START"] / "model.safetensors"), RELEASE_B=sha(rc.MODELS["B"] / "model.safetensors"), R2B=sha(R2B / "model.safetensors"),
               R2A=sha(r2.receive_models()["A"] / "model.safetensors"), R1=sha(r2.receive_models()["R1"] / "model.safetensors"))
    want = dict(START=fc.EXPECTED_SHA["START"], RELEASE_B=fc.EXPECTED_SHA["RELEASE_B"], R2B=R2B_SHA, R2A=fc.EXPECTED_SHA["A"], R1=r2.EXPECTED_SHA["R1"])
    if out != want:
        raise RuntimeError(f"a read-only checkpoint differs from its recorded SHA-256: {out}")
    return out


# ----------------------------------------------------------------------------------------------
# Teacher: one run from the stalled state
# ----------------------------------------------------------------------------------------------

def teacher(args):
    import mujoco
    import handover_correction_trial as hc
    import paired_release_check as pr
    from integration_scenes import task_env
    from sim_data_factory import student_eval
    from sim_data_factory.task_success import ContactModel, evaluate_states
    import live_release_check as lr
    target = run_dir() / "teacher_collection.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    run_dir().mkdir(parents=True, exist_ok=True)
    frozen()
    result = load(SOURCE_CASE / "result.json")
    trace = load(SOURCE_CASE / "action_trace.json")
    if result["identity_sha256"][:12] != "165a177645af" or trace[SOURCE_STEP]["policy"] != "RECEIVE" or trace[SOURCE_STEP]["selected_chunk_index"] != 0:
        raise RuntimeError("the stored source is not R2 full/s5000/B with a RECEIVE C25 inference at step 650")
    g = rc.stored_state(SOURCE_CASE, SOURCE_STEP)
    if abs(float(g["states"][-1][0]) - SOURCE_STEP * td.DT) > 1e-6:
        raise RuntimeError("stored state time does not match step 650")
    selection = dict(run_id=RUN, fixed_before_teacher=time.strftime("%Y-%m-%dT%H:%M:%S%z"), seed=5000, source_case=str(SOURCE_CASE), control_step=SOURCE_STEP,
                     sim_time_s=float(g["states"][-1][0]), state_sha256=hc.state_sha(g["states"][-1]), q_state=g["q"][-1].tolist(),
                     rgb_sha256={s: hashlib.sha256(g["rgb"][s].tobytes()).hexdigest() for s in ("left", "right")}, prefix_states=len(g["states"]),
                     prefix="START to 9.5 s, then RECEIVE_R2_B to 13.0 s; history check only, none of its actions becomes a label",
                     files={f: sha(SOURCE_CASE / f) for f in ("episode.npz", "cameras/left_wrist_rgb.npz", "cameras/right_wrist_rgb.npz", "result.json")},
                     rule="one state, one teacher run, no retry, no other time; the whole stored integration state is restored (block, joints, velocities, both grippers)")
    (run_dir() / "selection.json").write_text(json.dumps(selection, indent=2))
    original, sensors = student_eval.Observations, []

    def first_stored(*a, **k):
        student_eval.Observations = original
        sensors.append(pr.stored_first_sensor(dict(rgb=g["rgb"])))
        return sensors[-1]

    student_eval.Observations = first_stored
    try:
        entry = hc.continuation(run_dir() / "corrections" / "s5000_onset_0650", 5000, g["states"], g["q"], g["states"][-1],
                                dict(classification="teacher_recovery_from_student_stalled_transfer_onset_state", source_student_case=str(SOURCE_CASE), source_step=SOURCE_STEP,
                                     source_state_sha256=selection["state_sha256"], note="a child of the Train parent, not a new independent initial condition"))
    finally:
        student_eval.Observations = original
    entry.update(id="s5000_onset_0650", seed=5000, source_step=SOURCE_STEP, full_teacher_handover_admitted=bool(entry.get("admitted")))
    receive = dict(valid=False, failure_class=None, reason=None)
    motion = None
    if not entry.get("path"):
        receive.update(failure_class="planning failure" if entry.get("teacher_failure") else "execution failure", reason=str(entry.get("teacher_failure") or entry.get("terminal_failure")))
    else:
        path = Path(entry["path"])
        rows = load(path / "task_check_rows.json")
        start, end, why = rb.semantic_bounds(rows, entry["events"], start_at_first_row=True)
        with np.load(path / "episode.npz", allow_pickle=False) as a:
            suffix, suffix_q, cmd = a["integration_state"].copy(), a["q_state"].astype(np.float64), a["q_cmd"].astype(np.float64)
        phases = [d.get("phase") for d in load(path / "diagnostics.json")]
        spans = {}
        for i, p in enumerate(phases):
            spans.setdefault(p, [i, i])[1] = i
        dq = np.abs(suffix_q - suffix_q[0])
        first = lambda cols, level: next((round((SOURCE_STEP + i) * td.DT, 2) for i in range(len(dq)) if dq[i, cols].max() > level), None)
        m = task_env("desk", grippers="both").model
        d, cm = mujoco.MjData(m), ContactModel(m)
        frames = []
        for i in (0, end or len(suffix) - 1):
            mujoco.mj_setState(m, d, suffix[i], int(mujoco.mjtState.mjSTATE_INTEGRATION))
            mujoco.mj_forward(m, d)
            frames.append(lr.frames(m, d, cm))
        motion = dict(phase_spans_local=spans, left_arm_moved_0p05_rad_s=first(list(td.LEFT_ARM), 0.05), left_arm_moved_1_rad_s=first(list(td.LEFT_ARM), 1.0),
                      right_arm_moved_0p05_rad_s=first(list(td.RIGHT_ARM), 0.05), right_arm_first_25_step_command_move_rad=float(np.linalg.norm(cmd[24][list(td.RIGHT_ARM)] - suffix_q[0][list(td.RIGHT_ARM)])),
                      left_arm_first_25_step_command_move_rad=float(np.linalg.norm(cmd[24][list(td.LEFT_ARM)] - suffix_q[0][list(td.LEFT_ARM)])),
                      right_first_pad_touch_s=next((r["time"] for r in rows if max(r["pad_force_N"]["right"]) > 0), None),
                      block_in_left_site_mm=dict(start=frames[0]["block_in_left_site_mm"], at_readiness_or_end=frames[1]["block_in_left_site_mm"]))
        if why:
            fault = entry.get("teacher_failure") or entry.get("terminal_failure")
            receive.update(failure_class="execution failure" if fault else "not completed within the 40 s limit", reason=f"{why}; stop={entry['stop']}; {fault}")
        else:
            check = evaluate_states(m, np.concatenate([g["states"], suffix[1:end + 1]]), np.concatenate([g["q"], suffix_q[1:end + 1]]), control_dt=td.DT)
            seg = rows[:end + 1]
            receive.update(valid=bool(check["valid"]), receive=[0, end], receive_abs_s=[round(SOURCE_STEP * td.DT, 2), round(rows[end]["time"], 2)], prefix_checker_valid=bool(check["valid"]),
                           prefix_failures=check["failures"][:3], events_up_to_readiness={k: round(v, 2) for k, v in check["events"].items()},
                           right_pad_peak_before_readiness_N=round(max(max(r["pad_force_N"]["right"]) for r in seg), 2),
                           after_readiness=dict(stop=entry["stop"], teacher_failure=entry.get("teacher_failure"), terminal_failure=entry.get("terminal_failure"),
                                                whole_run_valid=entry.get("checker_valid"), handover=entry["events"].get("handover")))
            if not check["valid"]:
                receive.update(failure_class="execution failure", reason="the history up to readiness is not checker-valid")
    entry["receiving"], entry["motion"] = receive, motion
    target.write_text(json.dumps(dict(run_id=RUN, results=[entry], receiving_valid=[entry["id"]] if receive["valid"] else [],
                                      note="teacher readiness is neither a student success nor a handover success"), indent=2, default=str))
    print(json.dumps(dict(id=entry["id"], reentry=entry.get("reentry"), stop=entry.get("stop"), transitions=entry.get("transitions"), receiving=receive, motion=motion,
                          events={k: round(v, 2) for k, v in (entry.get("events") or {}).items()}), default=str)[:2500])
    print(f"[written] {target}")


def onset_entry():
    r = load(run_dir() / "teacher_collection.json")["results"][0]
    if not r["receiving"]["valid"]:
        return None
    for file, digest in r["files"].items():
        if sha(Path(r["path"]) / file) != digest:
            raise ValueError(f"correction file changed: {r['path']}/{file}")
    return dict(id=r["id"], seed=5000, path=r["path"], origin="stalled transfer-onset state, this run", source_step=r["source_step"], suffix_transitions=r["transitions"],
                receive=r["receiving"]["receive"], receive_abs_s=r["receiving"]["receive_abs_s"], files=r["files"])


# ----------------------------------------------------------------------------------------------
# Plan: R2_B's own plan as the replay; E replaces 1,667 positions
# ----------------------------------------------------------------------------------------------

def datasets():
    st, replay, train_view, old_view, new_view, _ = r2.datasets()
    record = load(r2.run_dir() / "protocol.json")
    got = dict(train=r2.view_sha(train_view), existing=r2.view_sha(old_view), actual_entry=r2.view_sha(new_view))
    want = dict(train=record["data"]["train_view"]["sha256"], existing=record["data"]["existing_corrections_sha256"], actual_entry=record["data"]["actual_entry_sha256"])
    if got != want:
        raise RuntimeError("the R2 views rebuilt now differ from the views RECEIVE_R2_B was trained on")
    e = onset_entry()
    onset_view = None if e is None else rc.segment_view(st, run_dir(), r2.correction_segments([e], 300000))
    return st, replay, train_view, old_view, new_view, onset_view


def build_plan(new_view, onset_view):
    """R2_B's plan unchanged, plus for E: which compared-slot positions take a sample of the onset correction, and which."""
    with np.load(r2.run_dir() / "sample_plan.npz", allow_pickle=False) as a:
        plan = {k: a[k].copy() for k in a.files}
    if sha(r2.run_dir() / "sample_plan.npz") != load(r2.run_dir() / "protocol.json")["sample_plan"]["sha256"]:
        raise RuntimeError("the R2 sample plan changed")
    n = len(onset_view)
    flat = np.flatnonzero((plan["cmp_parent"].ravel() == 5000) & (plan["b_from_new"].ravel() == 1))          # the 3,333 seed-5000 entry-correction positions, in order
    chosen = flat[::2]
    if len(flat) != 3333 or len(chosen) != REPLACED:
        raise RuntimeError(f"unexpected position counts: {len(flat)}, {len(chosen)}")
    rng = np.random.default_rng(SEED)
    early = rng.permutation(np.array([1] * FROM_START + [0] * (REPLACED - FROM_START)))
    u = rng.random(REPLACED)
    window = min(START_WINDOW, n)
    t = np.where(early == 1, np.floor(u * window), np.floor(u * n)).astype(np.int64)
    e_from = np.zeros(plan["cmp_parent"].size, dtype=np.int64)
    e_t = np.full(plan["cmp_parent"].size, -1, dtype=np.int64)
    e_early = np.zeros(plan["cmp_parent"].size, dtype=np.int64)
    e_from[chosen], e_t[chosen], e_early[chosen] = 1, t, early
    shape = plan["cmp_parent"].shape
    plan.update(e_from_onset=e_from.reshape(shape), e_t=e_t.reshape(shape), e_from_start_window=e_early.reshape(shape))
    return plan


def check_plan(plan, train_view, old_view, new_view, onset_view):
    key = next(iter(onset_view.episodes))
    n = len(onset_view)
    on = plan["e_from_onset"] == 1
    assert on.sum() == REPLACED and (plan["cmp_parent"][on] == 5000).all() and (plan["b_from_new"][on] == 1).all()
    kept = (plan["cmp_parent"] == 5000) & (plan["b_from_new"] == 1) & ~on
    assert kept.sum() == 3333 - REPLACED
    assert plan["e_from_start_window"][on].sum() == FROM_START and (plan["e_t"][on][plan["e_from_start_window"][on] == 1] < min(START_WINDOW, n)).all()
    assert (plan["e_t"][on] >= 0).all() and (plan["e_t"][on] < n).all() and (plan["e_t"][~on] == -1).all()
    assert not (set(onset_view.episodes) & (set(train_view.episodes) | set(old_view.episodes) | set(new_view.episodes)))
    ep, std, arm = onset_view.episodes[key], onset_view.stats["action"]["std"], list(rc.ARM)
    for t in (0, max(n - 60, 0), n - 1):
        item = onset_view[t]
        pad = item["action_is_pad"].numpy()
        assert pad.sum() == max(0, t + 50 - n) and (not pad.any() or pad[-1])                 # padding only past the semantic end
        j = min(3, n - 1 - t)
        assert np.allclose(item["action"][j].numpy()[arm], (ep["action"][t + j][arm] - ep["state"][t][arm]) / std[arm], atol=1e-6)   # this suffix's own q_meas[t] and future command
        assert int(item["meta_episode"]) == key and int(item["meta_t"]) == t
    return True


def protocol(args):
    import arm_output_trial as trial
    target = run_dir() / "protocol.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    fz = frozen()
    if onset_entry() is None:
        target.write_text(json.dumps(dict(run_id=RUN, status="NO_VALID_ONSET_CORRECTION: the C / E training is not started", teacher=load(run_dir() / "teacher_collection.json")["results"][0]["receiving"]), indent=2))
        raise SystemExit("no valid onset correction: the C / E training is not started")
    st, replay, train_view, old_view, new_view, onset_view = datasets()
    plan = build_plan(new_view, onset_view)
    check_plan(plan, train_view, old_view, new_view, onset_view)
    np.savez_compressed(run_dir() / "sample_plan.npz", **plan)
    r2b_run = load(R2B_TRAINING / "run.json")
    config = load(R2B / "config.json")
    e = onset_view.rows[0]
    record = dict(
        run_id=RUN, name=LABEL + " / RECEIVE R3 onset recovery, replay-matched C / E", fixed_before_training=time.strftime("%Y-%m-%dT%H:%M:%S%z"), hardware_access=False, val_test=False, frozen=fz,
        start=dict(checkpoint=str(R2B), model_sha256=R2B_SHA, optimizer_state=str(R2B / "training_state.pt"), optimizer_state_sha256=sha(R2B / "training_state.pt"),
                   optimizer_step_in_file=int(r2b_run["steps"]), use="both conditions: all RECEIVE_R2_B weights and its stored AdamW state, read-only; nothing re-initialised; torch.manual_seed(4100)"),
        config_read_from_checkpoint={k: config.get(k) for k in ("type", "chunk_size", "n_action_steps", "kl_weight", "optimizer_lr", "optimizer_lr_backbone", "optimizer_weight_decay", "use_vae")},
        conditions=dict(c=NAMES["c"], e=NAMES["e"]),
        data=dict(replay="RECEIVE_R2_B's own plan (5,000 x 8) on the same three views, re-verified by view SHA", r2_sample_plan_sha256=sha(r2.run_dir() / "sample_plan.npz"),
                  views=dict(train=r2.view_sha(train_view), existing=r2.view_sha(old_view), actual_entry=r2.view_sha(new_view), onset=r2.view_sha(onset_view)),
                  onset_correction={k: e[k] for k in ("id", "seed", "source_step", "receive", "receive_abs_s", "path")}, onset_observations=len(onset_view),
                  not_used=["any student prefix action", "teacher steps at or after the semantic readiness sample", "commands of another time or demonstration"]),
        sample_plan=dict(file=str(run_dir() / "sample_plan.npz"), sha256=sha(run_dir() / "sample_plan.npz"), updates=UPDATES, batch=BATCH, seed=SEED,
                         c="every sample of RECEIVE_R2_B's plan", e=f"the same, except {REPLACED} of the 3,333 seed-5000 entry-correction positions of slots 6-7 (every second one in order) take the onset "
                         f"correction: {FROM_START} uniform over its first min({START_WINDOW}, N) observations, {REPLACED - FROM_START} uniform over all N; the other {3333 - REPLACED} keep the entry correction",
                         positions_that_differ=int((plan["e_from_onset"] == 1).sum()), seed_5000_entry_samples_kept_in_e=3333 - REPLACED,
                         identical_in_c_and_e="slots 0-5, every seed-5002 and seed-5008 position, and the remaining seed-5000 positions",
                         note="a pre-set treatment, not an optimal ratio; it does not separate 'correction present' from 'start-window sampling'"),
        unchanged=dict(normalization_sha256=sha(R2B_TRAINING / "normalization.npz"), horizon_weights_sha256=sha(rc.HORIZON_WEIGHTS), loss="B horizon weighting (single_demo_trial.loss_terms)",
                       optimizer="AdamW over policy.get_optim_params(), lr 1e-4 (backbone 1e-5), wd 1e-4, clip 1.0"),
        evaluation=dict(checkpoint="step-005000 of each condition, fixed before training", contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                        full_order={str(k): list(v) for k, v in FULL_ORDER.items()}, conditional_order={str(k): list(v) for k, v in COND_ORDER.items()},
                        budget="original reset 10 + teacher right_approach states 4 = 14 student runs, one each as ordered, whatever the results",
                        primary="original-reset whole-run valid handover on 5000 and preservation on 5008", horizon="absolute SIM 40 s or the first fault"),
        environment=dict(python=sys.executable, backend=trial.environment_record()))
    target.write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(onset=record["data"]["onset_correction"], observations=len(onset_view), differ=record["sample_plan"]["positions_that_differ"],
                          from_start_window=int(plan["e_from_start_window"].sum()), views_match_r2=True), indent=1))
    print(f"[written] {target}")


def train(args):
    import shutil
    import warnings
    import arm_output_trial as trial
    import single_demo_trial as sd
    import torch
    from torch.utils.data import default_collate
    from dapier_act_policy import ACTPolicy
    condition = args.condition
    record = load(run_dir() / "protocol.json")
    out = training_dir(condition)
    load_report = trial.gpu_other_load()
    if [p for p in load_report.get("other_processes", []) if isinstance(p["used_memory_mib"], int) and p["used_memory_mib"] > 1000]:
        raise RuntimeError("another heavy GPU job is running")
    trial.apply_reproducibility_settings()
    st, replay, train_view, old_view, new_view, onset_view = datasets()
    if sha(run_dir() / "sample_plan.npz") != record["sample_plan"]["sha256"] or r2.view_sha(onset_view) != record["data"]["views"]["onset"]:
        raise RuntimeError("sample plan or onset view changed since the protocol")
    with np.load(run_dir() / "sample_plan.npz", allow_pickle=False) as a:
        plan = {k: a[k] for k in a.files}
    before = frozen()
    policy = ACTPolicy.from_pretrained(R2B, local_files_only=True, strict=True).cuda()
    policy.check_normalization(st)
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=1e-4, weight_decay=1e-4)
    state = torch.load(R2B / "training_state.pt", map_location="cpu", weights_only=False)
    optimizer.load_state_dict(state["optimizer"])
    groups = [dict(lr=g["lr"], weight_decay=g["weight_decay"], tensors=len(g["params"])) for g in optimizer.param_groups]
    with np.load(rc.HORIZON_WEIGHTS, allow_pickle=False) as a:
        r = torch.tensor(a["r"], dtype=torch.float32, device="cuda")
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(R2B_TRAINING / "normalization.npz", out / "normalization.npz")
    onset_key = next(iter(onset_view.episodes))
    consumed_e = np.full((UPDATES, BATCH), -1, dtype=np.int64)
    consumed_t = np.full((UPDATES, BATCH), -1, dtype=np.int64)
    valid_targets = 0
    torch.manual_seed(SEED)
    started_wall, started = time.strftime("%Y-%m-%dT%H:%M:%S%z"), time.monotonic()
    with (out / "metrics.jsonl").open("x") as log, warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        for step in range(1, UPDATES + 1):
            u = step - 1
            items = [train_view[int(i)] for i in plan["train_index"][u]] + [old_view[int(i)] for i in plan["old_index"][u]]
            for i, f, o, t in zip(plan["b_index"][u], plan["b_from_new"][u], plan["e_from_onset"][u], plan["e_t"][u]):
                if condition == "e" and o:
                    items.append(onset_view[int(t)])
                else:
                    items.append(new_view[int(i)] if f else train_view[int(i)])
            batch, meta = trial.split_meta(default_collate(items))
            consumed_e[u], consumed_t[u] = meta["meta_episode"].numpy(), meta["meta_t"].numpy()
            valid_targets += int((~batch["action_is_pad"]).sum())
            batch = {k: v.cuda(non_blocking=True) for k, v in batch.items()}
            policy.train()
            loss, parts = sd.loss_terms(policy, batch, r)
            if not torch.isfinite(loss):
                raise RuntimeError(f"non-finite loss at step {step}")
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            grad = torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0, error_if_nonfinite=True)
            optimizer.step()
            if step == 1 or step % 100 == 0:
                entry = dict(step=step, elapsed_s=round(time.monotonic() - started, 2), loss=loss.item(), gradient_norm=grad.item(), **parts)
                log.write(json.dumps(entry) + "\n")
                log.flush()
                if step % 1000 == 0 or step == 1:
                    print(f"[{condition.upper()}][{step:05d}] loss={entry['loss']:.4f} arm={parts['arm_l1_term']:.4f} grip={parts['gripper_l1_term']:.4f} grad={entry['gradient_norm']:.3f} "
                          f"t={entry['elapsed_s']:.0f}s", flush=True)
        name = out / CHECKPOINT
        policy.save_pretrained(name)
        torch.save(dict(optimizer=optimizer.state_dict(), step=UPDATES, cpu_rng=torch.get_rng_state(), cuda_rng=torch.cuda.get_rng_state()), name / "training_state.pt")
    seconds = round(time.monotonic() - started, 2)
    exp_e = np.concatenate([plan["train_key"], plan["old_key"], plan["b_key"]], axis=1)
    exp_t = np.concatenate([plan["train_t"], plan["old_t"], plan["b_t"]], axis=1)
    if condition == "e":
        on = np.concatenate([np.zeros((UPDATES, 6), bool), plan["e_from_onset"] == 1], axis=1)
        exp_e[on], exp_t[on] = onset_key, np.concatenate([np.zeros((UPDATES, 6), np.int64), plan["e_t"]], axis=1)[on]
    if not (np.array_equal(consumed_e, exp_e) and np.array_equal(consumed_t, exp_t)):
        raise RuntimeError("consumed samples differ from the plan")
    np.savez_compressed(out / "consumed_samples.npz", episode=consumed_e, timestep=consumed_t)
    with np.load(R2B_TRAINING / "consumed_samples.npz", allow_pickle=False) as a:
        r2e, r2t = a["episode"], a["timestep"]
    differ = (consumed_e != r2e) | (consumed_t != r2t)
    new_rows = {x["view_key"]: x for x in new_view.rows}
    cmp = consumed_e[:, 6:]
    seed_key = {x["seed"]: x["view_key"] for x in train_view.rows}
    onset_t = consumed_t[consumed_e == onset_key]
    run = dict(kind="DAPIER ACT RECEIVE R3 onset recovery C / E (SIM only)", run_id=RUN, condition=condition, condition_name=NAMES[condition], policy_variant=policy.config.type,
               dataset=str(td.DATASET), train_episodes=[int(x["manifest_index"]) for x in train_view.rows], holdout_episodes=[], contract_revision=trial.load_manifest()["contract_revision"],
               control_dt=td.DT, hardware_access=False, task_success=None, seed=SEED, batch_size=BATCH, steps=UPDATES, warm_start=str(R2B), warm_start_model_sha256=R2B_SHA,
               optimizer_state_loaded=True, optimizer_state_sha256=sha(R2B / "training_state.pt"), optimizer_step_at_load=int(state["step"]), optimizer_param_groups=groups,
               normalization_sha256=sha(out / "normalization.npz"), phase_views=record["data"]["views"], sample_plan_sha256=record["sample_plan"]["sha256"],
               consumed_sha256=trial.array_sha(consumed_e, consumed_t), consumed_equals_plan=True, samples=int(consumed_e.size), valid_action_targets=valid_targets,
               positions_different_from_r2b_training=int(differ.sum()),
               consumed=dict(compared_slots_6_7={str(s): dict(original_receiving=int((cmp == seed_key[s]).sum()),
                                                              actual_entry_correction=int(sum((cmp == k).sum() for k, x in new_rows.items() if x["seed"] == s)),
                                                              onset_correction=int((cmp == onset_key).sum()) if s == 5000 else 0) for s in r2.SEEDS},
                             onset_samples=int(len(onset_t)), onset_from_first_window=int((plan["e_from_start_window"] == 1).sum()) if condition == "e" else 0,
                             onset_distinct_observations=int(len(np.unique(onset_t))), onset_samples_with_t_below_50=int((onset_t < START_WINDOW).sum())),
               frozen_before=before, frozen_after=frozen(), started=started_wall, training_seconds=seconds, environment=trial.environment_record(), gpu_other_load_at_start=load_report,
               deterministic_warnings=sorted({str(w.message).split("\n")[0][:200] for w in caught}))
    (out / "run.json").write_text(json.dumps(run, indent=2, default=str))
    print(json.dumps(dict(condition=condition, seconds=seconds, differ_from_r2b=run["positions_different_from_r2b_training"], valid_action_targets=valid_targets, consumed=run["consumed"]), indent=1))
    print(f"[written] {name}")


# ----------------------------------------------------------------------------------------------
# fixed: stored observations, deployment mode (no physics)
# ----------------------------------------------------------------------------------------------

def onset_correction():
    e = onset_entry()
    path = Path(e["path"])
    with np.load(path / "episode.npz", allow_pickle=False) as a:
        data = dict(q=a["q_state"].astype(np.float64), cmd=a["q_cmd"].astype(np.float64))
    return dict(end=e["receive"][1], offset=e["source_step"], view=rc.segment_view(rc.stats(), run_dir(), r2.correction_segments([e], 300000)), **data)


def fixed(args):
    from dapier_act_policy import ACTPolicy
    target = run_dir() / "fixed_observations.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    frozen()
    sets = {"5000 entry correction": fc.correction(5000), "5008 entry correction": fc.correction(5008), "5000 onset correction (new)": onset_correction()}
    K = fc.K
    L, R, GR = fc.GROUPS["left_arm"], fc.GROUPS["right_arm"], td.RIGHT_GRIPPER
    data = dict(run_id=RUN, mode="eval, latent zero; inputs = two wrist RGB + q_state; k = 0..24; arm rad, gripper opening fraction", moving_threshold_rad=fc.MOVING, tables={}, windows={})
    for name, path in models().items():
        policy = ACTPolicy.from_pretrained(path, local_files_only=True, strict=True).cuda().eval()
        for label, c in sets.items():
            n = c["end"]
            pred = fc.predict(policy, c["view"])[:, :K]
            steps = np.arange(n)[:, None] + np.arange(K)[None]
            target_c, valid, anchor = c["cmd"][np.minimum(steps, n - 1)], steps < n, c["q"][:n]
            data["tables"][f"{name} | {label}"] = dict(observations=n, valid_targets=int(valid.sum()),
                                                       groups={g: fc.group_stats(pred, target_c, anchor, valid, cols) for g, cols in fc.GROUPS.items()})

            def window(lo, hi, cols):
                sel = np.arange(lo, min(hi, n))
                sel = sel[valid[sel, K - 1]]
                want, got = (target_c[sel, K - 1] - anchor[sel])[:, cols], (pred[sel, K - 1] - anchor[sel])[:, cols]
                move = np.linalg.norm(want, axis=1) > fc.MOVING
                ratio = (got[move] * want[move]).sum(1) / (want[move] ** 2).sum(1) if move.any() else np.array([])
                return dict(observations=int(len(sel)), teacher_move_mean=round(float(np.linalg.norm(want, axis=1).mean()), 4), predicted_move_mean=round(float(np.linalg.norm(got, axis=1).mean()), 4),
                            moving=int(move.sum()), progress_median=None if not len(ratio) else round(float(np.median(ratio)), 2), progress_min=None if not len(ratio) else round(float(ratio.min()), 2),
                            mae=round(float(np.abs(pred[sel] - target_c[sel])[..., cols][valid[sel]].mean()), 4))

            w = {}
            if label.startswith("5000 entry"):
                w["entry 9.5-10.5 s right arm (local 0-50)"] = window(0, 51, R)
                w["entry 9.5-10.5 s right gripper k24: target / predicted"] = [[round(float(target_c[i, K - 1, GR]), 3), round(float(pred[i, K - 1, GR]), 3)] for i in (0, 25, 50)]
                w["transfer onset 12.7-13.1 s left arm (local 160-180)"] = window(160, 181, L)
                w["transfer body 13.3-16.5 s left arm (local 190-350)"] = window(190, 351, L)
            elif label.startswith("5008"):
                trans = [i for i, p in enumerate(c["phases"][:n]) if p == "transfer_approach"]
                w["transfer_approach left arm"] = window(trans[0], trans[-1] + 1, L)
                w["transfer_approach right arm"] = window(trans[0], trans[-1] + 1, R)
            else:
                w["first 50 observations left arm"] = window(0, 50, L)
                w["first 50 observations right arm"] = window(0, 50, R)
                w["all observations left arm"] = window(0, n, L)
                w["all observations right arm"] = window(0, n, R)
            data["windows"][f"{name} | {label}"] = w
        del policy
    target.write_text(json.dumps(data, indent=2))
    for k, w in data["windows"].items():
        print(k)
        for a, b in w.items():
            print("   ", a, b)
    print(f"[written] {target}")


# ----------------------------------------------------------------------------------------------
# Evaluation under a verified identity
# ----------------------------------------------------------------------------------------------

def jobs():
    out = []
    for seed, order in FULL_ORDER.items():
        count = {}
        for mdl in order:
            count[mdl] = count.get(mdl, 0) + 1
            out.append(("full", seed, mdl, count[mdl]))
    return out + [("conditional", seed, mdl, 1) for seed, order in COND_ORDER.items() for mdl in order]


def identity(sel):
    record = load(run_dir() / "protocol.json")
    paths = dict(START=rc.MODELS["START"], RELEASE_B=rc.MODELS["B"], RECEIVE_R2B=R2B, RECEIVE_C=models()["C"], RECEIVE_E=models()["E"])
    body = dict(run_id=RUN, models={k: {f: sha(p / f) for f in ("model.safetensors", "config.json")} | dict(normalization=sha(p.parent / "normalization.npz")) for k, p in paths.items()},
                optimizer_start_sha256=record["start"]["optimizer_state_sha256"], views=record["data"]["views"], sample_plan_sha256=record["sample_plan"]["sha256"],
                onset_state_sha256=load(run_dir() / "selection.json")["state_sha256"], conditional_states={str(s): v["record"]["state_sha256"] for s, v in sel.items()},
                resets=r2.identity()["resets"], sources={p: sha(FACTORY_ROOT / p) for p in SOURCES}, contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                execution=dict(prefix=rc.PREFIX, arm_bound_policy="project", ensemble="off", horizon_steps=rc.HORIZON, torch_seed=4100))
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def case_dir(ident, kind, seed, model, repeat):
    return run_dir() / "runs" / ident["sha256"][:12] / kind / f"s{seed}" / f"{model}_run{repeat}"


def complete(out, ident, case):
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (r.get("identity_sha256") == ident["sha256"] and r.get("case") == list(case) and meta.get("complete") and r.get("boundary_contract") == "repaired"
          and all((out / f).exists() for f in ("episode.npz", "action_trace.json", "task_check_rows.json")))
    return r if ok else None


def arm_motion(path, entry_step):
    """Descriptive markers of the transfer in a stored run (analysis only): when the left arm left its entry pose, how far the
    right arm wandered before that."""
    with np.load(Path(path) / "episode.npz", allow_pickle=False) as a:
        q = a["q_state"].astype(np.float64)
    L, R = list(td.LEFT_ARM), list(td.RIGHT_ARM)
    dl = np.abs(q[entry_step:] - q[entry_step])[:, L].max(1)
    first = lambda level: next((round((entry_step + i) * td.DT, 2) for i in range(len(dl)) if dl[i] > level), None)
    depart = first(DEPART[0])
    hi = len(q) if depart is None else int(round(depart / td.DT))
    lo = min(entry_step + 150, hi - 1)                                    # 3 s after entry: the right arm's retreat is over in the teacher runs
    drift = float(np.abs(q[lo:hi, R] - q[lo, R]).max()) if hi > lo else 0.0
    return dict(left_arm_departs_s=depart, left_arm_at_transfer_s=first(DEPART[1]), left_arm_max_change_rad=round(float(dl.max()), 3),
                right_arm_drift_before_departure_rad=round(drift, 3), right_gripper_open_s=next((round((entry_step + i) * td.DT, 2) for i in range(len(q) - entry_step) if q[entry_step + i, td.RIGHT_GRIPPER] > 0.95), None))


def one(args):
    sel = fc.selection()
    ident = identity(sel)
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    out = case_dir(ident, args.kind, args.seed, args.model, args.repeat)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    rc.MODELS["RECEIVE"] = models()[args.model]
    name = NAMES[args.model.lower()]
    if args.kind == "full":
        result = rc.execute(out, args.seed, ("START", "RECEIVE", "B"), None, f"seed {args.seed} START -> {name} -> RELEASE_B run{args.repeat}", contract="repaired")
        t0 = result["receive_entry_s"] or 0.0
        entry = next((s["step"] for s in result["switches"] if s["to"] == "RECEIVE"), None)
        result.update(chain=f"START -> {name} -> RELEASE_B", transfer=None if entry is None else arm_motion(out, entry), handover_event=result["handover_s"] is not None)
    else:
        s = sel[args.seed]
        result = rc.execute(out, args.seed, ("RECEIVE", "B"), dict(states=s["states"], q=s["q"], rgb=s["rgb"], step=s["step"]),
                            f"{fc.LABEL}: s{args.seed} teacher right_approach state, {name}", contract="repaired")
        t0 = s["step"] * td.DT
        result.update(chain=f"{name} -> RELEASE_B", classification=fc.LABEL + " (" + LABEL + " switches)", start_state=s["record"], transfer=None,
                      handover_event=result["handover_s"] is not None and result["handover_s"] > t0)
    rows = load(out / "task_check_rows.json")
    result.update(kind=args.kind, case=[args.kind, args.seed, args.model, args.repeat], receive_model=args.model, receive_name=name, repeat=args.repeat, identity_sha256=ident["sha256"],
                  contract_checks=rb.contract_checks(result), phases=r2.phase_details(result, rows, t0))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(kind=args.kind, seed=args.seed, model=args.model, repeat=args.repeat, order=result["phases"]["order"], transfer=result["transfer"], termination=result["termination"],
                          run_valid=result["run_valid"], handover_event=result["handover_event"], task_success=result["task_success"], final_support=result["final_support"]), default=str))


def table_row(r):
    q = r["receiving"] or {}
    fault = r["first_fault"]
    return dict(kind=r["kind"], seed=r["seed"], receive=r["receive_model"], repeat=r["repeat"], pick_lift_s=r["pick_lift_s"],
                receive_entry_s=r["receive_entry_s"] if r["kind"] == "full" else r["start_state"]["sim_time_s"], transfer=r["transfer"], right_grasp_s=r["right_grasp_s"],
                semantic_ready_s=r["phases"]["semantic_ready_s"], release_b_switch_s=r["gate_s"], left_release_s=r["left_release_while_right_holds_first_s"],
                left_retreat_s=r["left_clear_by_donor_clearance_first_s"], handover_event_s=r["handover_s"] if r["handover_event"] else None,
                first_fault_s=None if fault is None else round(fault["time"], 2), fault_stage=r["fault_stage"], fault_pairs=r["phases"]["fault_pairs"], termination=r["termination"],
                run_valid=r["run_valid"], final_support=r["final_support"], task_success=r["task_success"], order=r["phases"]["order"],
                projection={k: dict(steps=v["projected_steps"], of=v["dispatched"], longest_s=v["longest_s"], max_raw_excess_rad=round(v["max_raw_excess_rad"], 4), per_axis=v["per_axis"])
                            for k, v in r["projection"].items()},
                right_before_two_pad_hold=q.get("right_before_two_pad_hold"), block_in_left_site=q.get("block_in_left_site"), left_hold_lost_steps=q.get("left_hold_lost_steps"),
                c25_regular=r["contract_checks"]["c25_consumption_regular"], switches=r["contract_checks"]["switches"])


def run(args):
    for c in ("c", "e"):
        if not (training_dir(c) / CHECKPOINT / "model.safetensors").exists():
            raise FileNotFoundError(f"{NAMES[c]} {CHECKPOINT} does not exist: run `train --condition {c}` first")
    frozen()
    sel = fc.selection()
    ident = identity(sel)
    plan = jobs()
    (run_dir() / "logs").mkdir(exist_ok=True)
    (run_dir() / f"evaluation_identity_{ident['sha256'][:12]}.json").write_text(json.dumps(dict(identity=ident, cases=[list(j) for j in plan], fixed=time.strftime("%Y-%m-%dT%H:%M:%S%z")), indent=2))
    status = []
    for case in plan:
        kind, seed, mdl, rep = case
        out = case_dir(ident, *case)
        if complete(out, ident, case) is not None:
            status.append(dict(case=list(case), status="completed (reused: same identity, complete files)"))
            print(f"[reused under identity {ident['sha256'][:12]}] {case}")
            continue
        if out.exists():
            status.append(dict(case=list(case), status="NOT RUN: an incomplete folder of this identity exists; neither reused nor overwritten", path=str(out)))
            print(f"[not run] {case}: incomplete folder {out}")
            continue
        done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--kind", kind, "--seed", str(seed), "--model", mdl, "--repeat", str(rep), "--identity", ident["sha256"]],
                              capture_output=True, text=True)
        (run_dir() / "logs" / f"{ident['sha256'][:12]}_{kind}_s{seed}_{mdl}_run{rep}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        ok = done.returncode == 0 and complete(out, ident, case) is not None
        status.append(dict(case=list(case), returncode=done.returncode, status="completed" if ok else "EXECUTION ERROR (not a task result)", **({} if ok else dict(stderr_tail=done.stderr[-600:]))))
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(line[-1][:900] if ok and line else f"[EXECUTION ERROR, not a task failure] {case}: returncode {done.returncode}: {done.stderr[-300:]}", flush=True)
    results = [r for case in plan if (r := complete(case_dir(ident, *case), ident, case)) is not None]
    tally = {}
    for kind in ("full", "conditional"):
        for mdl in ("C", "E"):
            for seed in (5000, 5008, 5002):
                rs = [r for r in results if r["kind"] == kind and r["receive_model"] == mdl and r["seed"] == seed]
                if rs:
                    tally[f"{kind} {mdl} s{seed}"] = dict(runs=len(rs), left_arm_departed=sum(bool(r["transfer"] and r["transfer"]["left_arm_departs_s"]) for r in rs) if kind == "full" else None,
                                                         semantic_ready=sum(r["phases"]["semantic_ready_s"] is not None for r in rs), release_b_switch=sum(r["gate_reached"] for r in rs),
                                                         handover_event=sum(r["handover_event"] for r in rs), valid_task_success=sum(r["task_success"] for r in rs))
    (run_dir() / "results.json").write_text(json.dumps(dict(
        run_id=RUN, note=f"{LABEL}; switches read SIM ground truth; Train parents. 'full' = original reset; 'conditional' = teacher right_approach states ({fc.LABEL}). The two tables, the teacher "
                         "result and earlier runs are never added to each other.",
        identity_sha256=ident["sha256"], identity_unchanged_during_run=identity(sel)["sha256"] == ident["sha256"], planned_cases=len(plan), completed_cases=len(results),
        execution_status=status, tally=tally, table=[table_row(r) for r in results], results=results), indent=2, default=str))
    errors = [s for s in status if not s["status"].startswith("completed")]
    print(f"[written] {run_dir() / 'results.json'}: {len(results)} of {len(plan)} cases completed, {len(errors)} execution error(s) / not run; identity {ident['sha256'][:12]}")
    summary(args)


def summary(args=None):
    path = run_dir() / "results.json"
    if not path.exists():
        print(f"RECEIVE R3 ({RUN}): unavailable, no results.json")
        return
    data = load(path)
    fmt = lambda v: "-" if v is None else (f"{v:.2f}" if isinstance(v, float) else str(v))
    print(f"\n=== RECEIVE R3 onset recovery C / E ({RUN}) — {LABEL}; 전환은 SIM 정답 기반. 다른 평가·교사 결과와 합산하지 않는다 ===")
    print(f"C = RECEIVE_REPLAY_CONTROL, E = RECEIVE_ONSET_RECOVERY. 완료 {data['completed_cases']}/{data['planned_cases']}건, "
          f"실행 오류 {sum(not s['status'].startswith('completed') for s in data['execution_status'])}건, identity {data['identity_sha256'][:12]}")
    for kind, title in (("full", "원래 reset 전체 연결"), ("conditional", "교사 right_approach 상태(조건부 진단, 원래 reset 아님)")):
        print(f"\n[{title}]\n| seed | 모델 | 회 | 왼팔 출발 | 수신 준비 | B 전환 | handover | 첫 fault | 최종 지지 | 전체 유효 |\n|---|---|---|---|---|---|---|---|---|---|")
        for t in data["table"]:
            if t["kind"] == kind:
                depart = "-" if not t["transfer"] else fmt(t["transfer"]["left_arm_departs_s"])
                print(f"| {t['seed']} | {t['receive']} | {t['repeat']} | {depart} | {fmt(t['semantic_ready_s'])} | {fmt(t['release_b_switch_s'])} | {fmt(t['handover_event_s'])} | "
                      f"{fmt(t['first_fault_s'])} | {t['final_support']} | {'예' if t['task_success'] else '아니오'} |")
    print("\n집계(실행 / 수신 준비 / B 전환 / handover 이벤트 / 전체 유효):")
    for k, v in data["tally"].items():
        print(f"  {k}: {v['runs']} / {v['semantic_ready']} / {v['release_b_switch']} / {v['handover_event']} / {v['valid_task_success']}")
    print(f"  결과: {path}\n  보고서: {run_dir() / 'REPORT.md'}")


def video(args):
    import recovery_trial as rt
    sel = fc.selection()
    ident = identity(sel)
    for seed in (5000, 5008):
        out = run_dir() / "videos" / f"seed{seed}_original_reset_C_E_run1_{ident['sha256'][:12]}.mp4"
        if out.exists():
            print(f"[exists] {out}")
            continue
        rows = []
        for mdl in ("C", "E"):
            case = ("full", seed, mdl, 1)
            r = complete(case_dir(ident, *case), ident, case)
            if r is None:
                print(f"[unavailable] {case}: no completed case under this identity; the row is not replaced")
                continue
            row = rt.teacher_row(f"{r['chain']} (original reset, run 1)", Path(r["path"]))
            ev = dict(r["events"])
            ev.update({f"SWITCH->{s['to']}": s["sim_time_s"] for s in r["switches"]})
            row["events"] = dict(sorted(ev.items(), key=lambda kv: kv[1]))
            row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}" + ("" if r["gate_reached"] else " (RELEASE_B NOT REACHED)"),
                               "VALID HANDOVER, right hand holds" if r["task_success"] else f"no valid handover; final support: {r['final_support']}")
            rows.append(row)
        if rows:
            rt.render(rows, out, f"{LABEL} / R3 / seed {seed} original reset", 2000, False, right_gripper=True)
            print(f"[written] {out}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("teacher", "protocol", "fixed", "run", "summary", "video"):
        sub.add_parser(name)
    t = sub.add_parser("train")
    t.add_argument("--condition", choices=("c", "e"), required=True)
    o = sub.add_parser("one")
    o.add_argument("--kind", choices=("full", "conditional"), required=True)
    o.add_argument("--seed", type=int, choices=(5000, 5002, 5008), required=True)
    o.add_argument("--model", choices=("C", "E"), required=True)
    o.add_argument("--repeat", type=int, choices=(1, 2), required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    dict(teacher=teacher, protocol=protocol, train=train, fixed=fixed, run=run, one=one, summary=summary, video=video)[args.command](args)


if __name__ == "__main__":
    main()
