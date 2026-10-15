#!/usr/bin/env python3
"""RECEIVE-BRIDGE R1 / work6: teacher corrections from the actual Train hand-over states of START_R1_E, and a replay-matched
comparison of two copies of the frozen RECEIVE (SIM / learning only; no hardware, no Val / Test teacher or label).

This file is the execution code (teacher, sample plan, training, student cases) and is part of the run identity; tables,
frames and videos live in receive_bridge_r1_report.py and are not.

  protocol  sources, exclusions, the 44 student cases and their order, sampling rules: fixed before any teacher run
  teacher   one teacher run from each of five stored RECEIVE-entry states of START-PICK R1 E (Train, first run only);
            at the first repaired 50 Hz readiness sample the frozen RELEASE_B continues to the absolute 40 s
  plan      CTRL = the samples RECEIVE_FROZEN's own training consumed; ADAPT = the same first four slots, and four slots
            from the valid new correction prefixes [0, T_ready)
  train     --condition ctrl | adapt : RECEIVE_FROZEN weights + its stored AdamW state, 5,000 updates each
  run       44 student cases (2 unchanged references, 18 main Train, 4 probes with the frozen START, 20 development Val)
  all       every step that is not complete yet

Switches read SIM ground truth: SIM-ORACLE-GATED THREE-POLICY. Teacher results, references, main runs, probes and Val are
never added together.
"""
import argparse
import hashlib
import json
import os
import shutil
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
import receive_onset_r3 as r3   # noqa: E402
import receive_r3_recheck as rk   # noqa: E402
import receive_r3_val as rv   # noqa: E402
import start_pick_d1 as d1   # noqa: E402
import start_pick_r1 as p1   # noqa: E402
import server_s0 as s0   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "receive-bridge-r1-work6-20261006-01"
TITLE = "RECEIVE-BRIDGE R1 / work6 / replay control vs actual-hand-over adaptation"
LABEL = "SIM-ORACLE-GATED THREE-POLICY / RECEIVE-BRIDGE R1 work6"
R1_RUNS = FACTORY_ROOT / "evaluation" / p1.RUN / "runs" / "5e16923912ed"
R1_IDENTITY = "5e16923912ed"
MODELS = dict(START_FROZEN=p1.START, START_R1_E=p1.checkpoint("e"), RECEIVE_FROZEN=rk.MODELS["E"], RELEASE_B=rk.MODELS["RELEASE_B"])
EXPECTED = dict(START_FROZEN=rk.EXPECTED["START"], START_R1_E="05c7593fa0f738013dde6e9fcae434b3c22b9c64b6b2294065ddbe5723774ed5", RECEIVE_FROZEN=rk.EXPECTED["E"],
                RELEASE_B=rk.EXPECTED["RELEASE_B"])
SOURCE_ORDER = ((5002, 375), (5008, 400), (5098, 350), (5101, 300), (5199, 825))     # Train seed, stored control step of RECEIVE's first command
NAMES = dict(ctrl="RECEIVE_BRIDGE_CTRL", adapt="RECEIVE_BRIDGE_ADAPT")
TRAIN9, VAL10 = p1.TRAIN9, p1.VAL10
UPDATES, BATCH, SEED, COMMON = 5000, 8, 4100, 4
CHECKPOINT = "step-005000"
NEW_BASE = 400000
E_TRAINING = r3.training_dir("e")
REPLAY_STEPS = 25
SOURCES = d1.SOURCES + ("factory/v1/start_pick_r1.py", "factory/v1/server_s0.py", "factory/v1/receive_bridge_r1.py")

load, sha, array_sha = rk.load, rk.sha, rk.array_sha


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def training_dir(condition):
    return FACTORY_ROOT / "training" / f"{RUN}-{condition}"


def checkpoint(condition):
    return training_dir(condition) / CHECKPOINT


def source_case(seed):
    return R1_RUNS / "train" / f"s{seed}" / "E_rep1"


def cases():
    """44 student cases in the fixed order: (group, split, seed, START, RECEIVE)."""
    alt = lambda n: ("CTRL", "ADAPT") if n % 2 else ("ADAPT", "CTRL")
    out = [("reference", "train", s, "FROZEN", "FROZEN") for s in (5000, 5008)]
    for n, seed in enumerate(TRAIN9, 1):
        out += [("main", "train", seed, "R1E", m) for m in alt(n)]
    out += [("probe", "train", 5000, "FROZEN", "ADAPT"), ("probe", "train", 5000, "FROZEN", "CTRL"), ("probe", "train", 5008, "FROZEN", "CTRL"), ("probe", "train", 5008, "FROZEN", "ADAPT")]
    for n, seed in enumerate(VAL10, 1):
        out += [("main", "val", seed, "R1E", m) for m in alt(n)]
    return out


def frozen():
    got = {k: dict(path=str(p), model=sha(p / "model.safetensors"), config=sha(p / "config.json"), normalization=sha(p.parent / "normalization.npz")) for k, p in MODELS.items()}
    if any(got[k]["model"] != EXPECTED[k] or got[k]["normalization"] != rk.NORMALIZATION for k in MODELS):
        raise RuntimeError(f"a read-only model or its normalization differs from the instructed SHA-256: {got}")
    if sha(td.DATASET / "manifest.json") != p1.MANIFEST_SHA:
        raise RuntimeError("the frozen manifest differs from the instructed SHA-256")
    return got


def protocol(args=None):
    import torch
    import arm_output_trial as trial
    target = run_dir() / "protocol.json"
    if target.exists():
        return load(target)
    run_dir().mkdir(parents=True, exist_ok=True)
    fz = frozen()
    state = torch.load(MODELS["RECEIVE_FROZEN"] / "training_state.pt", map_location="cpu", weights_only=False)
    e_run = load(E_TRAINING / "run.json")
    sources = []
    for order, (seed, step) in enumerate(SOURCE_ORDER, 1):
        case = source_case(seed)
        result = load(case / "result.json")
        sw = next(s for s in result["switches"] if s["to"] == "RECEIVE")
        if sw["step"] != step or result["identity_sha256"][:12] != R1_IDENTITY or result["case"] != ["train", seed, "E", 1]:
            raise RuntimeError(f"seed {seed}: the stored RECEIVE entry is not step {step} of START-PICK R1 identity {R1_IDENTITY} (train, E, first run)")
        sources.append(dict(order=order, seed=seed, split="train", source_case=str(case), control_step=step, receive_entry_s=round(step * td.DT, 2),
                            remaining_absolute_sim_s=round((rc.HORIZON - step) * td.DT, 2), files={f: sha(case / f) for f in ("episode.npz", "cameras/left_wrist_rgb.npz", "cameras/right_wrist_rgb.npz", "result.json")}))
    record = dict(
        run_id=RUN, name=TITLE, label=LABEL, fixed_before_teacher=time.strftime("%Y-%m-%dT%H:%M:%S%z"), hardware_access=False, val_test_used_for_teacher_or_training=False,
        host=s0.host(), environment=dict(python=sys.executable, backend_of_this_writer_process=trial.environment_record(), env={k: os.environ.get(k) for k in rk.ENV_KEYS},
                                         note="this is the process that wrote the protocol; training and student children record their own settings"),
        frozen=fz, manifest_sha256=p1.MANIFEST_SHA,
        start=dict(checkpoint=str(MODELS["RECEIVE_FROZEN"]), model_sha256=EXPECTED["RECEIVE_FROZEN"], optimizer_state=str(MODELS["RECEIVE_FROZEN"] / "training_state.pt"),
                   optimizer_state_sha256=sha(MODELS["RECEIVE_FROZEN"] / "training_state.pt"), optimizer_step_in_file=int(state["step"]),
                   use="both conditions: all RECEIVE_FROZEN weights and its stored AdamW state, read-only; nothing re-initialised; torch.manual_seed(4100)"),
        correction_sources=dict(rule="the state just before RECEIVE's first command in the first START-PICK R1 E run of each Train parent that entered RECEIVE; one teacher run each, in this order, "
                                     "no retry, no other start point", sources=sources,
                                excluded=[dict(case="train 5008 E rep2", reason="a repeat of the same reset; not an independent source"),
                                          dict(case="val 5079 E, val 5134 E", reason="Val: no teacher run, label, training or normalization use"),
                                          dict(case="train 5000 E", reason="START_R1_E did not reach RECEIVE"), dict(case="any START_R1_C run", reason="START_R1_E is the fixed state generator")],
                                note="a development correction set chosen from the Train conditions START_R1_E picked; not a random or generalization sample"),
        teacher=dict(tool="handover_correction_trial.continuation (ReentryTeacher: the unchanged HandoverTeacher re-entered from the restored state)",
                     restore="the stored full integration state (time, qpos, qvel, act, ctrl, warmstart) via mj_setState; the stored prefix states are the checker history; nothing is moved, "
                             "re-grasped, settled or zeroed", time_budget="the run stops at the absolute SIM 40 s of the parent run (2000 control steps in total)",
                     readiness="first 50 Hz sample at or after the checker's right_grasp where both hands hold and the table gives no support (rb.semantic_bounds, start_at_first_row)",
                     then="frozen RELEASE_B continues from that very state (rc.execute chain ('B',), repaired contract) to the absolute 40 s or the first existing fault",
                     outputs=["teacher_ready_valid", "teacher_receive_plus_frozen_B_valid"], note="a teacher + student-B mixed diagnostic; never a student success from the original reset"),
        training=dict(conditions=NAMES, updates=UPDATES, batch=BATCH, seed=SEED, final_checkpoint=CHECKPOINT,
                      replay=dict(source=str(E_TRAINING / "consumed_samples.npz"), sha256=sha(E_TRAINING / "consumed_samples.npz"), consumed_sha256_in_run_json=e_run["consumed_sha256"],
                                  meaning="the 5,000 x 8 samples RECEIVE_FROZEN's own training consumed (Train69 receiving view, re-cut earlier corrections, actual-entry corrections, the onset "
                                          "correction), rebuilt from the same four views"),
                      slots="per update the eight replay samples are put in a fixed-RNG order; slots 0-3 are common to CTRL and ADAPT; slots 4-7: CTRL keeps the replay samples, ADAPT takes "
                            "new correction samples (source parent first, by joined fixed-RNG permutations of the valid sources, then a uniform timestep of its [0, T_ready))",
                      if_no_valid_correction="no training is started; the teacher results are the end of the job",
                      if_some="every valid prefix is used, whatever its number; 'at least one' is a data-existence condition, not a sufficiency or adoption criterion",
                      unchanged="architecture, two wrist RGB + measured q, Train normalization, B horizon loss, KL, AdamW lr 1e-4 (backbone 1e-5) wd 1e-4 clip 1.0, float32, no autocast",
                      note="what the compared slots hold and how often the old distribution is seen change together; not a factorial design, not an optimal ratio"),
        evaluation=dict(cases=[list(c) for c in cases()], planned=len(cases()),
                        groups=dict(reference="START_FROZEN -> RECEIVE_FROZEN -> RELEASE_B on Train 5000 / 5008, once each: two same-period references",
                                    main="START_R1_E -> RECEIVE_BRIDGE_{CTRL|ADAPT} -> RELEASE_B on Train9 and development Val10 (odd position CTRL->ADAPT, even ADAPT->CTRL)",
                                    probe="START_FROZEN -> RECEIVE_BRIDGE_{CTRL|ADAPT} -> RELEASE_B on Train 5000 (ADAPT->CTRL) and 5008 (CTRL->ADAPT), after Train and before Val"),
                        contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"], checkpoint=f"{CHECKPOINT} of each condition, fixed before the teacher runs",
                        rule="original reset, one new process per case, absolute SIM 40 s or the first existing fault; a task fault is that case's result; no repeat, no replacement; Val runs "
                             "whatever the Train results; an executor error keeps its attempt folder and only that case is run again",
                        denominators="teacher 5, reference 2, main Train 18, probe 4, development Val 20 are separate tables; nothing is added to S0, D1 or START-PICK R1"))
    target.write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(sources=[(s["seed"], s["control_step"]) for s in sources], cases=len(cases()), optimizer_step_in_file=record["start"]["optimizer_step_in_file"])))
    return record


# ----------------------------------------------------------------------------------------------
# Teacher from the stored hand-over states
# ----------------------------------------------------------------------------------------------

def replay_check(seed, case, step):
    """Serializer check (not a task result): restore the stored state and apply the next stored controls; the stored states
    that followed must come back. Nothing stored is changed."""
    import mujoco
    import frozen_transfer_check as ft
    from sim_data_factory.reset_sampler import make_task_episode
    meta = ft.demonstration(seed)["metadata"]
    ctx = make_task_episode(seed, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
    m, d = ctx["model"], ctx["data"]
    spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    with np.load(case / "episode.npz", allow_pickle=False) as a:
        states, ctrl = a["integration_state"].copy(), a["ctrl_applied"].copy()
    mujoco.mj_setState(m, d, states[step], spec)
    mujoco.mj_forward(m, d)
    back = np.empty(mujoco.mj_stateSize(m, spec))
    mujoco.mj_getState(m, d, back, spec)
    same = bool(np.array_equal(back, states[step]))
    n = min(REPLAY_STEPS, len(ctrl) - step)
    worst = 0.0
    for i in range(n):
        d.ctrl[:] = ctrl[step + i]
        for _ in range(ctx["physics_steps"]):
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        mujoco.mj_getState(m, d, back, spec)
        worst = max(worst, float(np.abs(back - states[step + i + 1]).max()))
    return dict(restored_state_equals_stored=same, replayed_control_steps=n, max_abs_state_difference=worst, bit_identical=worst == 0.0,
                state_size=int(len(states[step])), layout="mjSTATE_INTEGRATION: time, qpos, qvel, act, ctrl, qfrc_warmstart and the other integration fields of this model")


def teacher_one(args):
    import handover_correction_trial as hc
    import paired_release_check as pr
    from integration_scenes import task_env
    from sim_data_factory import student_eval
    from sim_data_factory.task_success import evaluate_states
    seed = args.seed
    step = dict(SOURCE_ORDER)[seed]
    name = f"s{seed}_entry_{step:04d}"
    target = run_dir() / "teacher" / f"{name}.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; the teacher is not run twice from one source")
    proto = load(run_dir() / "protocol.json")
    src = next(s for s in proto["correction_sources"]["sources"] if s["seed"] == seed)
    case = Path(src["source_case"])
    if any(sha(case / f) != v for f, v in src["files"].items()):
        raise RuntimeError("a source file changed since the protocol")
    target.parent.mkdir(parents=True, exist_ok=True)
    frozen()
    g = rc.stored_state(case, step)
    with np.load(case / "episode.npz", allow_pickle=False) as a:
        qvel = a["qvel"][step].copy()
    snap = run_dir() / "source_states" / f"{name}.npz"
    snap.parent.mkdir(exist_ok=True)
    np.savez_compressed(snap, integration_state=g["states"][-1], q_state=g["q"][-1], qvel=qvel, left_wrist_rgb=g["rgb"]["left"], right_wrist_rgb=g["rgb"]["right"])
    entry = dict(id=name, seed=seed, order=src["order"], source_case=str(case), source_step=step, source_sim_time_s=float(g["states"][-1][0]), snapshot=str(snap),
                 state_sha256=hc.state_sha(g["states"][-1]), prefix_states=len(g["states"]), serializer_check=replay_check(seed, case, step))
    original, limit = student_eval.Observations, hc.TEACHER_MAX_STEPS
    sensors = []

    def first_stored(*a, **k):                                             # the recorded first observation is the stored frame of that state
        student_eval.Observations = original
        sensors.append(pr.stored_first_sensor(dict(rgb=g["rgb"])))
        return sensors[-1]

    student_eval.Observations, hc.TEACHER_MAX_STEPS = first_stored, rc.HORIZON - step        # the absolute 40 s of the parent run, not a new 40 s
    try:
        got = hc.continuation(run_dir() / "corrections" / name, seed, g["states"], g["q"], g["states"][-1],
                              dict(classification="teacher_receive_from_actual_START_R1_E_handover_state", source_student_case=str(case), source_step=step,
                                   source_state_sha256=entry["state_sha256"], note="a child of the Train parent, not a new independent initial condition"))
    finally:
        student_eval.Observations, hc.TEACHER_MAX_STEPS = original, limit
    entry.update(got)
    entry["first_observation_rerender_equals_stored"] = sensors[0].rerender_equals_stored if sensors else None
    receive = dict(teacher_ready_valid=False, failure_class=None, reason=None)
    mixed = dict(teacher_receive_plus_frozen_B_valid=None, reason="RELEASE_B was not started: no valid readiness")
    if not got.get("path"):
        receive.update(failure_class="planning failure" if got.get("teacher_failure") else "execution failure", reason=str(got.get("teacher_failure") or got.get("terminal_failure")))
    else:
        path = Path(got["path"])
        rows = load(path / "task_check_rows.json")
        _, end, why = rb.semantic_bounds(rows, got["events"], start_at_first_row=True)
        with np.load(path / "episode.npz", allow_pickle=False) as a:
            suffix, suffix_q = a["integration_state"].copy(), a["q_state"].astype(np.float64)
        entry["files"] = {f: sha(path / f) for f in ("episode.npz", "cameras/left_wrist_rgb.npz", "cameras/right_wrist_rgb.npz", "task_check_rows.json")}
        right_touch = next((round(r["time"], 2) for r in rows if max(r["pad_force_N"]["right"]) > 0), None)
        receive.update(right_first_pad_touch_s=right_touch, teacher_stop=got["stop"], teacher_transitions=got["transitions"])
        if why:
            fault = got.get("teacher_failure") or got.get("terminal_failure")
            receive.update(failure_class="execution failure" if fault else "readiness not reached within the absolute 40 s", reason=f"{why}; stop={got['stop']}; {fault}")
        else:
            m = task_env("desk", grippers="both").model
            hist, hist_q = np.concatenate([g["states"], suffix[1:end + 1]]), np.concatenate([g["q"], suffix_q[1:end + 1]])
            check = evaluate_states(m, hist, hist_q, control_dt=td.DT)
            receive.update(teacher_ready_valid=bool(check["valid"]), receive=[0, end], T_ready_local=end, receive_abs_s=[round(step * td.DT, 2), round(rows[end]["time"], 2)],
                           absolute_step_at_ready=step + end, prefix_checker_valid=bool(check["valid"]), prefix_failures=check["failures"][:3],
                           events_up_to_readiness={k: round(v, 2) for k, v in check["events"].items()}, observations=end,
                           teacher_own_continuation=dict(stop=got["stop"], whole_run_valid=got.get("checker_valid"), handover=got["events"].get("handover"), note="not used; RELEASE_B continues instead"))
            if not check["valid"]:
                receive.update(failure_class="execution failure", reason="the history up to readiness is not checker-valid")
            else:
                rgb = {s: np.load(path / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}
                if end >= len(rgb["left"]):
                    mixed = dict(teacher_receive_plus_frozen_B_valid=None, reason="unavailable: no stored camera frame at the readiness state")
                else:
                    out = run_dir() / "teacher_plus_b" / name
                    result = rc.execute(out, seed, ("B",), dict(step=step + end, states=hist, q=hist_q, rgb={s: rgb[s][end] for s in rgb}),
                                        f"{LABEL}: teacher RECEIVE from the stored hand-over state of Train {seed}, then frozen RELEASE_B", contract="repaired")
                    mixed = dict(teacher_receive_plus_frozen_B_valid=bool(result["task_success"]), path=str(out), release_b_from_s=round((step + end) * td.DT, 2),
                                 handover_s=result["handover_s"], termination=result["termination"], final_support=result["final_support"], first_fault=result["first_fault"],
                                 restore_identity=load(out / "metadata.json").get("restored_from"), events={k: round(v, 2) for k, v in result["events"].items()},
                                 note="START_R1_E student prefix + teacher RECEIVE + student RELEASE_B: a mixed diagnostic, not a student run from the original reset")
    entry["receiving"], entry["mixed"] = receive, mixed
    target.write_text(json.dumps(entry, indent=2, default=str))
    print(json.dumps(dict(id=name, restore_identity=got.get("restore_identity"), replay=entry["serializer_check"]["max_abs_state_difference"], reentry=got.get("reentry"), stop=got.get("stop"),
                          receiving={k: receive.get(k) for k in ("teacher_ready_valid", "receive_abs_s", "failure_class", "reason", "right_first_pad_touch_s")},
                          mixed={k: mixed.get(k) for k in ("teacher_receive_plus_frozen_B_valid", "handover_s", "termination", "final_support", "reason")}), default=str)[:1800])


def teacher(args=None):
    target = run_dir() / "teacher_collection.json"
    if target.exists():
        return load(target)
    load(run_dir() / "protocol.json")
    (run_dir() / "logs").mkdir(exist_ok=True)
    status = []
    for seed, step in SOURCE_ORDER:
        name = f"s{seed}_entry_{step:04d}"
        if (run_dir() / "teacher" / f"{name}.json").exists():
            continue
        if (run_dir() / "corrections" / name).exists() or (run_dir() / "teacher_plus_b" / name).exists():
            status.append(dict(id=name, status="NOT RUN AGAIN: a folder of an unfinished attempt exists; the single teacher attempt of this source is spent", unavailable=True))
            continue
        done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "teacher-one", "--seed", str(seed)], capture_output=True, text=True)
        (run_dir() / "logs" / f"teacher_{name}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        status.append(dict(id=name, returncode=done.returncode, **({} if done.returncode == 0 else dict(status="executor_error (not a teacher task result)", stderr_tail=done.stderr[-800:]))))
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(line[-1] if line and done.returncode == 0 else f"[executor_error] {name}: {done.stderr[-400:]}", flush=True)
    results = []
    for seed, step in SOURCE_ORDER:
        f = run_dir() / "teacher" / f"s{seed}_entry_{step:04d}.json"
        results.append(load(f) if f.exists() else dict(id=f"s{seed}_entry_{step:04d}", seed=seed, source_step=step, unavailable=True, receiving=dict(teacher_ready_valid=False, failure_class="unavailable", reason="no teacher result file")))
    data = dict(run_id=RUN, attempts=len(SOURCE_ORDER), execution_status=status, results=results, receiving_valid=[r["id"] for r in results if r["receiving"]["teacher_ready_valid"]],
                teacher_ready_valid=sum(bool(r["receiving"]["teacher_ready_valid"]) for r in results),
                teacher_receive_plus_frozen_B_valid=sum(bool((r.get("mixed") or {}).get("teacher_receive_plus_frozen_B_valid")) for r in results),
                note="teacher readiness is neither a student success nor a handover success; the mixed result is START_R1_E prefix + teacher RECEIVE + student RELEASE_B")
    target.write_text(json.dumps(data, indent=2, default=str))
    print(json.dumps({k: data[k] for k in ("receiving_valid", "teacher_ready_valid", "teacher_receive_plus_frozen_B_valid")}))
    return data


def new_entries():
    out = []
    for r in load(run_dir() / "teacher_collection.json")["results"]:
        if not r["receiving"]["teacher_ready_valid"]:
            continue
        for file, digest in r["files"].items():
            if sha(Path(r["path"]) / file) != digest:
                raise ValueError(f"correction file changed: {r['path']}/{file}")
        out.append(dict(id=r["id"], seed=r["seed"], path=r["path"], origin="actual START_R1_E hand-over state, this run", source_step=r["source_step"], suffix_transitions=r["transitions"],
                        receive=r["receiving"]["receive"], receive_abs_s=r["receiving"]["receive_abs_s"], files=r["files"]))
    return out


# ----------------------------------------------------------------------------------------------
# Views, plan, training
# ----------------------------------------------------------------------------------------------

def datasets():
    st, replay, train_view, old_view, new_view, onset_view = r3.datasets()
    entries = new_entries()
    bridge_view = rc.segment_view(st, run_dir(), r2.correction_segments(entries, NEW_BASE)) if entries else None
    views = dict(train=train_view, existing=old_view, actual_entry=new_view, onset=onset_view, bridge=bridge_view)
    index = {}
    for name, view in views.items():
        if view is None:
            continue
        total = 0
        for key, ep in view.episodes.items():
            if key in index:
                raise RuntimeError(f"view key {key} appears in two views")
            index[key] = (name, total, len(ep["action"]))
            total += len(ep["action"])
    return st, views, index


def build_plan(index, bridge_keys):
    with np.load(E_TRAINING / "consumed_samples.npz", allow_pickle=False) as a:
        ek, et = a["episode"].copy(), a["timestep"].copy()
    if ek.shape != (UPDATES, BATCH) or any(int(k) not in index for k in np.unique(ek)):
        raise RuntimeError("the stored RECEIVE_FROZEN consumption does not match the rebuilt views")
    rng = np.random.default_rng(SEED)
    order = rng.permuted(np.tile(np.arange(BATCH), (UPDATES, 1)), axis=1)                      # which replay sample sits in which slot: fixed before training, the same for both
    ck, ct = np.take_along_axis(ek, order, 1), np.take_along_axis(et, order, 1)
    count = UPDATES * (BATCH - COMMON)
    pool = np.array(bridge_keys)
    blocks = -(-count // len(pool))
    nk = rng.permuted(np.tile(pool, (blocks, 1)), axis=1).reshape(-1)[:count]
    nt = np.floor(rng.random(count) * np.array([index[int(k)][2] for k in nk])).astype(np.int64)
    ak, at = ck.copy(), ct.copy()
    ak[:, COMMON:], at[:, COMMON:] = nk.reshape(UPDATES, -1), nt.reshape(UPDATES, -1)
    return dict(replay_slot_order=order, ctrl_key=ck, ctrl_t=ct, adapt_key=ak, adapt_t=at)


def exposure(index, keys, t):
    names = dict(train="Train69 receiving view", existing="earlier corrections (re-cut)", actual_entry="R2 actual-entry corrections", onset="R3 onset correction", bridge="new START_R1_E hand-over corrections")
    per_view, per_key, valid = {}, {}, 0
    for k, x in zip(keys.ravel(), t.ravel()):
        name, _, length = index[int(k)]
        per_view[names[name]] = per_view.get(names[name], 0) + 1
        per_key[int(k)] = per_key.get(int(k), 0) + 1
        valid += min(50, length - int(x))
    return dict(samples=int(keys.size), valid_future_targets=int(valid), per_view=per_view, bridge_per_source={str(k): v for k, v in sorted(per_key.items()) if k >= NEW_BASE},
                distinct_observations=int(len({(int(a), int(b)) for a, b in zip(keys.ravel(), t.ravel())})))


def plan(args=None):
    target = run_dir() / "plan.json"
    if target.exists():
        return load(target)
    entries = new_entries()
    if not entries:
        target.write_text(json.dumps(dict(run_id=RUN, status="NO_VALID_CORRECTION: the CTRL / ADAPT training is not started", valid_corrections=0), indent=2))
        return load(target)
    st, views, index = datasets()
    bridge = views["bridge"]
    keys = [r["view_key"] for r in bridge.rows]
    sp = build_plan(index, keys)
    std, arm = bridge.stats["action"]["std"], list(rc.ARM)
    off = 0
    for key in keys:                                                        # mask / anchor / label checks on each new prefix
        ep, n = bridge.episodes[key], len(bridge.episodes[key]["action"])
        for t in (0, max(n - 60, 0), n - 1):
            item = bridge[off + t]
            pad = item["action_is_pad"].numpy()
            assert pad.sum() == max(0, t + 50 - n) and (not pad.any() or pad[-1]) and int(item["meta_episode"]) == key and int(item["meta_t"]) == t
            j = min(3, n - 1 - t)
            assert np.allclose(item["action"][j].numpy()[arm], (ep["action"][t + j][arm] - ep["state"][t][arm]) / std[arm], atol=1e-6)
        off += n
    assert np.array_equal(sp["ctrl_key"][:, :COMMON], sp["adapt_key"][:, :COMMON]) and np.array_equal(sp["ctrl_t"][:, :COMMON], sp["adapt_t"][:, :COMMON])
    assert (sp["adapt_key"][:, COMMON:] >= NEW_BASE).all() and (sp["ctrl_key"] < NEW_BASE).all()
    assert np.array_equal(np.sort(sp["ctrl_key"], 1), np.sort(np.load(E_TRAINING / "consumed_samples.npz")["episode"], 1))
    np.savez_compressed(run_dir() / "sample_plan.npz", **sp)
    record = dict(run_id=RUN, fixed_before_training=time.strftime("%Y-%m-%dT%H:%M:%S%z"), valid_corrections=len(entries),
                  bridge_view=dict(sha256=r2.view_sha(bridge), rows=[dict(view_key=r["view_key"], id=r["id"], seed=r["seed"], source_step=r["source_step"], receive=r["receive"],
                                                                         receive_abs_s=r["receive_abs_s"], observations=r["receive"][1] - r["receive"][0], path=r["path"]) for r in bridge.rows],
                                   observations=len(bridge)),
                  replay_views={k: r2.view_sha(v) for k, v in views.items() if k != "bridge"}, sample_plan=dict(file=str(run_dir() / "sample_plan.npz"), sha256=sha(run_dir() / "sample_plan.npz")),
                  ctrl=exposure(index, sp["ctrl_key"], sp["ctrl_t"]), adapt=exposure(index, sp["adapt_key"], sp["adapt_t"]),
                  common_slots=exposure(index, sp["ctrl_key"][:, :COMMON], sp["ctrl_t"][:, :COMMON]))
    target.write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(valid=len(entries), bridge_observations=len(bridge), ctrl=record["ctrl"]["per_view"], adapt=record["adapt"]["per_view"], per_source=record["adapt"]["bridge_per_source"])))
    return record


def trained(condition):
    out = training_dir(condition)
    try:
        run, pl = load(out / "run.json"), load(run_dir() / "plan.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (run.get("run_id") == RUN and run.get("steps") == UPDATES and run.get("consumed_equals_plan") and run.get("sample_plan_sha256") == pl.get("sample_plan", {}).get("sha256")
          and run.get("warm_start_model_sha256") == EXPECTED["RECEIVE_FROZEN"] and run.get("execution_source_sha256") == sha(__file__)
          and (out / CHECKPOINT / "model.safetensors").exists() and run.get("model_sha256") == sha(out / CHECKPOINT / "model.safetensors"))
    return run if ok else None


def train(args):
    import warnings
    import arm_output_trial as trial
    import single_demo_trial as sd
    import torch
    from torch.utils.data import default_collate
    from dapier_act_policy import ACTPolicy
    condition = args.condition
    if trained(condition) is not None:
        print(f"[stored training of this plan] {training_dir(condition)}")
        return
    pl = load(run_dir() / "plan.json")
    if not pl.get("valid_corrections"):
        raise SystemExit("no valid correction: nothing is trained")
    out = training_dir(condition)
    if out.exists():
        kept = out.with_name(out.name + time.strftime(".unfinished-%Y%m%dT%H%M%S"))
        out.rename(kept)
        print(f"[unfinished attempt kept] {kept}")
    load_report = trial.gpu_other_load()
    trial.apply_reproducibility_settings()
    before = frozen()
    st, views, index = datasets()
    if sha(run_dir() / "sample_plan.npz") != pl["sample_plan"]["sha256"] or r2.view_sha(views["bridge"]) != pl["bridge_view"]["sha256"]:
        raise RuntimeError("sample plan or the new correction view changed since the plan")
    with np.load(run_dir() / "sample_plan.npz", allow_pickle=False) as a:
        keys, ts = a[f"{condition}_key"].copy(), a[f"{condition}_t"].copy()
    source = MODELS["RECEIVE_FROZEN"]
    policy = ACTPolicy.from_pretrained(source, local_files_only=True, strict=True).cuda()
    policy.check_normalization(st)
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=1e-4, weight_decay=1e-4)
    state = torch.load(source / "training_state.pt", map_location="cpu", weights_only=False)
    optimizer.load_state_dict(state["optimizer"])
    groups = [dict(lr=g["lr"], weight_decay=g["weight_decay"], tensors=len(g["params"])) for g in optimizer.param_groups]
    adam = lambda: sorted({int(v["step"]) for v in optimizer.state_dict()["state"].values() if "step" in v})
    adam_at_load = adam()
    with np.load(rc.HORIZON_WEIGHTS, allow_pickle=False) as a:
        r = torch.tensor(a["r"], dtype=torch.float32, device="cuda")
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(source.parent / "normalization.npz", out / "normalization.npz")
    consumed_e = np.full((UPDATES, BATCH), -1, dtype=np.int64)
    consumed_t = np.full((UPDATES, BATCH), -1, dtype=np.int64)
    valid_targets = 0
    torch.manual_seed(SEED)
    started_wall, started = time.strftime("%Y-%m-%dT%H:%M:%S%z"), time.monotonic()
    with (out / "metrics.jsonl").open("x") as log, warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        for step in range(1, UPDATES + 1):
            u = step - 1
            items = []
            for k, t in zip(keys[u], ts[u]):
                name, offset, _ = index[int(k)]
                items.append(views[name][offset + int(t)])
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
        torch.save(dict(optimizer=optimizer.state_dict(), step=UPDATES, inherited_step=int(state["step"]), cpu_rng=torch.get_rng_state(), cuda_rng=torch.cuda.get_rng_state()), name / "training_state.pt")
    seconds = round(time.monotonic() - started, 2)
    if not (np.array_equal(consumed_e, keys) and np.array_equal(consumed_t, ts)):
        raise RuntimeError("consumed samples differ from the plan")
    np.savez_compressed(out / "consumed_samples.npz", episode=consumed_e, timestep=consumed_t)
    run = dict(kind="DAPIER ACT RECEIVE-BRIDGE R1 CTRL / ADAPT (SIM only)", run_id=RUN, condition=condition, condition_name=NAMES[condition], policy_variant=policy.config.type,
               dataset=str(td.DATASET), train_episodes=[int(x["manifest_index"]) for x in views["train"].rows], holdout_episodes=[], contract_revision=trial.load_manifest()["contract_revision"],
               control_dt=td.DT, hardware_access=False, task_success=None, seed=SEED, batch_size=BATCH, steps=UPDATES, warm_start=str(source), warm_start_model_sha256=EXPECTED["RECEIVE_FROZEN"],
               optimizer_state_loaded=True, optimizer_state_sha256=sha(source / "training_state.pt"), optimizer_step_at_load=int(state["step"]), adam_step_counters_at_load=adam_at_load,
               adam_step_counters_at_end=adam(), optimizer_param_groups=groups, normalization_sha256=sha(out / "normalization.npz"), sample_plan_sha256=pl["sample_plan"]["sha256"],
               bridge_view_sha256=pl["bridge_view"]["sha256"], consumed_sha256=trial.array_sha(consumed_e, consumed_t), consumed_equals_plan=True, samples=int(consumed_e.size),
               valid_action_targets=valid_targets, exposure=exposure(index, consumed_e, consumed_t), precision="float32, no autocast", model_sha256=sha(name / "model.safetensors"),
               config_sha256=sha(name / "config.json"), execution_source_sha256=sha(__file__), frozen_before=before, frozen_unchanged_after=frozen() == before, started=started_wall,
               training_seconds=seconds, environment=trial.environment_record(), host=s0.host(), gpu_other_load_at_start=load_report,
               deterministic_warnings=sorted({str(w.message).split("\n")[0][:200] for w in caught}))
    (out / "run.json").write_text(json.dumps(run, indent=2, default=str))
    print(json.dumps(dict(condition=condition, seconds=seconds, valid_action_targets=valid_targets, per_view=run["exposure"]["per_view"], model=run["model_sha256"][:12])))


# ----------------------------------------------------------------------------------------------
# Student cases
# ----------------------------------------------------------------------------------------------

def model_paths():
    return dict(START=dict(FROZEN=MODELS["START_FROZEN"], R1E=MODELS["START_R1_E"]), RECEIVE=dict(FROZEN=MODELS["RECEIVE_FROZEN"], CTRL=checkpoint("ctrl"), ADAPT=checkpoint("adapt")))


def identity():
    import arm_output_trial as trial
    models = frozen()
    for c in ("ctrl", "adapt"):
        run = trained(c)
        if run is None:
            raise RuntimeError(f"training {c} of this plan is not complete")
        models[NAMES[c]] = dict(path=str(checkpoint(c)), model=run["model_sha256"], config=sha(checkpoint(c) / "config.json"), normalization=sha(training_dir(c) / "normalization.npz"),
                                training_run_sha256=sha(training_dir(c) / "run.json"))
    ps = p1.parents()
    resets = {}
    for split in ps:
        for seed, p in ps[split].items():
            meta = load(Path(d1.manifest()[p["manifest_index"]]["path"]) / "metadata.json")
            resets[str(seed)] = hashlib.sha256(json.dumps([meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"]], sort_keys=True).encode()).hexdigest()
    body = dict(run_id=RUN, models=models, manifest_sha256=p1.MANIFEST_SHA, cases=[list(c) for c in cases()], resets=resets, sources={p: sha(FACTORY_ROOT / p) for p in SOURCES},
                contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"], sample_plan_sha256=load(run_dir() / "plan.json")["sample_plan"]["sha256"],
                execution=dict(chain="START -> RECEIVE -> RELEASE_B", chunk=50, prefix=rc.PREFIX, control_hz=round(1 / td.DT), arm_bound_policy="project", ensemble="off", smoothing="off",
                               horizon_steps=rc.HORIZON, torch_seed=4100, policy_inputs=["left / right wrist RGB", "measured q_state"]),
                backend=trial.environment_record(), host=s0.host(), environment={k: os.environ.get(k) for k in rk.ENV_KEYS})
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def case_dir(ident, case):
    group, split, seed, start, receive = case
    return run_dir() / "runs" / ident["sha256"][:12] / group / split / f"s{seed}" / f"START_{start}__RECEIVE_{receive}"


def complete(ident, case):
    out = case_dir(ident, case)
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (r.get("identity_sha256") == ident["sha256"] and r.get("case") == list(case) and r.get("job") == RUN and meta.get("complete") and r.get("boundary_contract") == "repaired"
          and "child_settings" in r and all((out / f).exists() for f in ("episode.npz", "action_trace.json", "task_check_rows.json", "predicted_chunks.npz")))
    return r if ok else None


def handover_frame(model, state):
    """Block pose in the left tool frame and the arm state at one stored integration state (analysis only)."""
    import mujoco
    import live_release_check as lr
    from sim_data_factory import core
    from sim_data_factory.task_success import ContactModel
    d = mujoco.MjData(model)
    mujoco.mj_setState(model, d, state, int(mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_forward(model, d)
    out = lr.frames(model, d, ContactModel(model))
    site, block = model.site("left_cube_grasp").id, model.body("red_block").id
    rot = d.site_xmat[site].reshape(3, 3).T @ d.xmat[block].reshape(3, 3)
    angle = float(np.degrees(np.arccos(np.clip((np.trace(rot) - 1) / 2, -1, 1))))
    out.update(block_rot_in_left_site=rot.round(4).tolist(), block_rot_angle_from_site_axes_deg=round(angle, 2), block_world_pos_m=d.xpos[block].round(4).tolist(),
               qvel_abs_max=float(np.abs(d.qvel).max()), q_state=core.encode_state(model, d).round(5).tolist())
    return out


def one(args):
    from integration_scenes import task_env
    from sim_data_factory.task_success import evaluate_states
    ident = identity()
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    case = (args.group, args.split, args.seed, args.start, args.receive)
    if list(case) not in ident["cases"]:
        raise ValueError(f"{case} is not one of the 44 fixed cases")
    parent = p1.parents()[args.split][args.seed]
    out = case_dir(ident, case)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    paths = model_paths()
    rc.MODELS["START"], rc.MODELS["RECEIVE"] = paths["START"][args.start], paths["RECEIVE"][args.receive]
    loaded, original = {}, rc.load_policy

    def remember(name):
        got = original(name)
        loaded[name] = got
        return got

    rc.load_policy = remember
    started = time.time()
    chain = f"START_{args.start} -> RECEIVE_{args.receive} -> RELEASE_B"
    try:
        result = rc.execute(out, args.seed, ("START", "RECEIVE", "B"), None, f"{LABEL}: {args.group} {args.split} parent {args.seed} {chain}", contract="repaired")
    finally:
        rc.load_policy = original
    names = dict(FROZEN_S="START_FROZEN", R1E="START_R1_E", FROZEN="RECEIVE_FROZEN", CTRL=NAMES["ctrl"], ADAPT=NAMES["adapt"])
    want = dict(START=ident["models"][names["FROZEN_S" if args.start == "FROZEN" else "R1E"]]["model"], RECEIVE=ident["models"][names[args.receive]]["model"], B=EXPECTED["RELEASE_B"])
    got = {k: v[2]["model_sha256"] for k, v in loaded.items()}
    if any(got[k] != want[k] for k in got):
        raise RuntimeError(f"a loaded policy is not the intended one: {got}")
    rows = load(out / "task_check_rows.json")
    trace = load(out / "action_trace.json")
    with np.load(out / "episode.npz", allow_pickle=False) as a:
        states, q = a["integration_state"].copy(), a["q_state"].copy()
    rgb = {s: np.load(out / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}
    model = task_env("desk", grippers="both").model

    def moment(step):
        if step is None or step >= len(rgb["left"]):
            return None
        return dict(step=int(step), sim_time_s=round(float(states[step][0]), 2), state_sha256=array_sha(states[step]), q_sha256=array_sha(q[step]),
                    rgb_sha256=dict(left=array_sha(rgb["left"][step]), right=array_sha(rgb["right"][step])))

    entry = next((s for s in result["switches"] if s["to"] == "RECEIVE"), None)
    gate = next((s for s in result["switches"] if s["to"] == "B"), None)
    success, fault, events = result["task_success"], result["first_fault"], result["events"]
    stage = None if success else "START" if entry is None else "RECEIVE" if gate is None else "RELEASE_B"
    kind = None if success else ("fault" if fault is not None else "task not completed within the 40 s horizon")
    entry_valid = None
    if entry is not None:
        n = entry["step"] + 1
        chk = evaluate_states(model, states[:n], q[:n].astype(np.float64), control_dt=td.DT)
        entry_valid = dict(valid=bool(chk["valid"]), failures=chk["failures"][:3], events={k: round(v, 2) for k, v in chk["events"].items()})
    upto = len(states) if entry is None else entry["step"] + 1                # what happened before RECEIVE took over: comparable across the two RECEIVE models
    fault_t = None if fault is None else fault.get("time")
    result.update(job=RUN, kind="full", classification=LABEL, group=args.group, split="train" if args.split == "train" else "val (development)", case=list(case), parent=parent,
                  start_model=args.start, receive_model=args.receive, identity_sha256=ident["sha256"], chain=chain, loaded_models=got,
                  handover_event=result["handover_s"] is not None, completed_40s=result["termination"] == "horizon_completed", first_failure_stage=stage, first_failure_kind=kind,
                  events_before_fault=None if fault_t is None else sorted(k for k, v in events.items() if v <= fault_t),
                  receive_entry_valid_before=entry_valid, remaining_after_receive_entry_s=None if entry is None else round(rc.HORIZON * td.DT - entry["sim_time_s"], 2),
                  pre_receive=dict(states=int(upto), q_sha256=array_sha(q[:upto]), state_sha256=array_sha(states[:upto]), pick_lift_s=events.get("pick_lift"),
                                   receive_entry_step=None if entry is None else entry["step"]),
                  handover_state_at_receive_entry=None if entry is None else handover_frame(model, states[entry["step"]]),
                  transfer=None if entry is None else r3.arm_motion(out, entry["step"]), block_contact=p1.contact_summary(rows, result),
                  counts=dict(commands_dispatched=sum(t.get("ctrl_applied") is not None for t in trace), completed_transitions=result["completed_transitions"],
                              sim_elapsed_s=result["last_completed_state_sim_s"], wall_s=round(time.time() - started, 1)),
                  moments=dict(reset=moment(0), receive_entry=moment(entry["step"] if entry else None), release_b_entry=moment(gate["step"] if gate else None)),
                  contract_checks=rb.contract_checks(result), phases=r2.phase_details(result, rows, result["receive_entry_s"] or 0.0),
                  child_settings=dict(rk.child_settings({dict(START="START_" + args.start, RECEIVE="RECEIVE_" + args.receive, B="RELEASE_B")[k]: v[0] for k, v in loaded.items()}), host=s0.host(),
                                      float32_matmul_precision=__import__("torch").get_float32_matmul_precision(), autocast=False))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(case=list(case), events={k: round(v, 2) for k, v in events.items()}, receive_entry=result["receive_entry_s"], ready=result["phases"]["semantic_ready_s"],
                          release_b=result["gate_s"], termination=result["termination"], first_failure_stage=stage, first_failure_kind=kind, task_success=success,
                          final_support=result["final_support"]), default=str))


def run(args=None):
    ident = identity()
    (run_dir() / "logs").mkdir(exist_ok=True)
    if load(run_dir() / "protocol.json")["evaluation"]["cases"] != ident["cases"]:
        raise RuntimeError("the 44 cases differ from the list fixed before the teacher runs")
    target = run_dir() / "evaluation_identity.json"
    if target.exists():
        if load(target)["sha256"] != ident["sha256"]:
            raise RuntimeError("evaluation_identity.json holds another identity (model, plan, execution source, contract or host changed): nothing is run, nothing is overwritten")
        print(f"[resumed evaluation] identity {ident['sha256'][:12]}; completed cases are not run again and are not new runs")
    else:
        target.write_text(json.dumps(dict(ident, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z")), indent=2, default=str))
    attempts_file = run_dir() / "attempts.json"
    attempts = load(attempts_file) if attempts_file.exists() else []
    for case in cases():
        out = case_dir(ident, case)
        tag = "_".join(str(x) for x in case)
        if complete(ident, case) is not None:
            continue
        if out.exists():
            kept = run_dir() / "attempts" / f"{tag}_{time.strftime('%Y%m%dT%H%M%S')}"
            kept.parent.mkdir(exist_ok=True)
            out.rename(kept)
            attempts.append(dict(case=list(case), kept=str(kept), note="incomplete folder of an earlier executor attempt"))
        done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--group", case[0], "--split", case[1], "--seed", str(case[2]), "--start", case[3], "--receive", case[4],
                               "--identity", ident["sha256"]], capture_output=True, text=True)
        n = sum(a["case"] == list(case) for a in attempts)
        (run_dir() / "logs" / f"{ident['sha256'][:12]}_{tag}{'' if not n else f'_retry{n}'}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        ok = done.returncode == 0 and complete(ident, case) is not None
        if not ok:
            attempts.append(dict(case=list(case), returncode=done.returncode, status="executor_error (not a task result)", stderr_tail=done.stderr[-800:]))
        attempts_file.write_text(json.dumps(attempts, indent=2))
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(line[-1][:600] if ok and line else f"[executor_error, not a task failure] {case}: returncode {done.returncode}: {done.stderr[-400:]}", flush=True)


def everything(args=None):
    me = [sys.executable, str(Path(__file__).resolve())]
    steps = [["protocol"], ["teacher"], ["plan"]]
    for step in steps:
        print(f"--- {' '.join(step)} ---", flush=True)
        if subprocess.run(me + step).returncode:
            raise SystemExit(f"step `{' '.join(step)}` failed; later steps are not started")
    if not load(run_dir() / "plan.json").get("valid_corrections"):
        print("no valid correction prefix: training and the student evaluation are not started")
    else:
        for step in (["train", "--condition", "ctrl"], ["train", "--condition", "adapt"], ["run"]):
            print(f"--- {' '.join(step)} ---", flush=True)
            if subprocess.run(me + step).returncode:
                raise SystemExit(f"step `{' '.join(step)}` failed; later steps are not started")
    report = Path(__file__).with_name("receive_bridge_r1_report.py")
    if report.exists():
        print("--- report ---", flush=True)
        subprocess.run([sys.executable, str(report), "all"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("protocol", "teacher", "plan", "run", "all"):
        sub.add_parser(name)
    t = sub.add_parser("teacher-one")
    t.add_argument("--seed", type=int, required=True, choices=[s for s, _ in SOURCE_ORDER])
    t = sub.add_parser("train")
    t.add_argument("--condition", choices=("ctrl", "adapt"), required=True)
    o = sub.add_parser("one")
    o.add_argument("--group", required=True)
    o.add_argument("--split", choices=("train", "val"), required=True)
    o.add_argument("--seed", type=int, required=True)
    o.add_argument("--start", choices=("FROZEN", "R1E"), required=True)
    o.add_argument("--receive", choices=("FROZEN", "CTRL", "ADAPT"), required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    return {"protocol": protocol, "teacher": teacher, "teacher-one": teacher_one, "plan": plan, "train": train, "run": run, "one": one, "all": everything}[args.command](args)


if __name__ == "__main__":
    main()
