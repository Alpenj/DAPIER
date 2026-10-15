#!/usr/bin/env python3
"""START-PICK R1 / work6: coverage replay control vs candidate (SIM / learning only; no hardware, no teacher run, no Test).

Two copies of the frozen START (three-parent step-020000, with its stored AdamW state) get 10,000 more updates each on the
pick stretch [0, T) of stored Train69 demonstrations, T = the unchanged START -> RECEIVE rule (first local C25 boundary at
or after the checker's pick_lift where the left hand holds):

  C = START_REPLAY_CONTROL        8 slots from focus 3 (5000 / 5002 / 5008)
  E = START_COVERAGE_CANDIDATE    2 slots = C's own first two (same parent and timestep), 6 slots from the other 66 parents

  prefix    T per parent recomputed with the existing checker from the SHA-verified states; prefix manifest
  protocol  sample plan (80,000 samples per condition), the 42 evaluation cases and their order, fixed before training
  train     --condition c | e
  offline   D1's analysis (1,347 C25-boundary teacher observations, eval mode, zero latent) for C and E
  run       22 Train + 20 development-Val original-reset runs of START_{C|E} -> RECEIVE_E -> RELEASE_B, one new process each
  video     representative C / E pairs from the scheduled runs
  summary   this run's own tables (used by `act-sim results`)
  all       every step that is not complete yet, in this order

Switches read SIM ground truth: SIM-ORACLE-GATED THREE-POLICY. Never merged with S0, D1 or earlier laptop runs.
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
import server_s0 as s0   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "start-pick-r1-work6-20261006-01"
TITLE = "START-PICK R1 / work6 / coverage replay control vs candidate"
LABEL = "SIM-ORACLE-GATED THREE-POLICY / START-PICK R1 work6"
START_TRAINING = rc.START_TRAINING
START = START_TRAINING / rc.START_CHECKPOINT
START_SHA = rk.EXPECTED["START"]
FROZEN = dict(RECEIVE_E=rk.MODELS["E"], RELEASE_B=rk.MODELS["RELEASE_B"])
FROZEN_SHA = dict(RECEIVE_E=rk.EXPECTED["E"], RELEASE_B=rk.EXPECTED["RELEASE_B"])
MANIFEST_SHA = "e36be80d36a2df8e38117a41eac60a7bbe29751cafea94996e7f78d0b88aa0bf"
NAMES = dict(c="START_REPLAY_CONTROL", e="START_COVERAGE_CANDIDATE")
FOCUS = d1.FOCUS
TRAIN9 = (5000, 5002, 5008, 5011, 5042, 5098, 5101, 5151, 5199)
VAL10 = (5001, 5046, 5072, 5079, 5081, 5094, 5134, 5150, 5153, 5163)
UPDATES, BATCH, SEED, COMMON = 10000, 8, 4100, 2
CHECKPOINT, MIDWAY = "step-010000", 5000
K, CHUNK = rc.PREFIX, 50
SOURCES = d1.SOURCES                                   # the executor and its helpers; this file's own SHA is recorded apart
EVENTS = ("reach", "left_grasp", "pick_lift", "right_grasp", "handover")
STAGES = ("START", "RECEIVE", "RELEASE_B")

load, sha, array_sha = rk.load, rk.sha, rk.array_sha


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def training_dir(condition):
    return FACTORY_ROOT / "training" / f"{RUN}-{condition.lower()}"


def checkpoint(condition):
    return training_dir(condition) / CHECKPOINT


def cases():
    """42 cases in the fixed order: Train9 (odd position C->E, even E->C), then 5000 E->C and 5008 C->E, then Val10."""
    out = []
    for split, seeds in (("train", TRAIN9), ("val", VAL10)):
        for n, seed in enumerate(seeds, 1):
            out += [(split, seed, m, 1) for m in (("C", "E") if n % 2 else ("E", "C"))]
        if split == "train":
            out += [("train", 5000, "E", 2), ("train", 5000, "C", 2), ("train", 5008, "C", 2), ("train", 5008, "E", 2)]
    return out


# ----------------------------------------------------------------------------------------------
# Pick stretch [0, T) of Train69
# ----------------------------------------------------------------------------------------------

def replay_set():
    import arm_output_trial as trial
    st = rc.stats()
    episodes = trial.load_manifest()["episodes"]
    ids = [i for i, e in enumerate(episodes) if e["split"] == "train" and e["training_eligible"]]
    return st, trial.TrialEpisodes(td.DATASET, ids, required_split="train", stats=st, arm_target="measured_relative")


def prefix(args=None):
    """T from the existing checker on the stored states (episode.npz is SHA-verified by the loader), next to the T that the
    stored task_check_rows.json / metadata.json give (those two files have no pre-deletion SHA record)."""
    from integration_scenes import task_env
    from sim_data_factory.task_success import evaluate_states
    target = run_dir() / "prefix_manifest.json"
    if target.exists():
        return load(target)
    run_dir().mkdir(parents=True, exist_ok=True)
    if sha(td.DATASET / "manifest.json") != MANIFEST_SHA:
        raise RuntimeError("the frozen manifest differs from the instructed SHA-256")
    st, replay = replay_set()
    model = task_env("desk", grippers="both").model
    rows_out, missing = [], []
    for row in replay.rows:
        path, key = Path(row["path"]), row["manifest_index"]
        with np.load(path / "episode.npz", allow_pickle=False) as a:
            states, q = a["integration_state"].copy(), a["q_state"].astype(np.float64)
        check = evaluate_states(model, states, q, control_dt=td.DT)
        rows = check.pop("rows")
        T, _, why = rb.semantic_bounds(rows, check["events"])
        stored_T = rb.semantic_bounds(load(path / "task_check_rows.json"), load(path / "metadata.json")["task_check"]["events"])[0]
        phases = [d.get("phase") for d in load(path / "diagnostics.json")]
        entry = dict(view_key=key, manifest_index=key, seed=row["seed"], split=row["split"], generation=row["generation"], parent_physics_group=row.get("parent_physics_group"),
                     pool="focus3" if row["seed"] in FOCUS else "other66", transitions=len(replay.episodes[key]["action"]), T=T, T_from_stored_rows_file=stored_T,
                     T_agrees=T == stored_T, recomputed_checker_valid=bool(check["valid"]), pick_lift_s=check["events"].get("pick_lift"),
                     episode_files_sha256_verified=row.get("file_sha256_verified"))
        if T is None or T <= 0:
            missing.append(dict(entry, reason=why or "T is not positive"))
            continue
        t = np.arange(T)
        entry.update(T_s=round(T * td.DT, 2), observations=int(T), valid_future_targets=int(np.minimum(CHUNK, T - t).sum()), left_holds_at_T=bool(rb.hands(rows[T])[0]),
                     pick_lift_before_T=bool(rows[T]["time"] >= check["events"]["pick_lift"] - 1e-9), boundary="first local C25 boundary (index % 25 == 0) at or after the checker's pick_lift "
                     "where both left pads >= grip_force_min_N and the table gives no support (rb.semantic_bounds start; the rule of the START -> RECEIVE switch)",
                     phase_observations={p: int(sum(x == p for x in phases[:T])) for p in dict.fromkeys(phases[:T])})
        rows_out.append(entry)
    data = dict(run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), dataset=str(td.DATASET), manifest_sha256=MANIFEST_SHA, parents=len(rows_out), missing=missing,
                evidence="T: existing checker (task_success.evaluate_states) re-run here on integration_state / q_state of episode.npz, whose SHA-256 the loader checks against the manifest. "
                         "T_from_stored_rows_file: the same rule on the stored task_check_rows.json + metadata.json events (no pre-deletion SHA record; shown for agreement only). "
                         "phase names come from diagnostics.json (no pre-deletion SHA record) and are used only to tabulate exposure, never to cut or label.",
                all_T_agree=all(r["T_agrees"] for r in rows_out), observations=int(sum(r["observations"] for r in rows_out)),
                valid_future_targets=int(sum(r["valid_future_targets"] for r in rows_out)), rows=rows_out)
    target.write_text(json.dumps(data, indent=2, default=str))
    print(json.dumps({k: data[k] for k in ("parents", "missing", "all_T_agree", "observations", "valid_future_targets")}, default=str))
    return data


def pick_view(st, replay, table):
    segments = [(r["view_key"], replay.episodes[r["view_key"]], (0, r["T"]), dict(manifest_index=r["manifest_index"], seed=r["seed"], split=r["split"], pool=r["pool"])) for r in table["rows"]]
    view = rc.segment_view(st, td.DATASET, segments)
    offset, total = {}, 0
    for key, ep in view.episodes.items():
        offset[key], total = total, total + len(ep["action"])
    return view, offset


def build_plan(table):
    rng = np.random.default_rng(SEED)
    T = {r["view_key"]: r["T"] for r in table["rows"]}
    pools = {p: np.array([r["view_key"] for r in table["rows"] if r["pool"] == p]) for p in ("focus3", "other66")}

    def draw(pool, count):
        blocks = -(-count // len(pool))
        keys = rng.permuted(np.tile(pool, (blocks, 1)), axis=1).reshape(-1)[:count]          # fixed-RNG permutations of the pool, joined: equal exposure per parent
        t = np.floor(rng.random(count) * np.array([T[k] for k in keys])).astype(np.int64)     # uniform over the parent's own [0, T)
        return keys, t

    out = {}
    for name, pool, width in (("common", "focus3", COMMON), ("c", "focus3", BATCH - COMMON), ("e", "other66", BATCH - COMMON)):
        keys, t = draw(pools[pool], UPDATES * width)
        out[f"{name}_key"], out[f"{name}_t"] = keys.reshape(UPDATES, width), t.reshape(UPDATES, width)
    return out


def plan_rows(plan, condition):
    return np.concatenate([plan["common_key"], plan[f"{condition}_key"]], axis=1), np.concatenate([plan["common_t"], plan[f"{condition}_t"]], axis=1)


def exposure(table, keys, t):
    """Per pool / parent / phase exposure and the number of future targets left after the padding mask."""
    rows = {r["view_key"]: r for r in table["rows"]}
    episodes = d1.manifest()
    phases = {int(k): [d.get("phase") for d in load(Path(episodes[int(k)]["path"]) / "diagnostics.json")] for k in np.unique(keys)}
    per_parent, per_phase, per_pool = {}, {}, {}
    valid = 0
    for k, x in zip(keys.ravel(), t.ravel()):
        r = rows[int(k)]
        n = min(CHUNK, r["T"] - int(x))
        valid += n
        per_parent[r["seed"]] = per_parent.get(r["seed"], 0) + 1
        per_pool.setdefault(r["pool"], dict(samples=0, valid_future_targets=0))
        per_pool[r["pool"]]["samples"] += 1
        per_pool[r["pool"]]["valid_future_targets"] += n
        key = f"{r['pool']}/{phases[int(k)][int(x)]}"
        per_phase[key] = per_phase.get(key, 0) + 1
    counts = np.array(list(per_parent.values()))
    return dict(samples=int(keys.size), valid_future_targets=int(valid), masked_future_targets=int(keys.size * CHUNK - valid), per_pool=per_pool, parents_seen=len(per_parent),
                per_parent_min=int(counts.min()), per_parent_max=int(counts.max()), per_parent={str(k): v for k, v in sorted(per_parent.items())}, per_pool_phase=dict(sorted(per_phase.items())),
                distinct_observations=int(len({(int(a), int(b)) for a, b in zip(keys.ravel(), t.ravel())})))


def start_identity():
    got = dict(model=sha(START / "model.safetensors"), config=sha(START / "config.json"), normalization=sha(START_TRAINING / "normalization.npz"), optimizer_state=sha(START / "training_state.pt"))
    if got["model"] != START_SHA or got["normalization"] != rk.NORMALIZATION:
        raise RuntimeError(f"START_FROZEN or its normalization differs from the instructed SHA-256: {got}")
    frozen = {k: dict(path=str(p), model=sha(p / "model.safetensors"), config=sha(p / "config.json"), normalization=sha(p.parent / "normalization.npz")) for k, p in FROZEN.items()}
    if any(frozen[k]["model"] != FROZEN_SHA[k] or frozen[k]["normalization"] != rk.NORMALIZATION for k in FROZEN):
        raise RuntimeError("RECEIVE_E / RELEASE_B differ from the instructed SHA-256")
    return dict(path=str(START), **got), frozen


def protocol(args=None):
    import torch
    import arm_output_trial as trial
    target = run_dir() / "protocol.json"
    if target.exists():
        return load(target)
    table = prefix()
    start, frozen = start_identity()
    st, replay = replay_set()
    view, offset = pick_view(st, replay, table)
    plan = build_plan(table)
    for key in (table["rows"][0]["view_key"], table["rows"][-1]["view_key"]):                # mask / anchor / label spot checks on the view itself
        ep, full, T = view.episodes[key], replay.episodes[key], len(view.episodes[key]["action"])
        assert np.shares_memory(ep["action"], full["action"]) and np.shares_memory(ep["rgb"]["left"], full["rgb"]["left"])
        std, arm = view.stats["action"]["std"], list(rc.ARM)
        for t in (0, max(T - 60, 0), T - 1):
            item = view[offset[key] + t]
            pad = item["action_is_pad"].numpy()
            assert pad.sum() == max(0, t + CHUNK - T) and (not pad.any() or pad[-1]) and int(item["meta_episode"]) == key and int(item["meta_t"]) == t
            j = min(3, T - 1 - t)
            assert np.allclose(item["action"][j].numpy()[arm], (full["action"][t + j][arm] - full["state"][t][arm]) / std[arm], atol=1e-6)      # q_meas[t] anchor, command of [0, T) only
    np.savez_compressed(run_dir() / "sample_plan.npz", **plan)
    state = torch.load(START / "training_state.pt", map_location="cpu", weights_only=False)
    start_run = load(START_TRAINING / "run.json")
    config = load(START / "config.json")
    ck, ct = plan_rows(plan, "c")
    ek, et = plan_rows(plan, "e")
    record = dict(
        run_id=RUN, name=TITLE, label=LABEL, fixed_before_training=time.strftime("%Y-%m-%dT%H:%M:%S%z"), hardware_access=False, teacher_collection=False, test_split_used=False,
        host=s0.host(), environment=dict(python=sys.executable, backend=trial.environment_record(), env={k: os.environ.get(k) for k in rk.ENV_KEYS}),
        tool=dict(file="factory/v1/start_pick_r1.py", sha256_at_protocol=sha(__file__), note="recorded, not part of the resume identity: later fixes of tables / videos do not turn stored runs into other runs"),
        start=dict(**start, optimizer_keys=sorted(state), optimizer_step_in_file=int(state["step"]), optimizer_param_groups=len(state["optimizer"]["param_groups"]),
                   optimizer_state_entries=len(state["optimizer"]["state"]), lineage_steps_of_its_own_run=start_run.get("steps"),
                   use="both conditions: all START_FROZEN weights and this stored AdamW state; nothing re-initialised; torch.manual_seed(4100); START_FROZEN itself is read-only"),
        frozen_later_policies=frozen, manifest=dict(path=str(td.DATASET / "manifest.json"), sha256=MANIFEST_SHA),
        config_read_from_checkpoint={k: config.get(k) for k in ("type", "chunk_size", "n_action_steps", "kl_weight", "optimizer_lr", "optimizer_lr_backbone", "optimizer_weight_decay", "use_vae")},
        conditions=NAMES,
        prefix_view=dict(file=str(run_dir() / "prefix_manifest.json"), sha256=sha(run_dir() / "prefix_manifest.json"), view_sha256=r2.view_sha(view), parents=len(view.rows), observations=len(view),
                         missing=table["missing"], rule="[0, T) per parent; every 50 Hz timestep is a sample; targets at t + k >= T are padding; anchor = the parent's own q_meas[t]; grippers absolute"),
        sample_plan=dict(file=str(run_dir() / "sample_plan.npz"), sha256=sha(run_dir() / "sample_plan.npz"), updates=UPDATES, batch=BATCH, seed=SEED,
                         slots="0-1 common (focus 3, same parent and timestep in C and E); 2-7 compared: C focus 3, E the other 66",
                         c=exposure(table, ck, ct), e=exposure(table, ek, et), common=exposure(table, plan["common_key"], plan["common_t"]),
                         note="E is focus 3 at 25 % + other 66 at 75 %, not a uniform Train69 training; the design contrasts two training distributions and does not separate "
                              "'less focus exposure' from 'other-66 data added'"),
        unchanged=dict(normalization_sha256=rk.NORMALIZATION, horizon_weights_sha256=sha(rc.HORIZON_WEIGHTS), loss="B horizon weighting (single_demo_trial.loss_terms), KL as in the config",
                       optimizer="AdamW over policy.get_optim_params(), lr 1e-4 (backbone 1e-5), wd 1e-4, clip 1.0, state loaded", precision="float32, no autocast (as the earlier trainers)",
                       augmentation="none beyond TrialEpisodes.__getitem__ (as the earlier trainers)"),
        training=dict(updates_per_condition=UPDATES, final_checkpoint=CHECKPOINT, midway=f"step-{MIDWAY:06d} is kept for resumption / curves only, never evaluated or selected",
                      naming="step counts are this run's additional updates; the inherited AdamW step counter is recorded apart", no="extension by loss, best-checkpoint selection"),
        evaluation=dict(chain="START_{C|E} -> frozen RECEIVE_E -> frozen RELEASE_B", checkpoint=f"{CHECKPOINT} of each condition, fixed before training", contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                        cases=[list(c) for c in cases()], planned=len(cases()),
                        order="Train9 in the listed order (odd position C->E, even E->C), then 5000 E->C and 5008 C->E, then Val10 the same way; sequential, one new process per case",
                        rule="a task fault is that case's result and the next case runs; Val runs whatever the Train results; no repeat until success, no replacement; "
                             "an executor error keeps its attempt folder and the case is run again later",
                        val="development split used before; not an independent Test; Val teacher actions are not read for training or normalization; Test is not run",
                        denominators="these 42 runs stand alone: never added to S0, D1 or any earlier laptop run; C and E are tallied apart; 11 runs per candidate on Train are 9 parents"))
    target.write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(parents=len(view.rows), observations=len(view), optimizer_step_in_file=record["start"]["optimizer_step_in_file"], c=record["sample_plan"]["c"]["per_pool"],
                          e=record["sample_plan"]["e"]["per_pool"], cases=len(cases())), default=str))
    return record


# ----------------------------------------------------------------------------------------------
# Training
# ----------------------------------------------------------------------------------------------

def trained(condition):
    """The finished training of this protocol, or None."""
    out = training_dir(condition)
    try:
        run, proto = load(out / "run.json"), load(run_dir() / "protocol.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (run.get("run_id") == RUN and run.get("steps") == UPDATES and run.get("consumed_equals_plan") and run.get("sample_plan_sha256") == proto["sample_plan"]["sha256"]
          and run.get("warm_start_model_sha256") == START_SHA and (out / CHECKPOINT / "model.safetensors").exists() and run.get("model_sha256") == sha(out / CHECKPOINT / "model.safetensors"))
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
        print(f"[stored training of this protocol] {training_dir(condition)}")
        return
    record = load(run_dir() / "protocol.json")
    out = training_dir(condition)
    if out.exists():                                                       # an unfinished attempt is kept under another name, never continued silently
        kept = out.with_name(out.name + time.strftime(".unfinished-%Y%m%dT%H%M%S"))
        out.rename(kept)
        print(f"[unfinished attempt kept] {kept}")
    load_report = trial.gpu_other_load()
    trial.apply_reproducibility_settings()
    table = load(run_dir() / "prefix_manifest.json")
    start, frozen = start_identity()
    st, replay = replay_set()
    view, offset = pick_view(st, replay, table)
    if sha(run_dir() / "sample_plan.npz") != record["sample_plan"]["sha256"] or r2.view_sha(view) != record["prefix_view"]["view_sha256"] or start["optimizer_state"] != record["start"]["optimizer_state"]:
        raise RuntimeError("sample plan, pick view or optimizer state changed since the protocol")
    with np.load(run_dir() / "sample_plan.npz", allow_pickle=False) as a:
        keys, ts = plan_rows({k: a[k] for k in a.files}, condition)
    index = np.vectorize(offset.get)(keys) + ts
    policy = ACTPolicy.from_pretrained(START, local_files_only=True, strict=True).cuda()
    policy.check_normalization(st)
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=1e-4, weight_decay=1e-4)
    state = torch.load(START / "training_state.pt", map_location="cpu", weights_only=False)
    optimizer.load_state_dict(state["optimizer"])
    groups = [dict(lr=g["lr"], weight_decay=g["weight_decay"], tensors=len(g["params"])) for g in optimizer.param_groups]
    adam_steps = sorted({int(v["step"]) for v in optimizer.state_dict()["state"].values() if "step" in v})
    with np.load(rc.HORIZON_WEIGHTS, allow_pickle=False) as a:
        r = torch.tensor(a["r"], dtype=torch.float32, device="cuda")
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(START_TRAINING / "normalization.npz", out / "normalization.npz")
    consumed_e = np.full((UPDATES, BATCH), -1, dtype=np.int64)
    consumed_t = np.full((UPDATES, BATCH), -1, dtype=np.int64)
    valid_targets = 0
    torch.manual_seed(SEED)
    started_wall, started = time.strftime("%Y-%m-%dT%H:%M:%S%z"), time.monotonic()

    def save(step):
        name = out / f"step-{step:06d}"
        policy.save_pretrained(name)
        torch.save(dict(optimizer=optimizer.state_dict(), step=step, inherited_step=int(state["step"]), cpu_rng=torch.get_rng_state(), cuda_rng=torch.cuda.get_rng_state()), name / "training_state.pt")
        return name

    with (out / "metrics.jsonl").open("x") as log, warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        for step in range(1, UPDATES + 1):
            u = step - 1
            batch, meta = trial.split_meta(default_collate([view[int(i)] for i in index[u]]))
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
            if step == MIDWAY:
                save(step)
        name = save(UPDATES)
    seconds = round(time.monotonic() - started, 2)
    if not (np.array_equal(consumed_e, keys) and np.array_equal(consumed_t, ts)):
        raise RuntimeError("consumed samples differ from the plan")
    np.savez_compressed(out / "consumed_samples.npz", episode=consumed_e, timestep=consumed_t)
    after = start_identity()
    run = dict(kind="DAPIER ACT START-PICK R1 C / E (SIM only)", run_id=RUN, condition=condition, condition_name=NAMES[condition], policy_variant=policy.config.type, dataset=str(td.DATASET),
               train_episodes=[int(x["manifest_index"]) for x in view.rows], holdout_episodes=[], contract_revision=trial.load_manifest()["contract_revision"], control_dt=td.DT,
               hardware_access=False, task_success=None, seed=SEED, batch_size=BATCH, steps=UPDATES, checkpoints=[f"step-{MIDWAY:06d}", CHECKPOINT], warm_start=str(START),
               warm_start_model_sha256=START_SHA, optimizer_state_loaded=True, optimizer_state_sha256=start["optimizer_state"], optimizer_step_at_load=int(state["step"]),
               adam_step_counters_at_load=adam_steps, adam_step_counters_at_end=sorted({int(v["step"]) for v in optimizer.state_dict()["state"].values() if "step" in v}),
               optimizer_param_groups=groups, normalization_sha256=sha(out / "normalization.npz"), pick_view_sha256=record["prefix_view"]["view_sha256"],
               sample_plan_sha256=record["sample_plan"]["sha256"], consumed_sha256=trial.array_sha(consumed_e, consumed_t), consumed_equals_plan=True, samples=int(consumed_e.size),
               valid_action_targets=valid_targets, exposure=exposure(table, consumed_e, consumed_t), precision="float32, no autocast", model_sha256=sha(name / "model.safetensors"),
               config_sha256=sha(name / "config.json"), start_and_later_policies_before=[start, frozen], start_and_later_policies_unchanged_after=after == (start, frozen),
               started=started_wall, training_seconds=seconds, environment=trial.environment_record(), host=s0.host(), gpu_other_load_at_start=load_report,
               deterministic_warnings=sorted({str(w.message).split("\n")[0][:200] for w in caught}))
    (out / "run.json").write_text(json.dumps(run, indent=2, default=str))
    print(json.dumps(dict(condition=condition, seconds=seconds, valid_action_targets=valid_targets, per_pool=run["exposure"]["per_pool"], model=run["model_sha256"][:12]), default=str))
    print(f"[written] {name}")


# ----------------------------------------------------------------------------------------------
# Offline: D1's analysis on the 1,347 C25-boundary teacher observations
# ----------------------------------------------------------------------------------------------

def offline(args=None):
    for c in ("c", "e"):
        folder = run_dir() / f"offline_{c.upper()}"
        if (folder / "offline.json").exists():
            print(f"[stored] {folder / 'offline.json'}")
            continue
        saved = d1.MODELS, d1.run_dir
        d1.MODELS, d1.run_dir = {**d1.MODELS, "START": checkpoint(c)}, (lambda folder=folder: folder)
        try:
            d1.offline(None)
        finally:
            d1.MODELS, d1.run_dir = saved


def offline_table():
    """Per model and pool: medians over parents of the stored D1-style rows, and the same from the raw per-boundary rows."""
    sources = dict(FROZEN=FACTORY_ROOT / "evaluation" / d1.RUN / "offline.json", C=run_dir() / "offline_C/offline.json", E=run_dir() / "offline_E/offline.json")
    out = {}
    for name, path in sources.items():
        if not path.exists():
            out[name] = None
            continue
        data = load(path)
        rows = {}
        for pool in ("focus 3", "other 66"):
            ps = [p for p in data["parents"] if p["group"] == pool]
            b = [x for p in ps for x in p["boundaries_detail"]]
            prog = [x["left_arm"]["progress"] for x in b if x["left_arm"]["progress"] is not None]
            per_phase = {}
            for ph in d1.PICK_PHASES:
                sel = [x for x in b if x["phase"] == ph]
                if sel:
                    per_phase[ph] = dict(boundaries=len(sel), left_arm_mae=round(float(np.mean([x["left_arm"]["mae"] for x in sel])), 4), right_arm_mae=round(float(np.mean([x["right_arm"]["mae"] for x in sel])), 4))
            rows[pool] = dict(parents=len(ps), boundaries=len(b), left_arm_mae=round(float(np.mean([x["left_arm"]["mae"] for x in b])), 4), right_arm_mae=round(float(np.mean([x["right_arm"]["mae"] for x in b])), 4),
                              left_arm_progress_median=round(float(np.median(prog)), 2), left_arm_progress_below_half=int(sum(v < 0.5 for v in prog)), moving_boundaries=len(prog),
                              right_gripper_k24_abs_err_mean=round(float(np.mean([abs(x["right_gripper_k24"][0] - x["right_gripper_k24"][1]) for x in b])), 3),
                              first_boundary_right_gripper_k24_pred_mean=round(float(np.mean([p["first_boundary"]["right_gripper_k24"][1] for p in ps])), 3),
                              first_boundary_right_gripper_k24_teacher_mean=round(float(np.mean([p["first_boundary"]["right_gripper_k24"][0] for p in ps])), 3),
                              parents_first_boundary_left_progress_below_half=data["groups"][pool]["parents_first_boundary_left_progress_below_half"],
                              group_medians_as_stored=data["groups"][pool], per_phase=per_phase)
            for gen in ("G000", "G001"):
                sel = [x for p in ps if p["generation"] == gen for x in p["boundaries_detail"]]
                if sel:
                    rows[pool][f"left_arm_mae_{gen}"] = round(float(np.mean([x["left_arm"]["mae"] for x in sel])), 4)
        out[name] = dict(source=str(path), rows=rows, note="per-boundary values are stored rounded to 4 decimals; these means are recomputed from them (not from raw tensors)")
    return out


# ----------------------------------------------------------------------------------------------
# Evaluation
# ----------------------------------------------------------------------------------------------

def parents():
    t9 = {p["seed"]: p for p in d1.train9()}
    if tuple(t9) != TRAIN9:
        raise RuntimeError(f"Train9 rebuilt from the manifest is {tuple(t9)}")
    val = rv.parents()
    if tuple(p["seed"] for p in val) != VAL10:
        raise RuntimeError("Val10 rebuilt from the manifest differs from the instructed list")
    return dict(train=t9, val={p["seed"]: dict(p, group="val 10") for p in val})


def identity():
    import arm_output_trial as trial
    start, frozen = start_identity()
    models = {}
    for c in ("c", "e"):
        run = trained(c)
        if run is None:
            raise RuntimeError(f"training {c} of this protocol is not complete")
        models["START_" + c.upper()] = dict(path=str(checkpoint(c)), model=run["model_sha256"], config=sha(checkpoint(c) / "config.json"), normalization=sha(training_dir(c) / "normalization.npz"),
                                            training_run_sha256=sha(training_dir(c) / "run.json"))
        if models["START_" + c.upper()]["normalization"] != rk.NORMALIZATION:
            raise RuntimeError("normalization of a trained copy differs")
    models.update(frozen)
    ps = parents()
    resets = {}
    for split in ps:
        for seed, p in ps[split].items():
            meta = load(Path(d1.manifest()[p["manifest_index"]]["path"]) / "metadata.json")
            resets[str(seed)] = hashlib.sha256(json.dumps([meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"]], sort_keys=True).encode()).hexdigest()
    body = dict(run_id=RUN, models=models, start_frozen_not_evaluated=start["model"], manifest_sha256=MANIFEST_SHA, cases=[list(c) for c in cases()], resets=resets,
                sources={p: sha(FACTORY_ROOT / p) for p in SOURCES}, contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                execution=dict(chain="START_{C|E} -> RECEIVE_E -> RELEASE_B", chunk=CHUNK, prefix=K, control_hz=round(1 / td.DT), arm_bound_policy="project", ensemble="off", smoothing="off",
                               horizon_steps=rc.HORIZON, torch_seed=4100, policy_inputs=["left / right wrist RGB", "measured q_state"]),
                backend=trial.environment_record(), host=s0.host(), environment={k: os.environ.get(k) for k in rk.ENV_KEYS})
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def case_dir(ident, case):
    split, seed, model, rep = case
    return run_dir() / "runs" / ident["sha256"][:12] / split / f"s{seed}" / f"{model}_rep{rep}"


def complete(ident, case):
    out = case_dir(ident, case)
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (r.get("identity_sha256") == ident["sha256"] and r.get("case") == list(case) and r.get("job") == RUN and meta.get("complete") and r.get("boundary_contract") == "repaired"
          and "child_settings" in r and all((out / f).exists() for f in ("episode.npz", "action_trace.json", "task_check_rows.json", "predicted_chunks.npz")))
    return r if ok else None


def contact_summary(rows, result):
    """Pad forces on the block from the checker rows (50 Hz samples; the integral is a sum over those samples)."""
    from sim_data_factory.task_success import CheckerConfig
    g = CheckerConfig().grip_force_min_N
    out = {}
    for side in ("left", "right"):
        f = np.array([r["pad_force_N"][side] for r in rows], dtype=np.float64)
        touch, hold = f.max(1) > 0, f.min(1) >= g
        out[side] = dict(peak_N=round(float(f.max()), 2), touch_s=round(float(touch.sum() * td.DT), 2), two_pad_hold_s=round(float(hold.sum() * td.DT), 2),
                         force_time_integral_Ns=round(float(f.sum() * td.DT), 2), first_touch_s=None if not touch.any() else round(float(rows[int(np.argmax(touch))]["time"]), 2))
    lift = result["events"].get("pick_lift")
    if lift is None:
        out["left_hold_lost_after_pick_lift_s"] = "unavailable (no pick_lift)"
    else:
        gate = result["gate_s"]
        lost = next((r["time"] for r in rows if r["time"] > lift and (gate is None or r["time"] < gate) and min(r["pad_force_N"]["left"]) < g), None)
        out["left_hold_lost_after_pick_lift_s"] = None if lost is None else round(float(lost), 2)
    return out


def one(args):
    ident = identity()
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    case = (args.split, args.seed, args.model, args.rep)
    if list(case) not in ident["cases"]:
        raise ValueError(f"{case} is not one of the 42 fixed cases")
    parent = parents()[args.split][args.seed]
    out = case_dir(ident, case)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    rc.MODELS["START"], rc.MODELS["RECEIVE"] = checkpoint(args.model), FROZEN["RECEIVE_E"]
    loaded, original = {}, rc.load_policy

    def remember(name):
        got = original(name)
        loaded[name] = got
        return got

    rc.load_policy = remember
    started = time.time()
    chain = f"START_{args.model} ({NAMES[args.model.lower()]}) -> RECEIVE_E -> RELEASE_B"
    try:
        result = rc.execute(out, args.seed, ("START", "RECEIVE", "B"), None, f"{LABEL}: {args.split} parent {args.seed} {chain} rep{args.rep}", contract="repaired")
    finally:
        rc.load_policy = original
    want = dict(START=ident["models"]["START_" + args.model]["model"], RECEIVE=FROZEN_SHA["RECEIVE_E"], B=FROZEN_SHA["RELEASE_B"])
    got = {k: v[2]["model_sha256"] for k, v in loaded.items()}
    if any(got[k] != want[k] for k in got):
        raise RuntimeError(f"a loaded policy is not the intended one: {got}")
    rows = load(out / "task_check_rows.json")
    trace = load(out / "action_trace.json")
    with np.load(out / "episode.npz", allow_pickle=False) as a:
        states, q = a["integration_state"].copy(), a["q_state"].copy()
    rgb = {s: np.load(out / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}

    def moment(step):
        if step is None or step >= len(rgb["left"]):
            return None
        return dict(step=int(step), sim_time_s=round(float(states[step][0]), 2), state_sha256=array_sha(states[step]), q_sha256=array_sha(q[step]),
                    rgb_sha256=dict(left=array_sha(rgb["left"][step]), right=array_sha(rgb["right"][step])))

    entry = next((s for s in result["switches"] if s["to"] == "RECEIVE"), None)
    gate = next((s for s in result["switches"] if s["to"] == "B"), None)
    success, fault = result["task_success"], result["first_fault"]
    stage = None if success else "START" if entry is None else "RECEIVE" if gate is None else "RELEASE_B"
    kind = None if success else ("fault" if fault is not None else "task not completed within the 40 s horizon")
    events = result["events"]
    entry_valid = None
    if entry is not None:                                                  # the existing checker on this run's own states up to the RECEIVE entry (analysis only)
        from integration_scenes import task_env
        from sim_data_factory.task_success import evaluate_states
        n = entry["step"] + 1
        chk = evaluate_states(task_env("desk", grippers="both").model, states[:n], q[:n].astype(np.float64), control_dt=td.DT)
        entry_valid = dict(valid=bool(chk["valid"]), failures=chk["failures"][:3], events={k: round(v, 2) for k, v in chk["events"].items()})
    fault_t = None if fault is None else fault.get("time")
    result.update(job=RUN, kind="full", classification=LABEL, split="train" if args.split == "train" else "val (development)", case=list(case), parent=parent, start_model=args.model,
                  start_name=NAMES[args.model.lower()], repeat=args.rep, identity_sha256=ident["sha256"], tool_sha256=sha(__file__), chain=chain, loaded_models=got,
                  handover_event=result["handover_s"] is not None, completed_40s=result["termination"] == "horizon_completed", first_failure_stage=stage, first_failure_kind=kind,
                  events_before_fault=None if fault_t is None else sorted(k for k, v in events.items() if v <= fault_t),
                  receive_entry_valid_before=entry_valid,
                  handover_state_at_receive_entry=None if entry is None else entry.get("state_before"),
                  transfer=None if entry is None else r3.arm_motion(out, entry["step"]), block_contact=contact_summary(rows, result),
                  counts=dict(commands_dispatched=sum(t.get("ctrl_applied") is not None for t in trace), completed_transitions=result["completed_transitions"],
                              sim_elapsed_s=result["last_completed_state_sim_s"], wall_s=round(time.time() - started, 1)),
                  moments=dict(reset=moment(0), receive_entry=moment(entry["step"] if entry else None), release_b_entry=moment(gate["step"] if gate else None)),
                  start_segment=d1.start_segment(out, args.seed, result) if args.split == "train" else "unavailable (the Val teacher trajectory is not read for a comparison)",
                  contract_checks=rb.contract_checks(result), phases=r2.phase_details(result, rows, result["receive_entry_s"] or 0.0),
                  child_settings=dict(rk.child_settings({dict(START="START_" + args.model, RECEIVE="RECEIVE_E", B="RELEASE_B")[k]: v[0] for k, v in loaded.items()}), host=s0.host(),
                                      float32_matmul_precision=__import__("torch").get_float32_matmul_precision(), autocast=False))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(case=list(case), events={k: round(v, 2) for k, v in events.items()}, receive_entry=result["receive_entry_s"], ready=result["phases"]["semantic_ready_s"],
                          release_b=result["gate_s"], termination=result["termination"], first_failure_stage=stage, first_failure_kind=kind, task_success=success,
                          final_support=result["final_support"]), default=str))


def table_row(r):
    fault, proj = r["first_fault"], r["projection"]
    return dict(split=r["case"][0], seed=r["seed"], model=r["start_model"], rep=r["repeat"], group=r["parent"]["group"], generation=r["parent"].get("generation"),
                reach_s=r["events"].get("reach"), left_grasp_s=r["events"].get("left_grasp"), pick_lift_s=r["pick_lift_s"], receive_entry_s=r["receive_entry_s"],
                ready_s=r["phases"]["semantic_ready_s"], release_b_s=r["gate_s"], handover_s=r["handover_s"], handover_event=r["handover_event"], completed_40s=r["completed_40s"],
                task_success=r["task_success"], final_support=r["final_support"], termination=r["termination"], first_failure_stage=r["first_failure_stage"],
                first_failure_kind=r["first_failure_kind"], first_fault_s=None if fault is None else round(fault["time"], 2),
                first_fault_contacts=None if fault is None else fault.get("forbidden_contacts"), events_before_fault=r["events_before_fault"],
                projection={k: dict(steps=v["projected_steps"], dispatched=v["dispatched"], longest_s=v["longest_s"], max_raw_excess_rad=round(v["max_raw_excess_rad"], 4), axes=sorted(v["per_axis"]))
                            for k, v in proj.items()},
                block_contact=r["block_contact"], handover_state=None if r["handover_state_at_receive_entry"] is None else {
                    k: r["handover_state_at_receive_entry"][k] for k in ("block_in_left_site_mm", "pad_force_N", "gripper_opening", "block_bottom_clearance_m")},
                left_tcp_block_min_mm=r["start_segment"]["left_tcp_block_min_mm"] if isinstance(r["start_segment"], dict) else None, path=r["path"])


def tally(rows):
    entered, reached = [r for r in rows if r["receive_entry_s"] is not None], [r for r in rows if r["release_b_s"] is not None]
    return dict(runs=len(rows), reach=sum(r["reach_s"] is not None for r in rows), left_grasp=sum(r["left_grasp_s"] is not None for r in rows), pick_lift=sum(r["pick_lift_s"] is not None for r in rows),
                receive_entered=len(entered), receive_ready_of_entered="N/A (RECEIVE not entered)" if not entered else f"{sum(r['ready_s'] is not None for r in entered)}/{len(entered)}",
                release_b_entered=len(reached), release_b_success_of_reached="N/A (RELEASE_B not reached)" if not reached else f"{sum(r['task_success'] for r in reached)}/{len(reached)}",
                handover_event=sum(r["handover_event"] for r in rows), completed_40s=sum(r["completed_40s"] for r in rows), task_success=sum(r["task_success"] for r in rows),
                final_right_hand_only=sum(r["final_support"] == "right hand" for r in rows),
                first_failure_stage={k: sum(r["first_failure_stage"] == k for r in rows) for k in STAGES},
                first_failure_kind={k: sum(r["first_failure_kind"] == k for r in rows) for k in ("fault", "task not completed within the 40 s horizon")},
                start_projection_steps=sum(r["projection"]["START"]["steps"] for r in rows))


def collect(ident):
    plan = cases()
    done = [(c, r) for c in plan if (r := complete(ident, c)) is not None]
    table = [table_row(r) for _, r in done]
    sel = lambda **kw: [t for t in table if all(t[k] == v if not callable(v) else v(t[k]) for k, v in kw.items())]
    tallies = {}
    for m in ("C", "E"):
        tallies[m] = {"Train9 first run (9 parents)": tally(sel(model=m, split="train", rep=1)), "focus 3 first run": tally(sel(model=m, split="train", rep=1, group="focus 3")),
                      "additional 6": tally(sel(model=m, split="train", rep=1, group="additional 6")), "5000 / 5008 with repeats (4 runs, 2 parents)": tally(sel(model=m, split="train", seed=lambda s: s in (5000, 5008))),
                      "Train all 11 runs (9 parents)": tally(sel(model=m, split="train")), "development Val10": tally(sel(model=m, split="val"))}
    return plan, done, table, tallies


def run(args=None):
    ident = identity()
    run_dir().mkdir(parents=True, exist_ok=True)
    (run_dir() / "logs").mkdir(exist_ok=True)
    proto = load(run_dir() / "protocol.json")
    if proto["evaluation"]["cases"] != ident["cases"]:
        raise RuntimeError("the 42 cases differ from the list fixed before training")
    target = run_dir() / "evaluation_identity.json"
    if target.exists():
        if load(target)["sha256"] != ident["sha256"]:
            raise RuntimeError("evaluation_identity.json holds another identity (model, source, contract or host changed): nothing is run, nothing is overwritten")
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
        if out.exists():                                                   # an executor attempt without a complete result: kept under another name, then the case is run
            kept = run_dir() / "attempts" / f"{tag}_{time.strftime('%Y%m%dT%H%M%S')}"
            kept.parent.mkdir(exist_ok=True)
            out.rename(kept)
            attempts.append(dict(case=list(case), kept=str(kept), note="incomplete folder of an earlier executor attempt"))
        done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--split", case[0], "--seed", str(case[1]), "--model", case[2], "--rep", str(case[3]), "--identity", ident["sha256"]],
                              capture_output=True, text=True)
        n = sum(a["case"] == list(case) for a in attempts)
        (run_dir() / "logs" / f"{ident['sha256'][:12]}_{tag}{'' if not n else f'_retry{n}'}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        ok = done.returncode == 0 and complete(ident, case) is not None
        if not ok:
            attempts.append(dict(case=list(case), returncode=done.returncode, status="executor_error (not a task result)", stderr_tail=done.stderr[-800:]))
        attempts_file.write_text(json.dumps(attempts, indent=2))
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(line[-1][:600] if ok and line else f"[executor_error, not a task failure] {case}: returncode {done.returncode}: {done.stderr[-400:]}", flush=True)
    write_results(ident)


def write_results(ident=None):
    ident = ident or load(run_dir() / "evaluation_identity.json")
    plan, done, table, tallies = collect(ident)
    attempts = load(run_dir() / "attempts.json") if (run_dir() / "attempts.json").exists() else []
    settings = [r["child_settings"] for _, r in done]

    def pairs(split, seeds, rep=1):
        out = []
        for seed in seeds:
            row = dict(seed=seed)
            for m in ("C", "E"):
                row[m] = next((t for t in table if (t["split"], t["seed"], t["model"], t["rep"]) == (split, seed, m, rep)), None)
            out.append(row)
        return out

    training = {c.upper(): {k: v for k, v in (trained(c) or {}).items() if k in ("steps", "model_sha256", "training_seconds", "valid_action_targets", "optimizer_step_at_load", "consumed_equals_plan",
                                                                                 "adam_step_counters_at_load", "adam_step_counters_at_end", "started")} | dict(
        checkpoint=str(checkpoint(c)), exposure=(trained(c) or {}).get("exposure"), loss=loss_curve(c)) for c in ("c", "e")}
    data = dict(run_id=RUN, name=TITLE, note=f"{LABEL}; switches read SIM ground truth; 42 fixed cases on work6; never added to S0, D1 or earlier laptop runs; C and E tallied apart; "
                                             "Val is a development split, not a Test", identity_sha256=ident["sha256"], written=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                planned_cases=len(plan), completed_cases=len(done), missing_cases=[list(c) for c in plan if c not in [d[0] for d in done]],
                infrastructure=dict(executor_error_attempts=sum("returncode" in a for a in attempts), kept_incomplete_folders=sum("kept" in a for a in attempts), attempts=attempts),
                training=training, tally=tallies, train9_pairs=pairs("train", TRAIN9), preservation=[t for t in table if t["split"] == "train" and t["seed"] in (5000, 5008)],
                val_pairs=pairs("val", VAL10), offline=offline_table(),
                child_settings=dict(first=settings[0] if settings else None, pids_distinct=len({s["pid"] for s in settings}),
                                    all_policies_eval=all(not v for s in settings for v in s["policy_training_flag"].values()),
                                    hosts=sorted({s["host"]["hostname"] for s in settings}), gl=sorted({json.dumps(s["gl"], sort_keys=True) for s in settings})),
                table=table)
    (run_dir() / "results.json").write_text(json.dumps(data, indent=2, default=str))
    print(f"[written] {run_dir() / 'results.json'}: {len(done)} of {len(plan)} cases completed; executor-error attempts {data['infrastructure']['executor_error_attempts']}")
    summary()


def loss_curve(condition):
    path = training_dir(condition) / "metrics.jsonl"
    if not path.exists():
        return None
    rows = [json.loads(x) for x in path.read_text().splitlines() if x]
    pick = lambda s: next((dict(step=r["step"], loss=round(r["loss"], 4), arm=round(r["arm_l1_term"], 4), gripper=round(r["gripper_l1_term"], 4)) for r in rows if r["step"] == s), None)
    return [pick(s) for s in (1, 1000, 5000, 10000)]


def summary(args=None):
    path = run_dir() / "results.json"
    if not path.exists():
        print(f"{TITLE} ({RUN}): unavailable, no results.json")
        return
    data = load(path)
    fmt = lambda v: "-" if v is None else (f"{v:.2f}" if isinstance(v, float) else str(v))
    yes = lambda t: "-" if t is None else ("성공" if t["task_success"] else f"실패({t['first_failure_stage']})")
    print(f"\n=== {TITLE} ({RUN}) — {LABEL}; 전환은 SIM 정답 기반 ===")
    print(f"work6 신규 42건(Train 22 + 개발용 Val 20), START_C / START_E 각 추가 10,000 updates의 step-010000. S0·D1·과거 노트북 결과와 합치지 않는다. "
          f"완료 {data['completed_cases']}/{data['planned_cases']}건, 실행기 오류 시도 {data['infrastructure']['executor_error_attempts']}건, identity {data['identity_sha256'][:12]}.")
    print("\n| 후보 | 묶음 | 실행 | reach | left_grasp | pick_lift | RECEIVE 진입 | 수신 준비(진입 중) | RELEASE_B 진입 | handover 이벤트 | 40 s 완주 | 전체 유효 | 첫 실패 START/RECEIVE/RELEASE_B |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for m in ("C", "E"):
        for k, v in data["tally"][m].items():
            f = v["first_failure_stage"]
            print(f"| {m} | {k} | {v['runs']} | {v['reach']} | {v['left_grasp']} | {v['pick_lift']} | {v['receive_entered']} | {v['receive_ready_of_entered']} | {v['release_b_entered']} | "
                  f"{v['handover_event']} | {v['completed_40s']} | {v['task_success']}/{v['runs']} | {f['START']}/{f['RECEIVE']}/{f['RELEASE_B']} |")
    for title, key in (("Train9 첫 회 대응표", "train9_pairs"), ("개발용 Val10 대응표", "val_pairs")):
        print(f"\n{title}\n| parent | 묶음 | C pick_lift | C 결과 | C 첫 fault | E pick_lift | E 결과 | E 첫 fault |\n|---|---|---|---|---|---|---|---|")
        for p in data[key]:
            c, e = p["C"], p["E"]
            g = (c or e or {}).get("group", "-")
            print(f"| {p['seed']} | {g} | {fmt(c and c['pick_lift_s'])} | {yes(c)} | {fmt(c and c['first_fault_s'])} | {fmt(e and e['pick_lift_s'])} | {yes(e)} | {fmt(e and e['first_fault_s'])} |")
    print("\n5000 / 5008 보존표 (반복 포함)\n| parent | 후보 | 회차 | pick_lift | RECEIVE 진입 | RELEASE_B 진입 | handover | 결과 |\n|---|---|---|---|---|---|---|---|")
    for t in sorted(data["preservation"], key=lambda t: (t["seed"], t["model"], t["rep"])):
        print(f"| {t['seed']} | {t['model']} | {t['rep']} | {fmt(t['pick_lift_s'])} | {fmt(t['receive_entry_s'])} | {fmt(t['release_b_s'])} | {fmt(t['handover_s'])} | {yes(t)} |")
    print(f"  결과: {path}\n  보고서: {run_dir() / 'REPORT.md'}")


def video(args=None):
    import recovery_trial as rt
    ident = load(run_dir() / "evaluation_identity.json")
    data = load(run_dir() / "results.json")
    extra = [p for p in data["train9_pairs"] if p["seed"] not in FOCUS and p["C"] and p["E"]]
    val = [p for p in data["val_pairs"] if p["C"] and p["E"]]
    differ = lambda ps: next((p for p in ps if p["C"]["task_success"] != p["E"]["task_success"]), None)
    picks = [("train5000_first_C_E", "train", 5000, "pre-set"), ("train5008_first_C_E", "train", 5008, "pre-set")]
    d = differ(extra)
    picks.append((f"additional_train_first_success_difference_train{d['seed']}", "train", d["seed"], "first C / E success difference in the fixed order") if d
                 else ("additional_train_no_success_difference_train5011", "train", 5011, "no C / E success difference among the additional six: 5011 failure comparison"))
    d = differ(val)
    common = next((p for p in val if not p["C"]["task_success"] and not p["E"]["task_success"]), None)
    if d or common:
        picks.append((f"val_first_success_difference_val{d['seed']}", "val", d["seed"], "first C / E success difference in the fixed order") if d
                     else (f"val_first_common_failure_val{common['seed']}", "val", common["seed"], "no C / E success difference on Val10: first common failure"))
    chosen = []
    for tag, split, seed, why in picks:
        out = run_dir() / "videos" / f"{tag}_{ident['sha256'][:12]}.mp4"
        chosen.append(dict(tag=tag, split=split, seed=seed, reason=why, path=str(out)))
        if out.exists():
            print(f"[exists] {out}")
            continue
        rows = []
        for m in ("C", "E"):
            r = complete(ident, (split, seed, m, 1))
            if r is None:
                print(f"[unavailable] {split} {seed} {m}: no stored result; not replaced by another run")
                continue
            row = rt.teacher_row(f"STUDENT / SIM-ORACLE-GATED / {'Train' if split == 'train' else 'development Val'} {seed}: START_{m} -> RECEIVE_E -> RELEASE_B", Path(r["path"]))
            ev = dict(r["events"])
            ev.update({f"SWITCH->{'RELEASE_B' if s['to'] == 'B' else s['to']}": s["sim_time_s"] for s in r["switches"]})
            row["events"] = dict(sorted(ev.items(), key=lambda kv: kv[1]))
            row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}" + ("" if r["task_success"] else f" (first failure in {r['first_failure_stage']})"),
                               "VALID HANDOVER, right hand holds" if r["task_success"] else f"NO VALID HANDOVER; final support: {r['final_support']}")
            rows.append(row)
        if rows:
            out.parent.mkdir(exist_ok=True)
            rt.render(rows, out, f"{LABEL} / {split} parent {seed} ({why})", 2000, False, right_gripper=True)
            print(f"[written] {out}")
    (run_dir() / "videos.json").write_text(json.dumps(chosen, indent=2))


def everything(args=None):
    """Each step in its own process; a step whose stored output matches this protocol is not done again."""
    me = [sys.executable, str(Path(__file__).resolve())]
    for step in (["prefix"], ["protocol"], ["train", "--condition", "c"], ["train", "--condition", "e"], ["offline"], ["run"], ["video"]):
        print(f"--- {' '.join(step)} ---", flush=True)
        code = subprocess.run(me + step).returncode
        if code:
            raise SystemExit(f"step `{' '.join(step)}` ended with return code {code}; later steps are not started")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("prefix", "protocol", "offline", "run", "summary", "video", "results", "all"):
        sub.add_parser(name)
    t = sub.add_parser("train")
    t.add_argument("--condition", choices=("c", "e"), required=True)
    o = sub.add_parser("one")
    o.add_argument("--split", choices=("train", "val"), required=True)
    o.add_argument("--seed", type=int, required=True)
    o.add_argument("--model", choices=("C", "E"), required=True)
    o.add_argument("--rep", type=int, required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    return dict(prefix=prefix, protocol=protocol, train=train, offline=offline, run=run, one=one, summary=summary, video=video, results=lambda a: write_results(), all=everything)[args.command](args)


if __name__ == "__main__":
    main()
