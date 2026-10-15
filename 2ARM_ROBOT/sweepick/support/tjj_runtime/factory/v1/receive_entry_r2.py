#!/usr/bin/env python3
"""RECEIVE R2: entry-state aligned candidates under the repaired boundary contract (local SIM / learning only; no hardware).

START, RECEIVE R1 and RELEASE B are read-only. Two new RECEIVE candidates start from the same START weights and AdamW state:
  A  repaired Train69 receiving view + the four existing corrections (re-cut at the semantic end) + original receiving
     samples of Train 5000 / 5002 / 5008 in the compared slots
  B  the same, except that in the compared slots a parent with a valid actual-entry teacher correction gives a sample of
     that correction instead (same parent schedule)

  teacher   the existing re-entered HandoverTeacher, once from each of the three fixed actual entry states
  protocol  phase views, the paired sample plan and their direct checks, fixed before training
  train     --condition a | b, 5,000 updates each (B only if at least one actual-entry correction is valid)
  run       same-entry-state comparison R1 / A / B (9 suffixes) and original reset START -> RECEIVE_{A,B} -> B (6 runs)
  summary   the stored table of this run (used by `act-sim results`)
  video     seed 5000 original reset A / B (+ one other parent if its outcome differs)

Every switch reads SIM ground truth: SIM-ORACLE-GATED THREE-POLICY CANDIDATE. Only the ACT inputs are unprivileged.
"""
import argparse
import hashlib
import json
import os
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

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "receive-entry-r2-20261005-01"
REPAIR = FACTORY_ROOT / "evaluation" / rb.RUN / "runs" / "d1a30249437e"
ENTRY_STATES = {5000: 475, 5002: 400, 5008: 375}             # stored control step of RECEIVE's first command; selects material only
SEEDS = tuple(ENTRY_STATES)
EXPECTED_SHA = dict(START="19368b679b4a97213698b449243ab8443b851ba6b25aecd713558b63c74f9dc7", R1="491348867127be08f369eae9a17021374469d13de184604161b0a20d33c22f87",
                    B="5a001b39fe271251f3218eabb2781079c4e0c5d218169d6dca41c18917a3402c")
START_TRAINING = rc.START_TRAINING
START_CHECKPOINT = rc.START_CHECKPOINT
CHECKPOINT = "step-005000"
NAMES = dict(a="REPAIRED_DATA_CONTROL", b="ACTUAL_ENTRY_CORRECTION")
UPDATES, BATCH, SEED = 5000, 8, 4100
PREFIX = rc.PREFIX
EXPECTED_VIEW = dict(parents=69, transitions=50650)          # from the stored bounds table: a cross-check, not a target to fit
SOURCES = rb.SOURCES + ("factory/v1/receive_entry_r2.py", "factory/v1/handover_correction_trial.py", "factory/v1/arm_output_trial.py", "factory/v1/single_demo_trial.py",
                        "source/production_source_g000/sim_data_factory/teacher.py")
LOCAL_ORDER = {5000: ("R1", "A", "B"), 5002: ("A", "B", "R1"), 5008: ("B", "R1", "A")}      # rotation fixed before any run
FULL_ORDER = {5000: ("A", "B"), 5002: ("B", "A"), 5008: ("A", "B")}
LABEL = rc.LABEL


def load(path):
    return json.loads(Path(path).read_text())


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def training_dir(condition):
    return FACTORY_ROOT / "training" / f"{RUN}-{condition}"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def receive_models():
    return dict(R1=rc.RECEIVE_TRAINING / rc.RECEIVE_CHECKPOINT, A=training_dir("a") / CHECKPOINT, B=training_dir("b") / CHECKPOINT)


def frozen():
    out = {k: sha(p / "model.safetensors") for k, p in (("START", rc.MODELS["START"]), ("R1", receive_models()["R1"]), ("B", rc.MODELS["B"]))}
    if out != EXPECTED_SHA:
        raise RuntimeError(f"a read-only checkpoint differs from the instructed SHA-256: {out}")
    return out


def entry_case(seed):
    return REPAIR / f"s{seed}" / "REPAIRED_RECEIVE"


# ----------------------------------------------------------------------------------------------
# Teacher: one run from each actual entry state
# ----------------------------------------------------------------------------------------------

def teacher(args):
    import handover_correction_trial as hc
    import paired_release_check as pr
    from integration_scenes import task_env
    from sim_data_factory import student_eval
    from sim_data_factory.task_success import evaluate_states
    target = run_dir() / "teacher_collection.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    run_dir().mkdir(parents=True, exist_ok=True)
    frozen()
    manifest = []
    for seed, step in ENTRY_STATES.items():
        case = entry_case(seed)
        result = load(case / "result.json")
        sw = next(s for s in result["switches"] if s["to"] == "RECEIVE")
        if sw["step"] != step or result["identity_sha256"][:12] != "d1a30249437e":
            raise RuntimeError(f"seed {seed}: the stored RECEIVE entry is not step {step} of identity d1a30249437e")
        g = rc.stored_state(case, step)
        manifest.append(dict(seed=seed, source_case=str(case), control_step=step, sim_time_s=float(g["states"][-1][0]), state_sha256=hc.state_sha(g["states"][-1]),
                             q_state=g["q"][-1].tolist(), rgb_sha256={s: hashlib.sha256(g["rgb"][s].tobytes()).hexdigest() for s in ("left", "right")},
                             prefix_states=len(g["states"]), prefix_events=sw["events_at_switch"], gripper_opening=sw["state_before"]["gripper_opening"],
                             files={f: sha(case / f) for f in ("episode.npz", "cameras/left_wrist_rgb.npz", "cameras/right_wrist_rgb.npz", "result.json")}))
    (run_dir() / "teacher_manifest.json").write_text(json.dumps(dict(
        run_id=RUN, fixed_before_teacher=time.strftime("%Y-%m-%dT%H:%M:%S%z"), rule="three states, one teacher run each, no retry, no replacement; the whole integration state "
        "(block, joints, both grippers) is restored as stored", states=manifest), indent=2))
    m = task_env("desk", grippers="both").model
    original = student_eval.Observations
    results = []
    for item in manifest:
        seed, step = item["seed"], item["control_step"]
        g = rc.stored_state(item["source_case"], step)
        name = f"s{seed}_entry_{step:04d}"
        sensors = []

        def first_stored(*a, **k):                                   # the recorded first observation is the stored frame of that state
            student_eval.Observations = original
            sensors.append(pr.stored_first_sensor(dict(rgb=g["rgb"])))
            return sensors[-1]

        student_eval.Observations = first_stored
        try:
            entry = hc.continuation(run_dir() / "corrections" / name, seed, g["states"], g["q"], g["states"][-1],
                                    dict(classification="teacher_recovery_from_actual_receive_entry_state", source_student_case=item["source_case"], source_step=step,
                                         source_state_sha256=item["state_sha256"], note="a child of the Train parent, not a new independent initial condition"))
        finally:
            student_eval.Observations = original
        entry.update(id=name, seed=seed, source_step=step, full_teacher_handover_admitted=bool(entry.get("admitted")))
        receive = dict(valid=False, failure_class=None, reason=None)
        if not entry.get("path"):
            receive.update(failure_class="planning failure" if entry.get("teacher_failure") else "execution failure", reason=str(entry.get("teacher_failure") or entry.get("terminal_failure")))
        else:
            path = Path(entry["path"])
            rows = load(path / "task_check_rows.json")
            start, end, why = rb.semantic_bounds(rows, entry["events"], start_at_first_row=True)
            if why:
                fault = entry.get("teacher_failure") or entry.get("terminal_failure")
                receive.update(failure_class="execution failure" if fault else "not completed within the 40 s limit", reason=f"{why}; stop={entry['stop']}; {fault}")
            else:
                with np.load(path / "episode.npz", allow_pickle=False) as a:
                    suffix, suffix_q = a["integration_state"].copy(), a["q_state"].astype(np.float64)
                hist = np.concatenate([g["states"], suffix[1:end + 1]])
                check = evaluate_states(m, hist, np.concatenate([g["q"], suffix_q[1:end + 1]]), control_dt=td.DT)
                receive.update(valid=bool(check["valid"]), receive=[0, end], receive_abs_s=[round(step * td.DT, 2), round(rows[end]["time"], 2)], prefix_checker_valid=bool(check["valid"]),
                               prefix_failures=check["failures"][:3], events_up_to_readiness={k: round(v, 2) for k, v in check["events"].items()},
                               after_readiness=dict(stop=entry["stop"], teacher_failure=entry.get("teacher_failure"), terminal_failure=entry.get("terminal_failure"),
                                                    whole_run_valid=entry.get("checker_valid"), handover=entry["events"].get("handover")))
                if not check["valid"]:
                    receive.update(failure_class="execution failure", reason="the history up to readiness is not checker-valid")
        entry["receiving"] = receive
        results.append(entry)
        print(json.dumps(dict(id=name, reentry=entry.get("reentry"), stop=entry.get("stop"), transitions=entry.get("transitions"), receiving=receive,
                              events={k: round(v, 2) for k, v in (entry.get("events") or {}).items()}), default=str)[:1500], flush=True)
    target.write_text(json.dumps(dict(run_id=RUN, results=results, receiving_valid=[r["id"] for r in results if r["receiving"]["valid"]],
                                      note="teacher readiness is neither a student success nor a handover success"), indent=2, default=str))
    print(f"[written] {target}")


# ----------------------------------------------------------------------------------------------
# Views and the paired sample plan
# ----------------------------------------------------------------------------------------------

def stats():
    return rc.stats()


def correction_segments(entries, base):
    out = []
    for i, e in enumerate(entries):
        path = Path(e["path"])
        with np.load(path / "episode.npz", allow_pickle=False) as a:
            state, action = a["q_state"][:-1].astype(np.float32), a["q_cmd"].astype(np.float32)
        rgb = {s: np.load(path / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}
        if not (len(state) == len(action) == len(rgb["left"]) == len(rgb["right"]) == e["suffix_transitions"]):
            raise ValueError(f"{path}: N+1/N or camera alignment mismatch")
        out.append((base + i, dict(state=state, action=action, rgb=rgb), tuple(e["receive"]), e))
    return out


def datasets():
    import arm_output_trial as trial
    st = stats()
    episodes = trial.load_manifest()["episodes"]
    train_ids = [i for i, e in enumerate(episodes) if e["split"] == "train" and e["training_eligible"]]
    replay = trial.TrialEpisodes(td.DATASET, train_ids, required_split="train", stats=st, arm_target="measured_relative")
    segments, excluded = [], []
    for row in replay.rows:
        path = Path(row["path"])
        start, end, why = rb.semantic_bounds(load(path / "task_check_rows.json"), load(path / "metadata.json")["task_check"]["events"])
        if why:
            excluded.append(dict(manifest_index=row["manifest_index"], seed=row["seed"], reason=why))
            continue
        segments.append((row["manifest_index"], replay.episodes[row["manifest_index"]], (start, end),
                         dict(manifest_index=row["manifest_index"], seed=row["seed"], split=row["split"], parent_physics_group=row["parent_physics_group"])))
    train_view = rc.segment_view(st, td.DATASET, segments)
    # the four existing valid corrections, re-cut at the semantic end (no physics re-run)
    old_entries = []
    old_events = {r["id"]: r["events"] for r in load(rb.R1 / "teacher_collection.json")["results"] + load(rc.CORRECTION / "collection.json")["results"]}
    for e in rc.correction_entries()[0]:
        start, end, why = rb.semantic_bounds(load(Path(e["path"]) / "task_check_rows.json"), old_events[e["id"]], start_at_first_row=True)
        if why:
            raise ValueError(f"{e['id']}: {why}")
        old_entries.append(dict(e, receive=[start, end], receive_abs_s=[round(e["source_step"] * td.DT, 2), round((e["source_step"] + end) * td.DT, 2)], old_grid_end=e["receive"][1]))
    old_view = rc.segment_view(st, rb.R1, correction_segments(old_entries, 100000))
    new_entries = []
    for r in load(run_dir() / "teacher_collection.json")["results"]:
        if r["receiving"]["valid"]:
            for file, digest in r["files"].items():
                if sha(Path(r["path"]) / file) != digest:
                    raise ValueError(f"correction file changed: {r['path']}/{file}")
            new_entries.append(dict(id=r["id"], seed=r["seed"], path=r["path"], origin="actual entry state, this run", source_step=r["source_step"], suffix_transitions=r["transitions"],
                                    receive=r["receiving"]["receive"], receive_abs_s=r["receiving"]["receive_abs_s"], files=r["files"]))
    new_view = rc.segment_view(st, run_dir(), correction_segments(new_entries, 200000)) if new_entries else None
    return st, replay, train_view, old_view, new_view, excluded


def build_plan(train_view, old_view, new_view):
    """Slots 0-3 Train receiving (parent first, then a step); 4-5 the existing corrections (parent first, then one of its
    corrections, then a step); 6-7 parent first over 5000 / 5002 / 5008 whatever the teacher outcome: A = a step of that
    parent's original receiving view, B = a step of its actual-entry correction if it is valid, otherwise A's very sample."""
    rng = np.random.default_rng(SEED)

    def table(view):
        off, total, length = {}, 0, {}
        for key, ep in view.episodes.items():
            off[key], length[key], total = total, len(ep["action"]), total + len(ep["action"])
        return off, length

    def balanced(pool, count):
        blocks = -(-count // len(pool))
        return rng.permuted(np.tile(np.asarray(pool), (blocks, 1)), axis=1).reshape(-1)[:count]

    n = UPDATES
    off, length = table(train_view)
    p = balanced(list(train_view.episodes), n * 4)
    t = np.floor(rng.random(n * 4) * np.array([length[k] for k in p])).astype(np.int64)
    plan = dict(train_key=p.reshape(n, 4), train_t=t.reshape(n, 4), train_index=(np.array([off[k] for k in p]) + t).reshape(n, 4))
    o_off, o_length = table(old_view)
    by_seed = {}
    for r in old_view.rows:
        by_seed.setdefault(r["seed"], []).append(r["view_key"])
    seeds = balanced(sorted(by_seed), n * 2)
    pick = rng.random(n * 2)
    keys = np.array([by_seed[s][int(u * len(by_seed[s]))] for s, u in zip(seeds.tolist(), pick.tolist())])
    ot = np.floor(rng.random(n * 2) * np.array([o_length[k] for k in keys])).astype(np.int64)
    plan.update(old_parent=seeds.reshape(n, 2), old_key=keys.reshape(n, 2), old_t=ot.reshape(n, 2), old_index=(np.array([o_off[k] for k in keys]) + ot).reshape(n, 2))
    key_of = {r["seed"]: r["view_key"] for r in train_view.rows}
    cmp_seed = balanced(list(SEEDS), n * 2)
    ua, ub = rng.random(n * 2), rng.random(n * 2)              # both drawn always, so A's samples do not depend on the teacher outcome
    a_key = np.array([key_of[s] for s in cmp_seed.tolist()])
    a_t = np.floor(ua * np.array([length[k] for k in a_key])).astype(np.int64)
    a_index = np.array([off[k] for k in a_key]) + a_t
    b_new, b_key, b_t, b_index = np.zeros(n * 2, dtype=np.int64), a_key.copy(), a_t.copy(), a_index.copy()
    if new_view is not None:
        n_off, n_length = table(new_view)
        new_of = {r["seed"]: r["view_key"] for r in new_view.rows}
        for i, s in enumerate(cmp_seed.tolist()):
            if s in new_of:
                k = new_of[s]
                b_new[i], b_key[i], b_t[i] = 1, k, int(ub[i] * n_length[k])
                b_index[i] = n_off[k] + b_t[i]
    shape = lambda x: np.asarray(x).reshape(n, 2)
    plan.update(cmp_parent=shape(cmp_seed), a_key=shape(a_key), a_t=shape(a_t), a_index=shape(a_index), b_from_new=shape(b_new), b_key=shape(b_key), b_t=shape(b_t), b_index=shape(b_index))
    return plan


def check_plan(plan, replay, train_view, old_view, new_view):
    rc.check_plan(dict(train_index=plan["train_index"], train_key=plan["train_key"], train_t=plan["train_t"], corr_index=plan["old_index"], corr_key=plan["old_key"],
                       corr_t=plan["old_t"], corr_parent=plan["old_parent"]), replay, train_view, old_view)
    seed_of = {r["view_key"]: r["seed"] for r in train_view.rows}
    for i, k, t, s in zip(plan["a_index"].ravel(), plan["a_key"].ravel(), plan["a_t"].ravel(), plan["cmp_parent"].ravel()):
        assert train_view.indices[int(i)] == (int(k), int(t)) and seed_of[int(k)] == int(s)
    same = plan["b_from_new"] == 0
    assert np.array_equal(plan["b_index"][same], plan["a_index"][same]) and np.array_equal(plan["b_key"][same], plan["a_key"][same])   # no correction: A's very sample
    if new_view is not None:
        new_seed = {r["view_key"]: r["seed"] for r in new_view.rows}
        assert not (set(new_view.episodes) & (set(train_view.episodes) | set(old_view.episodes)))
        for i, k, t, s, f in zip(plan["b_index"].ravel(), plan["b_key"].ravel(), plan["b_t"].ravel(), plan["cmp_parent"].ravel(), plan["b_from_new"].ravel()):
            if f:
                assert new_view.indices[int(i)] == (int(k), int(t)) and new_seed[int(k)] == int(s)
        key = new_view.rows[0]["view_key"]
        ep, std, arm = new_view.episodes[key], new_view.stats["action"]["std"], list(rc.ARM)
        n = len(ep["action"])
        first = next(i for i, (k, t) in enumerate(new_view.indices) if k == key)
        for t in (0, max(n - 60, 0), n - 1):
            item = new_view[first + t]
            pad = item["action_is_pad"].numpy()
            assert pad.sum() == max(0, t + 50 - n) and (not pad.any() or pad[-1])                 # padding only past the semantic end
            j = min(3, n - 1 - t)
            assert np.allclose(item["action"][j, arm].numpy(), (ep["action"][t + j][arm] - ep["state"][t][arm]) / std[arm], atol=1e-6)   # the suffix's own q_meas[t]
    return True


def view_sha(view):
    if view is None:
        return None
    return hashlib.sha256(json.dumps([[r.get("manifest_index", r.get("id")), r["seed"], r["receive"]] for r in view.rows], sort_keys=True).encode()).hexdigest()


def protocol(args):
    import arm_output_trial as trial
    target = run_dir() / "protocol.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    st, replay, train_view, old_view, new_view, excluded = datasets()
    plan = build_plan(train_view, old_view, new_view)
    check_plan(plan, replay, train_view, old_view, new_view)
    np.savez_compressed(run_dir() / "sample_plan.npz", **plan)
    lengths = [len(e["action"]) for e in train_view.episodes.values()]
    config = load(START_TRAINING / START_CHECKPOINT / "config.json")
    ab_differ = bool(plan["b_from_new"].any())
    seeds, counts = np.unique(plan["cmp_parent"], return_counts=True)
    record = dict(
        run_id=RUN, name=LABEL + " / RECEIVE R2 entry-aligned A/B", fixed_before_training=time.strftime("%Y-%m-%dT%H:%M:%S%z"), hardware_access=False, val_test=False,
        frozen=frozen(), start=dict(checkpoint=str(START_TRAINING / START_CHECKPOINT), model_sha256=EXPECTED_SHA["START"],
                                    optimizer_state_sha256=sha(START_TRAINING / START_CHECKPOINT / "training_state.pt"),
                                    use="both conditions: all START weights and the stored AdamW state, read-only; nothing re-initialised; torch.manual_seed(4100)"),
        config_read_from_checkpoint={k: config.get(k) for k in ("type", "chunk_size", "n_action_steps", "kl_weight", "optimizer_lr", "optimizer_lr_backbone", "optimizer_weight_decay", "use_vae")},
        conditions=dict(a=NAMES["a"], b=NAMES["b"] if ab_differ else "NOT TRAINED: no valid actual-entry correction, B would equal A"), ab_differ=ab_differ,
        data=dict(train_view=dict(rule="receive_boundary_repair.semantic_bounds", parents=len(train_view.rows), transitions=len(train_view), expected_from_bounds_table=EXPECTED_VIEW,
                                  matches_expected=dict(parents=len(train_view.rows), transitions=len(train_view)) == EXPECTED_VIEW, excluded=excluded, sha256=view_sha(train_view),
                                  phase_length_steps=dict(min=min(lengths), median=float(np.median(lengths)), max=max(lengths))),
                  existing_corrections=[{k: r[k] for k in ("id", "seed", "source_step", "receive", "receive_abs_s", "old_grid_end", "path")} for r in old_view.rows],
                  existing_corrections_transitions=len(old_view), existing_corrections_sha256=view_sha(old_view),
                  actual_entry_corrections=[] if new_view is None else [{k: r[k] for k in ("id", "seed", "source_step", "receive", "receive_abs_s", "path")} for r in new_view.rows],
                  actual_entry_transitions=0 if new_view is None else len(new_view), actual_entry_sha256=view_sha(new_view),
                  parents_without_valid_entry_correction=[s for s in SEEDS if new_view is None or s not in {r["seed"] for r in new_view.rows}],
                  not_used=["any student prefix action", "teacher steps at or after the semantic readiness sample", "REPLAY/run2 approach state", "s5008_strict_0755", "Val / Test"]),
        sample_plan=dict(file=str(run_dir() / "sample_plan.npz"), sha256=sha(run_dir() / "sample_plan.npz"), seed=SEED, updates=UPDATES, batch=BATCH,
                         slots={"0-3": "repaired Train69 receiving view: parent first (balanced), then a uniform step; identical in A and B",
                                "4-5": "the four existing corrections, semantic end: parent first (5000 / 5008 balanced), then one of its corrections, then a uniform step; identical in A and B",
                                "6-7": "parent first over 5000 / 5002 / 5008 (balanced, independent of the teacher outcome); A = a uniform step of that parent's original receiving view; "
                                       "B = a uniform step of that parent's actual-entry correction if valid, else A's very sample"},
                         compared_parent_counts={str(int(k)): int(v) for k, v in zip(seeds, counts)},
                         b_samples_from_actual_entry={str(s): int(((plan["cmp_parent"] == s) & (plan["b_from_new"] == 1)).sum()) for s in SEEDS},
                         note="an operating choice of this experiment, not an optimal ratio"),
        unchanged=dict(normalization_sha256=sha(START_TRAINING / "normalization.npz"), horizon_weights_sha256=sha(rc.HORIZON_WEIGHTS), loss="B horizon weighting (single_demo_trial.loss_terms)",
                       optimizer="AdamW over policy.get_optim_params(), lr 1e-4 (backbone 1e-5), wd 1e-4, clip 1.0, as in the existing continuation path"),
        evaluation=dict(checkpoint="step-005000 of each condition, fixed before training", contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"], local_states=[dict(seed=s, step=k) for s, k in ENTRY_STATES.items()],
                        local_order={str(k): list(v) for k, v in LOCAL_ORDER.items()}, full_order={str(k): list(v) for k, v in FULL_ORDER.items()},
                        budget="local 9 (6 without B) + full 6 (3 without B), one each, whatever the results", horizon="absolute SIM 40 s or the first fault; never ended at the handover event",
                        success="whole run valid + handover + final right-hand-only support; the handover event alone is reported separately",
                        note="the three local states are the states the corrections start from; they are not independent original-reset results"),
        environment=dict(MUJOCO_GL=os.environ.get("MUJOCO_GL"), python=sys.executable, backend=trial.environment_record()))
    target.write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(train_view={k: record["data"]["train_view"][k] for k in ("parents", "transitions", "matches_expected", "excluded")},
                          existing=[(r["id"], r["receive"]) for r in old_view.rows], actual_entry=record["data"]["actual_entry_corrections"], ab_differ=ab_differ,
                          compared=record["sample_plan"]["compared_parent_counts"], b_new=record["sample_plan"]["b_samples_from_actual_entry"]), indent=1, default=str))
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
    if condition == "b" and not record["ab_differ"]:
        raise SystemExit("B equals A (no valid actual-entry correction): it is not trained")
    out = training_dir(condition)
    load_report = trial.gpu_other_load()
    if [p for p in load_report.get("other_processes", []) if isinstance(p["used_memory_mib"], int) and p["used_memory_mib"] > 1000]:
        raise RuntimeError("another heavy GPU job is running")
    trial.apply_reproducibility_settings()
    st, replay, train_view, old_view, new_view, _ = datasets()
    if sha(run_dir() / "sample_plan.npz") != record["sample_plan"]["sha256"] or view_sha(train_view) != record["data"]["train_view"]["sha256"]:
        raise RuntimeError("sample plan or phase view changed since the protocol")
    with np.load(run_dir() / "sample_plan.npz", allow_pickle=False) as a:
        plan = {k: a[k] for k in a.files}
    before = frozen()
    source = START_TRAINING / START_CHECKPOINT
    policy = ACTPolicy.from_pretrained(source, local_files_only=True, strict=True).cuda()
    policy.check_normalization(st)
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=1e-4, weight_decay=1e-4)
    state = torch.load(source / "training_state.pt", map_location="cpu", weights_only=False)
    optimizer.load_state_dict(state["optimizer"])
    groups = [dict(lr=g["lr"], weight_decay=g["weight_decay"], tensors=len(g["params"])) for g in optimizer.param_groups]
    with np.load(rc.HORIZON_WEIGHTS, allow_pickle=False) as a:
        r = torch.tensor(a["r"], dtype=torch.float32, device="cuda")
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(START_TRAINING / "normalization.npz", out / "normalization.npz")
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
            if condition == "a":
                items += [train_view[int(i)] for i in plan["a_index"][u]]
            else:
                items += [new_view[int(i)] if f else train_view[int(i)] for i, f in zip(plan["b_index"][u], plan["b_from_new"][u])]
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
    cmp_key, cmp_t = (plan["a_key"], plan["a_t"]) if condition == "a" else (plan["b_key"], plan["b_t"])
    if not (np.array_equal(consumed_e, np.concatenate([plan["train_key"], plan["old_key"], cmp_key], axis=1)) and
            np.array_equal(consumed_t, np.concatenate([plan["train_t"], plan["old_t"], cmp_t], axis=1))):
        raise RuntimeError("consumed samples differ from the plan")
    np.savez_compressed(out / "consumed_samples.npz", episode=consumed_e, timestep=consumed_t)
    new_rows = [] if new_view is None else new_view.rows
    cmp = consumed_e[:, 6:]
    seed_key = {r["seed"]: r["view_key"] for r in train_view.rows}
    compared = {str(s): dict(original_receiving=int((cmp == seed_key[s]).sum()), actual_entry_correction=int(sum((cmp == r["view_key"]).sum() for r in new_rows if r["seed"] == s))) for s in SEEDS}
    run = dict(kind="DAPIER ACT RECEIVE R2 entry-aligned A/B (SIM only)", run_id=RUN, condition=condition, condition_name=NAMES[condition], policy_variant=policy.config.type,
               dataset=str(td.DATASET), train_episodes=[int(x["manifest_index"]) for x in train_view.rows], holdout_episodes=[], contract_revision=trial.load_manifest()["contract_revision"],
               control_dt=td.DT, hardware_access=False, task_success=None, seed=SEED, batch_size=BATCH, steps=UPDATES, warm_start=str(source), warm_start_model_sha256=EXPECTED_SHA["START"],
               optimizer_state_loaded=True, optimizer_state_sha256=sha(source / "training_state.pt"), optimizer_step_at_load=int(state["step"]), optimizer_param_groups=groups,
               normalization_sha256=sha(out / "normalization.npz"), phase_views=dict(train=record["data"]["train_view"]["sha256"], existing_corrections=view_sha(old_view), actual_entry=view_sha(new_view)),
               sample_plan_sha256=record["sample_plan"]["sha256"], consumed_sha256=trial.array_sha(consumed_e, consumed_t), consumed_equals_plan=True, samples=int(consumed_e.size),
               valid_action_targets=valid_targets, padded_action_targets=int(consumed_e.size * 50 - valid_targets),
               consumed=dict(train_view_slots_0_3=int(consumed_e[:, :4].size), train_parents_distinct=int(len(np.unique(consumed_e[:, :4]))),
                             existing_corrections_slots_4_5={x["id"]: int((consumed_e[:, 4:6] == x["view_key"]).sum()) for x in old_view.rows}, compared_slots_6_7=compared),
               frozen_before=before, frozen_after=frozen(), started=started_wall, training_seconds=seconds, environment=trial.environment_record(), gpu_other_load_at_start=load_report,
               deterministic_warnings=sorted({str(w.message).split("\n")[0][:200] for w in caught}))
    (out / "run.json").write_text(json.dumps(run, indent=2, default=str))
    print(json.dumps(dict(condition=condition, seconds=seconds, valid_action_targets=valid_targets, consumed=run["consumed"]), indent=1))
    print(f"[written] {name}")


# ----------------------------------------------------------------------------------------------
# Evaluation under a verified identity
# ----------------------------------------------------------------------------------------------

def conditions():
    return ("A", "B") if load(run_dir() / "protocol.json")["ab_differ"] else ("A",)


def identity():
    import frozen_transfer_check as ft
    record = load(run_dir() / "protocol.json")
    models = {"START": rc.MODELS["START"], "RELEASE_B": rc.MODELS["B"], "RECEIVE_R1": receive_models()["R1"], **{f"RECEIVE_{c}": receive_models()[c] for c in conditions()}}
    resets = {}
    for seed in SEEDS:
        meta = ft.demonstration(seed)["metadata"]
        resets[str(seed)] = hashlib.sha256(json.dumps([meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"]], sort_keys=True).encode()).hexdigest()
    body = dict(run_id=RUN, models={k: {f: sha(p / f) for f in ("model.safetensors", "config.json")} | dict(normalization=sha(p.parent / "normalization.npz")) for k, p in models.items()},
                optimizer_start_sha256=record["start"]["optimizer_state_sha256"], phase_views={k: record["data"][k] for k in ("existing_corrections_sha256", "actual_entry_sha256")} | dict(
                    train=record["data"]["train_view"]["sha256"]), sample_plan_sha256=record["sample_plan"]["sha256"],
                entry_states={str(s["seed"]): s["state_sha256"] for s in load(run_dir() / "teacher_manifest.json")["states"]}, resets=resets,
                sources={p: sha(FACTORY_ROOT / p) for p in SOURCES}, contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                execution=dict(prefix=PREFIX, arm_bound_policy="project", ensemble="off", horizon_steps=rc.HORIZON, torch_seed=4100))
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def case_dir(ident, kind, seed, model):
    return run_dir() / "runs" / ident["sha256"][:12] / kind / f"s{seed}" / model


def complete(out, ident, case):
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (r.get("identity_sha256") == ident["sha256"] and r.get("case") == list(case) and meta.get("complete") and r.get("boundary_contract") == "repaired"
          and all((out / f).exists() for f in ("episode.npz", "action_trace.json", "task_check_rows.json")))
    return r if ok else None


def phase_details(r, rows, t0):
    """Order of what happened: readiness, switch, release, retreat, handover event, fault; and what preceded a missing readiness."""
    g = 0.3
    times = [x["time"] for x in rows]
    left = np.array([min(x["pad_force_N"]["left"]) >= g for x in rows])
    right_any = np.array([max(x["pad_force_N"]["right"]) > 0 for x in rows])
    right_two = np.array([min(x["pad_force_N"]["right"]) >= g for x in rows])
    table = np.array([x["table_support_N"] > 0 for x in rows])
    lo = next((i for i, t in enumerate(times) if t >= t0 - 1e-9), 0)
    grasp = r["events"].get("right_grasp")
    ready = None if grasp is None else next((x["time"] for x in rows if x["time"] >= grasp - 1e-9 and rb.hands(x)[1]), None)
    lost = [times[i] for i in range(max(lo, 1), len(rows)) if left[i - 1] and not left[i]]
    fault = r["first_fault"]
    order = sorted([(k, v) for k, v in dict(receive_entry=r["receive_entry_s"], right_first_touch=next((times[i] for i in range(lo, len(rows)) if right_any[i]), None),
                                            right_two_pad_first=next((times[i] for i in range(lo, len(rows)) if right_two[i]), None), right_grasp_event=grasp, semantic_ready=ready,
                                            b_switch=r["gate_s"], left_release=r["left_release_while_right_holds_first_s"], left_retreat=r["left_clear_by_donor_clearance_first_s"],
                                            handover_event=r["handover_s"], left_hold_first_lost=lost[0] if lost else None,
                                            block_on_table=next((times[i] for i in range(lo, len(rows)) if table[i]), None),
                                            first_fault=None if fault is None else fault["time"]).items() if v is not None], key=lambda kv: kv[1])
    return dict(semantic_ready_s=None if ready is None else round(ready, 2), order=[[k, round(float(v), 2)] for k, v in order],
                fault_pairs=None if fault is None else fault.get("forbidden_contacts") or fault.get("detail") or fault.get("reason"))


def one(args):
    ident = identity()
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    out = case_dir(ident, args.kind, args.seed, args.model)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    rc.MODELS["RECEIVE"] = receive_models()[args.model]                # the RECEIVE slot of the unchanged chain executor
    if args.kind == "local":
        step = ENTRY_STATES[args.seed]
        result = rc.execute(out, args.seed, ("RECEIVE", "B"), rc.stored_state(entry_case(args.seed), step), f"entry s{args.seed} step {step} RECEIVE_{args.model}", contract="repaired")
        t0 = step * td.DT
    else:
        result = rc.execute(out, args.seed, ("START", "RECEIVE", "B"), None, f"seed {args.seed} START -> RECEIVE_{args.model} -> B", contract="repaired")
        t0 = result["receive_entry_s"] or 0.0
    rows = load(out / "task_check_rows.json")
    result.update(kind=args.kind, case=[args.kind, args.seed, args.model], receive_model=args.model, receive_checkpoint=str(receive_models()[args.model]), identity_sha256=ident["sha256"],
                  chain=result["chain"].replace("RECEIVE", f"RECEIVE_{args.model}"), handover_event=result["handover_s"] is not None, contract_checks=rb.contract_checks(result),
                  phases=phase_details(result, rows, t0))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(kind=args.kind, seed=args.seed, model=args.model, order=result["phases"]["order"], termination=result["termination"], run_valid=result["run_valid"],
                          handover_event=result["handover_event"], task_success=result["task_success"], final_support=result["final_support"]), default=str))


def table_row(r):
    q = r["receiving"] or {}
    fault = r["first_fault"]
    return dict(kind=r["kind"], seed=r["seed"], receive=r["receive_model"], pick_lift_s=r["pick_lift_s"], receive_entry_s=r["receive_entry_s"] if r["kind"] == "full" else ENTRY_STATES[r["seed"]] * td.DT,
                right_grasp_s=r["right_grasp_s"], semantic_ready_s=r["phases"]["semantic_ready_s"], b_switch_s=r["gate_s"], left_release_s=r["left_release_while_right_holds_first_s"],
                left_retreat_s=r["left_clear_by_donor_clearance_first_s"], handover_event_s=r["handover_s"], first_fault_s=None if fault is None else round(fault["time"], 2),
                fault_stage=r["fault_stage"], fault_pairs=r["phases"]["fault_pairs"], termination=r["termination"], run_valid=r["run_valid"], final_support=r["final_support"],
                task_success=r["task_success"], order=r["phases"]["order"],
                projection={k: dict(steps=v["projected_steps"], of=v["dispatched"], longest_s=v["longest_s"], max_raw_excess_rad=round(v["max_raw_excess_rad"], 4), per_axis=v["per_axis"])
                            for k, v in r["projection"].items()},
                right_before_two_pad_hold=q.get("right_before_two_pad_hold"), right_two_pad_hold=q.get("right_two_pad_hold"), block_in_left_site=q.get("block_in_left_site"),
                left_hold_lost_steps=q.get("left_hold_lost_steps"), c25_regular=r["contract_checks"]["c25_consumption_regular"],
                switches=r["contract_checks"]["switches"])


def planned():
    cond = conditions()
    jobs = [("local", s, m) for s in SEEDS for m in LOCAL_ORDER[s] if m == "R1" or m in cond]
    return jobs + [("full", s, m) for s in SEEDS for m in FULL_ORDER[s] if m in cond]


def run(args):
    record = load(run_dir() / "protocol.json")
    for c in conditions():
        if not (receive_models()[c] / "model.safetensors").exists():
            raise FileNotFoundError(f"RECEIVE_{c} {CHECKPOINT} does not exist: run `train --condition {c.lower()}` first")
    frozen()
    ident = identity()
    jobs = planned()
    (run_dir() / "logs").mkdir(exist_ok=True)
    (run_dir() / f"evaluation_identity_{ident['sha256'][:12]}.json").write_text(json.dumps(dict(identity=ident, cases=[list(j) for j in jobs], fixed=time.strftime("%Y-%m-%dT%H:%M:%S%z")), indent=2))
    status = []
    for kind, seed, model in jobs:
        out = case_dir(ident, kind, seed, model)
        case = (kind, seed, model)
        if complete(out, ident, case) is not None:
            status.append(dict(case=list(case), status="completed (reused: same identity, complete files)"))
            print(f"[reused under identity {ident['sha256'][:12]}] {kind} {seed} RECEIVE_{model}")
            continue
        if out.exists():
            status.append(dict(case=list(case), status="NOT RUN: an incomplete folder of this identity exists; neither reused nor overwritten", path=str(out)))
            print(f"[not run] {kind} {seed} RECEIVE_{model}: incomplete folder {out}")
            continue
        done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--kind", kind, "--seed", str(seed), "--model", model, "--identity", ident["sha256"]], capture_output=True, text=True)
        (run_dir() / "logs" / f"{ident['sha256'][:12]}_{kind}_s{seed}_{model}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        ok = done.returncode == 0 and complete(out, ident, case) is not None
        status.append(dict(case=list(case), returncode=done.returncode, status="completed" if ok else "EXECUTION ERROR (not a task result)", **({} if ok else dict(stderr_tail=done.stderr[-600:]))))
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(line[-1][:900] if ok and line else f"[EXECUTION ERROR, not a task failure] {kind} {seed} RECEIVE_{model}: returncode {done.returncode}: {done.stderr[-300:]}", flush=True)
    results = [r for kind, seed, model in jobs if (r := complete(case_dir(ident, kind, seed, model), ident, (kind, seed, model))) is not None]
    by = {tuple(r["case"]): r for r in results}
    pairs = {}
    for s in SEEDS:
        if ("full", s, "A") in by and ("full", s, "B") in by:
            pairs[str(s)] = rb.pair_difference(by[("full", s, "A")], by[("full", s, "B")])
    tally = {}
    for kind in ("local", "full"):
        for model in ("R1", "A", "B"):
            rs = [r for r in results if r["kind"] == kind and r["receive_model"] == model]
            if rs:
                tally[f"{kind} RECEIVE_{model}"] = dict(runs=len(rs), semantic_ready=sum(r["phases"]["semantic_ready_s"] is not None for r in rs), b_switch=sum(r["gate_reached"] for r in rs),
                                                        handover_event=sum(r["handover_event"] for r in rs), valid_task_success=sum(r["task_success"] for r in rs))
    (run_dir() / "results.json").write_text(json.dumps(dict(
        run_id=RUN, note=f"{LABEL}; switches read SIM ground truth; Train parents; one run per case, not a success rate. local = restored actual entry states that the corrections start from; "
                         "full = original reset. The two tables are never added to each other or to teacher results.",
        identity_sha256=ident["sha256"], identity_unchanged_during_run=identity()["sha256"] == ident["sha256"], ab_differ=record["ab_differ"], planned_cases=len(jobs),
        completed_cases=len(results), execution_status=status, tally=tally, table=[table_row(r) for r in results], full_pair_difference_not_controlled=pairs, results=results), indent=2, default=str))
    errors = [s for s in status if not s["status"].startswith("completed")]
    print(f"[written] {run_dir() / 'results.json'}: {len(results)} of {len(jobs)} cases completed, {len(errors)} execution error(s) / not run; identity {ident['sha256'][:12]}")
    summary(args)


def summary(args=None):
    """The stored table of this run only (never merged with other evaluations)."""
    path = run_dir() / "results.json"
    if not path.exists():
        print(f"RECEIVE R2 ({RUN}): unavailable, no results.json")
        return
    data = load(path)
    print(f"\n=== RECEIVE R2 entry-aligned A/B ({RUN}) — {LABEL}; 전환은 SIM 정답 기반. Train parent, 조건당 1회. 다른 평가와 합산하지 않는다 ===")
    print(f"완료 {data['completed_cases']}/{data['planned_cases']}건, 실행 오류 {sum(not s['status'].startswith('completed') for s in data['execution_status'])}건, identity {data['identity_sha256'][:12]}")
    for kind, title in (("local", "같은 실제 진입 상태 복원(교정 시작 상태와 같음; 원래 reset 결과 아님)"), ("full", "원래 reset 전체 연결")):
        print(f"\n[{title}]\n| seed | RECEIVE | 수신 준비 | B 전환 | handover 이벤트 | 첫 fault | 최종 지지 | 전체 유효 성공 |\n|---|---|---|---|---|---|---|---|")
        for t in data["table"]:
            if t["kind"] == kind:
                fault = "-" if t["first_fault_s"] is None else f"{t['first_fault_s']} s {t['fault_pairs']}"
                print(f"| {t['seed']} | {t['receive']} | {t['semantic_ready_s'] or '-'} | {t['b_switch_s'] or '-'} | {'-' if t['handover_event_s'] is None else round(t['handover_event_s'], 2)} | {fault} | "
                      f"{t['final_support']} | {'예' if t['task_success'] else '아니오'} |")
    print("\n집계(실행 수 / 수신 준비 / B 전환 / handover 이벤트 / 전체 유효 성공):")
    for k, v in data["tally"].items():
        print(f"  {k}: {v['runs']} / {v['semantic_ready']} / {v['b_switch']} / {v['handover_event']} / {v['valid_task_success']}")
    print(f"  결과: {path}\n  보고서: {run_dir() / 'REPORT.md'}")


def video(args):
    import recovery_trial as rt
    ident = identity()
    data = load(run_dir() / "results.json")
    full = {(t["seed"], t["receive"]): t for t in data["table"] if t["kind"] == "full"}
    seeds = [5000] + [s for s in (5002, 5008) if (s, "A") in full and (s, "B") in full and
                      (full[(s, "A")]["task_success"], full[(s, "A")]["handover_event_s"] is None, full[(s, "A")]["semantic_ready_s"] is None)
                      != (full[(s, "B")]["task_success"], full[(s, "B")]["handover_event_s"] is None, full[(s, "B")]["semantic_ready_s"] is None)][:1]
    for seed in seeds:
        out = run_dir() / "videos" / f"seed{seed}_original_reset_A_B_{ident['sha256'][:12]}.mp4"
        if out.exists():
            print(f"[exists] {out}")
            continue
        rows = []
        for model in conditions():
            r = complete(case_dir(ident, "full", seed, model), ident, ("full", seed, model))
            if r is None:
                print(f"[unavailable] seed {seed} RECEIVE_{model}: no completed case under this identity; the row is not replaced")
                continue
            row = rt.teacher_row(f"{r['chain']} (original reset)", Path(r["path"]))
            ev = dict(r["events"])
            ev.update({f"SWITCH->{s['to']}": s["sim_time_s"] for s in r["switches"]})
            row["events"] = dict(sorted(ev.items(), key=lambda kv: kv[1]))
            row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}" + ("" if r["gate_reached"] else " (B NOT REACHED)"),
                               "VALID HANDOVER, right hand holds" if r["task_success"] else f"no valid handover; final support: {r['final_support']}")
            rows.append(row)
        if rows:
            rt.render(rows, out, f"{LABEL} / R2 / seed {seed} original reset", 2000, False, right_gripper=True)
            print(f"[written] {out}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("teacher", "protocol", "run", "summary", "video"):
        sub.add_parser(name)
    t = sub.add_parser("train")
    t.add_argument("--condition", choices=("a", "b"), required=True)
    o = sub.add_parser("one")
    o.add_argument("--kind", choices=("local", "full"), required=True)
    o.add_argument("--seed", type=int, choices=SEEDS, required=True)
    o.add_argument("--model", choices=("R1", "A", "B"), required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    dict(teacher=teacher, protocol=protocol, train=train, run=run, one=one, summary=summary, video=video)[args.command](args)


if __name__ == "__main__":
    main()
