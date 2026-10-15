#!/usr/bin/env python3
"""RECEIVE phase / switch boundary contract repair and a fixed-weight comparison (local SIM only; no training, no teacher
collection, no hardware). START, RECEIVE R1 and B are read-only; receive-candidate-20261005-01 stays as it is.

  bounds    Train69 (and the corrections R1 used): real event / contact timeline, the old C25-grid phase and the repaired
            semantic phase side by side; the actual student entry states of 5000 / 5002 / 5008 next to the views
  history   what the duplicated current state in the gate history changed, computed on the stored runs
  run       original reset, Train 5000 / 5002 / 5008 x LEGACY_RECEIVE / REPAIRED_RECEIVE, one run each, own process
  video     seed 5000, both contracts

Repaired contract: (1) the phase end and the RECEIVE -> B switch use one readiness predicate (existing right_grasp
confirmed + both hands hold now + no table support) evaluated on every 50 Hz sample; (2) the phase start clock is the
policy's own C25 clock (local control index), never an absolute multiple of 25; (3) the gate history holds the current
state once. Checker thresholds, physics and success conditions are the existing ones.
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

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "receive-boundary-repair-20261005-01"
R1 = FACTORY_ROOT / "evaluation" / rc.RUN
SEEDS = (5000, 5002, 5008)
CONTRACTS = dict(LEGACY_RECEIVE="legacy", REPAIRED_RECEIVE="repaired")
ORDER = {5000: ("LEGACY_RECEIVE", "REPAIRED_RECEIVE"), 5002: ("REPAIRED_RECEIVE", "LEGACY_RECEIVE"), 5008: ("LEGACY_RECEIVE", "REPAIRED_RECEIVE")}   # fixed before any run
CHAIN = rc.CHAINS["candidate"]
PREFIX = rc.PREFIX
SOURCES = ("factory/v1/receive_boundary_repair.py", "factory/v1/receive_candidate.py", "factory/v1/live_release_check.py", "factory/v1/paired_release_check.py",
           "source/production_source_g000/sim_data_factory/student_eval.py", "source/production_source_g000/sim_data_factory/core.py",
           "source/production_source_g000/sim_data_factory/recorder.py", "source/production_source_g000/sim_data_factory/task_success.py",
           "source/production_source_g000/sim_data_factory/sensor.py", "source/production_source_g000/dapier_act_policy.py")
CONTRACT_TEXT = dict(
    LEGACY_RECEIVE="receive-candidate-20261005-01 as run: START -> RECEIVE and RECEIVE -> B are checked only where the active policy is due for a C25 inference; "
                   "the gate history contains the current state twice",
    REPAIRED_RECEIVE="START -> RECEIVE as before (existing pick_lift + left hand holds now, at START's regular C25 boundary). While RECEIVE is active the B readiness "
                     "(existing right_grasp confirmed + both hands hold now + table support 0) is checked at every 50 Hz control step; when it holds the rest of "
                     "RECEIVE's chunk is discarded and B infers from that moment's RGB / q. Normal consumption stays C25. The gate history contains each state once.")


def load(path):
    return json.loads(Path(path).read_text())


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# ----------------------------------------------------------------------------------------------
# One readiness predicate for the training phase end and for the runtime switch
# ----------------------------------------------------------------------------------------------

def hands(row, cfg=None):
    """(left holds, both hold) at the checker's own grip level with no table support."""
    from sim_data_factory.task_success import CheckerConfig
    cfg = cfg or CheckerConfig()
    left = min(row["pad_force_N"]["left"]) >= cfg.grip_force_min_N and row["table_support_N"] == 0.0
    return left, left and min(row["pad_force_N"]["right"]) >= cfg.grip_force_min_N


def semantic_bounds(rows, events, start_at_first_row=False):
    """Receiving phase [start, end) in row indices of this episode (local control index of the policy that starts at row 0).
    start: first local C25 boundary (index % 25 == 0) at or after the checker's pick_lift where the left hand holds.
    end:   first 50 Hz sample at or after the checker's right_grasp where both hands hold now and the table gives no support.
    Event times are compared with the rows' own time stamps, so a restored offset (0 / 410 / 755 / 900) needs no index math."""
    eps = 1e-9
    if "pick_lift" not in events:
        return None, None, "the checker has no pick_lift event"
    if "right_grasp" not in events:
        return None, None, "the checker has no right_grasp event"
    start = 0 if start_at_first_row else next((j for j, r in enumerate(rows) if j % PREFIX == 0 and r["time"] >= events["pick_lift"] - eps and hands(r)[0]), None)
    if start is None:
        return None, None, "no local C25 boundary after pick_lift with a left-hand hold"
    end = next((j for j, r in enumerate(rows) if j > start and r["time"] >= events["right_grasp"] - eps and hands(r)[1]), None)
    if end is None:
        return start, None, "both hands never hold with no table support at or after the right_grasp confirmation"
    return start, end, None


def history_states(prefix_states, recorded_states):
    """Gate history: the stored prefix and the recorder's N+1 states joined so that a state of one time stamp appears once.
    Only an equal-time boundary sample is dropped; samples at different times are kept even if the pose is the same."""
    parts = [np.asarray(s) for s in list(prefix_states) + list(recorded_states)]
    out = [parts[0]]
    for s in parts[1:]:
        if float(s[0]) != float(out[-1][0]):
            out.append(s)
    return np.stack(out)


def intervals(mask, times):
    out, start = [], None
    for i, v in enumerate(list(mask) + [False]):
        if v and start is None:
            start = i
        if not v and start is not None:
            out.append([round(float(times[start]), 2), round(float(times[i - 1]), 2), i - start])
            start = None
    return out


# ----------------------------------------------------------------------------------------------
# bounds: the stored timelines, old and repaired phase, and the entry states
# ----------------------------------------------------------------------------------------------

def bounds(args):
    import mujoco
    import arm_output_trial as trial
    import frozen_transfer_check as ft
    from integration_scenes import task_env
    from sim_data_factory.task_success import ContactModel
    target = run_dir() / "bounds_table.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    run_dir().mkdir(parents=True, exist_ok=True)
    episodes = trial.load_manifest()["episodes"]
    train = [(i, e) for i, e in enumerate(episodes) if e["split"] == "train" and e["training_eligible"]]
    table = []
    for index, e in train:
        path = Path(e["path"])
        rows, events = load(path / "task_check_rows.json"), load(path / "metadata.json")["task_check"]["events"]
        with np.load(path / "episode.npz", allow_pickle=False) as a:
            n_state, n_action, sim_time = len(a["q_state"]), len(a["q_cmd"]), a["sim_time"].copy()
        times = np.array([r["time"] for r in rows])
        aligned = bool(len(rows) == n_state == n_action + 1 and np.allclose(times, sim_time, atol=1e-9) and np.allclose(times, np.arange(len(rows)) * td.DT, atol=1e-6))
        both = np.array([hands(r)[1] for r in rows])
        after = both & (times >= events.get("right_grasp", np.inf) - 1e-9)
        old = rc.receive_bounds(rows, events)
        new = semantic_bounds(rows, events)
        table.append(dict(
            manifest_index=index, seed=e["seed"], rows=len(rows), actions=n_action, rows_states_actions_time_aligned=aligned,
            pick_lift_s=round(events["pick_lift"], 2) if "pick_lift" in events else None, right_grasp_s=round(events["right_grasp"], 2) if "right_grasp" in events else None,
            both_hands_intervals_s=intervals(both, times), both_hands_total_s=round(float(both.sum() * td.DT), 2),
            ready_after_right_grasp_intervals_s=intervals(after, times), ready_after_right_grasp_s=round(float(after.sum() * td.DT), 2),
            c25_boundaries_ready_s=[round(float(times[j]), 2) for j in range(0, len(rows), PREFIX) if after[j]],
            old=dict(start=old[0], end=old[1], start_s=None if old[0] is None else round(old[0] * td.DT, 2), end_s=None if old[1] is None else round(old[1] * td.DT, 2), excluded=old[2]),
            new=dict(start=new[0], end=new[1], start_s=None if new[0] is None else round(new[0] * td.DT, 2), end_s=None if new[1] is None else round(new[1] * td.DT, 2), excluded=new[2])))
    corrections = []
    for e in rc.correction_entries()[0]:
        rows = load(Path(e["path"]) / "task_check_rows.json")
        events = next(r for r in load(R1 / "teacher_collection.json")["results"] + load(rc.CORRECTION / "collection.json")["results"] if r["id"] == e["id"])["events"]
        new = semantic_bounds(rows, events, start_at_first_row=True)
        off = e["source_step"]
        corrections.append(dict(id=e["id"], seed=e["seed"], phase_start_step=off, old_end_local=e["receive"][1], old_end_abs_step=off + e["receive"][1], old_end_s=round((off + e["receive"][1]) * td.DT, 2),
                                new_end_local=new[1], new_end_abs_step=None if new[1] is None else off + new[1], new_end_s=None if new[1] is None else round(rows[new[1]]["time"], 2),
                                local_c25_boundaries_around_old_end=[off + (e["receive"][1] // PREFIX) * PREFIX if (e["receive"][1] % PREFIX) else off + e["receive"][1],
                                                                     off + -(-e["receive"][1] // PREFIX) * PREFIX],
                                old_end_is_a_local_c25_boundary=bool(e["receive"][1] % PREFIX == 0), right_grasp_s=round(events["right_grasp"], 2), new_excluded=new[2]))
    old_in = [t for t in table if not t["old"]["excluded"]]
    new_in = [t for t in table if not t["new"]["excluded"]]
    summary = dict(train_parents=len(table), aligned=sum(t["rows_states_actions_time_aligned"] for t in table),
                   old_included=len(old_in), old_excluded=len(table) - len(old_in), old_transitions=sum(t["old"]["end"] - t["old"]["start"] for t in old_in),
                   new_included=len(new_in), new_excluded=[dict(seed=t["seed"], reason=t["new"]["excluded"]) for t in table if t["new"]["excluded"]],
                   new_transitions=sum(t["new"]["end"] - t["new"]["start"] for t in new_in),
                   end_shift_steps_where_both_exist=sorted({t["old"]["end"] - t["new"]["end"] for t in old_in}),
                   both_hands_total_s_range=[min(t["both_hands_total_s"] for t in table), max(t["both_hands_total_s"] for t in table)],
                   ready_after_right_grasp_s_range=[min(t["ready_after_right_grasp_s"] for t in table), max(t["ready_after_right_grasp_s"] for t in table)],
                   parents_with_split_both_hands_contact=sum(len(t["both_hands_intervals_s"]) > 1 for t in table))

    # actual student entry states (R1 original-reset candidate runs) next to the views of the same parent
    m = task_env("desk", grippers="both").model
    d, cm = mujoco.MjData(m), ContactModel(m)
    spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    arm = list(rc.ARM)

    def frames_of(states, lo, hi):
        out = []
        for k in range(lo, hi):
            mujoco.mj_setState(m, d, states[k], spec)
            mujoco.mj_forward(m, d)
            f = lr.frames(m, d, cm)
            out.append((np.array(f["q_state"]), np.array(f["block_in_left_site_mm"]), np.array(f["block_in_right_site_mm"])))
        return out

    def nearest(entry, states, lo, hi, t0):
        if lo is None or hi is None or hi <= lo:
            return None
        fr = frames_of(states, lo, hi)
        q = np.array([f[0] for f in fr])
        dist = np.abs(q[:, arm] - entry[0][arm]).max(1)
        k = int(dist.argmin())
        dq = q[k] - entry[0]
        return dict(view_s=[round(t0 + lo * td.DT, 2), round(t0 + (hi - 1) * td.DT, 2)], nearest_s=round(t0 + (lo + k) * td.DT, 2), arm_q_max_abs_diff_rad=round(float(dist[k]), 4),
                    left_arm_max_abs_diff_rad=round(float(np.abs(dq[list(td.LEFT_ARM)]).max()), 4), right_arm_max_abs_diff_rad=round(float(np.abs(dq[list(td.RIGHT_ARM)]).max()), 4),
                    gripper_opening_diff=[round(float(dq[td.LEFT_GRIPPER]), 3), round(float(dq[td.RIGHT_GRIPPER]), 3)],
                    block_in_left_site_diff_mm=round(float(np.linalg.norm(fr[k][1] - entry[1])), 2), block_in_right_site_diff_mm=round(float(np.linalg.norm(fr[k][2] - entry[2])), 2))

    by_seed = {t["seed"]: t for t in table}
    entries = {}
    for seed in SEEDS:
        result = load(R1 / "runs/full" / f"s{seed}" / "candidate/result.json")
        sw = next(s for s in result["switches"] if s["to"] == "RECEIVE")
        sb = sw["state_before"]
        entry = (np.array(sb["q_state"]), np.array(sb["block_in_left_site_mm"]), np.array(sb["block_in_right_site_mm"]))
        demo = ft.demonstration(seed)
        t = by_seed[seed]
        item = dict(student_entry_s=sw["sim_time_s"], student_entry_step=sw["step"], gripper_opening=sb["gripper_opening"], block_in_left_site_mm=sb["block_in_left_site_mm"],
                    block_in_right_site_mm=sb["block_in_right_site_mm"],
                    own_demonstration_in_R1_view=None if t["old"]["excluded"] else nearest(entry, demo["integration_state"], t["old"]["start"], t["old"]["end"], 0.0),
                    own_demonstration_in_repaired_view=None if t["new"]["excluded"] else nearest(entry, demo["integration_state"], t["new"]["start"], t["new"]["end"], 0.0),
                    corrections_in_R1_fit={})
        for e in rc.correction_entries()[0]:
            if e["seed"] == seed:
                with np.load(Path(e["path"]) / "episode.npz", allow_pickle=False) as z:
                    states = z["integration_state"].copy()
                item["corrections_in_R1_fit"][e["id"]] = nearest(entry, states, e["receive"][0], e["receive"][1], e["source_step"] * td.DT)
        entries[str(seed)] = item
    data = dict(run_id=RUN, source="stored Train69 metadata / task_check_rows / episode.npz and the R1 corrections; SIM ground truth, diagnostics only (never a policy input)",
                attached_audit_file="RECEIVE_CODE_AUDIT.json was not found on this host; its content is not used", summary=summary, corrections=corrections, entry_states=entries, train=table)
    target.write_text(json.dumps(data, indent=2))
    print(json.dumps(dict(summary=summary, seed_5000={k: by_seed[5000][k] for k in ("pick_lift_s", "right_grasp_s", "both_hands_intervals_s", "ready_after_right_grasp_intervals_s",
                                                                                   "c25_boundaries_ready_s", "old", "new")},
                          corrections=corrections, entry_states=entries), indent=1))
    print(f"[written] {target}")


# ----------------------------------------------------------------------------------------------
# history: effect of the duplicated current state, on stored runs (no rollout)
# ----------------------------------------------------------------------------------------------

def history(args):
    from integration_scenes import task_env
    from sim_data_factory.task_success import evaluate_states
    target = run_dir() / "history_duplicate.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    run_dir().mkdir(parents=True, exist_ok=True)
    m = task_env("desk", grippers="both").model
    cases = []
    for cond, n in (("CONTROL", 1), ("CONTROL", 2), ("RECOVERY", 1), ("RECOVERY", 2)):
        p = rc.LIVE / cond / f"run{n}"
        sw = load(p / "metadata.json")["composition_result"]["switch"]
        cases.append((f"live {cond}{n}", p, None, [dict(step=sw["step"], to="B")]))
    for p in sorted((FACTORY_ROOT / "evaluation/pre-gate-check-20261005-01/runs").glob("run*")):
        sw = load(p / "result.json")["switch"]
        cases.append((f"pre-gate {p.name}", p, None, [dict(step=sw["step"], to="B")]))
    for p in sorted((R1 / "runs").glob("*/*/*")):
        r = load(p / "result.json")
        restore = None
        if r["kind"] == "local":
            name, path, step, seed, _ = next(s for s in rc.LOCAL_STATES if s[0] == r["state"])
            restore = rc.stored_state(path, step)
        cases.append((f"R1 {r['kind']} {r.get('state') or r['seed']} {r['chain_name']}", p, restore, [dict(step=s["step"], to=s["to"]) for s in r["switches"]]))
    out = []
    for label, p, restore, switches in cases:
        if not switches:
            continue
        with np.load(p / "episode.npz", allow_pickle=False) as a:
            states, sim_time = a["integration_state"].copy(), a["sim_time"].copy()
        step0 = 0 if restore is None else restore["step"]
        prefix = np.empty((0, states.shape[1])) if restore is None else restore["states"][:-1]
        for sw in switches:
            own = states[:sw["step"] - step0 + 1]
            once = history_states(prefix, own)
            twice = np.concatenate([prefix, own, own[-1:]])
            need = "right_grasp" if sw["to"] == "B" else "pick_lift"
            e_once = evaluate_states(m, once, control_dt=td.DT)["events"]
            e_twice = evaluate_states(m, twice, control_dt=td.DT)["events"]
            out.append(dict(run=label, switch_to=sw["to"], step=sw["step"], sim_time_s=round(sw["step"] * td.DT, 2), samples_once=len(once), samples_as_legacy=len(twice),
                            last_two_legacy_time_stamps_equal=bool(twice[-1][0] == twice[-2][0]), required_event=need,
                            event_with_duplicate=e_twice.get(need), event_counted_once=e_once.get(need),
                            switch_condition_holds_when_counted_once=need in e_once, depended_on_duplicate=need in e_twice and need not in e_once))
    recorder = dict(n_plus_one=True, evidence="EpisodeRecorder.append stores the observation state once and then every next_state; load_episode enforces len(states) == transitions + 1; "
                                              "core.rollout appends after the step, so rec.states[-1] is the current state at the next gate check",
                    recorder_sha256=sha(FACTORY_ROOT / "source/production_source_g000/sim_data_factory/recorder.py"),
                    checker_sha256=sha(FACTORY_ROOT / "source/production_source_g000/sim_data_factory/task_success.py"),
                    checker_effect="evaluate_states adds control_dt to its confirmation counters per row, so a second row of the same time adds one extra 0.02 s to "
                                   "left_grasp / pick_lift / right_grasp / handover confirmation at the last sample only")
    data = dict(run_id=RUN, recorder=recorder, switches_checked=len(out), depended_on_duplicate=[o for o in out if o["depended_on_duplicate"]], switches=out,
                note="a stored switch that depended on the duplicate would have been taken one or more control steps later; the stored results themselves are not changed or invalidated")
    target.write_text(json.dumps(data, indent=2, default=str))
    print(json.dumps(dict(switches_checked=len(out), depended_on_duplicate=[(o["run"], o["switch_to"], o["sim_time_s"]) for o in out if o["depended_on_duplicate"]],
                          equal_time_stamps=all(o["last_two_legacy_time_stamps_equal"] for o in out)), indent=1))
    print(f"[written] {target}")


# ----------------------------------------------------------------------------------------------
# run: six original-reset rollouts under a verified identity
# ----------------------------------------------------------------------------------------------

def identity():
    import frozen_transfer_check as ft
    models = {k: {f: sha(rc.MODELS[k] / f) for f in ("model.safetensors", "config.json")} | dict(normalization=sha(rc.MODELS[k].parent / "normalization.npz")) for k in ("START", "RECEIVE", "B")}
    resets = {}
    for seed in SEEDS:
        meta = ft.demonstration(seed)["metadata"]
        resets[str(seed)] = hashlib.sha256(json.dumps([meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"]], sort_keys=True).encode()).hexdigest()
    body = dict(run_id=RUN, models=models, sources={p: sha(FACTORY_ROOT / p) for p in SOURCES}, resets=resets, contracts=CONTRACT_TEXT, chain=list(CHAIN),
                execution=dict(prefix=PREFIX, arm_bound_policy="project", ensemble="off", horizon_steps=rc.HORIZON, torch_seed=4100))
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def case_dir(ident, seed, name):
    return run_dir() / "runs" / ident["sha256"][:12] / f"s{seed}" / name


def complete(out, ident, seed, name):
    """A stored case is reused only if it is whole and was produced under exactly this identity."""
    try:
        r = load(out / "result.json")
        meta = load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (r.get("identity_sha256") == ident["sha256"] and r.get("case") == [seed, name] and meta.get("complete") and r.get("boundary_contract") == CONTRACTS[name]
          and all((out / f).exists() for f in ("episode.npz", "action_trace.json", "task_check_rows.json")))
    return r if ok else None


def contract_checks(r):
    """From the run's own trace: C25 consumption, anchors, chunk discard at a switch, world state untouched."""
    trace = load(Path(r["path"]) / "action_trace.json")
    irregular = []
    for a, b in zip(trace, trace[1:]):
        same = a["policy"] == b["policy"]
        if same and not (b["selected_chunk_index"] == a["selected_chunk_index"] + 1 or (a["selected_chunk_index"] == PREFIX - 1 and b["selected_chunk_index"] == 0)):
            irregular.append(round(b["time"], 2))
        if not same and b["selected_chunk_index"] != 0:
            irregular.append(round(b["time"], 2))
    return dict(c25_consumption_regular=not irregular, irregular_at_s=irregular[:5], max_chunk_index_used=max(t["selected_chunk_index"] for t in trace),
                switches=[dict(to=s["to"], step=s["step"], local_control_index=s.get("local_control_index"), at_c25_boundary=s.get("at_c25_boundary"),
                               chunk_steps_discarded=s.get("chunk_steps_discarded"), world_state_unchanged=s["world_state_unchanged_by_switch"],
                               successor_first_chunk_index=s.get("first_command_after", {}).get("chunk_index"),
                               successor_anchor_equals_measured=s.get("first_command_after", {}).get("anchor_equals_measured")) for s in r["switches"]])


def one(args):
    ident = identity()
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    out = case_dir(ident, args.seed, args.contract)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    result = rc.execute(out, args.seed, CHAIN, None, f"seed {args.seed} {args.contract}", contract=CONTRACTS[args.contract])
    rows = load(out / "task_check_rows.json")
    grasp = result["events"].get("right_grasp")
    ready = None if grasp is None else next((r["time"] for r in rows if r["time"] >= grasp - 1e-9 and hands(r)[1]), None)
    result.update(kind="full", case=[args.seed, args.contract], contract_name=args.contract, identity_sha256=ident["sha256"],
                  semantic_ready_s=None if ready is None else round(ready, 2), contract_checks=contract_checks(result))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(seed=args.seed, contract=args.contract, receive=result["receive_entry_s"], right_grasp=grasp, ready=result["semantic_ready_s"], gate=result["gate_s"],
                          termination=result["termination"], last_s=result["last_completed_state_sim_s"], run_valid=result["run_valid"], task_success=result["task_success"],
                          final_support=result["final_support"]), default=str))


def table_row(r):
    fault = r["first_fault"]
    text = None if fault is None else f"{fault['time']:.2f} s {fault['reason']} {fault.get('forbidden_contacts') or fault.get('detail') or ''}".strip()
    if fault is None and not r["task_success"]:
        text = f"no fault; ended at 40 s with {r['final_support']}"
    entry = next((s for s in r["switches"] if s["to"] == "RECEIVE"), None)
    return dict(seed=r["seed"], contract=r["contract_name"], pick_lift_s=r["pick_lift_s"], receive_entry_s=r["receive_entry_s"],
                entry_state=None if entry is None else dict(gripper_opening=entry["state_before"]["gripper_opening"], block_in_left_site_mm=entry["state_before"]["block_in_left_site_mm"],
                                                            block_in_right_site_mm=entry["state_before"]["block_in_right_site_mm"]),
                right_grasp_s=r["right_grasp_s"], semantic_ready_s=r["semantic_ready_s"], b_switch_s=r["gate_s"], failure_before_gate=None if r["gate_reached"] else text,
                failure_after_gate=text if r["gate_reached"] else None, handover_s=r["handover_s"], run_valid=r["run_valid"], final_support=r["final_support"], task_success=r["task_success"],
                projection={k: dict(steps=v["projected_steps"], of=v["dispatched"], longest_s=v["longest_s"], max_raw_excess_rad=round(v["max_raw_excess_rad"], 4), per_axis=v["per_axis"])
                            for k, v in r["projection"].items()}, contract_checks=r["contract_checks"])


def pair_difference(a, b):
    """How two runs of one seed differ before any contract can act (render / prefix variation; not a controlled pair)."""
    out = {}
    with np.load(Path(a["path"]) / "episode.npz", allow_pickle=False) as x, np.load(Path(b["path"]) / "episode.npz", allow_pickle=False) as y:
        sa, sb = x["integration_state"], y["integration_state"]
        n = min(len(sa), len(sb))
        same = np.array([np.array_equal(sa[i], sb[i]) for i in range(n)])
    out["first_state_difference_step"] = None if same.all() else int(np.argmin(same))
    ea, eb = (next((s for s in r["switches"] if s["to"] == "RECEIVE"), None) for r in (a, b))
    if ea and eb:
        out.update(receive_entry_step=[ea["step"], eb["step"]],
                   entry_q_max_abs_diff_rad=float(np.abs(np.array(ea["state_before"]["q_state"]) - np.array(eb["state_before"]["q_state"])).max()))
    return out


def run(args):
    for name in ("bounds_table.json", "history_duplicate.json"):
        if not (run_dir() / name).exists():
            raise FileNotFoundError(f"{name} is missing: run `bounds` and `history` first")
    ident = identity()
    r1 = load(R1 / "protocol.json")["frozen"]
    if any(ident["models"][k] != r1[k] for k in ("START", "B")):
        raise RuntimeError("START / B differ from the files receive-candidate-20261005-01 froze")
    (run_dir() / "logs").mkdir(parents=True, exist_ok=True)
    proto = run_dir() / "protocol.json"
    if not proto.exists():
        proto.write_text(json.dumps(dict(
            run_id=RUN, name=rc.LABEL + " / boundary contract comparison, fixed weights", fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"), identity=ident,
            cases=[[s, n] for s in SEEDS for n in ORDER[s]], order="per parent, alternating which contract runs first; fixed here before any run",
            budget="six runs, one each, whatever the results; no repeat", success="whole run valid + handover + final right-hand-only support",
            comparison="each run starts from its own original reset in its own process; the two runs of a parent are not a controlled pair (render / prefix variation is recorded)",
            training=False, teacher_collection=False, val_test=False, hardware_access=False), indent=2))
    status = []
    for seed in SEEDS:
        for name in ORDER[seed]:
            out = case_dir(ident, seed, name)
            if complete(out, ident, seed, name) is not None:
                status.append(dict(case=[seed, name], status="completed (reused: same identity, complete files)"))
                print(f"[reused under identity {ident['sha256'][:12]}] {seed} {name}")
                continue
            if out.exists():
                status.append(dict(case=[seed, name], status="NOT RUN: an incomplete folder of this identity exists; it is neither reused nor overwritten", path=str(out)))
                print(f"[not run] {seed} {name}: incomplete folder {out}")
                continue
            done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--seed", str(seed), "--contract", name, "--identity", ident["sha256"]], capture_output=True, text=True)
            (run_dir() / "logs" / f"{ident['sha256'][:12]}_s{seed}_{name}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
            ok = done.returncode == 0 and complete(out, ident, seed, name) is not None
            status.append(dict(case=[seed, name], returncode=done.returncode, status="completed" if ok else "EXECUTION ERROR (not a task result)",
                               **({} if ok else dict(stderr_tail=done.stderr[-600:]))))
            line = [x for x in done.stdout.splitlines() if x.startswith("{")]
            print(line[-1] if ok and line else f"[EXECUTION ERROR, not a task failure] {seed} {name}: returncode {done.returncode}: {done.stderr[-300:]}", flush=True)
    results = [r for seed in SEEDS for name in ORDER[seed] if (r := complete(case_dir(ident, seed, name), ident, seed, name)) is not None]
    by = {tuple(r["case"]): r for r in results}
    pairs = {str(s): pair_difference(by[(s, "LEGACY_RECEIVE")], by[(s, "REPAIRED_RECEIVE")]) for s in SEEDS if (s, "LEGACY_RECEIVE") in by and (s, "REPAIRED_RECEIVE") in by}
    after = identity()
    (run_dir() / "results.json").write_text(json.dumps(dict(
        run_id=RUN, note=f"{rc.LABEL}; fixed weights; switches read SIM ground truth; Train parents; one run per case, not a success rate; kept apart from the 16 runs of {rc.RUN}",
        identity_sha256=ident["sha256"], identity_unchanged_during_run=after["sha256"] == ident["sha256"], planned_cases=6, completed_cases=len(results),
        execution_status=status, table=[table_row(r) for r in results], pair_difference_not_controlled=pairs, results=results), indent=2, default=str))
    for row in (table_row(r) for r in results):
        print(json.dumps({k: row[k] for k in ("seed", "contract", "pick_lift_s", "receive_entry_s", "right_grasp_s", "semantic_ready_s", "b_switch_s", "failure_before_gate", "failure_after_gate",
                                              "handover_s", "final_support", "task_success")}, default=str))
    errors = [s for s in status if not s["status"].startswith("completed")]
    print(f"[written] {run_dir() / 'results.json'}: {len(results)} of 6 cases completed, {len(errors)} execution error(s) / not run; identity {ident['sha256'][:12]}")


def video(args):
    import recovery_trial as rt
    ident = identity()
    out = run_dir() / "videos" / f"seed5000_legacy_repaired_{ident['sha256'][:12]}.mp4"
    if out.exists():
        print(f"[exists] {out}")
        return
    rows = []
    for name in ("LEGACY_RECEIVE", "REPAIRED_RECEIVE"):
        r = complete(case_dir(ident, 5000, name), ident, 5000, name)
        if r is None:
            print(f"[unavailable] seed 5000 {name}: no completed case under this identity; the row is not replaced")
            continue
        row = rt.teacher_row(f"{name}: {r['chain']} (fixed weights)", Path(r["path"]))
        ev = dict(r["events"])
        ev.update({f"SWITCH->{s['to']}": s["sim_time_s"] for s in r["switches"]})
        row["events"] = dict(sorted(ev.items(), key=lambda kv: kv[1]))
        row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}" + ("" if r["gate_reached"] else " (B NOT REACHED)"),
                           "VALID HANDOVER, right hand holds" if r["task_success"] else f"no valid handover; final support: {r['final_support']}")
        rows.append(row)
    if rows:
        rt.render(rows, out, f"{rc.LABEL} / seed 5000 original reset / boundary contract", 2000, False, right_gripper=True)
        print(f"[written] {out}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("bounds", "history", "run", "video"):
        sub.add_parser(name)
    o = sub.add_parser("one")
    o.add_argument("--seed", type=int, choices=SEEDS, required=True)
    o.add_argument("--contract", choices=tuple(CONTRACTS), required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    dict(bounds=bounds, history=history, run=run, one=one, video=video)[args.command](args)


if __name__ == "__main__":
    main()
