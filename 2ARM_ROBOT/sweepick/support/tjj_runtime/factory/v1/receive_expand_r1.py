#!/usr/bin/env python3
"""RECEIVE-EXPAND R1 / work6: collect teacher RECEIVE corrections from 48 further Train parents, train one RECEIVE copy
(DATA) on a derived manifest, and compare it with the stored RECEIVE_BRIDGE_CTRL on fixed student runs (SIM only; no
hardware; no Val / Test in collection, training or normalization).

Execution code (part of the run identity). Tables, frames and the hand-off ZIP are made by receive_expand_r1_report.py.

  protocol  the 48 sources and their order, the training setting and the evaluation bundles, fixed before any run
  collect   48 mixed runs: START_R1_E from the original reset, the original teacher takes RECEIVE at the G switch,
            frozen RELEASE_B ends (one attempt per parent)
  manifest  derived training manifest: the two POST-C0 pilot corrections + the new ones with a valid readiness prefix
            AND a valid whole mixed handover; source-fit parents fixed here, before training
  plan      DATA = the first six slots of RECEIVE_BRIDGE_CTRL's own sample plan + two slots from the new pool
  train     RECEIVE_DATA: RECEIVE_FROZEN weights + its stored AdamW state, 5,000 updates on CUDA
  evaluate  46-54 student runs, CTRL vs DATA under the G contract (Train9, source-fit, development Val10, frozen-START probe)
  all       every step that is not complete yet; two worker processes for collection and evaluation

Switches read SIM ground truth: student-action / SIM-oracle-gated. No default model or contract is changed.
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import train_rollout_diagnostic as td   # noqa: E402
import receive_candidate as rc   # noqa: E402
import receive_boundary_repair as rb   # noqa: E402
import receive_entry_r2 as r2   # noqa: E402
import receive_r3_recheck as rk   # noqa: E402
import start_pick_d1 as d1   # noqa: E402
import start_pick_r1 as p1   # noqa: E402
import receive_bridge_r1 as br   # noqa: E402
import constraint_causal_c0 as c0   # noqa: E402
import post_c0_pilot_r1 as pr1   # noqa: E402
import server_s0 as s0   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "receive-expand-r1-work6-20261006-01"
TITLE = "RECEIVE-EXPAND R1 / work6 / 48-parent correction collection, RECEIVE_DATA vs stored CTRL"
LABEL = "student-action / SIM-ORACLE-GATED THREE-POLICY / RECEIVE-EXPAND R1 work6 (contract G)"
WORKERS = 2
PILOT12 = (5012, 5019, 5020, 5022, 5023, 5030, 5102, 5109, 5110, 5111, 5112, 5116)
G000 = (5031, 5032, 5033, 5035, 5038, 5039, 5040, 5043, 5048, 5049, 5051, 5052, 5054, 5062, 5067, 5068, 5074, 5078, 5089, 5095, 5097)
G001 = (5119, 5121, 5125, 5128, 5139, 5140, 5141, 5142, 5145, 5149, 5152, 5156, 5158, 5161, 5165, 5166, 5167, 5168, 5176, 5178, 5183, 5187, 5190, 5193, 5194, 5195, 5198)
UPDATES, BATCH, SEED, OLD_SLOTS = 5000, 8, 4100, 6
CHECKPOINT = "step-005000"
NEW_BASE = 500000
PILOT_STAGING = pr1.run_dir() / "staging_manifest.json"
BRIDGE_PLAN = br.run_dir() / "sample_plan.npz"
SOURCES = pr1.SOURCES + ("factory/v1/receive_expand_r1.py",)
PROFILE_FIRST = 2

load, sha, array_sha = rk.load, rk.sha, rk.array_sha


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def training_dir():
    return FACTORY_ROOT / "training" / f"{RUN}-data"


def sources48():
    """Train69 - Train9 - pilot12 from the pinned manifest, in the collection order (generation queues taken in turn)."""
    rows = [(i, e) for i, e in enumerate(d1.manifest()) if e["split"] == "train" and e["training_eligible"]]
    rest = {g: sorted([(e["seed"], i, e) for i, e in rows if e["generation"] == g and e["seed"] not in p1.TRAIN9 and e["seed"] not in PILOT12]) for g in ("G000", "G001")}
    got = {g: tuple(s for s, _, _ in rest[g]) for g in rest}
    differs = {g: dict(only_in_manifest=sorted(set(got[g]) - set(want)), only_in_instruction=sorted(set(want) - set(got[g]))) for g, want in (("G000", G000), ("G001", G001)) if got[g] != want}
    order, a, b = [], list(rest["G000"]), list(rest["G001"])
    while a or b:
        if a:
            order.append(a.pop(0))
        if b:
            order.append(b.pop(0))
    out = []
    for seed, i, e in order:
        meta = load(Path(e["path"]) / "metadata.json")
        out.append(dict(seed=seed, manifest_index=i, generation=e["generation"], split=e["split"], parent_physics_group=e["parent_physics_group"],
                        reset_sha256=hashlib.sha256(json.dumps([meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"]], sort_keys=True).encode()).hexdigest()))
    return out, differs


def identity(stage):
    """stage 'collect': fixed before any run. stage 'evaluate': adds the trained DATA model, the manifest and the plan."""
    import arm_output_trial as trial
    models = c0.bridge_models()
    srcs, _ = sources48()
    body = dict(run_id=RUN, stage=stage, models=models, manifest_sha256=p1.MANIFEST_SHA, collection_sources=[s["seed"] for s in srcs], workers=WORKERS, sources={p: sha(FACTORY_ROOT / p) for p in SOURCES},
                contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"], teacher=dict(version="original", file=str(pr1.TEACHER_SOURCE), sha256=sha(pr1.TEACHER_SOURCE)),
                execution=dict(start_to_receive="G: the unchanged pick_lift + left-hold condition on every 50 Hz step (the C0 / POST-C0 option)", receive_to_release="repaired 50 Hz readiness",
                               chunk=50, prefix=rc.PREFIX, control_hz=round(1 / td.DT), horizon_steps=rc.HORIZON, torch_seed=4100, arm_bound_policy="project", ensemble="off", smoothing="off"),
                backend=trial.environment_record(), host=s0.host(), environment={k: os.environ.get(k) for k in rk.ENV_KEYS})
    if stage == "evaluate":
        run = trained()
        if run is None:
            raise RuntimeError("RECEIVE_DATA of this plan is not trained")
        body["models"]["RECEIVE_DATA"] = dict(path=str(training_dir() / CHECKPOINT), model=run["model_sha256"], config=sha(training_dir() / CHECKPOINT / "config.json"),
                                              normalization=sha(training_dir() / "normalization.npz"), training_run_sha256=sha(training_dir() / "run.json"))
        body.update(training_manifest_sha256=sha(run_dir() / "training_manifest.json"), sample_plan_sha256=sha(run_dir() / "sample_plan.npz"), evaluation_cases=eval_cases())
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def protocol(args=None):
    target = run_dir() / "protocol.json"
    if target.exists():
        return load(target)
    run_dir().mkdir(parents=True, exist_ok=True)
    srcs, differs = sources48()
    ident = identity("collect")
    decision = load(pr1.run_dir() / "decision.json")
    if decision["contract"] != "G" or decision["teacher"] != "original":
        raise RuntimeError("POST-C0 decision is not G / original")
    record = dict(
        run_id=RUN, name=TITLE, label=LABEL, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"), hardware_access=False, val_test_used_for_collection_or_training=False, identity=ident,
        basis=dict(post_c0_decision=str(pr1.run_dir() / "decision.json"), contract="G", teacher="original", note="explicit options of this run's collection and evaluation; no default changes"),
        collection=dict(sources=srcs, count=len(srcs), list_differs_from_instruction=differs, attachment="collection_sources48.json was not among the received files; the list is rebuilt from the pinned manifest "
                        "(Train69 - Train9 - pilot12) and compared with the seeds written in the instruction", order="generation queues in ascending seed taken in turn (G000, G001, ...), the longer queue continues; "
                        "the two workers take alternate cases of this list", rule="one mixed attempt per parent: student START_R1_E + original teacher RECEIVE + student RELEASE_B; no retry, no replacement, no Val / Test"),
        pool=dict(main="readiness prefix checker-valid AND whole mixed handover strict-valid, contract G, original teacher", reuse="POST-C0 pilot 5109 and 5112 (their files and rows are re-read and checked)",
                  apart="prefix valid but RELEASE_B failed: kept and counted, not trained on", never="POST-C0 teacher_local controls (5002, 5098): a stored C25 entry with the derived teacher; earlier Bridge corrections "
                  "stay in the old replay only"),
        training=dict(model="RECEIVE_DATA", start=ident["models"]["RECEIVE_FROZEN"], optimizer_state=str(Path(ident["models"]["RECEIVE_FROZEN"]["path"]) / "training_state.pt"), updates=UPDATES, batch=BATCH, seed=SEED,
                      slots=f"0-{OLD_SLOTS - 1}: the same slots of RECEIVE_BRIDGE_CTRL's own sample plan (same sample, same position); {OLD_SLOTS}-{BATCH - 1}: new pool, parent first (joined fixed-RNG permutations), "
                            "then its entry state, then a uniform teacher timestep", ctrl="RECEIVE_BRIDGE_CTRL is reused as stored; it is not trained again",
                      final=CHECKPOINT, unchanged="architecture, loss (B horizon weights), KL, AdamW lr 1e-4 / backbone 1e-5 / wd 1e-4 / clip 1.0, float32, no AMP / TF32 change / compile / augmentation",
                      note="5,000 updates and 75 / 25 are this run's preset; a stored control against a new expansion, not a paired training and not a pure data-count effect"),
        evaluation=dict(chain="START_R1_E -> RECEIVE_{CTRL|DATA} -> RELEASE_B under G; the probe uses START_FROZEN", train9=list(p1.TRAIN9), val10=list(p1.VAL10), probe=[5000, 5008],
                        source_fit="5109, 5112 and at most four new main-pool parents: the two lowest seeds of G000 and of G001, empty places filled by the remaining ones in generation / seed order; "
                                   "fixed from teacher admission only, before DATA is trained", order="within a bundle the first model alternates per parent (CTRL first, then DATA first, ...)",
                        rule="original reset, one run each, absolute 40 s or the first existing fault; no retry; bundles are separate tables; nothing from S0 / Bridge / C0 / POST-C0 is added"),
        budget="physical runs: 48 mixed + 46-54 student", parallel=dict(workers=WORKERS, child="4 torch threads, the POST-C0 child settings"),
        gpu="ACT parameters / inputs / outputs on CUDA in every child (recorded); a profiled forward in the first cases of each batch and a profiled stretch of the DATA training")
    target.write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(identity=ident["sha256"][:12], sources=len(srcs), differs=differs, first=[s["seed"] for s in srcs[:6]])))
    return record


# ----------------------------------------------------------------------------------------------
# One child process: a mixed collection run or a student run
# ----------------------------------------------------------------------------------------------

def spec_of(case):
    kind, *rest = case.split(":")
    if kind == "collect":
        return dict(case=case, kind="collect", seed=int(rest[0]), start="R1E", receive="TEACHER", bundle="collect", gate50=True)
    bundle, seed, receive = rest
    return dict(case=case, kind="evaluate", bundle=bundle, seed=int(seed), start="FROZEN" if bundle == "probe" else "R1E", receive=receive, gate50=True)


def case_dir(ident, case):
    return run_dir() / "runs" / f"{ident['stage']}_{ident['sha256'][:12]}" / case.replace(":", "_")


def complete(ident, case):
    out = case_dir(ident, case)
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = r.get("identity_sha256") == ident["sha256"] and r.get("expand_case") == case and meta.get("complete") and "cuda" in r and (out / "episode.npz").exists()
    return r if ok else None


def one(args):
    from integration_scenes import task_env
    from sim_data_factory.task_success import evaluate_states
    spec = spec_of(args.case)
    ident = identity(spec["kind"])
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    out = case_dir(ident, args.case)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    started, timers = time.time(), {}
    log, _ = pr1.install_runtime(dict(gate50=True), timers)
    models = ident["models"]
    rc.MODELS["START"] = Path(models["START_FROZEN" if spec["start"] == "FROZEN" else "START_R1_E"]["path"])
    receive_name = dict(CTRL="RECEIVE_BRIDGE_CTRL", DATA="RECEIVE_DATA").get(spec["receive"])
    if receive_name:
        rc.MODELS["RECEIVE"] = Path(models[receive_name]["path"])
    loaded, original = {}, rc.load_policy

    def remember(name):
        if name == "RECEIVE" and spec["receive"] == "TEACHER":
            got = (pr1.TeacherSeat("original", None, ("insert",)), rc.stats(), dict(checkpoint="teacher:original", model_sha256=None, config_sha256=None, normalization_sha256=None))
        else:
            got = original(name)
        loaded[name] = got
        return got

    rc.load_policy = remember
    try:
        result = rc.execute(out, spec["seed"], ("START", "RECEIVE", "B"), None, f"{LABEL}: {args.case}", contract="repaired")
    finally:
        rc.load_policy = original
    got = {k: v[2]["model_sha256"] for k, v in loaded.items()}
    want = dict(START=models["START_FROZEN" if spec["start"] == "FROZEN" else "START_R1_E"]["model"], RECEIVE=None if receive_name is None else models[receive_name]["model"], B=models["RELEASE_B"]["model"])
    if any(got[k] != want[k] for k in got):
        raise RuntimeError(f"a loaded policy is not the intended one: {got}")
    rows, trace = load(out / "task_check_rows.json"), load(out / "action_trace.json")
    with np.load(out / "episode.npz", allow_pickle=False) as a:
        states, q, cmd = a["integration_state"].copy(), a["q_state"].copy(), a["q_cmd"].copy()
    model = task_env("desk", grippers="both").model
    entry = next((s for s in result["switches"] if s["to"] == "RECEIVE"), None)
    gate = next((s for s in result["switches"] if s["to"] == "B"), None)
    events, fault = result["events"], result["first_fault"]
    strict = bool(result["task_success"]) and log["faults"] == 0
    stage = None if strict else ("START" if entry is None else "RECEIVE" if gate is None else "RELEASE_B")
    kind = None if strict else ("teacher_failure" if (fault or {}).get("reason") == "teacher_failure" else "fault" if fault is not None else "task not completed within the 40 s horizon")
    names = lambda pair: [f"{g} ({model.body(int(model.geom_bodyid[model.geom(g).id])).name})" if not g.startswith("geom") else f"{g} ({model.body(int(model.geom_bodyid[int(g[4:])])).name})" for g in pair]
    contact_bodies = None if fault is None or not fault.get("forbidden_contacts") else [names(p) for p in fault["forbidden_contacts"]]
    teacher_part = None
    if spec["receive"] == "TEACHER":
        t_rows = [(k, t) for k, t in enumerate(trace) if str(t.get("controller", "")).startswith("teacher")]
        ready_valid, prefix = False, None
        if entry is not None and gate is not None:
            a_, b_ = entry["step"], gate["step"]
            chk = evaluate_states(model, states[:b_ + 1], q[:b_ + 1].astype(np.float64), control_dt=td.DT)
            ready_valid = bool(chk["valid"])
            own = [k for k, _ in t_rows]
            prefix = dict(absolute_steps=[a_, b_], absolute_s=[round(a_ * td.DT, 2), round(b_ * td.DT, 2)], rows_in_episode=[a_, b_], observations=b_ - a_, checker_valid_up_to_readiness=ready_valid,
                          failures=chk["failures"][:3], entry_state_sha256=array_sha(states[a_]),
                          teacher_rows_are_exactly_the_prefix=bool(own == list(range(a_, b_))),
                          teacher_q_cmd_equals_episode_q_cmd=bool(np.allclose(np.array([trace[k]["q_cmd"] for k in range(a_, b_)]), cmd[a_:b_], atol=0, rtol=0)),
                          alignment="row k of episode.npz q_state / q_cmd / cameras is the observation and the command of absolute control step k")
        teacher_part = dict(version="original", entered=entry is not None, commands=len(t_rows), phases=list(dict.fromkeys(t["teacher_phase"] for _, t in t_rows)),
                            planner_wall_s=round(sum(t["planner_wall_s"] for _, t in t_rows), 2), teacher_ready_valid=ready_valid, prefix=prefix, mixed_full_handover_valid=strict,
                            failure=fault if (fault or {}).get("reason") == "teacher_failure" else None)
    upto = len(states) if entry is None else entry["step"] + 1
    profile = bool(args.profile)
    cuda = pr1.cuda_evidence(loaded, stored_case=out if profile else None)
    inference = float(sum(t.get("inference_wall_s", 0.0) for t in trace))
    planner = 0.0 if teacher_part is None else teacher_part["planner_wall_s"]
    timing = dict(wall_s=round(time.time() - started, 1), act_inference_s=round(inference, 2), render_observe_s=round(timers.get("render_observe_s", 0.0), 2), planner_s=planner,
                  checker_s=round(timers.get("checker_s", 0.0), 2), rollout_total_s=round(timers.get("rollout_total_s", 0.0), 2),
                  physics_and_recording_s=round(timers.get("rollout_total_s", 0.0) - timers.get("render_observe_s", 0.0) - timers.get("student_next_command_s", 0.0) - planner, 2),
                  student_pre_post_s=round(timers.get("student_next_command_s", 0.0) - inference, 2), inference_calls=sum(1 for t in trace if t.get("inference_wall_s", 0.0) > 0), profiled=profile)
    last_ok = len(states) - 1
    result.update(job=RUN, classification=LABEL, expand_case=args.case, spec=spec, identity_sha256=ident["sha256"], strict_valid_handover=strict, first_failure_stage=stage, first_failure_kind=kind,
                  first_fault_bodies=contact_bodies, last_normal_state=dict(step=last_ok, sim_time_s=round(float(states[last_ok][0]), 2), state_sha256=array_sha(states[last_ok])),
                  teacher=teacher_part, handover_event=result["handover_s"] is not None, completed_40s=result["termination"] == "horizon_completed", block_contact=p1.contact_summary(rows, result),
                  pre_receive=dict(states=int(upto), q_sha256=array_sha(q[:upto]), receive_entry_step=None if entry is None else entry["step"]),
                  handover_state_at_receive_entry=None if entry is None else br.handover_frame(model, states[entry["step"]]),
                  ready_s=None if gate is None else gate["sim_time_s"], cuda=cuda, timing=timing,
                  child_settings=dict(rk.child_settings({k: v[0] for k, v in loaded.items() if not isinstance(v[0], pr1.TeacherSeat)}), host=s0.host(), workers=WORKERS))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(case=args.case, pick=events.get("pick_lift"), entry=None if entry is None else entry["sim_time_s"], ready=None if gate is None else gate["sim_time_s"], handover=events.get("handover"),
                          strict=strict, stage=stage, kind=kind, teacher=None if teacher_part is None else teacher_part["teacher_ready_valid"], cuda=all(p["all_on_cuda"] for p in cuda["policies"].values()),
                          wall=timing["wall_s"]), default=str)[:600])


def batch(stage, cases, name):
    ident = identity(stage)
    (run_dir() / "logs").mkdir(exist_ok=True)
    marker = run_dir() / f"identity_{stage}.json"
    if marker.exists():
        if load(marker)["sha256"] != ident["sha256"]:
            raise RuntimeError(f"identity_{stage}.json holds another identity (model, source, plan or host changed): nothing is run, nothing is overwritten")
    else:
        marker.write_text(json.dumps(dict(ident, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z")), indent=2, default=str))
    groups = [cases[0::2], cases[1::2]]
    started = time.time()

    def worker(group_index):
        out = []
        for n, case in enumerate(groups[group_index]):
            if complete(ident, case) is not None:
                continue
            folder = case_dir(ident, case)
            if folder.exists():
                kept = run_dir() / "attempts" / f"{folder.name}_{time.strftime('%Y%m%dT%H%M%S')}"
                kept.parent.mkdir(exist_ok=True)
                folder.rename(kept)
                out.append(dict(case=case, kept=str(kept), note="incomplete folder of an earlier attempt"))
            t0 = time.time()
            done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--case", case, "--identity", ident["sha256"]] + (["--profile"] if n < PROFILE_FIRST // 2 + (group_index == 0 and PROFILE_FIRST % 2) else []),
                                  capture_output=True, text=True)
            (run_dir() / "logs" / f"{folder.name}_{time.strftime('%H%M%S')}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
            ok = done.returncode == 0 and complete(ident, case) is not None
            if not ok:
                out.append(dict(case=case, returncode=done.returncode, status="infrastructure_error (not a task result)", stderr_tail=done.stderr[-900:]))
            line = [x for x in done.stdout.splitlines() if x.startswith("{")]
            print((line[-1][:560] if ok and line else f"[infrastructure_error] {case}: {done.stderr[-400:]}") + f"  [{time.time() - t0:.0f}s]", flush=True)
        return out

    extra = []
    with ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for x in pool.map(worker, range(len(groups))):
            extra += x
    attempts_file = run_dir() / "attempts.json"
    attempts_file.write_text(json.dumps((load(attempts_file) if attempts_file.exists() else []) + extra, indent=2))
    done = [c for c in cases if complete(ident, c) is not None]
    status_file = run_dir() / "batch_status.json"
    status = load(status_file) if status_file.exists() else {}
    status[name] = dict(planned=len(cases), completed=len(done), missing=[c for c in cases if c not in done], workers=WORKERS, batch_wall_s=round(time.time() - started, 1),
                        sum_of_case_wall_s=round(sum(complete(ident, c)["timing"]["wall_s"] for c in done), 1), note="sum / batch is an overlap ratio, not a measured speed-up")
    status_file.write_text(json.dumps(status, indent=2))
    print(f"[{name}] {len(done)}/{len(cases)} cases; batch wall {status[name]['batch_wall_s']}s", flush=True)


def collect(args=None):
    proto = load(run_dir() / "protocol.json")
    batch("collect", [f"collect:{s['seed']}" for s in proto["collection"]["sources"]], "collect")


# ----------------------------------------------------------------------------------------------
# Derived manifest, plan, training
# ----------------------------------------------------------------------------------------------

def training_manifest(args=None):
    target = run_dir() / "training_manifest.json"
    if target.exists():
        return load(target)
    proto = load(run_dir() / "protocol.json")
    ident = load(run_dir() / "identity_collect.json")
    entries, apart, excluded = [], [], []
    for e in load(PILOT_STAGING)["entries"]:
        if e["group"] != "pilot":
            excluded.append(dict(case=e["case"], parent=e["source_parent"], reason="POST-C0 teacher_local control: stored C25 entry with the derived teacher; not a new G correction"))
            continue
        path = Path(e["path"])
        if any(sha(path / f) != v for f, v in e["files"].items()):
            raise RuntimeError(f"{e['case']}: a staged file changed")
        r = load(path / "result.json")
        if r["spec"].get("contract") != "G" or r["teacher"]["version"] != "original" or not (r["teacher"]["teacher_ready_valid"] and r["strict_valid_handover"]):
            raise RuntimeError(f"{e['case']}: not a G / original / fully valid correction")
        entries.append(dict(origin="POST-C0 R1 pilot (reused)", case=e["case"], parent=e["source_parent"], generation=next(x["generation"] for x in d1.manifest() if x["seed"] == e["source_parent"]),
                            contract="G", teacher="original", rows_in_episode=e["rows_in_episode"], absolute_steps=e["prefix_absolute_steps"], observations=e["observations"],
                            entry_state_sha256=e["entry_state_sha256"], path=str(path), files=e["files"]))
    for s in proto["collection"]["sources"]:
        r = complete(ident, f"collect:{s['seed']}")
        if r is None or r["teacher"] is None or not r["teacher"]["teacher_ready_valid"]:
            continue
        p = r["teacher"]["prefix"]
        item = dict(origin="this run", case=r["expand_case"], parent=s["seed"], generation=s["generation"], contract="G", teacher="original", rows_in_episode=p["rows_in_episode"], absolute_steps=p["absolute_steps"],
                    observations=p["observations"], entry_state_sha256=p["entry_state_sha256"], path=r["path"],
                    files={f: sha(Path(r["path"]) / f) for f in ("episode.npz", "cameras/left_wrist_rgb.npz", "cameras/right_wrist_rgb.npz", "action_trace.json")})
        (entries if r["strict_valid_handover"] else apart).append(item)
    seen = {}
    for e in entries:
        key = (e["parent"], e["entry_state_sha256"], e["teacher"], e["contract"])
        if key in seen:
            raise RuntimeError(f"duplicate correction {key}")
        seen[key] = e
    for i, e in enumerate(entries):
        e["view_key"] = NEW_BASE + i
    new = [e for e in entries if e["origin"] == "this run"]
    fit = []
    for gen in ("G000", "G001"):
        fit += sorted([e["parent"] for e in new if e["generation"] == gen])[:2]
    rest = [e["parent"] for e in sorted(new, key=lambda e: (e["generation"], e["parent"])) if e["parent"] not in fit]
    fit = sorted(fit + rest[:max(0, 4 - len(fit))])
    data = dict(run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), status="derived training manifest of this run only; the pinned Train manifest, the normalization and every default stay as they are",
                collection_identity=ident["sha256"], main_pool=entries, valid_prefix_but_release_failed=apart, excluded=excluded,
                counts=dict(parents=len({e["parent"] for e in entries}), entry_states=len({e["entry_state_sha256"] for e in entries}), demonstrations=len(entries), observations=int(sum(e["observations"] for e in entries)),
                            new_parents_this_run=len(new), reused_parents=len(entries) - len(new), by_generation={g: sum(e["generation"] == g for e in entries) for g in ("G000", "G001")},
                            apart=len(apart)),
                source_fit=dict(parents=[5109, 5112] + fit, new=fit, rule=proto["evaluation"]["source_fit"], fixed="before DATA training, from teacher admission only"))
    target.write_text(json.dumps(data, indent=2, default=str))
    print(json.dumps(dict(counts=data["counts"], source_fit=data["source_fit"]["parents"], new=[e["parent"] for e in new], apart=[e["parent"] for e in apart])))
    return data


def new_view(st):
    """The teacher rows of each main-pool episode as a view: slices of the stored arrays, nothing copied or relabelled."""
    man = load(run_dir() / "training_manifest.json")
    segments, checks = [], []
    for e in man["main_pool"]:
        path = Path(e["path"])
        if any(sha(path / f) != v for f, v in e["files"].items()):
            raise RuntimeError(f"{e['case']}: a manifest file changed")
        with np.load(path / "episode.npz", allow_pickle=False) as a:
            state, action = a["q_state"][:-1].astype(np.float32), a["q_cmd"].astype(np.float32)
        rgb = {s: np.load(path / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}
        if not (len(state) == len(action) == len(rgb["left"]) == len(rgb["right"])):
            raise ValueError(f"{path}: N+1/N or camera alignment mismatch")
        a_, b_ = e["rows_in_episode"]
        trace = load(path / "action_trace.json")
        controllers = [str(t.get("controller", t.get("policy"))) for t in trace[a_:b_]]
        ok = all(c.startswith("teacher") for c in controllers) and not str(trace[a_ - 1].get("controller", "")).startswith("teacher") and b_ - a_ == e["observations"]
        cmd_ok = bool(np.array_equal(np.array([t["q_cmd"] for t in trace[a_:b_]], dtype=np.float32), action[a_:b_]))
        if not (ok and cmd_ok):
            raise RuntimeError(f"{e['case']}: rows {a_}-{b_} are not exactly the teacher's RECEIVE commands")
        checks.append(dict(case=e["case"], parent=e["parent"], rows=[a_, b_], episode_rows=len(action), rgb_rows=len(rgb["left"]), state_rows=len(state), teacher_rows=b_ - a_, row_before_is_student=True,
                           teacher_q_cmd_equals_episode_q_cmd=cmd_ok, valid_future_targets=int(sum(min(50, b_ - a_ - t) for t in range(b_ - a_)))))
        segments.append((e["view_key"], dict(state=state, action=action, rgb=rgb), (a_, b_), dict(id=e["case"], seed=e["parent"], split="train", origin=e["origin"])))
    return rc.segment_view(st, run_dir(), segments), checks


def datasets():
    st, views, index = br.datasets()
    if any(k >= NEW_BASE for k in index):
        raise RuntimeError("view key clash")
    view, checks = new_view(st)
    views = dict({k: v for k, v in views.items() if v is not None}, expand=view)
    total = 0
    for key, ep in view.episodes.items():
        index[key] = ("expand", total, len(ep["action"]))
        total += len(ep["action"])
    return st, views, index, checks


def plan(args=None):
    target = run_dir() / "plan.json"
    if target.exists():
        return load(target)
    man = load(run_dir() / "training_manifest.json")
    if not man["main_pool"]:
        target.write_text(json.dumps(dict(run_id=RUN, status="NO_VALID_CORRECTION: nothing is trained", valid=0), indent=2))
        return load(target)
    st, views, index, checks = datasets()
    with np.load(BRIDGE_PLAN, allow_pickle=False) as a:
        ck, ct = a["ctrl_key"].copy(), a["ctrl_t"].copy()
    ctrl_run = load(br.training_dir("ctrl") / "run.json")
    with np.load(br.training_dir("ctrl") / "consumed_samples.npz", allow_pickle=False) as a:
        if not (np.array_equal(a["episode"], ck) and np.array_equal(a["timestep"], ct)):
            raise RuntimeError("the Bridge plan is not what RECEIVE_BRIDGE_CTRL consumed")
    if ck.shape != (UPDATES, BATCH) or sha(BRIDGE_PLAN) != ctrl_run["sample_plan_sha256"]:
        raise RuntimeError("unexpected Bridge sample plan")
    rng = np.random.default_rng(SEED)
    parents = sorted({e["parent"] for e in man["main_pool"]})
    by_parent = {p: [e["view_key"] for e in man["main_pool"] if e["parent"] == p] for p in parents}
    count = UPDATES * (BATCH - OLD_SLOTS)
    blocks = -(-count // len(parents))
    chosen = rng.permuted(np.tile(np.array(parents), (blocks, 1)), axis=1).reshape(-1)[:count]
    pick, u = rng.random(count), rng.random(count)
    keys = np.array([by_parent[int(p)][int(x * len(by_parent[int(p)]))] for p, x in zip(chosen, pick)])
    ts = np.floor(u * np.array([index[int(k)][2] for k in keys])).astype(np.int64)
    dk, dt = ck.copy(), ct.copy()
    dk[:, OLD_SLOTS:], dt[:, OLD_SLOTS:] = keys.reshape(UPDATES, -1), ts.reshape(UPDATES, -1)
    view = views["expand"]
    std, arm, off = view.stats["action"]["std"], list(rc.ARM), 0
    for key, ep in view.episodes.items():                                  # mask / anchor / label checks on each new prefix
        n = len(ep["action"])
        for t in (0, max(n - 60, 0), n - 1):
            item = view[off + t]
            pad = item["action_is_pad"].numpy()
            assert pad.sum() == max(0, t + 50 - n) and (not pad.any() or pad[-1]) and int(item["meta_episode"]) == key and int(item["meta_t"]) == t
            j = min(3, n - 1 - t)
            assert np.allclose(item["action"][j].numpy()[arm], (ep["action"][t + j][arm] - ep["state"][t][arm]) / std[arm], atol=1e-6)
        off += n
    assert np.array_equal(dk[:, :OLD_SLOTS], ck[:, :OLD_SLOTS]) and (dk[:, OLD_SLOTS:] >= NEW_BASE).all() and (ck < NEW_BASE).all()
    np.savez_compressed(run_dir() / "sample_plan.npz", data_key=dk, data_t=dt, ctrl_key=ck, ctrl_t=ct)
    names = dict(train="Train69 receiving view", existing="earlier corrections (re-cut)", actual_entry="R2 actual-entry corrections", onset="R3 onset correction", bridge="Bridge corrections", expand="new G corrections")

    def exposure(k, t):
        per_view, per_parent, valid = {}, {}, 0
        key_parent = {e["view_key"]: e["parent"] for e in man["main_pool"]}
        for a, b in zip(k.ravel(), t.ravel()):
            name, _, length = index[int(a)]
            per_view[names[name]] = per_view.get(names[name], 0) + 1
            valid += min(50, length - int(b))
            if int(a) in key_parent:
                per_parent[key_parent[int(a)]] = per_parent.get(key_parent[int(a)], 0) + 1
        return dict(samples=int(k.size), valid_future_targets=int(valid), per_view=per_view, new_per_parent={str(p): v for p, v in sorted(per_parent.items())},
                    distinct_observations=int(len({(int(a), int(b)) for a, b in zip(k.ravel(), t.ravel())})))

    record = dict(run_id=RUN, fixed_before_training=time.strftime("%Y-%m-%dT%H:%M:%S%z"), valid=len(man["main_pool"]), pairing="exact: slots 0-5 are the very samples of RECEIVE_BRIDGE_CTRL's consumed plan, in place",
                  bridge_plan=dict(file=str(BRIDGE_PLAN), sha256=sha(BRIDGE_PLAN), equals_ctrl_consumed=True), new_view=dict(sha256=r2.view_sha(view), observations=len(view), parents=len(parents), sample_checks=checks),
                  replay_views={k: r2.view_sha(v) for k, v in views.items() if k != "expand"}, sample_plan=dict(file=str(run_dir() / "sample_plan.npz"), sha256=sha(run_dir() / "sample_plan.npz")),
                  data=exposure(dk, dt), ctrl_reference=exposure(ck, ct), old_slots=exposure(dk[:, :OLD_SLOTS], dt[:, :OLD_SLOTS]))
    target.write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(valid=record["valid"], new_observations=len(view), data=record["data"]["per_view"], per_parent=record["data"]["new_per_parent"])))
    return record


def trained():
    out = training_dir()
    try:
        run, pl = load(out / "run.json"), load(run_dir() / "plan.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (run.get("run_id") == RUN and run.get("steps") == UPDATES and run.get("consumed_equals_plan") and run.get("sample_plan_sha256") == pl.get("sample_plan", {}).get("sha256")
          and run.get("execution_source_sha256") == sha(__file__) and (out / CHECKPOINT / "model.safetensors").exists() and run.get("model_sha256") == sha(out / CHECKPOINT / "model.safetensors"))
    return run if ok else None


def train(args=None):
    import warnings
    import arm_output_trial as trial
    import single_demo_trial as sd
    import torch
    from torch.profiler import profile, ProfilerActivity
    from torch.utils.data import default_collate
    from dapier_act_policy import ACTPolicy
    if trained() is not None:
        print(f"[stored training of this plan] {training_dir()}")
        return
    pl = load(run_dir() / "plan.json")
    if not pl.get("valid"):
        raise SystemExit("no valid correction: nothing is trained")
    out = training_dir()
    if out.exists():
        kept = out.with_name(out.name + time.strftime(".unfinished-%Y%m%dT%H%M%S"))
        out.rename(kept)
        print(f"[unfinished attempt kept] {kept}")
    load_report = trial.gpu_other_load()
    trial.apply_reproducibility_settings()
    before = c0.bridge_models()
    st, views, index, _ = datasets()
    if sha(run_dir() / "sample_plan.npz") != pl["sample_plan"]["sha256"] or r2.view_sha(views["expand"]) != pl["new_view"]["sha256"]:
        raise RuntimeError("sample plan or the new view changed since the plan")
    with np.load(run_dir() / "sample_plan.npz", allow_pickle=False) as a:
        keys, ts = a["data_key"].copy(), a["data_t"].copy()
    source = Path(before["RECEIVE_FROZEN"]["path"])
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
    consumed_e, consumed_t = np.full((UPDATES, BATCH), -1, dtype=np.int64), np.full((UPDATES, BATCH), -1, dtype=np.int64)
    valid_targets, cuda_info, prof = 0, {}, None
    torch.manual_seed(SEED)
    started_wall, started = time.strftime("%Y-%m-%dT%H:%M:%S%z"), time.monotonic()
    torch.cuda.reset_peak_memory_stats()
    with (out / "metrics.jsonl").open("x") as log, warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        for step in range(1, UPDATES + 1):
            u = step - 1
            items = []
            for k, t in zip(keys[u], ts[u]):
                name, offset, _ = index[int(k)]
                items.append(views[name][offset + int(t)])
            batch_, meta = trial.split_meta(default_collate(items))
            consumed_e[u], consumed_t[u] = meta["meta_episode"].numpy(), meta["meta_t"].numpy()
            valid_targets += int((~batch_["action_is_pad"]).sum())
            batch_ = {k: v.cuda(non_blocking=True) for k, v in batch_.items()}
            if step == 11:                                                  # three ordinary updates run inside the profiler; nothing extra is computed
                prof = profile(activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA])
                prof.__enter__()
                cuda_info = dict(parameter_devices=sorted({str(p.device) for p in policy.parameters()}), buffer_devices=sorted({str(b.device) for b in policy.buffers()}),
                                 batch_devices={k: str(v.device) for k, v in batch_.items()}, horizon_weights_device=str(r.device))
            policy.train()
            loss, parts = sd.loss_terms(policy, batch_, r)
            if not torch.isfinite(loss):
                raise RuntimeError(f"non-finite loss at step {step}")
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            grad = torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0, error_if_nonfinite=True)
            optimizer.step()
            if step == 11:
                cuda_info["loss_device"] = str(loss.device)
            if step == 13 and prof is not None:
                torch.cuda.synchronize()
                prof.__exit__(None, None, None)
                rows = []
                for e in prof.key_averages():
                    t = getattr(e, "device_time_total", None)
                    t = getattr(e, "cuda_time_total", 0) if t is None else t
                    if t and t > 0:
                        rows.append((float(t), e.key, int(e.count)))
                rows.sort(reverse=True)
                cuda_info["profiled_updates"] = dict(steps=[11, 12, 13], cuda_kernel_events=len(rows), top=[dict(name=k[:60], device_time_us=round(t, 1), calls=c) for t, k, c in rows[:10]])
                prof = None
            if step == 1 or step % 100 == 0:
                entry = dict(step=step, elapsed_s=round(time.monotonic() - started, 2), loss=loss.item(), gradient_norm=grad.item(), **parts)
                log.write(json.dumps(entry) + "\n")
                log.flush()
                if step % 1000 == 0 or step == 1:
                    print(f"[DATA][{step:05d}] loss={entry['loss']:.4f} arm={parts['arm_l1_term']:.4f} grip={parts['gripper_l1_term']:.4f} grad={entry['gradient_norm']:.3f} t={entry['elapsed_s']:.0f}s", flush=True)
        name = out / CHECKPOINT
        policy.save_pretrained(name)
        torch.save(dict(optimizer=optimizer.state_dict(), step=UPDATES, inherited_step=int(state["step"]), cpu_rng=torch.get_rng_state(), cuda_rng=torch.cuda.get_rng_state()), name / "training_state.pt")
    seconds = round(time.monotonic() - started, 2)
    if not (np.array_equal(consumed_e, keys) and np.array_equal(consumed_t, ts)):
        raise RuntimeError("consumed samples differ from the plan")
    np.savez_compressed(out / "consumed_samples.npz", episode=consumed_e, timestep=consumed_t)
    cuda_info.update(pid=os.getpid(), gpu=s0.host().get("gpu"), peak_memory_mib=round(torch.cuda.max_memory_allocated() / 2**20, 1), torch=torch.__version__, cuda=torch.version.cuda)
    run = dict(kind="DAPIER ACT RECEIVE-EXPAND R1 DATA (SIM only)", run_id=RUN, condition="data", condition_name="RECEIVE_DATA", policy_variant=policy.config.type, dataset=str(td.DATASET),
               train_episodes=[int(x["manifest_index"]) for x in views["train"].rows], holdout_episodes=[], control_dt=td.DT, hardware_access=False, task_success=None, seed=SEED, batch_size=BATCH, steps=UPDATES,
               warm_start=str(source), warm_start_model_sha256=before["RECEIVE_FROZEN"]["model"], optimizer_state_loaded=True, optimizer_state_sha256=sha(source / "training_state.pt"),
               optimizer_step_at_load=int(state["step"]), adam_step_counters_at_load=adam_at_load, adam_step_counters_at_end=adam(), optimizer_param_groups=groups,
               normalization_sha256=sha(out / "normalization.npz"), sample_plan_sha256=pl["sample_plan"]["sha256"], new_view_sha256=pl["new_view"]["sha256"],
               training_manifest_sha256=sha(run_dir() / "training_manifest.json"), consumed_sha256=trial.array_sha(consumed_e, consumed_t), consumed_equals_plan=True, samples=int(consumed_e.size),
               valid_action_targets=valid_targets, precision="float32, no autocast", model_sha256=sha(name / "model.safetensors"), config_sha256=sha(name / "config.json"),
               execution_source_sha256=sha(__file__), stored_models_unchanged_after=c0.bridge_models() == before, started=started_wall, training_seconds=seconds, updates_per_second=round(UPDATES / seconds, 1),
               cuda=cuda_info, environment=trial.environment_record(), host=s0.host(), gpu_other_load_at_start=load_report,
               deterministic_warnings=sorted({str(w.message).split("\n")[0][:200] for w in caught}))
    (out / "run.json").write_text(json.dumps(run, indent=2, default=str))
    print(json.dumps(dict(seconds=seconds, valid_action_targets=valid_targets, model=run["model_sha256"][:12], cuda=cuda_info.get("parameter_devices"), kernels=(cuda_info.get("profiled_updates") or {}).get("cuda_kernel_events"))))


# ----------------------------------------------------------------------------------------------
# Student evaluation
# ----------------------------------------------------------------------------------------------

def eval_cases():
    fit = load(run_dir() / "training_manifest.json")["source_fit"]["parents"]
    out = []
    for bundle, seeds in (("train9", p1.TRAIN9), ("sourcefit", fit), ("val10", p1.VAL10), ("probe", (5000, 5008))):
        for n, seed in enumerate(seeds, 1):
            out += [f"evaluate:{bundle}:{seed}:{m}" for m in (("CTRL", "DATA") if n % 2 else ("DATA", "CTRL"))]
    return out


def evaluate(args=None):
    batch("evaluate", eval_cases(), "evaluate")


def everything(args=None):
    me = [sys.executable, str(Path(__file__).resolve())]
    for step in ("protocol", "collect", "manifest", "plan"):
        print(f"--- {step} --- {time.strftime('%H:%M:%S')}", flush=True)
        if subprocess.run(me + [step]).returncode:
            raise SystemExit(f"step `{step}` failed; later steps are not started")
    if not load(run_dir() / "plan.json").get("valid"):
        print("no valid correction: training and the student evaluation are not started", flush=True)
    else:
        for step in ("train", "evaluate"):
            print(f"--- {step} --- {time.strftime('%H:%M:%S')}", flush=True)
            if subprocess.run(me + [step]).returncode:
                raise SystemExit(f"step `{step}` failed; later steps are not started")
    report = Path(__file__).with_name("receive_expand_r1_report.py")
    if report.exists():
        print(f"--- report --- {time.strftime('%H:%M:%S')}", flush=True)
        subprocess.run([sys.executable, str(report), "all"])
    print(f"=== ALL DONE {time.strftime('%H:%M:%S')} ===", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("protocol", "collect", "manifest", "plan", "train", "evaluate", "all"):
        sub.add_parser(name)
    o = sub.add_parser("one")
    o.add_argument("--case", required=True)
    o.add_argument("--identity", required=True)
    o.add_argument("--profile", action="store_true")
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    return {"protocol": protocol, "collect": collect, "manifest": training_manifest, "plan": plan, "train": train, "evaluate": evaluate, "one": one, "all": everything}[args.command](args)


if __name__ == "__main__":
    main()
