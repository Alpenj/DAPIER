#!/usr/bin/env python3
"""RECEIVE R3 frozen development-Val screen: evaluation only (local SIM; no training, no teacher, no selection, no Test,
no hardware). The frozen START -> RECEIVE_{C|E} -> RELEASE_B chain of receive_r3_recheck, unchanged, from the original
reset of each of the ten Val parents of the frozen split manifest: C once and E once per parent, 20 new runs, one new
process each, in an order fixed beforehand.

Val is a development split that earlier work already used; these runs are not an independent Test, not unseen
generalization, not hardware, and the chain is not an unprivileged autonomous student (switches read SIM ground truth).

  run       check the parents and the reset lookup, fix the protocol, run the 20 cases
  summary   this run's own tables (used by `act-sim results`)
  video     at most three C / E comparisons: first valid E success, first E failure, first parent only C solved
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
import receive_onset_r3 as r3   # noqa: E402
import receive_r3_recheck as rk   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "receive-r3-val-screen-20261005-01"
LABEL = "SIM-ORACLE-GATED THREE-POLICY / FROZEN DEVELOPMENT-VAL SCREEN"
VAL10 = (5001, 5046, 5072, 5079, 5081, 5094, 5134, 5150, 5153, 5163)      # checked against the manifest, in its order
CONFIG_SHA = "679ca7c151b501e92d22ca41380203c08cc979ee41359e2865d3fdbc43cf56db"
MODELS, NAMES = rk.MODELS, rk.NAMES
SOURCES = rk.SOURCES + ("factory/v1/receive_r3_val.py", "factory/v1/frozen_transfer_check.py")
RECHECK = FACTORY_ROOT / "evaluation" / rk.RUN


def load(path):
    return json.loads(Path(path).read_text())


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parents():
    """The ten Val parents from the frozen split manifest, in manifest order; the list must be exactly the instructed one."""
    episodes = load(td.DATASET / "manifest.json")["episodes"]
    rows = [(i, e) for i, e in enumerate(episodes) if e["split"] == "val"]
    if tuple(e["seed"] for _, e in rows) != VAL10:
        raise RuntimeError(f"the manifest's Val parents are not the instructed Val10: {[e['seed'] for _, e in rows]}")
    out = []
    for position, (i, e) in enumerate(rows, 1):
        meta = load(Path(e["path"]) / "metadata.json")
        if meta["seed"] != e["seed"]:
            raise RuntimeError(f"manifest row {i}: metadata seed mismatch")
        reset = hashlib.sha256(json.dumps([meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"]], sort_keys=True).encode()).hexdigest()
        out.append(dict(position=position, manifest_index=i, seed=e["seed"], split=e["split"], generation=e["generation"], parent_physics_group=e["parent_physics_group"],
                        physics_parent_id=e["physics_parent_id"], reset_sha256=reset, block_xy_yaw=meta["sampling"]["block_xy_yaw"],
                        metadata_sha256=e["file_sha256"]["metadata.json"], episode_sha256=e["file_sha256"]["episode.npz"]))
    return out


def cases():
    """Odd manifest positions C -> E, even positions E -> C."""
    return [(p["seed"], m, 1) for p in parents() for m in (("C", "E") if p["position"] % 2 else ("E", "C"))]


def identity():
    import arm_output_trial as trial
    got = {k: sha(p / "model.safetensors") for k, p in MODELS.items()}
    norm = {k: sha(p.parent / "normalization.npz") for k, p in MODELS.items()}
    config = {k: sha(p / "config.json") for k, p in MODELS.items()}
    if got != rk.EXPECTED or set(norm.values()) != {rk.NORMALIZATION} or set(config.values()) != {CONFIG_SHA}:
        raise RuntimeError(f"a read-only model / normalization / config differs from the instructed SHA-256: {got} {norm} {config}")
    reference = load(RECHECK / "protocol.json")["identity"]
    executor = [p for p in rk.SOURCES if p != "factory/v1/receive_r3_recheck.py"]
    body = dict(run_id=RUN, models={k: dict(path=str(p), model=got[k], config=config[k], normalization=norm[k]) for k, p in MODELS.items()},
                manifest_sha256=sha(td.DATASET / "manifest.json"), parents=parents(), sources={p: sha(FACTORY_ROOT / p) for p in SOURCES}, contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                execution=dict(chain="START -> RECEIVE -> RELEASE_B", chunk=50, prefix=rc.PREFIX, control_hz=round(1 / td.DT), arm_bound_policy="project", ensemble="off", smoothing="off",
                               horizon_steps=rc.HORIZON, torch_seed=4100, policy_inputs=["left / right wrist RGB", "measured q_state"]),
                backend=trial.environment_record(), environment={k: __import__("os").environ.get(k) for k in rk.ENV_KEYS},
                same_as_recheck=dict(models={k: reference["models"][k]["model"] == got[k] for k in got}, contract_text=reference["contract"] == rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                                     execution={k: reference["execution"].get(k) == v for k, v in dict(prefix=rc.PREFIX, arm_bound_policy="project", ensemble="off", horizon_steps=rc.HORIZON,
                                                                                                    torch_seed=4100).items()},
                                     executor_sources_unchanged={p: reference["sources"].get(p) == sha(FACTORY_ROOT / p) for p in executor},
                                     recheck_module_unchanged=reference["sources"].get("factory/v1/receive_r3_recheck.py") == sha(FACTORY_ROOT / "factory/v1/receive_r3_recheck.py")))
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def reset_lookup_check():
    """No rollout: the reset that the unchanged executor builds for a parent equals that parent's stored original initial
    state, and for a Train parent it equals the reset the recheck runs actually started from."""
    import frozen_transfer_check as ft
    from sim_data_factory.reset_sampler import make_task_episode
    out = dict(val={}, train_reference={})
    for p in parents():
        demo = ft.demonstration(p["seed"])
        meta = demo["metadata"]
        ctx = make_task_episode(p["seed"], meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
        state = ctx["initial"]["integration_state"]
        out["val"][str(p["seed"])] = dict(split=demo["row"]["split"], reset_equals_stored_demonstration_initial_state=bool(np.array_equal(state, demo["integration_state"][0])),
                                          reset_state_sha256=rk.array_sha(state))
    recheck = load(RECHECK / "results.json")["results"]
    for seed in (5000, 5008):
        demo = ft.demonstration(seed)
        meta = demo["metadata"]
        ctx = make_task_episode(seed, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
        stored = {r["moments"]["reset"]["state_sha256"] for r in recheck if r["seed"] == seed}
        out["train_reference"][str(seed)] = dict(same_lookup_gives_the_recheck_reset=stored == {rk.array_sha(ctx["initial"]["integration_state"])})
    out["all_val_resets_are_the_stored_originals"] = all(v["reset_equals_stored_demonstration_initial_state"] and v["split"] == "val" for v in out["val"].values())
    out["train_reference_unchanged"] = all(v["same_lookup_gives_the_recheck_reset"] for v in out["train_reference"].values())
    try:
        ft.demonstration(next(e["seed"] for e in load(td.DATASET / "manifest.json")["episodes"] if e["split"] == "test"))
        out["test_parent_refused_by_the_lookup"] = False
    except ValueError:
        out["test_parent_refused_by_the_lookup"] = True
    return out


def case_dir(ident, seed, model, rep):
    return run_dir() / "runs" / ident["sha256"][:12] / f"s{seed}" / model


def complete(out, ident, case):
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    need = ("task_success", "run_valid", "first_failure_stage", "moments", "child_settings", "phases", "projection", "events")
    ok = (r.get("identity_sha256") == ident["sha256"] and r.get("case") == list(case) and r.get("job") == RUN and meta.get("complete") and r.get("boundary_contract") == "repaired"
          and all(k in r for k in need) and all((out / f).exists() for f in ("episode.npz", "action_trace.json", "task_check_rows.json")))
    return r if ok else None


def one(args):
    ident = identity()
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    parent = next(p for p in ident["parents"] if p["seed"] == args.seed)
    out = case_dir(ident, args.seed, args.model, 1)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    rc.MODELS["RECEIVE"] = MODELS[args.model]
    loaded, original = {}, rc.load_policy

    def remember(name):
        got = original(name)
        loaded[name] = got[0]
        return got

    rc.load_policy = remember
    started = time.time()
    try:
        result = rc.execute(out, args.seed, ("START", "RECEIVE", "B"), None, f"{LABEL}: Val parent {args.seed} START -> {NAMES[args.model]} -> RELEASE_B", contract="repaired")
    finally:
        rc.load_policy = original
    rows = load(out / "task_check_rows.json")
    trace = load(out / "action_trace.json")
    with np.load(out / "episode.npz", allow_pickle=False) as a:
        states, q = a["integration_state"].copy(), a["q_state"].copy()
    rgb = {s: np.load(out / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}

    def moment(step):
        if step is None or step >= len(rgb["left"]):
            return None
        return dict(step=int(step), sim_time_s=round(float(states[step][0]), 2), state_sha256=rk.array_sha(states[step]), q_sha256=rk.array_sha(q[step]),
                    rgb_sha256=dict(left=rk.array_sha(rgb["left"][step]), right=rk.array_sha(rgb["right"][step])))

    entry = next((s for s in result["switches"] if s["to"] == "RECEIVE"), None)
    gate = next((s for s in result["switches"] if s["to"] == "B"), None)
    success, fault = result["task_success"], result["first_fault"]
    stage = None if success else "START" if entry is None else "RECEIVE" if gate is None else "RELEASE_B"
    kind = None if success else ("fault" if fault is not None else "task not completed within the 40 s horizon")
    result.update(job=RUN, kind="full", classification=LABEL, split="val (development)", case=[args.seed, args.model, 1], receive_model=args.model, receive_name=NAMES[args.model], repeat=1,
                  parent=parent, identity_sha256=ident["sha256"], chain=f"START -> {NAMES[args.model]} -> RELEASE_B", handover_event=result["handover_s"] is not None,
                  completed_40s=result["termination"] == "horizon_completed", first_failure_stage=stage, first_failure_kind=kind,
                  transfer=None if entry is None else r3.arm_motion(out, entry["step"]),
                  counts=dict(commands_dispatched=sum(t.get("ctrl_applied") is not None for t in trace), completed_transitions=result["completed_transitions"],
                              sim_elapsed_s=result["last_completed_state_sim_s"], wall_s=round(time.time() - started, 1)),
                  moments=dict(reset=moment(0), receive_entry=moment(entry["step"] if entry else None), release_b_entry=moment(gate["step"] if gate else None)),
                  contract_checks=rb.contract_checks(result), phases=r2.phase_details(result, rows, result["receive_entry_s"] or 0.0),
                  child_settings=rk.child_settings({("RECEIVE_" + args.model if k == "RECEIVE" else "RELEASE_B" if k == "B" else k): p for k, p in loaded.items()}))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(seed=args.seed, model=args.model, events={k: round(v, 2) for k, v in result["events"].items()}, receive_entry=result["receive_entry_s"],
                          ready=result["phases"]["semantic_ready_s"], release_b=result["gate_s"], termination=result["termination"], first_failure_stage=stage, first_failure_kind=kind,
                          task_success=success, final_support=result["final_support"]), default=str))


def table_row(r):
    row = rk.table_row(r)
    ev = r["events"]
    row.update(position=r["parent"]["position"], reach_s=ev.get("reach"), left_grasp_s=ev.get("left_grasp"), completed_40s=r["completed_40s"], counts=r["counts"],
               receive_entered=r["receive_entered"], release_b_entered=r["gate_reached"], handover_event=r["handover_event"])
    return row


def pairs(results):
    """Per parent: C and E side by side, and whether their reset / RECEIVE entry were the same state."""
    by = {(r["seed"], r["receive_model"]): r for r in results}
    out = []
    for p in parents():
        c, e = by.get((p["seed"], "C")), by.get((p["seed"], "E"))
        item = dict(position=p["position"], seed=p["seed"], order="C -> E" if p["position"] % 2 else "E -> C")
        for name, r in (("C", c), ("E", e)):
            item[name] = None if r is None else dict(task_success=r["task_success"], first_failure_stage=r["first_failure_stage"], first_failure_kind=r["first_failure_kind"],
                                                     pick_lift=r["pick_lift_s"] is not None, receive_entered=r["receive_entered"], semantic_ready=r["phases"]["semantic_ready_s"] is not None,
                                                     release_b_entered=r["gate_reached"], handover_event=r["handover_event"], final_support=r["final_support"])
        if c and e:
            same = lambda key: None if not (c["moments"][key] and e["moments"][key]) else c["moments"][key]["state_sha256"] == e["moments"][key]["state_sha256"]
            item.update(reset_state_identical=same("reset"), reset_rgb_identical=c["moments"]["reset"]["rgb_sha256"] == e["moments"]["reset"]["rgb_sha256"],
                        receive_entry_state_identical=same("receive_entry"),
                        receive_entry_steps=[(x["moments"]["receive_entry"] or {}).get("step") for x in (c, e)],
                        outcome="both succeeded" if c["task_success"] and e["task_success"] else "E only" if e["task_success"] else "C only (E regression)" if c["task_success"] else "neither")
        out.append(item)
    return out


def tally(results):
    out = {}
    for mdl in ("C", "E"):
        rs = [r for r in results if r["receive_model"] == mdl]
        reached = [r for r in rs if r["gate_reached"]]
        proj = {}
        for name in ("START", "RECEIVE", "B"):
            parts = [r["projection"][name] for r in rs if name in r["projection"]]
            proj["RELEASE_B" if name == "B" else name] = dict(runs_with_projection=sum(p["projected_steps"] > 0 for p in parts), steps_total=sum(p["projected_steps"] for p in parts),
                                                             steps_max=max((p["projected_steps"] for p in parts), default=0), longest_s_max=max((p["longest_s"] for p in parts), default=0.0),
                                                             max_raw_excess_rad=round(max((p["max_raw_excess_rad"] for p in parts), default=0.0), 4), axes=sorted({a for p in parts for a in p["per_axis"]}))
        out[mdl] = dict(runs=len(rs), reach=sum("reach" in r["events"] for r in rs), left_grasp=sum("left_grasp" in r["events"] for r in rs), pick_lift=sum(r["pick_lift_s"] is not None for r in rs),
                        receive_entered=sum(r["receive_entered"] for r in rs), semantic_ready=sum(r["phases"]["semantic_ready_s"] is not None for r in rs), release_b_entered=len(reached),
                        handover_event=sum(r["handover_event"] for r in rs), completed_40s=sum(r["completed_40s"] for r in rs), run_valid=sum(r["run_valid"] for r in rs),
                        task_success=sum(r["task_success"] for r in rs), success_parents=[r["seed"] for r in rs if r["task_success"]],
                        release_b_success_of_reached="N/A (RELEASE_B not reached, not evaluated)" if not reached else f"{sum(r['task_success'] for r in reached)}/{len(reached)}",
                        receive_ready_of_entered="N/A (RECEIVE not entered, not evaluated)" if not any(r["receive_entered"] for r in rs)
                        else f"{sum(r['phases']['semantic_ready_s'] is not None for r in rs)}/{sum(r['receive_entered'] for r in rs)}",
                        first_failure_stage={k: sum(r["first_failure_stage"] == k for r in rs) for k in ("START", "RECEIVE", "RELEASE_B")},
                        first_failure_kind={k: sum(r["first_failure_kind"] == k for r in rs) for k in ("fault", "task not completed within the 40 s horizon")},
                        handover_event_but_not_success=sum(r["handover_event"] and not r["task_success"] for r in rs), projection=proj)
    return out


def run(args):
    ident = identity()
    plan = cases()
    run_dir().mkdir(parents=True, exist_ok=True)
    (run_dir() / "logs").mkdir(exist_ok=True)
    proto = run_dir() / "protocol.json"
    if proto.exists():
        stored = load(proto)
        if stored["identity"]["sha256"] != ident["sha256"] or stored["cases"] != [list(c) for c in plan]:
            raise RuntimeError("protocol.json of this job has another identity or case list: nothing is run, nothing is overwritten")
        print(f"[resumed job] protocol of {stored['fixed_before_execution']} (identity {ident['sha256'][:12]}); completed cases of this job are not run again and are not new runs")
    else:
        lookup = reset_lookup_check()
        if not (lookup["all_val_resets_are_the_stored_originals"] and lookup["train_reference_unchanged"] and lookup["test_parent_refused_by_the_lookup"]):
            (run_dir() / "reset_lookup_FAILED.json").write_text(json.dumps(lookup, indent=2))
            raise RuntimeError("the reset lookup check failed; no case is run (see reset_lookup_FAILED.json)")
        proto.write_text(json.dumps(dict(
            run_id=RUN, name=LABEL, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"), identity=ident, cases=[list(c) for c in plan],
            order="manifest order; odd positions C -> E, even positions E -> C; sequential, one new process per case, no parallel GPU job",
            budget="20 new original-reset runs (10 Val parents x C / E, one each); no repeat, no replacement, no exclusion, no extension",
            split="Val of the frozen G000/G001 manifest: a development split used repeatedly before; not an independent Test, not unseen generalization, not hardware, not an unprivileged "
                  "autonomous student (switches read SIM ground truth). Test is not evaluated; the manifest is only read for split membership and the lookup refuses a Test parent",
            reset_lookup_check=lookup, denominators="whole-run counts keep upstream failures; RECEIVE readiness is over runs that entered RECEIVE; RELEASE_B is over runs that reached it (N/A if none)",
            decision_rule="no pass line. E success on a parent = one recorded success of this fixed configuration on that development-Val condition; E > C = an observed difference on this fixed set; "
                          "E = C = no further advantage observed; a parent C solved and E did not = regression row; mostly failing = the frozen reference stays and one next target is named by the "
                          "first failing policy; nothing is retrained, collected, tuned or repeated",
            videos="first valid E success parent, first E failure parent, first parent only C solved (manifest order, at most three, duplicates once)",
            not_called=["select", "teacher", "train", "any optimizer step", "normalization recomputation", "Test evaluation"],
            lineage_note="whether any Val parent influenced earlier design choices beyond its documented use as a development split is not re-audited here: UNKNOWN",
            training=False, teacher_collection=False, test=False, hardware_access=False), indent=2))
    status = []
    for case in plan:
        seed, mdl, rep = case
        out = case_dir(ident, *case)
        if complete(out, ident, case) is not None:
            status.append(dict(case=list(case), status="completed (same job resumed: stored case of this identity, not a new run)"))
            print(f"[stored case of this job] {case}")
            continue
        if out.exists():
            status.append(dict(case=list(case), status="NOT RUN: an incomplete folder of this identity exists; neither reused nor overwritten", path=str(out)))
            print(f"[not run] {case}: incomplete folder {out}")
            continue
        started = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--seed", str(seed), "--model", mdl, "--identity", ident["sha256"]], capture_output=True, text=True)
        (run_dir() / "logs" / f"{ident['sha256'][:12]}_s{seed}_{mdl}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        ok = done.returncode == 0 and complete(out, ident, case) is not None
        status.append(dict(case=list(case), started=started, returncode=done.returncode, status="completed (new run)" if ok else "executor_error (not a task result)",
                           **({} if ok else dict(stderr_tail=done.stderr[-600:]))))
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(line[-1][:700] if ok and line else f"[executor_error, not a task failure] {case}: returncode {done.returncode}: {done.stderr[-300:]}", flush=True)
    results = [r for case in plan if (r := complete(case_dir(ident, *case), ident, case)) is not None]
    settings = [r["child_settings"] for r in results]
    same = lambda key: len({json.dumps(s[key], sort_keys=True, default=str) for s in settings}) <= 1
    (run_dir() / "results.json").write_text(json.dumps(dict(
        run_id=RUN, note=f"{LABEL}; switches read SIM ground truth; fixed weights; development Val parents; one run per case; not added to any earlier table",
        identity_sha256=ident["sha256"], identity_unchanged_during_run=identity()["sha256"] == ident["sha256"], planned_cases=len(plan), completed_cases=len(results),
        executor_errors=[s for s in status if not s["status"].startswith("completed")], execution_status=status, tally=tally(results), pairs=pairs(results),
        child_settings=dict(first=settings[0] if settings else None, pids_distinct=len({s["pid"] for s in settings}),
                            all_policies_eval=all(not v for s in settings for v in s["policy_training_flag"].values()),
                            identical_across_children={k: same(k) for k in ("torch_threads", "cudnn_benchmark", "cudnn_deterministic", "deterministic_algorithms", "deterministic_warn_only", "torch",
                                                                            "cuda", "mujoco", "environment", "gl")} if settings else None,
                            torch_initial_seeds=sorted({s["rng"]["torch_initial_seed"] for s in settings})),
        table=[table_row(r) for r in results], results=results), indent=2, default=str))
    print(f"[written] {run_dir() / 'results.json'}: {len(results)} of {len(plan)} cases completed, {len([s for s in status if not s['status'].startswith('completed')])} executor error(s) / not run; "
          f"identity {ident['sha256'][:12]}")
    summary(args)


def summary(args=None):
    path = run_dir() / "results.json"
    if not path.exists():
        print(f"RECEIVE R3 frozen development-Val screen ({RUN}): unavailable, no results.json")
        return
    data = load(path)
    print(f"\n=== RECEIVE R3 frozen development-Val screen ({RUN}) — {LABEL}; 전환은 SIM 정답 기반 ===")
    print(f"개발용 Val10(과거 반복 사용), parent당 C·E 각 1회, 가중치 고정. 독립 Test·일반화·실물·비특권 자율 결과가 아니다. "
          f"완료 {data['completed_cases']}/{data['planned_cases']}건, 실행기 오류 {len(data['executor_errors'])}건, identity {data['identity_sha256'][:12]}.")
    print("\n| 모델 | 전체 유효 성공 | pick_lift | RECEIVE 진입 | 수신 준비 | RELEASE_B 진입 | RELEASE_B 도달 중 성공 | 첫 실패 START/RECEIVE/RELEASE_B |\n|---|---|---|---|---|---|---|---|")
    for mdl, v in data["tally"].items():
        f = v["first_failure_stage"]
        print(f"| {mdl} | {v['task_success']}/{v['runs']} | {v['pick_lift']} | {v['receive_entered']} | {v['semantic_ready']} | {v['release_b_entered']} | {v['release_b_success_of_reached']} | "
              f"{f['START']}/{f['RECEIVE']}/{f['RELEASE_B']} |")
    show = lambda x: "미실행" if x is None else ("성공" if x["task_success"] else f"실패({x['first_failure_stage']})")
    print("\n| # | parent | C | E | 진입 상태 동일 | 비교 |\n|---|---|---|---|---|---|")
    for p in data["pairs"]:
        same = p.get("receive_entry_state_identical")
        print(f"| {p['position']} | {p['seed']} | {show(p['C'])} | {show(p['E'])} | {'-' if same is None else ('예' if same else '아니오')} | {p.get('outcome', '-')} |")
    print(f"  결과: {path}\n  보고서: {run_dir() / 'REPORT.md'}\n  영상: {run_dir() / 'videos'}")


def video(args):
    import recovery_trial as rt
    ident = identity()
    data = load(run_dir() / "results.json")
    order = [p for p in data["pairs"] if p["C"] and p["E"]]
    picks = [("first_valid_E_success", next((p for p in order if p["E"]["task_success"]), None)), ("first_E_failure", next((p for p in order if not p["E"]["task_success"]), None)),
             ("first_C_only_success", next((p for p in order if p["C"]["task_success"] and not p["E"]["task_success"]), None))]
    done = set()
    for tag, p in picks:
        if p is None:
            print(f"[no such category] {tag}")
            continue
        if p["seed"] in done:
            print(f"[same parent as an earlier video] {tag}: parent {p['seed']}")
            continue
        done.add(p["seed"])
        out = run_dir() / "videos" / f"{tag}_val{p['seed']}_C_E_{ident['sha256'][:12]}.mp4"
        if out.exists():
            print(f"[exists] {out}")
            continue
        rows = []
        for mdl in ("C", "E"):
            case = (p["seed"], mdl, 1)
            r = complete(case_dir(ident, *case), ident, case)
            row = rt.teacher_row(f"STUDENT / SIM-ORACLE-GATED / development Val {p['seed']}: {r['chain']}", Path(r["path"]))
            ev = dict(r["events"])
            ev.update({f"SWITCH->{'RELEASE_B' if s['to'] == 'B' else s['to']}": s["sim_time_s"] for s in r["switches"]})
            row["events"] = dict(sorted(ev.items(), key=lambda kv: kv[1]))
            row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}" + ("" if r["task_success"] else f" (first failure in {r['first_failure_stage']})"),
                               "VALID HANDOVER, right hand holds" if r["task_success"] else f"NO VALID HANDOVER; final support: {r['final_support']}")
            rows.append(row)
        rt.render(rows, out, f"{LABEL} / parent {p['seed']} ({tag.replace('_', ' ')})", 2000, False, right_gripper=True)
        print(f"[written] {out}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("run", "summary", "video"):
        sub.add_parser(name)
    o = sub.add_parser("one")
    o.add_argument("--seed", type=int, choices=VAL10, required=True)
    o.add_argument("--model", choices=("C", "E"), required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    dict(run=run, one=one, summary=summary, video=video)[args.command](args)


if __name__ == "__main__":
    main()
