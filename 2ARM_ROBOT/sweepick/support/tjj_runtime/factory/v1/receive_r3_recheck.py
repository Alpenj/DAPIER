#!/usr/bin/env python3
"""RECEIVE R3 frozen recheck: evaluation only (local SIM; no training, no teacher, no selection, no hardware).

START, RELEASE_B, RECEIVE_C (RECEIVE_REPLAY_CONTROL) and RECEIVE_E (RECEIVE_ONSET_RECOVERY) are read-only and checked by
SHA-256. 22 new original-reset runs of START -> RECEIVE_{C|E} -> RELEASE_B under the unchanged repaired contract:
seed 5000 and 5008 five times per model, seed 5002 once per model, each in a new process, in an order fixed beforehand.
Repeats of one reset are re-runs of the same initial condition, not independent initial conditions.

  run       fix the protocol, then run the 22 cases (a completed case of this same job and identity is not run twice)
  summary   this run's own tables (used by `act-sim results`)
  video     first C / E pair of 5000 and of 5008 in the fixed order; the first failed E run if there is one

Switches read SIM ground truth: SIM-ORACLE-GATED THREE-POLICY CANDIDATE / FROZEN RECHECK.
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
import receive_candidate as rc   # noqa: E402
import receive_boundary_repair as rb   # noqa: E402
import receive_entry_r2 as r2   # noqa: E402
import receive_onset_r3 as r3   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "receive-r3-recheck-20261005-01"
LABEL = rc.LABEL + " / FROZEN RECHECK"
MODELS = dict(START=rc.MODELS["START"], RELEASE_B=rc.MODELS["B"], C=r3.training_dir("c") / r3.CHECKPOINT, E=r3.training_dir("e") / r3.CHECKPOINT)
EXPECTED = dict(START="19368b679b4a97213698b449243ab8443b851ba6b25aecd713558b63c74f9dc7", RELEASE_B="5a001b39fe271251f3218eabb2781079c4e0c5d218169d6dca41c18917a3402c",
                C="a5961c81851199ac27ae09f38782c86f1d5beaa8b740f2f53f73390920a93fa4", E="69f8e02417faa651c6dc3899976a8f48dc75ef1bb19a57a5e47e2d68b61bfee2")
NORMALIZATION = "e803716b7d306a89516647173c890807dc8c2e92125409b4b1fbbde86dacb258"
NAMES = dict(C="RECEIVE_REPLAY_CONTROL", E="RECEIVE_ONSET_RECOVERY")
SOURCES = r3.SOURCES + ("factory/v1/receive_r3_recheck.py",)
ENV_KEYS = ("MUJOCO_GL", "CUBLAS_WORKSPACE_CONFIG", "OPENBLAS_NUM_THREADS", "PYTHONHASHSEED", "CUDA_VISIBLE_DEVICES", "PYOPENGL_PLATFORM", "OMP_NUM_THREADS", "DAPIER_SO101_MJCF")


def cases():
    """The 22 cases in the fixed order: repeats 1/3/5 5000 C->E then 5008 E->C; repeats 2/4 5000 E->C then 5008 C->E; last 5002 C->E."""
    out = []
    for rep in range(1, 6):
        a, b = (("C", "E"), ("E", "C")) if rep % 2 else (("E", "C"), ("C", "E"))
        out += [(5000, m, rep) for m in a] + [(5008, m, rep) for m in b]
    return out + [(5002, "C", 1), (5002, "E", 1)]


def load(path):
    return json.loads(Path(path).read_text())


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_sha(a):
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def identity():
    import arm_output_trial as trial
    got = {k: sha(p / "model.safetensors") for k, p in MODELS.items()}
    norm = {k: sha(p.parent / "normalization.npz") for k, p in MODELS.items()}
    if got != EXPECTED or set(norm.values()) != {NORMALIZATION}:
        raise RuntimeError(f"a read-only model or normalization differs from the instructed SHA-256: {got} {norm}")
    reference = load(r3.run_dir() / "evaluation_identity_e716e4d4886b.json")["identity"]
    body = dict(run_id=RUN, models={k: dict(path=str(p), model=got[k], config=sha(p / "config.json"), normalization=norm[k]) for k, p in MODELS.items()},
                resets=r2.identity()["resets"], sources={p: sha(FACTORY_ROOT / p) for p in SOURCES}, contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                execution=dict(chain="START -> RECEIVE -> RELEASE_B", prefix=rc.PREFIX, control_hz=round(1 / td.DT), arm_bound_policy="project", ensemble="off", smoothing="off",
                               horizon_steps=rc.HORIZON, torch_seed=4100, policy_inputs=["left / right wrist RGB", "q_state"]),
                backend=trial.environment_record(), environment={k: os.environ.get(k) for k in ENV_KEYS},
                same_as_r3_evaluation=dict(models_C_E_START_RELEASE=all(reference["models"][a]["model.safetensors"] == got[b] for a, b in
                                                                         (("START", "START"), ("RELEASE_B", "RELEASE_B"), ("RECEIVE_C", "C"), ("RECEIVE_E", "E"))),
                                           contract_text=reference["contract"] == rb.CONTRACT_TEXT["REPAIRED_RECEIVE"], execution_prefix=reference["execution"]["prefix"] == rc.PREFIX,
                                           executor_sources_unchanged={p: reference["sources"].get(p) == sha(FACTORY_ROOT / p) for p in reference["sources"] if p != "factory/v1/receive_onset_r3.py"}))
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def case_dir(ident, seed, model, rep):
    return run_dir() / "runs" / ident["sha256"][:12] / f"s{seed}" / f"{model}_rep{rep}"


def complete(out, ident, case):
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (r.get("identity_sha256") == ident["sha256"] and r.get("case") == list(case) and r.get("job") == RUN and meta.get("complete") and r.get("boundary_contract") == "repaired"
          and all((out / f).exists() for f in ("episode.npz", "action_trace.json", "task_check_rows.json")))
    return r if ok else None


def child_settings(policies):
    """What this evaluation process actually ran with, read after the models were loaded and the run finished."""
    import random
    import mujoco
    import torch
    out = dict(pid=os.getpid(), policy_training_flag={k: bool(p.training) for k, p in policies.items()},
               latent="deployment path predict_canonical_chunk: eval(), no action in the batch -> the model's zero latent (use_vae config: "
                      + str({k: bool(p.config.use_vae) for k, p in policies.items()}) + ")",
               rng=dict(torch_manual_seed_at_start=4100, torch_initial_seed=int(torch.initial_seed()), numpy_global_state_sha=array_sha(np.random.get_state()[1]),
                        python_random_state_sha=hashlib.sha256(repr(random.getstate()).encode()).hexdigest(), note="numpy / python global RNGs are not seeded by the executor; their state is recorded only"),
               torch_threads=torch.get_num_threads(), cudnn_benchmark=torch.backends.cudnn.benchmark, cudnn_deterministic=torch.backends.cudnn.deterministic,
               deterministic_algorithms=torch.are_deterministic_algorithms_enabled(), deterministic_warn_only=torch.is_deterministic_algorithms_warn_only_enabled(),
               torch=torch.__version__, cuda=torch.version.cuda, cudnn=torch.backends.cudnn.version(), gpu=torch.cuda.get_device_name(0), mujoco=mujoco.__version__,
               numpy=np.__version__, python=sys.version.split()[0], environment={k: os.environ.get(k) for k in ENV_KEYS})
    try:                                                                  # a context opened after the run only to read the driver strings
        from integration_scenes import task_env
        import OpenGL.GL as G
        renderer = mujoco.Renderer(task_env("desk", grippers="both").model, 120, 160)
        out["gl"] = dict(vendor=G.glGetString(G.GL_VENDOR).decode(), renderer=G.glGetString(G.GL_RENDERER).decode(), version=G.glGetString(G.GL_VERSION).decode())
        renderer.close()
    except Exception as error:   # noqa: BLE001
        out["gl"] = f"unavailable: {type(error).__name__}"
    return out


def one(args):
    ident = identity()
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    out = case_dir(ident, args.seed, args.model, args.rep)
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
    try:
        result = rc.execute(out, args.seed, ("START", "RECEIVE", "B"), None, f"{LABEL}: seed {args.seed} START -> {NAMES[args.model]} -> RELEASE_B rep{args.rep}", contract="repaired")
    finally:
        rc.load_policy = original
    rows = load(out / "task_check_rows.json")
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
    success = result["task_success"]
    stage = None if success else "START" if entry is None else "RECEIVE" if gate is None else "RELEASE_B"
    fault = result["first_fault"]
    kind = None if success else ("fault" if fault is not None else "block on the table at 40 s" if result["final_support"] == "table" else f"no fault; 40 s ended with {result['final_support']}")
    result.update(job=RUN, kind="full", classification=LABEL, case=[args.seed, args.model, args.rep], receive_model=args.model, receive_name=NAMES[args.model], repeat=args.rep,
                  identity_sha256=ident["sha256"], chain=f"START -> {NAMES[args.model]} -> RELEASE_B", handover_event=result["handover_s"] is not None,
                  first_failure_stage=stage, first_failure_kind=kind, transfer=None if entry is None else r3.arm_motion(out, entry["step"]),
                  moments=dict(reset=moment(0), receive_entry=moment(entry["step"] if entry else None), release_b_entry=moment(gate["step"] if gate else None)),
                  contract_checks=rb.contract_checks(result), phases=r2.phase_details(result, rows, result["receive_entry_s"] or 0.0),
                  child_settings=child_settings({("RECEIVE_" + args.model if k == "RECEIVE" else "RELEASE_B" if k == "B" else k): p for k, p in loaded.items()}))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(seed=args.seed, model=args.model, rep=args.rep, receive_entry=result["receive_entry_s"], ready=result["phases"]["semantic_ready_s"], release_b=result["gate_s"],
                          handover=result["handover_s"], termination=result["termination"], first_failure_stage=stage, task_success=success, final_support=result["final_support"]), default=str))


def table_row(r):
    q = r["receiving"] or {}
    fault = r["first_fault"]
    return dict(seed=r["seed"], receive=r["receive_model"], rep=r["repeat"], pick_lift_s=r["pick_lift_s"], receive_entry_s=r["receive_entry_s"], transfer=r["transfer"],
                right_grasp_s=r["right_grasp_s"], semantic_ready_s=r["phases"]["semantic_ready_s"], release_b_entry_s=r["gate_s"], left_release_s=r["left_release_while_right_holds_first_s"],
                left_retreat_s=r["left_clear_by_donor_clearance_first_s"], handover_event_s=r["handover_s"], first_fault_s=None if fault is None else round(fault["time"], 2),
                fault_pairs=r["phases"]["fault_pairs"], first_failure_stage=r["first_failure_stage"], first_failure_kind=r["first_failure_kind"], termination=r["termination"],
                run_valid=r["run_valid"], terminal_handover=r["terminal_handover_success"], task_success=r["task_success"], final_support=r["final_support"], order=r["phases"]["order"],
                projection={k: dict(steps=v["projected_steps"], of=v["dispatched"], longest_s=v["longest_s"], max_raw_excess_rad=round(v["max_raw_excess_rad"], 4), per_axis=v["per_axis"])
                            for k, v in r["projection"].items()},
                right_before_two_pad_hold=q.get("right_before_two_pad_hold"), block_in_left_site=q.get("block_in_left_site"), left_hold_lost_steps=q.get("left_hold_lost_steps"),
                c25_regular=r["contract_checks"]["c25_consumption_regular"], switches=r["contract_checks"]["switches"], moments=r["moments"], pid=r["child_settings"]["pid"])


def trajectories(results):
    """How alike the runs of one parent were: identities at reset / RECEIVE entry / RELEASE_B entry, and the first differing
    state between repeats. Identical prefixes are reported, never used to drop a run."""
    out = {}
    for seed in (5000, 5008, 5002):
        rs = [r for r in results if r["seed"] == seed]
        if not rs:
            continue
        distinct = lambda key, field: len({(r["moments"][key] or {}).get(field) if not isinstance((r["moments"][key] or {}).get(field), dict) else json.dumps(r["moments"][key][field], sort_keys=True)
                                           for r in rs if r["moments"][key]})
        item = dict(runs=len(rs), reset_state_distinct=distinct("reset", "state_sha256"), reset_rgb_distinct=distinct("reset", "rgb_sha256"),
                    receive_entry_reached=sum(r["moments"]["receive_entry"] is not None for r in rs), receive_entry_state_distinct=distinct("receive_entry", "state_sha256"),
                    receive_entry_rgb_distinct=distinct("receive_entry", "rgb_sha256"), receive_entry_steps=sorted({r["moments"]["receive_entry"]["step"] for r in rs if r["moments"]["receive_entry"]}),
                    release_b_entry_reached=sum(r["moments"]["release_b_entry"] is not None for r in rs), release_b_entry_state_distinct=distinct("release_b_entry", "state_sha256"), per_model={})
        cache = {}
        for r in rs:
            with np.load(Path(r["path"]) / "episode.npz", allow_pickle=False) as a:
                cache[(r["receive_model"], r["repeat"])] = (a["integration_state"].copy(), a["q_state"].copy())
        ref_q = None
        for mdl in ("C", "E"):
            reps = sorted(k[1] for k in cache if k[0] == mdl)
            if not reps:
                continue
            base = cache[(mdl, reps[0])][0]
            first = {}
            for rep in reps[1:]:
                x = cache[(mdl, rep)][0]
                n = min(len(base), len(x))
                same = np.array([np.array_equal(base[i], x[i]) for i in range(n)])
                first[f"rep{rep} vs rep{reps[0]}"] = None if same.all() else int(np.argmin(same))
            entries = [r for r in rs if r["receive_model"] == mdl and r["moments"]["receive_entry"]]
            qs = [cache[(mdl, r["repeat"])][1][r["moments"]["receive_entry"]["step"]] for r in entries]
            ref_q = qs[0] if ref_q is None and qs else ref_q
            item["per_model"][mdl] = dict(first_state_difference_step=first, entry_states_distinct=len({r["moments"]["receive_entry"]["state_sha256"] for r in entries}),
                                          entry_q_max_abs_diff_within_model_rad=float(max((np.abs(a - qs[0]).max() for a in qs), default=0.0)),
                                          entry_q_max_abs_diff_from_first_listed_rad=float(max((np.abs(a - ref_q).max() for a in qs), default=0.0)))
        out[str(seed)] = item
    return out


def tally(results):
    out = {}
    for seed in (5000, 5008, 5002):
        for mdl in ("C", "E"):
            rs = [r for r in results if r["seed"] == seed and r["receive_model"] == mdl]
            if not rs:
                continue
            reached = [r for r in rs if r["gate_reached"]]
            proj = {}
            for name in ("START", "RECEIVE", "B"):
                parts = [r["projection"][name] for r in rs if name in r["projection"]]
                proj["RELEASE_B" if name == "B" else name] = dict(runs_with_projection=sum(p["projected_steps"] > 0 for p in parts), steps_total=sum(p["projected_steps"] for p in parts),
                                                                 steps_max=max((p["projected_steps"] for p in parts), default=0), longest_s_max=max((p["longest_s"] for p in parts), default=0.0),
                                                                 max_raw_excess_rad=round(max((p["max_raw_excess_rad"] for p in parts), default=0.0), 4),
                                                                 axes=sorted({a for p in parts for a in p["per_axis"]}))
            out[f"s{seed} {mdl}"] = dict(runs=len(rs), receive_entered=sum(r["receive_entered"] for r in rs), semantic_ready=sum(r["phases"]["semantic_ready_s"] is not None for r in rs),
                                         release_b_entered=len(reached), handover_event=sum(r["handover_event"] for r in rs), task_success=sum(r["task_success"] for r in rs),
                                         release_b_success_of_reached=f"{sum(r['task_success'] for r in reached)}/{len(reached)}",
                                         first_failure_stage={k: sum(r["first_failure_stage"] == k for r in rs) for k in ("START", "RECEIVE", "RELEASE_B")},
                                         handover_event_but_not_success=sum(r["handover_event"] and not r["task_success"] for r in rs), projection=proj)
    return out


def decision(t):
    e5000, e5008, c5000 = t.get("s5000 E"), t.get("s5008 E"), t.get("s5000 C")
    if not (e5000 and e5008 and e5000["runs"] == 5 and e5008["runs"] == 5):
        return "INCOMPLETE: not all ten E runs of 5000 / 5008 completed; no decision"
    if e5000["task_success"] == 5 and e5008["task_success"] == 5:
        text = "E 5/5 on 5000 and 5/5 on 5008: kept as a frozen SIM reference configuration whose repeated behaviour was observed on these two fixed Train conditions; not a default, not generalization, not hardware"
    else:
        text = f"E failed at least once (5000 {e5000['task_success']}/5, 5008 {e5008['task_success']}/5); the existing checkpoints stay; no retraining, correction or retry is started"
    if c5000 and c5000["task_success"]:
        text += f". C also succeeded on 5000 ({c5000['task_success']}/{c5000['runs']}): E's repeated success and an effect of the correction of its own are different statements"
    return text + ". Seed 5002 stays a separate unresolved condition."


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
        print(f"[resumed job] protocol of {stored['fixed_before_execution']} (identity {ident['sha256'][:12]}); completed cases of this job are not run again and do not count as new repeats")
    else:
        proto.write_text(json.dumps(dict(
            run_id=RUN, name=LABEL, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"), identity=ident, cases=[list(c) for c in plan],
            order="repeats 1/3/5: 5000 C->E then 5008 E->C; repeats 2/4: 5000 E->C then 5008 C->E; last 5002 C->E; sequential, one new process per case, no parallel GPU job",
            budget="22 new original-reset runs (5000: C5 E5, 5008: C5 E5, 5002: C1 E1); no repeat until success, no replacement, no extension",
            denominators="the whole-run denominator keeps upstream failures; the RELEASE_B denominator is the runs that reached it; earlier runs are a reference table only",
            decision_rule="E 5/5 on both 5000 and 5008 -> kept as a frozen SIM reference configuration for these two fixed Train conditions (no default / generalization / hardware promotion); "
                          "C success on 5000 is reported as it is; any E failure -> counts per parent and the first failing policy, nothing retrained; 5002 reported apart",
            not_called=["select", "teacher", "train", "any optimizer step", "conditional-state evaluation"],
            past_settings="the effective determinism settings of the earlier R3 child processes are not in their logs: UNKNOWN; this job records its own children",
            repeats="re-runs of the same initial condition; not independent initial conditions, robustness or generalization",
            training=False, teacher_collection=False, val_test=False, hardware_access=False), indent=2))
    status = []
    for case in plan:
        seed, mdl, rep = case
        out = case_dir(ident, *case)
        if complete(out, ident, case) is not None:
            status.append(dict(case=list(case), status="completed (same job resumed: stored case of this identity, not a new repeat)"))
            print(f"[stored case of this job] {case}")
            continue
        if out.exists():
            status.append(dict(case=list(case), status="NOT RUN: an incomplete folder of this identity exists; neither reused nor overwritten", path=str(out)))
            print(f"[not run] {case}: incomplete folder {out}")
            continue
        started = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--seed", str(seed), "--model", mdl, "--rep", str(rep), "--identity", ident["sha256"]],
                              capture_output=True, text=True)
        (run_dir() / "logs" / f"{ident['sha256'][:12]}_s{seed}_{mdl}_rep{rep}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        ok = done.returncode == 0 and complete(out, ident, case) is not None
        status.append(dict(case=list(case), started=started, returncode=done.returncode, status="completed (new run)" if ok else "EXECUTOR ERROR (not a task result)",
                           **({} if ok else dict(stderr_tail=done.stderr[-600:]))))
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(line[-1] if ok and line else f"[EXECUTOR ERROR, not a task failure] {case}: returncode {done.returncode}: {done.stderr[-300:]}", flush=True)
    results = [r for case in plan if (r := complete(case_dir(ident, *case), ident, case)) is not None]
    t = tally(results)
    settings = [r["child_settings"] for r in results]
    same = lambda key: len({json.dumps(s[key], sort_keys=True, default=str) for s in settings}) <= 1
    (run_dir() / "results.json").write_text(json.dumps(dict(
        run_id=RUN, note=f"{LABEL}; switches read SIM ground truth; fixed weights; Train parents; new runs only (earlier R3 runs are not in these denominators); repeats are re-runs of one initial condition",
        identity_sha256=ident["sha256"], identity_unchanged_during_run=identity()["sha256"] == ident["sha256"], planned_cases=len(plan), completed_cases=len(results),
        executor_errors=[s for s in status if not s["status"].startswith("completed")], execution_status=status, tally=t, decision=decision(t), trajectories=trajectories(results),
        child_settings=dict(first=settings[0] if settings else None, pids_distinct=len({s["pid"] for s in settings}),
                            identical_across_children={k: same(k) for k in ("policy_training_flag", "torch_threads", "cudnn_benchmark", "cudnn_deterministic", "deterministic_algorithms",
                                                                            "deterministic_warn_only", "torch", "cuda", "mujoco", "environment", "gl")} if settings else None,
                            torch_initial_seeds=sorted({s["rng"]["torch_initial_seed"] for s in settings})),
        table=[table_row(r) for r in results], results=results), indent=2, default=str))
    print(f"[written] {run_dir() / 'results.json'}: {len(results)} of {len(plan)} cases completed, {len([s for s in status if not s['status'].startswith('completed')])} executor error(s) / not run; "
          f"identity {ident['sha256'][:12]}")
    summary(args)


def summary(args=None):
    path = run_dir() / "results.json"
    if not path.exists():
        print(f"RECEIVE R3 frozen recheck ({RUN}): unavailable, no results.json")
        return
    data = load(path)
    fmt = lambda v: "-" if v is None else (f"{v:.2f}" if isinstance(v, float) else str(v))
    print(f"\n=== RECEIVE R3 frozen recheck ({RUN}) — {LABEL}; 전환은 SIM 정답 기반. 신규 실행만(과거 R3 결과와 합산하지 않음) ===")
    print(f"C = RECEIVE_REPLAY_CONTROL, E = RECEIVE_ONSET_RECOVERY, 가중치 고정. 완료 {data['completed_cases']}/{data['planned_cases']}건, 실행기 오류 {len(data['executor_errors'])}건, "
          f"identity {data['identity_sha256'][:12]}. 같은 reset의 재실행이며 독립 초기조건이 아니다.")
    print("\n| seed | 모델 | 실행 | RECEIVE 진입 | 수신 준비 | RELEASE_B 진입 | handover 이벤트 | 전체 유효 성공 | RELEASE_B 도달 중 성공 | 첫 실패 START/RECEIVE/RELEASE_B |\n|---|---|---|---|---|---|---|---|---|---|")
    for k, v in data["tally"].items():
        seed, mdl = k.split()
        f = v["first_failure_stage"]
        print(f"| {seed[1:]} | {mdl} | {v['runs']} | {v['receive_entered']} | {v['semantic_ready']} | {v['release_b_entered']} | {v['handover_event']} | {v['task_success']} | "
              f"{v['release_b_success_of_reached']} | {f['START']}/{f['RECEIVE']}/{f['RELEASE_B']} |")
    print("\n| seed | 모델 | 회 | 왼팔 출발 | 수신 준비 | RELEASE_B | handover | 첫 fault | 최종 지지 | 성공 |\n|---|---|---|---|---|---|---|---|---|---|")
    for t in data["table"]:
        depart = "-" if not t["transfer"] else fmt(t["transfer"]["left_arm_departs_s"])
        print(f"| {t['seed']} | {t['receive']} | {t['rep']} | {depart} | {fmt(t['semantic_ready_s'])} | {fmt(t['release_b_entry_s'])} | {fmt(t['handover_event_s'])} | {fmt(t['first_fault_s'])} | "
              f"{t['final_support']} | {'예' if t['task_success'] else '아니오'} |")
    print(f"\n판단: {data['decision']}")
    print(f"  결과: {path}\n  보고서: {run_dir() / 'REPORT.md'}\n  영상: {run_dir() / 'videos'}")


def video(args):
    import recovery_trial as rt
    ident = identity()
    data = load(run_dir() / "results.json")

    def row_of(case):
        r = complete(case_dir(ident, *case), ident, case)
        if r is None:
            print(f"[unavailable] {case}: no completed case under this identity; the row is not replaced")
            return None
        row = rt.teacher_row(f"FROZEN RECHECK {r['chain']} rep{r['repeat']}", Path(r["path"]))
        ev = dict(r["events"])
        ev.update({f"SWITCH->{'RELEASE_B' if s['to'] == 'B' else s['to']}": s["sim_time_s"] for s in r["switches"]})
        row["events"] = dict(sorted(ev.items(), key=lambda kv: kv[1]))
        row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}" + ("" if r["task_success"] else f" (first failure in {r['first_failure_stage']})"),
                           "VALID HANDOVER, right hand holds" if r["task_success"] else f"NO VALID HANDOVER; final support: {r['final_support']}")
        return row

    jobs = [(f"seed{seed}_first_C_E_rep1_{ident['sha256'][:12]}.mp4", [(seed, "C", 1), (seed, "E", 1)], f"seed {seed} original reset, first pair in the fixed order") for seed in (5000, 5008)]
    failed = next((t for t in data["table"] if t["receive"] == "E" and t["seed"] in (5000, 5008) and not t["task_success"]), None)
    if failed is not None:
        jobs.append((f"first_E_failure_seed{failed['seed']}_rep{failed['rep']}_{ident['sha256'][:12]}.mp4", [(failed["seed"], "E", failed["rep"])], f"first failed E run: seed {failed['seed']} rep {failed['rep']}"))
    for name, group, title in jobs:
        out = run_dir() / "videos" / name
        if out.exists():
            print(f"[exists] {out}")
            continue
        rows = [x for x in (row_of(c) for c in group) if x is not None]
        if rows:
            rt.render(rows, out, f"{LABEL} / {title}", 2000, False, right_gripper=True)
            print(f"[written] {out}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("run", "summary", "video"):
        sub.add_parser(name)
    o = sub.add_parser("one")
    o.add_argument("--seed", type=int, choices=(5000, 5002, 5008), required=True)
    o.add_argument("--model", choices=("C", "E"), required=True)
    o.add_argument("--rep", type=int, choices=(1, 2, 3, 4, 5), required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    dict(run=run, one=one, summary=summary, video=video)[args.command](args)


if __name__ == "__main__":
    main()
