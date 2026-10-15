#!/usr/bin/env python3
"""POST-C0 R1 / work6: A / G switch check, a local change of the teacher's insertion contact rule, and a small Train
correction-collection pilot (SIM only; 0 training updates; no hardware; no Val / Test).

Execution code (part of the run identity). Tables, frames and the staging manifest are made by post_c0_pilot_r1_report.py.

  protocol   every fixed case, the decision rules and the 12 pilot sources, written before any run
  ag         14 student runs: S4 (5000, FROZEN -> ADAPT) A x3 / G x3, R1 (5098, R1_E -> ADAPT) A x2 / G x2, R2 (5008, FROZEN -> FROZEN) A x2 / G x2
  teacher    static: the seven 5008 candidates first rejected only by right pad - block contact, before / after the derived rule;
             dynamics: the candidates whose whole preview is valid (at most 7) and the 5002 / 5098 controls (2)
  decide     contract (A or G) and teacher (original or derived) by the rules of the protocol
  pilot      12 further Train parents: START_R1_E from the original reset, the teacher takes RECEIVE, frozen RELEASE_B ends
  all        every step that is not complete yet, two worker processes at a time

In a mixed run the teacher sits where the RECEIVE student sat in the unchanged executor: same recorder, same repaired
readiness switch to RELEASE_B, same absolute 40 s, same checker. It is a data-collection run, never a student result.
"""
import argparse
import hashlib
import importlib.util
import json
import os
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
import receive_r3_recheck as rk   # noqa: E402
import start_pick_d1 as d1   # noqa: E402
import start_pick_r1 as p1   # noqa: E402
import receive_bridge_r1 as br   # noqa: E402
import constraint_causal_c0 as c0   # noqa: E402
import server_s0 as s0   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "post-c0-pilot-r1-work6-20261006-01"
TITLE = "POST-C0 R1 / work6 / A-G switch check, teacher insertion contact, Train correction pilot"
LABEL = "SIM-ORACLE-GATED THREE-POLICY / POST-C0 R1 work6"
WORKERS = 2
TEACHER_SOURCE = FACTORY_ROOT / "source/production_source_g000/sim_data_factory/teacher.py"
DERIVED = Path(__file__).resolve().parent / "derived" / "teacher_right_pad_insert.py"
DERIVED_NAME = "teacher_right_pad_insert_v1"
AG = dict(S4=(5000, "FROZEN", "ADAPT", 3), R1=(5098, "R1E", "ADAPT", 2), R2=(5008, "FROZEN", "FROZEN", 2))
CONTROL_ENTRY = {5002: 375, 5098: 350}
LOCAL_SEED, LOCAL_STEP = 5008, 400
SOURCES = c0.SOURCES + ("factory/v1/post_c0_pilot_r1.py",)

load, sha, array_sha = rk.load, rk.sha, rk.array_sha


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def ag_cases():
    """S4: A, G, G, A, A, G (the first of each pair alternates); guards: A, G, G, A."""
    out = [["S4_A_1", "S4_G_1", "S4_G_2", "S4_A_2", "S4_A_3", "S4_G_3"], ["R1_A_1", "R1_G_1", "R1_G_2", "R1_A_2"], ["R2_A_1", "R2_G_1", "R2_G_2", "R2_A_2"]]
    return out


def pilot_sources():
    rows = [(i, e) for i, e in enumerate(d1.manifest()) if e["split"] == "train" and e["training_eligible"]]
    out, shortage = [], {}
    for gen in ("G000", "G001"):
        rest = sorted([(e["seed"], i, e) for i, e in rows if e["generation"] == gen and e["seed"] not in p1.TRAIN9])
        shortage[gen] = dict(remaining=len(rest), wanted=6, short=max(0, 6 - len(rest)))
        for seed, i, e in rest[:6]:
            meta = load(Path(e["path"]) / "metadata.json")
            out.append(dict(seed=seed, manifest_index=i, generation=gen, split=e["split"], parent_physics_group=e["parent_physics_group"],
                            reset_sha256=hashlib.sha256(json.dumps([meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"]], sort_keys=True).encode()).hexdigest()))
    return out, shortage


# ----------------------------------------------------------------------------------------------
# Derived teacher: right pad - block contact allowed in the insertion preview / live insertion only
# ----------------------------------------------------------------------------------------------

EDITS = [
    ("collisions: an allowed right pad - block contact still fails above the checker's penetration limit (derived insertion allowance only)",
     "                if other in self.cm.pad_index and self.cm.pad_index[other][0] in allow_pad_block:\n                    continue\n",
     "                if other in self.cm.pad_index and self.cm.pad_index[other][0] in allow_pad_block:\n"
     "                    cap = getattr(allow_pad_block, 'right_pad_cap_m', None)\n"
     "                    if cap is not None and self.cm.pad_index[other][0] == 'right' and -c.dist > cap:\n"
     "                        bad.append(('block', self.m.geom(other).name, 'right pad penetration above the checker limit'))\n"
     "                    continue\n"),
    ("_select_handover: the ranked candidates are numbered; one of them can be evaluated alone",
     "        for clear, p, a_l, j_l, a_r, n, beta, depth, goal_l, grasp, ik in ranked:\n",
     "        for rank_index, (clear, p, a_l, j_l, a_r, n, beta, depth, goal_l, grasp, ik) in enumerate(ranked):\n"
     "            if getattr(self, 'only_rank', None) is not None and rank_index != self.only_rank:\n"
     "                continue\n"
     "            self.rank_log.append(rank_index)\n"),
    ("_select_handover: stage -> pregrasp preview", "plan.path_collisions(base, both, pre_c, carry=carry, allow_pad_block=('left',))", "plan.path_collisions(base, both, pre_c, carry=carry, allow_pad_block=self._allow('approach'))"),
    ("_select_handover: pregrasp -> grasp (insertion) preview", "plan.path_collisions(base, pre_c, grasp_c, carry=carry, allow_pad_block=('left',))", "plan.path_collisions(base, pre_c, grasp_c, carry=carry, allow_pad_block=self._allow('insert'))"),
    ("_plan_right_grasp: live pregrasp path", "carry=('left', rel),\n                                   allow_pad_block=('left',))", "carry=('left', rel),\n                                   allow_pad_block=self._allow('approach'))"),
    ("_plan_right_grasp: live insertion", "plan.path_collisions(base, pre_c, grasp_c, carry=('left', rel), allow_pad_block=('left',))", "plan.path_collisions(base, pre_c, grasp_c, carry=('left', rel), allow_pad_block=self._allow('insert'))"),
]


class InsertAllow(tuple):
    """allow_pad_block of the derived insertion checks: the usual tuple, plus the penetration limit for the right pad."""
    right_pad_cap_m = None


def build_derived():
    """Write the derived teacher next to this tool (the original file is only read). Each edit must match exactly once."""
    src = TEACHER_SOURCE.read_text()
    out = src
    for label, old, new in EDITS:
        if out.count(old) != 1:
            raise RuntimeError(f"derived teacher: '{label}' matches {out.count(old)} times in the original source")
        out = out.replace(old, new)
    header = f'# DERIVED from sim_data_factory/teacher.py (sha256 {hashlib.sha256(src.encode()).hexdigest()}) by post_c0_pilot_r1.build_derived: {DERIVED_NAME}.\n# The original teacher is unchanged; with allow_right_pad_insert = False this file plans exactly as the original.\n'
    DERIVED.parent.mkdir(exist_ok=True)
    if not DERIVED.exists() or DERIVED.read_text() != header + out:
        DERIVED.write_text(header + out)
    return dict(name=DERIVED_NAME, file=str(DERIVED), sha256=sha(DERIVED), original=str(TEACHER_SOURCE), original_sha256=sha(TEACHER_SOURCE), edits=[e[0] for e in EDITS])


def derived_module():
    name = "sim_data_factory.teacher_right_pad_insert"
    if name in sys.modules:
        return sys.modules[name]
    import sim_data_factory  # noqa: F401
    spec = importlib.util.spec_from_file_location(name, DERIVED)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def make_teacher(model, version, only_rank=None, scope=("insert",)):
    """version 'original': the unchanged teacher (hc.reentry_teacher). 'derived': the derived module with the insertion allowance.
    'derived_off': the derived module with the allowance off (must plan exactly as the original; used to number candidates)."""
    import handover_correction_trial as hc
    from sim_data_factory.task_success import CheckerConfig
    if version == "original":
        return hc.reentry_teacher(model)
    mod = derived_module()
    cap = CheckerConfig().max_penetration_m

    class ReentryTeacher(mod.HandoverTeacher):                             # the re-entry of handover_correction_trial.reentry_teacher, on the derived class
        def __init__(self, m):
            super().__init__(m)
            self.phase, self.reentry, self.rank_log = "reenter", None, []
            self.allow_right_pad_insert, self.only_rank, self.allow_scope = version == "derived", only_rank, tuple(scope)

        def _allow(self, which):
            if self.allow_right_pad_insert and which in self.allow_scope:
                a = InsertAllow(("left", "right"))
                a.right_pad_cap_m = cap
                return a
            return ("left",)

        def _advance(self, d, ev, dt):
            if self.phase != "reenter":
                return super()._advance(d, ev, dt)
            cfg = self.cfg
            fl, fr = np.asarray(ev["pad_force_N"]["left"]), np.asarray(ev["pad_force_N"]["right"])
            lifted = ev["table_support_N"] == 0 and ev["block_bottom_clearance_m"] >= 0.02
            left, right = fl.min() >= 0.5 * cfg.grip_target_N * 0.3, fr.min() >= 0.5 * cfg.grip_target_N * 0.3
            if not (lifted and left):
                raise mod.TeacherFailure(f"re-entry: no lifted left hold (left pads {fl.tolist()}, clearance {ev['block_bottom_clearance_m']:.3f})")
            self.grip["left"] = dict(mode="force", stable=cfg.grip_confirm_s)
            if right:
                s = self.plan.site["right"]
                rot = d.site_xmat[s].reshape(3, 3).copy()
                self.info["right_axes"] = (rot[:, 0], rot[:, 2])
                self.grip["right"] = dict(mode="force", stable=0.0)
                self.reentry = "both hands hold -> right_grasp (force confirmation, then left_release)"
                self.phase = "right_grasp"
            else:
                self._plan_transfer(d)
                self.reentry = "left-only lifted hold -> transfer planning from this state"
                self.phase = "transfer_approach"

    return ReentryTeacher(model)


def static_plan(seed, state, version, only_rank=None, scope=("insert",)):
    """One planning call on a restored state, nothing stepped. Returns the failure / the chosen plan and every rejected 24-sample path check."""
    import mujoco
    import frozen_transfer_check as ft
    from sim_data_factory.reset_sampler import make_task_episode
    meta = ft.demonstration(seed)["metadata"]
    ctx = make_task_episode(seed, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
    m, d = ctx["model"], ctx["data"]
    mujoco.mj_setState(m, d, state, int(mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_forward(m, d)
    teacher = make_teacher(m, version, only_rank, scope)
    plan, hits, tried_ref, calls = teacher.plan, [], [], [0]
    orig_path, orig_select = plan.path_collisions, teacher._select_handover

    def path(base_qpos, start, goal, carry=None, samples=24, allow_pad_block=(), block_qpos=None):
        hit = orig_path(base_qpos, start, goal, carry=carry, samples=samples, allow_pad_block=allow_pad_block, block_qpos=block_qpos)
        calls[0] += 1
        if hit is not None:
            ranks = getattr(teacher, "rank_log", [])
            hits.append(dict(call=calls[0], rank=ranks[-1] if ranks else None, samples=samples, u=round(hit[0], 3), right_pad_allowed="right" in allow_pad_block,
                             moved=dict(left_arm=bool(np.abs((goal - start)[0:5]).max() > 1e-9), right_arm=bool(np.abs((goal - start)[6:11]).max() > 1e-9), left_gripper=bool(abs((goal - start)[5]) > 1e-9)),
                             pairs=sorted({" | ".join(map(str, b)) for b in hit[1]})))
        return hit

    def select(d_, rel, base, carry, tried):
        tried_ref.append(tried)
        return orig_select(d_, rel, base, carry, tried)

    plan.path_collisions, teacher._select_handover = path, select
    teacher.next_command(ctx)
    tried = tried_ref[0] if tried_ref else []
    info = {k: (np.asarray(v).tolist() if isinstance(v, (np.ndarray, list, tuple)) else v) for k, v in teacher.info.items() if k in (
        "handover_point", "planned_right_beta_deg", "planned_right_depth_m", "planned_gripper_clearance_mm", "transfer_order", "handover_candidates_tried", "handover_clearance_top_mm")}
    return dict(version=version, only_rank=only_rank, scope=list(scope), failure=teacher.failure, reentry=teacher.reentry, phase=teacher.phase, plan=info, ranks_evaluated=list(getattr(teacher, "rank_log", [])),
                tried=[list(map(lambda x: x if isinstance(x, (int, float, str)) else np.asarray(x).round(4).tolist(), t)) for t in tried], rejected_path_checks=hits)


def teacher_static(args=None):
    target = run_dir() / "teacher_local" / "static.json"
    if target.exists():
        return load(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    derived = build_derived()
    g = rc.stored_state(br.source_case(LOCAL_SEED), LOCAL_STEP)
    state = g["states"][-1]
    right_pad = lambda p: p.startswith("block | right_pgripper_pad")
    orig, off = static_plan(LOCAL_SEED, state, "original"), static_plan(LOCAL_SEED, state, "derived_off")
    same = (str(orig["failure"]) == str(off["failure"]) and orig["tried"] == off["tried"] and [(h["u"], h["pairs"]) for h in orig["rejected_path_checks"]] == [(h["u"], h["pairs"]) for h in off["rejected_path_checks"]])
    control_same = {}
    for seed, step in CONTROL_ENTRY.items():
        st = rc.stored_state(br.source_case(seed), step)["states"][-1]
        a, b = static_plan(seed, st, "original"), static_plan(seed, st, "derived_off")
        control_same[str(seed)] = dict(same_plan=a["plan"] == b["plan"] and str(a["failure"]) == str(b["failure"]), plan=a["plan"], derived_on=static_plan(seed, st, "derived")["plan"])
    # the candidates whose FIRST rejection is right pad - block contact alone
    first = {}
    for h in off["rejected_path_checks"]:
        if h["samples"] == 24 and h["rank"] is not None:
            first.setdefault(h["rank"], h)
    seven = [r for r, h in sorted(first.items()) if all(right_pad(p) for p in h["pairs"])]
    rows = []
    for r in seven:
        before = static_plan(LOCAL_SEED, state, "derived_off", only_rank=r)
        hb = before["rejected_path_checks"][0] if before["rejected_path_checks"] else None
        n_checks_before = hb["call"] if hb else None                         # 1 = stage -> pregrasp, 2 = pregrasp -> grasp
        scope = ("approach", "insert") if n_checks_before == 1 else ("insert",)
        after = static_plan(LOCAL_SEED, state, "derived", only_rank=r, scope=scope)
        reason_after = None if after["failure"] is None else (after["tried"][-1][0] if after["tried"] else "other")
        rows.append(dict(rank=r, candidate=dict(handover_point=before["tried"][-1][1] if before["tried"] else None, right_beta_deg=before["tried"][-1][2] if before["tried"] else None),
                         before=dict(reason=before["tried"][-1][0] if before["tried"] else None, rejecting_check="stage -> pregrasp" if n_checks_before == 1 else "pregrasp -> grasp (insertion)",
                                     u=None if hb is None else hb["u"], pairs=None if hb is None else hb["pairs"]),
                         scope_used=list(scope), after=dict(first_rejection_resolved=reason_after != "right_insert_collision", whole_preview_valid=after["failure"] is None, reason=reason_after,
                                                            rejected_path_checks=after["rejected_path_checks"][:4], plan=after["plan"] if after["failure"] is None else None)))
    data = dict(run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), derived=derived, seed=LOCAL_SEED, step=LOCAL_STEP, state_sha256=array_sha(state), dynamics_steps=0,
                derived_with_allowance_off_equals_original=dict(local_5008=bool(same), controls=control_same),
                original=dict(failure=orig["failure"], candidates_tried=len(orig["tried"]), ranked_candidates_evaluated=len(off["ranks_evaluated"]),
                              u0_left_arm_self_contact_checks=sum(h["u"] == 0 and any("left_shoulder" in p for p in h["pairs"]) for h in orig["rejected_path_checks"]),
                              note="a self-contact at u = 0 is in the planner's posed candidate (left arm at the candidate hand-over pose), not in the student's present pose"),
                seven=seven, expected_seven=7, rows=rows, first_rejection_resolved=sum(r["after"]["first_rejection_resolved"] for r in rows), whole_preview_valid=[r["rank"] for r in rows if r["after"]["whole_preview_valid"]],
                rule="right gripper pad - block contact in the labelled insertion check that rejected the candidate is 'allowed' instead of 'rejected', while its depth stays within the checker's 0.002 m; "
                     "every other contact, the rest of the insertion and the release / retreat / transfer previews are checked as before; the same allowance applies to the live check of that phase")
    target.write_text(json.dumps(data, indent=2, default=str))
    print(json.dumps(dict(equal_off=data["derived_with_allowance_off_equals_original"]["local_5008"], controls={k: v["same_plan"] for k, v in control_same.items()}, seven=seven,
                          rows=[(r["rank"], r["before"]["rejecting_check"], r["after"]["first_rejection_resolved"], r["after"]["whole_preview_valid"], r["after"]["reason"]) for r in rows]), default=str))
    return data


# ----------------------------------------------------------------------------------------------
# The teacher inside the unchanged executor
# ----------------------------------------------------------------------------------------------

class TeacherSeat:
    """What load_policy returns for RECEIVE in a mixed run; the actor is made when the executor seats it."""
    def __init__(self, version, only_rank, scope):
        self.version, self.only_rank, self.scope = version, only_rank, scope

    def reset(self):
        pass


class TeacherActor:
    """Sits where a Student sits: next_command -> (canonical command, diagnostics). It never has a C25 boundary, so the executor
    looks at the repaired RECEIVE -> RELEASE_B readiness on every 50 Hz step, exactly as for a RECEIVE student."""
    def __init__(self, seat):
        self.seat, self.teacher, self.failure = seat, None, None
        self.chunk, self.chunk_index, self.execution_prefix = np.zeros((50, 12), dtype=np.float32), 0, rc.PREFIX
        self.trace, self.chunks, self.calls, self.planner_s = [], [], 0, 0.0

    def discard_chunk(self):
        pass

    def next_command(self, context):
        from sim_data_factory import core
        if self.teacher is None:
            self.teacher = make_teacher(context["model"], self.seat.version, self.seat.only_rank, self.seat.scope)
        q = core.encode_state(context["model"], context["data"]).astype(np.float32)
        t0 = time.perf_counter()
        command, diag = self.teacher.next_command(context)
        dt = time.perf_counter() - t0
        self.planner_s += dt
        command = np.asarray(command, dtype=float)
        if self.teacher.failure:
            f = self.teacher.failure
            self.failure = dict(reason="teacher_failure", detail=str(f.get("reason")), phase=f.get("phase"), time=float(f.get("time", context["data"].time)))
        if not self.chunks:
            self.chunks.append(dict(id=0, time=float(context["data"].time), anchor=q.copy(), actions=np.zeros((50, 12), dtype=np.float32)))
        self.calls += 1
        self.trace.append(dict(time=float(context["data"].time), q_measured=q.tolist(), q_raw=command.tolist(), q_cmd=command.tolist(), projection_mask=[False] * 10, q_applied_minus_raw=[0.0] * 12,
                               selected_chunk_index=0, inference_wall_s=0.0, planner_wall_s=dt, teacher_phase=diag.get("phase"), controller="teacher " + self.seat.version))
        return command, diag


def install_runtime(spec, timers):
    """Patches of this child process: the C0 gate option, the teacher seat, and timers."""
    from sim_data_factory import core, student_eval, task_success
    log = dict(faults=0, first=None, pairs={})
    c0.install(dict(passthrough=False, gate50=spec.get("gate50", False), shadow=False), log)
    Student = student_eval.Student
    device_log = spec.setdefault("_devices", {})

    def seat(policy, stats, sensor, prefix, coeff, bound):
        if isinstance(policy, TeacherSeat):
            return TeacherActor(policy)
        return Student(policy, stats, sensor, prefix, coeff, bound)

    student_eval.Student = seat

    def timed(owner, name, key):
        original = getattr(owner, name)

        def wrapper(*a, **k):
            t0 = time.perf_counter()
            try:
                return original(*a, **k)
            finally:
                timers[key] = timers.get(key, 0.0) + time.perf_counter() - t0

        setattr(owner, name, wrapper)

    timed(student_eval.Observations, "observe", "render_observe_s")
    timed(Student, "next_command", "student_next_command_s")
    timed(core, "rollout", "rollout_total_s")
    timed(task_success, "evaluate_states", "checker_s")
    return log, device_log


def cuda_evidence(loaded, stored_case=None):
    """Where the ACT parameters, buffers, inputs and outputs really are, and CUDA kernels of a short profiled forward. The
    profiled forwards run after the rollout on a stored observation with the torch RNG state put back."""
    import torch
    from torch.profiler import profile, ProfilerActivity
    from sim_data_factory.r2_data import policy_observation
    from dapier_act_policy import MEASURED_Q_KEY
    out = dict(pid=os.getpid(), torch=torch.__version__, cuda=torch.version.cuda, cuda_available=torch.cuda.is_available())
    try:
        q = subprocess.run(["nvidia-smi", "--query-gpu=uuid,name,memory.used,utilization.gpu", "--format=csv,noheader"], capture_output=True, text=True, timeout=20).stdout.strip()
        apps = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader"], capture_output=True, text=True, timeout=20).stdout.strip().splitlines()
        out.update(nvidia_smi_gpu=q, nvidia_smi_compute_apps=apps, this_pid_listed=any(x.split(",")[0].strip() == str(os.getpid()) for x in apps))
    except (OSError, subprocess.SubprocessError) as e:
        out["nvidia_smi"] = f"unavailable: {e}"
    policies = {k: v[0] for k, v in loaded.items() if not isinstance(v[0], TeacherSeat)}
    out["policies"] = {}
    for name, policy in policies.items():
        pd, bd = sorted({str(p.device) for p in policy.parameters()}), sorted({str(b.device) for b in policy.buffers()})
        out["policies"][name] = dict(parameter_devices=pd, buffer_devices=bd, parameters=int(sum(p.numel() for p in policy.parameters())), all_on_cuda=all(d.startswith("cuda") for d in pd + bd))
    if policies and stored_case is not None:
        name, policy = next(iter(policies.items()))
        stats = loaded[name][1]
        with np.load(Path(stored_case) / "episode.npz", allow_pickle=False) as a:
            q0 = a["q_state"][0].astype(np.float32)
        rgb = {s: np.load(Path(stored_case) / f"cameras/{s}_wrist_rgb.npz")["rgb"][0] for s in ("left", "right")}
        cpu_state, cuda_state = torch.get_rng_state(), torch.cuda.get_rng_state()
        batch = {k: v[None] for k, v in policy_observation(q0, rgb, stats, device="cuda").items()}
        batch[MEASURED_Q_KEY] = torch.tensor(q0, device="cuda")[None]
        seen = {}
        hook = policy.model.register_forward_hook(lambda module, args, output: seen.update(forward_output_device=str((output[0] if isinstance(output, (tuple, list)) else output).device)))
        try:
            with torch.inference_mode(), profile(activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA]) as prof:
                for _ in range(3):
                    chunk = policy.predict_canonical_chunk(batch)
                torch.cuda.synchronize()
        finally:
            hook.remove()
            torch.set_rng_state(cpu_state)
            torch.cuda.set_rng_state(cuda_state)
        rows = []
        for e in prof.key_averages():
            t = getattr(e, "device_time_total", None)
            t = getattr(e, "cuda_time_total", 0) if t is None else t
            if t and t > 0:
                rows.append((float(t), e.key, int(e.count)))
        rows.sort(reverse=True)
        out["profiled_forward"] = dict(policy=name, forwards=3, input_devices={k: str(v.device) for k, v in batch.items()}, model_output_device=seen.get("forward_output_device"),
                                       canonical_chunk_device_before_numpy=str(chunk.device), cuda_kernel_events=len(rows), top=[dict(name=k[:60], device_time_us=round(t, 1), calls=c) for t, k, c in rows[:8]],
                                       rng_state_restored=True, note="the action goes to numpy on the CPU after this, as in the executor")
    return out


def identity():
    import arm_output_trial as trial
    models = c0.bridge_models()
    sources, shortage = pilot_sources()
    body = dict(run_id=RUN, models=models, manifest_sha256=p1.MANIFEST_SHA, ag=[c for g in ag_cases() for c in g], pilot_sources=[s["seed"] for s in sources], workers=WORKERS,
                sources={p: sha(FACTORY_ROOT / p) for p in SOURCES}, derived_teacher_edits=hashlib.sha256(json.dumps(EDITS).encode()).hexdigest(), contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                execution=dict(chunk=50, prefix=rc.PREFIX, control_hz=round(1 / td.DT), horizon_steps=rc.HORIZON, torch_seed=4100, arm_bound_policy="project", ensemble="off", smoothing="off",
                               A="C25 START -> RECEIVE switch", G="the same semantic condition on every 50 Hz step (the C0 option)", mixed="the teacher takes the RECEIVE seat of the unchanged executor"),
                backend=trial.environment_record(), host=s0.host(), environment={k: os.environ.get(k) for k in rk.ENV_KEYS})
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def protocol(args=None):
    target = run_dir() / "protocol.json"
    if target.exists():
        return load(target)
    run_dir().mkdir(parents=True, exist_ok=True)
    ident = identity()
    sources, shortage = pilot_sources()
    record = dict(
        run_id=RUN, name=TITLE, label=LABEL, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"), training_updates=0, hardware_access=False, val_test_used=False, identity=ident,
        ag=dict(groups=ag_cases(), configs={k: dict(seed=v[0], start=v[1], receive=v[2], repeats_each=v[3]) for k, v in AG.items()},
                note="repeats are consistency checks of one reset, not independent conditions; C0 results are not in these denominators",
                contract_rule="G is this pilot's experimental contract only if S4 G is 3/3 strict, S4 A is not 3/3, all 8 guard runs (R1, R2; A and G) are strict, and within every configuration the "
                              "stretch before the first semantic state (q, wrist RGB, issued commands) is the same in all its runs; otherwise A. Not a default, not a statistical claim"),
        teacher_local=dict(source=dict(seed=LOCAL_SEED, step=LOCAL_STEP, case=str(br.source_case(LOCAL_SEED))), controls={str(k): v for k, v in CONTROL_ENTRY.items()}, derived=DERIVED_NAME,
                           candidates="the ranked candidates of the unchanged order whose first rejection is right pad - block contact alone (seven in C0); no new search, no more candidates",
                           budget="one dynamics run per candidate whose whole preview is valid after the change (at most 7), one per control (2); no retry",
                           teacher_rule="the derived teacher is this pilot's teacher only if it makes at least one more candidate valid over the whole preview, at least one of those gives a valid "
                                        "readiness prefix in dynamics, and both controls still give a valid readiness prefix and a valid mixed handover; otherwise the original teacher"),
        pilot=dict(sources=sources, shortage=shortage, selection="Train parents of the frozen manifest that are not in Train9; per generation the six lowest seeds; fixed before any result; no Val / Test",
                   run="START_R1_E from the original reset once; at the RECEIVE entry of the chosen contract the chosen teacher takes over in the same run; at the repaired readiness frozen RELEASE_B "
                       "takes over; absolute 40 s; no retry, no replacement", label="mixed data-collection runs (student START + teacher RECEIVE + student RELEASE_B), never a student-only result",
                   training_target="the teacher's commands of [RECEIVE entry, readiness) only; START's own commands are not relabelled as teacher actions"),
        budget="physical attempts: A/G 14 + teacher local at most 9 + pilot 12 = at most 35; static plan checks and unit checks are counted apart",
        parallel=dict(workers=WORKERS, note="two child processes at a time, the child settings of Bridge / C0 (4 torch threads); sum of case wall / batch wall is an overlap ratio, not a measured speed-up"),
        gpu="ACT inference must be on CUDA: every child records parameter / buffer / input / output devices and a short profiled forward; MuJoCo physics, IK / planning, file I/O and SHA run on the CPU")
    target.write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(identity=ident["sha256"][:12], ag=14, pilot=[s["seed"] for s in sources], shortage=shortage)))
    return record


def derived_scope():
    """The labelled checks in which the allowance is used: those of the candidates whose whole preview became valid (insertion only if none)."""
    rows = load(run_dir() / "teacher_local" / "static.json")["rows"]
    used = sorted({x for r in rows if r["after"]["whole_preview_valid"] for x in r["scope_used"]})
    return used or ["insert"]


def case_spec(case):
    kind = case.split(":")[0]
    if kind == "ag":
        config, cond, rep = case.split(":")[1].split("_")
        seed, start, receive, _ = AG[config]
        return dict(case=case, kind="ag", config=config, condition=cond, rep=int(rep), seed=seed, start=start, receive=receive, gate50=cond == "G", teacher=None, restore=None)
    if kind == "tl":
        _, seed, what = case.split(":")
        seed = int(seed)
        if what == "control":
            return dict(case=case, kind="teacher_local", seed=seed, start=None, receive="TEACHER", teacher=dict(version="derived", only_rank=None, scope=derived_scope()), restore=CONTROL_ENTRY[seed], gate50=False)
        static = load(run_dir() / "teacher_local" / "static.json")
        row = next(r for r in static["rows"] if r["rank"] == int(what))
        return dict(case=case, kind="teacher_local", seed=seed, start=None, receive="TEACHER", teacher=dict(version="derived", only_rank=int(what), scope=row["scope_used"]), restore=LOCAL_STEP, gate50=False)
    decision = load(run_dir() / "decision.json")
    seed = int(case.split(":")[1])
    return dict(case=case, kind="pilot", seed=seed, start="R1E", receive="TEACHER", teacher=dict(version=decision["teacher"], only_rank=None, scope=derived_scope()), restore=None, gate50=decision["contract"] == "G",
                contract=decision["contract"])


def case_dir(ident, case):
    return run_dir() / "runs" / ident["sha256"][:12] / case.replace(":", "_")


def complete(ident, case):
    out = case_dir(ident, case)
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = r.get("identity_sha256") == ident["sha256"] and r.get("pilot_case") == case and meta.get("complete") and "cuda" in r and (out / "episode.npz").exists()
    return r if ok else None


def one(args):
    from integration_scenes import task_env
    from sim_data_factory.task_success import evaluate_states
    ident = identity()
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    spec = case_spec(args.case)
    out = case_dir(ident, args.case)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    started, timers = time.time(), {}
    log, _ = install_runtime(spec, timers)
    models = ident["models"]
    names = dict(FROZEN="START_FROZEN", R1E="START_R1_E")
    if spec["start"] is not None:
        rc.MODELS["START"] = Path(models[names[spec["start"]]]["path"])
    if spec["receive"] != "TEACHER":
        rc.MODELS["RECEIVE"] = Path(models[dict(FROZEN="RECEIVE_FROZEN", ADAPT="RECEIVE_BRIDGE_ADAPT")[spec["receive"]]]["path"])
    loaded, original = {}, rc.load_policy

    def remember(name):
        if name == "RECEIVE" and spec["receive"] == "TEACHER":
            t = spec["teacher"]
            got = (TeacherSeat(t["version"], t["only_rank"], tuple(t["scope"])), rc.stats(), dict(checkpoint="teacher:" + t["version"], model_sha256=None, config_sha256=None, normalization_sha256=None))
        else:
            got = original(name)
        loaded[name] = got
        return got

    rc.load_policy = remember
    restore, chain = None, ("START", "RECEIVE", "B")
    if spec["restore"] is not None:
        restore = rc.stored_state(br.source_case(spec["seed"]), spec["restore"])
        chain = ("RECEIVE", "B")
    label = f"{LABEL}: {args.case} Train {spec['seed']}"
    try:
        result = rc.execute(out, spec["seed"], chain, restore, label, contract="repaired")
    finally:
        rc.load_policy = original
    rows, trace = load(out / "task_check_rows.json"), load(out / "action_trace.json")
    with np.load(out / "episode.npz", allow_pickle=False) as a:
        states, q = a["integration_state"].copy(), a["q_state"].copy()
    model = task_env("desk", grippers="both").model
    step0 = spec["restore"] or 0
    entry = next((s for s in result["switches"] if s["to"] == "RECEIVE"), None)
    gate = next((s for s in result["switches"] if s["to"] == "B"), None)
    entry_step = step0 if spec["restore"] is not None else (None if entry is None else entry["step"])
    events, fault = result["events"], result["first_fault"]
    strict = bool(result["task_success"]) and log["faults"] == 0
    stage = None if strict else ("START" if entry_step is None else "RECEIVE" if gate is None else "RELEASE_B")
    semantic = None
    if "pick_lift" in events and spec["restore"] is None:
        j = next((k for k, r in enumerate(rows) if r["time"] >= events["pick_lift"] - 1e-9 and rb.hands(r)[0]), None)
        if j is not None:
            rgb = {s: np.load(out / f"cameras/{s}_wrist_rgb.npz")["rgb"][:j + 1] for s in ("left", "right")}
            semantic = dict(first_semantic_step=j, first_semantic_s=round(j * td.DT, 2), switch_step=entry_step, wait_s=None if entry_step is None else round((entry_step - j) * td.DT, 2),
                            before=dict(q_sha256=array_sha(q[:j + 1]), state_sha256=array_sha(states[:j + 1]), rgb_sha256={s: array_sha(v) for s, v in rgb.items()},
                                        commands_sha256=array_sha(np.array([t["q_cmd"] for t in trace[:j]])), normalized_action_sha256=array_sha(np.array([t["normalized_action"] for t in trace[:j] if "normalized_action" in t]))),
                            at_first_semantic=br.handover_frame(model, states[j]), at_switch=None if entry_step is None else br.handover_frame(model, states[entry_step]))
    teacher_part = None
    if spec["receive"] == "TEACHER":
        t_rows = [t for t in trace if str(t.get("controller", "")).startswith("teacher")]
        ready_valid, prefix = False, None
        if gate is not None and entry_step is not None:
            hist = states if restore is None else np.concatenate([restore["states"], states[1:]])
            hq = q if restore is None else np.concatenate([restore["q"], q[1:]])
            chk = evaluate_states(model, hist[:gate["step"] + 1], hq[:gate["step"] + 1].astype(np.float64), control_dt=td.DT)
            ready_valid = bool(chk["valid"])
            prefix = dict(absolute_steps=[entry_step, gate["step"]], absolute_s=[round(entry_step * td.DT, 2), round(gate["step"] * td.DT, 2)], observations=gate["step"] - entry_step,
                          local_rows_in_this_episode=[entry_step - step0, gate["step"] - step0], checker_valid_up_to_readiness=ready_valid, failures=chk["failures"][:3],
                          entry_state_sha256=array_sha(hist[entry_step]), alignment="row k of episode.npz q_state / q_cmd / cameras is the observation and the command of absolute step step0 + k; "
                          "RECEIVE targets are q_cmd of the teacher rows only")
        teacher_part = dict(version=spec["teacher"]["version"], only_rank=spec["teacher"]["only_rank"], scope=spec["teacher"]["scope"], entered=entry_step is not None, commands=len(t_rows),
                            phases=list(dict.fromkeys(t["teacher_phase"] for t in t_rows)), planner_wall_s=round(sum(t["planner_wall_s"] for t in t_rows), 2),
                            teacher_ready_valid=ready_valid, prefix=prefix, mixed_full_handover_valid=strict,
                            failure=fault if (fault or {}).get("reason") == "teacher_failure" else None, right_first_pad_touch_s=next((round(r["time"], 2) for r in rows if max(r["pad_force_N"]["right"]) > 0), None))
    stored = out
    cuda = cuda_evidence(loaded, stored_case=stored if spec["restore"] is None else br.source_case(spec["seed"]))
    inference = float(sum(t.get("inference_wall_s", 0.0) for t in trace))
    wall = time.time() - started
    timing = dict(wall_s=round(wall, 1), act_inference_s=round(inference, 2), render_observe_s=round(timers.get("render_observe_s", 0.0), 2), planner_s=None if teacher_part is None else teacher_part["planner_wall_s"],
                  checker_s=round(timers.get("checker_s", 0.0), 2), rollout_total_s=round(timers.get("rollout_total_s", 0.0), 2),
                  physics_and_recording_s=round(timers.get("rollout_total_s", 0.0) - timers.get("render_observe_s", 0.0) - timers.get("student_next_command_s", 0.0) - (0.0 if teacher_part is None else teacher_part["planner_wall_s"]), 2),
                  student_pre_post_s=round(timers.get("student_next_command_s", 0.0) - inference, 2), inference_calls=sum(1 for t in trace if t.get("inference_wall_s", 0.0) > 0),
                  note="physics_and_recording = rollout - render - controller calls (mj_step, snapshots, recorder); the rest of the wall time is model loading, file I/O, SHA and the profiled forward")
    result.update(job=RUN, classification=LABEL, pilot_case=args.case, spec={k: v for k, v in spec.items() if not k.startswith("_")}, identity_sha256=ident["sha256"], strict_valid_handover=strict,
                  first_failure_stage=stage, first_failure_kind=None if strict else ("teacher_failure" if (fault or {}).get("reason") == "teacher_failure" else "fault" if fault is not None else "task not completed within the 40 s horizon"),
                  contact_first=log["first"], gate_trace=semantic, teacher=teacher_part, handover_event=result["handover_s"] is not None, completed_40s=result["termination"] == "horizon_completed",
                  block_contact=p1.contact_summary(rows, result), cuda=cuda, timing=timing,
                  child_settings=dict(rk.child_settings({k: v[0] for k, v in loaded.items() if not isinstance(v[0], TeacherSeat)}), host=s0.host(), workers=WORKERS))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(case=args.case, events={k: round(v, 2) for k, v in events.items()}, receive_entry=None if entry_step is None else round(entry_step * td.DT, 2), release_b=result["gate_s"],
                          termination=result["termination"], strict=strict, stage=stage, teacher=None if teacher_part is None else dict(ready=teacher_part["teacher_ready_valid"], failure=(teacher_part["failure"] or {}).get("detail", None)),
                          cuda=all(p["all_on_cuda"] for p in cuda["policies"].values()), kernels=(cuda.get("profiled_forward") or {}).get("cuda_kernel_events"), wall=timing["wall_s"], inference=timing["act_inference_s"]), default=str)[:900])


def batch(groups, name):
    """Groups run two at a time; the cases of one group run in order."""
    ident = identity()
    if load(run_dir() / "protocol.json")["identity"]["sha256"] != ident["sha256"]:
        raise RuntimeError("the identity differs from protocol.json (model, source, contract or host changed): nothing is run")
    (run_dir() / "logs").mkdir(exist_ok=True)
    attempts_file = run_dir() / "attempts.json"
    started = time.time()

    def worker(group):
        out = []
        for case in group:
            if complete(ident, case) is not None:
                continue
            folder = case_dir(ident, case)
            if folder.exists():
                kept = run_dir() / "attempts" / f"{folder.name}_{time.strftime('%Y%m%dT%H%M%S')}"
                kept.parent.mkdir(exist_ok=True)
                folder.rename(kept)
                out.append(dict(case=case, kept=str(kept), note="incomplete folder of an earlier attempt"))
            t0 = time.time()
            done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--case", case, "--identity", ident["sha256"]], capture_output=True, text=True)
            (run_dir() / "logs" / f"{folder.name}_{time.strftime('%H%M%S')}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
            ok = done.returncode == 0 and complete(ident, case) is not None
            if not ok:
                out.append(dict(case=case, returncode=done.returncode, status="infrastructure_error (not a task result)", stderr_tail=done.stderr[-900:]))
            line = [x for x in done.stdout.splitlines() if x.startswith("{")]
            print((line[-1][:700] if ok and line else f"[infrastructure_error] {case}: {done.stderr[-400:]}") + f"  [{time.time() - t0:.0f}s]", flush=True)
        return out

    extra = []
    with ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for x in pool.map(worker, groups):
            extra += x
    attempts = (load(attempts_file) if attempts_file.exists() else []) + extra
    attempts_file.write_text(json.dumps(attempts, indent=2))
    cases = [c for g in groups for c in g]
    done = [c for c in cases if complete(ident, c) is not None]
    status_file = run_dir() / "batch_status.json"
    status = load(status_file) if status_file.exists() else {}
    status[name] = dict(planned=len(cases), completed=len(done), missing=[c for c in cases if c not in done], workers=WORKERS, batch_wall_s=round(time.time() - started, 1),
                        sum_of_case_wall_s=round(sum(complete(ident, c)["timing"]["wall_s"] for c in done), 1), note="sum / batch is an overlap ratio, not a measured speed-up")
    status_file.write_text(json.dumps(status, indent=2))
    print(f"[{name}] {len(done)}/{len(cases)} cases; batch wall {status[name]['batch_wall_s']}s", flush=True)


def ag(args=None):
    batch([[f"ag:{c}" for c in g] for g in ag_cases()], "ag")


def teacher_dynamics(args=None):
    static = load(run_dir() / "teacher_local" / "static.json")
    valid = static["whole_preview_valid"]
    cases = [f"tl:{LOCAL_SEED}:{r}" for r in valid] + [f"tl:{s}:control" for s in CONTROL_ENTRY]
    half = (len(cases) + 1) // 2
    batch([cases[:half], cases[half:]] if len(cases) > 1 else [cases], "teacher_local")


def decide(args=None):
    target = run_dir() / "decision.json"
    if target.exists():
        return load(target)
    ident = identity()
    got = lambda c: complete(ident, c)
    runs = {c: got(f"ag:{c}") for g in ag_cases() for c in g}
    strict = lambda names: [bool(runs[n] and runs[n]["strict_valid_handover"]) for n in names]
    s4g, s4a = strict(["S4_G_1", "S4_G_2", "S4_G_3"]), strict(["S4_A_1", "S4_A_2", "S4_A_3"])
    guards = strict([c for g in ag_cases()[1:] for c in g])
    same_before = {}
    for config in AG:
        rs = [r for n, r in runs.items() if n.startswith(config) and r is not None and r["gate_trace"] is not None]
        same_before[config] = dict(runs_with_semantic_state=len(rs), identical=len({json.dumps(r["gate_trace"]["before"], sort_keys=True) for r in rs}) == 1 if rs else None,
                                   first_semantic_steps=sorted({r["gate_trace"]["first_semantic_step"] for r in rs}))
    confound = any(v["identical"] is False for v in same_before.values())
    use_g = all(s4g) and not all(s4a) and all(guards) and not confound and all(v is not None for v in runs.values())
    static = load(run_dir() / "teacher_local" / "static.json")
    tl = {r: got(f"tl:{LOCAL_SEED}:{r}") for r in static["whole_preview_valid"]}
    controls = {s: got(f"tl:{s}:control") for s in CONTROL_ENTRY}
    more_valid = len(static["whole_preview_valid"]) > 0
    dyn_valid = [r for r, x in tl.items() if x is not None and x["teacher"]["teacher_ready_valid"]]
    controls_ok = all(x is not None and x["teacher"]["teacher_ready_valid"] and x["strict_valid_handover"] for x in controls.values())
    use_derived = more_valid and bool(dyn_valid) and controls_ok
    data = dict(run_id=RUN, decided=time.strftime("%Y-%m-%dT%H:%M:%S%z"), contract="G" if use_g else "A", teacher="derived" if use_derived else "original",
                contract_basis=dict(S4_G_strict=s4g, S4_A_strict=s4a, guards_strict=guards, before_first_semantic=same_before, any_run_missing=any(v is None for v in runs.values())),
                teacher_basis=dict(candidates_valid_over_whole_preview=static["whole_preview_valid"], dynamics_with_valid_readiness=dyn_valid,
                                   controls={str(s): None if x is None else dict(teacher_ready_valid=x["teacher"]["teacher_ready_valid"], mixed_strict=x["strict_valid_handover"]) for s, x in controls.items()}),
                note="an experimental choice for this pilot's metadata only; no default changes; not a statistical or generalization claim")
    target.write_text(json.dumps(data, indent=2, default=str))
    print(json.dumps(dict(contract=data["contract"], teacher=data["teacher"], S4_G=s4g, S4_A=s4a, guards=guards, dynamics_valid=dyn_valid, controls=data["teacher_basis"]["controls"])))
    return data


def pilot(args=None):
    load(run_dir() / "decision.json")
    seeds = [s["seed"] for s in load(run_dir() / "protocol.json")["pilot"]["sources"]]
    cases = [f"pilot:{s}" for s in seeds]
    batch([cases[0::2], cases[1::2]], "pilot")


def everything(args=None):
    me = [sys.executable, str(Path(__file__).resolve())]
    for step in ("protocol", "ag", "teacher-static", "teacher-dynamics", "decide", "pilot"):
        print(f"--- {step} --- {time.strftime('%H:%M:%S')}", flush=True)
        if subprocess.run(me + [step]).returncode:
            raise SystemExit(f"step `{step}` failed; later steps are not started")
    report = Path(__file__).with_name("post_c0_pilot_r1_report.py")
    if report.exists():
        print(f"--- report --- {time.strftime('%H:%M:%S')}", flush=True)
        subprocess.run([sys.executable, str(report), "all"])
    print(f"=== ALL DONE {time.strftime('%H:%M:%S')} ===", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("protocol", "ag", "teacher-static", "teacher-dynamics", "decide", "pilot", "all"):
        sub.add_parser(name)
    o = sub.add_parser("one")
    o.add_argument("--case", required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    return {"protocol": protocol, "ag": ag, "teacher-static": teacher_static, "teacher-dynamics": teacher_dynamics, "decide": decide, "pilot": pilot, "one": one, "all": everything}[args.command](args)


if __name__ == "__main__":
    main()
