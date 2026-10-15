#!/usr/bin/env python3
"""CONSTRAINT-CAUSAL C0 / work6: no training. What the software arm-range handling, the C25 wait before RECEIVE and the
contact / IK rejections actually change, on fixed configurations (SIM only; no hardware, no new correction data).

Execution code (part of the run identity). Every variant below is a patch applied inside this tool's own child process;
the default reject / project behaviour, the models, the pinned MJCF and the checker are untouched.

  map        compiled limits, the command path and unit tests of the passthrough adapter -> constraint_map.json
  protocol   20 fixed student / diagnostic cases and the conditional teacher budget (at most 3)
  run        the 20 cases, two worker processes at a time (one configuration's cases stay in one worker)
  teacher    static audit of the 5008 / 5101 insert rejections and of the 5199 IK rejection; conditional continuations
  all        map, protocol, run, teacher

Conditions:  A = project + C25 switch (as now)        P  = raw passthrough to data.ctrl + C25 switch
             G = project + 50 Hz semantic switch      PG = both
             SHADOW = A, but a contact-category substep fault is recorded and the run goes on to 40 s (never a valid success)

Switches read SIM ground truth: SIM-ORACLE-GATED THREE-POLICY. Not added to any earlier run.
"""
import argparse
import hashlib
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
import receive_entry_r2 as r2   # noqa: E402
import receive_r3_recheck as rk   # noqa: E402
import start_pick_d1 as d1   # noqa: E402
import start_pick_r1 as p1   # noqa: E402
import receive_bridge_r1 as br   # noqa: E402
import server_s0 as s0   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "constraint-causal-c0-work6-20261006-01"
TITLE = "CONSTRAINT-CAUSAL C0 / work6 / command path, C25 wait and rejection audit"
LABEL = "SIM-ORACLE-GATED THREE-POLICY / CONSTRAINT-CAUSAL C0 work6"
WORKERS = 2
CONFIGS = dict(S1=(5000, "R1E", "CTRL"), S2=(5008, "R1E", "CTRL"), S3=(5098, "R1E", "CTRL"), S4=(5000, "FROZEN", "ADAPT"),
               R1=(5098, "R1E", "ADAPT"), R2=(5008, "FROZEN", "FROZEN"))
CONDITIONS = ("A", "P", "G", "PG")
GROUPS = [[f"{s}_{c}" for c in CONDITIONS] for s in ("S1", "S2", "S3", "S4")] + [["R1_A", "R2_A"], ["S3_SHADOW", "S4_SHADOW"]]
CASES = [c for g in GROUPS for c in g]
TEACHER_ENTRY = {5008: 400, 5101: 300, 5199: 825}
SOURCES = br.SOURCES + ("factory/v1/constraint_causal_c0.py", "source/production_source_g000/sim_data_factory/student_eval.py", "source/production_source_g000/sim_data_factory/core.py",
                        "source/production_source_g000/sim_data_factory/teacher.py", "source/production_source_g000/sim_data_factory/task_success.py",
                        "source/production_source_g000/dapier_act_policy.py")

load, sha, array_sha = rk.load, rk.sha, rk.array_sha


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def bridge_models():
    """Model paths exactly as the Bridge evaluation identity recorded them."""
    ident = load(br.run_dir() / "evaluation_identity.json")["models"]
    want = dict(START_FROZEN="19368b679b4a", START_R1_E="05c7593fa0f7", RECEIVE_FROZEN="69f8e02417fa", RECEIVE_BRIDGE_CTRL="c619fc513e8a", RECEIVE_BRIDGE_ADAPT="1ed13b67bb05", RELEASE_B="5a001b39fe27")
    out = {}
    for name, prefix in want.items():
        path = Path(ident[name]["path"])
        got = sha(path / "model.safetensors")
        if not got.startswith(prefix) or got != ident[name]["model"] or sha(path.parent / "normalization.npz") != rk.NORMALIZATION:
            raise RuntimeError(f"{name}: model / normalization differs from the Bridge identity")
        out[name] = dict(path=str(path), model=got, config=sha(path / "config.json"), normalization=rk.NORMALIZATION)
    return out


def parse(case):
    config, cond = case.split("_")
    seed, start, receive = CONFIGS[config]
    return dict(case=case, config=config, condition=cond, seed=seed, start=start, receive=receive, passthrough=cond in ("P", "PG"), gate50=cond in ("G", "PG"), shadow=cond == "SHADOW")


# ----------------------------------------------------------------------------------------------
# The C0 patches (this process only)
# ----------------------------------------------------------------------------------------------

def passthrough_decode(q, model, *, names=None, units=None):
    """core.decode_command without the arm-range rejection: order / units / shape / finite checks, the gripper range and
    the gripper unit conversion are the same; a finite arm target goes on as it is, inside or outside ctrlrange."""
    from sim_data_factory import core
    names, units = core.ACTION_NAMES if names is None else names, core.UNITS if units is None else units
    q = np.array(q, dtype=float, copy=True)
    if tuple(names) != core.ACTION_NAMES or tuple(units) != core.UNITS or q.shape != (12,) or not np.isfinite(q).all():
        raise ValueError("joint order/units/finite 12-vector required")
    if np.any(q[[5, 11]] < 0) or np.any(q[[5, 11]] > 1):
        raise ValueError("gripper opening must be in [0,1]")
    q[[5, 11]] = q[[5, 11]] * core.MOTOR_MAX_RAD
    g = [5, 11]
    if np.any(q[g] < model.actuator_ctrlrange[g, 0]) or np.any(q[g] > model.actuator_ctrlrange[g, 1]):
        raise ValueError("gripper command outside model actuator limits; no clipping")
    return q


def passthrough_project(command, model):
    """Stands where project_arm_command stood: the mask says what the projection WOULD have changed; the command is not changed."""
    from dapier_act_policy import ARM_AXES
    raw = np.asarray(command)
    if raw.shape != (12,) or not np.isfinite(raw).all():
        return command, None
    arm = list(ARM_AXES)
    q = raw.astype(float)
    mask = (q[arm] < model.actuator_ctrlrange[arm, 0]) | (q[arm] > model.actuator_ctrlrange[arm, 1])
    return command, dict(q_raw=q.tolist(), projection_mask=mask.tolist(), q_applied_minus_raw=[0.0] * 12)


def install(spec, log):
    """Apply the condition's patches. `log` collects contact records."""
    import mujoco
    from sim_data_factory import core, student_eval, task_success
    if spec["passthrough"]:
        core.decode_command = passthrough_decode
        student_eval.project_arm_command = passthrough_project
    if spec["gate50"]:
        original = rc.Chain.due_for_inference

        def due(self):                                                      # the START -> RECEIVE condition is looked at on every 50 Hz step, not only at C25 boundaries
            if not self.switches and not self.retired:
                return self.active.chunk is not None
            return original(self)

        rc.Chain.due_for_inference = due
    base = task_success.ContactModel.substep_failure

    def detail(cm, data):
        m, wrench, rows = cm.model, np.zeros(6), []
        for i, c in enumerate(data.contact):
            a, b = int(c.geom1), int(c.geom2)
            if cm.block_geom in (a, b) or not (cm.side[a] or cm.side[b]):
                continue
            mujoco.mj_contactForce(m, data, i, wrench)
            va, vb = np.zeros(6), np.zeros(6)
            mujoco.mj_objectVelocity(m, data, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[a]), va, 0)
            mujoco.mj_objectVelocity(m, data, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[b]), vb, 0)
            rows.append(dict(geoms=[m.geom(g).name or f"geom{g}" for g in (a, b)], bodies=[m.body(int(m.geom_bodyid[g])).name for g in (a, b)], dist_m=float(c.dist),
                             normal_force_N=float(max(0.0, wrench[0])), body_relative_speed_m_s=float(np.linalg.norm(va[3:] - vb[3:]))))
        return rows

    def substep(cm, data):
        fault = base(cm, data)
        if fault is None:
            return None
        rows = detail(cm, data)
        category = "numerical" if fault["warnings"] else ("forbidden_contact" if fault["forbidden_contacts"] else "penetration")
        if not log["faults"]:
            log["first"] = dict(fault, category=category, contacts=rows, block_contacts=[dict(geoms=[cm.model.geom(int(g)).name or f"geom{int(g)}" for g in (c.geom1, c.geom2)], dist_m=float(c.dist))
                                                                                         for c in data.contact if cm.block_geom in (int(c.geom1), int(c.geom2))])
        log["faults"] += 1
        for r in rows:
            p = log["pairs"].setdefault(" | ".join(r["geoms"]), dict(substeps=0, first_s=fault["time"], last_s=fault["time"], max_penetration_m=0.0, max_force_N=0.0, intervals=1))
            if fault["time"] - p["last_s"] > 0.0021:
                p["intervals"] += 1
            p["substeps"], p["last_s"] = p["substeps"] + 1, fault["time"]
            p["max_penetration_m"], p["max_force_N"] = max(p["max_penetration_m"], -r["dist_m"]), max(p["max_force_N"], r["normal_force_N"])
        if spec["shadow"] and category != "numerical":
            return None                                                     # recorded, not a reason to stop: observation goes on (never a valid success)
        return dict(fault, category=category, contacts=rows)

    task_success.ContactModel.substep_failure = substep


# ----------------------------------------------------------------------------------------------
# Constraint map and unit tests
# ----------------------------------------------------------------------------------------------

def constraint_map(args=None):
    import mujoco
    from integration_scenes import task_env
    from sim_data_factory import core, student_eval
    from sim_data_factory.teacher import TeacherConfig
    from sim_data_factory.task_success import CheckerConfig
    from dataclasses import asdict
    from dapier_act_policy import ARM_AXES
    target = run_dir() / "constraint_map.json"
    if target.exists():
        return load(target)
    run_dir().mkdir(parents=True, exist_ok=True)
    m = task_env("desk", grippers="both").model
    acts = []
    for i in range(m.nu):
        j = int(m.actuator_trnid[i, 0])
        acts.append(dict(name=m.actuator(i).name, ctrllimited=bool(m.actuator_ctrllimited[i]), ctrlrange=m.actuator_ctrlrange[i].tolist(), forcelimited=bool(m.actuator_forcelimited[i]),
                         forcerange=m.actuator_forcerange[i].tolist(), kp=float(m.actuator_gainprm[i, 0]), kd=float(-m.actuator_biasprm[i, 2]), gear=float(m.actuator_gear[i, 0]),
                         joint=m.joint(j).name, joint_limited=bool(m.jnt_limited[j]), joint_range=m.jnt_range[j].tolist(), joint_actfrclimited=bool(m.jnt_actfrclimited[j]),
                         ctrlrange_equals_joint_range=bool(np.array_equal(m.actuator_ctrlrange[i], m.jnt_range[j]))))
    clamp = not bool(m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL)
    # unit tests of the adapter on the compiled model
    q0 = core.encode_state(m, task_env("desk", grippers="both").data)
    inside = q0.copy()
    out = q0.copy()
    out[1] = m.actuator_ctrlrange[1, 1] + 0.3
    tests = {}
    a, ia = student_eval.project_arm_command(inside.copy(), m)
    b, ib = passthrough_project(inside.copy(), m)
    tests["in_range_same_ctrl_A_and_P"] = bool(np.array_equal(core.decode_command(a, m), passthrough_decode(b, m)))
    try:
        core.decode_command(out, m)
        tests["out_of_range_default_decoder"] = "accepted"
    except ValueError as e:
        tests["out_of_range_default_decoder"] = f"rejected: {e}"
    pa, _ = student_eval.project_arm_command(out.copy(), m)
    tests["out_of_range_project_then_decoder"] = dict(sent=float(core.decode_command(pa, m)[1]), bound=float(m.actuator_ctrlrange[1, 1]))
    pp, info = passthrough_project(out.copy(), m)
    raw_ctrl = passthrough_decode(pp, m)
    tests["out_of_range_passthrough_decoder"] = dict(sent=float(raw_ctrl[1]), bound=float(m.actuator_ctrlrange[1, 1]), mask_marks_axis=bool(info["projection_mask"][list(ARM_AXES).index(1)]))
    for name, bad in (("nan", np.r_[np.nan, q0[1:]]), ("gripper_above_1", np.r_[q0[:5], 1.2, q0[6:]]), ("eleven_axes", q0[:11])):
        try:
            passthrough_decode(bad, m)
            tests[f"passthrough_rejects_{name}"] = False
        except ValueError:
            tests[f"passthrough_rejects_{name}"] = True
    finals = []
    for ctrl in (core.decode_command(pa, m), raw_ctrl):                        # the same physics with the clamped and with the raw control
        d = mujoco.MjData(m)
        d.qpos[:] = task_env("desk", grippers="both").data.qpos
        mujoco.mj_forward(m, d)
        d.ctrl[:] = ctrl
        for _ in range(50):
            mujoco.mj_step(m, d)
        finals.append((d.qpos.copy(), d.actuator_force.copy(), d.ctrl.copy()))
    tests["engine"] = dict(data_ctrl_holds_raw_value=float(finals[1][2][1]), qpos_max_abs_diff_raw_vs_clamped_after_50_substeps=float(np.abs(finals[0][0] - finals[1][0]).max()),
                           actuator_force_max_abs_diff=float(np.abs(finals[0][1] - finals[1][1]).max()))
    effective = tests["engine"]["qpos_max_abs_diff_raw_vs_clamped_after_50_substeps"] > 0
    files = ("source/production_source_g000/sim_data_factory/student_eval.py", "source/production_source_g000/sim_data_factory/core.py", "source/production_source_g000/sim_data_factory/teacher.py",
             "source/production_source_g000/sim_data_factory/task_success.py", "source/production_source_g000/dapier_act_policy.py", "factory/v1/receive_boundary_repair.py",
             "factory/v1/receive_candidate.py", "factory/v1/handover_correction_trial.py", "factory/v1/receive_bridge_r1.py")
    data = dict(
        run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), sources={f: sha(FACTORY_ROOT / f) for f in files}, mjcf=dict(path=os.environ.get("DAPIER_SO101_MJCF"), sha256=sha(os.environ["DAPIER_SO101_MJCF"])),
        compiled=dict(timestep=float(m.opt.timestep), physics_steps_per_control=10, disableflags=int(m.opt.disableflags), native_ctrl_clamp_enabled=clamp, geom_margin_unique=np.unique(m.geom_margin).tolist(),
                      geom_gap_unique=np.unique(m.geom_gap).tolist(), actuators=acts),
        path=[
            dict(stage="policy raw chunk", where="dapier_act_policy.ACTPolicy.predict_canonical_chunk", constraint="bounded gripper head (sigmoid) -> opening fraction in [0, 1]; arm head unbounded", status="exists (gripper only)"),
            dict(stage="measured-relative anchor", where="predict_canonical_chunk with observation.measured_q; student_eval.Student.next_command", constraint="arm target = q_meas of the generation observation + predicted displacement; "
                 "one anchor for the 50 steps of the chunk", status="exists (representation, not a limit)"),
            dict(stage="chunk consumption", where="student_eval.Student (execution_prefix 25)", constraint="25 of 50 steps executed, then a new inference; no smoothing, no ensemble, no rate / step limit", status="no lead / speed / hold limit found in this path"),
            dict(stage="arm range handling", where="student_eval.project_arm_command (arm_bound_policy='project') or nothing ('reject')", unit="rad", value="model.actuator_ctrlrange, no margin",
                 constraint="project: an arm target outside ctrlrange becomes the bound; reject: left as it is for the decoder", status="exists; the executor uses 'project'"),
            dict(stage="decoder", where="core.decode_command (called by the Student and again by core.rollout)", unit="rad / opening fraction", value="model.actuator_ctrlrange; gripper [0, 1]",
                 constraint="a finite command outside ctrlrange raises ValueError('command outside model actuator limits; no clipping'): with 'reject' the run ends as student_action_rejected and the command never reaches data.ctrl",
                 status="exists: 'projection off' is a rejection, not a raw execution"),
            dict(stage="unit conversion", where="core.decode_command", constraint="gripper opening fraction x MOTOR_MAX_RAD -> gear rad; arm rad unchanged; order ACTION_NAMES", status="exists (conversion)"),
            dict(stage="data.ctrl", where="core.rollout: d.ctrl[:] = ctrl, then 10 x mj_step", constraint="none added", status="no wrapper found"),
            dict(stage="engine input clamp", where="MuJoCo mj_fwdActuation", unit="rad", value="actuator ctrlrange (ctrllimited = true on all 12; clamp flag enabled)",
                 constraint="the control is clamped to ctrlrange inside the engine; data.ctrl keeps the written value", status="exists; ctrlrange equals the joint range on every axis"),
            dict(stage="actuator force", where="MuJoCo position actuator", unit="N m", value="forcerange +-3.35 (arms), +-2.94 (grippers); kp 998.22, kd 2.731", constraint="force saturation", status="exists"),
            dict(stage="joint limit", where="MuJoCo joint range", unit="rad", value="equal to ctrlrange", constraint="limit constraint in the solver", status="exists"),
            dict(stage="contact termination", where="task_success.ContactModel.substep_failure via ctx['substep_check'] in core.rollout", unit="m", value="any contact (dist <= 0, geom margin 0) of a robot geom "
                 "with anything but the block; or penetration > 0.002 m; or an engine warning", constraint="the control step is cut and the run ends (physics_substep_fault)", status="exists"),
            dict(stage="teacher only", where="teacher.HandoverTeacher.next_command", constraint="np.clip(cmd, ctrlrange) on the teacher's own plan; MotionLimits 0.8 rad/s, 2.5 rad/s2, 15 rad/s3 on its segments", status="exists for the teacher, not for the student")],
        not_found=["a lead / step / speed / hold / envelope limit on student commands", "any hardware motor limit register in the SIM path (not assumed from the real robot)"],
        teacher_config=asdict(TeacherConfig()), checker_config=asdict(CheckerConfig()),
        unit_tests=tests,
        passthrough=dict(reaches_data_ctrl=tests["engine"]["data_ctrl_holds_raw_value"] > m.actuator_ctrlrange[1, 1], changes_dynamics=bool(effective),
                         verdict="software path changed, no effective treatment: the engine clamps the raw control to the same interval the projection uses" if not effective
                         else "the raw control changes the dynamics"),
        note="the pinned limits are the model's; whether they match the real robot is not checked here")
    target.write_text(json.dumps(data, indent=2, default=str))
    print(json.dumps(dict(native_clamp=clamp, tests=tests, verdict=data["passthrough"]["verdict"]), default=str))
    return data


# ----------------------------------------------------------------------------------------------
# Student / diagnostic cases
# ----------------------------------------------------------------------------------------------

def identity():
    import arm_output_trial as trial
    models = bridge_models()
    resets = {}
    for seed in sorted({v[0] for v in CONFIGS.values()}):
        meta = load(Path(d1.manifest()[p1.parents()["train"][seed]["manifest_index"]]["path"]) / "metadata.json")
        resets[str(seed)] = hashlib.sha256(json.dumps([meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"]], sort_keys=True).encode()).hexdigest()
    body = dict(run_id=RUN, models=models, manifest_sha256=p1.MANIFEST_SHA, cases=CASES, groups=GROUPS, configs={k: list(v) for k, v in CONFIGS.items()}, resets=resets, workers=WORKERS,
                sources={p: sha(FACTORY_ROOT / p) for p in SOURCES}, contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                execution=dict(chain="START -> RECEIVE -> RELEASE_B", chunk=50, prefix=rc.PREFIX, control_hz=round(1 / td.DT), horizon_steps=rc.HORIZON, torch_seed=4100, ensemble="off", smoothing="off",
                               A="arm_bound_policy project, C25 switch, substep fault ends the run", P="arm range neither projected nor rejected (this tool's adapter), C25 switch",
                               G="project; the unchanged START -> RECEIVE condition checked on every 50 Hz step", PG="P and G",
                               SHADOW="A; a contact-category substep fault is recorded and the run continues; a numerical fault still ends it"),
                backend=trial.environment_record(), host=s0.host(), environment={k: os.environ.get(k) for k in rk.ENV_KEYS})
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def protocol(args=None):
    target = run_dir() / "protocol.json"
    if target.exists():
        return load(target)
    run_dir().mkdir(parents=True, exist_ok=True)
    ident = identity()
    record = dict(run_id=RUN, name=TITLE, label=LABEL, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"), training=False, new_correction_data_admitted=False, hardware_access=False,
                  identity=ident, cases=[parse(c) for c in CASES], budget="20 student / diagnostic runs, once each, whatever the results; conditional teacher dynamics at most 3 "
                  "(5008 and 5101 from the first semantic state, 5199 with the rejected IK solution if only the residual acceptance rejected it)",
                  parallel=dict(workers=WORKERS, groups=GROUPS, note="one worker runs one group in order; two groups run at the same time; separate processes, no shared policy / RNG / EGL context; "
                                "a different identity from the earlier sequential runs"),
                  rule="a task fault is that case's result; no repeat until success; an infrastructure failure keeps its attempt and only that case is run again",
                  reading="20 runs for cause finding on fixed Train resets; not a generalization or success-rate estimate; configurations are never pooled",
                  strict="strict_valid_handover is the unchanged checker; a SHADOW run is never a valid success, whatever it shows after its first fault")
    target.write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(cases=len(CASES), identity=ident["sha256"][:12])))
    return record


def case_dir(ident, case):
    return run_dir() / "runs" / ident["sha256"][:12] / case


def complete(ident, case):
    out = case_dir(ident, case)
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = r.get("identity_sha256") == ident["sha256"] and r.get("c0_case") == case and meta.get("complete") and "child_settings" in r and (out / "episode.npz").exists()
    return r if ok else None


def one(args):
    from integration_scenes import task_env
    from sim_data_factory.task_success import evaluate_states
    ident = identity()
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    spec = parse(args.case)
    out = case_dir(ident, args.case)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    log = dict(faults=0, first=None, pairs={})
    install(spec, log)
    models = ident["models"]
    start = models["START_FROZEN" if spec["start"] == "FROZEN" else "START_R1_E"]
    receive = models[dict(FROZEN="RECEIVE_FROZEN", CTRL="RECEIVE_BRIDGE_CTRL", ADAPT="RECEIVE_BRIDGE_ADAPT")[spec["receive"]]]
    rc.MODELS["START"], rc.MODELS["RECEIVE"] = Path(start["path"]), Path(receive["path"])
    loaded, original = {}, rc.load_policy

    def remember(name):
        got = original(name)
        loaded[name] = got
        return got

    rc.load_policy = remember
    started = time.time()
    chain = f"START_{spec['start']} -> RECEIVE_{spec['receive']} -> RELEASE_B [{spec['condition']}]"
    try:
        result = rc.execute(out, spec["seed"], ("START", "RECEIVE", "B"), None, f"{LABEL}: {args.case} Train {spec['seed']} {chain}", contract="repaired")
    finally:
        rc.load_policy = original
    got = {k: v[2]["model_sha256"] for k, v in loaded.items()}
    if got.get("START") != start["model"] or got.get("RECEIVE", receive["model"]) != receive["model"] or got.get("B", models["RELEASE_B"]["model"]) != models["RELEASE_B"]["model"]:
        raise RuntimeError(f"a loaded policy is not the intended one: {got}")
    rows, trace = load(out / "task_check_rows.json"), load(out / "action_trace.json")
    with np.load(out / "episode.npz", allow_pickle=False) as a:
        states, q, ctrl = a["integration_state"].copy(), a["q_state"].copy(), a["ctrl_applied"].copy()
    model = task_env("desk", grippers="both").model
    lo, hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
    arm = list(rc.ARM)
    sent = [t for t in trace if t.get("ctrl_applied") is not None]
    raw = np.array([t["q_raw"] for t in sent])
    applied = np.array([t["ctrl_applied"] for t in sent])
    mask = np.array([t["projection_mask"] for t in sent], dtype=bool)
    outside = ((applied[:, arm] < lo[arm]) | (applied[:, arm] > hi[arm])).any(1)
    first = lambda v: None if not v.any() else dict(step=int(np.argmax(v)), sim_time_s=round(float(np.argmax(v)) * td.DT, 2), policy=sent[int(np.argmax(v))]["policy"])
    command_path = dict(dispatched_steps=len(sent), steps_raw_outside_arm_range=int(mask.any(1).sum()), first_raw_outside=first(mask.any(1)),
                        steps_data_ctrl_outside_arm_range=int(outside.sum()), first_data_ctrl_outside=first(outside),
                        max_raw_excess_rad=float((np.maximum(raw[:, arm] - hi[arm], 0) + np.maximum(lo[arm] - raw[:, arm], 0)).max()) if len(raw) else 0.0,
                        max_data_ctrl_excess_rad=float((np.maximum(applied[:, arm] - hi[arm], 0) + np.maximum(lo[arm] - applied[:, arm], 0)).max()) if len(applied) else 0.0,
                        engine_effective_ctrl_sha256=array_sha(np.clip(applied, lo, hi)), note="data.ctrl is what core.rollout wrote; the engine clamps it to ctrlrange internally")
    entry = next((s for s in result["switches"] if s["to"] == "RECEIVE"), None)
    gate = next((s for s in result["switches"] if s["to"] == "B"), None)
    events = result["events"]
    semantic = None                                                           # first 50 Hz state where the unchanged condition holds (pick_lift confirmed + left hand holds, no table support)
    if "pick_lift" in events:
        j = next((k for k, r in enumerate(rows) if r["time"] >= events["pick_lift"] - 1e-9 and rb.hands(r)[0]), None)
        if j is not None:
            a, b = br.handover_frame(model, states[j]), None if entry is None else br.handover_frame(model, states[entry["step"]])
            keep = lambda f: dict(block_in_left_site_mm=f["block_in_left_site_mm"], rot_deg=f["block_rot_angle_from_site_axes_deg"], clearance_mm=round(1000 * f["block_bottom_clearance_m"], 1))
            semantic = dict(first_semantic_step=j, first_semantic_s=round(j * td.DT, 2), switch_step=None if entry is None else entry["step"], switch_s=None if entry is None else entry["sim_time_s"],
                            wait_s=None if entry is None else round((entry["step"] - j) * td.DT, 2), at_first_semantic=keep(a), at_switch=None if b is None else keep(b),
                            left_arm_max_joint_change_rad=None if entry is None else round(float(np.abs(q[entry["step"], list(td.LEFT_ARM)] - q[j, list(td.LEFT_ARM)]).max()), 4))
    success, fault = result["task_success"], result["first_fault"]
    stage = None if success else "START" if entry is None else "RECEIVE" if gate is None else "RELEASE_B"
    strict = bool(success) and log["faults"] == 0
    shadow = None
    if spec["shadow"]:
        first_fault = log["first"]
        t0 = None if first_fault is None else first_fault["time"]
        fstage = None if t0 is None else "START" if entry is None or t0 < entry["sim_time_s"] else "RECEIVE" if gate is None or t0 < gate["sim_time_s"] else "RELEASE_B"
        last = rows[-1]
        g = 0.3
        shadow = dict(first_fault=first_fault, first_fault_stage=fstage, substeps_with_fault=log["faults"], pairs=log["pairs"], completed_40s=result["termination"] == "horizon_completed",
                      events_after_first_fault=None if t0 is None else {k: round(v, 2) for k, v in events.items() if v > t0},
                      post_fault_transfer_observed=bool(t0 is not None and events.get("handover") is not None and events["handover"] > t0),
                      post_fault_right_hand_only_at_end=bool(min(last["pad_force_N"]["right"]) >= g and max(last["pad_force_N"]["left"]) < 0.01 and last["table_support_N"] == 0.0),
                      checker_valid_over_50hz_samples=bool(result.get("prefix_checker_valid")), note="observation after a recorded fault; never a valid success under the existing rule")
        stage = fstage if not strict else None
    upto = len(states) if entry is None else entry["step"] + 1
    result.update(job=RUN, classification=LABEL, c0_case=args.case, spec=spec, identity_sha256=ident["sha256"], chain=chain, loaded_models=got, strict_valid_handover=strict,
                  first_failure_stage=stage, first_failure_kind=None if strict else ("fault" if (fault is not None or log["faults"]) else "task not completed within the 40 s horizon"),
                  contact_audit=dict(substeps_with_fault=log["faults"], first=log["first"], pairs=log["pairs"]), shadow=shadow, command_path=command_path, gate_trace=semantic,
                  handover_event=result["handover_s"] is not None, completed_40s=result["termination"] == "horizon_completed",
                  pre_receive=dict(states=int(upto), q_sha256=array_sha(q[:upto]), receive_entry_step=None if entry is None else entry["step"]),
                  block_contact=p1.contact_summary(rows, result), contract_checks=rb.contract_checks(result), phases=r2.phase_details(result, rows, result["receive_entry_s"] or 0.0),
                  counts=dict(sim_elapsed_s=result["last_completed_state_sim_s"], wall_s=round(time.time() - started, 1)),
                  child_settings=dict(rk.child_settings({k: v[0] for k, v in loaded.items()}), host=s0.host(), workers=WORKERS))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(case=args.case, events={k: round(v, 2) for k, v in events.items()}, receive_entry=result["receive_entry_s"], release_b=result["gate_s"], termination=result["termination"],
                          strict=strict, stage=stage, ctrl_outside=command_path["steps_data_ctrl_outside_arm_range"], raw_outside=command_path["steps_raw_outside_arm_range"],
                          shadow=None if shadow is None else dict(faults=shadow["substeps_with_fault"], post_fault_transfer=shadow["post_fault_transfer_observed"])), default=str))


def run(args=None):
    ident = identity()
    proto = load(run_dir() / "protocol.json")
    if proto["identity"]["sha256"] != ident["sha256"]:
        raise RuntimeError("the identity differs from protocol.json (model, source, contract or host changed): nothing is run")
    (run_dir() / "logs").mkdir(exist_ok=True)
    attempts_file = run_dir() / "attempts.json"
    attempts = load(attempts_file) if attempts_file.exists() else []
    started = time.time()

    def worker(group):
        out = []
        for case in group:
            if complete(ident, case) is not None:
                continue
            folder = case_dir(ident, case)
            if folder.exists():
                kept = run_dir() / "attempts" / f"{case}_{time.strftime('%Y%m%dT%H%M%S')}"
                kept.parent.mkdir(exist_ok=True)
                folder.rename(kept)
                out.append(dict(case=case, kept=str(kept), note="incomplete folder of an earlier attempt"))
            t0 = time.time()
            done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--case", case, "--identity", ident["sha256"]], capture_output=True, text=True)
            (run_dir() / "logs" / f"{ident['sha256'][:12]}_{case}_{time.strftime('%H%M%S')}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
            ok = done.returncode == 0 and complete(ident, case) is not None
            if not ok:
                out.append(dict(case=case, returncode=done.returncode, status="infrastructure_error (not a task result)", stderr_tail=done.stderr[-800:]))
            line = [x for x in done.stdout.splitlines() if x.startswith("{")]
            print((line[-1][:500] if ok and line else f"[infrastructure_error] {case}: {done.stderr[-300:]}") + f"  wall {time.time() - t0:.0f}s", flush=True)
        return out

    with ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for extra in pool.map(worker, GROUPS):
            attempts += extra
    attempts_file.write_text(json.dumps(attempts, indent=2))
    wall = round(time.time() - started, 1)
    done = [c for c in CASES if complete(ident, c) is not None]
    (run_dir() / "run_status.json").write_text(json.dumps(dict(completed=len(done), planned=len(CASES), missing=[c for c in CASES if c not in done], workers=WORKERS, wall_s_this_invocation=wall,
                                                                sum_of_case_wall_s=round(sum(complete(ident, c)["counts"]["wall_s"] for c in done), 1)), indent=2))
    print(f"[run] {len(done)}/{len(CASES)} cases; wall {wall}s with {WORKERS} workers")


# ----------------------------------------------------------------------------------------------
# Teacher rejection audit and conditional continuations
# ----------------------------------------------------------------------------------------------

def r1_case(seed):
    return br.source_case(seed)


def restored_teacher(seed, state):
    import mujoco
    import frozen_transfer_check as ft
    import handover_correction_trial as hc
    from sim_data_factory.reset_sampler import make_task_episode
    meta = ft.demonstration(seed)["metadata"]
    ctx = make_task_episode(seed, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
    m, d = ctx["model"], ctx["data"]
    mujoco.mj_setState(m, d, state, int(mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_forward(m, d)
    return ctx, hc.reentry_teacher(m)


def insert_audit(seed, step, label):
    """Static: the unchanged planner on the restored state, with its path checks recorded and re-evaluated at other margins.
    Nothing is stepped; no decision of the planner is changed."""
    g = rc.stored_state(r1_case(seed), step)
    ctx, teacher = restored_teacher(seed, g["states"][-1])
    m, d, plan = ctx["model"], ctx["data"], teacher.plan
    calls, tried_ref = [], []
    orig_path, orig_select = plan.path_collisions, teacher._select_handover

    def path(base_qpos, start, goal, carry=None, samples=24, allow_pad_block=(), block_qpos=None):
        hit = orig_path(base_qpos, start, goal, carry=carry, samples=samples, allow_pad_block=allow_pad_block, block_qpos=block_qpos)
        rec = dict(index=len(calls), samples=samples, allow_pad_block=list(allow_pad_block), margin_m=plan.gripper_clearance_m, hit=None)
        if hit is not None:
            saved = plan.gripper_clearance_m
            alt = {}
            for name, margin in (("nominal_contact_only", 0.0), ("live_margin_3mm", 0.003)):
                plan.gripper_clearance_m = margin
                h = orig_path(base_qpos, start, goal, carry=carry, samples=samples, allow_pad_block=allow_pad_block, block_qpos=block_qpos)
                alt[name] = None if h is None else dict(u=h[0], bad=[list(map(str, b)) for b in h[1][:4]])
            plan.gripper_clearance_m = saved
            dist = []
            for u in np.linspace(0, 1, samples):
                c = start + (goal - start) * (35 * u**4 - 84 * u**5 + 70 * u**6 - 20 * u**7)
                dist.append(plan.gripper_distance(base_qpos, c, plan.carried_block(base_qpos, c, carry) if carry is not None else block_qpos))
            rec.update(hit=dict(u=hit[0], sample_index=int(round(hit[0] * (samples - 1))), bad=[list(map(str, b)) for b in hit[1][:6]], kind="clearance margin" if any(len(b) == 3 and b[2] == "clearance" for b in hit[1]) else "contact"),
                       at_other_margins=alt, min_gripper_gripper_distance_mm_on_path=round(1000 * min(dist), 2))
        calls.append(rec)
        return hit

    def select(d_, rel, base, carry, tried):
        tried_ref.append(tried)
        return orig_select(d_, rel, base, carry, tried)

    plan.path_collisions, teacher._select_handover = path, select
    rel = teacher.site_block_rel(d, "left")
    block_pred = plan.carried_block(d.qpos.copy(), teacher.measured_gear(d), ("left", rel))
    actual = d.qpos[teacher.cm.block_qpos:teacher.cm.block_qpos + 7].copy()
    teacher.next_command(ctx)
    tried = tried_ref[0] if tried_ref else []
    kinds = {}
    for t in tried:
        kinds[t[0]] = kinds.get(t[0], 0) + 1
    hits = [c for c in calls if c["hit"] is not None and c["samples"] == 24]
    pairs = {}
    for c in hits:
        for b in c["hit"]["bad"]:
            key = " | ".join(b)
            pairs[key] = pairs.get(key, 0) + 1
    frame = br.handover_frame(m, g["states"][-1])
    return dict(label=label, seed=seed, step=step, sim_time_s=round(step * td.DT, 2), state_sha256=array_sha(g["states"][-1]), teacher_failure=teacher.failure, reentry=teacher.reentry,
                function="teacher.HandoverTeacher._select_handover -> Planner.path_collisions -> Planner.collisions", predicate="first: geom_distance(left gripper geom, right gripper geom) < gripper_clearance_m "
                "for the 3 x 3 housing / pad pairs (returns only those); then any contact with dist <= 0 except block-table and block-pad of the allowed side",
                margin=dict(plan_clearance_m=teacher.cfg.plan_clearance_m, live_clearance_m=teacher.cfg.live_clearance_m, unit="m", note="geom_distance is a vertex-to-OBB approximation (planner heuristic), not an exact surface distance"),
                scene="the planner poses a separate MjData of the same compiled model (same geometry and masks); the block follows the left tool frame by the measured hand->block transform",
                carried_block_check=dict(position_error_mm=round(1000 * float(np.linalg.norm(block_pred[:3] - actual[:3])), 4), quaternion_abs_dot=round(float(abs(np.dot(block_pred[3:], actual[3:]))), 6)),
                block_in_left_site_mm=frame["block_in_left_site_mm"], block_clearance_mm=round(1000 * frame["block_bottom_clearance_m"], 1),
                candidates_tried=len(tried), tried_by_reason=kinds, handover_clearance_top_mm=teacher.info.get("handover_clearance_top_mm"), path_checks=len(calls), path_checks_rejected=len(hits),
                rejected_by_kind={k: sum(c["hit"]["kind"] == k for c in hits) for k in ("clearance margin", "contact")}, rejecting_pairs=dict(sorted(pairs.items(), key=lambda kv: -kv[1])[:12]),
                would_pass_with_nominal_contact_only=sum(c["at_other_margins"]["nominal_contact_only"] is None for c in hits),
                would_pass_with_live_margin=sum(c["at_other_margins"]["live_margin_3mm"] is None for c in hits),
                min_gripper_distance_mm=dict(min=min((c["min_gripper_gripper_distance_mm_on_path"] for c in hits), default=None), max=max((c["min_gripper_gripper_distance_mm_on_path"] for c in hits), default=None)),
                first_rejections=hits[:6], note="static check on stored states; a limited candidate list failing is not a proof that no path exists")


def first_semantic_step(seed):
    case = r1_case(seed)
    result, rows = load(case / "result.json"), load(case / "task_check_rows.json")
    lift = result["events"]["pick_lift"]
    return next(k for k, r in enumerate(rows) if r["time"] >= lift - 1e-9 and rb.hands(r)[0])


def teacher_run(name, seed, step, note, shadow_ik=False):
    """One teacher run from a stored state of the R1 E run (as RECEIVE-BRIDGE R1 did), then frozen RELEASE_B from the first readiness."""
    import handover_correction_trial as hc
    import paired_release_check as pr
    from integration_scenes import task_env
    from sim_data_factory import student_eval, teacher as teacher_module
    from sim_data_factory.task_success import evaluate_states
    target = run_dir() / "teacher" / f"{name}.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; this teacher attempt is spent")
    target.parent.mkdir(parents=True, exist_ok=True)
    case = r1_case(seed)
    g = rc.stored_state(case, step)
    entry = dict(id=name, seed=seed, source_case=str(case), source_step=step, source_sim_time_s=float(g["states"][-1][0]), state_sha256=array_sha(g["states"][-1]), note=note,
                 serializer_check=br.replay_check(seed, case, step), admitted_to_training=False)
    ik_audit = dict(used=False, calls=[])
    originals = (teacher_module.Planner.solve, teacher_module.HandoverTeacher._plan_right_grasp)
    if shadow_ik:
        flag = dict(pregrasp=False)

        def plan_right_grasp(self, d):
            flag["pregrasp"] = True
            try:
                return originals[1](self, d)
            finally:
                flag["pregrasp"] = False

        def solve(self, base_qpos, seed_q, side, pos, approach, jaw_candidates, cfg):
            ctrl, info = originals[0](self, base_qpos, seed_q, side, pos, approach, jaw_candidates, cfg)
            first_call, flag["pregrasp"] = flag["pregrasp"], False              # only the first solve of _plan_right_grasp is the pregrasp IK
            if ctrl is not None or not first_call or ik_audit["used"]:
                return ctrl, info
            m = self.m
            seeds = [np.array(seed_q, float)]
            for arm in self.SEED_ARMS:
                s = np.array(seed_q, float)
                o = teacher_module.SIDE_OFFSET[side]
                s[o:o + 5] = arm
                s[o] = seed_q[o]
                seeds.append(s)
            rows = []
            for s in seeds:                                                 # the planner's own steps, without its acceptance line
                c = np.array(s, float)
                roll_ok = True
                for _ in range(3):
                    r = teacher_module.solve_bimanual_position_ik(m, c, {side: pos}, site_names={side: f"{side}_cube_grasp"}, tool_axis_targets={side: approach}, max_iterations=300,
                                                                  tool_axis_tolerance_rad=np.deg2rad(2.0), axis_formulation="axis_direction")
                    c = np.array(r.action_rad)
                    rolled = self.set_roll(base_qpos, c, side, jaw_candidates)
                    if rolled is None:
                        roll_ok = False
                        break
                    c = rolled
                if not roll_ok:
                    rows.append(dict(result="roll_range"))
                    continue
                self.pose(base_qpos, c)
                p, rot = self.site_frame(side)
                perr = float(np.linalg.norm(p - pos))
                aerr = float(np.degrees(np.arccos(np.clip(np.dot(rot[:, 0], approach / np.linalg.norm(approach)), -1, 1))))
                o = teacher_module.SIDE_OFFSET[side]
                lo, hi = m.actuator_ctrlrange[o:o + 5, 0], m.actuator_ctrlrange[o:o + 5, 1]
                rows.append(dict(result="finite", pos_err_m=perr, axis_err_deg=aerr, pos_ok=perr <= cfg.ik_pos_tol_m, axis_ok=aerr <= cfg.ik_axis_tol_deg, ctrl=c.tolist(),
                                 inside_joint_range=bool(np.all(c[o:o + 5] >= lo) and np.all(c[o:o + 5] <= hi)), joint_margin_min_rad=float(np.minimum(c[o:o + 5] - lo, hi - c[o:o + 5]).min())))
            first = rows[0]
            only_residual = first["result"] == "finite" and first["inside_joint_range"] and np.isfinite(first["pos_err_m"]) and not (first["pos_ok"] and first["axis_ok"])
            ik_audit.update(time_s=None, side=side, target_pos=np.asarray(pos).tolist(), approach=np.asarray(approach).tolist(), reported=str(info), thresholds=dict(ik_pos_tol_m=cfg.ik_pos_tol_m, ik_axis_tol_deg=cfg.ik_axis_tol_deg,
                            rule="rejected if pos_err > ik_pos_tol_m OR axis_err > ik_axis_tol_deg (teacher.Planner._solve_one); the solver itself aims for 2 deg"), seeds=rows,
                            rejected_only_by_residual_acceptance=bool(only_residual), which_residual=None if first["result"] != "finite" else dict(position=not first["pos_ok"], axis=not first["axis_ok"]),
                            note="the message the teacher reports is the first seed's; the path-collision check of the unchanged code follows and may still refuse this pose")
            if not only_residual:
                return ctrl, info
            ik_audit["used"] = True
            return np.array(first["ctrl"]), dict(pos_err_m=first["pos_err_m"], axis_err_deg=first["axis_err_deg"], shadow="residual-rejected first-seed solution, SIM shadow only")

        teacher_module.Planner.solve, teacher_module.HandoverTeacher._plan_right_grasp = solve, plan_right_grasp
    original, limit = student_eval.Observations, hc.TEACHER_MAX_STEPS
    sensors = []

    def first_stored(*a, **k):
        student_eval.Observations = original
        sensors.append(pr.stored_first_sensor(dict(rgb=g["rgb"])))
        return sensors[-1]

    student_eval.Observations, hc.TEACHER_MAX_STEPS = first_stored, rc.HORIZON - step
    try:
        got = hc.continuation(run_dir() / "teacher" / name / "correction", seed, g["states"], g["q"], g["states"][-1],
                              dict(classification="c0_conditional_teacher_continuation", source_student_case=str(case), source_step=step, source_state_sha256=entry["state_sha256"], note=note))
    finally:
        student_eval.Observations, hc.TEACHER_MAX_STEPS = original, limit
        teacher_module.Planner.solve, teacher_module.HandoverTeacher._plan_right_grasp = originals
    entry.update(got)
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
        receive.update(right_first_pad_touch_s=next((round(r["time"], 2) for r in rows if max(r["pad_force_N"]["right"]) > 0), None), teacher_stop=got["stop"], teacher_transitions=got["transitions"])
        if why:
            fault = got.get("teacher_failure") or got.get("terminal_failure")
            receive.update(failure_class="execution failure" if fault else "readiness not reached within the absolute 40 s", reason=f"{why}; stop={got['stop']}; {fault}")
        else:
            m = task_env("desk", grippers="both").model
            hist, hist_q = np.concatenate([g["states"], suffix[1:end + 1]]), np.concatenate([g["q"], suffix_q[1:end + 1]])
            check = evaluate_states(m, hist, hist_q, control_dt=td.DT)
            receive.update(teacher_ready_valid=bool(check["valid"]), receive_abs_s=[round(step * td.DT, 2), round(rows[end]["time"], 2)], prefix_failures=check["failures"][:3],
                           events_up_to_readiness={k: round(v, 2) for k, v in check["events"].items()})
            rgb = {s: np.load(path / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}
            if check["valid"] and end < len(rgb["left"]):
                out = run_dir() / "teacher" / name / "plus_b"
                result = rc.execute(out, seed, ("B",), dict(step=step + end, states=hist, q=hist_q, rgb={s: rgb[s][end] for s in rgb}), f"{LABEL}: {name}, teacher RECEIVE then frozen RELEASE_B", contract="repaired")
                mixed = dict(teacher_receive_plus_frozen_B_valid=bool(result["task_success"]), path=str(out), handover_s=result["handover_s"], termination=result["termination"],
                             final_support=result["final_support"], first_fault=result["first_fault"])
            elif not check["valid"]:
                receive.update(failure_class="execution failure", reason="the history up to readiness is not checker-valid")
    entry["receiving"], entry["mixed"], entry["ik_shadow"] = receive, mixed, ik_audit if shadow_ik else None
    target.write_text(json.dumps(entry, indent=2, default=str))
    print(json.dumps(dict(id=name, restore_identity=got.get("restore_identity"), reentry=got.get("reentry"), stop=got.get("stop"), receiving={k: receive.get(k) for k in ("teacher_ready_valid", "receive_abs_s", "failure_class", "reason")},
                          mixed={k: mixed.get(k) for k in ("teacher_receive_plus_frozen_B_valid", "handover_s", "termination", "reason")},
                          ik=None if not shadow_ik else {k: ik_audit.get(k) for k in ("used", "reported", "rejected_only_by_residual_acceptance", "which_residual")}), default=str)[:2200])
    return entry


def teacher_step(args):
    kind, seed = args.kind, args.seed
    out = run_dir() / "teacher"
    out.mkdir(parents=True, exist_ok=True)
    if kind == "audit":
        target = out / f"insert_audit_s{seed}.json"
        if target.exists():
            return
        entry_step, first = TEACHER_ENTRY[seed], first_semantic_step(seed)
        data = dict(seed=seed, at_entry=insert_audit(seed, entry_step, "stored RECEIVE entry (C25 boundary)"), at_first_semantic=insert_audit(seed, first, "first 50 Hz state where the semantic condition holds"),
                    dynamics_steps=0)
        target.write_text(json.dumps(data, indent=2, default=str))
        print(json.dumps({k: {x: data[k][x] for x in ("step", "teacher_failure", "candidates_tried", "tried_by_reason", "rejected_by_kind", "would_pass_with_nominal_contact_only", "would_pass_with_live_margin", "min_gripper_distance_mm",
                                                      "block_in_left_site_mm", "block_clearance_mm", "carried_block_check")} for k in ("at_entry", "at_first_semantic")}, default=str)[:3000])
    elif kind == "gate":
        step = first_semantic_step(seed)
        teacher_run(f"s{seed}_first_semantic_{step:04d}", seed, step, "the first 50 Hz state of the R1 E run where pick_lift is confirmed and the left hand holds; unchanged teacher; not a student result, not training data")
    elif kind == "ik":
        teacher_run(f"s{seed}_ik_shadow_{TEACHER_ENTRY[seed]:04d}", seed, TEACHER_ENTRY[seed], "the stored RECEIVE entry; the teacher is replayed to its pregrasp IK; if only the residual acceptance rejects the "
                    "first-seed solution, that same solution is used once (SIM shadow); not a success under the original rule, not training data", shadow_ik=True)


def teacher(args=None):
    """Static audits first (two at a time), then the conditional dynamics: 5008 and 5101 from the first semantic state, 5199 IK shadow."""
    (run_dir() / "logs").mkdir(parents=True, exist_ok=True)
    me = [sys.executable, str(Path(__file__).resolve()), "teacher-step"]

    def go(job):
        kind, seed = job
        name = f"teacher_{kind}_s{seed}"
        marker = dict(audit=run_dir() / "teacher" / f"insert_audit_s{seed}.json")
        if kind == "audit" and marker["audit"].exists():
            return dict(job=name, status="stored")
        if kind != "audit" and list((run_dir() / "teacher").glob(f"s{seed}_{'first_semantic' if kind == 'gate' else 'ik_shadow'}_*")):
            return dict(job=name, status="stored or spent (a folder / result of this attempt exists)")
        done = subprocess.run(me + ["--kind", kind, "--seed", str(seed)], capture_output=True, text=True)
        (run_dir() / "logs" / f"{name}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(f"[{name}] " + (line[-1][:1500] if line and done.returncode == 0 else f"returncode {done.returncode}: {done.stderr[-500:]}"), flush=True)
        return dict(job=name, returncode=done.returncode, **({} if done.returncode == 0 else dict(stderr_tail=done.stderr[-800:])))

    status = []
    with ThreadPoolExecutor(max_workers=WORKERS) as pool:
        status += list(pool.map(go, [("audit", 5008), ("audit", 5101)]))
        status += list(pool.map(go, [("gate", 5008), ("gate", 5101), ("ik", 5199)]))
    (run_dir() / "teacher_status.json").write_text(json.dumps(status, indent=2))


def everything(args=None):
    me = [sys.executable, str(Path(__file__).resolve())]
    for step in (["map"], ["protocol"], ["run"], ["teacher"]):
        print(f"--- {' '.join(step)} ---", flush=True)
        if subprocess.run(me + step).returncode:
            raise SystemExit(f"step `{' '.join(step)}` failed; later steps are not started")
    report = Path(__file__).with_name("constraint_causal_c0_report.py")
    if report.exists():
        subprocess.run([sys.executable, str(report), "all"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("map", "protocol", "run", "teacher", "all"):
        sub.add_parser(name)
    o = sub.add_parser("one")
    o.add_argument("--case", required=True, choices=CASES)
    o.add_argument("--identity", required=True)
    t = sub.add_parser("teacher-step")
    t.add_argument("--kind", required=True, choices=("audit", "gate", "ik"))
    t.add_argument("--seed", type=int, required=True, choices=sorted(TEACHER_ENTRY))
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    return {"map": constraint_map, "protocol": protocol, "run": run, "one": one, "teacher": teacher, "teacher-step": teacher_step, "all": everything}[args.command](args)


if __name__ == "__main__":
    main()
