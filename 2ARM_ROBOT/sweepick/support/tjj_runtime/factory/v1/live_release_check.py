#!/usr/bin/env python3
"""SIM-ORACLE-GATED POLICY COMPOSITION / ORIGINAL RESET (Train seed 5000; local SIM only, no training, no hardware).

START controls from the original reset. At the first regular C25 inference boundary where the existing checker's
right_grasp holds for the run so far AND both hands currently hold the block with both pads (checker grip level), the
active policy is replaced once by START (CONTROL), A (REPLAY) or B (RECOVERY). Nothing in the world is restored or moved:
the same MjData keeps stepping; the successor infers from the RGB and q_state of that moment with a fresh chunk and anchor.
The gate reads SIM ground truth (contact forces, block state), so this is not a privileged-information-free student run.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import train_rollout_diagnostic as td   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "live-release-check-20261004-01"
MODELS = dict(START=FACTORY_ROOT / "training/three-parent-c25-20261004-01/step-020000",
              A=FACTORY_ROOT / "training/recovery-subset-ab-20261004-01-a/step-005000",
              B=FACTORY_ROOT / "training/recovery-subset-ab-20261004-01-b/step-005000")
CONDITIONS = (("CONTROL", "START"), ("REPLAY", "A"), ("RECOVERY", "B"))
SEED, HORIZON, PREFIX = 5000, 2000, 25
ARM = list(td.LEFT_ARM + td.RIGHT_ARM)
TRAINED_STARTS = ("s5000_project_1178", "s5000_project_1678")
SELECTION = FACTORY_ROOT / "evaluation/handover-correction-20261004-01/selection.json"
GATE = dict(source="SIM_GT", predicate="task_success.evaluate_states(run so far).events has 'right_grasp' AND ContactModel.evidence(now): "
            "min pad force of the left hand and of the right hand >= CheckerConfig.grip_force_min_N, table_support_N == 0",
            when="checked only where the active Student is due for a regular new inference (chunk_index == 25); first boundary that passes; once",
            thresholds="the checker's own (grip_force_min_N 0.3 N, grip_confirm_s 0.2 s); none added")


def load(path):
    return json.loads(Path(path).read_text())


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


class Composed:
    """One active Student at a time; a single replacement, no carry-over of the old chunk."""

    def __init__(self, first):
        self.active, self.retired, self.switch = first, None, None

    @property
    def failure(self):
        return self.active.failure

    def next_command(self, context):
        return self.active.next_command(context)

    def due_for_inference(self):
        return self.active.chunk is not None and self.active.chunk_index == self.active.execution_prefix

    def replace(self, successor, info):
        self.active.discard_chunk()
        self.retired, self.active, self.switch = self.active, successor, info

    def discard_chunk(self):
        self.active.discard_chunk()

    @property
    def trace(self):
        return (self.retired.trace if self.retired else []) + self.active.trace


def frames(model, data, cm):
    """Block position in each hand's tool frame, pad forces, gripper openings (analysis only; never a policy input)."""
    from sim_data_factory import core
    out = {}
    for side in ("left", "right"):
        s = model.site(f"{side}_cube_grasp").id
        rot = data.site_xmat[s].reshape(3, 3)
        out[f"block_in_{side}_site_mm"] = (rot.T @ (data.xpos[cm.block_body] - data.site_xpos[s]) * 1000).round(2).tolist()
    ev = cm.evidence(data)
    q = core.encode_state(model, data)
    out.update(pad_force_N=ev["pad_force_N"], block_bottom_clearance_m=ev["block_bottom_clearance_m"], table_support_N=ev["table_support_N"],
               gripper_opening=dict(left=float(q[td.LEFT_GRIPPER]), right=float(q[td.RIGHT_GRIPPER])), q_state=q.tolist())
    return out


def load_policy(name):
    import torch  # noqa: F401
    from dapier_act_policy import ACTPolicy
    from sim_data_factory.r2_data import sha
    ckpt = MODELS[name]
    policy = ACTPolicy.from_pretrained(ckpt, local_files_only=True, strict=True).cuda().eval()
    with np.load(ckpt.parent / "normalization.npz", allow_pickle=False) as a:
        stats = {k: {s: a[k + "_" + s].copy() for s in ("mean", "std")} for k in ("state", "action")}
    policy.check_normalization(stats)
    identity = dict(checkpoint=str(ckpt), model_sha256=sha(ckpt / "model.safetensors"), config_sha256=sha(ckpt / "config.json"),
                    normalization_sha256=sha(ckpt.parent / "normalization.npz"))
    return policy, stats, identity


def one(label, successor, out, switch_enabled=True):
    import mujoco
    import torch
    import arm_output_trial as trial
    import arm_projection_trial as ap
    import frozen_transfer_check as ft
    from sim_data_factory import core
    from sim_data_factory.recorder import EpisodeRecorder
    from sim_data_factory.reset_sampler import make_task_episode
    from sim_data_factory.student_eval import Observations, Student, include_terminal
    from sim_data_factory.task_success import CheckerConfig, ContactModel, evaluate_states
    trial.apply_reproducibility_settings()
    torch.set_num_threads(4)
    torch.manual_seed(4100)
    demo = ft.demonstration(SEED)
    meta = demo["metadata"]
    ctx = make_task_episode(SEED, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
    m, d = ctx["model"], ctx["data"]
    cm = ContactModel(m)
    ctx["substep_check"] = cm.substep_failure
    cfg = CheckerConfig()
    p_start, s_start, id_start = load_policy("START")
    p_next, s_next, id_next = (p_start, s_start, id_start) if successor == "START" else load_policy(successor)
    p_start.reset()
    sensor = Observations(width=160, height=120, camera_ids=("left_wrist_rgb", "right_wrist_rgb"), depth_camera_ids=())
    actor = Composed(Student(p_start, s_start, sensor, PREFIX, None, "project"))
    rec = EpisodeRecorder(out, dict(
        contract_revision=core.REVISION, seed=SEED, control_dt=ctx["control_dt"], classification="SIM-ORACLE-GATED POLICY COMPOSITION / ORIGINAL RESET",
        hardware_access=False, object_attachment=False, external_forces=False, runtime_qpos_writes=0, teacher_behavior_calls=0, condition=label,
        policies=dict(before_switch=id_start, after_switch=id_next, successor=successor), gate=GATE, reset=ctx["reset"],
        execution=dict(prefix=PREFIX, arm_bound_policy="project", ensemble="off", horizon_transitions=HORIZON),
        policy_inputs=["current measured canonical q_state", "raw left/right wrist RGB"], compiled_model_sha256=ctx["compiled_model_sha256"]))
    spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    gate_checks = []
    started = time.time()
    try:
        for step in range(HORIZON):
            if switch_enabled and actor.switch is None and actor.due_for_inference():
                ev = cm.evidence(d)
                now = (min(ev["pad_force_N"]["left"]) >= cfg.grip_force_min_N and min(ev["pad_force_N"]["right"]) >= cfg.grip_force_min_N
                       and ev["table_support_N"] == 0.0)
                history = None
                if now:
                    hist = evaluate_states(m, np.stack([s["integration_state"] for s in rec.states] + [core.snapshot(ctx)["integration_state"]]),
                                           control_dt=ctx["control_dt"])
                    history = hist["events"].get("right_grasp")
                gate_checks.append(dict(step=step, time=round(float(d.time), 2), both_hands_now=bool(now), right_grasp_event=history))
                if now and history is not None:
                    before = np.empty(mujoco.mj_stateSize(m, spec))
                    mujoco.mj_getState(m, d, before, spec)
                    last = actor.active.trace[-1]
                    info = dict(step=step, sim_time_s=round(float(d.time), 2), right_grasp_event_s=history, successor=successor, state_before=frames(m, d, cm),
                                last_command_before=dict(q_raw=last["q_raw"], q_applied=last["q_cmd"], q_measured=last["q_measured"]))
                    p_next.reset()
                    actor.replace(Student(p_next, s_next, sensor, PREFIX, None, "project"), info)
                    after = np.empty_like(before)
                    mujoco.mj_getState(m, d, after, spec)
                    info["world_state_unchanged_by_switch"] = bool(np.array_equal(before, after))
            before_time, before_count = float(d.time), len(rec.commands)
            progressed = core.rollout(ctx, actor, sensor, rec, 1)
            if actor.active.trace:
                row = actor.active.trace[-1]
                row.update(ctrl_applied=d.ctrl.tolist() if d.time > before_time else None, complete_transition=len(rec.commands) > before_count)
                if actor.switch is not None and "first_command_after" not in actor.switch and actor.switch["step"] == step:
                    prev = np.array(actor.switch["last_command_before"]["q_applied"])
                    actor.switch["first_command_after"] = dict(q_raw=row["q_raw"], q_applied=row["q_cmd"], anchor_q_measured=row["q_measured"],
                                                               chunk_index=row["selected_chunk_index"], chunk_generation_id=row["chunk_generation_id"],
                                                               anchor_equals_measured=bool(np.array_equal(actor.active.chunks[0]["anchor"], np.asarray(row["q_measured"], dtype=np.float32))),
                                                               target_change_from_previous=(np.array(row["q_cmd"]) - prev).round(5).tolist(),
                                                               target_change_arm_max_rad=float(np.abs((np.array(row["q_cmd"]) - prev)[ARM]).max()),
                                                               decoded=row["ctrl_validated"] is not None)
            if not progressed:
                actor.discard_chunk()
                break
    finally:
        sensor.close()
    fault = ctx.get("terminal_failure")
    states = np.stack([s["integration_state"] for s in rec.states])
    q = np.stack([s["q_state"] for s in rec.states])
    check = evaluate_states(m, states, q, control_dt=ctx["control_dt"])
    rows = check.pop("rows")
    prefix_valid = check["valid"]
    check = include_terminal(check, fault)
    switch = actor.switch
    t_sw = switch["sim_time_s"] if switch else None
    g = cfg.grip_force_min_N
    L = np.array([min(r["pad_force_N"]["left"]) >= g for r in rows])
    R = np.array([min(r["pad_force_N"]["right"]) >= g for r in rows])
    off = np.array([max(r["pad_force_N"]["left"]) < cfg.release_force_max_N for r in rows])
    last = rows[-1]
    support = ("table" if last["table_support_N"] > 0 else "both hands" if L[-1] and R[-1] else "left hand" if L[-1] else "right hand" if R[-1] else "other (no two-pad hold)")
    release = [i for i in range(len(rows)) if off[i] and R[i]]
    retreat = [i for i in release if rows[i]["gripper_block_distance_m"]["left"] >= cfg.donor_clearance_m]
    trace = actor.trace
    dispatched = [t for t in trace if t.get("ctrl_applied") is not None]
    lo, hi = m.actuator_ctrlrange[ARM, 0], m.actuator_ctrlrange[ARM, 1]
    def projection(part):
        if not part:
            return dict(dispatched=0, projected_steps=0, longest_s=0.0, max_raw_excess_rad=0.0, per_axis={})
        mask = np.array([t["projection_mask"] for t in part], dtype=bool)
        raw = np.array([t["q_raw"] for t in part], dtype=np.float64)
        ex = np.maximum(raw[:, ARM] - hi, 0) + np.maximum(lo - raw[:, ARM], 0)
        return dict(dispatched=len(part), projected_steps=int(mask.any(1).sum()), longest_s=round(ap.longest(mask.any(1)) * td.DT, 2),
                    max_raw_excess_rad=float(ex.max()), per_axis={m.actuator(i).name: int(c) for i, c in zip(ARM, mask.sum(0)) if c})
    cut = switch["step"] if switch else len(dispatched)
    events = check["events"]
    result = dict(
        condition=label, sequence=f"START -> {successor}", gate_source="SIM_GT", switch_reached=switch is not None,
        switch=switch if switch else "SWITCH_NOT_REACHED", gate_checks_passed_both_hands=[c for c in gate_checks if c["both_hands_now"]][:5], gate_checks=len(gate_checks),
        completed_transitions=len(rec.commands), last_completed_state_sim_s=round(float(states[-1][0]), 3),
        termination="horizon_completed" if fault is None else fault["reason"], first_fault=fault,
        events=events, events_before_switch={k: v for k, v in events.items() if t_sw is None or v <= t_sw},
        new_events_after_switch={k: v for k, v in events.items() if t_sw is not None and v > t_sw},
        prefix_checker_valid=prefix_valid, run_valid=bool(check["valid"]), terminal_handover_success=bool(check["handover_success"]),
        task_success=bool(check["valid"] and check["handover_success"] and support == "right hand"),
        final_support=support, last_pad_force_N=last["pad_force_N"], last_block_clearance_m=last["block_bottom_clearance_m"],
        left_release_while_right_holds_first_s=None if not release else round(release[0] * td.DT, 2),
        left_clear_by_donor_clearance_first_s=None if not retreat else round(retreat[0] * td.DT, 2),
        projection_before_switch=projection(dispatched[:cut]), projection_after_switch=projection(dispatched[cut:]),
        first_observation_sha256=hashlib.sha256(np.ascontiguousarray(rec.cameras["left_wrist_rgb"][0]["rgb"]).tobytes()).hexdigest()[:16],
        wall_s=round(time.time() - started, 1), path=str(out))
    rec.finish(dict(composition_result=json.loads(json.dumps(result, default=str))))
    (out / "task_check_rows.json").write_text(json.dumps(rows))
    (out / "action_trace.json").write_text(json.dumps(trace))
    return result


def geometry(results):
    """The live switch states next to the two trained correction start states, in the same tool frames."""
    import mujoco
    from integration_scenes import task_env
    from sim_data_factory.task_success import ContactModel
    m = task_env("desk", grippers="both").model
    d, cm = mujoco.MjData(m), ContactModel(m)
    out = {}
    for c in load(SELECTION)["candidates"]:
        if c["id"] in TRAINED_STARTS:
            with np.load(Path(c["source_case"]) / "episode.npz", allow_pickle=False) as a:
                state = a["integration_state"][c["step"]].copy()
            mujoco.mj_setState(m, d, state, int(mujoco.mjtState.mjSTATE_INTEGRATION))
            mujoco.mj_forward(m, d)
            out[c["id"] + " (trained correction start)"] = frames(m, d, cm)
    ref = np.array(out[TRAINED_STARTS[0] + " (trained correction start)"]["q_state"])
    for r in results:
        if r["switch_reached"]:
            f = r["switch"]["state_before"]
            out[f"{r['condition']} run{r['run']} switch @ {r['switch']['sim_time_s']} s"] = f
    for k, f in out.items():
        f["q_state_max_abs_diff_from_s5000_project_1178_rad"] = float(np.abs(np.array(f["q_state"])[ARM] - ref[ARM]).max())
    return out


def run(args):
    target = run_dir() / "results.json"
    done = load(target)["results"] if target.exists() else []
    if not (run_dir() / "protocol.json").exists():
        run_dir().mkdir(parents=True, exist_ok=True)
        (run_dir() / "protocol.json").write_text(json.dumps(dict(
            run_id=RUN, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"), name="SIM-ORACLE-GATED POLICY COMPOSITION / ORIGINAL RESET",
            seed=SEED, conditions=[dict(label=a, sequence=f"START -> {b}") for a, b in CONDITIONS], gate=GATE,
            gate_code="live_release_check.one: actor.due_for_inference() -> ContactModel.evidence(d) -> evaluate_states(states so far)",
            switch="once; old chunk discarded; successor infers from the RGB and q_state of that moment; no setState / restore / interpolation / forced release",
            not_reached="SWITCH_NOT_REACHED is reported apart; no retry with other times or conditions",
            repeat="only if START -> B gives run-valid new handover with final right-hand support: each condition once more (6 runs at most)",
            models={k: str(v) for k, v in MODELS.items()}, training=False, val_test=False, hardware_access=False), indent=2))
    for label, successor in CONDITIONS:
        if any(r["condition"] == label and r["run"] == args.repeat for r in done):
            print(f"[reused] {label} run{args.repeat}")
            continue
        result = one(label, successor, run_dir() / "runs" / label / f"run{args.repeat}")
        result["run"] = args.repeat
        done.append(result)
        target.write_text(json.dumps(dict(run_id=RUN, note="SIM-ORACLE-GATED POLICY COMPOSITION / ORIGINAL RESET; not a privileged-information-free student run",
                                          results=done, geometry=geometry(done)), indent=2, default=str))
        sw = result["switch"]
        print(json.dumps(dict(condition=label, run=args.repeat, sequence=result["sequence"], switch=sw if isinstance(sw, str) else dict(step=sw["step"], t=sw["sim_time_s"]),
                              termination=result["termination"], last_s=result["last_completed_state_sim_s"], events={k: round(v, 2) for k, v in result["events"].items()},
                              run_valid=result["run_valid"], task_success=result["task_success"], final_support=result["final_support"]), default=str), flush=True)
    print(f"[written] {target}")


def video(args):
    import recovery_trial as rt
    rows = []
    results = load(run_dir() / "results.json")["results"]
    for label, successor in CONDITIONS:
        r = next(x for x in results if x["condition"] == label and x["run"] == args.repeat)
        row = rt.teacher_row(f"SIM-ORACLE-GATED: {label} START -> {successor} (seed 5000, original reset)", Path(r["path"]))
        ev = dict(r["events"])
        if r["switch_reached"]:
            ev[f"SWITCH->{successor}"] = r["switch"]["sim_time_s"]
        row["events"] = dict(sorted(ev.items(), key=lambda kv: kv[1]))
        row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}",
                           ("VALID HANDOVER, right hand holds" if r["task_success"] else f"no valid handover; final support: {r['final_support']}"))
        rows.append(row)
    rt.render(rows, run_dir() / "videos" / f"seed5000_control_replay_recovery_run{args.repeat}.mp4", "SIM-ORACLE-GATED POLICY COMPOSITION / ORIGINAL RESET", 2000, False, right_gripper=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("run", "video"):
        p = sub.add_parser(name)
        p.add_argument("--repeat", type=int, default=1)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    dict(run=run, video=video)[args.command](args)


if __name__ == "__main__":
    main()
