#!/usr/bin/env python3
"""Server S0: first validation of an execution host (no training, no teacher, no selection, no hardware).

  check     imports, MJCF compile, EGL headless render, strict load of START / RECEIVE_E / RELEASE_B, normalization load,
            inference on stored observations of the laptop reference runs (compared with the chunks stored there)
  run       three original-reset runs START -> RECEIVE_E -> RELEASE_B, seed 5000, 5008, 5011 once each, one new process
            each, unchanged executor (C25, repaired contract, projection, checker, physics, 40 s); nothing is repeated
  compare   each run against its stored laptop reference: outcome, event times and the first divergence
  summary   this job's own tables

The runs get this job's run-id and the identity of this host; they are not added to any earlier local denominator.
Switches read SIM ground truth: SIM-ORACLE-GATED THREE-POLICY CANDIDATE / SERVER S0.
"""
import argparse
import getpass
import hashlib
import json
import os
import platform
import socket
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import train_rollout_diagnostic as td   # noqa: E402
import receive_candidate as rc   # noqa: E402
import receive_fit_check as fit   # noqa: E402
import receive_r3_recheck as rk   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "server-s0-20261006-01"
LABEL = rc.LABEL + " / SERVER S0"
SEEDS = (5000, 5008, 5011)
RECHECK = FACTORY_ROOT / "evaluation/receive-r3-recheck-20261005-01/runs/4f539ef7304f"
D1 = FACTORY_ROOT / "evaluation/start-pick-d1-20261006-01/runs/9971de44b522"
REFERENCE = {5000: RECHECK / "s5000/E_rep1", 5008: RECHECK / "s5008/E_rep1", 5011: D1 / "s5011"}
LOCAL_PAIR = {5000: (RECHECK / "s5000/E_rep1", RECHECK / "s5000/E_rep2"), 5008: (RECHECK / "s5008/E_rep1", RECHECK / "s5008/E_rep2")}
MODELS = dict(START=rk.MODELS["START"], RECEIVE_E=rk.MODELS["E"], RELEASE_B=rk.MODELS["RELEASE_B"])
EXPECTED = dict(START=rk.EXPECTED["START"], RECEIVE_E=rk.EXPECTED["E"], RELEASE_B=rk.EXPECTED["RELEASE_B"])
EVENTS = ("pick_lift_s", "right_grasp_s", "receive_entry_s", "gate_s", "handover_s")
STEPS = (1e-6, 1e-3, 1e-2)

load, sha, array_sha = rk.load, rk.sha, rk.array_sha
_recheck_identity = rk.identity


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def host():
    def smi(field):
        try:
            return subprocess.run(["nvidia-smi", f"--query-gpu={field}", "--format=csv,noheader"], capture_output=True, text=True, timeout=20).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return None
    return dict(hostname=socket.gethostname(), user=getpass.getuser(), kernel=platform.release(), machine=platform.machine(), nvidia_driver=smi("driver_version"),
                gpu=smi("name"), python_executable=sys.executable)


def identity():
    """The recheck identity (same models, sources, contract, execution) under this job's run-id, plus this host and this file."""
    body = _recheck_identity()
    body.pop("sha256")
    body.update(run_id=RUN, host=host(), cases=[[s, "E", 1] for s in SEEDS])
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def bind():
    """Point the unchanged recheck executor at this job."""
    rk.RUN, rk.LABEL, rk.identity = RUN, LABEL, identity
    if "factory/v1/server_s0.py" not in rk.SOURCES:
        rk.SOURCES = rk.SOURCES + ("factory/v1/server_s0.py",)


def reference_arrays(path):
    with np.load(path / "episode.npz", allow_pickle=False) as a:
        states, q = a["integration_state"].copy(), a["q_state"].copy()
    with np.load(path / "predicted_chunks.npz", allow_pickle=False) as a:
        chunks = dict(time=a["time"].copy(), actions=a["actions"].astype(np.float64))
    rgb = {s: np.load(path / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}
    return states, q, chunks, rgb


def check(args=None):
    out = run_dir()
    out.mkdir(parents=True, exist_ok=True)
    target = out / "checks.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    rows = {}
    import lerobot
    import mujoco
    import torch
    import torchvision
    import safetensors
    import cv2
    import arm_output_trial as trial
    import frozen_transfer_check as ft
    from integration_scenes import task_env
    from sim_data_factory.reset_sampler import make_task_episode
    rows["1_imports"] = dict(python=platform.python_version(), torch=torch.__version__, torchvision=torchvision.__version__, numpy=np.__version__, mujoco=mujoco.__version__,
                             lerobot=lerobot.__version__, safetensors=safetensors.__version__, opencv=cv2.__version__, cuda_available=torch.cuda.is_available(),
                             cuda=torch.version.cuda, cudnn=torch.backends.cudnn.version(), gpu=torch.cuda.get_device_name(0), lerobot_path=str(Path(lerobot.__file__).parent), host=host())
    mjcf = os.environ["DAPIER_SO101_MJCF"]
    arm = mujoco.MjModel.from_xml_path(mjcf)
    env = task_env("desk", grippers="both")
    rows["2_mjcf_compile"] = dict(path=mjcf, sha256=sha(mjcf), arm=dict(nq=arm.nq, nv=arm.nv, nu=arm.nu, ngeom=arm.ngeom, nmesh=arm.nmesh),
                                  task_scene=dict(nq=env.model.nq, nv=env.model.nv, nu=env.model.nu, ngeom=env.model.ngeom, ncam=env.model.ncam, timestep=env.model.opt.timestep))
    import OpenGL.GL as G
    renderer = mujoco.Renderer(env.model, 120, 160)
    data = mujoco.MjData(env.model)
    mujoco.mj_forward(env.model, data)
    renderer.update_scene(data, camera=0)
    frame = renderer.render()
    rows["3_egl_render"] = dict(MUJOCO_GL=os.environ.get("MUJOCO_GL"), vendor=G.glGetString(G.GL_VENDOR).decode(), renderer=G.glGetString(G.GL_RENDERER).decode(),
                                version=G.glGetString(G.GL_VERSION).decode(), frame_shape=list(frame.shape), frame_min=int(frame.min()), frame_max=int(frame.max()),
                                distinct_values=int(len(np.unique(frame))), display=os.environ.get("DISPLAY"))
    renderer.close()
    rc.MODELS["RECEIVE"] = MODELS["RECEIVE_E"]
    policies, loads, norms = {}, {}, {}
    for name, key in (("START", "START"), ("RECEIVE_E", "RECEIVE"), ("RELEASE_B", "B")):
        policy, stats, ident = rc.load_policy(key)                       # ACTPolicy.from_pretrained(strict=True) + check_normalization
        policies[name] = policy
        loads[name] = dict(**ident, expected_model_sha256=EXPECTED[name], sha_match=ident["model_sha256"] == EXPECTED[name], strict=True, training_flag=bool(policy.training),
                           parameters=int(sum(p.numel() for p in policy.parameters())), device=str(next(policy.parameters()).device))
        norms[name] = dict(path=str(MODELS[name].parent / "normalization.npz"), sha256=ident["normalization_sha256"], expected=rk.NORMALIZATION,
                           sha_match=ident["normalization_sha256"] == rk.NORMALIZATION, shapes={k: {s: list(v.shape) for s, v in d.items()} for k, d in stats.items()},
                           finite=bool(all(np.isfinite(v).all() for d in stats.values() for v in d.values())), std_min=float(min(v.min() for d in stats.values() for v in [d["std"]])),
                           checked_by_policy=True)
    rows["4_strict_load"], rows["5_normalization"] = loads, norms
    trial.apply_reproducibility_settings()
    torch.manual_seed(4100)
    fixed = {}
    for seed in (5000, 5008):
        ref = REFERENCE[seed]
        result = load(ref / "result.json")
        states, _, chunks, rgb = reference_arrays(ref)
        meta = ft.demonstration(seed)["metadata"]
        ctx = make_task_episode(seed, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
        points = [("START", 0)] + [(dict(RECEIVE="RECEIVE_E", B="RELEASE_B")[s["to"]], int(s["step"])) for s in result["switches"]]
        for name, step in points:
            t = float(states[step][0])
            index = np.flatnonzero(np.isclose(chunks["time"], t, atol=1e-9))
            got, _, _, _, trace = fit.runner_prediction(policies[name], ctx, states[step], {s: rgb[s][step] for s in ("left", "right")})
            row = dict(reference=str(ref), step=step, sim_time_s=round(t, 2), state_sha256=array_sha(states[step]), rgb_sha256={s: array_sha(rgb[s][step]) for s in ("left", "right")},
                       inference_wall_s=trace["inference_wall_s"], finite=bool(np.isfinite(got).all()), chunk_shape=list(got.shape))
            if len(index):
                diff = np.abs(got - chunks["actions"][index[0]])
                row.update(laptop_chunk_index=int(index[0]), max_abs_diff_all=float(diff.max()), max_abs_diff_k0_24=float(diff[:rc.PREFIX].max()), mean_abs_diff=float(diff.mean()),
                           bit_identical=bool(np.array_equal(got.astype(np.float32), chunks["actions"][index[0]].astype(np.float32))))
            else:
                row["laptop_chunk_index"] = None
            fixed[f"s{seed}_{name}"] = row
    rows["6_fixed_observation_inference"] = dict(note="stored laptop state + stored laptop wrist frames -> this host's forward pass (Student.next_command path); compared with the chunk the laptop "
                                                      "produced from the same observation. Units: rad (arms), normalized opening (grippers).", cases=fixed)
    target.write_text(json.dumps(dict(run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), checks=rows), indent=2, default=str))
    print(json.dumps(rows, indent=1, default=str))


def run(args=None):
    bind()
    ident = identity()
    out = run_dir()
    (out / "logs").mkdir(parents=True, exist_ok=True)
    proto = out / "protocol.json"
    if proto.exists():
        raise FileExistsError(f"{proto} exists: this job was already started; it is not run again")
    proto.write_text(json.dumps(dict(
        run_id=RUN, name=LABEL, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"), identity=ident, cases=[[s, "E", 1] for s in SEEDS],
        order="5000, 5008, 5011; sequential, one new process per case", budget="3 new original-reset runs, once each; no repeat until success, no replacement, no extension",
        references={str(s): str(p) for s, p in REFERENCE.items()},
        reference_meaning="5000 / 5008: earlier local successes; 5011: earlier local START fault. A different outcome is recorded with its first divergence and is not repaired.",
        denominators="this job's three runs stand alone; they are not added to earlier local runs",
        training=False, teacher_collection=False, val_test=False, hardware_access=False), indent=2, default=str))
    status = []
    for seed in SEEDS:
        log = out / "logs" / f"s{seed}.log"
        cmd = [sys.executable, str(Path(__file__).resolve()), "one", "--seed", str(seed), "--identity", ident["sha256"]]
        t0 = time.time()
        with open(log, "w") as f:
            code = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT).returncode
        status.append(dict(seed=seed, returncode=code, wall_s=round(time.time() - t0, 1), log=str(log), path=str(rk.case_dir(ident, seed, "E", 1))))
        print(json.dumps(status[-1]))
    (out / "run_status.json").write_text(json.dumps(status, indent=2))
    compare()


def one(args):
    bind()
    rk.one(argparse.Namespace(seed=args.seed, model="E", rep=1, identity=args.identity))


def first_difference(a, b):
    """First control step where two q_state tracks differ at all, and by more than each of STEPS (rad / normalized opening)."""
    n = min(len(a), len(b))
    gap = np.abs(a[:n] - b[:n]).max(1)
    out = dict(compared_steps=int(n), max_abs=float(gap.max()))
    for name, limit in (("any", 0.0),) + tuple((f"gt_{x:g}", x) for x in STEPS):
        hit = np.flatnonzero(gap > limit)
        out[name] = None if not len(hit) else dict(step=int(hit[0]), sim_time_s=round(int(hit[0]) * td.DT, 2), channel=int(np.abs(a[hit[0]] - b[hit[0]]).argmax()), abs=float(gap[hit[0]]))
    return out


def pair(a_dir, b_dir):
    sa, qa, ca, ra = reference_arrays(a_dir)
    sb, qb, cb, rb_ = reference_arrays(b_dir)
    out = dict(reset_state_identical=bool(np.array_equal(sa[0], sb[0])), q_state=first_difference(qa, qb))
    frames = {}
    for s in ("left", "right"):
        n = min(len(ra[s]), len(rb_[s]))
        d = np.abs(ra[s][:n].astype(np.int16) - rb_[s][:n].astype(np.int16))
        per = d.reshape(n, -1).max(1)
        hit = np.flatnonzero(per > 0)
        frames[s] = dict(step0_pixels_differing=int((d[0].max(-1) > 0).sum()), step0_max_abs=int(d[0].max()),
                         first_differing_frame=None if not len(hit) else dict(step=int(hit[0]), sim_time_s=round(int(hit[0]) * td.DT, 2), pixels=int((d[hit[0]].max(-1) > 0).sum()), max_abs=int(per[hit[0]])))
    out["wrist_rgb"] = frames
    c0 = np.abs(ca["actions"][0] - cb["actions"][0])
    out["first_chunk"] = dict(max_abs=float(c0.max()), max_abs_k0_24=float(c0[:rc.PREFIX].max()), bit_identical=bool(np.array_equal(ca["actions"][0], cb["actions"][0])))
    return out


def outcome(r):
    fault = r.get("first_fault")
    return dict(task_success=r["task_success"], termination=r["termination"], final_support=r["final_support"], fault_stage=r.get("fault_stage"),
                first_fault=None if fault is None else dict(sim_time_s=round(fault["time"], 2), forbidden_contacts=fault.get("forbidden_contacts")),
                projected_steps={k: v["projected_steps"] for k, v in (r.get("projection") or {}).items()}, **{k: r.get(k) for k in EVENTS})


def compare(args=None):
    bind()
    ident = load(run_dir() / "protocol.json")["identity"]
    rows = []
    for seed in SEEDS:
        mine = rk.case_dir(ident, seed, "E", 1)
        row = dict(seed=seed, path=str(mine), reference=str(REFERENCE[seed]))
        if not (mine / "result.json").exists():
            row["status"] = "no result on this host"
            rows.append(row)
            continue
        a, b = outcome(load(mine / "result.json")), outcome(load(REFERENCE[seed] / "result.json"))
        row.update(server=a, laptop_reference=b, same_outcome=all(a[k] == b[k] for k in ("task_success", "termination", "final_support", "fault_stage")),
                   event_time_difference_s={k: None if a[k] is None or b[k] is None else round(a[k] - b[k], 2) for k in EVENTS}, server_vs_laptop=pair(mine, REFERENCE[seed]))
        if seed in LOCAL_PAIR:
            row["laptop_rep1_vs_laptop_rep2"] = pair(*LOCAL_PAIR[seed])
            row["laptop_event_range"] = {k: [x for x in sorted({load(RECHECK / f"s{seed}/E_rep{i}/result.json").get(k) for i in range(1, 6)} - {None})] for k in EVENTS}
        rows.append(row)
    (run_dir() / "results.json").write_text(json.dumps(dict(run_id=RUN, name=LABEL, identity_sha256=ident["sha256"], written=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                                                            note="three runs of this host only; never merged with earlier local denominators", cases=rows), indent=2, default=str))
    summary()


def summary(args=None):
    data = load(run_dir() / "results.json")
    print(f"\n=== SERVER S0 ({RUN}, identity {data['identity_sha256'][:12]}) — {LABEL}; this host's three runs only ===")
    print("| seed | host | success | termination | pick_lift | right_grasp | receive | release_b | handover | fault stage |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for row in data["cases"]:
        if "server" not in row:
            print(f"| {row['seed']} | server | {row['status']} |")
            continue
        for who, r in (("server", row["server"]), ("laptop ref", row["laptop_reference"])):
            print(f"| {row['seed']} | {who} | {r['task_success']} | {r['termination']} | " + " | ".join(str(r[k]) for k in EVENTS) + f" | {r['fault_stage']} |")
    print("\n| seed | pair | reset equal | step-0 px differing L/R | first chunk max diff | q first > 1e-6 | q first > 1e-3 | q first > 1e-2 |")
    print("|---|---|---|---|---|---|---|---|")
    for row in data["cases"]:
        for label, key in (("server vs laptop", "server_vs_laptop"), ("laptop rep1 vs rep2", "laptop_rep1_vs_laptop_rep2")):
            p = row.get(key)
            if p is None:
                continue
            q = p["q_state"]
            cell = lambda x: "none" if x is None else f"{x['sim_time_s']} s (ch {x['channel']})"   # noqa: E731
            print(f"| {row['seed']} | {label} | {p['reset_state_identical']} | {p['wrist_rgb']['left']['step0_pixels_differing']} / {p['wrist_rgb']['right']['step0_pixels_differing']} | "
                  f"{p['first_chunk']['max_abs']:.2e} | {cell(q['gt_1e-06'])} | {cell(q['gt_0.001'])} | {cell(q['gt_0.01'])} |")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("check", "run", "compare", "summary"):
        sub.add_parser(name)
    p = sub.add_parser("one")
    p.add_argument("--seed", type=int, required=True, choices=SEEDS)
    p.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    return dict(check=check, run=run, compare=compare, summary=summary, one=one)[args.command](args)


if __name__ == "__main__":
    main()
