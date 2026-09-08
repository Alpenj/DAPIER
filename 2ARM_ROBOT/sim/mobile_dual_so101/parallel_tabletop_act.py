#!/usr/bin/env python3
"""SIM ONLY: native ACT closed-loop wiring, not calibrated task evaluation.

CPU processes own their physics and ACT instances; EGL may use the GPU to render.
No dataset actions, future states or object ground truth enter policy inference.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

from mobile_dual_so101 import apply_control_as_pose, actuator_targets_from_qpos
from replay_recorded_episode import (
    ARM, GRIPPER, block_contacts, build_tabletop, finite_vector, inspect_contacts,
    load_episode, recorded_to_sim, sha256, sim_to_recorded, physics_settings,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from train_act_baseline import INPUT_KEYS, RGB_KEYS


def policy_observation(state, images):
    import torch
    state = finite_vector(state, 12, "policy state")
    if set(images) != set(RGB_KEYS):
        raise ValueError("both wrist images are required")
    result = {"observation.state": torch.tensor(state, dtype=torch.float32)[None]}
    for key in RGB_KEYS:
        rgb = np.asarray(images[key])
        if rgb.shape != (240, 320, 3) or rgb.dtype != np.uint8:
            raise ValueError("expected 240x320 uint8 RGB")
        result[key] = torch.from_numpy(rgb.copy()).permute(2, 0, 1)[None].float() / 255
    assert set(result) == INPUT_KEYS
    return result


def bounded_command(model, profile, proposal, previous, fps, arm_speed, gripper_speed):
    proposal = finite_vector(proposal, 12, "ACT proposal")
    # ACT's regression head is unbounded. Project only the command candidate;
    # retain the raw proposal and projection for diagnostics.
    a = sim_to_recorded(model, model.actuator_ctrlrange[:, 0], profile)
    b = sim_to_recorded(model, model.actuator_ctrlrange[:, 1], profile)
    projected = np.clip(proposal, np.minimum(a, b), np.maximum(a, b))
    previous = finite_vector(previous, 12, "previous command")
    recorded_to_sim(model, previous, profile)
    rates = finite_vector([fps, arm_speed, gripper_speed], 3, "control rates")
    if np.any(rates <= 0):
        raise ValueError("control rates must be positive")
    maximum = np.full(12, arm_speed / fps)
    maximum[GRIPPER] = gripper_speed / fps
    limited = previous + np.clip(projected - previous, -maximum, maximum)
    return limited, recorded_to_sim(model, limited, profile), projected


def run_worker(worker_id, episode_ids, options):
    import torch
    from PIL import Image
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    torch.set_num_threads(options["threads"])
    torch.manual_seed(options["seed"])
    checkpoint = Path(options["checkpoint"])
    config = PreTrainedConfig.from_pretrained(checkpoint, local_files_only=True)
    config.device = "cpu"  # Do not stage checkpoint weights on CUDA in each child.
    policy = ACTPolicy.from_pretrained(checkpoint, config=config, local_files_only=True, strict=True).eval()
    if (set(policy.config.input_features) != INPUT_KEYS or policy.config.n_action_steps != 1
            or policy.config.temporal_ensemble_coeff is not None
            or list(policy.config.output_features["action"].shape) != [12]):
        raise ValueError("expected two-wrist + state ACT, every-step inference, 12 joint outputs")
    pre, post = make_pre_post_processors(policy.config, pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": "cpu"}})
    profile = options["profile"]
    model = build_tabletop(Path(options["model"]), profile)
    fps = options["fps"]
    substeps = int(np.ceil(1 / fps / .002))
    model.opt.timestep = 1 / (fps * substeps)
    renderer = mujoco.Renderer(model, height=240, width=320)
    reports = []
    viewer = None
    try:
        for episode_id in episode_ids:
            output = Path(options["output"]) / f"episode-{episode_id:03d}"
            output.mkdir(exist_ok=False)
            data = mujoco.MjData(model)
            apply_control_as_pose(model, data, recorded_to_sim(model, options["initial_state"], profile))
            # Reproducible engineering perturbation, not a measured task-start distribution.
            rng = np.random.default_rng(options["seed"] + episode_id)
            block_adr = model.joint("red_block_free").qposadr[0]
            data.qpos[block_adr:block_adr + 2] += rng.uniform(-.005, .005, 2)
            mujoco.mj_forward(model, data)
            if options["viewer"] and worker_id == 0:
                from mujoco import viewer as mj_viewer
                viewer = mj_viewer.launch_passive(model, data)
                viewer.cam.lookat[:] = [.13, 0, .22]
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.25, 140, -35
                viewer.sync()
            policy.reset()
            previous = np.asarray(options["initial_state"]).copy()
            traces, failure = [], None
            policy_queries = 0
            deepest = {"depth_m": 0.0}
            started = time.perf_counter()
            for step in range(options["steps"]):
                proposal = None
                tick_start = time.perf_counter()
                try:
                    if viewer is not None and not viewer.is_running():
                        raise ValueError("viewer closed by user")
                    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                        raise ValueError("non-finite physics state")
                    if not np.isclose(data.time, step / fps, atol=1e-9, rtol=0):
                        raise ValueError("stale or mis-timed observation")
                    state = sim_to_recorded(model, actuator_targets_from_qpos(model, data.qpos), profile)
                    measured_q = np.asarray(actuator_targets_from_qpos(model, data.qpos))
                    overshoot = np.maximum(0, np.maximum(model.actuator_ctrlrange[:, 0] - measured_q,
                                                        measured_q - model.actuator_ctrlrange[:, 1]))
                    images = {}
                    for side, key in zip(("left", "right"), RGB_KEYS):
                        renderer.update_scene(data, camera=f"{side}_wrist_rgb")
                        images[key] = renderer.render().copy()
                        if step in (0, options["steps"] - 1):
                            Image.fromarray(images[key]).save(output / f"{side}-{step:04d}.png")
                    before_qpos = data.qpos.copy()
                    captured_time = float(data.time)
                    inference_start = time.perf_counter()
                    policy_queries += 1
                    with torch.inference_mode():
                        proposal = post(policy.select_action(pre(policy_observation(state, images))))[0].cpu().numpy()
                    inference_seconds = time.perf_counter() - inference_start
                    if inference_seconds > options["inference_timeout"]:
                        raise ValueError("inference exceeded bounded smoke timeout")
                    if data.time != captured_time or not np.array_equal(data.qpos, before_qpos):
                        raise ValueError("state changed between capture and control")
                    command, target, projected = bounded_command(model, profile, proposal, previous, fps,
                        options["arm_speed"], options["gripper_speed"])
                    inspect_contacts(model, data, deepest, step)
                    data.ctrl[:] = target
                    for _ in range(substeps):
                        mujoco.mj_step(model, data)
                        inspect_contacts(model, data, deepest, step)
                    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                        raise ValueError("non-finite physics after action")
                    warnings = [int(w.number) for w in data.warning]
                    if any(warnings):
                        raise ValueError(f"MuJoCo warning: {warnings}")
                    traces.append({"step": step, "observation_time_s": captured_time,
                        "next_time_s": float(data.time), "state_source_units": state.tolist(),
                        "proposal_source_units": proposal.tolist(), "command_source_units": command.tolist(),
                        "range_projected_source_units": projected.tolist(),
                        "measured_limit_overshoot_rad": overshoot.tolist(),
                        "ctrl_rad": target.tolist(), "qpos_before": before_qpos.tolist(),
                        "qpos_after": data.qpos.tolist(), "inference_seconds": inference_seconds,
                        "image_sha256": {key: hashlib.sha256(im.tobytes()).hexdigest() for key, im in images.items()},
                        "verifier_only_block_position_m": data.body("red_block").xpos.tolist(),
                        "verifier_only_bilateral_contact": block_contacts(model, data)})
                    previous = command
                    if viewer is not None:
                        viewer.set_images([(mujoco.MjrRect(i * 320, 0, 320, 240), images[key])
                                           for i, key in enumerate(RGB_KEYS)])
                        viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100,
                            mujoco.mjtGridPos.mjGRID_TOPLEFT,
                            "ACT + TWO WRIST RGB | SIM ONLY",
                            f"step {step + 1}/{options['steps']} | {data.time:.2f}s | NOT CALIBRATED")])
                        viewer.sync()
                        time.sleep(max(0, 1 / fps - (time.perf_counter() - tick_start)))
                except ValueError as error:
                    failure = {"step": step, "reason": str(error)}
                    if proposal is not None:
                        failure["proposal_source_units"] = np.asarray(proposal).tolist()
                    break  # No further simulation command or fallback/replay after rejection.
            report = {"episode": episode_id, "seed": options["seed"] + episode_id,
                "physics_settings": physics_settings(model),
                "worker_id": worker_id, "pid": os.getpid(), "transitions": len(traces),
                "policy_queries": policy_queries, "simulation_seconds": float(data.time),
                "range_projected_steps": sum(not np.allclose(t["proposal_source_units"], t["range_projected_source_units"], atol=1e-6, rtol=0) for t in traces),
                "slew_limited_steps": sum(not np.allclose(t["range_projected_source_units"], t["command_source_units"], atol=1e-6, rtol=0) for t in traces),
                "distinct_wrist_images": {key: len({t["image_sha256"][key] for t in traces}) for key in RGB_KEYS},
                "max_measured_limit_overshoot_rad": max((max(t["measured_limit_overshoot_rad"]) for t in traces), default=0),
                "failure": failure, "wall_seconds": time.perf_counter() - started,
                "deepest_contact": deepest, "task_success": None,
                "mujoco_warning_counts": [int(w.number) for w in data.warning]}
            (output / "trace.json").write_text(json.dumps(traces, indent=2) + "\n")
            (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
            reports.append(report)
            print(json.dumps(report), flush=True)
            if viewer is not None:
                viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT, "STOPPED | SIM ONLY",
                    failure["reason"] if failure else "Bounded run complete; close window to finish")])
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(.05)
                viewer.close()
                viewer = None
    finally:
        if viewer is not None:
            viewer.close()
        renderer.close()
    return {"pid": os.getpid(), "worker_id": worker_id, "episodes": reports}


def run(args):
    root, checkpoint, output = args.dataset.resolve(), args.checkpoint.resolve(), args.output.resolve()
    for protected in (root, checkpoint):
        if output == protected or protected in output.parents or output in protected.parents:
            raise ValueError("use a separate new output, outside dataset and checkpoint")
    if output.exists():
        raise ValueError("output already exists; never overwrite a previous run")
    if not 1 <= args.workers <= 4 or not 1 <= args.episodes <= 100 or not 1 <= args.steps <= 1500:
        raise ValueError("bounded smoke requires workers 1..4, episodes 1..100, steps 1..1500")
    rates = finite_vector([args.arm_speed, args.gripper_speed, args.inference_timeout], 3, "limits")
    if np.any(rates <= 0) or not 1 <= args.threads <= 8 or args.seed < 0:
        raise ValueError("positive limits and 1..8 CPU threads required")
    profile = json.loads(args.scene.read_text())
    states, _, fps, _ = load_episode(root, args.initial_episode)
    if fps != 15 or not 0 <= args.initial_frame < len(states):
        raise ValueError("expected 15Hz dataset and valid initialization frame")
    paths = [p for p in root.rglob("*") if p.is_file()]
    paths += [p for p in checkpoint.rglob("*") if p.is_file()] + [args.scene, args.model]
    before = {str(p): sha256(p) for p in paths}
    output.mkdir(parents=True, exist_ok=False)
    options = {**vars(args), "dataset": str(root), "checkpoint": str(checkpoint), "output": str(output),
        "model": str(args.model.resolve()), "scene": str(args.scene.resolve()),
        "profile": profile, "fps": fps, "initial_state": states[args.initial_frame].tolist()}
    (output / "run_contract.json").write_text(json.dumps(options, indent=2) + "\n")
    workers = min(args.workers, args.episodes)
    started = time.perf_counter()
    with ProcessPoolExecutor(max_workers=workers, mp_context=multiprocessing.get_context("spawn")) as executor:
        futures = [executor.submit(run_worker, i, list(range(i, args.episodes, workers)), options)
                   for i in range(workers)]
        results = [future.result() for future in futures]
    elapsed = time.perf_counter() - started
    episodes = [episode for worker in results for episode in worker["episodes"]]
    pids = {worker["pid"] for worker in results}
    preserved = before == {str(p): sha256(p) for p in paths}
    complete = len(pids) == workers and all(ep["transitions"] == args.steps and ep["failure"] is None for ep in episodes)
    report = {"kind": "uncalibrated native-ACT closed-loop engineering smoke", "schema_version": 1,
        "hardware_execution": False, "learning_executed": False, "policy": "native ACT checkpoint",
        "policy_device": "cpu", "physics_backend": "MuJoCo CPU", "render_backend": os.environ.get("MUJOCO_GL", "default"),
        "workers": workers, "worker_pids": sorted(pids), "independent_processes": len(pids) == workers,
        "fps_simulation_time": fps, "real_time_control_verified": False,
        "wall_seconds_including_startup": elapsed, "transitions": sum(ep["transitions"] for ep in episodes),
        "all_requested_transitions_completed": complete, "episodes": episodes,
        "initialization_only": {"episode": args.initial_episode, "frame": args.initial_frame},
        "policy_input_keys": sorted(INPUT_KEYS), "stored_actions_or_future_states_passed_to_policy": False,
        "object_ground_truth_passed_to_policy": False, "base_alignment_reapplied": False,
        "source_checkpoint_scene_hashes_unchanged": preserved, "input_sha256": before,
        "task_success_rate": None, "task_evaluation_valid": False,
        "physical_mapping_verified": False, "ready_for_sim_to_real": False,
        "limitations": ["Unmeasured optics/joint zeros/object pose/contact parameters invalidate task success claims.",
            "Command slew limits are not physical velocity or collision safety verification.",
            "Current scene penetration is measured, not suppressed; no collision-safe hardware adapter exists here.",
            "CPU inference wall latency is measured; 15Hz here is simulated time only.",
            "No optimizer or reinforcement learning runs in this entrypoint."]}
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    if not preserved:
        raise RuntimeError("protected input changed during run")
    print(json.dumps({k: report[k] for k in ("transitions", "all_requested_transitions_completed", "worker_pids", "wall_seconds_including_startup", "task_success_rate")}))
    return 0 if complete else 2


def self_test():
    from types import SimpleNamespace
    model = SimpleNamespace(actuator_ctrlrange=np.tile([-np.pi, np.pi], (12, 1)))
    profile = {"arm_signs": [-1] + [1] * 9, "arm_zero_offsets_deg": [7] + [0] * 9}
    previous = np.zeros(12)
    previous[GRIPPER] = 50
    proposal = previous + 10
    bounded, target, projected = bounded_command(model, profile, proposal, previous, 15, 30, 60)
    assert np.allclose(bounded[ARM] - previous[ARM], 2)
    assert np.allclose(bounded[GRIPPER] - previous[GRIPPER], 4)
    assert np.allclose(sim_to_recorded(model, target, profile), bounded)
    images = {key: np.zeros((240, 320, 3), dtype=np.uint8) for key in RGB_KEYS}
    obs = policy_observation(previous, images)
    assert set(obs) == INPUT_KEYS and obs[RGB_KEYS[0]].shape == (1, 3, 240, 320)
    clipped, _, projected = bounded_command(model, profile, np.full(12, 1000), previous, 15, 30, 60)
    assert np.allclose(projected[GRIPPER], 100) and np.allclose(clipped[GRIPPER], 54)
    for bad in (proposal + np.nan, np.zeros(11)):
        try:
            bounded_command(model, profile, bad, previous, 15, 30, 60)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid proposal accepted")
    for bad_images in ({}, {**images, RGB_KEYS[0]: np.zeros((240, 320, 3))}):
        try:
            policy_observation(previous, bad_images)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid camera input accepted")
    print("PASS: camera contract, joint mapping, slew bounds, invalid input rejection")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--viewer", action="store_true", help="show worker 0 in real time; pause at end until window closes")
    for name in ("dataset", "checkpoint", "model", "output"):
        parser.add_argument(f"--{name}", type=Path)
    parser.add_argument("--scene", type=Path, default=Path(__file__).with_name("tabletop_replay.json"))
    parser.add_argument("--initial-episode", type=int, default=0)
    parser.add_argument("--initial-frame", type=int, default=0)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--steps", type=int, default=30)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--arm-speed", type=float, default=30, help="command slew limit, degrees/s")
    parser.add_argument("--gripper-speed", type=float, default=60, help="command slew limit, percentage points/s")
    parser.add_argument("--inference-timeout", type=float, default=10, help="bounded smoke timeout, seconds")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif any(getattr(args, name) is None for name in ("dataset", "checkpoint", "model", "output")):
        parser.error("dataset, checkpoint, model and new output are required")
    else:
        raise SystemExit(run(args))
