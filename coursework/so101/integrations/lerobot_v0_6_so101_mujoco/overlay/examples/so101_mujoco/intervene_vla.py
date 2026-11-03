#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run wrist-only VLA with pause, keyboard correction, resume, and evidence capture."""

from __future__ import annotations

import argparse
import contextlib
import threading
import time
from pathlib import Path

import numpy as np
import torch
from teleoperate import X11HeldKeyPoller, wait_for_viewer_shutdown

from lerobot.configs.policies import PreTrainedConfig
from lerobot.envs import SO101MujocoEnvConfig, make_env_pre_post_processors, preprocess_observation
from lerobot.envs.so101_mujoco import (
    HUMAN_AUTHORITY,
    PICK_CLEAR_ACTION,
    POLICY_AUTHORITY,
    CartesianJogController,
    InterventionEpisodeRecorder,
    JointJogController,
    SO101MujocoEnv,
    VLAInterventionSession,
)
from lerobot.policies import make_policy, make_pre_post_processors
from lerobot.utils.constants import ACTION
from lerobot.utils.random_utils import set_seed


class InterventionKeyboard:
    """Thread-safe authority and manual jog controls for the passive viewer."""

    def __init__(
        self,
        controller: CartesianJogController,
        session: VLAInterventionSession,
        *,
        joint_speed_degrees: float,
        gripper_speed_percent: float,
        cartesian_speed_m: float,
    ) -> None:
        import glfw

        self.controller = controller
        self.session = session
        self.joint_speed_degrees = float(joint_speed_degrees)
        self.gripper_speed_percent = float(gripper_speed_percent)
        self.cartesian_speed_m = float(cartesian_speed_m)
        self._lock = threading.Lock()
        self._pending_edges: set[int] = set()
        self._armed_keys: set[int] = set()
        self._handled_edges: set[int] = set()
        self._takeover_requested = False
        self._resume_requested = False
        self._camera_requested = False
        self._new_episode_requested = False
        self._stop_requested = False
        self._camera_mode = "external"
        self._episode_seed = 0
        self._step_index = 0
        self._continuous_keys = {
            glfw.KEY_UP,
            glfw.KEY_DOWN,
            glfw.KEY_W,
            glfw.KEY_S,
            glfw.KEY_A,
            glfw.KEY_D,
            glfw.KEY_R,
            glfw.KEY_F,
            glfw.KEY_O,
            glfw.KEY_L,
        }
        self._shift_edges = {glfw.KEY_J, glfw.KEY_K, glfw.KEY_C, glfw.KEY_N, glfw.KEY_Q}
        self._direct_edges = {glfw.KEY_SPACE, glfw.KEY_ENTER}
        self._command_keys = self._continuous_keys | self._shift_edges | self._direct_edges
        self._shift_keys = {glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT}
        self.key_names = {
            glfw.KEY_UP: "Up",
            glfw.KEY_DOWN: "Down",
            glfw.KEY_W: "w",
            glfw.KEY_S: "s",
            glfw.KEY_A: "a",
            glfw.KEY_D: "d",
            glfw.KEY_R: "r",
            glfw.KEY_F: "f",
            glfw.KEY_O: "o",
            glfw.KEY_L: "l",
            glfw.KEY_J: "j",
            glfw.KEY_K: "k",
            glfw.KEY_C: "c",
            glfw.KEY_N: "n",
            glfw.KEY_Q: "q",
            glfw.KEY_SPACE: "space",
            glfw.KEY_ENTER: "Return",
            glfw.KEY_LEFT_SHIFT: "Shift_L",
            glfw.KEY_RIGHT_SHIFT: "Shift_R",
        }

    def on_key(self, keycode: int) -> None:
        with self._lock:
            if keycode in self._command_keys:
                self._armed_keys.add(keycode)
                self._pending_edges.add(keycode)

    def poll_controls(self, poller: X11HeldKeyPoller, elapsed_seconds: float) -> None:
        import glfw

        pressed = poller.pressed()
        shift_pressed = bool(self._shift_keys & pressed)
        with self._lock:
            released = self._armed_keys - pressed
            self._armed_keys.intersection_update(pressed)
            self._handled_edges.difference_update(released)
            edge_candidates = self._pending_edges & self._direct_edges
            if shift_pressed:
                edge_candidates |= self._pending_edges & self._shift_edges
            self._pending_edges.clear()
            new_edges = edge_candidates - self._handled_edges
            self._handled_edges.update(new_edges)

            if glfw.KEY_SPACE in new_edges:
                self._takeover_requested = True
            if glfw.KEY_ENTER in new_edges:
                self._resume_requested = True
            if glfw.KEY_C in new_edges:
                self._camera_requested = True
            if glfw.KEY_N in new_edges:
                self._new_episode_requested = True
            if glfw.KEY_Q in new_edges:
                self._stop_requested = True
            if self.session.authority != HUMAN_AUTHORITY:
                return
            if glfw.KEY_J in new_edges:
                self.controller.select_previous_joint()
            if glfw.KEY_K in new_edges:
                self.controller.select_next_joint()

            active = self._armed_keys & pressed & self._continuous_keys if shift_pressed else set()
            selected_direction = int(glfw.KEY_UP in active) - int(glfw.KEY_DOWN in active)
            if selected_direction:
                speed = (
                    self.gripper_speed_percent
                    if self.controller.selected_joint == 5
                    else self.joint_speed_degrees
                )
                self.controller.jog_selected_joint(selected_direction * speed * elapsed_seconds)
            gripper_direction = int(glfw.KEY_O in active) - int(glfw.KEY_L in active)
            if gripper_direction:
                self.controller.adjust_gripper(
                    gripper_direction * self.gripper_speed_percent * elapsed_seconds
                )
            delta = (
                self.cartesian_speed_m
                * elapsed_seconds
                * np.array(
                    [
                        int(glfw.KEY_W in active) - int(glfw.KEY_S in active),
                        int(glfw.KEY_A in active) - int(glfw.KEY_D in active),
                        int(glfw.KEY_R in active) - int(glfw.KEY_F in active),
                    ],
                    dtype=np.float64,
                )
            )
            if np.any(delta):
                self.controller.move(delta)

    def consume_takeover_request(self) -> bool:
        with self._lock:
            requested = self._takeover_requested
            self._takeover_requested = False
            return requested

    def request_takeover(self) -> None:
        with self._lock:
            self._takeover_requested = True

    def consume_resume_request(self) -> bool:
        with self._lock:
            requested = self._resume_requested
            self._resume_requested = False
            return requested

    def request_resume(self) -> None:
        with self._lock:
            self._resume_requested = True

    def consume_camera_request(self) -> bool:
        with self._lock:
            requested = self._camera_requested
            self._camera_requested = False
            return requested

    def consume_new_episode_request(self) -> bool:
        with self._lock:
            requested = self._new_episode_requested
            self._new_episode_requested = False
            return requested

    @property
    def stop_requested(self) -> bool:
        with self._lock:
            return self._stop_requested

    def update_status(self, *, camera_mode: str, episode_seed: int, step_index: int) -> None:
        with self._lock:
            self._camera_mode = camera_mode
            self._episode_seed = episode_seed
            self._step_index = step_index

    @property
    def status_text(self) -> str:
        with self._lock:
            authority = self.session.authority.upper()
            return (
                f"authority: {authority}\n"
                f"scene seed: {self._episode_seed}\n"
                f"step: {self._step_index}\n"
                f"viewer camera: {self._camera_mode}\n"
                f"interventions: {self.session.intervention_segments}\n"
                f"selected: {self.controller.selected_joint_name}"
            )


VIEWER_HELP = (
    "Space: TAKE OVER   Enter: RESUME VLA\n"
    "HUMAN only — Shift+W/S: X   Shift+A/D: Y   Shift+R/F: Z\n"
    "Shift+O/L: gripper   Shift+Up/Down: selected joint\n"
    "Shift+J/K: select joint   Shift+C: external/top/wrist camera\n"
    "Shift+N: next episode   Shift+Q: quit"
)


def update_viewer_texts(viewer_handle, keyboard: InterventionKeyboard) -> None:
    import mujoco

    viewer_handle.set_texts(
        [
            (None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, "VLA INTERVENTION", VIEWER_HELP),
            (None, mujoco.mjtGridPos.mjGRID_TOPRIGHT, "SO-101 SIM", keyboard.status_text),
        ]
    )


def set_viewer_camera(viewer_handle, model, mode: str) -> None:
    import mujoco

    with viewer_handle.lock():
        if mode == "external":
            viewer_handle.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer_handle.cam.fixedcamid = -1
        else:
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, mode)
            if camera_id < 0:
                raise RuntimeError(f"scene is missing the {mode} camera")
            viewer_handle.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer_handle.cam.fixedcamid = camera_id
    print(f"viewer_camera={mode}")


def load_policy(args: argparse.Namespace, env_cfg: SO101MujocoEnvConfig):
    policy_cfg = PreTrainedConfig.from_pretrained(args.policy_path)
    policy_cfg.pretrained_path = args.policy_path
    policy_cfg.n_action_steps = args.action_horizon
    policy = make_policy(cfg=policy_cfg, env_cfg=env_cfg)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_cfg,
        pretrained_path=args.policy_path,
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)}},
    )
    env_preprocessor, env_postprocessor = make_env_pre_post_processors(env_cfg=env_cfg, policy_cfg=policy_cfg)
    return policy, preprocessor, postprocessor, env_preprocessor, env_postprocessor


def infer_action(
    observation,
    *,
    task: str,
    policy,
    preprocessor,
    postprocessor,
    env_preprocessor,
    env_postprocessor,
) -> np.ndarray:
    batch = preprocess_observation(observation)
    batch["task"] = [task]
    batch = env_preprocessor(batch)
    batch = preprocessor(batch)
    with torch.inference_mode():
        action = policy.select_action(batch)
    action = postprocessor(action)
    action = env_postprocessor({ACTION: action})[ACTION]
    action_numpy = action.detach().to("cpu").numpy()
    if action_numpy.shape != (1, 6):
        raise RuntimeError(f"policy returned unexpected action shape {action_numpy.shape}")
    return action_numpy[0].astype(np.float32)


def run(args: argparse.Namespace) -> None:
    set_seed(args.seed)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    env_cfg = SO101MujocoEnvConfig(
        episode_length=args.steps,
        observation_height=args.height,
        observation_width=args.width,
        cube_xy_randomization=args.cube_randomization,
        camera_names=("wrist",),
        home_action=tuple(float(value) for value in PICK_CLEAR_ACTION),
        action_smoothing=True,
        action_chunk_steps=args.action_horizon,
        action_trace_path=str(args.output_dir / "action_trace.jsonl"),
    )
    env = SO101MujocoEnv(**env_cfg.gym_kwargs)
    print(f"loading_policy={args.policy_path}")
    policy, preprocessor, postprocessor, env_preprocessor, env_postprocessor = load_policy(args, env_cfg)
    observation, info = env.reset(seed=args.seed)
    assert env.model is not None and env.data is not None
    controller = CartesianJogController(
        env.model,
        joint_controller=JointJogController(home_action=PICK_CLEAR_ACTION),
    )
    session = VLAInterventionSession()
    keyboard = InterventionKeyboard(
        controller,
        session,
        joint_speed_degrees=args.joint_speed_degrees,
        gripper_speed_percent=args.gripper_speed_percent,
        cartesian_speed_m=args.cartesian_speed_m,
    )
    recorder = InterventionEpisodeRecorder(args.output_dir / "interventions")
    recorder.start_episode(episode_index=0, seed=args.seed, task=env.task_description)
    policy.reset()

    viewer_handle = None
    last_policy_action: np.ndarray | None = None
    episode_index = 0
    step_index = 0
    completed_episodes = 0
    successes = 0
    camera_modes = ("external", "top", "wrist")
    camera_mode = "external"
    episode_seed = args.seed
    active_episode = True

    print(
        "controls=Space takeover | Enter resume VLA | Shift+W/S,A/D,R/F Cartesian | "
        "Shift+O/L gripper | Shift+C camera | Shift+N next | Shift+Q quit"
    )
    print(f"intervention_output={recorder.root}")
    try:
        with contextlib.ExitStack() as stack:
            key_poller = None
            if not args.no_viewer:
                import mujoco.viewer

                viewer_handle = stack.enter_context(
                    mujoco.viewer.launch_passive(
                        env.model,
                        env.data,
                        key_callback=keyboard.on_key,
                        show_left_ui=False,
                        show_right_ui=False,
                    )
                )
                key_poller = X11HeldKeyPoller(keyboard.key_names)
                stack.callback(key_poller.close)

            while completed_episodes < args.episodes and not keyboard.stop_requested:
                if viewer_handle is not None and not viewer_handle.is_running():
                    break
                frame_started = time.perf_counter()
                if key_poller is not None:
                    keyboard.poll_controls(key_poller, 1.0 / env.fps)

                if args.scripted_takeover_step == step_index and session.authority == POLICY_AUTHORITY:
                    keyboard.request_takeover()
                if (
                    args.scripted_takeover_step is not None
                    and step_index == args.scripted_takeover_step + args.scripted_takeover_frames
                    and session.authority == HUMAN_AUTHORITY
                ):
                    keyboard.request_resume()

                if keyboard.consume_takeover_request() and session.take_over(observation["agent_pos"]):
                    controller.set_action(observation["agent_pos"])
                    policy.reset()
                    print(f"authority=human episode={episode_index} step={step_index}")
                if keyboard.consume_resume_request() and session.resume_policy():
                    policy.reset()
                    print(f"authority=policy episode={episode_index} step={step_index} queue=reset")
                if viewer_handle is not None and keyboard.consume_camera_request():
                    camera_mode = camera_modes[(camera_modes.index(camera_mode) + 1) % len(camera_modes)]
                    set_viewer_camera(viewer_handle, env.model, camera_mode)

                manually_ended = keyboard.consume_new_episode_request()
                if manually_ended:
                    done = True
                    reward = 0.0
                    termination_reason = "manual_next"
                else:
                    policy_action = None
                    if session.authority == POLICY_AUTHORITY:
                        policy_action = infer_action(
                            observation,
                            task=env.task_description,
                            policy=policy,
                            preprocessor=preprocessor,
                            postprocessor=postprocessor,
                            env_preprocessor=env_preprocessor,
                            env_postprocessor=env_postprocessor,
                        )
                        last_policy_action = policy_action.copy()
                    decision = session.choose_action(
                        policy_action=policy_action,
                        human_action=(
                            controller.get_action() if session.authority == HUMAN_AUTHORITY else None
                        ),
                    )
                    with viewer_handle.lock() if viewer_handle is not None else contextlib.nullcontext():
                        next_observation, reward, terminated, truncated, info = env.step(decision.action)
                    done = terminated or truncated
                    termination_reason = "success" if terminated else "time_limit" if truncated else "running"
                    recorder.record_frame(
                        step_index=step_index,
                        source=decision.source,
                        intervention_segment=decision.intervention_segment,
                        observation_state=observation["agent_pos"],
                        wrist_rgb=observation["pixels"]["wrist"],
                        requested_action=decision.action,
                        applied_action=info["action_applied"],
                        last_policy_action=last_policy_action,
                        reward=reward,
                        success=bool(info["is_success"]),
                        done=done,
                    )
                    observation = next_observation
                    step_index += 1

                keyboard.update_status(
                    camera_mode=camera_mode, episode_seed=episode_seed, step_index=step_index
                )
                if viewer_handle is not None:
                    update_viewer_texts(viewer_handle, keyboard)
                    viewer_handle.sync()

                if done:
                    success = bool(info["is_success"])
                    manifest = recorder.finish_episode(
                        success=success,
                        termination_reason=termination_reason,
                        intervention_segments=session.intervention_segments,
                    )
                    active_episode = False
                    completed_episodes += 1
                    successes += int(success)
                    print(
                        f"episode={episode_index} success={success} "
                        f"intervention_frames={session.intervention_frames} manifest={manifest}"
                    )
                    if completed_episodes >= args.episodes:
                        break
                    episode_index += 1
                    episode_seed = args.seed + episode_index
                    with viewer_handle.lock() if viewer_handle is not None else contextlib.nullcontext():
                        observation, info = env.reset(seed=episode_seed)
                    step_index = 0
                    last_policy_action = None
                    session.reset_episode()
                    controller.set_action(observation["agent_pos"])
                    policy.reset()
                    recorder.start_episode(
                        episode_index=episode_index,
                        seed=episode_seed,
                        task=env.task_description,
                    )
                    active_episode = True

                remaining = 1.0 / env.fps - (time.perf_counter() - frame_started)
                if remaining > 0:
                    time.sleep(remaining)
        if viewer_handle is not None:
            wait_for_viewer_shutdown(viewer_handle)
    finally:
        if active_episode:
            recorder.finish_episode(
                success=bool(info["is_success"]),
                termination_reason="viewer_closed_or_quit",
                intervention_segments=session.intervention_segments,
            )
        recorder.close()
        env.close()

    print(f"finished episodes={completed_episodes} successes={successes} intervention_output={recorder.root}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy-path", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--steps", type=int, default=700)
    parser.add_argument("--seed", type=int, default=1400)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--cube-randomization", type=float, default=0.025)
    parser.add_argument("--action-horizon", type=int, default=25)
    parser.add_argument("--joint-speed-degrees", type=float, default=35.0)
    parser.add_argument("--gripper-speed-percent", type=float, default=60.0)
    parser.add_argument("--cartesian-speed-m", type=float, default=0.08)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--scripted-takeover-step", type=int)
    parser.add_argument("--scripted-takeover-frames", type=int, default=3)
    args = parser.parse_args()
    if args.episodes <= 0 or args.steps <= 0 or args.action_horizon <= 0:
        parser.error("--episodes, --steps, and --action-horizon must be positive")
    if args.seed < 0 or args.cube_randomization < 0:
        parser.error("--seed and --cube-randomization must be non-negative")
    if args.scripted_takeover_step is not None and args.scripted_takeover_step < 0:
        parser.error("--scripted-takeover-step must be non-negative")
    if args.scripted_takeover_frames <= 0:
        parser.error("--scripted-takeover-frames must be positive")
    return args


if __name__ == "__main__":
    run(parse_args())
