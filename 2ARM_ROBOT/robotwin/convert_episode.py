#!/usr/bin/env python3
"""Convert one RoboTwin HDF5 episode to DAPIER's canonical Dual SO-101 arrays."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile

import cv2
import h5py
import numpy as np


JOINT_FIELDS = (
    ("left_arm_joint_states", 5),
    ("left_ee_joint_states", 1),
    ("right_arm_joint_states", 5),
    ("right_ee_joint_states", 1),
)
CAMERAS = {
    "cam_left_wrist": (240, 320),
    "cam_head": (460, 640),
    "cam_right_wrist": (240, 320),
}
MAX_ARM_VELOCITY_RAD_S = 0.5


def _joints(group: h5py.Group, label: str) -> np.ndarray:
    arrays = []
    frame_count = None
    for name, width in JOINT_FIELDS:
        if name not in group:
            raise ValueError(f"{label} is missing {name}")
        values = np.asarray(group[name], dtype=np.float32)
        if values.ndim == 1:
            values = values[:, None]
        if values.ndim != 2 or values.shape[1] != width:
            raise ValueError(f"{label}/{name} expected (*,{width}), got {values.shape}")
        frame_count = len(values) if frame_count is None else frame_count
        if len(values) != frame_count:
            raise ValueError(f"{label} joint streams have different frame counts")
        arrays.append(values)
    result = np.concatenate(arrays, axis=1)
    if not np.isfinite(result).all():
        raise ValueError(f"{label} contains NaN or Inf")
    if ((result[:, (5, 11)] < 0) | (result[:, (5, 11)] > 1)).any():
        raise ValueError(f"{label} grippers must be normalized to [0,1]")
    return result


def _colors(group: h5py.Group, expected_hw: tuple[int, int], frames: int) -> np.ndarray:
    if "colors" not in group:
        raise ValueError(f"{group.name} is missing colors")
    decoded = []
    for encoded in group["colors"]:
        image = cv2.imdecode(np.frombuffer(bytes(encoded), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"failed to decode JPEG in {group.name}")
        if image.shape[:2] != expected_hw:
            raise ValueError(
                f"{group.name} expected {expected_hw}, got {image.shape[:2]}"
            )
        # RoboTwin encodes its RGB ndarray directly through OpenCV. Keeping the
        # decoded channel order preserves the original numeric RGB values.
        decoded.append(np.moveaxis(image, -1, 0))
    result = np.asarray(decoded, dtype=np.uint8)
    if len(result) != frames:
        raise ValueError(f"{group.name} frame count does not match state/action")
    return result


def load_episode(path: Path) -> dict[str, np.ndarray | int]:
    with h5py.File(path, "r") as episode:
        state = _joints(episode["state"], "state")
        action = _joints(episode["action"], "action")
        if state.shape != action.shape or len(state) == 0:
            raise ValueError("state/action must have the same non-zero shape")
        raw_frequency = np.asarray(episode["additional_info/frequency"])
        if raw_frequency.ndim != 0 or raw_frequency.dtype.kind not in "iuf":
            raise ValueError("frequency must be a numeric scalar positive integer")
        value = raw_frequency.item()
        if (
            not np.isfinite(value)
            or value <= 0
            or value != int(value)
            or int(value) > np.iinfo(np.int64).max
        ):
            raise ValueError("frequency must be a finite positive int64 integer")
        frequency = int(value)
        if not np.allclose(action[:-1], state[1:], atol=1e-5, rtol=0):
            raise ValueError("RoboTwin action[t] must equal measured state[t+1]")
        # Include the first measured state: action-only differences miss the
        # initial transition and every transition in a single-frame episode.
        trajectory = np.concatenate((state[:1], action), axis=0)
        arm = np.c_[trajectory[:, :5], trajectory[:, 6:11]]
        if len(arm) > 1 and np.abs(np.diff(arm, axis=0)).max() > (
            MAX_ARM_VELOCITY_RAD_S / frequency + 1e-4
        ):
            raise ValueError("RoboTwin arm action exceeds the 0.5 rad/s data limit")
        vision = episode["vision"]
        missing = set(CAMERAS) - set(vision)
        if missing:
            raise ValueError(f"vision is missing {sorted(missing)}")
        depth = np.asarray(vision["cam_head/depths"])
        if depth.shape != (len(state), 460, 640):
            raise ValueError(f"H201 depth expected {(len(state),460,640)}, got {depth.shape}")
        if not np.isfinite(depth).all() or (depth < 0).any():
            raise ValueError("H201 depth must contain finite non-negative millimetres")
        depth_mm = np.clip(np.rint(depth), 0, np.iinfo(np.uint16).max).astype(np.uint16)
        return {
            "fps": frequency,
            "observation_state": state,
            "action": action,
            "left_wrist_rgb": _colors(vision["cam_left_wrist"], CAMERAS["cam_left_wrist"], len(state)),
            "top_h201_depth_mm": depth_mm[:, None],
            "right_wrist_rgb": _colors(vision["cam_right_wrist"], CAMERAS["cam_right_wrist"], len(state)),
        }


def convert(source: Path, output: Path) -> dict[str, object]:
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"refusing to overwrite {output}")
    values = load_episode(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{output.name}.", dir=output.parent)
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        with temporary.open("wb") as stream:
            np.savez_compressed(stream, **values)
        # A same-directory hard link publishes complete bytes without replacing
        # an output created by another writer after the initial existence check.
        # Unsupported filesystems fail closed; never fall back to replace().
        os.link(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)
    report = {
        "source": str(source),
        "output": str(output),
        "frames": int(values["action"].shape[0]),
        "fps": int(values["fps"]),
        "state_action_shape": list(values["action"].shape),
        "left_rgb_shape": list(values["left_wrist_rgb"].shape),
        "h201_depth_shape": list(values["top_h201_depth_mm"].shape),
        "right_rgb_shape": list(values["right_wrist_rgb"].shape),
        "hardware_execution": False,
    }
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    print(json.dumps(convert(args.source.resolve(), args.output.absolute()), indent=2))


if __name__ == "__main__":
    main()
