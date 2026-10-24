#!/usr/bin/env python3

import tempfile
from pathlib import Path

import cv2
import h5py
import numpy as np

from convert_episode import convert


def _jpeg(image: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


def main() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        source = root / "episode.hdf5"
        output = root / "episode.npz"
        with h5py.File(source, "w") as episode:
            episode.create_group("additional_info").create_dataset("frequency", data=15)
            for group_name in ("state", "action"):
                group = episode.create_group(group_name)
                for name, width in (
                    ("left_arm_joint_states", 5),
                    ("left_ee_joint_states", 1),
                    ("right_arm_joint_states", 5),
                    ("right_ee_joint_states", 1),
                ):
                    group.create_dataset(name, data=np.zeros((2, width), np.float32))
            vision = episode.create_group("vision")
            for name, height, width in (
                ("cam_left_wrist", 240, 320),
                ("cam_head", 460, 640),
                ("cam_right_wrist", 240, 320),
            ):
                camera = vision.create_group(name)
                encoded = [_jpeg(np.zeros((height, width, 3), np.uint8)) for _ in range(2)]
                camera.create_dataset("colors", data=encoded, dtype=f"S{max(map(len, encoded))}")
                if name == "cam_head":
                    camera.create_dataset("depths", data=np.full((2, 460, 640), 500.0))
        report = convert(source, output)
        with np.load(output) as episode:
            assert episode["observation_state"].shape == (2, 12)
            assert episode["action"].shape == (2, 12)
            assert episode["left_wrist_rgb"].shape == (2, 3, 240, 320)
            assert episode["top_h201_depth_mm"].shape == (2, 1, 460, 640)
            assert episode["right_wrist_rgb"].shape == (2, 3, 240, 320)
            assert episode["top_h201_depth_mm"].dtype == np.uint16
        assert report["hardware_execution"] is False
        with h5py.File(source, "r+") as episode:
            episode["action/left_arm_joint_states"][1, 0] = 0.1
        try:
            convert(source, root / "unsafe.npz")
        except ValueError as error:
            assert "0.5 rad/s" in str(error)
        else:
            raise AssertionError("unsafe arm action was accepted")
    print("PASS: RoboTwin HDF5 -> DAPIER Dual SO-101 canonical episode")


if __name__ == "__main__":
    main()
