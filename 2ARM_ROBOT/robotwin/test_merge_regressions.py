"""Hardware-free regression gates for canonical episode conversion.

Fixtures are handcrafted HDF5, not RoboTwin writer/rollout or physical IL data.
"""

from pathlib import Path
import tempfile
import subprocess
import sys
import unittest
from unittest.mock import patch

import cv2
import h5py
import numpy as np

import convert_episode as converter


class ConverterRegressionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "episode.hdf5"
        self.output = self.root / "episode.npz"

    def fixture(self, *, frames=2, frequency=15, state=None, action=None):
        state = np.zeros((frames, 12), np.float32) if state is None else state
        action = np.zeros((frames, 12), np.float32) if action is None else action
        with h5py.File(self.source, "w") as episode:
            episode.create_dataset("additional_info/frequency", data=frequency)
            for label, values in (("state", state), ("action", action)):
                offset = 0
                for name, width in converter.JOINT_FIELDS:
                    episode.create_dataset(f"{label}/{name}", data=values[:, offset:offset + width])
                    offset += width
            for name, color in (("cam_left_wrist", (200, 30, 10)),
                                ("cam_right_wrist", (10, 60, 180))):
                image = np.full((240, 320, 3), color, np.uint8)
                ok, encoded = cv2.imencode(".jpg", image)
                self.assertTrue(ok)
                payload = encoded.tobytes()
                episode.create_dataset(f"vision/{name}/colors", data=[payload] * frames,
                                       dtype=f"S{len(payload)}")
            episode.create_dataset("vision/cam_head/depths",
                                   data=np.full((frames, 460, 640), 500, np.uint16))

    def test_valid_episode_schema_and_camera_roles(self):
        self.fixture()
        converter.convert(self.source, self.output)
        with np.load(self.output, allow_pickle=False) as episode:
            self.assertEqual(set(episode.files), {"fps", "observation_state", "action",
                             "left_wrist_rgb", "right_wrist_rgb", "top_h201_depth_mm"})
            self.assertEqual(episode["action"].shape, (2, 12))
            self.assertEqual(episode["observation_state"].dtype, np.float32)
            self.assertEqual(episode["left_wrist_rgb"].shape, (2, 3, 240, 320))
            self.assertEqual(episode["right_wrist_rgb"].dtype, np.uint8)
            self.assertGreater(episode["left_wrist_rgb"][0, 0].mean(), 190)
            self.assertGreater(episode["right_wrist_rgb"][0, 2].mean(), 170)
            self.assertEqual(episode["top_h201_depth_mm"].shape, (2, 1, 460, 640))
            self.assertEqual(episode["top_h201_depth_mm"].dtype, np.uint16)
            self.assertTrue((episode["top_h201_depth_mm"] == 500).all())

    def test_first_transition_jump_is_rejected(self):
        state = np.zeros((2, 12), np.float32)
        action = state.copy()
        state[1, 0] = 2
        action[:, 0] = 2
        self.fixture(state=state, action=action)
        with self.assertRaisesRegex(ValueError, "0.5 rad/s"):
            converter.convert(self.source, self.output)
        self.assertFalse(self.output.exists())

    def test_single_frame_jump_is_rejected(self):
        action = np.zeros((1, 12), np.float32)
        action[0, 6] = 2
        self.fixture(frames=1, action=action)
        with self.assertRaisesRegex(ValueError, "0.5 rad/s"):
            converter.convert(self.source, self.output)

    def test_single_frame_hold_is_allowed(self):
        self.fixture(frames=1)
        self.assertEqual(converter.convert(self.source, self.output)["frames"], 1)

    def test_small_transition_is_allowed(self):
        state = np.zeros((2, 12), np.float32)
        action = state.copy()
        state[:, 0] = (0, 0.01)
        action[:, 0] = (0.01, 0.02)
        self.fixture(state=state, action=action)
        self.assertEqual(converter.convert(self.source, self.output)["frames"], 2)

    def test_final_transition_jump_is_rejected(self):
        action = np.zeros((2, 12), np.float32)
        action[1, 0] = 2
        self.fixture(action=action)
        with self.assertRaisesRegex(ValueError, "0.5 rad/s"):
            converter.convert(self.source, self.output)

    def test_shift_mismatch_is_rejected(self):
        action = np.full((2, 12), 0.001, np.float32)
        self.fixture(action=action)
        with self.assertRaisesRegex(ValueError, "measured state"):
            converter.convert(self.source, self.output)

    def test_gripper_and_nonfinite_values_are_rejected(self):
        for index, value in ((5, 50), (11, -0.1), (0, np.nan), (6, np.inf)):
            with self.subTest(index=index, value=value):
                state = np.zeros((2, 12), np.float32)
                state[:, index] = value
                self.fixture(state=state)
                with self.assertRaises(ValueError):
                    converter.convert(self.source, self.output)

    def test_fractional_frequency_is_rejected(self):
        self.fixture(frequency=29.97)
        with self.assertRaisesRegex(ValueError, "frequency"):
            converter.convert(self.source, self.output)

    def test_invalid_frequency_is_rejected(self):
        for frequency in (0, -1, np.nan, np.inf, True, "15", np.uint64(2**63), np.array([15])):
            with self.subTest(frequency=str(frequency)):
                self.fixture(frequency=frequency)
                with self.assertRaisesRegex(ValueError, "frequency"):
                    converter.load_episode(self.source)

    def test_integral_float_frequency_is_preserved(self):
        self.fixture(frequency=15.0)
        self.assertEqual(converter.convert(self.source, self.output)["fps"], 15)

    def test_existing_output_is_unchanged(self):
        self.fixture()
        self.output.write_bytes(b"existing user's output")
        with self.assertRaises(FileExistsError):
            converter.convert(self.source, self.output)
        self.assertEqual(self.output.read_bytes(), b"existing user's output")

    def test_concurrently_created_output_is_unchanged(self):
        self.fixture()
        save = np.savez_compressed

        def save_then_race(*args, **kwargs):
            save(*args, **kwargs)
            with self.output.open("xb") as stream:
                stream.write(b"other writer's result")

        with patch.object(converter.np, "savez_compressed", side_effect=save_then_race):
            with self.assertRaises(FileExistsError):
                converter.convert(self.source, self.output)
        self.assertEqual(self.output.read_bytes(), b"other writer's result")
        self.assertEqual(list(self.root.glob(f".{self.output.name}.*")), [])

    def test_dangling_symlink_is_preserved(self):
        self.fixture()
        target = self.root / "not-created.npz"
        try:
            self.output.symlink_to(target)
        except (NotImplementedError, OSError) as error:
            self.skipTest(f"symlinks unavailable: {error}")
        with self.assertRaises(FileExistsError):
            converter.convert(self.source, self.output)
        self.assertTrue(self.output.is_symlink())
        self.assertFalse(target.exists())

    def test_cli_does_not_follow_dangling_output_symlink(self):
        self.fixture()
        target = self.root / "not-created.npz"
        try:
            self.output.symlink_to(target)
        except (NotImplementedError, OSError) as error:
            self.skipTest(f"symlinks unavailable: {error}")
        result = subprocess.run(
            [sys.executable, "-B", converter.__file__, str(self.source), str(self.output)],
            capture_output=True, text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("FileExistsError", result.stderr)
        self.assertTrue(self.output.is_symlink())
        self.assertFalse(target.exists())

    def test_save_failure_does_not_publish_output(self):
        self.fixture()
        with patch.object(converter.np, "savez_compressed", side_effect=OSError("fixture failure")):
            with self.assertRaisesRegex(OSError, "fixture failure"):
                converter.convert(self.source, self.output)
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.root.glob(f".{self.output.name}.*")), [])


if __name__ == "__main__":
    unittest.main()
