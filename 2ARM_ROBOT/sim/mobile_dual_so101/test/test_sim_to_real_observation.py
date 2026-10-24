from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
RESEARCH_SRC = PROJECT_DIR.parents[1] / "research" / "src"
sys.path.insert(0, str(PROJECT_DIR))
sys.path.insert(0, str(RESEARCH_SRC))

try:
    import mujoco  # noqa: F401
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

from dapier_research.observation_contract import (  # noqa: E402
    validate_observation_for_use,
)
from sim_policy import ActionChunkExecutor, HoldChunkPolicy  # noqa: E402
from sim_to_real_observation import (  # noqa: E402
    SensorPolicyExecutor,
    build_sim_to_real_policy_observation,
    portable_robot_state,
)
from vision_guided_reach import (  # noqa: E402
    RenderedRgbdFrame,
    SimBlueShoeDetector,
    build_vision_policy_observation,
)
from dapier_research.vision_target import PinholeIntrinsics  # noqa: E402


def _robot_state() -> dict[str, object]:
    return {
        "base_pose_map": (1.0, 2.0, 0.3),
        "left_arm_rad": (0.0,) * 5,
        "left_gripper_normalized": 0.5,
        "right_arm_rad": (0.0,) * 5,
        "right_gripper_normalized": 0.5,
        "base_velocity": (0.0, 0.0),
    }


def _synthetic_frame() -> RenderedRgbdFrame:
    rgb = np.zeros((7, 9, 3), dtype=np.uint8)
    rgb[2:5, 3:6] = (20, 45, 230)
    depth = np.full((7, 9), np.nan, dtype=np.float32)
    depth[2:5, 3:6] = 1.0
    return RenderedRgbdFrame(
        rgb=rgb,
        depth_m=depth,
        intrinsics=PinholeIntrinsics(
            width=9,
            height=7,
            fx=100.0,
            fy=100.0,
            cx=4.0,
            cy=3.0,
        ),
        target_from_optical=np.eye(4, dtype=np.float64),
        world_from_target=np.eye(4, dtype=np.float64),
        timestamp_ns=123,
        clock_domain="mujoco_sim_time",
        camera_frame="fixture_camera_optical",
        target_frame="tb3_base_link",
    )


class SimToRealObservationTest(unittest.TestCase):
    def test_adapter_strips_simulator_world_pose_and_adds_sensor_provenance(self) -> None:
        observation = build_sim_to_real_policy_observation(
            _synthetic_frame(),
            robot_state=_robot_state(),
            detector=SimBlueShoeDetector(ignore_top_fraction=0.0),
        )
        self.assertNotIn("base_pose_map", observation["robot"])
        self.assertNotIn("shoe", observation)
        self.assertTrue(observation["sim_to_real_observation_compatible"])
        self.assertTrue(observation["target_available"])
        self.assertFalse(observation["ground_truth"])
        provenance = validate_observation_for_use(
            observation,
            use="policy_runtime",
        )
        self.assertEqual(provenance.source_kind, "sensor_runtime")
        self.assertFalse(provenance.ground_truth_used)
        self.assertIn("fixture_camera_optical", provenance.sensor_frames)

    def test_raw_legacy_vision_observation_requires_explicit_provenance(self) -> None:
        raw = build_vision_policy_observation(
            _synthetic_frame(),
            robot_state=_robot_state(),
            detector=SimBlueShoeDetector(ignore_top_fraction=0.0),
        )
        with self.assertRaisesRegex(ValueError, "explicit observation_provenance"):
            validate_observation_for_use(raw, use="policy_runtime")

    def test_strict_executor_rejects_ground_truth_before_policy_query(self) -> None:
        delegate = ActionChunkExecutor(
            HoldChunkPolicy((0.0,) * 12, chunk_size=2),
            mode="receding_horizon",
        )
        executor = SensorPolicyExecutor(delegate)
        legacy = {
            "ground_truth": True,
            "shoe": {"position_map_m": (0.3, 0.0, 0.01)},
            "robot": _robot_state(),
        }
        with self.assertRaisesRegex(ValueError, "explicit observation_provenance"):
            executor.next_action(legacy)
        self.assertEqual(executor.policy_queries, 0)

        observation = build_sim_to_real_policy_observation(
            _synthetic_frame(),
            robot_state=_robot_state(),
            detector=SimBlueShoeDetector(ignore_top_fraction=0.0),
        )
        self.assertEqual(executor.next_action(observation), (0.0,) * 12)
        self.assertEqual(executor.policy_queries, 1)

    def test_detection_failure_never_falls_back_to_object_truth(self) -> None:
        observation = build_sim_to_real_policy_observation(
            _synthetic_frame(),
            robot_state=_robot_state(),
            detector=SimBlueShoeDetector(
                minimum_blue=250,
                ignore_top_fraction=0.0,
            ),
        )
        self.assertFalse(observation["target_available"])
        self.assertIsNone(observation["target"])
        self.assertEqual(observation["failure"]["code"], "detection_not_found")
        self.assertNotIn("shoe", observation)
        provenance = validate_observation_for_use(
            observation,
            use="policy_runtime",
        )
        self.assertFalse(provenance.ground_truth_used)

    def test_portable_robot_state_requires_both_arms(self) -> None:
        sanitized = portable_robot_state(_robot_state())
        self.assertEqual(
            set(sanitized),
            {
                "left_arm_rad",
                "left_gripper_normalized",
                "right_arm_rad",
                "right_gripper_normalized",
                "base_velocity",
            },
        )
        with self.assertRaisesRegex(ValueError, "missing portable fields"):
            portable_robot_state({"left_arm_rad": (0.0,) * 5})


if __name__ == "__main__":
    unittest.main()
