from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

from collision_guard import CollisionAssessment  # noqa: E402
from physics_ik import IKResult  # noqa: E402
from vision_guided_reach import (  # noqa: E402
    RenderedRgbdFrame,
    SimBlueShoeDetector,
    build_vision_policy_observation,
    estimate_shoe_from_frame,
    frame_from_rendered_rgbd,
    plan_vision_guided_reach_from_frame,
    render_rgbd_frame,
)
from dapier_research.vision_target import (  # noqa: E402
    PinholeIntrinsics,
    VisionTargetError,
)
from mobile_dual_so101 import ACTION_NAMES  # noqa: E402


TINY_RGBD_SCENE = """
<mujoco model="vision_projection_fixture">
  <option gravity="0 0 0"/>
  <visual>
    <headlight ambient="0.8 0.8 0.8" diffuse="0.8 0.8 0.8" specular="0 0 0"/>
  </visual>
  <worldbody>
    <camera name="fixture_camera" pos="0 0 0" quat="1 0 0 0" fovy="60"/>
    <body name="blue_target" pos="0.10 0.05 -1.0">
      <geom
        name="blue_target_geom"
        type="box"
        size="0.03 0.03 0.01"
        rgba="0.02 0.08 0.95 1"
        contype="0"
        conaffinity="0"
      />
    </body>
  </worldbody>
</mujoco>
"""


def _safe_collision() -> CollisionAssessment:
    return CollisionAssessment(
        safe=True,
        reason="fixture path is clear",
        minimum_clearance_m=0.05,
        required_clearance_m=0.03,
        path_fraction=1.0,
        checked_samples=2,
        first_body="left_arm",
        second_body="right_arm",
        first_geom_id=1,
        second_geom_id=2,
    )


def _synthetic_frame(*, target_from_optical: np.ndarray | None = None) -> RenderedRgbdFrame:
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
        target_from_optical=(
            np.eye(4, dtype=np.float64)
            if target_from_optical is None
            else target_from_optical
        ),
        world_from_target=np.eye(4, dtype=np.float64),
        timestamp_ns=123,
        clock_domain="mujoco_sim_time",
        camera_frame="fixture_camera_optical",
        target_frame="map_sim_world",
    )


class VisionGuidedReachTest(unittest.TestCase):
    def test_rendered_rgbd_recovers_visible_target_without_segmentation_ids(self) -> None:
        model = mujoco.MjModel.from_xml_string(TINY_RGBD_SCENE)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        frame = render_rgbd_frame(
            model,
            data,
            width=160,
            height=120,
            camera_name="fixture_camera",
            target_body_name=None,
        )
        detection = SimBlueShoeDetector(ignore_top_fraction=0.0).detect(frame.rgb)
        self.assertFalse(detection.uses_privileged_labels)

        reconstructed = estimate_shoe_from_frame(
            frame,
            detector=SimBlueShoeDetector(ignore_top_fraction=0.0),
        )
        target_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "blue_target"
        )
        target_world = np.asarray(data.xpos[target_id], dtype=np.float64)
        reconstruction_error = float(
            np.linalg.norm(
                np.asarray(reconstructed.position_target_m, dtype=np.float64)
                - target_world
            )
        )
        self.assertLess(reconstruction_error, 0.04)
        self.assertFalse(reconstructed.ground_truth_used)
        self.assertEqual(
            reconstructed.source,
            "rgb_detection_plus_aligned_metric_depth",
        )

    def test_mujoco_camera_axes_are_converted_to_ros_optical_axes(self) -> None:
        model = mujoco.MjModel.from_xml_string(TINY_RGBD_SCENE)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        rgb = np.zeros((3, 3, 3), dtype=np.uint8)
        depth = np.ones((3, 3), dtype=np.float32)

        frame = frame_from_rendered_rgbd(
            model,
            data,
            rgb=rgb,
            depth_m=depth,
            camera_name="fixture_camera",
            target_body_name=None,
        )
        optical_forward_world = frame.target_from_optical[:3, 2]
        optical_down_world = frame.target_from_optical[:3, 1]
        np.testing.assert_allclose(optical_forward_world, (0.0, 0.0, -1.0))
        np.testing.assert_allclose(optical_down_world, (0.0, -1.0, 0.0))

    def test_policy_observation_contains_sensor_target_not_object_truth(self) -> None:
        observation = build_vision_policy_observation(
            _synthetic_frame(),
            robot_state={"left_arm_rad": [0.0] * 5},
            detector=SimBlueShoeDetector(ignore_top_fraction=0.0),
        )
        self.assertFalse(observation["ground_truth"])
        self.assertTrue(observation["target_available"])
        self.assertNotIn("shoe", observation)
        self.assertFalse(observation["control_authorized"])
        self.assertFalse(observation["hardware_execution"])
        target = observation["target"]
        self.assertIsInstance(target, dict)
        self.assertFalse(target["ground_truth_used"])

    def test_sensor_target_drives_ik_and_emits_only_research_intent(self) -> None:
        frame = _synthetic_frame()
        start = (0.0,) * len(ACTION_NAMES)
        goal = tuple(0.01 * index for index in range(len(ACTION_NAMES)))
        ik = IKResult(
            action_rad=goal,
            converged=True,
            iterations=4,
            residual_m_by_side={"left": 0.0002},
            tool_axis_error_rad_by_side={},
            planning_qpos_writes=5,
        )
        with (
            patch(
                "vision_guided_reach.solve_bimanual_position_ik",
                return_value=ik,
            ) as solver,
            patch(
                "vision_guided_reach.check_bimanual_path",
                return_value=_safe_collision(),
            ),
        ):
            plan = plan_vision_guided_reach_from_frame(
                object(),
                frame,
                start,
                side="left",
                detector=SimBlueShoeDetector(ignore_top_fraction=0.0),
                approach_offset_target_m=(0.0, 0.0, 0.08),
                sequence=7,
            )

        self.assertTrue(plan.planning_accepted)
        self.assertFalse(plan.ground_truth_used_for_target)
        self.assertFalse(plan.hardware_execution)
        ik_targets = solver.call_args.args[2]
        np.testing.assert_allclose(ik_targets["left"], (0.0, 0.0, 1.08))
        self.assertIsNotNone(plan.control_intent)
        intent = plan.control_intent
        self.assertEqual(intent["kind"], "arm_joint_position")
        self.assertEqual(intent["sequence"], 7)
        self.assertEqual(intent["joint_names"], list(ACTION_NAMES))
        self.assertNotIn("hardware_authorized", intent)
        self.assertNotIn("device", intent)

    def test_detection_failure_produces_no_control_intent(self) -> None:
        frame = _synthetic_frame()
        detector = SimBlueShoeDetector(
            minimum_blue=250,
            ignore_top_fraction=0.0,
        )
        with self.assertRaises(VisionTargetError) as raised:
            plan_vision_guided_reach_from_frame(
                object(),
                frame,
                (0.0,) * len(ACTION_NAMES),
                detector=detector,
            )
        self.assertEqual(raised.exception.code, "detection_not_found")

    def test_invalid_planning_limits_fail_before_solver(self) -> None:
        frame = _synthetic_frame()
        with patch("vision_guided_reach.solve_bimanual_position_ik") as solver:
            with self.assertRaisesRegex(ValueError, "positive"):
                plan_vision_guided_reach_from_frame(
                    object(),
                    frame,
                    (0.0,) * len(ACTION_NAMES),
                    detector=SimBlueShoeDetector(ignore_top_fraction=0.0),
                    required_clearance_m=0.0,
                )
        solver.assert_not_called()


if __name__ == "__main__":
    unittest.main()
