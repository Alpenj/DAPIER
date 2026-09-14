from __future__ import annotations

from contextlib import redirect_stderr
from io import StringIO
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

import mobile_dual_so101
from collision_guard import (
    DEFAULT_MAX_JOINT_STEP_RAD,
    CollisionAssessment,
)
from mobile_dual_so101 import (
    ACTION_NAMES,
    HUMANOID_HOME_ACTION,
    apply_control_as_pose,
    build_model,
)
from sim_teleop import (
    KEY_DECREASE,
    KEY_INCREASE,
    KEY_LEFT_ARM,
    KEY_LEFT_ARROW,
    KEY_RIGHT_ARROW,
    KEY_RIGHT_ARM,
    KEY_STOP,
    KEY_UP_ARROW,
    KEYPAD_1,
    SimTeleopController,
    apply_teleop_targets,
    run_sim_teleop,
    synchronize_stop_hold,
    synchronize_control_panel,
)


class SimTeleopControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model, _ = build_model(arm_mount_height_m=0.30)

    def setUp(self) -> None:
        self.controller = SimTeleopController(
            self.model,
            initial_action=HUMANOID_HOME_ACTION,
            step_rad=math.radians(2.0),
            min_key_interval_s=0.05,
        )
        first_joint_id = int(self.model.actuator_trnid[0, 0])
        self.first_actuated_qpos_address = int(
            self.model.jnt_qposadr[first_joint_id]
        )

    @staticmethod
    def safe_assessment() -> CollisionAssessment:
        return CollisionAssessment(
            safe=True,
            reason="protected clearance satisfied",
            minimum_clearance_m=0.03,
            required_clearance_m=0.03,
            path_fraction=1.0,
            checked_samples=2,
            first_body="left_arm",
            second_body="right_arm",
            first_geom_id=1,
            second_geom_id=2,
        )

    def test_initial_status_is_simulation_only_and_deterministic(self) -> None:
        status = self.controller.status()
        self.assertEqual(status["action_order"], ACTION_NAMES)
        self.assertEqual(status["selected_arm"], "left")
        self.assertEqual(status["selected_joint"], "shoulder_pan")
        self.assertEqual(status["targets_rad"], HUMANOID_HOME_ACTION)
        self.assertFalse(status["stopped"])
        self.assertFalse(status["published"])
        self.assertFalse(status["hardware_dispatch_authorized"])
        self.assertFalse(status["hardware_execution"])

    def test_recording_rate_rejects_invalid_values_before_viewer_launch(self) -> None:
        for value in (0, -1, True, 20.0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                run_sim_teleop(
                    self.model,
                    initial_action=HUMANOID_HOME_ACTION,
                    record_fps=value,
                )

    def test_arm_joint_selection_increment_and_rate_limit(self) -> None:
        self.assertTrue(self.controller.handle_key(KEY_RIGHT_ARM, now_ns=1).accepted)
        self.assertTrue(self.controller.handle_key(ord("2"), now_ns=2).accepted)
        before = self.controller.targets
        with patch(
            "sim_teleop.check_bimanual_path",
            return_value=self.safe_assessment(),
        ):
            update = self.controller.handle_key(KEY_INCREASE, now_ns=1_000_000_000)
        self.assertTrue(update.accepted)
        self.assertEqual(update.action_name, "right_shoulder_lift")
        self.assertAlmostEqual(
            self.controller.targets[7],
            before[7] + math.radians(2.0),
        )

        repeated = self.controller.handle_key(
            KEY_INCREASE,
            now_ns=1_000_000_001,
        )
        self.assertFalse(repeated.accepted)
        self.assertIn("rate limit", repeated.reason)
        self.assertEqual(self.controller.targets, update.targets)

    def test_named_obstacles_reach_shared_collision_guard(self) -> None:
        controller = SimTeleopController(
            self.model,
            initial_action=HUMANOID_HOME_ACTION,
            obstacle_geom_names=("box_floor",),
        )
        with patch(
            "sim_teleop.check_bimanual_path",
            return_value=self.safe_assessment(),
        ) as guard:
            self.assertTrue(controller.handle_key(KEY_INCREASE, now_ns=1).accepted)
        self.assertEqual(
            guard.call_args.kwargs["obstacle_geom_names"], ("box_floor",)
        )

    def test_glfw_keypad_and_arrow_fallbacks_drive_the_same_contract(self) -> None:
        self.assertTrue(
            self.controller.handle_key(KEY_RIGHT_ARROW, now_ns=1).accepted
        )
        self.assertTrue(
            self.controller.handle_key(KEYPAD_1 + 1, now_ns=2).accepted
        )
        before = self.controller.targets
        with patch(
            "sim_teleop.check_bimanual_path",
            return_value=self.safe_assessment(),
        ):
            update = self.controller.handle_key(KEY_UP_ARROW, now_ns=1_000_000_000)
        self.assertTrue(update.accepted)
        self.assertEqual(update.action_name, "right_shoulder_lift")
        self.assertGreater(self.controller.targets[7], before[7])
        self.assertNotEqual(KEY_LEFT_ARROW, KEY_RIGHT_ARROW)

    def test_default_repeat_accumulates_without_legacy_key_overlap(self) -> None:
        controller = SimTeleopController(
            self.model,
            initial_action=HUMANOID_HOME_ACTION,
        )
        safe = CollisionAssessment(
            safe=True,
            reason="protected clearance satisfied",
            minimum_clearance_m=0.03,
            required_clearance_m=0.03,
            path_fraction=1.0,
            checked_samples=2,
            first_body="left_arm",
            second_body="right_arm",
            first_geom_id=1,
            second_geom_id=2,
        )
        before = controller.targets
        with patch("sim_teleop.check_bimanual_path", return_value=safe):
            first = controller.handle_key(KEY_INCREASE, now_ns=1)
            repeated = controller.handle_key(KEY_INCREASE, now_ns=2)
        self.assertTrue(first.accepted)
        self.assertTrue(repeated.accepted)
        self.assertAlmostEqual(
            controller.targets[0], before[0] + math.radians(10.0)
        )
        for legacy_key in (ord("L"), ord("R"), ord("="), ord("-")):
            with self.subTest(legacy_key=legacy_key):
                self.assertFalse(
                    controller.handle_key(legacy_key, now_ns=3).accepted
                )

    def test_smooth_slew_uses_fixed_collision_sampling(self) -> None:
        controller = SimTeleopController(
            self.model,
            initial_action=HUMANOID_HOME_ACTION,
            step_rad=math.radians(10.0),
            max_speed_rad_s=math.radians(45.0),
            accel_rad_s2=math.radians(180.0),
        )
        safe = CollisionAssessment(
            safe=True,
            reason="protected clearance satisfied",
            minimum_clearance_m=0.03,
            required_clearance_m=0.03,
            path_fraction=1.0,
            checked_samples=2,
            first_body="left_arm",
            second_body="right_arm",
            first_geom_id=1,
            second_geom_id=2,
        )
        before = controller.control_targets
        with patch("sim_teleop.check_bimanual_path", return_value=safe) as guard:
            self.assertTrue(controller.handle_key(KEY_INCREASE, now_ns=1).accepted)
            desired = controller.targets
            first_control = controller.advance(0.02)
            for _ in range(199):
                final_control = controller.advance(0.02)
        self.assertGreater(first_control[0], before[0])
        self.assertLess(first_control[0], desired[0])
        self.assertAlmostEqual(final_control[0], desired[0], places=7)
        self.assertTrue(
            all(
                call.kwargs["max_joint_step_rad"] == DEFAULT_MAX_JOINT_STEP_RAD
                for call in guard.call_args_list
            )
        )

    def test_slew_reversal_stays_inside_actuator_range(self) -> None:
        controller = SimTeleopController(
            self.model,
            initial_action=HUMANOID_HOME_ACTION,
            step_rad=math.radians(5.0),
            max_speed_rad_s=math.radians(45.0),
            accel_rad_s2=math.radians(180.0),
        )
        safe = CollisionAssessment(
            safe=True,
            reason="protected clearance satisfied",
            minimum_clearance_m=0.03,
            required_clearance_m=0.03,
            path_fraction=1.0,
            checked_samples=2,
            first_body="left_arm",
            second_body="right_arm",
            first_geom_id=1,
            second_geom_id=2,
        )
        self.assertTrue(controller.handle_key(ord("2"), now_ns=1).accepted)
        lower, upper = self.model.actuator_ctrlrange[1]
        with patch("sim_teleop.check_bimanual_path", return_value=safe) as guard:
            for index in range(4):
                controller.handle_key(KEY_DECREASE, now_ns=2 + index)
            for _ in range(200):
                moving_down = controller.advance(0.02)
                if moving_down[1] < float(lower) + 0.005:
                    break
            self.assertLess(moving_down[1], float(lower) + 0.005)
            for index in range(10):
                controller.handle_key(KEY_INCREASE, now_ns=10 + index)
            observed = []
            for _ in range(40):
                observed.append(controller.advance(0.02))
        self.assertTrue(
            all(
                float(lower) <= control[1] <= float(upper)
                for control in observed
            )
        )
        for call in guard.call_args_list:
            candidate = call.args[2]
            self.assertGreaterEqual(candidate[1], float(lower))
            self.assertLessEqual(candidate[1], float(upper))

    def test_control_panel_and_keyboard_share_safe_targets(self) -> None:
        controller = SimTeleopController(
            self.model,
            initial_action=HUMANOID_HOME_ACTION,
        )
        data = mujoco.MjData(self.model)
        apply_control_as_pose(self.model, data, HUMANOID_HOME_ACTION)
        before_qpos = tuple(data.qpos)
        panel_targets = list(HUMANOID_HOME_ACTION)
        panel_targets[3] += 0.05
        data.ctrl[:] = panel_targets
        safe = CollisionAssessment(
            safe=True,
            reason="protected clearance satisfied",
            minimum_clearance_m=0.03,
            required_clearance_m=0.03,
            path_fraction=1.0,
            checked_samples=2,
            first_body="left_arm",
            second_body="right_arm",
            first_geom_id=1,
            second_geom_id=2,
        )
        with patch("sim_teleop.check_bimanual_path", return_value=safe):
            panel_update = synchronize_control_panel(
                controller,
                data,
                HUMANOID_HOME_ACTION,
                now_ns=1,
            )
            self.assertIsNotNone(panel_update)
            self.assertTrue(panel_update.accepted)
            self.assertEqual(panel_update.action_name, "control_panel")
            self.assertEqual(controller.targets, tuple(panel_targets))
            self.assertEqual(controller.control_targets, tuple(panel_targets))

            self.assertTrue(controller.handle_key(ord("4"), now_ns=2).accepted)
            keyboard_update = controller.handle_key(KEY_INCREASE, now_ns=3)
            self.assertTrue(keyboard_update.accepted)
            self.assertGreater(controller.targets[3], panel_targets[3])
            self.assertEqual(controller.control_targets, tuple(panel_targets))
            smoothed = controller.advance(0.02)

        self.assertEqual(tuple(data.qpos), before_qpos)
        self.assertGreater(smoothed[3], panel_targets[3])
        self.assertLess(smoothed[3], controller.targets[3])

        unsafe = CollisionAssessment(
            safe=False,
            reason="protected clearance violated; target rejected",
            minimum_clearance_m=0.01,
            required_clearance_m=0.03,
            path_fraction=0.5,
            checked_samples=2,
            first_body="left_gripper",
            second_body="tower_mount_structure",
            first_geom_id=1,
            second_geom_id=2,
        )
        safe_targets = controller.control_targets
        unsafe_targets = list(safe_targets)
        unsafe_targets[3] += 0.05
        data.ctrl[:] = unsafe_targets
        with patch("sim_teleop.check_bimanual_path", return_value=unsafe):
            rejected = synchronize_control_panel(
                controller,
                data,
                safe_targets,
                now_ns=4,
            )
        self.assertIsNotNone(rejected)
        self.assertFalse(rejected.accepted)
        self.assertEqual(controller.control_targets, safe_targets)

    def test_stop_holds_current_pose_and_blocks_motion_until_resume(self) -> None:
        stopped = self.controller.handle_key(KEY_STOP, now_ns=1)
        self.assertTrue(stopped.accepted)
        self.assertTrue(stopped.stopped)
        blocked = self.controller.handle_key(KEY_DECREASE, now_ns=1_000_000_000)
        self.assertFalse(blocked.accepted)
        self.assertIn("stopped", blocked.reason)

        held = list(HUMANOID_HOME_ACTION)
        held[0] += 0.01
        self.controller.hold_current(held)
        self.assertEqual(self.controller.targets, tuple(held))
        resumed = self.controller.handle_key(KEY_STOP, now_ns=2_000_000_000)
        self.assertTrue(resumed.accepted)
        self.assertFalse(resumed.stopped)
        self.assertEqual(self.controller.targets, tuple(held))

    def test_stop_hold_is_latched_once_instead_of_following_drift(self) -> None:
        data = mujoco.MjData(self.model)
        apply_control_as_pose(self.model, data, HUMANOID_HOME_ACTION)
        self.controller.handle_key(KEY_STOP, now_ns=1)
        data.qpos[self.first_actuated_qpos_address] += 0.01
        mujoco.mj_forward(self.model, data)

        stopped = synchronize_stop_hold(
            self.controller,
            self.model,
            data,
            was_stopped=False,
        )
        self.assertTrue(stopped)
        latched = self.controller.targets
        data.qpos[self.first_actuated_qpos_address] += 0.02
        mujoco.mj_forward(self.model, data)

        stopped = synchronize_stop_hold(
            self.controller,
            self.model,
            data,
            was_stopped=stopped,
        )
        self.assertTrue(stopped)
        self.assertEqual(self.controller.targets, latched)

    def test_collision_guard_rejection_does_not_change_targets(self) -> None:
        unsafe = CollisionAssessment(
            safe=False,
            reason="protected clearance violated; target rejected",
            minimum_clearance_m=0.01,
            required_clearance_m=0.03,
            path_fraction=0.5,
            checked_samples=2,
            first_body="left_gripper",
            second_body="right_gripper",
            first_geom_id=1,
            second_geom_id=2,
        )
        before = self.controller.targets
        with patch("sim_teleop.check_bimanual_path", return_value=unsafe):
            update = self.controller.handle_key(
                KEY_INCREASE,
                now_ns=1_000_000_000,
            )
        self.assertFalse(update.accepted)
        self.assertIn("clearance", update.reason)
        self.assertEqual(self.controller.targets, before)
        self.assertFalse(update.hardware_execution)

    def test_targets_feed_actuator_physics_without_teleporting_qpos(self) -> None:
        data = mujoco.MjData(self.model)
        apply_control_as_pose(self.model, data, HUMANOID_HOME_ACTION)
        before = data.qpos.copy()
        with patch(
            "sim_teleop.check_bimanual_path",
            return_value=self.safe_assessment(),
        ):
            update = self.controller.handle_key(
                KEY_INCREASE,
                now_ns=1_000_000_000,
            )
        self.assertTrue(update.accepted)
        control_targets = self.controller.advance(0.02)
        apply_teleop_targets(self.model, data, control_targets)
        self.assertEqual(tuple(data.qpos), tuple(before))
        mujoco.mj_step(self.model, data)
        qpos_address = self.first_actuated_qpos_address
        self.assertNotEqual(
            float(data.qpos[qpos_address]),
            float(before[qpos_address]),
        )
        self.assertTrue(all(math.isfinite(float(value)) for value in data.qpos))

    def test_main_teleop_flag_calls_sim_runner_with_bounded_settings(self) -> None:
        with (
            patch("mobile_dual_so101.validate_model", return_value={"hardware_execution": False}),
            patch("sim_teleop.run_sim_teleop") as run_sim_teleop,
        ):
            result = mobile_dual_so101.main(
                [
                    "--arm-mount-height-m",
                    "0.30",
                    "--smoke-steps",
                    "0",
                    "--teleop",
                    "--teleop-step-deg",
                    "1.5",
                    "--teleop-min-key-interval-ms",
                    "75",
                    "--teleop-clearance-m",
                    "0.04",
                    "--teleop-max-speed-deg-s",
                    "60",
                    "--teleop-accel-deg-s2",
                    "240",
                ]
            )
        self.assertEqual(result, 0)
        run_sim_teleop.assert_called_once()
        kwargs = run_sim_teleop.call_args.kwargs
        self.assertAlmostEqual(kwargs["step_rad"], math.radians(1.5))
        self.assertEqual(kwargs["min_key_interval_s"], 0.075)
        self.assertEqual(kwargs["required_clearance_m"], 0.04)

        self.assertAlmostEqual(kwargs["max_speed_rad_s"], math.radians(60.0))
        self.assertAlmostEqual(kwargs["accel_rad_s2"], math.radians(240.0))

    def test_main_rejects_conflicting_modes_and_invalid_limits(self) -> None:
        invalid_arguments = (
            ("--viewer", "--teleop"),
            ("--teleop", "--teleop-step-deg", "0"),
            ("--teleop", "--teleop-min-key-interval-ms", "-1"),
            ("--teleop", "--teleop-clearance-m", "0"),
            ("--teleop", "--teleop-max-speed-deg-s", "0"),
            ("--teleop", "--teleop-accel-deg-s2", "0"),
        )
        for extra_arguments in invalid_arguments:
            with self.subTest(arguments=extra_arguments):
                with redirect_stderr(StringIO()):
                    with self.assertRaises(SystemExit):
                        mobile_dual_so101.main(
                            ["--arm-mount-height-m", "0.30", *extra_arguments]
                        )


if __name__ == "__main__":
    unittest.main()
