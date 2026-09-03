from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest(
        "MuJoCo is not installed in this Python environment"
    ) from error

from mobile_dual_so101 import (
    HUMANOID_HOME_ACTION,
    TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
    apply_control_as_pose,
    build_model,
)
from physics_ik import (
    MotionLimits,
    execute_physics_trajectory,
    plan_septic_joint_trajectory,
    solve_bimanual_position_ik,
)


class PhysicsIKTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model, _ = build_model(
            arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
            mount_layout="tower",
        )
        data = mujoco.MjData(cls.model)
        apply_control_as_pose(cls.model, data, HUMANOID_HOME_ACTION)
        cls.home_positions = {
            side: data.site_xpos[
                mujoco.mj_name2id(
                    cls.model,
                    mujoco.mjtObj.mjOBJ_SITE,
                    f"{side}_gripperframe",
                )
            ].copy()
            for side in ("left", "right")
        }

    def solve_short_bimanual_move(self):
        targets = {
            side: position + np.asarray([0.010, 0.0, 0.010])
            for side, position in self.home_positions.items()
        }
        return solve_bimanual_position_ik(
            self.model,
            HUMANOID_HOME_ACTION,
            targets,
        )

    def test_bimanual_dls_ik_converges_without_runtime_state_writes(self) -> None:
        result = self.solve_short_bimanual_move()
        self.assertTrue(result.converged, result)
        self.assertLess(max(result.residual_m_by_side.values()), 5e-4)
        self.assertGreater(result.planning_qpos_writes, 0)
        self.assertEqual(result.runtime_qpos_writes, 0)
        self.assertFalse(result.hardware_execution)

    def test_septic_trajectory_has_zero_endpoint_velocity_acceleration_jerk(
        self,
    ) -> None:
        goal = np.asarray(HUMANOID_HOME_ACTION, dtype=float)
        goal[0] += 0.10
        limits = MotionLimits(
            max_velocity_rad_s=0.40,
            max_acceleration_rad_s2=1.0,
            max_jerk_rad_s3=5.0,
        )
        trajectory = plan_septic_joint_trajectory(
            self.model,
            HUMANOID_HOME_ACTION,
            goal,
            limits=limits,
        )
        endpoints = (
            (0.0, trajectory.start_rad),
            (trajectory.duration_s, goal),
        )
        for time_s, expected in endpoints:
            position, velocity, acceleration, jerk = trajectory.sample(time_s)
            np.testing.assert_allclose(position, expected, atol=1e-12)
            np.testing.assert_allclose(velocity, 0.0, atol=1e-12)
            np.testing.assert_allclose(acceleration, 0.0, atol=1e-12)
            np.testing.assert_allclose(jerk, 0.0, atol=1e-12)
        samples = [
            trajectory.sample(trajectory.duration_s * fraction / 1000.0)
            for fraction in range(1001)
        ]
        self.assertLessEqual(
            max(float(np.max(np.abs(sample[1]))) for sample in samples),
            limits.max_velocity_rad_s + 1e-9,
        )
        self.assertLessEqual(
            max(float(np.max(np.abs(sample[2]))) for sample in samples),
            limits.max_acceleration_rad_s2 + 1e-3,
        )
        self.assertLessEqual(
            max(float(np.max(np.abs(sample[3]))) for sample in samples),
            limits.max_jerk_rad_s3 + 1e-9,
        )

    def test_ik_goal_executes_through_actuator_physics(self) -> None:
        ik = self.solve_short_bimanual_move()
        self.assertTrue(ik.converged)
        limits = MotionLimits(
            max_velocity_rad_s=0.30,
            max_acceleration_rad_s2=0.80,
            max_jerk_rad_s3=4.0,
        )
        trajectory = plan_septic_joint_trajectory(
            self.model,
            HUMANOID_HOME_ACTION,
            ik.action_rad,
            limits=limits,
            minimum_duration_s=1.0,
        )
        _, report = execute_physics_trajectory(
            self.model,
            trajectory,
            settle_time_s=0.40,
        )
        self.assertTrue(report.finite_state)
        self.assertGreater(report.pre_settle_steps, 0)
        self.assertEqual(report.runtime_qpos_writes, 0)
        self.assertLess(report.final_tracking_error_rad, 0.02)
        self.assertLessEqual(
            report.maximum_target_velocity_rad_s,
            limits.max_velocity_rad_s + 1e-9,
        )
        self.assertLessEqual(
            report.maximum_target_acceleration_rad_s2,
            limits.max_acceleration_rad_s2 + 1e-3,
        )
        self.assertLessEqual(
            report.maximum_target_jerk_rad_s3,
            limits.max_jerk_rad_s3 + 1e-9,
        )
        self.assertLessEqual(
            report.maximum_actual_acceleration_rad_s2,
            limits.max_acceleration_rad_s2 + 1e-3,
        )
        self.assertGreater(
            report.maximum_actual_jerk_rad_s3,
            limits.max_jerk_rad_s3,
        )
        self.assertFalse(report.dynamics_limits_satisfied)
        self.assertEqual(report.dynamics_limit_violations, ("jerk",))
        self.assertFalse(report.simulation_motion_accepted)
        self.assertGreater(report.minimum_support_margin_m, 0.0)
        self.assertEqual(report.forbidden_contact_count, 0)
        self.assertFalse(report.hardware_execution)

    def test_invalid_motion_contract_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            MotionLimits(max_velocity_rad_s=0.0).validate()
        with self.assertRaisesRegex(ValueError, "left and/or right"):
            solve_bimanual_position_ik(
                self.model,
                HUMANOID_HOME_ACTION,
                {},
            )
        with self.assertRaisesRegex(ValueError, "same sides"):
            solve_bimanual_position_ik(
                self.model,
                HUMANOID_HOME_ACTION,
                {"left": self.home_positions["left"]},
                site_names={"right": "right_gripperframe"},
            )


if __name__ == "__main__":
    unittest.main()
