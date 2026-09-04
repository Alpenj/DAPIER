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
    SepticJointTrajectory,
    _forbidden_contact_count,
    execute_physics_trajectory,
    plan_septic_joint_trajectory,
    solve_bimanual_position_ik,
)


class PhysicsIKTest(unittest.TestCase):
    def test_runtime_same_arm_non_adjacent_contact_is_forbidden(self) -> None:
        model = mujoco.MjModel.from_xml_string(
            """
            <mujoco>
              <option gravity="0 0 0"/>
              <worldbody>
                <body name="left_shoulder">
                  <joint type="hinge" axis="1 0 0"/>
                  <geom name="left_shoulder_collision" type="sphere" size=".05"/>
                  <body name="left_upper_arm">
                    <joint type="hinge" axis="0 1 0"/>
                    <geom name="left_upper_arm_collision" type="sphere" size=".05"/>
                    <body name="left_wrist">
                      <joint type="hinge" axis="0 0 1"/>
                      <geom name="left_wrist_collision" type="sphere" size=".05"/>
                    </body>
                  </body>
                </body>
              </worldbody>
            </mujoco>
            """
        )
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        self.assertEqual(_forbidden_contact_count(model, data), 1)

    def test_runtime_contacts_with_base_and_step_support_are_forbidden(self) -> None:
        model = mujoco.MjModel.from_xml_string(
            """
            <mujoco>
              <option gravity="0 0 0"/>
              <worldbody>
                <body name="tb3_base_link">
                  <geom name="tb3_base_link_collision" type="box" size=".1 .1 .1"/>
                </body>
                <body name="tower_mount_structure">
                  <geom name="semi_support_base_collision" type="box" size=".1 .1 .1"/>
                  <geom name="semi_support_column_collision" type="box" size=".1 .1 .1"/>
                </body>
                <body name="left_wrist">
                  <freejoint/>
                  <geom name="left_wrist_collision" type="sphere" size=".05"/>
                </body>
              </worldbody>
            </mujoco>
            """
        )
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        self.assertEqual(_forbidden_contact_count(model, data), 3)

    def test_runtime_arm_floor_contact_is_forbidden(self) -> None:
        model = mujoco.MjModel.from_xml_string(
            """
            <mujoco>
              <worldbody>
                <geom name="floor" type="plane" size="0 0 .1"/>
                <body name="left_wrist" pos="0 0 .04">
                  <freejoint/>
                  <geom name="left_wrist_collision" type="sphere" size=".05"/>
                </body>
              </worldbody>
            </mujoco>
            """
        )
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        self.assertEqual(_forbidden_contact_count(model, data), 1)

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
            side: position + np.asarray([-0.005, 0.0, 0.0])
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

    def test_ik_can_hold_a_downward_tool_axis(self) -> None:
        target = self.home_positions["left"] + np.asarray([0.005, 0.0, -0.005])
        result = solve_bimanual_position_ik(
            self.model,
            HUMANOID_HOME_ACTION,
            {"left": target},
            tool_axis_targets={"left": (0.0, 0.0, -1.0)},
            max_iterations=250,
        )

        self.assertTrue(result.converged, result)
        self.assertLess(result.residual_m_by_side["left"], 5e-4)
        self.assertLess(result.tool_axis_error_rad_by_side["left"], np.deg2rad(2.0))

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
        self.assertGreater(
            report.maximum_actual_acceleration_rad_s2,
            limits.max_acceleration_rad_s2,
        )
        self.assertGreater(
            report.maximum_actual_jerk_rad_s3,
            limits.max_jerk_rad_s3,
        )
        self.assertFalse(report.dynamics_limits_satisfied)
        self.assertEqual(
            report.dynamics_limit_violations, ("acceleration", "jerk")
        )
        self.assertFalse(report.simulation_motion_accepted)
        self.assertGreater(report.minimum_support_margin_m, 0.0)
        self.assertEqual(report.forbidden_contact_count, 0)
        self.assertFalse(report.hardware_execution)

    def test_pre_settle_transients_are_in_all_runtime_safety_evidence(self) -> None:
        limits = MotionLimits(
            max_velocity_rad_s=0.30,
            max_acceleration_rad_s2=0.80,
            max_jerk_rad_s3=4.0,
        )
        trajectory = plan_septic_joint_trajectory(
            self.model,
            HUMANOID_HOME_ACTION,
            HUMANOID_HOME_ACTION,
            limits=limits,
            minimum_duration_s=float(self.model.opt.timestep),
        )
        original_step = mujoco.mj_step
        joint_ids = self.model.actuator_trnid[:, 0]
        qpos_addresses = self.model.jnt_qposadr[joint_ids]
        dof_addresses = self.model.jnt_dofadr[joint_ids]
        unactuated_qpos = next(
            index
            for index in range(self.model.nq)
            if index not in set(qpos_addresses)
        )
        clean_state: dict[str, np.ndarray] = {}
        calls = 0

        def step_with_first_interval_hazard(model, data) -> None:
            nonlocal calls
            if clean_state:
                data.qpos[:] = clean_state["qpos"]
                data.qvel[:] = clean_state["qvel"]
                data.qacc[:] = clean_state["qacc"]
            original_step(model, data)
            if calls == 0:
                clean_state.update(
                    qpos=data.qpos.copy(),
                    qvel=data.qvel.copy(),
                    qacc=data.qacc.copy(),
                )
                data.qpos[qpos_addresses[0]] += 0.20
                data.qpos[unactuated_qpos] = np.inf
                data.qvel[dof_addresses[0]] = limits.max_velocity_rad_s * 2.0
                data.qacc[dof_addresses[0]] = limits.max_acceleration_rad_s2 * 2.0
                data.actuator_force[0] = 2.0 * max(
                    abs(value) for value in model.actuator_forcerange[0]
                )
            else:
                data.qpos[qpos_addresses] = data.ctrl
                data.qvel[dof_addresses] = 0.0
                data.qacc[dof_addresses] = 0.0
                data.actuator_force[:] = 0.0
            calls += 1

        timestep = float(self.model.opt.timestep)
        with (
            patch("physics_ik.mujoco.mj_step", side_effect=step_with_first_interval_hazard),
            patch(
                "physics_ik._combined_center_of_mass",
                side_effect=lambda _model, data: np.asarray(
                    [1.0, 1.0, 0.0]
                    if data.time <= timestep
                    else [-0.1, 0.0, 0.0]
                ),
            ),
            patch(
                "physics_ik._forbidden_contact_count",
                side_effect=lambda _model, data: int(data.time <= timestep),
            ),
        ):
            _, report = execute_physics_trajectory(
                self.model,
                trajectory,
                pre_settle_time_s=timestep,
                settle_time_s=0.0,
            )

        self.assertGreater(report.maximum_tracking_error_rad, 0.10)
        self.assertGreater(report.maximum_actuator_force_ratio, 1.0)
        self.assertLess(report.minimum_support_margin_m, 0.0)
        self.assertEqual(report.forbidden_contact_count, 1)
        self.assertFalse(report.finite_state)
        self.assertIn("velocity", report.dynamics_limit_violations)
        self.assertIn("acceleration", report.dynamics_limit_violations)
        self.assertFalse(report.simulation_motion_accepted)

    def test_physics_trajectory_can_continue_from_previous_leg(self) -> None:
        peak = np.asarray(HUMANOID_HOME_ACTION, dtype=float)
        peak[0] += 0.02
        outbound = plan_septic_joint_trajectory(
            self.model, HUMANOID_HOME_ACTION, peak
        )
        data, _ = execute_physics_trajectory(self.model, outbound)
        inbound = plan_septic_joint_trajectory(
            self.model, peak, HUMANOID_HOME_ACTION
        )

        final_data, report = execute_physics_trajectory(
            self.model,
            inbound,
            initial_data=data,
            pre_settle_time_s=0.0,
        )

        self.assertIs(final_data, data)
        self.assertEqual(report.runtime_qpos_writes, 0)
        self.assertLess(report.final_tracking_error_rad, 0.001)

    def test_acceptance_gates_tracking_error_limits(self) -> None:
        limits = MotionLimits(
            max_velocity_rad_s=100.0,
            max_acceleration_rad_s2=100_000.0,
            max_jerk_rad_s3=100_000_000.0,
            max_tracking_error_rad=0.001,
            max_final_tracking_error_rad=0.001,
        )
        timestep = float(self.model.opt.timestep)
        trajectory = plan_septic_joint_trajectory(
            self.model,
            HUMANOID_HOME_ACTION,
            HUMANOID_HOME_ACTION,
            limits=limits,
            minimum_duration_s=timestep,
        )
        real_step = mujoco.mj_step
        joint_ids = self.model.actuator_trnid[:, 0]
        qpos_addresses = self.model.jnt_qposadr[joint_ids]

        def step_with_tracking_error(model, data) -> None:
            real_step(model, data)
            data.qpos[qpos_addresses] = data.ctrl
            data.qpos[qpos_addresses[0]] += 0.01

        with patch("physics_ik.mujoco.mj_step", side_effect=step_with_tracking_error):
            _, report = execute_physics_trajectory(
                self.model,
                trajectory,
                pre_settle_time_s=0.0,
                settle_time_s=0.0,
            )

        self.assertIn("tracking_error", report.dynamics_limit_violations)
        self.assertIn("final_tracking_error", report.dynamics_limit_violations)
        self.assertFalse(report.simulation_motion_accepted)

    def test_acceptance_gates_target_rate_limits(self) -> None:
        limits = MotionLimits(
            max_velocity_rad_s=0.1,
            max_acceleration_rad_s2=0.2,
            max_jerk_rad_s3=0.3,
        )
        goal = np.asarray(HUMANOID_HOME_ACTION, dtype=float)
        goal[1] += 0.01
        trajectory = SepticJointTrajectory(
            np.asarray(HUMANOID_HOME_ACTION, dtype=float),
            goal,
            0.01,
            limits,
        )

        _, report = execute_physics_trajectory(
            self.model,
            trajectory,
            pre_settle_time_s=0.0,
            settle_time_s=0.0,
        )

        self.assertIn("target_velocity", report.dynamics_limit_violations)
        self.assertIn("target_acceleration", report.dynamics_limit_violations)
        self.assertIn("target_jerk", report.dynamics_limit_violations)
        self.assertFalse(report.simulation_motion_accepted)

    def test_invalid_motion_contract_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            MotionLimits(max_velocity_rad_s=0.0).validate()
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            MotionLimits(max_tracking_error_rad=0.0).validate()
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            MotionLimits(max_final_tracking_error_rad=0.0).validate()
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
