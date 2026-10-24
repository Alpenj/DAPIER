import argparse
import math
import unittest

from shoe_sorting_data.mujoco_hardware_teleop import (
    ActiveArm,
    ArmEndpoint,
    CONFIRM_TEXT,
    _dispatch_arm,
    _poll_arm,
    _shutdown_arm,
    calibrated_ctrl_to_ticks,
    find_project_root,
    load_arm_calibration,
    relative_ctrl_to_ticks,
    validate_hardware_authorization,
)


def args(**overrides):
    values = {
        "hardware_dispatch_authorized": False,
        "confirm": None,
        "left_port": None,
        "left_expected_serial": None,
        "right_port": None,
        "right_expected_serial": None,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


class MujocoHardwareTeleopTest(unittest.TestCase):
    def test_all_six_controls_map_to_motor_ids_without_margin(self):
        start = {motor_id: 2048 for motor_id in range(1, 7)}
        limits = {motor_id: (0, 4095) for motor_id in range(1, 7)}
        controls = [math.radians(value) for value in (10, -20, 30, -40, 50, -60)]

        targets = relative_ctrl_to_ticks(controls, [0.0] * 6, start, limits)

        self.assertEqual(tuple(targets), (1, 2, 3, 4, 5, 6))
        self.assertEqual(targets[1], 2048 + round(10 * 4096 / 360))
        self.assertEqual(targets[6], 2048 + round(-60 * 4096 / 360))

    def test_stored_endpoints_are_allowed_and_outside_saturates(self):
        start = {motor_id: 2048 for motor_id in range(1, 7)}
        limits = {motor_id: (0, 4095) for motor_id in range(1, 7)}
        endpoint = (4095 - 2048) / (4096 / (2 * math.pi))
        controls = [0.0] * 6
        controls[2] = endpoint
        targets = relative_ctrl_to_ticks(controls, [0.0] * 6, start, limits)
        self.assertEqual(targets[3], 4095)

        controls[2] += math.radians(1)
        targets = relative_ctrl_to_ticks(controls, [0.0] * 6, start, limits)
        self.assertEqual(targets[3], 4095)

        controls[2] = math.radians(-1000)
        targets = relative_ctrl_to_ticks(controls, [0.0] * 6, start, limits)
        self.assertEqual(targets[3], 0)

    def test_hardware_authority_is_explicit_and_supports_both_arms(self):
        self.assertEqual(validate_hardware_authorization(args()), ())
        endpoints = validate_hardware_authorization(
            args(
                hardware_dispatch_authorized=True,
                confirm=CONFIRM_TEXT,
                left_port="/dev/serial/by-id/left",
                left_expected_serial="LEFT",
                right_port="/dev/serial/by-id/right",
                right_expected_serial="RIGHT",
            )
        )
        self.assertEqual([endpoint.side for endpoint in endpoints], ["left", "right"])
        self.assertEqual([endpoint.control_offset for endpoint in endpoints], [0, 6])

    def test_partial_duplicate_or_unconfirmed_endpoints_are_blocked(self):
        with self.assertRaisesRegex(ValueError, "confirm"):
            validate_hardware_authorization(args(hardware_dispatch_authorized=True))
        with self.assertRaisesRegex(ValueError, "provided together"):
            validate_hardware_authorization(
                args(
                    hardware_dispatch_authorized=True,
                    confirm=CONFIRM_TEXT,
                    left_port="/dev/serial/by-id/left",
                )
            )
        with self.assertRaisesRegex(ValueError, "different controller"):
            validate_hardware_authorization(
                args(
                    hardware_dispatch_authorized=True,
                    confirm=CONFIRM_TEXT,
                    left_port="/dev/serial/by-id/same",
                    left_expected_serial="LEFT",
                    right_port="/dev/serial/by-id/same",
                    right_expected_serial="RIGHT",
                )
            )

    def test_single_voltage_spike_is_retried(self):
        class VoltageRetryBus:
            def __init__(self):
                self.motor_6_reads = 0

            def read_state(self, motor_id):
                voltage = 12.2
                if motor_id == 6:
                    self.motor_6_reads += 1
                    if self.motor_6_reads == 1:
                        voltage = 10.9
                return {
                    "status": "ok",
                    "model_number_raw": 777,
                    "operating_mode_raw": 0,
                    "hardware_error_status": 0,
                    "voltage_volts": voltage,
                    "temperature_celsius": 35,
                    "current_milliamps": 0.0,
                    "load_fraction": 0.0,
                    "position_tick": 2048,
                    "minimum_position_limit_tick": 0,
                    "maximum_position_limit_tick": 4095,
                }

        bus = VoltageRetryBus()
        arm = ActiveArm(
            ArmEndpoint("right", "/dev/test", "SERIAL", 6),
            bus,
            {motor_id: 2048 for motor_id in range(1, 7)},
            {motor_id: (0, 4095) for motor_id in range(1, 7)},
            {motor_id: 2048 for motor_id in range(1, 7)},
        )
        _poll_arm(arm)
        self.assertEqual(bus.motor_6_reads, 2)

    def test_shutdown_retries_torque_off_verification(self):
        class ShutdownRetryBus:
            def __init__(self):
                self.disable_calls = 0
                self.closed = False

            def disable_all_torque(self, _motor_ids):
                self.disable_calls += 1

            def read_state(self, _motor_id):
                return {
                    "status": "ok",
                    "torque_enabled": self.disable_calls < 2,
                }

            def close(self):
                self.closed = True

        bus = ShutdownRetryBus()
        arm = ActiveArm(
            ArmEndpoint("right", "/dev/test", "SERIAL", 6),
            bus,
            {motor_id: 2048 for motor_id in range(1, 7)},
            {motor_id: (0, 4095) for motor_id in range(1, 7)},
            {motor_id: 2048 for motor_id in range(1, 7)},
        )
        self.assertTrue(_shutdown_arm(arm))
        self.assertEqual(bus.disable_calls, 2)
        self.assertTrue(bus.closed)

    def test_initial_dispatch_holds_all_six_joints(self):
        class MotionBus:
            def __init__(self):
                self.commands = []

            def start_motion(self, motor_id, **command):
                self.commands.append((motor_id, command))

        bus = MotionBus()
        start_ticks = {motor_id: 2000 + motor_id for motor_id in range(1, 7)}
        arm = ActiveArm(
            ArmEndpoint("right", "/dev/test", "SERIAL", 6),
            bus,
            start_ticks,
            {motor_id: (0, 4095) for motor_id in range(1, 7)},
            {motor_id: -1 for motor_id in range(1, 7)},
        )

        writes = _dispatch_arm(arm, [0.0] * 6, [0.0] * 6)

        self.assertEqual(writes, 6)
        self.assertEqual([motor_id for motor_id, _ in bus.commands], list(range(1, 7)))
        self.assertEqual(
            [command["target_tick"] for _, command in bus.commands],
            list(start_ticks.values()),
        )
        self.assertTrue(
            all(command["torque_limit_raw"] == 1000 for _, command in bus.commands)
        )

    def test_measured_calibration_maps_zero_and_gripper_endpoints(self):
        calibration = {
            "joint_zero_ticks": {str(i): 1000 + i * 100 for i in range(1, 6)},
            "joint_signs": {str(i): 1 for i in range(1, 6)},
            "gripper": {
                "open_tick": 1641,
                "closed_tick": 1465,
                "model_open_radians": 0.57,
                "model_closed_radians": -0.57,
            },
        }
        limits = {motor_id: (0, 4095) for motor_id in range(1, 7)}

        opened, _ = calibrated_ctrl_to_ticks(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.57],
            calibration,
            limits,
        )
        closed, _ = calibrated_ctrl_to_ticks(
            [0.0, 0.0, 0.0, 0.0, 0.0, -0.57],
            calibration,
            limits,
        )
        self.assertEqual([opened[i] for i in range(1, 6)], [1100, 1200, 1300, 1400, 1500])
        self.assertEqual(opened[6], 1641)
        self.assertEqual(closed[6], 1465)

    def test_right_calibration_matches_measured_controller_identity(self):
        root = find_project_root()
        calibration = load_arm_calibration(
            root,
            side="right",
            identity_sha256="dbaaa6edc7dd65e79a3f1b100abf7892478b990628d34f942964412634535803",
        )
        self.assertIsNotNone(calibration)
        self.assertEqual(calibration["joint_zero_ticks"]["2"], 1944)
        self.assertEqual(calibration["gripper"]["open_tick"], 1641)
        self.assertEqual(calibration["gripper"]["closed_tick"], 1465)


if __name__ == "__main__":
    unittest.main()
