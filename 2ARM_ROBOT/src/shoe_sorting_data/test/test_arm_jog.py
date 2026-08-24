import unittest

from shoe_sorting_data.arm_jog import (
    JogPolicy,
    build_guarded_motion_packet,
    build_torque_disable_packet,
    confirm_temperature_sample,
    execute_guarded_jog,
    execute_joint_full_range,
    validate_preflight,
)


def motor_state(position=2048, **overrides):
    state = {
        "status": "ok",
        "model_number_raw": 777,
        "minimum_position_limit_tick": 0,
        "maximum_position_limit_tick": 4095,
        "position_offset_tick": 85,
        "operating_mode_raw": 0,
        "torque_enabled": False,
        "moving": False,
        "position_tick": position,
        "voltage_volts": 12.2,
        "temperature_celsius": 30,
        "hardware_error_status": 0,
        "current_milliamps": 0.0,
        "load_fraction": 0.0,
    }
    state.update(overrides)
    return state


class FakeBus:
    def __init__(self, *, runtime_error=False):
        self.states = {motor_id: motor_state() for motor_id in range(1, 7)}
        self.runtime_error = runtime_error
        self.motion_calls = []
        self.disable_calls = 0

    def read_state(self, motor_id):
        state = dict(self.states[motor_id])
        if self.runtime_error and self.motion_calls and motor_id == self.motion_calls[0][0]:
            state["hardware_error_status"] = 1
        return state

    def start_motion(self, motor_id, **command):
        self.motion_calls.append((motor_id, command))
        self.states[motor_id]["torque_enabled"] = True
        self.states[motor_id]["position_tick"] = command["target_tick"]

    def disable_all_torque(self, motor_ids):
        self.disable_calls += 1
        self.runtime_error = False
        for motor_id in motor_ids:
            self.states[motor_id]["torque_enabled"] = False
            self.states[motor_id]["moving"] = False


class ArmJogTest(unittest.TestCase):
    def test_packets_are_limited_to_guarded_ram_contract(self):
        disable = build_torque_disable_packet(1)
        self.assertEqual(disable[4], 0x03)
        self.assertEqual(disable[5:7], bytes([40, 0]))

        motion = build_guarded_motion_packet(
            1,
            target_tick=2056,
            speed_raw=30,
            acceleration_raw=10,
            torque_limit_raw=100,
        )
        self.assertEqual(motion[4], 0x03)
        self.assertEqual(motion[5], 40)
        self.assertEqual(motion[6:16], bytes([1, 10, 8, 8, 0, 0, 30, 0, 100, 0]))

    def test_preflight_requires_all_six_torque_off_and_guarded_target(self):
        states = {motor_id: motor_state() for motor_id in range(1, 7)}
        self.assertEqual(
            validate_preflight(states, target_motor_id=1, delta_ticks=8, policy=JogPolicy()),
            2056,
        )
        states[2]["torque_enabled"] = True
        with self.assertRaisesRegex(ValueError, "torque is already enabled"):
            validate_preflight(states, target_motor_id=1, delta_ticks=8, policy=JogPolicy())

    def test_preflight_rejects_stored_and_guarded_limit_violations(self):
        states = {motor_id: motor_state() for motor_id in range(1, 7)}
        states[1] = motor_state(position=20)
        with self.assertRaisesRegex(ValueError, "guarded limits"):
            validate_preflight(states, target_motor_id=1, delta_ticks=8, policy=JogPolicy())

        states[1] = motor_state(
            position=800,
            minimum_position_limit_tick=900,
            maximum_position_limit_tick=3300,
        )
        with self.assertRaisesRegex(ValueError, "outside stored limits"):
            validate_preflight(states, target_motor_id=1, delta_ticks=8, policy=JogPolicy())

    def test_success_reaches_target_and_disables_every_motor(self):
        bus = FakeBus()
        result = execute_guarded_jog(bus, sleep=lambda _seconds: None)

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["motion_command_sent"])
        self.assertTrue(result["target_reached"])
        self.assertTrue(result["torque_off_verified"])
        self.assertEqual(bus.motion_calls[0][1]["target_tick"], 2056)
        self.assertEqual(bus.disable_calls, 1)
        self.assertTrue(all(not state["torque_enabled"] for state in bus.states.values()))

    def test_runtime_fault_fails_and_still_disables_every_motor(self):
        bus = FakeBus(runtime_error=True)
        result = execute_guarded_jog(bus, sleep=lambda _seconds: None)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("hardware error", result["failure"])
        self.assertTrue(result["torque_off_verified"])
        self.assertEqual(bus.disable_calls, 1)

    def test_all_six_motors_use_full_range_without_artificial_delta_cap(self):
        for motor_id in range(1, 7):
            with self.subTest(motor_id=motor_id):
                bus = FakeBus()
                result = execute_joint_full_range(
                    bus,
                    motor_id=motor_id,
                    delta_degrees=40.0,
                    sleep=lambda _seconds: None,
                )

                self.assertEqual(result["status"], "PASS")
                self.assertEqual(result["profile"], "joint-full-range")
                self.assertIsNone(result["artificial_delta_limit"])
                self.assertEqual(result["position_margin_ticks"], 0)
                self.assertEqual(bus.motion_calls[0][0], motor_id)
                self.assertEqual(
                    bus.motion_calls[0][1],
                    {
                        "target_tick": 2503,
                        "speed_raw": 120,
                        "acceleration_raw": 40,
                        "torque_limit_raw": 1000,
                    },
                )
                self.assertTrue(result["torque_off_verified"])

    def test_full_range_accepts_reverse_and_stored_range_endpoint(self):
        bus = FakeBus()
        result = execute_joint_full_range(
            bus,
            motor_id=6,
            delta_degrees=-40.0,
            sleep=lambda _seconds: None,
        )
        self.assertEqual(bus.motion_calls[0][0], 6)
        self.assertEqual(bus.motion_calls[0][1]["target_tick"], 1593)
        self.assertLess(result["commanded_degrees"], 0)

        endpoint_bus = FakeBus()
        endpoint_bus.states[1]["position_tick"] = 2048
        endpoint_degrees = (4095 - 2048) * 360.0 / 4096.0
        endpoint = execute_joint_full_range(
            endpoint_bus,
            motor_id=1,
            delta_degrees=endpoint_degrees,
            sleep=lambda _seconds: None,
        )
        self.assertEqual(endpoint["status"], "PASS")
        self.assertEqual(endpoint_bus.motion_calls[0][1]["target_tick"], 4095)

        blocked_bus = FakeBus()
        blocked = execute_joint_full_range(
            blocked_bus,
            motor_id=1,
            delta_degrees=1000.0,
        )
        self.assertEqual(blocked["status"], "BLOCKED")
        self.assertIn("guarded limits", blocked["failure"])
        self.assertEqual(blocked_bus.motion_calls, [])

    def test_full_range_rejects_non_finite_or_sub_tick_angles(self):
        for value in (0.0, float("inf"), float("nan"), 0.001):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    execute_joint_full_range(FakeBus(), motor_id=1, delta_degrees=value)

    def test_single_temperature_spike_is_retried_but_confirmed_high_remains(self):
        normal = motor_state(temperature_celsius=34)
        retried = confirm_temperature_sample(
            motor_state(temperature_celsius=90),
            lambda: normal,
            maximum_celsius=45,
        )
        self.assertEqual(retried["temperature_celsius"], 34)
        self.assertTrue(retried["temperature_retry_triggered"])
        self.assertEqual(retried["first_temperature_celsius"], 90)

        confirmed = confirm_temperature_sample(
            motor_state(temperature_celsius=90),
            lambda: motor_state(temperature_celsius=91),
            maximum_celsius=45,
        )
        self.assertEqual(confirmed["temperature_celsius"], 91)

    def test_unsafe_command_is_blocked_before_any_write(self):
        bus = FakeBus()
        with self.assertRaisesRegex(ValueError, "must not exceed"):
            execute_guarded_jog(bus, delta_ticks=49)
        self.assertEqual(bus.motion_calls, [])
        self.assertEqual(bus.disable_calls, 0)


if __name__ == "__main__":
    unittest.main()
