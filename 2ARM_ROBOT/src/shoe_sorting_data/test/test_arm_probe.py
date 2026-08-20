import unittest

from shoe_sorting_data.arm_probe import (
    build_read_packet,
    decode_angle_limits,
    decode_calibration,
    decode_control_state,
    decode_telemetry,
    parse_position_response,
    parse_read_response,
)


class ArmProbeProtocolTest(unittest.TestCase):
    def test_build_packet_can_only_encode_read_instruction(self):
        self.assertEqual(
            build_read_packet(1, 56),
            bytes([0xFF, 0xFF, 1, 4, 0x02, 56, 2, 0xBE]),
        )

    def test_parse_valid_position_response(self):
        packet_without_checksum = bytes([0xFF, 0xFF, 1, 4, 0, 0, 8])
        checksum = (~(sum(packet_without_checksum[2:]) & 0xFF)) & 0xFF
        self.assertEqual(
            parse_position_response(packet_without_checksum + bytes([checksum]), motor_id=1),
            2048,
        )

    def test_parse_rejects_bad_checksum_and_motor_error(self):
        with self.assertRaisesRegex(ValueError, "checksum"):
            parse_position_response(bytes([0xFF, 0xFF, 1, 4, 0, 0, 8, 0]), motor_id=1)

        packet_without_checksum = bytes([0xFF, 0xFF, 1, 4, 2, 0, 8])
        checksum = (~(sum(packet_without_checksum[2:]) & 0xFF)) & 0xFF
        with self.assertRaisesRegex(ValueError, "error byte"):
            parse_position_response(packet_without_checksum + bytes([checksum]), motor_id=1)

    def test_parse_variable_length_read_response(self):
        payload = bytes(range(15))
        packet_without_checksum = bytes([0xFF, 0xFF, 2, 17, 0]) + payload
        checksum = (~(sum(packet_without_checksum[2:]) & 0xFF)) & 0xFF
        self.assertEqual(
            parse_read_response(
                packet_without_checksum + bytes([checksum]), motor_id=2, payload_size=15
            ),
            payload,
        )

    def test_decode_sts_telemetry_preserves_raw_values(self):
        payload = bytearray(15)
        payload[0:2] = (2048).to_bytes(2, "little")
        payload[2:4] = ((1 << 15) | 12).to_bytes(2, "little")
        payload[4:6] = ((1 << 10) | 250).to_bytes(2, "little")
        payload[6] = 74
        payload[7] = 31
        payload[9] = 4
        payload[10] = 1
        payload[13:15] = (20).to_bytes(2, "little")

        decoded = decode_telemetry(bytes(payload))

        self.assertEqual(decoded["position_tick"], 2048)
        self.assertEqual(decoded["speed_raw"], -12)
        self.assertEqual(decoded["load_raw"], -250)
        self.assertEqual(decoded["load_fraction"], -0.25)
        self.assertEqual(decoded["voltage_volts"], 7.4)
        self.assertEqual(decoded["temperature_celsius"], 31)
        self.assertEqual(decoded["hardware_error_status"], 4)
        self.assertTrue(decoded["moving"])
        self.assertEqual(decoded["current_milliamps"], 130.0)
        self.assertIsNone(decoded["estimated_joint_torque_nm"])

    def test_decode_read_only_configuration_blocks(self):
        self.assertEqual(
            decode_angle_limits(bytes([0xE2, 0x00, 0x60, 0x0F])),
            {
                "minimum_position_limit_tick": 226,
                "maximum_position_limit_tick": 3936,
            },
        )
        self.assertEqual(
            decode_calibration(bytes([0x34, 0x12, 0x00])),
            {
                "position_offset_raw": 0x1234,
                "position_offset_tick": 0x1234,
                "operating_mode_raw": 0,
            },
        )
        self.assertEqual(decode_calibration(bytes([0xE7, 0xFC, 0]))["position_offset_tick"], -793)
        payload = bytes([1, 5, 0, 8, 0, 0, 100, 0, 0xE8, 0x03])
        decoded = decode_control_state(payload)
        self.assertTrue(decoded["torque_enabled"])
        self.assertEqual(decoded["goal_position_tick"], 2048)
        self.assertEqual(decoded["goal_speed_raw"], 100)
        self.assertEqual(decoded["torque_limit_raw"], 1000)
        self.assertEqual(decoded["torque_limit_fraction"], 1.0)


if __name__ == "__main__":
    unittest.main()
