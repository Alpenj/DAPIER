from __future__ import annotations

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[3]
INTERFACES = REPO_ROOT / "so101_ros2" / "dapier_interfaces" / "msg"
BRIDGE_SOURCE = (
    REPO_ROOT
    / "so101_ros2"
    / "dapier_safety_bridge"
    / "src"
    / "safety_bridge_node.cpp"
)


class RosInterfaceContractTest(unittest.TestCase):
    def parse_fields(self, name: str) -> list[tuple[str, str]]:
        path = INTERFACES / name
        self.assertTrue(path.is_file(), path)
        fields: list[tuple[str, str]] = []
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            field_type, field_name = line.split()
            fields.append((field_type, field_name))
        return fields

    def fields_by_name(self, name: str) -> dict[str, str]:
        return {
            field_name: field_type
            for field_type, field_name in self.parse_fields(name)
        }

    def test_research_intent_keeps_authorization_explicit_and_false_by_contract(self) -> None:
        fields = self.parse_fields("ResearchControlIntent.msg")
        self.assertIn(("bool", "control_authorized"), fields)
        self.assertNotIn(("string", "device_path"), fields)
        self.assertNotIn(("string", "serial_port"), fields)
        self.assertEqual(fields[0], ("string", "schema_version"))
        self.assertEqual(fields[1], ("uint64", "sequence"))

    def test_localization_shape_matches_language_neutral_contract(self) -> None:
        fields = self.fields_by_name("LocalizationEstimate.msg")
        self.assertEqual(fields["position_m"], "float64[3]")
        self.assertEqual(fields["orientation_xyzw"], "float64[4]")
        self.assertEqual(fields["covariance"], "float64[36]")
        self.assertIn("simulator_truth_used", fields)
        self.assertIn("control_authorized", fields)

    def test_safe_command_remains_non_authorizing_and_observable(self) -> None:
        fields = self.fields_by_name("SafeCommand.msg")
        self.assertIn("dispatch_allowed", fields)
        self.assertIn("safe_stop_latched", fields)
        self.assertIn("hardware_execution", fields)
        self.assertIn("expires_at_monotonic_ns", fields)

    def test_replan_acknowledgement_binds_sequence_and_identity(self) -> None:
        fields = {name for _, name in self.parse_fields("ReplanAcknowledgement.msg")}
        self.assertEqual(
            fields,
            {
                "localization_sequence",
                "parent_frame",
                "child_frame",
                "map_id",
                "session_id",
                "reset_generation",
            },
        )

    def test_bridge_never_publishes_direct_actuator_topics(self) -> None:
        source = BRIDGE_SOURCE.read_text(encoding="utf-8")
        self.assertIn('"/dapier/safe_command"', source)
        for forbidden in (
            '"/cmd_vel"',
            '"/joint_trajectory"',
            '"/position_controller/commands"',
            '"/dev/tty',
            '"/dev/serial',
        ):
            self.assertNotIn(forbidden, source)
        self.assertIn("message.hardware_execution = false", source)


if __name__ == "__main__":
    unittest.main()
