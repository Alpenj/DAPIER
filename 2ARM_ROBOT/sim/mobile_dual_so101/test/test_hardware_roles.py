import json
from pathlib import Path
import re
import subprocess
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
ROOT = PROJECT_DIR.parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_modules.camera import CameraRole


class HardwareRolesTest(unittest.TestCase):
    def test_physical_and_system_entrypoints_gate_actions_on_interactive_tty(self):
        scripts = {
            "capture_ros2_hardware_snapshot.sh": "if ! command -v ros2",
            "install_hardware_aliases.sh": "sudo install",
        }
        for name, action in scripts.items():
            source = (ROOT / "scripts" / name).read_text(encoding="utf-8")
            with self.subTest(script=name):
                gate = "[[ -t 0 && -t 1 ]]"
                self.assertIn(gate, source)
                gate_index = source.rindex(gate)
                self.assertLess(gate_index, source.index(action))
                self.assertLess(source.index('if [[ "${1:-}" == "-h"'), gate_index)

    def test_astra_poll_and_viewer_are_fail_closed_without_device_access(self):
        source = (ROOT / "scripts/run_astra_openni2_color").read_text(
            encoding="utf-8"
        )
        self.assertIn("poll/viewer disabled", source)
        self.assertNotIn("/dev/dapier", source)
        self.assertNotIn("exec ", source)

    def test_current_manifest_matches_camera_and_device_entrypoints(self):
        roles = json.loads(
            (ROOT / "config/hardware_roles.json").read_text(encoding="utf-8")
        )
        self.assertEqual(
            roles["platform"], "SO101_dual_arm_on_turtlebot3_waffle_pi"
        )
        devices = roles["devices"]
        self.assertEqual(
            {device["mission_role"] for device in devices.values() if "mission_role" in device},
            {role.value for role in CameraRole},
        )
        self.assertEqual(devices["left_arm"]["hub_downstream_port"], 3)
        self.assertEqual(devices["right_arm"]["hub_downstream_port"], 4)
        self.assertEqual(devices["left_gripper_rgb"]["hub_downstream_port"], 1)
        self.assertEqual(devices["right_gripper_rgb"]["hub_downstream_port"], 2)

        aliases = (ROOT / "scripts/install_hardware_aliases.sh").read_text(
            encoding="utf-8"
        )
        configured = set(
            re.search(r"for role in ([^;]+); do", aliases).group(1).split()
        )
        expected = {
            device["stable_alias"].removeprefix("/dev/dapier/")
            for device in devices.values()
            if "stable_alias" in device
        }
        self.assertEqual(configured, expected)

        astra = (ROOT / "scripts/run_astra_openni2_color").read_text(
            encoding="utf-8"
        )
        self.assertNotIn(devices["front_rgbd"]["stable_alias"], astra)
        self.assertNotIn(devices["workspace_rgbd"]["stable_alias"], astra)
        self.assertIn("poll/viewer disabled", astra)

        legacy = (ROOT / "scripts/run_mujoco_hardware_teleop.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("physical motion is disabled until local safety integration", legacy)
        self.assertNotIn("DAPIER_ENABLE_LEGACY_JDCOBOT", legacy)

    def test_usb_snapshot_requires_exact_readonly_confirmation(self):
        script = ROOT / "scripts/capture_usb_snapshot"
        self.assertEqual(subprocess.run([script, "--help"], check=False).returncode, 0)
        self.assertEqual(
            subprocess.run(
                [script, ROOT / "output/usb_snapshots/test", "--confirm", "WRONG"],
                check=False,
            ).returncode,
            2,
        )

    def test_ros_snapshot_requires_exact_readonly_confirmation(self):
        script = ROOT / "scripts/capture_ros2_hardware_snapshot.sh"
        self.assertEqual(subprocess.run(["bash", script, "--help"], check=False).returncode, 0)
        self.assertEqual(
            subprocess.run(
                ["bash", script, ROOT / "output/hardware_snapshots/test", "--confirm", "WRONG"],
                check=False,
            ).returncode,
            2,
        )

    def test_astra_stream_requires_exact_readonly_confirmation(self):
        script = ROOT / "scripts/run_astra_openni2_color"
        self.assertEqual(
            subprocess.run(
                ["bash", script, "viewer", "--operator-present", "--confirm", "WRONG"],
                check=False,
            ).returncode,
            2,
        )

    def test_udev_install_requires_exact_confirmation(self):
        script = ROOT / "scripts/install_hardware_aliases.sh"
        self.assertEqual(subprocess.run(["bash", script, "--help"], check=False).returncode, 0)
        self.assertEqual(
            subprocess.run(["bash", script, "--confirm", "WRONG"], check=False).returncode,
            2,
        )


if __name__ == "__main__":
    unittest.main()
