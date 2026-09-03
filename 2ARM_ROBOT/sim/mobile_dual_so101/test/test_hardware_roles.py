import json
from pathlib import Path
import re
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
ROOT = PROJECT_DIR.parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mission_modules.camera import CameraRole


class HardwareRolesTest(unittest.TestCase):
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
        self.assertIn(devices["front_rgbd"]["stable_alias"], astra)
        self.assertNotIn(devices["workspace_rgbd"]["stable_alias"], astra)


if __name__ == "__main__":
    unittest.main()
