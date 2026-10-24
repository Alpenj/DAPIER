import ast
import builtins
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
import runpy
import subprocess
import sys
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
PACKAGE = ROOT / "src/shoe_sorting_data/shoe_sorting_data"
PHYSICAL_MOTION_ENTRYPOINTS = (
    (SCRIPTS / "run_mujoco_hardware_teleop.sh", "exec systemd-inhibit"),
    (PACKAGE / "arm_jog.py", "_run_hardware(args)"),
    (PACKAGE / "wheel_test_ros.py", "rclpy.init"),
)


class Cycle6StaticSafetyTest(unittest.TestCase):
    def test_every_physical_motion_entrypoint_is_unconditionally_disabled(self):
        for path, backend_call in PHYSICAL_MOTION_ENTRYPOINTS:
            source = path.read_text(encoding="utf-8")
            with self.subTest(path=path.name):
                guard = "physical motion is disabled until local safety integration"
                self.assertIn(guard, source)
                self.assertNotIn(backend_call, source)

        help_result = subprocess.run(
            ["bash", SCRIPTS / "run_mujoco_hardware_teleop.sh", "--help"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(help_result.returncode, 0)

    def test_arm_jog_cli_blocks_before_hardware_backend_but_keeps_help(self):
        original_import = builtins.__import__
        sys.path.insert(0, str(PACKAGE.parent))
        try:
            from shoe_sorting_data import arm_jog

            with patch.object(arm_jog, "_run_hardware") as backend:
                with (
                    redirect_stderr(StringIO()),
                    self.assertRaises(SystemExit) as blocked,
                ):
                    arm_jog.main([
                        "preflight",
                        "--port",
                        "/not-a-device",
                        "--expected-serial",
                        "STATIC-ONLY",
                    ])
                self.assertEqual(blocked.exception.code, 2)
                backend.assert_not_called()

                with (
                    redirect_stdout(StringIO()),
                    self.assertRaises(SystemExit) as help_exit,
                ):
                    arm_jog.main(["--help"])
                self.assertEqual(help_exit.exception.code, 0)
                backend.assert_not_called()

            with (
                patch.object(arm_jog, "verify_controller_identity") as identity,
                patch.object(arm_jog, "STS3215Bus") as bus_type,
                self.assertRaisesRegex(arm_jog.JogSafetyError, "physical motion is disabled"),
            ):
                arm_jog._run_hardware(Mock())
            identity.assert_not_called()
            bus_type.assert_not_called()

            def reject_serial_import(name, *args, **kwargs):
                if name == "serial":
                    raise AssertionError("serial backend import attempted")
                return original_import(name, *args, **kwargs)

            with (
                patch("builtins.__import__", side_effect=reject_serial_import),
                self.assertRaisesRegex(arm_jog.JogSafetyError, "physical motion is disabled"),
            ):
                arm_jog.STS3215Bus("/not-a-device")

            bus = object.__new__(arm_jog.STS3215Bus)
            bus.connection = Mock()
            with self.assertRaisesRegex(arm_jog.JogSafetyError, "physical motion is disabled"):
                bus._send(b"not-sent")
            bus.connection.write.assert_not_called()
        finally:
            sys.path.remove(str(PACKAGE.parent))

    def test_wheel_cli_blocks_before_ros_import_but_keeps_help(self):
        path = PACKAGE / "wheel_test_ros.py"
        original_import = builtins.__import__

        def reject_ros_import(name, *args, **kwargs):
            if name.split(".", 1)[0] in {
                "geometry_msgs",
                "nav_msgs",
                "rclpy",
                "sensor_msgs",
            }:
                raise AssertionError(f"ROS backend import attempted: {name}")
            return original_import(name, *args, **kwargs)

        for argv, expected_code in (
            ([str(path), "--help"], 0),
            (
                [
                    str(path),
                    "--output",
                    "/not-written.json",
                    "--wheels-off-ground-confirmed",
                ],
                2,
            ),
        ):
            with self.subTest(argv=argv):
                with (
                    patch.object(sys, "argv", argv),
                    patch("builtins.__import__", side_effect=reject_ros_import),
                    redirect_stdout(StringIO()),
                    redirect_stderr(StringIO()),
                    self.assertRaises(SystemExit) as exit_status,
                ):
                    runpy.run_path(path, run_name="__main__")
                self.assertEqual(exit_status.exception.code, expected_code)

    def test_sim_teleop_speed_limits_are_in_the_iterated_invalid_arguments(self):
        source = Path(__file__).with_name("test_sim_teleop.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        assignment = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "invalid_arguments"
                for target in node.targets
            )
        )
        invalid_arguments = ast.literal_eval(assignment.value)
        self.assertIn(("--teleop", "--teleop-max-speed-deg-s", "0"), invalid_arguments)
        self.assertIn(("--teleop", "--teleop-accel-deg-s2", "0"), invalid_arguments)
        self.assertTrue(
            any(
                isinstance(node, ast.For)
                and isinstance(node.iter, ast.Name)
                and node.iter.id == "invalid_arguments"
                for node in ast.walk(tree)
            )
        )

    def test_dual_arm_entrypoint_is_readonly_and_writes_private_new_logs(self):
        source = (SCRIPTS / "dual_so101_smoke").read_text(encoding="utf-8")
        tree = ast.parse(source)
        calls = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }

        self.assertIn("physical motion is disabled", source)
        self.assertNotIn("signal", {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)})
        self.assertTrue({"enable_torque", "sync_write"}.isdisjoint(calls))
        self.assertIn("validate_calibration", source)
        self.assertIn("unsafe configured motor limits", source)
        self.assertGreaterEqual(source.count("verify_arm_identities("), 3)
        self.assertIn('"connected_endpoint_identity_bound": False', source)
        for token in ("os.O_EXCL", "os.O_NOFOLLOW", "0o600"):
            self.assertIn(token, source)

    def test_alias_install_uses_one_verified_private_staging_copy(self):
        source = (SCRIPTS / "install_hardware_aliases.sh").read_text(encoding="utf-8")

        self.assertNotIn("[RULES_FILE]", source)
        self.assertIn('config/99-dapier-hardware.rules', source)
        for token in ("! -L", "0022", "sha256sum", "mktemp -d", "chmod 0400", "trap cleanup", "staged_rules"):
            self.assertIn(token, source)
        self.assertGreaterEqual(source.count("sha256sum"), 3)
        self.assertIn('sudo install -m 0644 "${staged_rules}"', source)

    def test_astra_runtime_is_static_only(self):
        source = (SCRIPTS / "run_astra_openni2_color").read_text(encoding="utf-8")

        self.assertIn("poll/viewer disabled", source)
        self.assertNotIn("exec ", source)
        self.assertNotIn("/dev/dapier", source)

    def test_snapshot_outputs_are_private_atomic_and_no_clobber(self):
        for name in ("capture_ros2_hardware_snapshot.sh", "capture_usb_snapshot"):
            source = (SCRIPTS / name).read_text(encoding="utf-8")
            with self.subTest(script=name):
                for token in ("umask 077", "mktemp -d", "|| -L", "trap cleanup", "mv -Tn"):
                    self.assertIn(token, source)
                self.assertLess(source.index('[[ -e "$1" || -L "$1" ]]'), source.index("realpath -m"))

    def test_output_ignore_and_disabled_state_are_documented(self):
        self.assertIn("/output/", (ROOT / ".gitignore").read_text(encoding="utf-8"))
        for path in (
            ROOT / "README.md",
            ROOT / "docs/research/MOBILE_DUAL_SO101_EXECUTION_LEDGER_20260902.md",
        ):
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path.name):
                self.assertIn("local safety integration", text)
                self.assertIn("physical motion", text)
                self.assertIn("camera streaming", text)


if __name__ == "__main__":
    unittest.main()
