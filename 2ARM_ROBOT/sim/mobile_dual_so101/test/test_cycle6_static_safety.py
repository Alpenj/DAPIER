import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"


class Cycle6StaticSafetyTest(unittest.TestCase):
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
