import hashlib
import json
import os
from pathlib import Path
import runpy
import tempfile
from types import SimpleNamespace
from unittest import mock
import unittest


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "dual_so101_smoke"
SMOKE = runpy.run_path(str(SCRIPT))


def calibration() -> dict:
    return {
        name: {
            "id": motor_id,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 0,
            "range_max": 4095,
        }
        for name, (motor_id, _) in SMOKE["MOTORS"].items()
    }


def write_profile(root: Path) -> Path:
    arms = {}
    for side in ("left", "right"):
        calibration_path = root / f"{side}.json"
        calibration_path.write_text(json.dumps(calibration()), encoding="utf-8")
        arms[side] = {
            "port": f"/synthetic/{side}",
            "controller_serial": f"synthetic-controller-{side}",
            "calibration_path": calibration_path.name,
            "calibration_sha256": hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
        }
    profile = root / "profile.json"
    profile.write_text(
        json.dumps({"schema_version": SMOKE["PROFILE_SCHEMA_VERSION"], "arms": arms}),
        encoding="utf-8",
    )
    profile.chmod(0o600)
    return profile


class FakeBus:
    def __init__(self):
        self.connected = False
        self.position = {name: 0 for name in SMOKE["MOTORS"]}

    @property
    def is_connected(self):
        return self.connected

    def connect(self):
        self.connected = True

    def disconnect(self, disable_torque=True):
        if disable_torque:
            raise AssertionError("read-only disconnect must not change torque")
        self.connected = False

    def sync_read(self, register, *_args, **_kwargs):
        if register == "Present_Position":
            return dict(self.position)
        value = 0 if register in {"Torque_Enable", "Status", "Moving"} else 1
        return {name: value for name in SMOKE["MOTORS"]}

    def read_calibration(self):
        return {name: SimpleNamespace(**entry) for name, entry in calibration().items()}


class DualSO101SmokeTest(unittest.TestCase):
    def test_eeprom_read_distinguishes_file_match_from_physical_model_zero(self):
        bus = FakeBus()
        expected = calibration()
        audit = SMOKE["inspect_motor_calibration"](bus, expected)
        self.assertTrue(audit["matches_saved_calibration"])
        self.assertFalse(audit["physical_model_zero_verified"])
        expected["elbow_flex"]["homing_offset"] = 73
        audit = SMOKE["inspect_motor_calibration"](bus, expected)
        self.assertFalse(audit["matches_saved_calibration"])
        self.assertEqual(audit["mismatched_fields"], ["elbow_flex.homing_offset"])

    def test_trusted_profile_binds_roles_and_rejects_untrusted_input(self):
        with tempfile.TemporaryDirectory() as directory:
            profile = write_profile(Path(directory))
            loaded = SMOKE["load_trusted_profile"](profile)
            self.assertEqual(set(loaded), {"left", "right"})
            self.assertEqual(loaded["left"]["port"], "/synthetic/left")

            profile.chmod(0o640)
            with self.assertRaisesRegex(ValueError, "owner-only"):
                SMOKE["load_trusted_profile"](profile)
            profile.chmod(0o600)
            with (
                mock.patch.object(SMOKE["os"], "getuid", return_value=os.getuid() + 1),
                self.assertRaisesRegex(ValueError, "owned"),
            ):
                SMOKE["load_trusted_profile"](profile)

        source = SCRIPT.read_text(encoding="utf-8")
        for option in (
            "--left-port",
            "--right-port",
            "--left-calibration",
            "--right-calibration",
            "--left-controller-serial",
            "--right-controller-serial",
        ):
            self.assertNotIn(f'add_argument("{option}"', source)

    def test_profile_rejects_schema_identity_and_digest_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = write_profile(root)
            original = json.loads(profile.read_text(encoding="utf-8"))
            mutations = (
                ("missing field", lambda value: value["arms"]["left"].pop("port")),
                ("unexpected field", lambda value: value.update({"extra": True})),
                (
                    "distinct",
                    lambda value: value["arms"]["right"].update(
                        controller_serial=value["arms"]["left"]["controller_serial"]
                    ),
                ),
                (
                    "same device",
                    lambda value: value["arms"]["right"].update(
                        port=value["arms"]["left"]["port"]
                    ),
                ),
                (
                    "digest mismatch",
                    lambda value: value["arms"]["left"].update(
                        calibration_sha256="0" * 64
                    ),
                ),
            )
            for message, mutate in mutations:
                candidate = json.loads(json.dumps(original))
                mutate(candidate)
                profile.write_text(json.dumps(candidate), encoding="utf-8")
                with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                    SMOKE["load_trusted_profile"](profile)

    def test_calibration_rejects_unsafe_motor_limits(self):
        raw = calibration()
        SMOKE["validate_calibration"](json.dumps(raw).encode(), "left")
        for minimum, maximum in ((-1, 100), (100, 100), (100, 4096)):
            candidate = json.loads(json.dumps(raw))
            candidate["shoulder_pan"].update(range_min=minimum, range_max=maximum)
            with self.subTest(limits=(minimum, maximum)), self.assertRaisesRegex(
                ValueError, "unsafe configured motor limits"
            ):
                SMOKE["validate_calibration"](json.dumps(candidate).encode(), "left")

    def test_nonzero_motion_is_unconditionally_disabled(self):
        with self.assertRaisesRegex(ValueError, "physical motion is disabled"):
            SMOKE["validate_motion_request"](
                0.1, "VISIBLE_DUAL_SO101_3DEG", True
            )
        with self.assertRaisesRegex(ValueError, "finite"):
            SMOKE["validate_motion_request"](float("nan"), "", False)
        with self.assertRaisesRegex(ValueError, SMOKE["READONLY_CONFIRMATION"]):
            SMOKE["validate_motion_request"](0.0, "", True)
        SMOKE["validate_motion_request"](
            0.0, SMOKE["READONLY_CONFIRMATION"], True
        )

    def test_identity_is_rejected_on_either_side_of_connect(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = write_profile(root)
            log = root / "smoke.json"
            bus = FakeBus()
            serials = iter(
                [
                    "synthetic-controller-left",
                    "synthetic-controller-right",
                    "synthetic-controller-left",
                    "changed-after-connect",
                ]
            )
            argv = [
                str(SCRIPT),
                "--profile", str(profile),
                "--confirm", SMOKE["READONLY_CONFIRMATION"],
                "--operator-present",
                "--log", str(log),
            ]
            with (
                mock.patch.dict(
                    SMOKE["main"].__globals__,
                    {
                        "load_bus": lambda *_args: bus,
                        "controller_serial": lambda _port: next(serials),
                    },
                ),
                mock.patch.object(SMOKE["os"], "isatty", return_value=True),
                mock.patch("sys.argv", argv),
            ):
                with self.assertRaisesRegex(RuntimeError, "private log"):
                    SMOKE["main"]()
            self.assertFalse(bus.connected)

    def test_mocked_readonly_path_redacts_identity_and_uses_new_private_log(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = write_profile(root)
            log = root / "smoke.json"
            argv = [
                str(SCRIPT),
                "--profile", str(profile),
                "--confirm", SMOKE["READONLY_CONFIRMATION"],
                "--operator-present",
                "--inspect-calibration",
                "--log", str(log),
            ]
            with (
                mock.patch.dict(
                    SMOKE["main"].__globals__,
                    {
                        "load_bus": lambda *_args: FakeBus(),
                        "controller_serial": lambda port: (
                            f"synthetic-controller-{Path(port).name}"
                        ),
                    },
                ),
                mock.patch.object(SMOKE["os"], "isatty", return_value=True),
                mock.patch("sys.argv", argv),
                mock.patch("builtins.print"),
            ):
                self.assertEqual(SMOKE["main"](), 0)

            text = log.read_text(encoding="utf-8")
            record = json.loads(text)
            self.assertEqual(log.stat().st_mode & 0o777, 0o600)
            self.assertFalse(record["motion_enabled"])
            self.assertFalse(record["hardware_execution"])
            self.assertEqual(record["controller_identity_revalidated"], {"left": True, "right": True})
            self.assertFalse(record["connected_endpoint_identity_bound"])
            for secret in ("synthetic-controller", "/synthetic/"):
                self.assertNotIn(secret, text)
            self.assertEqual(record["source_sha256"], hashlib.sha256(SCRIPT.read_bytes()).hexdigest())
            for side in ("left", "right"):
                arm = record["arms"][side]
                self.assertTrue(arm["motor_calibration_audit"]["matches_saved_calibration"])
                self.assertEqual(arm["calibration_sha256"], hashlib.sha256((root / f"{side}.json").read_bytes()).hexdigest())
                self.assertEqual(arm["device_id"], f"dapier_dual_follower_{side}")
                self.assertLessEqual(arm["position_started_at"], arm["position_finished_at"])
                self.assertGreaterEqual(arm["position_read_duration_ns"], 0)
                self.assertEqual(arm["position_finished_monotonic_ns"]-arm["position_started_monotonic_ns"],
                                 arm["position_read_duration_ns"])
                self.assertTrue(record["host_boot_id"])

            with self.assertRaisesRegex(ValueError, "already exists"):
                SMOKE["_open_new_log"](log)
            target = root / "target"
            target.write_text("keep", encoding="utf-8")
            symlink = root / "linked-log"
            symlink.symlink_to(target)
            with self.assertRaisesRegex(ValueError, "already exists"):
                SMOKE["_open_new_log"](symlink)
            self.assertEqual(target.read_text(encoding="utf-8"), "keep")


if __name__ == "__main__":
    unittest.main()
