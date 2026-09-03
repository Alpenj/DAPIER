import json
from pathlib import Path
import runpy
import tempfile
from unittest import mock
import unittest


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "dual_so101_smoke"
SMOKE = runpy.run_path(str(SCRIPT))


class FakeBus:
    def __init__(self, torque=1):
        self.torque = torque
        self.connected = False

    @property
    def is_connected(self):
        return self.connected

    def connect(self):
        self.connected = True

    def disconnect(self, disable_torque=True):
        if disable_torque:
            self.torque = 0
        self.connected = False

    def enable_torque(self):
        self.torque = 1

    def disable_torque(self, **_kwargs):
        self.torque = 0

    def sync_write(self, register, values):
        self.goal = values

    def sync_read(self, register, *_args, **_kwargs):
        values = {
            "Present_Position": 1.0,
            "Present_Load": 2,
            "Present_Current": 3,
            "Present_Velocity": 4,
        }
        if register == "Torque_Enable":
            return {"shoulder_pan": self.torque}
        return {"shoulder_pan": values.get(register, 0)}


class DualSO101SmokeTest(unittest.TestCase):
    def test_duplicate_arm_ports_and_active_torque_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "same device"):
            SMOKE["validate_arm_ports"]("/dev/ttyACM0", "/dev/ttyACM0")
        snapshot = {
            register: {"shoulder_pan": 0}
            for register in (
                "Torque_Enable",
                "Moving",
                "Status",
                "Operating_Mode",
                "Present_Temperature",
            )
        }
        SMOKE["validate_motion_health"]({"left": snapshot, "right": snapshot})
        bad = {side: {key: dict(value) for key, value in snapshot.items()} for side in ("left", "right")}
        bad["right"]["Torque_Enable"]["shoulder_pan"] = 1
        with self.assertRaisesRegex(RuntimeError, "Torque_Enable"):
            SMOKE["validate_motion_health"](bad)

    def test_motion_requires_explicit_per_arm_calibration(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            SMOKE["validate_motion_request"](float("nan"), "", None, None, False)
        with self.assertRaisesRegex(ValueError, "VISIBLE_DUAL_SO101_READONLY"):
            SMOKE["validate_motion_request"](0.0, "", None, None, False)
        with self.assertRaisesRegex(ValueError, "operator-present"):
            SMOKE["validate_motion_request"](
                0.0,
                SMOKE["READONLY_CONFIRMATION"],
                None,
                None,
                False,
            )
        SMOKE["validate_motion_request"](
            0.0,
            SMOKE["READONLY_CONFIRMATION"],
            None,
            None,
            True,
        )
        with self.assertRaisesRegex(ValueError, "explicit left and right"):
            SMOKE["validate_motion_request"](
                3.0,
                SMOKE["MOTION_CONFIRMATION"],
                None,
                None,
                True,
            )
        with self.assertRaisesRegex(ValueError, "operator-present"):
            SMOKE["validate_motion_request"](
                3.0,
                SMOKE["MOTION_CONFIRMATION"],
                Path("left.json"),
                Path("right.json"),
                False,
            )
        with self.assertRaisesRegex(ValueError, "must be distinct"):
            SMOKE["validate_motion_request"](
                3.0,
                SMOKE["MOTION_CONFIRMATION"],
                Path("same.json"),
                Path("same.json"),
                True,
            )
        SMOKE["validate_motion_request"](
            3.0,
            SMOKE["MOTION_CONFIRMATION"],
            Path("left.json"),
            Path("right.json"),
            True,
        )

        with tempfile.TemporaryDirectory() as directory:
            calibration = Path(directory) / "calibration.json"
            calibration.write_bytes(b"calibration")
            self.assertEqual(
                SMOKE["calibration_sha256"](calibration),
                "e152337e4e85aa3e81482f0ce329aec7bfad531413fe53fef84f1f0d4165caee",
            )

    def test_control_table_is_checked_before_hardware_connect(self):
        table = {name: object() for name in SMOKE["REQUIRED_CONTROL_TABLE_REGISTERS"]}
        SMOKE["validate_control_table"](table)
        table.pop("Present_Velocity")
        with self.assertRaisesRegex(RuntimeError, "Present_Velocity"):
            SMOKE["validate_control_table"](table)

    def test_trace_records_position_load_and_current(self):
        trace = []
        buses = {"left": FakeBus(), "right": FakeBus()}
        starts = {side: {"shoulder_pan": 0.0} for side in buses}

        with mock.patch.object(SMOKE["time"], "sleep"):
            SMOKE["interpolate"](buses, starts, 1.0, 1, trace)

        self.assertEqual(trace[0]["observed"], {"left": 1.0, "right": 1.0})
        self.assertEqual(trace[0]["present_load_raw"], {"left": 2, "right": 2})
        self.assertEqual(trace[0]["present_current_raw"], {"left": 3, "right": 3})
        self.assertEqual(trace[0]["present_velocity_raw"], {"left": 4, "right": 4})
        self.assertIn("monotonic_s", trace[0])
        self.assertEqual(
            set(SMOKE["health"](FakeBus())),
            {
                "Torque_Enable",
                "Operating_Mode",
                "Acceleration",
                "Goal_Time",
                "Goal_Velocity",
                "Torque_Limit",
                "Present_Load",
                "Present_Current",
                "Present_Temperature",
                "Present_Voltage",
                "Status",
                "Moving",
                "Maximum_Velocity_Limit",
                "Maximum_Acceleration",
            },
        )

    def test_torque_off_is_read_back_after_motion(self):
        buses = {"left": FakeBus(), "right": FakeBus()}
        snapshots = SMOKE["disable_torque_and_verify"](buses)
        self.assertEqual(
            {
                side: snapshot["Torque_Enable"]["shoulder_pan"]
                for side, snapshot in snapshots.items()
            },
            {"left": 0, "right": 0},
        )
        stuck = FakeBus()
        stuck.disable_torque = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, "after motion"):
            SMOKE["disable_torque_and_verify"]({"left": stuck, "right": FakeBus()})

    def test_full_motion_path_writes_complete_witnessed_record(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left_calibration = root / "left.json"
            right_calibration = root / "right.json"
            log = root / "smoke.json"
            left_calibration.write_text("left", encoding="utf-8")
            right_calibration.write_text("right", encoding="utf-8")

            def fake_load_bus(_port, _calibration):
                return FakeBus(torque=0)

            argv = [
                str(SCRIPT),
                "--left-port", "left",
                "--right-port", "right",
                "--left-calibration", str(left_calibration),
                "--right-calibration", str(right_calibration),
                "--move-deg", "1",
                "--confirm", SMOKE["MOTION_CONFIRMATION"],
                "--operator-present",
                "--log", str(log),
            ]
            with (
                mock.patch.dict(SMOKE["main"].__globals__, {"load_bus": fake_load_bus}),
                mock.patch.object(SMOKE["time"], "sleep"),
                mock.patch("sys.argv", argv),
                mock.patch("builtins.print"),
            ):
                self.assertEqual(SMOKE["main"](), 0)

            record = json.loads(log.read_text(encoding="utf-8"))
            self.assertEqual(record["schema_version"], "dapier.dual-so101-smoke.v0.2")
            self.assertTrue(record["user_witnessed"])
            self.assertTrue(record["hardware_execution"])
            self.assertTrue(record["motion_completed"])
            self.assertEqual(len(record["trace"]), 60)
            self.assertEqual(
                {
                    side: record["arms"][side]["health_after_torque_off"]["Torque_Enable"]["shoulder_pan"]
                    for side in ("left", "right")
                },
                {"left": 0, "right": 0},
            )
            self.assertEqual(
                {len(value) for value in record["calibration_sha256"].values()},
                {64},
            )
            self.assertIn("finished_at", record)


if __name__ == "__main__":
    unittest.main()
