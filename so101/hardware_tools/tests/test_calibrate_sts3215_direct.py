from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT = (
    Path(__file__).parents[1]
    / "writes_hardware"
    / "calibrate_sts3215_direct.py"
)
SPEC = importlib.util.spec_from_file_location("calibrate_sts3215_direct", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
direct = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(direct)

SAFE_SCRIPT = SCRIPT.with_name("calibrate_follower_safe.py")
SAFE_SPEC = importlib.util.spec_from_file_location(
    "calibrate_follower_safe", SAFE_SCRIPT
)
assert SAFE_SPEC is not None and SAFE_SPEC.loader is not None
safe = importlib.util.module_from_spec(SAFE_SPEC)
SAFE_SPEC.loader.exec_module(safe)


@pytest.mark.parametrize("value", [-2047, -1, 0, 1, 2047])
def test_sign_magnitude_round_trip(value: int) -> None:
    encoded = direct.encode_sign_magnitude(value, 11)
    assert direct.decode_sign_magnitude(encoded, 11) == value


def test_sign_magnitude_rejects_overflow() -> None:
    with pytest.raises(ValueError, match="exceeds"):
        direct.encode_sign_magnitude(2048, 11)


def valid_calibration() -> dict[str, dict[str, int]]:
    spans = {
        "shoulder_pan": (900, 3200),
        "shoulder_lift": (850, 3300),
        "elbow_flex": (800, 3050),
        "wrist_flex": (650, 3250),
        "wrist_roll": (0, 4095),
        "gripper": (1800, 3350),
    }
    return {
        name: {
            "id": motor_id,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": spans[name][0],
            "range_max": spans[name][1],
        }
        for name, motor_id in direct.MOTORS.items()
    }


def test_validate_calibration_accepts_so101_contract() -> None:
    direct.validate_calibration(valid_calibration())


def test_validate_calibration_rejects_incomplete_motion() -> None:
    calibration = valid_calibration()
    calibration["gripper"]["range_max"] = 1900
    with pytest.raises(ValueError, match="span"):
        direct.validate_calibration(calibration)


def test_validate_calibration_requires_full_turn_wrist() -> None:
    calibration = valid_calibration()
    calibration["wrist_roll"]["range_max"] = 3000
    with pytest.raises(ValueError, match="wrist_roll"):
        direct.validate_calibration(calibration)


def test_atomic_write_json(tmp_path: Path) -> None:
    target = tmp_path / "calibration.json"
    payload = valid_calibration()
    direct.atomic_write_json(target, payload)
    assert json.loads(target.read_text(encoding="utf-8")) == payload
    assert not list(tmp_path.glob("*.tmp"))


def test_direct_script_does_not_import_lerobot() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    assert "import lerobot" not in source
    assert "from lerobot" not in source


def test_safe_recorder_stops_immediately_on_incomplete_enter(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    center = {
        "shoulder_pan": 2047,
        "shoulder_lift": 2047,
        "elbow_flex": 2047,
        "wrist_flex": 2047,
        "gripper": 2047,
    }
    class FakeBus:
        motors = center

        def sync_read(
            self, data_name: str, motors: list[str], normalize: bool
        ) -> dict[str, int]:
            assert data_name == "Present_Position"
            assert not normalize
            return {name: center[name] for name in motors}

    monkeypatch.setattr(safe, "_read_command", lambda: "")
    monkeypatch.setattr(safe.time, "sleep", lambda _: None)

    with pytest.raises(safe.CalibrationIncompleteError, match="Incomplete spans"):
        safe._safe_record_ranges(FakeBus(), list(center), display_values=False)


def test_safe_recorder_status_fits_an_80_column_terminal() -> None:
    spans = {
        "shoulder_pan": 1234,
        "shoulder_lift": 1234,
        "elbow_flex": 999,
        "wrist_flex": 1234,
        "gripper": 999,
    }
    line = safe._format_status_line(spans)
    assert len(line) <= 79
    assert "P:1234*" in line
    assert "G:999*" in line


def test_safe_recorder_returns_after_complete_motion(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    center = {
        "shoulder_pan": 2047,
        "shoulder_lift": 2047,
        "elbow_flex": 2047,
        "wrist_flex": 2047,
        "gripper": 2047,
    }
    low = {
        "shoulder_pan": 1000,
        "shoulder_lift": 1000,
        "elbow_flex": 1200,
        "wrist_flex": 1000,
        "gripper": 1700,
    }
    high = {
        "shoulder_pan": 3100,
        "shoulder_lift": 3100,
        "elbow_flex": 2900,
        "wrist_flex": 3100,
        "gripper": 2400,
    }

    class FakeBus:
        motors = center

        def __init__(self) -> None:
            self.values = iter((center, center, low, high))

        def sync_read(
            self, data_name: str, motors: list[str], normalize: bool
        ) -> dict[str, int]:
            assert data_name == "Present_Position"
            assert not normalize
            values = next(self.values)
            return {name: values[name] for name in motors}

    commands = iter((None, None, ""))
    monkeypatch.setattr(safe, "_read_command", lambda: next(commands))
    monkeypatch.setattr(safe.time, "sleep", lambda _: None)

    mins, maxes = safe._safe_record_ranges(
        FakeBus(), list(center), display_values=False
    )
    assert mins == low
    assert maxes == high
