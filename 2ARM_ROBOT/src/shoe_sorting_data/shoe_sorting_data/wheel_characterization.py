"""Pure analysis helpers for an off-ground TurtleBot wheel characterization."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
from typing import Mapping, Sequence

WHEEL_SEPARATION_METERS = 0.287


def _response_key(kind: str) -> str:
    if kind == "linear":
        return "median_encoder_linear_x_mps"
    if kind == "angular":
        return "median_encoder_angular_z_radps"
    raise ValueError(f"unsupported wheel-test kind: {kind}")


def _positive_stages(
    stages: Sequence[Mapping[str, object]], *, kind: str
) -> list[Mapping[str, object]]:
    _response_key(kind)
    return sorted(
        (
            stage
            for stage in stages
            if stage.get("kind") == kind and float(stage.get("command", 0.0)) > 0.0
        ),
        key=lambda item: float(item["command"]),
    )


def summarize_stage(samples: Sequence[Mapping[str, object]]) -> dict[str, object]:
    if not samples:
        raise ValueError("a wheel-test stage needs at least one sample")

    def finite_values(key: str) -> list[float]:
        values = [float(sample[key]) for sample in samples if key in sample]
        return [value for value in values if math.isfinite(value)]

    def median(key: str) -> float | None:
        values = finite_values(key)
        return statistics.median(values) if values else None

    voltages = finite_values("battery_voltage")
    currents = finite_values("battery_current")
    left = median("left_wheel_mps")
    right = median("right_wheel_mps")
    encoder_linear = None if left is None or right is None else (left + right) / 2.0
    encoder_angular = (
        None if left is None or right is None else (right - left) / WHEEL_SEPARATION_METERS
    )
    wheel_asymmetry = None
    if left is not None and right is not None and max(abs(left), abs(right)) > 0:
        wheel_asymmetry = abs(abs(left) - abs(right)) / max(abs(left), abs(right))
    return {
        "sample_count": len(samples),
        "median_odom_linear_x_mps": median("odom_linear_x_mps"),
        "median_odom_angular_z_radps": median("odom_angular_z_radps"),
        "median_left_wheel_mps": left,
        "median_right_wheel_mps": right,
        "median_encoder_linear_x_mps": encoder_linear,
        "median_encoder_angular_z_radps": encoder_angular,
        "absolute_wheel_speed_asymmetry_fraction": wheel_asymmetry,
        "battery_voltage_range": [min(voltages), max(voltages)] if voltages else None,
        "battery_current_range": [min(currents), max(currents)] if currents else None,
        "battery_current_informative": any(abs(value) > 1e-9 for value in currents),
    }


def first_responsive_command(
    stages: Sequence[Mapping[str, object]], *, kind: str, tolerance: float
) -> float | None:
    response_key = _response_key(kind)
    for stage in _positive_stages(stages, kind=kind):
        response = stage.get("summary", {}).get(response_key)
        if response is not None and abs(float(response)) > tolerance:
            return float(stage["command"])
    return None


def first_bilateral_command(
    stages: Sequence[Mapping[str, object]], *, kind: str, tolerance: float
) -> float | None:
    """Return the first positive command for which both wheels move as intended."""
    response_key = _response_key(kind)
    for stage in _positive_stages(stages, kind=kind):
        summary = stage.get("summary", {})
        left_value = summary.get("median_left_wheel_mps")
        right_value = summary.get("median_right_wheel_mps")
        response_value = summary.get(response_key)
        if left_value is None or right_value is None or response_value is None:
            continue
        left = float(left_value)
        right = float(right_value)
        response = float(response_value)
        direction_is_correct = left > 0.0 and right > 0.0
        if kind == "angular":
            direction_is_correct = left < 0.0 and right > 0.0
        if direction_is_correct and abs(response) > tolerance:
            return float(stage["command"])
    return None


def _stage_metrics(stage: Mapping[str, object], *, kind: str) -> dict[str, float | None]:
    command = float(stage["command"])
    summary = stage.get("summary", {})
    response_value = summary.get(_response_key(kind))
    response = None if response_value is None else float(response_value)
    tracking_ratio = None if response is None or command == 0.0 else response / command
    asymmetry_value = summary.get("absolute_wheel_speed_asymmetry_fraction")
    asymmetry = None if asymmetry_value is None else float(asymmetry_value)
    return {
        "command": command,
        "measured_encoder_response": response,
        "tracking_ratio": tracking_ratio,
        "absolute_wheel_speed_asymmetry_fraction": asymmetry,
    }


def analyze_characterization(
    stages: Sequence[Mapping[str, object]],
    *,
    linear_tolerance_mps: float = 0.0025,
    angular_tolerance_radps: float = 0.0021,
    maximum_tracking_error_fraction: float = 0.05,
    maximum_asymmetry_fraction: float = 0.05,
) -> dict[str, object]:
    """Derive deadbands and conservative tracking ceilings from measured stages."""
    result: dict[str, object] = {
        "criteria": {
            "linear_stationary_tolerance_mps": linear_tolerance_mps,
            "angular_stationary_tolerance_radps": angular_tolerance_radps,
            "maximum_tracking_error_fraction": maximum_tracking_error_fraction,
            "maximum_absolute_wheel_speed_asymmetry_fraction": maximum_asymmetry_fraction,
        }
    }
    for kind, tolerance, unit_suffix in (
        ("linear", linear_tolerance_mps, "mps"),
        ("angular", angular_tolerance_radps, "radps"),
    ):
        candidates = _positive_stages(stages, kind=kind)
        acceptable: list[Mapping[str, object]] = []
        for stage in candidates:
            metrics = _stage_metrics(stage, kind=kind)
            tracking_ratio = metrics["tracking_ratio"]
            asymmetry = metrics["absolute_wheel_speed_asymmetry_fraction"]
            if (
                tracking_ratio is not None
                and abs(tracking_ratio - 1.0) <= maximum_tracking_error_fraction
                and asymmetry is not None
                and asymmetry <= maximum_asymmetry_fraction
            ):
                acceptable.append(stage)
        result[kind] = {
            f"first_detectable_positive_command_{unit_suffix}": first_responsive_command(
                stages, kind=kind, tolerance=tolerance
            ),
            f"first_bilateral_positive_command_{unit_suffix}": first_bilateral_command(
                stages, kind=kind, tolerance=tolerance
            ),
            "recommended_tracking_ceiling": (
                _stage_metrics(acceptable[-1], kind=kind) if acceptable else None
            ),
            "tested_positive_command_ceiling": (
                _stage_metrics(candidates[-1], kind=kind) if candidates else None
            ),
        }

    voltage_ranges = [
        stage.get("summary", {}).get("battery_voltage_range")
        for stage in stages
    ]
    finite_voltages = [
        float(value)
        for voltage_range in voltage_ranges
        if voltage_range is not None
        for value in voltage_range
        if math.isfinite(float(value))
    ]
    result["battery_voltage_range_v"] = (
        [min(finite_voltages), max(finite_voltages)] if finite_voltages else None
    )
    result["battery_current_informative"] = any(
        bool(stage.get("summary", {}).get("battery_current_informative"))
        for stage in stages
    )
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze an existing TurtleBot wheel test.")
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        print(json.dumps({"error": f"output already exists: {args.output}"}, indent=2))
        return 2
    try:
        source = json.loads(args.input.read_text(encoding="utf-8"))
        stages = source["stages"]
        analysis = {
            "schema_version": "dapier.turtlebot-wheel-analysis.v0.1",
            "created_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "source_file": str(args.input),
            "source_schema_version": source.get("schema_version"),
            "analysis": analyze_characterization(stages),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(analysis, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"output": str(args.output), "analysis": analysis["analysis"]}, indent=2))
        return 0
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(json.dumps({"error": str(error)}, indent=2))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
