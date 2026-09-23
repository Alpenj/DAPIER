"""Left wrist visual servo adapter for closed-loop fine alignment.

Features:
  - Consumes caller-supplied frame timestamp and measured q metadata.
    Camera exposure/joint-state association is not independently verified here.
  - Computes bounded proportional correction on wrist_flex and wrist_roll.
  - Fail-closed stale observation rejection (max_age_ns).
  - Fail-closed target loss / low-confidence rejection.
  - Strict step limit (max_delta_deg) to prevent violent commands.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np

from dapier_research.control_intent import ControlIntent, arm_joint_position_intent, validate_intent
from dapier_research.vision_target import PixelDetection, _validate_detection


class WristServoError(RuntimeError):
    pass


class StaleObservationError(WristServoError):
    pass


class TargetLostError(WristServoError):
    pass


@dataclass(frozen=True)
class WristObservation:
    timestamp_ns: int
    measured_q: dict[str, float]  # degrees at exposure time
    feature_center_uv: tuple[float, float] | None  # normalized image coordinates in [-1.0, 1.0]
    confidence: float


@dataclass(frozen=True)
class WristServoConfig:
    target_uv: tuple[float, float] = (0.0, 0.0)  # desired alignment target in normalized coordinates
    kp_roll: float = 2.0  # degrees per unit normalized error
    kp_flex: float = 2.0  # degrees per unit normalized error
    max_delta_deg: float = 0.50  # maximum correction step per cycle
    max_age_ns: int = 1_000_000_000  # 1.0 second freshness deadline
    min_confidence: float = 0.60


def wrist_observation_from_detection(
    detection: PixelDetection, *, image_shape: tuple[int, int],
    timestamp_ns: int, measured_q: Mapping[str, float],
) -> WristObservation:
    """Convert the existing RGB mask contract to wrist features; no depth required.

    The mask centroid is an image feature, not a physical grasp center. The
    perception producer must bind this mask to the supplied image and timestamp.
    """
    if (len(image_shape) != 2 or any(type(v) is not int or v < 2 for v in image_shape)
            or type(timestamp_ns) is not int or timestamp_ns < 0):
        raise WristServoError("invalid wrist image dimensions or acquisition timestamp")
    mask = _validate_detection(detection, image_shape=image_shape)
    y, x = np.nonzero(mask)
    if not len(x):
        raise TargetLostError("Target mask is empty")
    height, width = image_shape
    uv = (2 * float(x.mean()) / (width - 1) - 1,
          2 * float(y.mean()) / (height - 1) - 1)
    return WristObservation(timestamp_ns, dict(measured_q), uv, detection.confidence)


def compute_bounded_wrist_correction(
    obs: WristObservation,
    nominal_cmd: Mapping[str, float],
    config: WristServoConfig,
    now_monotonic_ns: int,
) -> dict[str, float]:
    """Compute bounded joint correction from wrist visual error.

    Args:
        obs: Timestamped wrist observation with measured q.
        nominal_cmd: Baseline commanded joint angles (degrees).
        config: Servo gains, bounds, and safety thresholds.
        now_monotonic_ns: Current monotonic timestamp in nanoseconds.

    Returns:
        Updated joint command dictionary with bounded corrections applied.

    Raises:
        StaleObservationError: If observation is older than max_age_ns.
        TargetLostError: If target feature is not found or confidence is too low.
    """
    if (len(config.target_uv) != 2
            or not all(math.isfinite(v) and -1 <= v <= 1 for v in config.target_uv)
            or not all(math.isfinite(v) for v in (config.kp_roll, config.kp_flex, config.max_delta_deg, config.min_confidence))
            or config.max_delta_deg <= 0 or not 0 <= config.min_confidence <= 1
            or type(config.max_age_ns) is not int or config.max_age_ns <= 0):
        raise WristServoError("invalid servo configuration")
    if (type(now_monotonic_ns) is not int or type(obs.timestamp_ns) is not int
            or now_monotonic_ns < 0 or obs.timestamp_ns < 0
            or not all(math.isfinite(v) for v in nominal_cmd.values())
            or not all(math.isfinite(v) for v in obs.measured_q.values())):
        raise WristServoError("invalid timestamp or joint-state input")
    # 1. Freshness check
    age_ns = now_monotonic_ns - obs.timestamp_ns
    if age_ns < 0 or age_ns > config.max_age_ns:
        raise StaleObservationError(
            f"Wrist observation is stale: age {age_ns * 1e-6:.1f}ms > max {config.max_age_ns * 1e-6:.1f}ms"
        )

    # 2. Target detection and confidence check
    if obs.feature_center_uv is None:
        raise TargetLostError("Target feature lost or not detected in wrist frame")
    if not math.isfinite(obs.confidence) or not config.min_confidence <= obs.confidence <= 1:
        raise TargetLostError(
            f"Target confidence {obs.confidence:.3f} is below threshold {config.min_confidence:.3f}"
        )

    u, v = obs.feature_center_uv
    if not (math.isfinite(u) and math.isfinite(v) and -1 <= u <= 1 and -1 <= v <= 1):
        raise TargetLostError(f"Non-finite feature coordinates: ({u}, {v})")

    # 3. Compute image error relative to target
    target_u, target_v = config.target_uv
    err_u = target_u - u
    err_v = target_v - v

    # 4. Proportional control with strict saturation bounding
    raw_delta_roll = config.kp_roll * err_u
    raw_delta_flex = config.kp_flex * err_v

    delta_roll = max(-config.max_delta_deg, min(config.max_delta_deg, raw_delta_roll))
    delta_flex = max(-config.max_delta_deg, min(config.max_delta_deg, raw_delta_flex))

    # 5. Apply correction to nominal command
    corrected_cmd = dict(nominal_cmd)
    if "wrist_roll" in corrected_cmd:
        corrected_cmd["wrist_roll"] = round(corrected_cmd["wrist_roll"] + delta_roll, 3)
    if "wrist_flex" in corrected_cmd:
        corrected_cmd["wrist_flex"] = round(corrected_cmd["wrist_flex"] + delta_flex, 3)

    return corrected_cmd


def wrist_correction_intent(
    obs: WristObservation,
    nominal: ControlIntent,
    config: WristServoConfig,
    now_monotonic_ns: int,
    *,
    sequence: int,
    joint_limits_rad: Mapping[str, tuple[float, float]],
) -> ControlIntent:
    """Convert a left-arm image correction to the existing C++ ingress contract.

    Inputs use the same host clock and calibrated joint axes. This proposal still
    needs path/axis validation and the executor's current-state and approval gates.
    """
    validate_intent(nominal)
    expected = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"}
    if nominal.kind != "arm_joint_position" or set(nominal.joint_names) != expected:
        raise WristServoError("complete single-arm nominal intent required")
    if (type(now_monotonic_ns) is not int or type(sequence) is not int
            or sequence <= nominal.sequence
            or not 0 <= now_monotonic_ns - nominal.source_monotonic_ns <= nominal.ttl_ns):
        raise StaleObservationError("stale nominal intent or non-increasing sequence")
    positions = dict(zip(nominal.joint_names, nominal.joint_position_rad))
    wrist_deg = {name: math.degrees(positions[name]) for name in ("wrist_flex", "wrist_roll")}
    corrected = compute_bounded_wrist_correction(obs, wrist_deg, config, now_monotonic_ns)
    # Bound against measured wrist positions too; an old nominal must not hide a jump.
    for name, value in corrected.items():
        measured = obs.measured_q.get(name)
        if measured is None or abs(value - measured) > config.max_delta_deg + 1e-9:
            raise WristServoError("wrist correction exceeds measured-state step bound")
        positions[name] = math.radians(value)
    for name, value in positions.items():
        bounds = joint_limits_rad.get(name)
        if (bounds is None or len(bounds) != 2 or not all(math.isfinite(x) for x in bounds)
                or not bounds[0] <= value <= bounds[1]):
            raise WristServoError(f"{name}: missing or exceeded joint limits; not clipped")
    return arm_joint_position_intent(
        sequence=sequence, source="wrist_servo_adapter", source_monotonic_ns=now_monotonic_ns,
        ttl_ns=nominal.ttl_ns, joint_names=nominal.joint_names,
        joint_position_rad=tuple(positions[name] for name in nominal.joint_names),
        joint_max_velocity_rad_s=nominal.joint_max_velocity_rad_s,
    )
