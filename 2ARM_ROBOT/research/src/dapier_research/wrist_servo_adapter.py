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
import hashlib
import io
import json
import math
from pathlib import Path
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


def block_grasp_observation_from_wrist(document, *, run_id, object_id,
                                       calibration_revision, sequence, source_kind):
    """Produce frame-bound object features for native CLOSE, without device I/O.

    Existing centroid processing establishes visibility, not jaw contact, lift
    or absence of support. Those facts remain null until an estimator can prove
    them; the native grasp gate rejects null before dispatch. Capture time is
    preserved even when replaying old frames, never replaced with processing time.
    """
    import cv2

    if (source_kind not in ("mock", "hardware") or type(sequence) is not int or sequence <= 0
            or any(not isinstance(v, str) or not v.strip()
                   for v in (run_id, object_id, calibration_revision))):
        raise WristServoError("invalid object observation identity")

    def source_bytes(source):
        path = Path(source["path"])
        if not path.is_file() or not 0 < path.stat().st_size <= 16_777_216:
            raise WristServoError("saved regular observation file required")
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != source["sha256"]:
            raise WristServoError("object observation source changed")
        return raw

    capture = json.loads(source_bytes(document["capture_source"]))
    if (capture.get("schema_version") != "dapier.wrist-frame.v1"
            or capture.get("side") != "left" or capture.get("clock") != "host_monotonic_ns"
            or capture.get("frame_acquired") is not True
            or capture.get("normal_stream_close") is not True
            or type(capture.get("timestamp_ns")) is not int or capture["timestamp_ns"] <= 0
            or not isinstance(capture.get("host_boot_id"), str) or not capture["host_boot_id"]):
        raise WristServoError("completed native wrist capture with original clock identity required")
    frame = {"path": capture["frame_path"], "sha256": capture["frame_sha256"]}
    raw = source_bytes(frame)
    image = (np.load(io.BytesIO(raw), allow_pickle=False) if Path(frame["path"]).suffix == ".npy"
             else cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR))
    if image is None or image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise WristServoError("object source must be an HxWx3 uint8 image")
    feature = None
    detection = document.get("detection")
    if detection is not None:
        if detection.get("frame_sha256") != frame["sha256"]:
            raise WristServoError("object mask must identify its source frame")
        mask = np.load(io.BytesIO(source_bytes(detection["mask_source"])), allow_pickle=False)
        try:
            visible = wrist_observation_from_detection(PixelDetection(
                detection["label"], mask, detection["confidence"], detection["detector"],
                detection["uses_privileged_labels"]), image_shape=image.shape[:2],
                timestamp_ns=capture["timestamp_ns"], measured_q={})
            feature = {"center_uv": list(visible.feature_center_uv), "confidence": visible.confidence,
                       "mask_source": detection["mask_source"]}
        except TargetLostError:
            pass
    return {"schema_version": "dapier.block-grasp-observation.v1", "source_kind": source_kind,
            "run_id": run_id, "object_id": object_id, "calibration_revision": calibration_revision,
            "producer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "boot_id": capture["host_boot_id"], "sequence": sequence,
            "captured_monotonic_ns": capture["timestamp_ns"], "frame_path": frame["path"],
            "frame_sha256": frame["sha256"], "capture_source": document["capture_source"],
            "visible_feature": feature, "bilateral_grasp_verified": None, "external_support": None,
            "bottom_clearance_lower_bound_m": None, "observer_physically_verified": False,
            "unknown_reason": "RGB feature visibility does not establish bilateral grasp, lift or external support"}


def block_support_observation_from_wrist(document, *, support_id, **identity):
    """Bind existing saved wrist features to the supported-ending input schema.

    Visibility is not support or release evidence. Preserve unknowns so the
    native consumer cannot open a gripper on a feature-only observation.
    """
    if not isinstance(support_id, str) or not support_id.strip():
        raise WristServoError("approved support identity required")
    observation = block_grasp_observation_from_wrist(document, **identity)
    return {**observation, "schema_version": "dapier.block-support-observation.v1",
            "support_id": support_id, "approved_support_verified": None,
            "object_released_verified": None}


def bind_native_wrist_observation(document, measured_source, measured_model_rad, *, now_ns):
    """Join saved native frame and already validated readback, without device I/O.

    A bounded timestamp gap is association evidence, not an assertion that joints
    were sampled at exposure. Keep both intervals for the downstream motion review.
    """
    def read_source(source):
        raw = Path(source["path"]).read_bytes()
        if hashlib.sha256(raw).hexdigest() != source["sha256"]:
            raise WristServoError("wrist acquisition/readback source changed")
        return json.loads(raw)

    capture = read_source(document["capture_source"])
    readback = read_source(measured_source)
    measured = readback["arms"]["left"]
    if (capture.get("schema_version") != "dapier.wrist-frame.v1"
            or capture.get("side") != "left" or capture.get("clock") != "host_monotonic_ns"
            or capture.get("frame_acquired") is not True
            or capture.get("normal_stream_close") is not True):
        raise WristServoError("completed native left wrist acquisition required")
    if (not isinstance(capture.get("host_boot_id"), str) or not capture["host_boot_id"]
            or capture["host_boot_id"] != readback.get("host_boot_id")):
        raise WristServoError("wrist/readback host boot differs")
    stamp = capture.get("timestamp_ns")
    start, finish = (measured.get(f"position_{part}_monotonic_ns") for part in ("started", "finished"))
    if (any(type(v) is not int for v in (stamp, start, finish, now_ns))
            or not 0 < start <= finish <= now_ns or finish-start > 100_000_000
            or not 0 <= now_ns-stamp <= 1_000_000_000
            or max(start-stamp, stamp-finish, 0) > 100_000_000):
        raise StaleObservationError("wrist frame/readback timing missing or outside 100ms association")
    frame = Path(capture["frame_path"])
    frame_sha = hashlib.sha256(frame.read_bytes()).hexdigest()
    if frame_sha != capture.get("frame_sha256"):
        raise WristServoError("native wrist frame changed after acquisition")
    association = {"frame_timestamp_ns":stamp, "read_start_ns":start, "read_finish_ns":finish,
                   "interval_gap_ns":max(start-stamp, stamp-finish, 0),
                   "exact_exposure_state_verified":False}
    return {"schema_version":"dapier.wrist-observation.v1", "side":"left",
            "clock":"host_monotonic_ns", "timestamp_ns":stamp,
            "measured_state_sha256":measured_source["sha256"],
            "measured_q_model_rad":list(measured_model_rad),
            "frame_source":{"path":str(frame), "sha256":frame_sha},
            "detection":document["detection"], "target_uv":document["target_uv"],
            "capture_source":document["capture_source"], "time_association":association}


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
