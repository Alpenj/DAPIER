"""C-2 P0-3: TF2 and sensor-provenance contracts.

Rules (DAPIER C-2 handoff, 2026-10-16):
- A transform used as REAL evidence must come from a TF2 lookup at the message time
  (`source == TF_SOURCE_LOOKUP`). Constants copied from a URDF, a launch file or a
  report are never REAL evidence; they may only be used as labelled MOCK data.
- A 3D observation used after BASE_STOPPED must carry detector provenance
  (sensor stamp, receive index, EMA / persistence-latch flags, anchor source,
  inlier count, RMS). `object_detector_3d.py` publishes a `PoseStamped` whose
  header stamp is the publish time and whose position is an EMA-filtered,
  persistence-latched value, so a new `PoseStamped` is NOT a new camera detection.
  Without a provenance sidecar the observation is refused with
  `POST_STOP_OBSERVATION_UNVERIFIED`.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np

from .contracts import CameraObservation3D


TF_SOURCE_LOOKUP = "TF2_LOOKUP_AT_MESSAGE_TIME"
TF_SOURCE_MOCK = "MOCK_TF_FOR_TEST"

PROVENANCE_SOURCE_SIDECAR = "DETECTOR_PROVENANCE_SIDECAR"
PROVENANCE_SOURCE_POSE_HEADER_ONLY = "POSE_HEADER_ONLY"
PROVENANCE_SOURCE_MOCK = "MOCK_PROVENANCE_FOR_TEST"

REJECT_OBSERVATION_UNVERIFIED = "POST_STOP_OBSERVATION_UNVERIFIED"
REJECT_TF_UNAVAILABLE = "TF_UNAVAILABLE_AT_MESSAGE_TIME"
REJECT_TF_STAMP_MISMATCH = "TF_STAMP_DOES_NOT_MATCH_SENSOR_STAMP"
REJECT_TF_FRAME_MISMATCH = "TF_FRAME_CHAIN_MISMATCH"
REJECT_TF_NOT_REAL = "TF_SOURCE_NOT_A_LOOKUP"
REJECT_REPUBLISHED = "REPUBLISHED_OBSERVATION_SAME_SENSOR_FRAME"
REJECT_MOCK_NOT_ALLOWED = "MOCK_EVIDENCE_NOT_ALLOWED_IN_REAL_MODE"

EVIDENCE_REAL = "REAL"
EVIDENCE_MOCK = "MOCK"

# Maximum distance between the TF lookup time and the sensor stamp the observation claims.
TF_STAMP_TOLERANCE_S = 0.05


class TransformRejected(Exception):
    """Raised when a transform cannot be used as evidence. `code` is one of the REJECT_* constants."""

    def __init__(self, code: str, detail: str = "") -> None:
        super().__init__(f"{code}: {detail}" if detail else code)
        self.code = code
        self.detail = detail


def quaternion_to_matrix(q_xyzw: Tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """ROS (static_transform_publisher) convention: q = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


@dataclass(frozen=True)
class VerifiedTransform:
    """A `parent_frame <- child_frame` transform with its origin recorded.

    `evidence_grade` is REAL only when `source == TF_SOURCE_LOOKUP`; anything else
    (URDF constants, launch-file constants, hand-typed numbers) is MOCK and is rejected
    by `build_post_stop_observation(..., allow_mock=False)`.
    """

    parent_frame: str
    child_frame: str
    stamp_s: float
    clock_id: str
    translation_m: Tuple[float, float, float]
    rotation_xyzw: Tuple[float, float, float, float]
    source: str
    lookup_time_s: Optional[float] = None
    tf_authority: str = ""

    def __post_init__(self) -> None:
        if self.source not in (TF_SOURCE_LOOKUP, TF_SOURCE_MOCK):
            raise ValueError(
                f"VerifiedTransform.source must be {TF_SOURCE_LOOKUP} or {TF_SOURCE_MOCK}; got {self.source!r}. "
                "Constants are not accepted as transform evidence."
            )
        if len(self.translation_m) != 3 or len(self.rotation_xyzw) != 4:
            raise ValueError("translation must have 3 and rotation 4 components")

    @property
    def evidence_grade(self) -> str:
        return EVIDENCE_REAL if self.source == TF_SOURCE_LOOKUP else EVIDENCE_MOCK

    def rotation_matrix(self) -> np.ndarray:
        return quaternion_to_matrix(self.rotation_xyzw)

    def apply_point(self, p_child: Tuple[float, float, float]) -> Tuple[float, float, float]:
        p = self.rotation_matrix() @ np.asarray(p_child, dtype=float).reshape(3) + np.asarray(self.translation_m, dtype=float)
        return (float(p[0]), float(p[1]), float(p[2]))

    def apply_direction(self, d_child: Tuple[float, float, float]) -> Tuple[float, float, float]:
        d = self.rotation_matrix() @ np.asarray(d_child, dtype=float).reshape(3)
        n = float(np.linalg.norm(d))
        if n > 1e-9:
            d = d / n
        return (float(d[0]), float(d[1]), float(d[2]))

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["evidence_grade"] = self.evidence_grade
        return data


def compose_static_chain(
    links: List[Tuple[str, str, Tuple[float, float, float], Tuple[float, float, float]]],
    *,
    stamp_s: float,
    clock_id: str,
    source: str = TF_SOURCE_MOCK,
) -> VerifiedTransform:
    """Compose `(parent, child, xyz, rpy)` links into one parent(first) <- child(last) transform.

    Only for building labelled MOCK transforms in tests and mock graphs. Never call with
    `source=TF_SOURCE_LOOKUP`: a composed constant chain is not a lookup.
    """
    if source == TF_SOURCE_LOOKUP:
        raise ValueError("compose_static_chain cannot produce a lookup-grade transform")
    if not links:
        raise ValueError("empty chain")
    rot = np.eye(3)
    trans = np.zeros(3)
    for parent, child, xyz, rpy in links:
        r_link = quaternion_to_matrix(rpy_to_quaternion(*rpy))
        trans = trans + rot @ np.asarray(xyz, dtype=float)
        rot = rot @ r_link
    # rotation matrix -> quaternion
    m = rot
    tr = float(np.trace(m))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw, qx, qy, qz = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        qw, qx, qy, qz = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        qw, qx, qy, qz = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        qw, qx, qy, qz = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return VerifiedTransform(
        parent_frame=links[0][0],
        child_frame=links[-1][1],
        stamp_s=stamp_s,
        clock_id=clock_id,
        translation_m=(float(trans[0]), float(trans[1]), float(trans[2])),
        rotation_xyzw=(float(qx), float(qy), float(qz), float(qw)),
        source=source,
        tf_authority="compose_static_chain (constants, MOCK only)",
    )


ANCHOR_SOURCES = ("qwen_fresh", "hsv", "yolo_world", "qwen_latched", "marker", "unknown")


@dataclass(frozen=True)
class DetectionProvenance:
    """Where a 3D target observation came from.

    Fields that the current `object_detector_3d.py` topics cannot supply are `None`;
    `is_verified()` is False while any of them is missing, which refuses the observation.
    """

    source_topic: str
    message_stamp_s: float
    receive_index: int
    clock_id: str
    provenance_source: str
    publisher_node: Optional[str] = None
    sensor_stamp_s: Optional[float] = None
    anchor_source: Optional[str] = None
    is_ema_filtered: Optional[bool] = None
    is_persistence_latched: Optional[bool] = None
    inlier_count: Optional[int] = None
    rms_m: Optional[float] = None
    detector_mode: Optional[str] = None
    qwen_bbox_stamp_s: Optional[float] = None
    notes: Tuple[str, ...] = field(default_factory=tuple)

    REQUIRED_FOR_VERIFICATION = (
        "sensor_stamp_s",
        "anchor_source",
        "is_ema_filtered",
        "is_persistence_latched",
        "inlier_count",
        "rms_m",
        "detector_mode",
    )

    def missing_fields(self) -> List[str]:
        return [name for name in self.REQUIRED_FOR_VERIFICATION if getattr(self, name) is None]

    @property
    def evidence_grade(self) -> str:
        return EVIDENCE_MOCK if self.provenance_source == PROVENANCE_SOURCE_MOCK else EVIDENCE_REAL

    def is_verified(self) -> bool:
        if self.provenance_source not in (PROVENANCE_SOURCE_SIDECAR, PROVENANCE_SOURCE_MOCK):
            return False
        return not self.missing_fields()

    def observation_id(self) -> str:
        stamp = self.sensor_stamp_s if self.sensor_stamp_s is not None else self.message_stamp_s
        kind = "sensor" if self.sensor_stamp_s is not None else "publish"
        return f"{self.source_topic}@{kind}:{int(round(stamp * 1e9))}"

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["missing_fields"] = self.missing_fields()
        data["verified"] = self.is_verified()
        data["evidence_grade"] = self.evidence_grade
        return data


class SensorFrameLedger:
    """Remembers which sensor frames have already been turned into an observation.

    `object_detector_3d.py` keeps publishing the same EMA/latched value at 20 Hz; a second
    `PoseStamped` built from an already consumed sensor frame is a re-publish, not a new
    camera detection, and is refused.
    """

    def __init__(self) -> None:
        self._consumed: Set[str] = set()

    def consume(self, observation_id: str) -> bool:
        """Return True the first time an id is seen, False for a re-publish."""
        if observation_id in self._consumed:
            return False
        self._consumed.add(observation_id)
        return True

    def already_consumed(self, observation_id: str) -> bool:
        return observation_id in self._consumed


def build_post_stop_observation(
    *,
    position_in_sensor_frame_m: Tuple[float, float, float],
    normal_in_sensor_frame: Tuple[float, float, float],
    sensor_frame_id: str,
    provenance: DetectionProvenance,
    transform: Optional[VerifiedTransform],
    ledger: Optional[SensorFrameLedger] = None,
    allow_mock: bool = False,
    target_frame: str = "base_footprint",
) -> Tuple[Optional[CameraObservation3D], Tuple[str, ...]]:
    """Turn a detector message into a `CameraObservation3D`, or refuse with reason codes.

    Returns `(observation, rejected_reasons)`. `observation` is None when any reason is present.
    """
    rejected: List[str] = []

    if not provenance.is_verified():
        rejected.append(
            f"{REJECT_OBSERVATION_UNVERIFIED}:source={provenance.provenance_source};missing={','.join(provenance.missing_fields()) or '-'}"
        )
    if provenance.evidence_grade == EVIDENCE_MOCK and not allow_mock:
        rejected.append(f"{REJECT_MOCK_NOT_ALLOWED}:provenance")

    if transform is None:
        rejected.append(REJECT_TF_UNAVAILABLE)
    else:
        if transform.source != TF_SOURCE_LOOKUP and not allow_mock:
            rejected.append(f"{REJECT_TF_NOT_REAL}:{transform.source}")
        if transform.parent_frame != target_frame or transform.child_frame != sensor_frame_id:
            rejected.append(
                f"{REJECT_TF_FRAME_MISMATCH}:have={transform.parent_frame}<-{transform.child_frame};"
                f"need={target_frame}<-{sensor_frame_id}"
            )
        if provenance.sensor_stamp_s is not None and math.isfinite(provenance.sensor_stamp_s):
            if not math.isfinite(transform.stamp_s) or abs(transform.stamp_s - provenance.sensor_stamp_s) > TF_STAMP_TOLERANCE_S:
                rejected.append(
                    f"{REJECT_TF_STAMP_MISMATCH}:tf={transform.stamp_s:.3f};sensor={provenance.sensor_stamp_s:.3f}"
                )
        if transform.clock_id != provenance.clock_id:
            rejected.append(f"CLOCK_DOMAIN_MISMATCH:{transform.clock_id}!={provenance.clock_id}")

    observation_id = provenance.observation_id()
    if ledger is not None and ledger.already_consumed(observation_id):
        rejected.append(f"{REJECT_REPUBLISHED}:{observation_id}")

    if rejected:
        return None, tuple(rejected)

    assert transform is not None
    p_base = transform.apply_point(position_in_sensor_frame_m)
    n_base = transform.apply_direction(normal_in_sensor_frame)
    observation = CameraObservation3D(
        observation_id=observation_id,
        frame_seq=provenance.receive_index,
        captured_at_s=float(provenance.sensor_stamp_s),  # verified above
        clock_id=provenance.clock_id,
        frame_id=target_frame,
        position_in_base_footprint_m=p_base,
        normal_in_base_footprint=n_base,
        inlier_count=int(provenance.inlier_count),
        rms_m=float(provenance.rms_m),
        detector_mode=str(provenance.detector_mode),
        is_ema_filtered_from_motion=bool(provenance.is_ema_filtered),
        is_persistence_latched=bool(provenance.is_persistence_latched),
        qwen_bbox_captured_at_s=provenance.qwen_bbox_stamp_s,
    )
    if ledger is not None:
        ledger.consume(observation_id)
    return observation, ()
