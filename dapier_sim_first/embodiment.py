"""SO-101 ordered-channel and unit conversion contract used by G0."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
import json
from math import degrees, isfinite, radians
from typing import Iterable

SO101_CHANNEL_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

SO101_ARM_ROLES = ("left", "right")

SO101_ACTION_UNITS = (
    "degree",
    "degree",
    "degree",
    "degree",
    "degree",
    "range_0_100",
)

SO101_SIM_UNITS = ("radian",) * 6

# These ranges are the six named joints in the pinned SO-101 new-calibration
# MJCF.  G0 compares every value with the model that MuJoCo actually loads.
SO101_NEW_CALIBRATION_SIM_LOWER = (
    -1.9198621771937616,
    -1.7453292519943224,
    -1.69,
    -1.6580628494556928,
    -2.7438472969992493,
    -0.17453297762778586,
)

SO101_NEW_CALIBRATION_SIM_UPPER = (
    1.9198621771937634,
    1.7453292519943366,
    1.69,
    1.6580627293335335,
    2.841206309382605,
    1.7453291995659765,
)

READBACK_SOURCES = frozenset({"sim_follower_readback"})


def _finite_tuple(values: Iterable[float], *, width: int) -> tuple[float, ...]:
    result = tuple(values)
    if len(result) != width:
        raise ValueError(f"expected {width} values, got {len(result)}")
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not isfinite(value)
        for value in result
    ):
        raise ValueError("values must contain only finite numbers")
    return tuple(float(value) for value in result)


def _inside_with_roundoff(value: float, lower: float, upper: float) -> bool:
    tolerance = 1e-12
    return lower - tolerance <= value <= upper + tolerance


def _normalize_roundoff(value: float, lower: float, upper: float) -> float:
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


@dataclass(frozen=True, slots=True)
class EmbodimentSpec:
    embodiment_id: str
    embodiment_revision: str
    channel_names: tuple[str, ...]
    action_units: tuple[str, ...]
    sim_units: tuple[str, ...]
    calibration_id: str
    sim_lower: tuple[float, ...]
    sim_upper: tuple[float, ...]

    def __post_init__(self) -> None:
        width = len(self.channel_names)
        if width == 0 or len(set(self.channel_names)) != width:
            raise ValueError("channel_names must be nonempty and unique")
        for field in (
            self.action_units,
            self.sim_units,
            self.sim_lower,
            self.sim_upper,
        ):
            if len(field) != width:
                raise ValueError("embodiment fields must have the same width")
        if any(
            lower >= upper
            for lower, upper in zip(self.sim_lower, self.sim_upper, strict=True)
        ):
            raise ValueError(
                "every simulator lower bound must be below its upper bound"
            )
        if (
            not isinstance(self.calibration_id, str)
            or not self.calibration_id.startswith("sha256:")
            or len(self.calibration_id) != 71
            or any(
                character not in "0123456789abcdefABCDEF"
                for character in self.calibration_id[7:]
            )
        ):
            raise ValueError("calibration_id must be sha256:<64-hex>")

    def units_for_source(self, source: str) -> tuple[str, ...]:
        return self.sim_units if source in READBACK_SOURCES else self.action_units

    @property
    def action_lower(self) -> tuple[float, ...]:
        return tuple(degrees(value) for value in self.sim_lower[:5]) + (0.0,)

    @property
    def action_upper(self) -> tuple[float, ...]:
        return tuple(degrees(value) for value in self.sim_upper[:5]) + (100.0,)

    def action_to_sim(self, values: Iterable[float]) -> tuple[float, ...]:
        action = _finite_tuple(values, width=len(self.channel_names))
        for index, (value, lower, upper) in enumerate(
            zip(action, self.action_lower, self.action_upper, strict=True)
        ):
            if value < lower or value > upper:
                raise ValueError(
                    f"action value for {self.channel_names[index]} is outside the declared bounds"
                )

        gripper_low = self.sim_lower[5]
        gripper_span = self.sim_upper[5] - gripper_low
        body = tuple(
            _normalize_roundoff(
                radians(value), self.sim_lower[index], self.sim_upper[index]
            )
            for index, value in enumerate(action[:5])
        )
        return body + (gripper_low + action[5] / 100.0 * gripper_span,)

    def sim_to_action(self, values: Iterable[float]) -> tuple[float, ...]:
        sim = _finite_tuple(values, width=len(self.channel_names))
        for index, (value, lower, upper) in enumerate(
            zip(sim, self.sim_lower, self.sim_upper, strict=True)
        ):
            if not _inside_with_roundoff(value, lower, upper):
                raise ValueError(
                    f"sim value for {self.channel_names[index]} is outside the declared bounds"
                )

        sim = tuple(
            _normalize_roundoff(value, lower, upper)
            for value, lower, upper in zip(
                sim, self.sim_lower, self.sim_upper, strict=True
            )
        )

        gripper_low = self.sim_lower[5]
        gripper_span = self.sim_upper[5] - gripper_low
        return tuple(degrees(value) for value in sim[:5]) + (
            (sim[5] - gripper_low) / gripper_span * 100.0,
        )

    def bounds_digest(self) -> str:
        payload = {
            "action_lower": self.action_lower,
            "action_units": self.action_units,
            "action_upper": self.action_upper,
            "channel_names": self.channel_names,
            "sim_lower": self.sim_lower,
            "sim_units": self.sim_units,
            "sim_upper": self.sim_upper,
        }
        encoded = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"), sort_keys=True
        ).encode()
        return f"sha256:{sha256(encoded).hexdigest()}"


def so101_new_calibration_spec(calibration_id: str) -> EmbodimentSpec:
    return EmbodimentSpec(
        embodiment_id="so101-single-arm",
        embodiment_revision="so101-new-calibration-v1",
        channel_names=SO101_CHANNEL_NAMES,
        action_units=SO101_ACTION_UNITS,
        sim_units=SO101_SIM_UNITS,
        calibration_id=calibration_id,
        sim_lower=SO101_NEW_CALIBRATION_SIM_LOWER,
        sim_upper=SO101_NEW_CALIBRATION_SIM_UPPER,
    )


@dataclass(frozen=True, slots=True)
class BimanualEmbodimentSpec:
    """Two independently calibrated SO-101 arms in deterministic left/right order."""

    left: EmbodimentSpec
    right: EmbodimentSpec

    def __post_init__(self) -> None:
        expected_contract = (
            ("embodiment_id", "so101-single-arm"),
            ("embodiment_revision", "so101-new-calibration-v1"),
            ("channel_names", SO101_CHANNEL_NAMES),
            ("action_units", SO101_ACTION_UNITS),
            ("sim_units", SO101_SIM_UNITS),
            ("sim_lower", SO101_NEW_CALIBRATION_SIM_LOWER),
            ("sim_upper", SO101_NEW_CALIBRATION_SIM_UPPER),
        )
        for role, spec in zip(SO101_ARM_ROLES, (self.left, self.right), strict=True):
            mismatches = tuple(
                field_name
                for field_name, expected in expected_contract
                if getattr(spec, field_name) != expected
            )
            if mismatches:
                mismatch_list = ", ".join(mismatches)
                raise ValueError(
                    f"{role} arm must use the canonical SO-101 new-calibration "
                    f"contract; mismatched fields: {mismatch_list}"
                )

    @property
    def embodiment_id(self) -> str:
        return "so101-bimanual-two-arm"

    @property
    def embodiment_revision(self) -> str:
        return "so101-bimanual-new-calibration-v1"

    @property
    def channel_names(self) -> tuple[str, ...]:
        return tuple(
            f"{role}_{name}"
            for role, spec in zip(SO101_ARM_ROLES, (self.left, self.right), strict=True)
            for name in spec.channel_names
        )

    @property
    def action_units(self) -> tuple[str, ...]:
        return self.left.action_units + self.right.action_units

    @property
    def sim_units(self) -> tuple[str, ...]:
        return self.left.sim_units + self.right.sim_units

    @property
    def calibration_ids(self) -> tuple[str, str]:
        return self.left.calibration_id, self.right.calibration_id

    @property
    def calibration_id(self) -> str:
        encoded = json.dumps(self.calibration_ids, separators=(",", ":")).encode()
        return f"sha256:{sha256(encoded).hexdigest()}"

    @property
    def action_lower(self) -> tuple[float, ...]:
        return self.left.action_lower + self.right.action_lower

    @property
    def action_upper(self) -> tuple[float, ...]:
        return self.left.action_upper + self.right.action_upper

    @property
    def sim_lower(self) -> tuple[float, ...]:
        return self.left.sim_lower + self.right.sim_lower

    @property
    def sim_upper(self) -> tuple[float, ...]:
        return self.left.sim_upper + self.right.sim_upper

    def units_for_source(self, source: str) -> tuple[str, ...]:
        return self.sim_units if source in READBACK_SOURCES else self.action_units

    def action_to_sim(self, values: Iterable[float]) -> tuple[float, ...]:
        action = _finite_tuple(values, width=len(self.channel_names))
        split = len(self.left.channel_names)
        return self.left.action_to_sim(action[:split]) + self.right.action_to_sim(
            action[split:]
        )

    def sim_to_action(self, values: Iterable[float]) -> tuple[float, ...]:
        sim = _finite_tuple(values, width=len(self.channel_names))
        split = len(self.left.channel_names)
        return self.left.sim_to_action(sim[:split]) + self.right.sim_to_action(
            sim[split:]
        )

    def bounds_digest(self) -> str:
        payload = {
            "arm_order": SO101_ARM_ROLES,
            "calibration_ids": self.calibration_ids,
            "channel_names": self.channel_names,
            "left_bounds": self.left.bounds_digest(),
            "right_bounds": self.right.bounds_digest(),
        }
        encoded = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"), sort_keys=True
        ).encode()
        return f"sha256:{sha256(encoded).hexdigest()}"


def so101_bimanual_new_calibration_spec(
    left_calibration_id: str,
    right_calibration_id: str,
) -> BimanualEmbodimentSpec:
    return BimanualEmbodimentSpec(
        left=so101_new_calibration_spec(left_calibration_id),
        right=so101_new_calibration_spec(right_calibration_id),
    )
