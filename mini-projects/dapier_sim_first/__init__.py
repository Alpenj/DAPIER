"""DAPIER-owned contracts for the SO-101 sim-first learning path."""

from .embodiment import SO101_CHANNEL_NAMES, EmbodimentSpec, so101_new_calibration_spec
from .digital_twin import (
    DigitalTwinContractError,
    JointTrace,
    TwinThresholds,
    evaluate_digital_twin,
)
from .protocols import Frame, FrameContractError, Leader, validate_frame

__all__ = [
    "EmbodimentSpec",
    "DigitalTwinContractError",
    "Frame",
    "FrameContractError",
    "Leader",
    "JointTrace",
    "SO101_CHANNEL_NAMES",
    "TwinThresholds",
    "evaluate_digital_twin",
    "so101_new_calibration_spec",
    "validate_frame",
]

__version__ = "0.3.0"
