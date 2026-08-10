"""DAPIER-owned contracts for the SO-101 sim-first learning path."""

from .embodiment import SO101_CHANNEL_NAMES, EmbodimentSpec, so101_new_calibration_spec
from .protocols import Frame, FrameContractError, Leader, validate_frame

__all__ = [
    "EmbodimentSpec",
    "Frame",
    "FrameContractError",
    "Leader",
    "SO101_CHANNEL_NAMES",
    "so101_new_calibration_spec",
    "validate_frame",
]

__version__ = "0.2.0"
