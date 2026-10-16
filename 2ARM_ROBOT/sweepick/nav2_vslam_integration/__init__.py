"""DAPIER C — VSLAM/NAV2 × Sweepick Visual Servo Integration package.

Connects the Navigation stack (Dapier_project_visaul_slam) and the Sweepick
Manipulation stack (DAPIER/2ARM_ROBOT/sweepick) under strict P0/P1/P2 contracts:
  DETECT -> NAVIGATE -> ARRIVED -> BASE_STOPPED -> REOBSERVE
         -> MANIPULATION_READY -> ARM_TASK -> RESULT

Boundaries:
- Does NOT modify Team A/B Native, Console, or Motor Writer files.
- Does NOT generate Motor/Torque authority or auto-START sessions.
- Does NOT fabricate unverified Base-to-Arm TF transforms.
"""

from .contracts import (
    ALLOWED_STAGE_TRANSITIONS,
    ApprovedSweepickScope,
    BaseStopEvidence,
    CameraObservation3D,
    CoordinateFrameContract,
    HardwareReadinessState,
    IntegrationOutcomeRecord,
    IntegrationStage,
    ManipulationResultStatus,
    NavigationArrivalEvidence,
    ReobserveVerdict,
    TaskManagerErrorReport,
    validate_stage_transition,
)
from .reobserve_gate import (
    BaseStopVerifier,
    PostStopReobserveGate,
    ReobserveGateConfig,
)
from .sweepick_adapter import (
    SweepickAdapterResult,
    SweepickExecutionAdapter,
)
from .pipeline_coordinator import (
    VslamSweepickCoordinator,
)
from .navigation_bridge import (
    NavigationManipulationBridge,
    transform_optical_pose_to_base_footprint,
)

__all__ = [
    "ALLOWED_STAGE_TRANSITIONS",
    "ApprovedSweepickScope",
    "BaseStopEvidence",
    "BaseStopVerifier",
    "CameraObservation3D",
    "CoordinateFrameContract",
    "HardwareReadinessState",
    "IntegrationOutcomeRecord",
    "IntegrationStage",
    "ManipulationResultStatus",
    "NavigationArrivalEvidence",
    "NavigationManipulationBridge",
    "PostStopReobserveGate",
    "ReobserveGateConfig",
    "ReobserveVerdict",
    "SweepickAdapterResult",
    "SweepickExecutionAdapter",
    "TaskManagerErrorReport",
    "VslamSweepickCoordinator",
    "transform_optical_pose_to_base_footprint",
    "validate_stage_transition",
]
