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
    TerminalEvidenceClassification,
    active_from_console_run_record,
    classify_terminal_evidence,
)
from .pipeline_coordinator import (
    BLOCK_ORIGIN_INTEGRATION_GATE,
    BLOCK_ORIGIN_SWEEPICK_CONSOLE,
    BLOCK_ORIGIN_SWEEPICK_NO_RUN,
    OperatorRecoveryDecision,
    VslamSweepickCoordinator,
    console_recovery_evidence,
)
from .tf_provenance import (
    DetectionProvenance,
    SensorFrameLedger,
    VerifiedTransform,
    build_post_stop_observation,
    compose_static_chain,
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
    # C-2
    "BLOCK_ORIGIN_INTEGRATION_GATE",
    "BLOCK_ORIGIN_SWEEPICK_CONSOLE",
    "BLOCK_ORIGIN_SWEEPICK_NO_RUN",
    "DetectionProvenance",
    "OperatorRecoveryDecision",
    "SensorFrameLedger",
    "TerminalEvidenceClassification",
    "VerifiedTransform",
    "active_from_console_run_record",
    "build_post_stop_observation",
    "classify_terminal_evidence",
    "compose_static_chain",
    "console_recovery_evidence",
]
