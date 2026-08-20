"""DYNA-lite data tools for the DAPIER shoe-sorting robot."""

from shoe_sorting_data.act_interchange import (
    ACT_INTERCHANGE_SCHEMA_VERSION,
    ActInterchangeReport,
    export_act_interchange,
    verify_act_interchange,
)
from shoe_sorting_data.camera_payload import (
    CAMERA_PAYLOAD_CONTRACT_VERSION,
    CameraFramePayload,
    CameraPayloadError,
    read_camera_payload,
    verify_camera_payload,
    write_camera_payload,
)
from shoe_sorting_data.contract import (
    EPISODE_SCHEMA_VERSION,
    build_manifest,
    load_manifest,
    save_manifest,
    validate_manifest,
)
from shoe_sorting_data.quality import ValidationReport, validate_episode
from shoe_sorting_data.lerobot_v3_encoder import (
    NATIVE_ENCODER_SCHEMA_VERSION,
    NativeEncoderDependencyError,
    NativeEncoderPlan,
    build_native_encoder_plan,
    encode_native_lerobot_v3,
    native_dependency_status,
)
from shoe_sorting_data.native_act_smoke import (
    NATIVE_ACT_SMOKE_SCHEMA_VERSION,
    NativeActSmokeError,
    native_act_dependency_status,
    run_native_act_smoke,
)
from shoe_sorting_data.offline_evaluator import (
    OFFLINE_EVAL_INPUT_SCHEMA_VERSION,
    OFFLINE_EVAL_REPORT_SCHEMA_VERSION,
    OfflineEvaluationError,
    build_offline_evaluator_fixture,
    evaluate_action_chunks,
)
from shoe_sorting_data.rollout_safety import (
    ROLLOUT_SAFETY_SCHEMA_VERSION,
    ROLLOUT_TRACE_SCHEMA_VERSION,
    JDcobotRos2DryRunAdapter,
    SafetySupervisor,
    SafetyContractError,
    build_rollout_safety_fixture,
    run_rollout_safety_smoke,
    supervise_action,
    validate_safety_config,
)
from shoe_sorting_data.perception_exemplar import (
    add_perception_exemplar,
    build_perception_registry,
    match_perception_exemplar,
)
from shoe_sorting_data.synthetic import generate_dataset, generate_episode
from shoe_sorting_data.skill_exemplar import (
    audit_exemplar_leakage,
    build_skill_exemplar,
    retrieve_skill_exemplars,
)

__all__ = [
    "ACT_INTERCHANGE_SCHEMA_VERSION",
    "CAMERA_PAYLOAD_CONTRACT_VERSION",
    "EPISODE_SCHEMA_VERSION",
    "NATIVE_ENCODER_SCHEMA_VERSION",
    "NATIVE_ACT_SMOKE_SCHEMA_VERSION",
    "OFFLINE_EVAL_INPUT_SCHEMA_VERSION",
    "OFFLINE_EVAL_REPORT_SCHEMA_VERSION",
    "ROLLOUT_SAFETY_SCHEMA_VERSION",
    "ROLLOUT_TRACE_SCHEMA_VERSION",
    "ActInterchangeReport",
    "CameraFramePayload",
    "CameraPayloadError",
    "NativeEncoderDependencyError",
    "NativeActSmokeError",
    "OfflineEvaluationError",
    "JDcobotRos2DryRunAdapter",
    "SafetySupervisor",
    "SafetyContractError",
    "NativeEncoderPlan",
    "ValidationReport",
    "add_perception_exemplar",
    "build_perception_registry",
    "build_manifest",
    "build_native_encoder_plan",
    "build_offline_evaluator_fixture",
    "build_rollout_safety_fixture",
    "build_skill_exemplar",
    "audit_exemplar_leakage",
    "export_act_interchange",
    "encode_native_lerobot_v3",
    "evaluate_action_chunks",
    "generate_dataset",
    "generate_episode",
    "load_manifest",
    "match_perception_exemplar",
    "native_dependency_status",
    "native_act_dependency_status",
    "read_camera_payload",
    "save_manifest",
    "retrieve_skill_exemplars",
    "run_rollout_safety_smoke",
    "run_native_act_smoke",
    "validate_episode",
    "validate_safety_config",
    "validate_manifest",
    "verify_act_interchange",
    "verify_camera_payload",
    "write_camera_payload",
    "supervise_action",
]
