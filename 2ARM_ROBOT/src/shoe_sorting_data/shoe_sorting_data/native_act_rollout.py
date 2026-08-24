"""Dry-run bridge from DAPIER-native ACT inference to the Stage 5 supervisor.

This module has no ROS imports and cannot publish hardware commands.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
from typing import Any, Mapping

from shoe_sorting_data.dapier_native_act import infer, run_smoke
from shoe_sorting_data.rollout_safety import (
    JDcobotRos2DryRunAdapter,
    SafetyContractError,
    SafetySupervisor,
    build_rollout_safety_fixture,
)


NATIVE_ACT_ROLLOUT_SCHEMA_VERSION = "dapier.native-act-rollout.v0.1"


def checkpoint_sha256(path: str | Path) -> str:
    """Hash the exact checkpoint bytes approved by the safety contract."""

    source = Path(path)
    if not source.is_file():
        raise ValueError(f"checkpoint does not exist: {source}")
    digest = hashlib.sha256()
    with source.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def evaluate_native_act_rollout(
    dataset_root: str | Path,
    checkpoint_path: str | Path,
    *,
    supervisor: SafetySupervisor,
    snapshot: Mapping[str, Any],
    proposal_sequence: int,
    item: int = 0,
    device: str = "cpu",
) -> dict[str, Any]:
    """Infer action 0, then submit it unchanged to an active dry-run supervisor."""

    if supervisor.state != "ACTIVE":
        raise SafetyContractError("native ACT rollout requires an already ACTIVE safety supervisor")
    if isinstance(proposal_sequence, bool) or proposal_sequence < 0:
        raise ValueError("proposal_sequence must be non-negative")
    inference = infer(dataset_root, checkpoint_path, item=item, device=device)
    if inference.get("control_authorized") is not False:
        raise ValueError("native ACT inference must remain a non-authorizing proposal")
    action_chunk = inference["action_chunk"]
    if not action_chunk or len(action_chunk[0]) != 12:
        raise ValueError("native ACT inference must return a non-empty 12-DoF action chunk")
    created_monotonic_ns = time.monotonic_ns()
    checkpoint_hash = checkpoint_sha256(checkpoint_path)
    episode_id = inference["episode_id"]
    proposal = {
        "proposal_id": f"native-act:{episode_id}:{inference['frame_index']}:{proposal_sequence}",
        "policy_query_id": f"native-act-query:{episode_id}:{inference['frame_index']}",
        "chunk_id": f"native-act-chunk:{checkpoint_hash[:12]}:{episode_id}:{inference['frame_index']}",
        "action_index": 0,
        "n_action_steps": 1,
        "proposal_sequence": proposal_sequence,
        "episode_id": episode_id,
        "human_approval_id": supervisor.human_approval_id,
        "hardware_profile_sha256": supervisor.config["expected_hardware_profile_sha256"],
        "policy_checkpoint_sha256": checkpoint_hash,
        "policy_reset_generation": supervisor.policy_reset_generation,
        "source_observation_id": f"{episode_id}:{inference['frame_index']}",
        "source_frame_index": inference["frame_index"],
        "source_observation_monotonic_ns": snapshot.get("observation_monotonic_ns"),
        "created_monotonic_ns": created_monotonic_ns,
        "action": list(action_chunk[0]),
    }
    snapshot_for_supervision = dict(snapshot)
    # ``now`` is the supervisor evaluation clock, not a sensor timestamp.  Set
    # it after inference so proposal freshness cannot be evaluated in the past.
    snapshot_for_supervision["now_monotonic_ns"] = created_monotonic_ns
    decision = supervisor.evaluate(proposal, snapshot_for_supervision)
    adapter_result = JDcobotRos2DryRunAdapter(supervisor.config).dispatch(decision)
    return {
        "schema_version": NATIVE_ACT_ROLLOUT_SCHEMA_VERSION,
        "created_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "scope": "native ACT proposal through safety supervisor and dry-run adapter; no ROS or hardware publish",
        "inference": inference,
        "proposal": proposal,
        "snapshot": snapshot_for_supervision,
        "decision": decision,
        "adapter_result": adapter_result,
        "mock_only": True,
        "hardware_execution": "NOT_ATTEMPTED",
        "control_authorized": False,
        "executed_action": None,
    }


def run_native_act_rollout_smoke(output_root: str | Path) -> dict[str, Any]:
    """Create a native ACT fixture and prove the safe dry-run path never publishes."""

    root = Path(output_root).resolve()
    if root.exists() and any(root.iterdir()):
        raise ValueError(f"smoke output must be absent or empty: {root}")
    root.mkdir(parents=True, exist_ok=True)
    act_receipt = run_smoke(root / "native_act")
    checkpoint = root / "native_act" / act_receipt["checkpoint"]["path"]
    preview = infer(root / "native_act" / "raw", checkpoint)
    action = list(preview["action_chunk"][0])
    config = build_rollout_safety_fixture()
    config["approved_policy_checkpoint_sha256"] = checkpoint_sha256(checkpoint)
    # ponytail: synthetic bounds only; replace with measured joint limits before any live rollout.
    config["joint_lower"] = [value - 0.1 for value in action]
    config["joint_upper"] = [value + 0.1 for value in action]
    config["max_delta_per_step"] = [0.01] * 12
    config["max_observation_age_ms"] = 500.0
    config["max_feedback_age_ms"] = 500.0
    supervisor = SafetySupervisor(config)
    supervisor.configure(config["expected_hardware_profile_sha256"])
    supervisor.arm(episode_id=preview["episode_id"], human_approval_id="approval_native_act_smoke")
    supervisor.activate()
    now_ns = time.monotonic_ns()
    trace = evaluate_native_act_rollout(
        root / "native_act" / "raw",
        checkpoint,
        supervisor=supervisor,
        proposal_sequence=0,
        snapshot={
            "now_monotonic_ns": now_ns,
            "observation_monotonic_ns": now_ns - 1_000_000,
            "feedback_monotonic_ns": now_ns - 1_000_000,
            "measured_action": action,
            "base_velocity": [0.0, 0.0],
            "recent_base_command": [0.0, 0.0],
            "e_stop_healthy": True,
            "watchdog_healthy": True,
            "camera_fresh": True,
            "target_valid": True,
            "operator_authorized": True,
        },
    )
    if not trace["decision"]["safety_passed"] or trace["adapter_result"]["published"]:
        raise RuntimeError("native ACT dry-run smoke failed its safety/no-publish contract")
    output = root / "native_act_rollout_trace.json"
    output.write_text(json.dumps(trace, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return trace
