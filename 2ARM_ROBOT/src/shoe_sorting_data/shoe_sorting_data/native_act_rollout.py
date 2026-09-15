"""Bridge DAPIER-native ACT inference through safety to an SO-101 command.

This module never opens a serial port.  A caller may inject an already-open,
identity-checked bus only after the supervisor explicitly authorizes hardware.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any, Mapping

from shoe_sorting_data.dapier_native_act import infer, run_smoke
from shoe_sorting_data.rollout_safety import (
    SafetyContractError,
    SafetySupervisor,
    build_rollout_safety_fixture,
)


NATIVE_ACT_ROLLOUT_SCHEMA_VERSION = "dapier.native-act-rollout.v0.1"
SO101_MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class SO101ArmAdapter:
    """Translate calibrated robot-frame actions; NOT a SIM joint calibration map."""

    def __init__(self, *, side: str = "right", left_hold_tolerance: float = 1e-6) -> None:
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.side = side
        self.active = slice(0, 6) if side == "left" else slice(6, 12)
        self.held = slice(6, 12) if side == "left" else slice(0, 6)
        self.held_name = "right" if side == "left" else "left"
        if not math.isfinite(left_hold_tolerance) or left_hold_tolerance < 0:
            raise ValueError("left_hold_tolerance must be finite and non-negative")
        self.left_hold_tolerance = left_hold_tolerance

    def dispatch(
        self,
        decision: Mapping[str, Any],
        measured_action: list[float],
        *,
        bus: Any | None = None,
    ) -> dict[str, Any]:
        action = decision.get("approved_action")
        if decision.get("safety_passed") is not True or not isinstance(action, list):
            return {
                "status": "NOT_DISPATCHED",
                "published": False,
                "reason": "supervisor_rejected",
                "executed_action": None,
            }
        if len(action) != 12 or len(measured_action) != 12:
            raise SafetyContractError("SO-101 policy and measured actions must contain 12 values")
        if any(
            not math.isfinite(float(value))
            for value in (*action, *measured_action)
        ):
            raise SafetyContractError("SO-101 actions must be finite")
        if any(
            abs(float(target) - float(current)) > self.left_hold_tolerance
            for target, current in zip(action[self.held], measured_action[self.held], strict=True)
        ):
            raise SafetyContractError(f"{self.side}-arm phase requires the {self.held_name} arm to hold its measured pose")
        gripper = float(action[self.active][-1])
        if not 0.0 <= gripper <= 1.0:
            raise SafetyContractError(f"{self.side} gripper action must be normalized to [0, 1]")
        command = {
            name: (math.degrees(float(value)) if index < 5 else gripper * 100.0)
            for index, (name, value) in enumerate(
                zip(SO101_MOTOR_NAMES, action[self.active], strict=True)
            )
        }
        if bus is None:
            return {
                "status": "SIMULATED_ONLY",
                "published": False,
                "reason": "no_motor_bus_injected",
                "would_write": {"register": "Goal_Position", "values": command},
                "executed_action": None,
            }
        if decision.get("hardware_dispatch_authorized") is not True:
            raise SafetyContractError("supervisor did not authorize hardware dispatch")
        bus.sync_write("Goal_Position", command, num_retry=2)
        return {
            "status": "DISPATCHED",
            "published": True,
            "reason": f"supervisor_authorized_{self.side}_arm_step",
            "executed_action": list(action),
            "written": {"register": "Goal_Position", "values": command},
        }

SO101RightArmAdapter = SO101ArmAdapter  # Existing callers retain the right-arm default.


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
    execute_arm: str = "right",
) -> dict[str, Any]:
    """Infer action 0, then submit it unchanged to an active dry-run supervisor."""

    adapter = SO101ArmAdapter(side=execute_arm)
    if supervisor.state != "ACTIVE":
        raise SafetyContractError("native ACT rollout requires an already ACTIVE safety supervisor")
    if isinstance(proposal_sequence, bool) or proposal_sequence < 0:
        raise ValueError("proposal_sequence must be non-negative")
    # Freeze source metadata before inference; current safety timestamps remain separate.
    snapshot = dict(snapshot)
    source = snapshot.get("policy_source_observation")
    if isinstance(source, Mapping):
        source = dict(source)
        snapshot["policy_source_observation"] = source
    inference = infer(dataset_root, checkpoint_path, item=item, device=device)
    if inference.get("control_authorized") is not False:
        raise ValueError("native ACT inference must remain a non-authorizing proposal")
    action_chunk = inference["action_chunk"]
    if not action_chunk or len(action_chunk[0]) != 12:
        raise ValueError("native ACT inference must return a non-empty 12-DoF action chunk")
    measured_action = list(snapshot.get("measured_action", []))
    if len(measured_action) != 12:
        raise ValueError("right-arm rollout requires a 12-DoF measured action")
    raw_policy_action = list(action_chunk[0])
    phase_action = list(measured_action)
    phase_action[adapter.active] = raw_policy_action[adapter.active]
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
        "source_observation_monotonic_ns": source.get("capture_monotonic_ns") if isinstance(source, Mapping) else None,
        "created_monotonic_ns": created_monotonic_ns,
        "action": phase_action,
        "raw_policy_action": raw_policy_action,
        "phase_mask": f"hold_{adapter.held_name}_execute_{execute_arm}",
    }
    snapshot_for_supervision = dict(snapshot)
    # ``now`` is the supervisor evaluation clock, not a sensor timestamp.  Set
    # it after inference so proposal freshness cannot be evaluated in the past.
    snapshot_for_supervision["now_monotonic_ns"] = created_monotonic_ns
    decision = supervisor.evaluate(proposal, snapshot_for_supervision)
    adapter_result = adapter.dispatch(
        decision,
        measured_action,
    )
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
            "policy_source_observation": {
                "version": 1,
                "observation_id": f"{preview['episode_id']}:{preview['frame_index']}",
                "frame_index": preview["frame_index"],
                "capture_monotonic_ns": now_ns - 1_000_000,
                "receive_monotonic_ns": now_ns - 1_000_000,
            },
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
