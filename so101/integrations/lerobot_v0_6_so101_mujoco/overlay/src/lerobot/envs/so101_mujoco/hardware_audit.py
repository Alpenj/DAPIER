# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Read-only, fail-closed inventory gate for physical wrist-camera work."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

HARDWARE_AUDIT_SCHEMA_VERSION = "dapier.so101.physical-wrist-gate.v1"


@dataclass(frozen=True)
class VideoDevice:
    device: str
    name: str


@dataclass(frozen=True)
class HardwareInventory:
    video_devices: tuple[VideoDevice, ...]
    serial_by_id: tuple[str, ...]


def discover_hardware_inventory(
    *,
    dev_root: Path = Path("/dev"),
    sys_root: Path = Path("/sys"),
) -> HardwareInventory:
    """List video and stable serial nodes without opening or commanding them."""
    video_devices: list[VideoDevice] = []
    for device in sorted(dev_root.glob("video*")):
        name_path = sys_root / "class" / "video4linux" / device.name / "name"
        try:
            name = name_path.read_text(encoding="utf-8").strip()
        except (FileNotFoundError, OSError):
            name = "unknown"
        video_devices.append(VideoDevice(device=str(device), name=name))

    serial_root = dev_root / "serial" / "by-id"
    try:
        serial_by_id = tuple(str(path) for path in sorted(serial_root.iterdir()))
    except (FileNotFoundError, OSError):
        serial_by_id = ()
    return HardwareInventory(
        video_devices=tuple(video_devices), serial_by_id=serial_by_id
    )


def build_physical_wrist_gate_receipt(
    inventory: HardwareInventory,
    *,
    expected_camera_name_substrings: tuple[str, ...],
    camera_profile_id: str,
    physical_alignment_verified: bool,
) -> dict[str, Any]:
    """Build a receipt that never grants motion authority or implies a rollout."""
    expected = tuple(
        term.strip().lower() for term in expected_camera_name_substrings if term.strip()
    )
    if not expected:
        raise ValueError(
            "At least one expected wrist-camera name substring is required"
        )
    matched = tuple(
        device
        for device in inventory.video_devices
        if any(term in device.name.lower() for term in expected)
    )
    reasons: list[str] = []
    if not matched:
        reasons.append("expected_wrist_camera_not_detected")
    if not inventory.serial_by_id:
        reasons.append("stable_robot_serial_device_not_detected")
    if not physical_alignment_verified:
        reasons.append("wrist_camera_profile_physical_alignment_unverified")
    ready = not reasons
    return {
        "schema_version": HARDWARE_AUDIT_SCHEMA_VERSION,
        "status": "ready_for_operator_validation" if ready else "blocked",
        "blocking_reasons": reasons,
        "expected_camera_name_substrings": list(expected_camera_name_substrings),
        "camera_profile_id": camera_profile_id,
        "inventory": {
            "video_devices": [asdict(device) for device in inventory.video_devices],
            "serial_by_id": list(inventory.serial_by_id),
        },
        "checks": {
            "expected_wrist_camera_detected": bool(matched),
            "stable_robot_serial_detected": bool(inventory.serial_by_id),
            "physical_camera_alignment_verified": bool(physical_alignment_verified),
        },
        "claims": {
            "device_nodes_opened": False,
            "motor_commands_sent": False,
            "physical_motion_authorized": False,
            "physical_rollout_executed": False,
        },
    }


def write_physical_wrist_gate_receipt(
    output_path: Path, receipt: dict[str, Any]
) -> Path:
    """Persist a read-only audit result for later human review."""
    if receipt.get("schema_version") != HARDWARE_AUDIT_SCHEMA_VERSION:
        raise ValueError("Unsupported hardware audit receipt")
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return path
