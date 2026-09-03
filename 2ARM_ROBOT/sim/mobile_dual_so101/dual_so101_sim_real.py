#!/usr/bin/env python3
"""Compare the bounded physical dual-arm smoke with the same MuJoCo sequence."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR))

from mobile_dual_so101 import (  # noqa: E402
    HUMANOID_HOME_ACTION,
    TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
    build_model,
)
from physics_ik import (  # noqa: E402
    MotionLimits,
    execute_physics_trajectory,
)


SCHEMA_VERSION = "dapier.dual-so101-sim-real-comparison.v1"


@dataclass(frozen=True)
class LinearStepTrajectory:
    start_rad: np.ndarray
    goal_rad: np.ndarray
    duration_s: float
    limits: MotionLimits
    update_period_s: float = 0.05

    def sample(self, time_s: float):
        updates = round(self.duration_s / self.update_period_s)
        update = min(updates, max(1, math.ceil(time_s / self.update_period_s)))
        target = self.start_rad + update / updates * (
            self.goal_rad - self.start_rad
        )
        velocity = (self.goal_rad - self.start_rad) / self.duration_s
        zeros = np.zeros_like(target)
        return target, velocity, zeros, zeros


def run_comparison(hardware_summary: Path) -> dict[str, object]:
    hardware = json.loads(hardware_summary.read_text(encoding="utf-8"))
    if hardware.get("schema_version") != "dapier.dual-so101-smoke-summary.v1":
        raise ValueError("unexpected hardware summary schema")
    if hardware["motion"]["joint"] != "shoulder_pan":
        raise ValueError("hardware evidence is not a shoulder-pan smoke")

    model, _ = build_model(
        arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
        mount_layout="tower",
    )
    start = np.asarray(HUMANOID_HOME_ACTION, dtype=np.float64)
    peak = start.copy()
    requested = hardware["motion"]["requested_degrees"]
    peak[0] += math.radians(float(requested["left"]))
    peak[6] += math.radians(float(requested["right"]))
    limits = MotionLimits()
    duration = float(hardware["motion"]["duration_each_way_seconds"])

    outbound = LinearStepTrajectory(start, peak, duration, limits)
    outbound_data, outbound_report = execute_physics_trajectory(
        model, outbound, pre_settle_time_s=0.0, settle_time_s=0.50
    )
    joint_ids = model.actuator_trnid[:, 0].astype(np.int32)
    qpos_addresses = model.jnt_qposadr[joint_ids].astype(np.int32)
    peak_qpos = outbound_data.qpos[qpos_addresses].copy()
    inbound = LinearStepTrajectory(peak, start, duration, limits)
    final_data, inbound_report = execute_physics_trajectory(
        model,
        inbound,
        initial_data=outbound_data,
        pre_settle_time_s=0.0,
        settle_time_s=0.50,
    )

    peak_degrees = np.degrees(
        peak_qpos[[0, 6]] - start[[0, 6]]
    )
    residual_degrees = np.degrees(
        final_data.qpos[qpos_addresses][[0, 6]] - start[[0, 6]]
    )
    simulated = {
        "left": {
            "excursion_degrees": float(peak_degrees[0]),
            "residual_degrees": float(residual_degrees[0]),
        },
        "right": {
            "excursion_degrees": float(peak_degrees[1]),
            "residual_degrees": float(residual_degrees[1]),
        },
    }
    measured = hardware["measured"]
    return {
        "schema_version": SCHEMA_VERSION,
        "source_hardware_record": hardware_summary.name,
        "sequence": "30 x 50 ms linear targets: left +3 deg / right -3 deg, then return",
        "control_update_period_s": 0.05,
        "simulated": simulated,
        "measured": {
            side: {
                "excursion_degrees": measured[side]["excursion_degrees"],
                "residual_degrees": measured[side]["residual_degrees"],
            }
            for side in ("left", "right")
        },
        "absolute_error_degrees": {
            side: {
                field: abs(simulated[side][field] - measured[side][field])
                for field in ("excursion_degrees", "residual_degrees")
            }
            for side in ("left", "right")
        },
        "outbound_physics": outbound_report.as_dict(),
        "inbound_physics": inbound_report.as_dict(),
        "strict_dynamics_gate_passed": (
            outbound_report.simulation_motion_accepted
            and inbound_report.simulation_motion_accepted
        ),
        "known_gate_failure": "20 Hz step command and finite-difference actual jerk",
        "hardware_execution": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("hardware_summary", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = run_comparison(args.hardware_summary)
    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
