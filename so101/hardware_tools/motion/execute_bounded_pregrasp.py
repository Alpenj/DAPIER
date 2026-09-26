#!/usr/bin/env python3
"""Launch the bounded native executor; Python never imports a motor SDK.

Default transport is a lagged mock. A native joint-endpoint result is not grasp,
Cartesian acceptance, or task success. Hardware approval remains in native code.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ik-result", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confirm")
    parser.add_argument("--operator-present", action="store_true")
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--native-executor", type=Path)
    parser.add_argument("--native-sha256")
    parser.add_argument("--transport", choices=("mock", "hardware"), default="mock")
    parser.add_argument("--previous-phase-trace", type=Path,
                        help="Completed native CLOSE/HOLD trace for model-only carry")
    parser.add_argument("--object-observation-binding", type=Path,
                        help="Existing native observation binding JSON for model-only carry")
    parser.add_argument("--maximum-duration-s", type=float,
                        help="Explicit candidate execution budget, up to 60s; part of the hardware approval plan")
    args = parser.parse_args(argv)
    if args.output.exists() or args.output.is_symlink():
        raise FileExistsError("refusing to overwrite execution evidence")
    for suffix in (".plan.json", ".endpoint.json"):
        if args.output.with_suffix(suffix).exists() or args.output.with_suffix(suffix).is_symlink():
            raise FileExistsError("refusing to overwrite derived execution evidence")
    native_invoked = False
    report = {
        "exit_code": 1, "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "motion_started": False, "hardware_execution": False, "accepted_for_execution": False,
        "rejection_stage": "NATIVE_CONFIGURATION_REQUIRED",
        "reason": "bounded plan, trusted execution profile and pinned native executable required",
        "motor_writes": {"Goal_Position": 0, "Torque_Enable_1": 0, "Torque_Enable_0": 0},
    }
    try:
        raw = args.ik_result.read_bytes()
        report["ik_source_sha256"] = hashlib.sha256(raw).hexdigest()
        ik = json.loads(raw)
        candidate = ik if ik.get("schema_version") == "dapier.offline-ik-candidate.v1" else None
        carry_candidate = candidate is not None and candidate.get("candidate_mode") == "carry_endpoint_ik"
        if args.previous_phase_trace is not None or args.object_observation_binding is not None or carry_candidate:
            if (not carry_candidate or args.profile is None or args.previous_phase_trace is None
                    or args.object_observation_binding is None or args.transport != "mock"):
                raise ValueError("carry requires candidate/profile/prior phase/observation binding and MOCK transport")
        if args.maximum_duration_s is not None and (candidate is None or args.profile is None):
            raise ValueError("maximum duration requires a candidate and profile; cannot override a prepared plan")
        plan_path = args.ik_result
        if ik.get("schema_version") == "dapier.offline-ik-candidate.v1" and args.profile is not None:
            sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "2ARM_ROBOT/research/src"))
            from dapier_research.real_sensor_ik_adapter import bounded_pregrasp_plan, bounded_carry_plan
            from dapier_research.control_intent import intent_from_mapping
            intent = intent_from_mapping(ik["goal_intent"]) if "goal_intent" in ik else None
            if carry_candidate:
                ik = bounded_carry_plan(ik, args.profile, args.previous_phase_trace,
                    args.object_observation_binding, now_s=time.time(), goal_intent=intent,
                    maximum_duration_s=args.maximum_duration_s)
            else:
                ik = bounded_pregrasp_plan(ik, args.profile, now_s=time.time(), goal_intent=intent,
                                          maximum_duration_s=args.maximum_duration_s)
            ik["candidate_source"] = {"path":str(args.ik_result.resolve()), "sha256":report["ik_source_sha256"]}
            args.output.parent.mkdir(parents=True, exist_ok=True)
            plan_path = args.output.with_suffix(".plan.json")
            with plan_path.open("x") as stream:
                json.dump(ik, stream, indent=2, allow_nan=False)
        if ik.get("schema_version") != "dapier.bounded-pregrasp-plan.v1":
            report["rejection_stage"] = "IK_PLAN_AUDIT"
            report["reason"] = "an IK success flag alone is not a bounded sensor execution plan"
        elif args.profile is not None and args.native_executor is not None and args.native_sha256:
            native = args.native_executor.resolve(strict=True)
            if hashlib.sha256(native.read_bytes()).hexdigest() != args.native_sha256:
                raise ValueError("native executable SHA mismatch")
            command = [str(native), "--plan", str(plan_path.resolve()),
                       "--profile", str(args.profile.resolve()), "--output", str(args.output.resolve()),
                       "--transport", args.transport]
            if args.confirm:
                command.extend(("--confirm", args.confirm))
            if args.operator_present:
                command.append("--operator-present")
            args.output.parent.mkdir(parents=True, exist_ok=True)
            native_invoked = True
            exit_code = subprocess.run(command, check=False).returncode
            if exit_code == 0 and candidate is not None and "compiled_sha256" in candidate.get("model", {}):
                sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "2ARM_ROBOT/sim/mobile_dual_so101"))
                from evaluate_single_shot_ik import check_native_feedback_endpoint
                native_result = json.loads(args.output.read_text().splitlines()[-1])
                # A supported PLACE stop is accepted short of its joint endpoint, so
                # its result must name this plan (prior phase/support binding) and profile.
                if carry_candidate and (
                        native_result.get("plan_sha256") != hashlib.sha256(plan_path.read_bytes()).hexdigest()
                        or native_result.get("profile_sha256") != hashlib.sha256(args.profile.read_bytes()).hexdigest()):
                    raise ValueError("native result is not bound to the dispatched carry plan/profile")
                try:
                    endpoint = check_native_feedback_endpoint(candidate, native_result)
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    endpoint = {"cartesian_endpoint_verified":False,"task_success":False,"error":str(exc)}
                with args.output.with_suffix(".endpoint.json").open("x") as stream:
                    json.dump(endpoint, stream, indent=2, allow_nan=False)
                if carry_candidate and args.transport == "mock":
                    # Only the model/native connection is accepted here. Physical
                    # endpoint/contact acceptance remains false in the evidence.
                    accepted = (endpoint.get("model_supported_stop_verified") if
                        candidate["planning_phase"] == "PLACE" else
                        endpoint.get("kinematic_endpoint_within_tolerance"))
                    return 0 if accepted is True else 1
                return 0 if endpoint.get("cartesian_endpoint_verified") is True else 1
            return exit_code
    except (OSError, ValueError, TypeError, AttributeError, KeyError) as exc:
        if native_invoked:
            # Never replace native evidence or claim zero writes after dispatch.
            print(json.dumps({"post_dispatch_error":str(exc),"native_evidence":str(args.output),
                              "motor_state":"consult native log; not assumed unchanged"}), file=sys.stderr)
            return 1
        report["rejection_stage"] = "IK_PLAN_AUDIT"
        report["reason"] = str(exc)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(report, indent=2))
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
