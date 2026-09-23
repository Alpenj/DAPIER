#!/usr/bin/env python3
"""Exercise the real launcher and native loop with only the transport mocked.

Usage: check_native_pregrasp.py NATIVE_BINARY NEW_EVIDENCE_DIRECTORY
No device paths are supplied. The mock has a lagged plant independent of commands.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "2ARM_ROBOT/research/src"))
from dapier_research.control_intent import arm_joint_position_intent
from dapier_research.wrist_servo_adapter import WristObservation, WristServoConfig, wrist_correction_intent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(binary, directory):
    binary, directory = Path(binary).resolve(strict=True), Path(directory)
    directory.mkdir(mode=0o700)
    joints = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    cal = directory / "synthetic-calibration.json"
    cal.write_text(json.dumps({name: dict(id=i+1, homing_offset=0, range_min=0, range_max=4095)
                               for i, name in enumerate(joints)}))
    profile = directory / "synthetic-profile.json"
    profile.write_text(json.dumps({
        "schema_version": "dapier.left-pregrasp-profile.v1", "device_id": "dapier_dual_follower_left",
        "physically_verified": False, "calibration_path": str(cal.resolve()),
        "arm_signs":[1]*10,"arm_zero_offsets_deg":[0.]*10,
        "calibration_sha256": sha(cal), "gripper_rad_limits": [0., 2.],
        "joints": [dict(name=name, minimum_rad=0. if name == "gripper" else -.5,
                        maximum_rad=2. if name == "gripper" else .5,
                        maximum_velocity_rad_s=.3, sign=1, zero_offset_deg=0.) for name in joints]}))
    profile.chmod(0o600)
    launcher = Path(__file__).with_name("execute_bounded_pregrasp.py")
    results = []
    for name in ("lagged", "stalled", "wrist"):
        stalled = name == "stalled"
        goal = [.08, -.03, .04, 0., 0., 1.] if name != "wrist" else [0.,0.,0.,0.,0.,1.]
        now = time.monotonic_ns()
        intent = arm_joint_position_intent(sequence=1, source="MOCK_sensor_fixture",
            joint_names=tuple(joints), joint_position_rad=tuple(goal),
            joint_max_velocity_rad_s=(.3,)*6, source_monotonic_ns=now, ttl_ns=250_000_000)
        if name == "wrist":
            intent = wrist_correction_intent(
                WristObservation(now, {"wrist_roll":0.,"wrist_flex":0.}, (.15,-.15), .95),
                intent, WristServoConfig(), now, sequence=2,
                joint_limits_rad={j:(0.,2.) if j=="gripper" else (-.5,.5) for j in joints})
            goal = list(intent.joint_position_rad)
        plan, output = directory / f"{name}-plan.json", directory / f"{name}-trace.jsonl"
        if not stalled:
            # Synthetic, explicitly unverified sensor/IK audit exercises the same
            # source loader/plan builder as real inputs. It is not a SIM/HW IK pass.
            block = directory / f"{name}-MOCK-observation.json"
            block.write_text(json.dumps({"rgb_timestamp_ns":time.time_ns(),
                                         "depth_evidence":{"metric_target_verified":False}}))
            def fingerprint(path):
                return {"path":str(path.resolve()),"sha256":sha(path)}
            measured = {**fingerprint(cal),"timestamp":datetime.now(timezone.utc).isoformat(),
                        "calibration":fingerprint(cal)}
            candidate = {"schema_version":"dapier.offline-ik-candidate.v1",
                "offline_candidate_accepted":True,"fixture":"MOCK_synthetic_not_physical_IK",
                "seed_posture":{"seed_q_rad":[0.,0.,0.,0.,0.,1.]*2,"left":measured,"right":measured},
                "solved_action_rad":goal+[0.,0.,0.,0.,0.,1.],"goal_intent":intent.as_dict(),
                "position_error_m":0.,"tool_axis_error_rad_by_side":{"left":0.},
                "structured_clearance":{"safe":True,"minimum_clearance_m":.05},
                "block_source":fingerprint(block),"model":{**fingerprint(cal),"gripper_ranges_rad":[[0.,2.],[0.,2.]]},
                "mapping":{"profile":fingerprint(profile),"physically_verified":False}}
            plan.write_text(json.dumps(candidate))
        else:
            plan.write_text(json.dumps({"schema_version": "dapier.bounded-pregrasp-plan.v1",
                "profile_sha256": sha(profile), "start_rad": [0., 0., 0., 0., 0., 1.],
                "goal_rad": goal, "goal_intent":intent.as_dict(), "maximum_duration_s": 4.,
                "path_tracking_tolerance_rad": .01, "mock_stalled": True}))
        completed = subprocess.run([sys.executable, str(launcher), "--ik-result", str(plan),
            "--profile", str(profile), "--native-executor", str(binary), "--native-sha256", sha(binary),
            "--transport", "mock", "--output", str(output)], capture_output=True, text=True, timeout=8)
        events = [json.loads(line) for line in output.read_text().splitlines()]
        result = events[-1]
        assert completed.returncode == int(stalled), (completed, result)
        assert not result["hardware_execution"] and not result["task_success"], result
        assert result["reached_joint_endpoint"] == (not stalled), result
        assert result["accepted_goal_intent"] == intent.as_dict()
        if not stalled:
            built = json.loads(output.with_suffix(".plan.json").read_text())
            assert not built["sensor_target_verified"] and not built["path_envelope_verified"]
        steps = [e for e in events if e["event"] == "step" and e["sent_rad"]]
        assert steps and all(e["sent_rad"] == e["limited_rad"] for e in steps)
        assert any(any(abs(a-b) > .0001 for a,b in zip(e["sent_rad"], e["measured_before_command_rad"]))
                   for e in steps), "feedback was copied from command"
        if stalled:
            assert all(e["measured_before_command_rad"] == [0.,0.,0.,0.,0.,1.] for e in steps)
            assert any(e["limited_rad"] != e["requested_rad"] for e in steps)
        results.append({"case": name, "phase": result["phase"], "trace_sha256": sha(output)})
    # Even a command containing --transport hardware cannot open a port without
    # the native attended TTY + exact confirmation gate. No profile/port is given.
    gate = subprocess.run([str(binary), "--transport", "hardware"], capture_output=True, text=True)
    refusal = json.loads(gate.stderr)
    assert gate.returncode == 1 and "before any device access" in refusal["reason"]
    assert not refusal.get("hardware_access_attempted", False)
    report = {"native_sha256": sha(binary), "launcher_sha256": sha(launcher), "cases": results,
              "unapproved_hardware_refused_before_access": True, "hardware_execution": False}
    (directory / "result.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main(*sys.argv[1:])
