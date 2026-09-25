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
import threading
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "2ARM_ROBOT/research/src"))
from dapier_research.control_intent import arm_joint_position_intent
from dapier_research.wrist_servo_adapter import (WristObservation, WristServoConfig, wrist_correction_intent,
                                               block_grasp_observation_from_wrist)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(binary, directory, mode="pregrasp", case=None):
    if mode not in ("pregrasp", "hold", "close", "lift", "support"):
        raise ValueError("check mode must be pregrasp, hold, close, lift or support")
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
    cases = ("hold", "hold_lost", "hold_stale", "hold_relabelled") if mode == "hold" else ("lagged", "stalled", "wrist")
    if mode == "close":
        cases = ("close", "close_unknown", "close_lost", "close_no_contact", "close_stale", "close_relabelled")
    if mode == "lift":
        cases = ("close", "lift", "lift_lost", "lift_stale", "lift_supported", "lift_low",
                 "lift_wrong_grasp", "lift_changed_aperture", "lift_old_grasp_frame")
    if mode == "support":
        cases = ("close", "lift", "place", "release", "place_unknown", "place_unapproved",
                 "release_lost", "release_no_release", "release_wrong_support", "place_ancestor_frame")
    if case is not None:
        if mode != "lift" or case not in cases[1:]:
            raise ValueError("single-case selection requires a LIFT case")
        cases = ("close", case)  # Preserve the real preceding native phase.
    for name in cases:
        hold_case = name.startswith("hold")
        close_case = name.startswith("close")
        lift_case = name.startswith("lift")
        place_case, release_case = name.startswith("place"), name.startswith("release")
        support_case = place_case or release_case
        start = [0.,0.,0.,0.,0.,1.]
        stalled = name == "stalled"
        goal = [.08, -.03, .04, 0., 0., 1.] if name != "wrist" else [0.,0.,0.,0.,0.,1.]
        if hold_case or close_case:
            goal = [0.,0.,0.,0.,0.,.8 if close_case else 1.]
        if lift_case:
            grasp_trace = directory / "close-trace.jsonl"
            grasp_events = [json.loads(line) for line in grasp_trace.read_text().splitlines()]
            confirmed = grasp_events[-1]
            prior_sequence = [e["observation"]["sequence"] for e in grasp_events if e["event"]=="grasp_observation"][-1]
            assert confirmed["observed_grasp_verified"] and not confirmed["task_success"]
            start = confirmed["final_measured_rad"]
            goal = list(start); goal[0] += .06
            if name == "lift_changed_aperture":
                goal[-1] -= .01
        if support_case:
            previous_trace = directory / ("place-trace.jsonl" if release_case else "lift-trace.jsonl")
            previous_events = [json.loads(line) for line in previous_trace.read_text().splitlines()]
            previous = previous_events[-1]
            start = previous["final_measured_rad"]
            goal = list(start)
            if release_case:
                goal[-1] += .12
            else:
                goal[0] -= .06
            prior_sequence = max(e["observation"]["sequence"] for e in previous_events if "observation" in e)
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
        stop, producer = threading.Event(), None
        if hold_case or close_case or lift_case or support_case:
            observation = directory / f"{name}-observation.json"
            binding = dict(path=str(observation.resolve()), run_id=name, object_id="MOCK-cube",
                           calibration_revision="MOCK", producer_sha256=sha(Path(__file__)),
                           boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                           observer_physically_verified=False)
            if lift_case:
                binding["run_id"] = "wrong-run" if name == "lift_wrong_grasp" else "close"
            if support_case:
                binding["run_id"] = "close"
                binding["support_id"] = "MOCK-wrong-table" if name == "release_wrong_support" else "MOCK-table"
            if name == "close_unknown":
                import inspect
                binding["producer_sha256"] = sha(Path(inspect.getfile(block_grasp_observation_from_wrist)))
            sequence = prior_sequence if lift_case or support_case else 0
            began = time.monotonic_ns()
            def publish():
                nonlocal sequence
                sequence += 1
                stamp = time.monotonic_ns()
                frame = directory / f"{name}-frame-{1 if name.endswith('_relabelled') else sequence}.bin"
                if not frame.exists():
                    frame.write_bytes(f"MOCK {name} synthetic frame {sequence}; not camera evidence".encode())
                value = {**binding, "schema_version":"dapier.block-hold-observation.v1",
                         "source_kind":"mock", "sequence":sequence, "captured_monotonic_ns":stamp,
                         "frame_path":str(frame.resolve()), "frame_sha256":sha(frame),
                         "bottom_clearance_lower_bound_m":.030, "bilateral_grasp_verified":True,
                         "external_support":name=='hold_lost' and stamp-began>=1_000_000_000}
                if close_case:
                    value["schema_version"] = "dapier.block-grasp-observation.v1"
                    value["bilateral_grasp_verified"] = (
                        stamp-began >= 800_000_000 and name != "close_no_contact")
                    # Inject loss only after actual phase entry, not a 50ms
                    # wall-clock window that the native reader might miss.
                    if name == "close_lost" and output.exists() and '"phase":"GRASP_CONFIRM"' in output.read_text():
                        value["bilateral_grasp_verified"] = False
                if lift_case:
                    elapsed = stamp-began
                    value["bilateral_grasp_verified"] = not (name == "lift_lost" and elapsed>=500_000_000)
                    value["bottom_clearance_lower_bound_m"] = .035 if elapsed>=1_000_000_000 else 0.
                    value["external_support"] = elapsed<1_000_000_000 or name=="lift_supported"
                    if name == "lift_low":
                        value["bottom_clearance_lower_bound_m"] = .01
                    if name == "lift_old_grasp_frame":
                        consumed = next(e["observation"] for e in grasp_events if e["event"]=="grasp_observation")
                        value["frame_path"] = consumed["frame_path"]
                        value["frame_sha256"] = consumed["frame_sha256"]
                if name == "close_unknown":
                    # Reuse the real saved-wrist producer. A visible RGB image
                    # supplies no independent jaw/contact evidence, hence null.
                    import cv2
                    import numpy as np
                    frame = directory / f"{name}-MOCK-frame-{sequence}.png"
                    assert cv2.imwrite(str(frame), np.full((8,8,3), sequence % 255, np.uint8))
                    capture = directory / f"{name}-capture-{sequence}.json"
                    capture.write_text(json.dumps({"schema_version":"dapier.wrist-frame.v1",
                        "side":"left", "clock":"host_monotonic_ns", "frame_acquired":True,
                        "normal_stream_close":True, "timestamp_ns":stamp,
                        "host_boot_id":binding["boot_id"], "frame_path":str(frame.resolve()), "frame_sha256":sha(frame)}))
                    value = block_grasp_observation_from_wrist(
                        {"capture_source":{"path":str(capture.resolve()), "sha256":sha(capture)}},
                        run_id=name, object_id=binding["object_id"], calibration_revision="MOCK",
                        sequence=sequence, source_kind="mock")
                if support_case:
                    elapsed = stamp-began
                    supported = release_case or elapsed>=800_000_000
                    released = release_case and elapsed>=1_000_000_000 and name!="release_no_release"
                    if name == "release_lost" and elapsed>=500_000_000:
                        supported=False
                    value.update(schema_version="dapier.block-support-observation.v1",
                        approved_support_verified=supported, object_released_verified=released,
                        external_support=supported, bilateral_grasp_verified=not released,
                        bottom_clearance_lower_bound_m=0. if supported else .035)
                    if name == "place_unknown":
                        value["approved_support_verified"]=None
                    if name == "place_unapproved":
                        value.update(external_support=True, approved_support_verified=False)
                    if name == "place_ancestor_frame":
                        close_events = [json.loads(line) for line in (directory/"close-trace.jsonl").read_text().splitlines()]
                        consumed = next(e["observation"] for e in close_events if e["event"]=="grasp_observation")
                        value.update(frame_path=consumed["frame_path"], frame_sha256=consumed["frame_sha256"])
                temporary = observation.with_suffix('.tmp')
                temporary.write_text(json.dumps(value))
                temporary.replace(observation)
            publish()
            def produce():
                while not stop.wait(.05):
                    if not name.endswith('_stale'):
                        publish()
            producer = threading.Thread(target=produce)
            prepared = {"schema_version":"dapier.bounded-pregrasp-plan.v1",
                "profile_sha256":sha(profile), "start_rad":start, "goal_rad":goal,
                "goal_intent":intent.as_dict(), "maximum_duration_s":9. if lift_case else 6. if close_case else 5.,
                "path_tracking_tolerance_rad":.01, "phase":"LIFT" if lift_case else "CLOSE" if close_case else "HOLD",
                "initial_torque_enabled":True, "grasp_observation" if close_case else "hold_observation":binding}
            if lift_case:
                prepared["grasp_confirmation"] = {"path":str(grasp_trace.resolve()), "sha256":sha(grasp_trace)}
            if support_case:
                prepared.pop("hold_observation")
                prepared.update(phase="RELEASE" if release_case else "PLACE", maximum_duration_s=6.,
                    support_observation=binding,
                    previous_phase={"path":str(previous_trace.resolve()), "sha256":sha(previous_trace)})
            plan.write_text(json.dumps(prepared))
        elif not stalled:
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
                "scene_object":{"bound_to_path_reference":True,"fixture":"MOCK_only"},
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
        try:
            if producer is not None:
                producer.start()
            completed = subprocess.run([sys.executable, str(launcher), "--ik-result", str(plan),
                "--profile", str(profile), "--native-executor", str(binary), "--native-sha256", sha(binary),
                "--transport", "mock", "--output", str(output)], capture_output=True, text=True, timeout=12 if lift_case else 8)
        finally:
            stop.set()
            if producer is not None and producer.ident is not None:
                producer.join(timeout=1.)
        events = [json.loads(line) for line in output.read_text().splitlines()]
        result = events[-1]
        if support_case:
            passed = name in ("place", "release")
            steps = [e for e in events if e["event"]=="step" and e["sent_rad"]]
            assert completed.returncode == int(not passed), (completed, result)
            assert not result["hardware_execution"] and not result["task_success"]
            assert not result.get("hardware_access_attempted", False)
            if passed:
                assert result["observed_support_verified"]
                assert result["observed_release_verified"] == release_case
                assert result["phase"] == ("SUPPORTED_RELEASED_HOLDING_TORQUE" if release_case else "SUPPORTED_PLACED_HOLDING")
                assert set(previous["observed_frame_history"]) < set(result["observed_frame_history"])
                assert steps and any(e["sent_rad"]!=e["measured_before_command_rad"] for e in steps)
                if place_case:
                    assert all(e["sent_rad"][-1]==start[-1] for e in steps)
                    assert any(e["phase"]=="SUPPORT_CONFIRM" for e in steps)
                    assert not result["reached_joint_endpoint"]
                else:
                    assert all(e["sent_rad"][:-1]==start[:-1] for e in steps)
                    assert "retain arm torque" in result["ending"]
            else:
                expected={"place_unknown":"unknown", "place_unapproved":"unapproved external support",
                    "release_lost":"support lost", "release_no_release":"without distinct observed object release",
                    "release_wrong_support":"support", "place_ancestor_frame":"reused"}
                assert expected[name] in result["reason"], result
                if name in ("place_unknown", "place_unapproved", "release_wrong_support", "place_ancestor_frame"):
                    assert not steps
            results.append({"case":name,"phase":result["phase"],"reason":result["reason"],
                            "support_verified":result.get("observed_support_verified",False),
                            "release_verified":result.get("observed_release_verified",False),"trace_sha256":sha(output)})
            continue
        if lift_case:
            passed = name == "lift"
            assert completed.returncode == int(not passed), (completed, result)
            assert result.get("observed_hold_verified", False) is passed, result
            assert not result["hardware_execution"] and not result["task_success"]
            assert not result.get("hardware_access_attempted", False)
            steps = [e for e in events if e["event"] == "step" and e["sent_rad"]]
            if passed:
                assert result["observed_lift_verified"] and result["phase"] == "HOLD_REACHED_HOLDING"
                assert result["observed_hold_span_s"] >= 3.
                assert result["hold_entered_monotonic_ns"] - steps[0]["time_ns"] >= 2_000_000_000
                assert any(e["phase"] == "HOLD_OBSERVING" for e in steps)
                assert any(e["sent_rad"] != e["measured_before_command_rad"] for e in steps)
                assert all(e["sent_rad"][-1] == start[-1] for e in steps)
            else:
                expected = {"lift_lost":"lost bilateral", "lift_stale":"fresh frame",
                    "lift_supported":"unsupported lift", "lift_low":"unsupported lift",
                    "lift_wrong_grasp":"another task/source", "lift_changed_aperture":"keep measured grasp aperture",
                    "lift_old_grasp_frame":"reused a prior grasp frame"}
                assert expected[name] in result["reason"], result
                if name in ("lift_wrong_grasp", "lift_changed_aperture", "lift_old_grasp_frame"):
                    assert not steps
            results.append({"case":name,"phase":result["phase"],"reason":result["reason"],
                            "hold_verified":result.get("observed_hold_verified",False), "trace_sha256":sha(output)})
            continue
        if close_case:
            passed = name == "close"
            assert completed.returncode == int(not passed), (completed, result)
            assert result.get("observed_grasp_verified", False) is passed, result
            assert not result["hardware_execution"] and not result["task_success"]
            assert not result.get("hardware_access_attempted", False)
            steps = [e for e in events if e["event"] == "step" and e["sent_rad"]]
            if passed:
                assert result["phase"] == "GRASP_CONFIRMED_HOLDING"
                assert result["final_measured_rad"][-1] > goal[-1] + .01
                assert not result["reached_joint_endpoint"]
                assert any(e["sent_rad"] != e["measured_before_command_rad"] for e in steps)
                stopped = [e["sent_rad"] for e in steps if e["phase"] == "GRASP_CONFIRM"]
                assert stopped and all(q == stopped[0] for q in stopped)
                assert all(abs(a-b) <= .01500000001 for e in steps
                           for a,b in zip(e["sent_rad"], e["measured_before_command_rad"]))
            elif name == "close_unknown":
                assert not steps and "unknown" in result["reason"], result
            elif name == "close_lost":
                assert any(e["phase"] == "GRASP_CONFIRM" for e in steps), result
                assert "lost bilateral grasp" in result["reason"], result
            else:
                expected = {"close_no_contact":"endpoint without", "close_stale":"fresh frame",
                            "close_relabelled":"raw frame reused"}
                assert expected[name] in result["reason"], result
            results.append({"case":name,"phase":result["phase"],"reason":result["reason"],
                            "grasp_verified":result.get("observed_grasp_verified",False),
                            "trace_sha256":sha(output)})
            continue
        if hold_case:
            passed = name == "hold"
            assert completed.returncode == int(not passed), (completed, result)
            assert result.get("observed_hold_verified", False) is passed, result
            assert not result["hardware_execution"] and not result["task_success"]
            assert not result.get("hardware_access_attempted", False)
            if passed:
                assert result["observed_hold_span_s"] >= 3.
                assert result["phase"] == "HOLD_REACHED_HOLDING"
            else:
                assert "HOLD" in result["reason"], result
            results.append({"case":name,"phase":result["phase"],"reason":result["reason"],
                            "hold_verified":result.get("observed_hold_verified",False),
                            "trace_sha256":sha(output)})
            continue
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
