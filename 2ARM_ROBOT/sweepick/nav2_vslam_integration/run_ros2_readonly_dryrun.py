#!/usr/bin/env python3
"""C-2 P0-4: ROS 2 Jazzy read-only dry-run.

Spins `SweepickNavReadOnlyNode` for `--duration` seconds against whatever ROS 2 graph is
reachable (ROS_AUTOMATIC_DISCOVERY_RANGE as set in the shell), records what was observed and
what the integration layer would decide, and writes a JSON report. It never publishes a Nav2
goal, `/cmd_vel`, or anything else, never opens a Sweepick device and never requests an arm
task: the coordinator is driven only up to MANIPULATION_READY and the report records
`ARM_TASK_NOT_REQUESTED_IN_DRYRUN`.

`--mock-graph` additionally starts `mock_ros2_graph_publisher.py` (labelled MOCK data) so the
rclpy path can be exercised on a machine without the robot. The report then says
`data_origin: MOCK_ROS_GRAPH`.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
SWEEPICK_ROOT = HERE.parent
if str(SWEEPICK_ROOT) not in sys.path:
    sys.path.insert(0, str(SWEEPICK_ROOT))

from nav2_vslam_integration import (  # noqa: E402
    IntegrationStage,
    SweepickExecutionAdapter,
    VslamSweepickCoordinator,
)
from nav2_vslam_integration.ros2_readonly_adapter import rclpy_available  # noqa: E402


class RefusingConsole:
    """A Console stand-in that supports nothing: even if the pipeline reached ARM_TASK it would be rejected."""

    def operation_contract(self):
        return {"scopes": [], "owner": {"device_occupancy": {}, "device_query_error": None}, "device_access": False}

    def status(self):
        return {"state": "IDLE", "active": None, "can_prepare": False, "owner_alive": False, "block_reason": "dry-run: no Console attached"}

    def prepare(self, *a, **k):
        raise RuntimeError("dry-run must never prepare a Sweepick session")

    def stop(self, job_id):
        raise RuntimeError("dry-run must never stop a Sweepick session")


def read_console_status(path: Path | None):
    if path is None:
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return {"state": "UNKNOWN", "error": f"{type(exc).__name__}: {exc}"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--goal-label", default="desk_red_object")
    ap.add_argument("--mock-graph", action="store_true")
    ap.add_argument("--console-status", type=Path, default=None, help="read-only copy of a Console status() JSON")
    ap.add_argument("--discovery-wait", type=float, default=3.0, help="seconds to wait for existing publishers before subscribing")
    ap.add_argument("--mock-fresh-frames", action="store_true", help="mock graph gives every pose a new raw sensor frame")
    args = ap.parse_args()

    report = {
        "schema": "dapier.c2.ros2-readonly-dryrun.v1",
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "environment": {
            "ROS_DISTRO": os.environ.get("ROS_DISTRO"),
            "ROS_AUTOMATIC_DISCOVERY_RANGE": os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE"),
            "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY"),
            "python": platform.python_version(),
            "rclpy_available": rclpy_available(),
        },
        "write_authority": {
            "publishers_created_by_adapter": 0, "action_goals_sent": 0, "cmd_vel_published": 0,
            "service_calls": 0, "sweepick_prepare_calls": 0, "motor_writes": 0,
        },
        "data_origin": "NO_DATA",
    }
    if not rclpy_available():
        report["error"] = "rclpy not importable; source /opt/ros/jazzy/setup.bash"
        args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False))
        return 2

    import rclpy
    from nav2_vslam_integration.ros2_readonly_adapter import SweepickNavReadOnlyNode

    mock_proc = None
    if args.mock_graph:
        argv = [sys.executable, str(HERE / "mock_ros2_graph_publisher.py"), "--duration", str(args.duration + args.discovery_wait + 2)]
        if args.mock_fresh_frames:
            argv.append("--fresh-frames")
        mock_proc = subprocess.Popen(argv, cwd=str(SWEEPICK_ROOT))
        report["mock_graph"] = {"pid": mock_proc.pid, "fresh_frames": args.mock_fresh_frames}

    rclpy.init()
    # Wait for the existing graph to be discovered so subscriptions can copy the publishers' QoS.
    probe = rclpy.create_node("sweepick_c2_discovery_probe")
    t_wait = time.monotonic() + args.discovery_wait
    while time.monotonic() < t_wait:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if probe.get_publishers_info_by_topic("/odom") or probe.get_publishers_info_by_topic("/docking/status"):
            time.sleep(0.5)
            break
    report["discovery"] = {"waited_s": round(args.discovery_wait - max(0.0, t_wait - time.monotonic()), 2),
                           "topics_with_publishers": sorted(t for t, _ in probe.get_topic_names_and_types() if probe.get_publishers_info_by_topic(t))}
    probe.destroy_node()
    node = SweepickNavReadOnlyNode(discovery_wait_s=args.discovery_wait)
    report["discovery"]["node_waited_s"] = node.discovery_waited_s
    try:
        for name, version_pkg in (("nav2_msgs", "nav2_msgs"), ("rclpy", "rclpy")):
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_xml = Path(get_package_share_directory(version_pkg)) / "package.xml"
                import re
                m = re.search(r"<version>([^<]+)</version>", pkg_xml.read_text())
                report["environment"][f"{name}_version"] = m.group(1) if m else None
            except Exception as exc:  # noqa: BLE001
                report["environment"][f"{name}_version"] = f"unknown ({type(exc).__name__})"

        state = node.state
        if args.console_status is not None:
            state.ingest_console_status(read_console_status(args.console_status))
        gate_timeline = []
        t_end = time.monotonic() + args.duration
        next_check = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_check:
                gate = state.arm_task_gate(now_s=node.now_s())
                gate_timeline.append({"t_s": round(node.now_s(), 3), "allowed": gate.allowed, "reasons": gate.reasons,
                                      "docking_state": state.docking_state, "task_state": state.task_state,
                                      "nav2_active": state.nav2_active_goal_ids()})
                next_check += 1.0

        now_s = node.now_s()
        topics = [{"topic": t, "types": ty, "publishers": [{"node": i.node_name, "qos": {"reliability": str(i.qos_profile.reliability), "durability": str(i.qos_profile.durability)}}
                                                           for i in node.get_publishers_info_by_topic(t)]}
                  for t, ty in sorted(node.get_topic_names_and_types())]
        publisher_nodes = {n for t in state.publisher_nodes_seen.values() for n in t}
        if not state.counts:
            origin = "NO_DATA"
        elif any("mock" in n.lower() for n in publisher_nodes):
            origin = "MOCK_ROS_GRAPH"
        else:
            origin = "REAL_ROS_GRAPH"
        report["data_origin"] = origin

        arrival, arrival_reasons = state.arrival_evidence(goal_label=args.goal_label, map_pose_lookup=node.map_pose_at)
        base_stop, stop_reasons = (None, ["NO_ARRIVAL"]) if arrival is None else state.base_stop_evidence(now_s=now_s, arrived_at_s=arrival.arrived_at_s)
        candidate = state.latest_post_stop_candidate(base_stop=base_stop) if base_stop is not None else {"observation": None, "rejected": ["NO_BASE_STOP_EVIDENCE"], "provenance": None}
        candidate_json = {k: v for k, v in candidate.items() if k != "observation_obj"}

        # Drive the coordinator up to MANIPULATION_READY with the observed evidence; never ARM_TASK.
        coord = VslamSweepickCoordinator(sweepick_adapter=SweepickExecutionAdapter(RefusingConsole()))
        pipeline = {"stages": [], "stopped_at": None, "outcome": None}
        try:
            coord.step_detect(None); coord.step_navigate()
            pipeline["stages"] = [s.value for s in coord.stage_history]
            if arrival is None:
                pipeline["stopped_at"] = "ARRIVED:" + ";".join(arrival_reasons)
            else:
                abort = coord.step_arrived(arrival)
                if abort is not None:
                    pipeline["outcome"] = abort.to_dict(); pipeline["stopped_at"] = "ARRIVED"
                elif base_stop is None:
                    pipeline["stopped_at"] = "BASE_STOPPED:" + ";".join(stop_reasons)
                else:
                    abort = coord.step_base_stopped(base_stop)
                    if abort is not None:
                        pipeline["outcome"] = abort.to_dict(); pipeline["stopped_at"] = "BASE_STOPPED"
                    else:
                        abort = coord.step_reobserve(candidate.get("observation_obj"), checked_at_s=now_s)
                        if abort is not None:
                            pipeline["outcome"] = abort.to_dict(); pipeline["stopped_at"] = "REOBSERVE"
                        else:
                            abort = coord.step_manipulation_ready()
                            if abort is not None:
                                pipeline["outcome"] = abort.to_dict(); pipeline["stopped_at"] = "MANIPULATION_READY"
                            else:
                                pipeline["stopped_at"] = "MANIPULATION_READY:ARM_TASK_NOT_REQUESTED_IN_DRYRUN"
            pipeline["stages"] = [s.value for s in coord.stage_history]
        except Exception as exc:  # noqa: BLE001
            pipeline["error"] = f"{type(exc).__name__}: {exc}"
        assert coord.arm_commands_invoked_count == 0

        report.update({
            "duration_s": args.duration,
            "adapter_interfaces": node.own_interfaces(),
            "subscriptions": node.subscriptions_made,
            "graph_topics": topics,
            "observed": state.snapshot(),
            "gate_timeline": gate_timeline,
            "arrival_evidence": {"evidence": arrival.__dict__ if arrival else None, "reasons": arrival_reasons},
            "base_stop_evidence": {"evidence": base_stop.__dict__ if base_stop else None, "reasons": stop_reasons},
            "post_stop_candidate": candidate_json,
            "tf_lookups": node.tf_lookup_log[-50:],
            "pipeline_dryrun": pipeline,
            "arm_commands_invoked_count": coord.arm_commands_invoked_count,
        })
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if mock_proc is not None:
            mock_proc.terminate()
            try:
                mock_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                mock_proc.kill()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str))
    print(json.dumps({"data_origin": report["data_origin"], "stopped_at": report.get("pipeline_dryrun", {}).get("stopped_at"),
                      "counts": report.get("observed", {}).get("message_counts")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
