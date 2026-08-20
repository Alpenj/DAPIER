"""ROS 2 mock topic publisher and recorder executables for Phase 0."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Sequence

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from shoe_sorting_data.recorder import ApproximateEpisodeRecorder


PERIOD_NS = 50_000_000
TOPIC_PREFIX = "/shoe_sorting/mock"


def _set_stamp(message: object, timestamp_ns: int) -> None:
    stamp = message.header.stamp
    stamp.sec = timestamp_ns // 1_000_000_000
    stamp.nanosec = timestamp_ns % 1_000_000_000


def _timestamp_ns(message: object) -> int:
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class MockTopicPublisher(Node):
    """Publish deterministic, metadata-only synthetic robot and camera topics."""

    def __init__(self, *, arm_dof: int = 5, gripper_dof: int = 1) -> None:
        super().__init__("shoe_mock_publisher")
        self.arm_dof = arm_dof
        self.gripper_dof = gripper_dof
        self.index = 0
        self.start_ns = self.get_clock().now().nanoseconds + 100_000_000
        self._topic_publishers = {
            name: self.create_publisher(message_type, f"{TOPIC_PREFIX}/{name}", 10)
            for name, message_type in {
                "left_joint_state": JointState,
                "right_joint_state": JointState,
                "left_joint_action": JointState,
                "right_joint_action": JointState,
                "base_velocity": TwistStamped,
                "base_command": TwistStamped,
                "workspace_rgb": Image,
                "workspace_depth": Image,
            }.items()
        }
        self.timer = self.create_timer(PERIOD_NS / 1_000_000_000, self.publish_sample)

    def _joint_message(self, timestamp_ns: int, *, side: str, action: bool) -> JointState:
        message = JointState()
        _set_stamp(message, timestamp_ns)
        message.name = [f"{side}_joint_{index + 1}" for index in range(self.arm_dof)] + [
            f"{side}_gripper"
        ]
        sign = 1.0 if side == "left" else -1.0
        offset = 0.003 if action else 0.0
        center = sign * (self.index * 0.005 + offset)
        message.position = [center + joint * 0.0001 for joint in range(self.arm_dof)] + [0.5]
        return message

    def _twist_message(self, timestamp_ns: int) -> TwistStamped:
        message = TwistStamped()
        _set_stamp(message, timestamp_ns)
        message.header.frame_id = "base_link"
        message.twist.linear.x = 0.0
        message.twist.angular.z = 0.0
        return message

    def _image_metadata(self, timestamp_ns: int, frame_name: str) -> Image:
        message = Image()
        _set_stamp(message, timestamp_ns)
        message.header.frame_id = frame_name
        return message

    def publish_sample(self) -> None:
        timestamp_ns = self.start_ns + self.index * PERIOD_NS
        messages = {
            "left_joint_state": self._joint_message(timestamp_ns, side="left", action=False),
            "right_joint_state": self._joint_message(timestamp_ns, side="right", action=False),
            "left_joint_action": self._joint_message(timestamp_ns, side="left", action=True),
            "right_joint_action": self._joint_message(timestamp_ns, side="right", action=True),
            "base_velocity": self._twist_message(timestamp_ns),
            "base_command": self._twist_message(timestamp_ns),
            "workspace_rgb": self._image_metadata(timestamp_ns - 2_000_000, "workspace_rgb"),
            "workspace_depth": self._image_metadata(timestamp_ns + 3_000_000, "workspace_depth"),
        }
        for name, message in messages.items():
            self._topic_publishers[name].publish(message)
        self.index += 1


class MockEpisodeRecorderNode(Node):
    """Subscribe to mock topics and feed complete sets into the pure recorder."""

    def __init__(
        self,
        output: Path,
        *,
        target_samples: int,
        arm_dof: int = 5,
        gripper_dof: int = 1,
    ) -> None:
        super().__init__("shoe_mock_recorder")
        if target_samples < 2:
            raise ValueError("target_samples must be at least 2")
        self.target_samples = target_samples
        self.recorder = ApproximateEpisodeRecorder(
            output,
            arm_dof=arm_dof,
            gripper_dof=gripper_dof,
        )
        self.camera_frame_ids = {"workspace_rgb": 0, "workspace_depth": 0}
        for name in (
            "left_joint_state",
            "right_joint_state",
            "left_joint_action",
            "right_joint_action",
        ):
            self.create_subscription(
                JointState,
                f"{TOPIC_PREFIX}/{name}",
                lambda message, stream_name=name: self._joint_callback(stream_name, message),
                10,
            )
        for name in ("base_velocity", "base_command"):
            self.create_subscription(
                TwistStamped,
                f"{TOPIC_PREFIX}/{name}",
                lambda message, stream_name=name: self._twist_callback(stream_name, message),
                10,
            )
        for name in ("workspace_rgb", "workspace_depth"):
            self.create_subscription(
                Image,
                f"{TOPIC_PREFIX}/{name}",
                lambda message, stream_name=name: self._camera_callback(stream_name, message),
                10,
            )

    @property
    def complete(self) -> bool:
        return self.recorder.sample_count >= self.target_samples

    def _joint_callback(self, name: str, message: JointState) -> None:
        self.recorder.update(
            name,
            timestamp_ns=_timestamp_ns(message),
            values=message.position,
        )

    def _twist_callback(self, name: str, message: TwistStamped) -> None:
        self.recorder.update(
            name,
            timestamp_ns=_timestamp_ns(message),
            values=(message.twist.linear.x, message.twist.angular.z),
        )

    def _camera_callback(self, name: str, message: Image) -> None:
        frame_id = self.camera_frame_ids[name]
        self.camera_frame_ids[name] += 1
        self.recorder.update(
            name,
            timestamp_ns=_timestamp_ns(message),
            frame_id=frame_id,
            valid=True,
        )


def _parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument("--timeout-seconds", type=float, default=15.0)
    parser.add_argument("--arm-dof", type=int, default=5)
    parser.add_argument("--gripper-dof", type=int, default=1)
    return parser


def _spin_until_complete(
    executor: SingleThreadedExecutor,
    recorder_node: MockEpisodeRecorderNode,
    timeout_seconds: float,
) -> str | None:
    deadline = time.monotonic() + timeout_seconds
    try:
        while not recorder_node.complete and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        return "keyboard_interrupt"
    if not recorder_node.complete:
        return "recording_timeout"
    return None


def _finish(recorder_node: MockEpisodeRecorderNode, abort_reason: str | None, *, accept: bool) -> int:
    if recorder_node.recorder.sample_count < 2:
        print(json.dumps({"error": abort_reason or "insufficient_samples"}, indent=2))
        return 1
    status = "aborted" if abort_reason else "accepted" if accept else "recorded"
    manifest_path, report = recorder_node.recorder.finalize(
        outcome_status=status,
        failure_reason=abort_reason,
    )
    print(
        json.dumps(
            {
                "manifest": str(manifest_path),
                "outcome": status,
                "quality": report.to_dict(),
            },
            indent=2,
        )
    )
    return 1 if abort_reason else 0


def publisher_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Publish deterministic Phase 0 mock ROS 2 topics.")
    parser.add_argument("--arm-dof", type=int, default=5)
    parser.add_argument("--gripper-dof", type=int, default=1)
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = MockTopicPublisher(arm_dof=args.arm_dof, gripper_dof=args.gripper_dof)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def recorder_main(argv: Sequence[str] | None = None) -> int:
    parser = _parser("Record Phase 0 episodes from mock ROS 2 topics.")
    parser.add_argument("--accept", action="store_true", help="mark completed synthetic output accepted")
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = MockEpisodeRecorderNode(
        args.output,
        target_samples=args.samples,
        arm_dof=args.arm_dof,
        gripper_dof=args.gripper_dof,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        abort_reason = _spin_until_complete(executor, node, args.timeout_seconds)
        return _finish(node, abort_reason, accept=args.accept)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


def demo_main(argv: Sequence[str] | None = None) -> int:
    args, ros_args = _parser("Publish and record one deterministic mock episode.").parse_known_args(argv)
    rclpy.init(args=ros_args)
    publisher = MockTopicPublisher(arm_dof=args.arm_dof, gripper_dof=args.gripper_dof)
    recorder = MockEpisodeRecorderNode(
        args.output,
        target_samples=args.samples,
        arm_dof=args.arm_dof,
        gripper_dof=args.gripper_dof,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(recorder)
    try:
        abort_reason = _spin_until_complete(executor, recorder, args.timeout_seconds)
        return _finish(recorder, abort_reason, accept=True)
    finally:
        executor.remove_node(recorder)
        executor.remove_node(publisher)
        recorder.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(demo_main())
