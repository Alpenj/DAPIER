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

"""ROS 2 ``JointTrajectory`` bridge for SO-101 MuJoCo and real followers."""

from __future__ import annotations

import time
from typing import Any

from .env import JOINT_NAMES
from .ros2_control import LinearJointTrajectory, SO101FollowerROSBackend, SO101MujocoROSBackend


def _duration_seconds(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def main(args: list[str] | None = None) -> None:
    """Run the bridge after ROS 2 has been sourced into the current shell."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import JointTrajectory
    except ImportError as exc:
        raise SystemExit(
            "ROS 2 Python packages are unavailable. Install/source ROS 2 Jazzy, then rerun this command."
        ) from exc

    class SO101BridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("so101_bridge")
            self.declare_parameter("backend", "mujoco")
            self.declare_parameter("rate_hz", 30.0)
            self.declare_parameter("command_topic", "/so101/joint_trajectory")
            self.declare_parameter("state_topic", "/joint_states")
            self.declare_parameter("follower_port", "")
            self.declare_parameter("follower_id", "so101_follower")
            self.declare_parameter("max_relative_target", 5.0)
            self.declare_parameter("calibrate", True)

            backend_name = str(self.get_parameter("backend").value)
            rate_hz = float(self.get_parameter("rate_hz").value)
            if rate_hz <= 0:
                raise ValueError(f"rate_hz must be positive, got {rate_hz}")

            if backend_name == "mujoco":
                self.backend = SO101MujocoROSBackend(fps=round(rate_hz))
            elif backend_name == "follower":
                self.backend = SO101FollowerROSBackend(
                    port=str(self.get_parameter("follower_port").value),
                    robot_id=str(self.get_parameter("follower_id").value),
                    max_relative_target=float(self.get_parameter("max_relative_target").value),
                    calibrate=bool(self.get_parameter("calibrate").value),
                )
            else:
                raise ValueError(f"backend must be 'mujoco' or 'follower', got {backend_name!r}")

            command_topic = str(self.get_parameter("command_topic").value)
            state_topic = str(self.get_parameter("state_topic").value)
            self.publisher = self.create_publisher(JointState, state_topic, 10)
            self.subscription = self.create_subscription(
                JointTrajectory, command_topic, self._trajectory_callback, 10
            )
            self.reset_service = self.create_service(Trigger, "/so101/reset", self._reset_callback)
            self.trajectory: LinearJointTrajectory | None = None
            self.positions = self.backend.read_positions()
            self.timer = self.create_timer(1.0 / rate_hz, self._control_tick)
            self.get_logger().info(
                f"SO-101 bridge ready: backend={backend_name}, command={command_topic}, state={state_topic}"
            )

        def _trajectory_callback(self, message: JointTrajectory) -> None:
            try:
                self.trajectory = LinearJointTrajectory(
                    joint_names=message.joint_names,
                    positions=[point.positions for point in message.points],
                    times_from_start=[_duration_seconds(point.time_from_start) for point in message.points],
                    current_positions=self.positions,
                    start_time=time.monotonic(),
                )
            except ValueError as exc:
                self.get_logger().error(f"Rejected trajectory: {exc}")
                return
            self.get_logger().info(
                f"Accepted {len(message.points)} trajectory point(s), duration={self.trajectory.duration:.3f}s"
            )

        def _reset_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
            if not hasattr(self.backend, "reset"):
                response.success = False
                response.message = "Reset is only available for the MuJoCo backend"
                return response
            self.trajectory = None
            self.positions = self.backend.reset()
            response.success = True
            response.message = "SO-101 MuJoCo state reset"
            return response

        def _control_tick(self) -> None:
            try:
                if self.trajectory is not None:
                    target, complete = self.trajectory.sample(time.monotonic())
                    self.positions = self.backend.command_positions(target)
                    if complete:
                        self.trajectory = None
                else:
                    self.positions = self.backend.read_positions()
            except Exception as exc:
                self.trajectory = None
                self.get_logger().error(f"Control tick failed; holding the last target: {exc}")

            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(JOINT_NAMES)
            message.position = self.positions.tolist()
            self.publisher.publish(message)

        def destroy_node(self) -> bool:
            self.backend.close()
            return super().destroy_node()

    rclpy.init(args=args)
    node = None
    try:
        node = SO101BridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
