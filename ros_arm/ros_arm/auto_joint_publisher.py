"""Publish a safe four-joint test pose once per second."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = (
    'base_shoulder',
    'shoulder_arm1',
    'arm1_arm2',
    'arn2_end_arm',
)


class AutoJointPublisher(Node):
    """Move one joint at a time around its center for mapping tests."""

    def __init__(self):
        super().__init__('auto_joint_publisher')
        self.declare_parameter('period_seconds', 1.0)
        self.declare_parameter('amplitude_degrees', 10.0)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('publish_controller_commands', False)

        period = float(self.get_parameter('period_seconds').value)
        self.amplitude = float(
            self.get_parameter('amplitude_degrees').value)
        self.publish_joint_states = bool(
            self.get_parameter('publish_joint_states').value)
        self.publish_controller_commands = bool(
            self.get_parameter('publish_controller_commands').value)
        self.joint_state_publisher = self.create_publisher(
            JointState, '/joint_states', 10
        ) if self.publish_joint_states else None
        self.controller_publisher = self.create_publisher(
            Float64MultiArray, '/arm_position_controller/commands', 10
        ) if self.publish_controller_commands else None

        # For each joint: center -> +amplitude -> -amplitude -> center.
        self.test_steps = []
        center = [0.0] * len(JOINT_NAMES)
        self.test_steps.append(('all center', center))
        for index, name in enumerate(JOINT_NAMES):
            positive = center.copy()
            positive[index] = self.amplitude
            negative = center.copy()
            negative[index] = -self.amplitude
            self.test_steps.extend([
                (f'{name} +{self.amplitude:g} deg', positive),
                (f'{name} -{self.amplitude:g} deg', negative),
                (f'{name} center', center.copy()),
            ])

        self.step_index = 0
        self.timer = self.create_timer(period, self.publish_next_step)
        self.get_logger().info(
            f'Publishing one safe test pose every {period:g}s; '
            f'amplitude=±{self.amplitude:g}°')

    def publish_next_step(self):
        label, degrees = self.test_steps[self.step_index]
        radians = [math.radians(value) for value in degrees]
        if self.joint_state_publisher is not None:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(JOINT_NAMES)
            message.position = radians
            self.joint_state_publisher.publish(message)
        if self.controller_publisher is not None:
            command = Float64MultiArray()
            command.data = radians
            self.controller_publisher.publish(command)
        servo_degrees = [round(90 + value) for value in degrees]
        self.get_logger().info(
            f'Step {self.step_index + 1}/{len(self.test_steps)} '
            f'{label}: servo={servo_degrees}')
        self.step_index = (self.step_index + 1) % len(self.test_steps)


def main(args=None):
    rclpy.init(args=args)
    node = AutoJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
