"""Bridge ROS 2 JointState commands to the Arduino robot arm over serial."""

import math
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RobotArmSerialBridge(Node):
    """Translate named URDF joint positions in radians to servo degrees."""

    # URDF joint name -> index in the Arduino A,b,s,f,u serial command.
    JOINT_ORDER = (
        'base_shoulder',
        'shoulder_arm1',
        'arm1_arm2',
        'arn2_end_arm',
    )

    def __init__(self):
        super().__init__('robot_arm_serial_bridge')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('center_angle', 90)
        self.declare_parameter('minimum_angle', 60)
        self.declare_parameter('maximum_angle', 120)
        self.declare_parameter('deadband_degrees', 2)
        self.declare_parameter('command_rate_hz', 15.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.center = self.get_parameter('center_angle').value
        self.minimum = self.get_parameter('minimum_angle').value
        self.maximum = self.get_parameter('maximum_angle').value
        self.deadband = self.get_parameter('deadband_degrees').value
        command_rate = self.get_parameter('command_rate_hz').value
        self.last_sent = None
        self.pending_angles = None

        try:
            self.serial = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as error:
            self.get_logger().fatal(f'Cannot open {port}: {error}')
            raise

        # Opening a Uno serial connection resets it; wait for its bootloader.
        time.sleep(2.0)
        self.serial.reset_input_buffer()
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.command_timer = self.create_timer(
            1.0 / command_rate, self.send_pending_command)
        self.read_timer = self.create_timer(0.05, self.read_serial_feedback)
        self.get_logger().info(
            f'Connected to Arduino on {port} at {baud} baud')
        self.get_logger().info(
            'Listening for /joint_states: ' + ', '.join(self.JOINT_ORDER))

    def joint_state_callback(self, message):
        """Map joints by name, convert radians to degrees, and send once."""
        positions = dict(zip(message.name, message.position))
        missing = [name for name in self.JOINT_ORDER if name not in positions]
        if missing:
            self.get_logger().warning(
                f'JointState is missing: {", ".join(missing)}',
                throttle_duration_sec=2.0)
            return

        servo_angles = []
        for name in self.JOINT_ORDER:
            # URDF zero radians corresponds to the physical 90-degree center.
            angle = round(self.center + math.degrees(positions[name]))
            angle = max(self.minimum, min(self.maximum, angle))
            servo_angles.append(angle)

        angles = tuple(servo_angles)
        if self.last_sent is not None:
            # Ignore tiny command noise that can make inexpensive analog
            # servos hunt back and forth around the target.
            angles = tuple(
                previous if abs(requested - previous) < self.deadband
                else requested
                for requested, previous in zip(angles, self.last_sent)
            )
        self.pending_angles = angles

    def send_pending_command(self):
        """Send at a bounded rate so rapid GUI updates cannot flood the Uno."""
        angles = self.pending_angles
        if angles is None or angles == self.last_sent:
            return

        command = 'A,' + ','.join(str(value) for value in angles) + '\n'
        try:
            self.serial.write(command.encode('ascii'))
            self.last_sent = angles
            self.get_logger().info(f'TX {command.strip()}')
        except serial.SerialException as error:
            self.get_logger().error(f'Serial write failed: {error}')

    def read_serial_feedback(self):
        """Log Arduino READY/OK/ERR responses without blocking ROS callbacks."""
        try:
            while self.serial.in_waiting:
                line = self.serial.readline().decode(
                    'ascii', errors='replace').strip()
                if line:
                    self.get_logger().info(f'RX {line}')
        except serial.SerialException as error:
            self.get_logger().error(f'Serial read failed: {error}')

    def destroy_node(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotArmSerialBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, serial.SerialException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
