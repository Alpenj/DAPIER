#!/usr/bin/env python3
"""Keyboard teleop for a Humble TurtleBot3 Burger from a Jazzy PC.

Publishes plain geometry_msgs/Twist at 20 Hz. This deliberately avoids the
Jazzy TurtleBot3 teleop executable, which publishes TwistStamped and therefore
does not match the Humble turtlebot3_node running on the Jetson Nano.
"""

import argparse
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.signals import SignalHandlerOptions


LINEAR_STEP = 0.02
ANGULAR_STEP = 0.15
MAPPING_MAX_LINEAR = 0.18
SPORT_MAX_LINEAR = 0.22
MAX_ANGULAR = 1.50
MAX_WHEEL_LINEAR = 0.22
HALF_WHEEL_SEPARATION = 0.080
MIN_INNER_WHEEL_RATIO = 0.20
MAX_CURVED_ANGULAR = (
    MAX_WHEEL_LINEAR
    * (1.0 - MIN_INNER_WHEEL_RATIO)
    / (2.0 * HALF_WHEEL_SEPARATION)
)
CONTROL_PERIOD = 0.05
MAX_LINEAR_ACCEL = 0.35
MAX_ANGULAR_ACCEL = 1.20

HELP = """
Physical TurtleBot3 Burger teleop (motor-aware limits)
-------------------------------------------------------
        w
   a    s    d
        x

w/x : increase/decrease forward speed
a/d : steer left/right while keeping the current speed
r   : straighten steering without stopping
s or SPACE : stop
Ctrl-C : stop and quit

At zero linear speed, a/d turns in place. At higher speed, steering is limited
so the inner wheel keeps moving and neither wheel exceeds 0.22 m/s.
Speed and steering ramp smoothly; s, SPACE, and Ctrl-C still stop immediately.
"""


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def limit_motion(
    linear: float, angular: float, max_linear: float = MAPPING_MAX_LINEAR
) -> tuple[float, float]:
    """Apply Burger limits while preserving useful curved steering.

    At high speed, a steering request reduces linear speed just enough to keep
    the faster wheel within the Burger's 0.22 m/s specification. At low speed,
    angular speed is reduced instead so steering never accelerates the robot.
    """
    linear = clamp(linear, max_linear)
    if abs(linear) < 1e-9:
        return 0.0, clamp(angular, MAX_ANGULAR)

    # inner >= ratio * outer gives:
    # |w| <= (1-ratio)*|v| / ((1+ratio)*half_wheel_separation)
    curve_budget = (
        (1.0 - MIN_INNER_WHEEL_RATIO)
        * abs(linear)
        / ((1.0 + MIN_INNER_WHEEL_RATIO) * HALF_WHEEL_SEPARATION)
    )
    angular = clamp(
        angular, min(MAX_ANGULAR, MAX_CURVED_ANGULAR, curve_budget)
    )

    # outer = |v| + |w|*half_wheel_separation. Preserve the steering
    # request and reduce |v| only when the outer wheel would exceed 0.22 m/s.
    curved_linear_limit = (
        MAX_WHEEL_LINEAR - abs(angular) * HALF_WHEEL_SEPARATION
    )
    linear = clamp(linear, max(0.0, min(max_linear, curved_linear_limit)))
    return linear, angular


def wheel_speeds(linear: float, angular: float) -> tuple[float, float]:
    left = linear - angular * HALF_WHEEL_SEPARATION
    right = linear + angular * HALF_WHEEL_SEPARATION
    return left, right


def slew(current: float, target: float, max_rate: float, seconds: float) -> float:
    """Move current toward target without exceeding max_rate per second."""
    if max_rate <= 0.0 or seconds < 0.0:
        raise ValueError("max_rate must be positive and seconds non-negative")
    maximum_change = max_rate * seconds
    difference = target - current
    if abs(difference) <= maximum_change:
        return target
    return current + maximum_change * (1.0 if difference > 0.0 else -1.0)


def make_twist(linear: float, angular: float) -> Twist:
    message = Twist()
    message.linear.x = linear
    message.angular.z = angular
    return message


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sport",
        action="store_true",
        help="allow the Burger's official 0.22 m/s translational maximum",
    )
    args = parser.parse_args()
    max_linear = SPORT_MAX_LINEAR if args.sport else MAPPING_MAX_LINEAR
    mode = "SPORT" if args.sport else "MAPPING"

    if not sys.stdin.isatty():
        raise SystemExit("ERROR: teleop requires an interactive terminal")

    original_settings = termios.tcgetattr(sys.stdin)
    # Keep the ROS context alive after Ctrl-C until zero-velocity packets have
    # been published. The default rclpy SIGINT handler shuts it down too early.
    rclpy.init(
        args=["--ros-args", "--log-level", "error"],
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = rclpy.create_node("tb3_real_teleop")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    target_linear = 0.0
    target_angular = 0.0
    command_linear = 0.0
    command_angular = 0.0
    previous_time = time.monotonic()
    previous_display = previous_time

    print(HELP)
    print(f"mode={mode} linear limit=+/-{max_linear:.2f} m/s")
    print(f"linear={command_linear:.2f} m/s angular={command_angular:.2f} rad/s")

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], CONTROL_PERIOD)
            immediate_stop = False
            if readable:
                key = sys.stdin.read(1)
                if key == "w":
                    target_linear += LINEAR_STEP
                elif key == "x":
                    target_linear -= LINEAR_STEP
                elif key == "a":
                    target_angular += ANGULAR_STEP
                elif key == "d":
                    target_angular -= ANGULAR_STEP
                elif key == "r":
                    target_angular = 0.0
                elif key in ("s", " "):
                    target_linear = 0.0
                    target_angular = 0.0
                    immediate_stop = True
                elif key == "\x03":
                    break
                else:
                    target_linear = 0.0
                    target_angular = 0.0
                    immediate_stop = True
                target_linear, target_angular = limit_motion(
                    target_linear, target_angular, max_linear
                )

            now = time.monotonic()
            seconds = min(0.2, max(0.0, now - previous_time))
            previous_time = now
            if immediate_stop:
                command_linear = 0.0
                command_angular = 0.0
            else:
                command_linear = slew(
                    command_linear, target_linear, MAX_LINEAR_ACCEL, seconds
                )
                command_angular = slew(
                    command_angular, target_angular, MAX_ANGULAR_ACCEL, seconds
                )
                command_linear, command_angular = limit_motion(
                    command_linear, command_angular, max_linear
                )

            if readable or now - previous_display >= 0.20:
                left, right = wheel_speeds(command_linear, command_angular)
                print(
                    f"\rlinear={command_linear:+.2f}/{target_linear:+.2f} m/s "
                    f"angular={command_angular:+.2f}/{target_angular:+.2f} rad/s "
                    f"wheels L={left:+.2f} R={right:+.2f} m/s   ",
                    end="",
                    flush=True,
                )
                previous_display = now

            publisher.publish(make_twist(command_linear, command_angular))
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        stop = make_twist(0.0, 0.0)
        for _ in range(3):
            publisher.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        print("\nStopped.")


if __name__ == "__main__":
    main()
