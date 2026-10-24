"""Hardware-free checks: python test_control_loop_guard.py."""
import signal
import time
from types import SimpleNamespace
import unittest
from unittest import mock

from control_loop_guard import ControlLoopGuard, guard_loop


class Bus:
    motors = {"joint": 1, "gripper": 6}

    def __init__(self, failure=None):
        self.failure = failure
        self.writes = []

    def write(self, register, motor, value, **kwargs):
        assert register == "Torque_Enable" and value == 0
        self.writes.append(motor)
        if self.failure == "blocked":
            time.sleep(1)
        if self.failure == "usb":
            raise OSError("USB disconnected")

    def read(self, *_args, **_kwargs):
        return 1 if self.failure == "readback" else 0


class GuardTest(unittest.TestCase):
    def setup_robot(self, failure=None):
        self.sent = []
        robot = SimpleNamespace(
            left_arm=SimpleNamespace(bus=Bus(failure)),
            right_arm=SimpleNamespace(bus=Bus()),
            send_action=lambda action: self.sent.append(action),
        )
        robot._dapier_control_guard = ControlLoopGuard(robot, 0.1, 0.05)
        return robot, robot._dapier_control_guard

    def test_normal_loop_preserves_commands_and_timer(self):
        robot, guard = self.setup_robot()
        previous = signal.getsignal(signal.SIGALRM)

        @guard_loop
        def loop(robot):
            robot.send_action({"gripper": 20})
            return 7

        self.assertEqual(loop(robot), 7)
        self.assertEqual(self.sent, [{"gripper": 20}])
        self.assertFalse(robot.left_arm.bus.writes)
        self.assertEqual(signal.getitimer(signal.ITIMER_REAL), (0, 0))
        self.assertEqual(signal.getsignal(signal.SIGALRM), previous)
        with self.assertRaises(RuntimeError):
            robot.send_action({})

    def test_fault_stops_both_sides_before_caller_cleanup_and_latches(self):
        for failure in (None, "usb", "readback", "blocked"):
            with self.subTest(failure=failure):
                robot, guard = self.setup_robot(failure)
                def loop():
                    raise OSError("camera or serial failed")
                with self.assertRaises(OSError):
                    guard.run(loop)
                self.assertEqual(robot.left_arm.bus.writes, ["joint", "gripper"])
                self.assertEqual(robot.right_arm.bus.writes, ["joint", "gripper"])
                self.assertEqual(bool(guard.stop_errors), failure is not None)
                with self.assertRaises(RuntimeError):
                    robot.send_action({})
                with self.assertRaises(RuntimeError):
                    guard.run(lambda: None)

    def test_stalled_loop_interrupt_and_keyboard_interrupt(self):
        for exception, loop in (
            (TimeoutError, lambda: time.sleep(1)),
            (KeyboardInterrupt, lambda: (_ for _ in ()).throw(KeyboardInterrupt())),
        ):
            robot, guard = self.setup_robot()
            started = time.monotonic()
            with self.assertRaises(exception):
                guard.run(loop)
            self.assertLess(time.monotonic() - started, 0.8)
            self.assertTrue(guard.faulted)
            self.assertTrue(robot.right_arm.bus.writes)

    def test_bad_deadline_rejected(self):
        robot, _ = self.setup_robot()
        for timeout in (0, -1, float("nan"), float("inf"), 3):
            with self.assertRaises(ValueError):
                ControlLoopGuard(robot, timeout)

    def test_successful_commands_refresh_deadline(self):
        robot, guard = self.setup_robot()
        def loop():
            for _ in range(6):
                time.sleep(0.03)
                robot.send_action({})
        guard.run(loop)
        self.assertEqual(len(self.sent), 6)
        self.assertFalse(guard.faulted)

    def test_late_command_is_rejected_even_if_signal_delivery_is_delayed(self):
        robot, guard = self.setup_robot()
        def loop():
            guard.deadline = time.monotonic() - 1
            robot.send_action({})
        with mock.patch.object(signal, "setitimer"):
            with self.assertRaises(TimeoutError):
                guard.run(loop)
        self.assertFalse(self.sent)
        self.assertTrue(guard.faulted)

    def test_existing_timer_is_not_replaced(self):
        _, guard = self.setup_robot()
        with mock.patch.object(signal, "getitimer", return_value=(5.0, 0.0)):
            with self.assertRaises(RuntimeError):
                guard.run(lambda: None)


if __name__ == "__main__":
    unittest.main()
