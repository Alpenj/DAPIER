"""Supplemental Linux host guard, NOT a device-side watchdog or permission to move.

SIGKILL, OS/GIL/native-code stalls and lost USB can prevent stopping. The physical
watchdog/identity/approval gates remain required. Torque-off can let an arm drop.
Only active control loops are timed; connection, episode saving and cleanup are
not covered. This does not establish hardware readiness.
"""

from functools import wraps
import inspect
import logging
import math
import signal
import threading
import time


class ControlLoopGuard:
    def __init__(self, robot, timeout_s=1.0, stop_timeout_s=0.25):
        for value in (timeout_s, stop_timeout_s):
            if not math.isfinite(value) or not 0.05 <= value <= 2:
                raise ValueError("guard deadlines must be finite and within 0.05..2 seconds")
        self.robot = robot
        self.timeout_s = timeout_s
        self.stop_timeout_s = stop_timeout_s
        self.active = False
        self.faulted = False
        self.deadline = 0.0
        self.stop_errors = []
        send = robot.send_action

        @wraps(send)
        def guarded_send(*args, **kwargs):
            if self.faulted or not self.active:
                raise RuntimeError("control commands blocked: inactive or fault latched")
            self._check_deadline()
            result = send(*args, **kwargs)
            self._check_deadline()
            self._refresh()
            return result

        robot.send_action = guarded_send

    @staticmethod
    def _expired(*_):
        raise TimeoutError("control/stop deadline expired")

    def _check_deadline(self):
        if time.monotonic() >= self.deadline:
            self._expired()

    def _refresh(self):
        self.deadline = time.monotonic() + self.timeout_s
        signal.setitimer(signal.ITIMER_REAL, self.timeout_s)

    def _stop(self):
        # Each motor is attempted even when a previous port/motor failed.
        # ponytail: best-effort host stop only; independent hardware must cover host/USB loss.
        for side in ("left_arm", "right_arm"):
            bus = getattr(self.robot, side).bus
            for motor in bus.motors:
                try:
                    signal.setitimer(signal.ITIMER_REAL, self.stop_timeout_s)
                    bus.write("Torque_Enable", motor, 0, normalize=False, num_retry=0)
                    if bus.read("Torque_Enable", motor, normalize=False, num_retry=0) != 0:
                        raise RuntimeError("torque-off readback failed")
                except BaseException as error:
                    self.stop_errors.append(f"{side}/{motor}: {type(error).__name__}")
                finally:
                    signal.setitimer(signal.ITIMER_REAL, 0)
        if self.stop_errors:
            logging.critical("STOP UNCONFIRMED; use physical power cut: %s", self.stop_errors)
        else:
            logging.error("Fault latched; all follower torque-off registers read back as zero")

    def run(self, loop, *args, **kwargs):
        if self.faulted or self.active:
            raise RuntimeError("control guard cannot rearm a faulted/nested loop")
        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError("control guard requires the main thread")
        if signal.getitimer(signal.ITIMER_REAL) != (0.0, 0.0):
            raise RuntimeError("control guard refuses to replace an existing timer")
        previous = signal.getsignal(signal.SIGALRM)
        signal.signal(signal.SIGALRM, self._expired)
        self.active = True
        try:
            self._refresh()
            return loop(*args, **kwargs)
        except BaseException:
            self.faulted = True
            self.active = False
            signal.setitimer(signal.ITIMER_REAL, 0)
            self._stop()
            raise
        finally:
            self.active = False
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, previous)


def guard_loop(loop):
    signature = inspect.signature(loop)

    @wraps(loop)
    def guarded(*args, **kwargs):
        robot = signature.bind(*args, **kwargs).arguments["robot"]
        return robot._dapier_control_guard.run(loop, *args, **kwargs)

    return guarded
