"""Shared device helpers of the real-arm tools (bus, port owner, register list). The commissioning run itself is archived
in sweepick_commission_v2_archived.py; real moves are made by sweepick_move_a.py.

Review copy of revision/sweepick_commission.py; the frozen revision source is unchanged.
"""
import fcntl
import os
import subprocess
import termios
from pathlib import Path
import sweepick_real_command as rc

IN = Path.home() / "sweepick_261007_commission/inputs"
CAL = Path.home() / ".config/dapier/lerobot-calibration"
RELEASE_TRIES = 3
READS = (("goal", "Goal_Position"), ("present", "Present_Position"), ("velocity", "Present_Velocity"), ("load", "Present_Load"), ("current", "Present_Current"), ("torque", "Torque_Enable"))


class Refuse(Exception):
    """Raised before anything was written."""


def real_bus(port):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    from scservo_sdk.scservo_def import COMM_SUCCESS
    from scservo_sdk.protocol_packet_handler import PKT_ID, PKT_LENGTH, PKT_ERROR, PKT_PARAMETER0
    bus = FeetechMotorsBus(port=port, motors={n: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES) for i, n in enumerate(rc.JOINTS)})
    packet_handler = bus.packet_handler

    def read_exact_response(port, motor_id, length):
        packet, result = packet_handler.rxPacket(port)
        if result != COMM_SUCCESS:
            return [], result, 0
        # The SDK validates checksum but otherwise slices payload and skips other IDs.
        if (len(packet) != PKT_PARAMETER0 + length + 1
                or packet[PKT_LENGTH] != length + 2 or packet[PKT_ID] != motor_id):
            raise ConnectionError(
                f"response_contract: expected ID={motor_id} LENGTH={length + 2}; "
                f"received packet={bytes(packet).hex()}")
        if packet[PKT_ERROR]:
            raise RuntimeError(
                f"servo response error: ID={motor_id} error=0x{packet[PKT_ERROR]:02x} "
                f"{packet_handler.getRxPacketError(packet[PKT_ERROR])}")
        return packet[PKT_PARAMETER0:-1], result, 0

    packet_handler.readRx = read_exact_response
    handler = bus.port_handler
    open_port, close_port, connect = handler.openPort, handler.closePort, bus.connect
    claimed = False

    def close_exclusive():
        nonlocal claimed
        try:
            if claimed:
                fcntl.ioctl(handler.ser.fileno(), termios.TIOCNXCL)
        finally:
            claimed = False
            close_port()

    def open_exclusive():
        nonlocal claimed
        if not open_port():
            return False
        try:
            fd = handler.ser.fileno()
            fcntl.ioctl(fd, termios.TIOCEXCL)
            claimed = True
            local = sorted(int(p.name) for p in Path("/proc/self/fd").iterdir()
                           if p.name.isdecimal() and int(p.name) != fd and os.path.realpath(p) == os.path.realpath(port))
            if local:
                raise Refuse(f"{port} is already open by this process on file descriptor(s) {local}")
            pids = {int(pid) for pid in port_owner(port).split() if pid.isdecimal()}
            if os.getpid() not in pids:
                raise Refuse(f"{port} ownership could not be confirmed after exclusive open")
            foreign = sorted(pids - {os.getpid()})
            if foreign:
                raise Refuse(f"{port} is already open by foreign process(es) {foreign}")
            return True
        except BaseException:
            try:
                close_exclusive()
            except BaseException:
                pass
            raise

    def connect_exclusive(*args, **kwargs):
        was_open = handler.is_open
        try:
            return connect(*args, **kwargs)
        except BaseException:
            if not was_open and handler.is_open:
                try:
                    handler.closePort()
                except BaseException:
                    pass
            raise

    handler.openPort, handler.closePort, bus.connect = open_exclusive, close_exclusive, connect_exclusive
    return bus


def port_owner(port):
    return subprocess.run(["fuser", port], capture_output=True, text=True).stdout.strip()
