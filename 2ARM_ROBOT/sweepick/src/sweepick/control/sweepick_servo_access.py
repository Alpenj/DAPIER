"""Shared device helpers of the real-arm tools (bus, port owner, register list). The commissioning run itself is archived
in sweepick_commission_v2_archived.py; real moves are made by sweepick_move_a.py."""
import subprocess
from pathlib import Path
from sweepick.control import sweepick_joint_command_mapping as rc

IN = Path.home() / "sweepick_261007_commission/inputs"
CAL = Path.home() / ".config/dapier/lerobot-calibration"
RELEASE_TRIES = 3
READS = (("goal", "Goal_Position"), ("present", "Present_Position"), ("velocity", "Present_Velocity"), ("load", "Present_Load"), ("current", "Present_Current"), ("torque", "Torque_Enable"))


class Refuse(Exception):
    """Raised before anything was written."""


def real_bus(port):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    return FeetechMotorsBus(port=port, motors={n: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES) for i, n in enumerate(rc.JOINTS)})


def port_owner(port):
    return subprocess.run(["fuser", port], capture_output=True, text=True).stdout.strip()
