"""기존 무장치 시험의 fake bus만 재사용한다. 보관된 commissioning writer는 포함하지 않는다."""
from pathlib import Path
from sweepick.control import sweepick_joint_command_mapping as rc

IN = Path.home() / "sweepick_261007_commission/inputs"

START = dict(shoulder_pan=2040, shoulder_lift=1031, elbow_flex=3150, wrist_flex=2043, wrist_roll=2116, gripper=3376)

LO = dict(shoulder_pan=726, shoulder_lift=772, elbow_flex=941, wrist_flex=866, wrist_roll=0, gripper=2001)

HI = dict(shoulder_pan=3406, shoulder_lift=3321, elbow_flex=3153, wrist_flex=3160, wrist_roll=4095, gripper=3431)

class Fake:
    """fail: dict of injected faults. Records every write."""
    def __init__(self, **fail):
        self.reg = dict(Goal_Position={n: 0 for n in rc.JOINTS}, Present_Position=dict(START), Torque_Enable={n: 0 for n in rc.JOINTS}, Protection_Current={n: 310 for n in rc.JOINTS}, Min_Position_Limit=dict(LO), Max_Position_Limit=dict(HI))
        self.fail, self.writes, self.n_goal, self.dead, self.disconnected = fail, [], 0, False, None
    def connect(self):
        if self.fail.get("connect"): raise ConnectionError("no port")
    def disconnect(self, disable_torque=True): self.disconnected = disable_torque
    def sync_read(self, reg, normalize=True):
        assert normalize is False
        if self.dead: raise ConnectionError("bus dead")
        if reg in ("Present_Velocity", "Present_Load", "Present_Current"):
            return {n: (self.fail.get("current", 0) if reg == "Present_Current" else 0) for n in rc.JOINTS}
        if reg == "Goal_Position" and self.fail.get("goal_readback") and self.n_goal:   # wrong only after the goals were written
            return {n: 7 for n in rc.JOINTS}
        return dict(self.reg[reg])
    def write(self, reg, n, v, normalize=True):
        assert normalize is False and reg in ("Goal_Position", "Torque_Enable")
        if self.dead: raise ConnectionError("bus dead")
        if reg == "Torque_Enable" and v == 0 and n in self.fail.get("release_fail", ()):
            self.fail["release_fail_count"] = self.fail.get("release_fail_count", 0) + 1
            if self.fail["release_fail_count"] <= self.fail.get("release_fail_times", 99): raise ConnectionError("no status packet")
        if reg == "Goal_Position":
            self.n_goal += 1
            if self.fail.get("write_fail_at") == self.n_goal: raise ConnectionError("write timeout")
            if self.fail.get("die_at") == self.n_goal: self.dead = True; raise ConnectionError("bus dead")
        self.writes.append((reg, n, v)); self.reg[reg][n] = v
        if reg == "Goal_Position" and self.reg["Torque_Enable"][n] and n != self.fail.get("stuck") and not (self.fail.get("no_return") and self.n_goal > self.fail["no_return"]):
            self.reg["Present_Position"][n] = v
