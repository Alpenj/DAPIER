"""Focused fake-bus checks for run() feedback order after actuator writes."""
from test_sweepick_move_a import Fake, MID, go, goals


NORMAL_READS = ["Goal_Position", "Present_Position", "Present_Velocity", "Present_Load",
                "Present_Current", "Torque_Enable", "Status", "Moving"]
POSTWRITE_READS = ["Goal_Position", "Torque_Enable", "Status", "Moving",
                   "Present_Velocity", "Present_Load", "Present_Current", "Present_Position"]


class OrderedFake(Fake):
    def __init__(self, start=MID, *, fail_postwrite_at=None):
        super().__init__(start)
        self.events = []
        self.fail_postwrite_at = fail_postwrite_at

    def sync_read(self, reg, normalize=True):
        self.events.append(("read", reg))
        if self.writes and reg == self.fail_postwrite_at:
            raise ConnectionError("postwrite read failed")
        return super().sync_read(reg, normalize)

    def write(self, reg, name, value, normalize=True):
        result = super().write(reg, name, value, normalize)
        self.events.append(("write", reg, name, value))
        return result


def reads_after(events, write_index):
    reads = []
    for event in events[write_index + 1:]:
        if event[0] == "write":
            break
        reads.append(event[1])
    return reads


def test_seed_postwrite_read_has_all_fields_in_acknowledgement_first_order(tmp_path):
    fake = OrderedFake()
    code, _ = go(tmp_path / "run", fake, target={"gripper": 3000}, execute=True)

    first_write = next(i for i, event in enumerate(fake.events) if event[0] == "write")
    assert code == 0
    assert reads_after(fake.events, first_write) == POSTWRITE_READS


def test_normal_read_order_is_unchanged_without_a_pending_write(tmp_path):
    fake = OrderedFake()
    code, _ = go(tmp_path / "plan", fake, mode="plan", target={"gripper": 3000})
    feedback_reads = [event[1] for event in fake.events
                      if event[0] == "read" and event[1] not in ("Min_Position_Limit", "Max_Position_Limit")]

    assert code == 0
    assert fake.writes == []
    assert feedback_reads[:8] == NORMAL_READS


def test_failed_postwrite_read_stops_before_any_later_actuation(tmp_path):
    fake = OrderedFake(fail_postwrite_at="Status")
    code, log = go(tmp_path / "run", fake, target={"gripper": 3000}, execute=True)

    first_write = next(i for i, event in enumerate(fake.events) if event[0] == "write")
    assert code == 15 and log["stopped_by"]["gate"] == "bus_lost"
    assert fake.writes == [("Goal_Position", "gripper", MID["gripper"])]
    assert reads_after(fake.events, first_write) == POSTWRITE_READS[:3] * 17           # 2026-10-18: the drive-initialization read is repeated up to 3 times (no write is); then the record read; 194127 + 215335: each loop read is asked up to 4 times
    assert log["read_after_loss"]["answered"] is False
    assert log["read_after_loss"]["failed_register"] == "Status"
    assert log["end"]["goal_register"] == fake.reg["Goal_Position"]
    assert log["end"]["torque"] == fake.reg["Torque_Enable"]
    assert "status" not in log["end"]


def test_postwrite_order_keeps_fresh_goal_deduplication(tmp_path):
    fake = OrderedFake()
    fake.reg["Torque_Enable"]["elbow_flex"] = 1
    fake.reg["Goal_Position"]["elbow_flex"] = MID["elbow_flex"]

    code, log = go(tmp_path / "run", fake, mode="next", target={"elbow_flex": MID["elbow_flex"] - 1},
                   torque_joints=["elbow_flex"], execute=True)

    assert code == 0
    assert goals(fake) == [MID["elbow_flex"] - 1]
    assert log["unchanged_arm_goal_writes_avoided"] > 0
