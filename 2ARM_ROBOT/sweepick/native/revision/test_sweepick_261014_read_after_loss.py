"""File-only replay of the two stored drive-initialization losses of 2026-10-14 (observation_09: third seed Goal write
unanswered; observation_07: the read after five acknowledged seed writes failed) on a fake bus. No device is opened.
These show what the executor RECORDS and SENDS after the loss; they say nothing about why the servo did not answer."""
import pytest
import sweepick_move_a as a
from test_sweepick_move_a import Fake as Base, go, IN

pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
OBS09 = dict(ticks=dict(shoulder_pan=2080, shoulder_lift=1108, elbow_flex=3142, wrist_flex=2045, wrist_roll=2048, gripper=3344),         # observation_09/pregrasp/run1/move.json start
             goal=dict(shoulder_pan=2075, shoulder_lift=1099, elbow_flex=3144, wrist_flex=2033, wrist_roll=2046, gripper=3350),
             target=dict(shoulder_pan=2383, shoulder_lift=2240, elbow_flex=1963, wrist_flex=3033, wrist_roll=2432))
OBS07 = dict(ticks=dict(shoulder_pan=2075, shoulder_lift=1099, elbow_flex=3144, wrist_flex=2033, wrist_roll=2046, gripper=3350),         # observation_07/pregrasp/run1/move.json start
             goal=dict(shoulder_pan=2072, shoulder_lift=1091, elbow_flex=3149, wrist_flex=2021, wrist_roll=2045, gripper=3356),
             target=dict(shoulder_pan=2379, shoulder_lift=2234, elbow_flex=1973, wrist_flex=3031, wrist_roll=2412))


class Stored(Base):
    """lost_goal_write=k: the k-th Goal write is not applied and not answered (observation_09, k=3).
    lost_read_after_goals=k: the first read after k Goal writes fails once (observation_07, k=5).
    then_dead: nothing answers after the loss. A Goal write powers its joint, as recorded on this device."""
    def __init__(self, rec, **fail):
        super().__init__(rec["ticks"], **fail); self.reg["Goal_Position"] = dict(rec["goal"]); self.log, self.goals, self.lost = [], 0, False
    def sync_read(self, reg, normalize=True):
        self.log.append(("read", reg))
        if self.fail.get("lost_read_after_goals") == self.goals and not self.lost:
            self.lost = True; self.dead = bool(self.fail.get("then_dead")); raise ConnectionError("response_contract: expected ID=4 LENGTH=4; received packet=ffff050400fe07f1")
        return super().sync_read(reg, normalize)
    def write(self, reg, n, v, normalize=True):
        self.log.append(("write", reg, n, v))
        if reg == "Goal_Position":
            self.goals += 1
            if self.fail.get("lost_goal_write") == self.goals:
                self.lost = True; self.dead = bool(self.fail.get("then_dead")); raise ConnectionError("Failed to write 'Goal_Position' on id_=3 with '3142' after 1 tries. [TxRxResult] There is no status packet!")
        super().write(reg, n, v, normalize)
        if reg == "Goal_Position": self.reg["Torque_Enable"][n] = 1


def after_loss(f):
    i = max(k for k, e in enumerate(f.log) if e[0] == "write"); return f.log[i + 1:] if f.fail.get("lost_goal_write") else f.log[next(k for k, e in enumerate(f.log) if e == ("read", "Goal_Position") and k > i) + 1:]


def test_observation_09_third_seed_write_unanswered_is_written_once_more_with_the_same_seed(tmp_path):
    """2026-10-18 contract change (three field BUS_LOSTs on seed writes): a SEED write that got no answer is read back and,
    the joint still standing on that seed, written once more with the SAME value; then the move goes on."""
    f = Stored(OBS09, lost_goal_write=3); code, log = go(tmp_path / "o", f, target=OBS09["target"], execute=True)
    writes = [e for e in f.log if e[0] == "write"]
    assert writes[:4] == [("write", "Goal_Position", "shoulder_pan", 2080), ("write", "Goal_Position", "shoulder_lift", 1108), ("write", "Goal_Position", "elbow_flex", 3142), ("write", "Goal_Position", "elbow_flex", 3142)]
    rec = [e for e in log["events"] if isinstance(e, dict) and e.get("seed_write_answer_lost") == "elbow_flex"]
    assert rec and rec[0]["recovery"] == "SAME_SEED_WRITTEN_ONCE_MORE_AND_ACKNOWLEDGED" and log["result"] != "BUS_LOST"

def test_observation_07_failed_read_after_five_seed_writes_is_read_once_more_and_the_move_goes_on(tmp_path):
    """2026-10-18 contract change: a read in drive initialization that fails once (a late answer of another servo) is read
    once more after the stale bytes are dropped; no write is repeated for it."""
    f = Stored(OBS07, lost_read_after_goals=5); code, log = go(tmp_path / "o", f, target=OBS07["target"], execute=True)
    rec = [e for e in log["events"] if isinstance(e, dict) and (e.get("init_read_lost") or e.get("event") == "loop_read_recovered")]
    assert rec and (rec[0].get("recovery") == "READ_REPEATED_AND_ANSWERED" or rec[0].get("event") == "loop_read_recovered") and log["result"] != "BUS_LOST"   # 194127: the loop read itself is asked again first
    seeds = [e for e in f.log if e[0] == "write" and e[1] == "Goal_Position"][:5]
    assert len(seeds) == 5 and len({e[2] for e in seeds}) == 5                 # each seed written once, none repeated

@pytest.mark.parametrize("rec, fail", [(OBS09, dict(lost_goal_write=3)), (OBS07, dict(lost_read_after_goals=5))])
def test_when_the_bus_stays_silent_the_record_is_the_previous_one(tmp_path, rec, fail):
    f = Stored(rec, then_dead=True, **fail); code, log = go(tmp_path / "o", f, target=rec["target"], execute=True)
    assert code == 15 and log["result"] == "BUS_LOST" and log["end"] == dict(error="no read possible", torque="UNKNOWN") and log["outcome"]["torque_released"] == "UNKNOWN"
    reads = after_loss(f)                                                    # 2026-10-18: a lost SEED adds the recovery's one read-back; still nothing is written
    assert log["read_after_loss"]["answered"] is False and "NOT known" in log["needs_a_person"] and 1 <= len(reads) <= 16 and all(e[0] == "read" for e in reads)   # 194127 + 215335: a loop read is asked up to 4 times


def test_the_answered_seed_path_is_unchanged(tmp_path):
    f = Stored(OBS09); code, log = go(tmp_path / "o", f, target=OBS09["target"], execute=True)
    assert code == 0 and log["result"] == "HOLDING" and "read_after_loss" not in log and log["goal_seed"]["joints"] == ARM and log["writes"]["Torque_Enable"] == 5
