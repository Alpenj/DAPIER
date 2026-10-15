"""2026-10-15: a failed bus open records the exception chain lerobot hides behind an empty first line. The gate
(bus_lost), result (REFUSED), exit code and the absence of any write are unchanged."""
import pytest
import sweepick_move_a as a, sweepick_real_command as rc
from test_obs_reach_20261015 import powered, mapping, view, S03, T03, ARM
from test_sweepick_move_a import Clock, IN

pytestmark = pytest.mark.skipif(not (IN / "CANDIDATE_Q12_CONFIG.json").exists(), reason="candidate config not on this machine")
LEROBOT = "\nCould not connect on port '/dev/dapier/right_arm'. Make sure you are using the correct port.\nTry running `lerobot-find-port`\n"


def failing(make):
    f = powered(S03, lag=0)
    def connect():
        make()
    f.connect = connect
    return f


def wrapped(cause):
    try:
        try:
            raise cause
        except Exception as e:
            raise ConnectionError(LEROBOT) from e
    except ConnectionError as w:
        raise w


def run(tmp, f):
    c = Clock()
    return a.run("next", "right", tmp, target=T03, torque_joints=ARM, open_bus=lambda port: f, owner=lambda p: "", mapping=mapping(), mapping_sha="m",
                 sleep=c.sleep, clock=c.now, execute=True, observation_view=True, settle_trim=())


def common(code, log, f):
    assert code == a.EXIT["REFUSED"] and log["result"] == "REFUSED" and log["refused"]["gate"] == "bus_lost" and log["refused"]["level"] == "HARD"
    assert log["writes"] == dict(Goal_Position=0, Torque_Enable=0) and f.writes == []
    return log["bus_open_error"]


def test_empty_first_line_with_an_os_cause_records_errno(tmp_path):
    f = failing(lambda: wrapped(OSError(16, "Device or resource busy")))
    r = common(*run(tmp_path / "o", f), f)
    assert r["type"] == "ConnectionError" and r["original_type"] == "OSError" and r["errno"] == 16 and r["strerror"] == "Device or resource busy"
    assert r["stage"] == "connect" and r["port"] == "/dev/dapier/right_arm" and r["message"].startswith("Could not connect on port")
    assert [c["link"] for c in r["chain"]] == ["raised", "cause"]
    detail = run(tmp_path / "o2", failing(lambda: wrapped(OSError(16, "Device or resource busy"))))[1]["refused"]["detail"]
    assert detail.startswith("the bus could not be opened: ConnectionError: Could not connect") and "errno 16" in detail


def test_cause_without_errno_stays_none(tmp_path):
    class SerialException(IOError):
        pass
    f = failing(lambda: wrapped(SerialException("could not open port /dev/dapier/right_arm: [Errno 2] No such file or directory")))
    r = common(*run(tmp_path / "o", f), f)
    assert r["original_type"] == "SerialException" and r["errno"] is None and r["strerror"] is None and "No such file" in r["chain"][1]["message"]


def test_bare_connection_error_with_empty_first_line(tmp_path):
    def bare():
        raise ConnectionError(LEROBOT)
    f = failing(bare)
    r = common(*run(tmp_path / "o", f), f)
    assert r["original_type"] is None and r["errno"] is None and len(r["chain"]) == 1 and r["message"].startswith("Could not connect")


def test_failure_while_constructing_the_bus_is_recorded_as_that_stage(tmp_path):
    c = Clock()
    def boom(port):
        raise FileNotFoundError(2, "No such file or directory")
    code, log = a.run("next", "right", tmp_path / "o", target=T03, torque_joints=ARM, open_bus=boom, owner=lambda p: "", mapping=mapping(), mapping_sha="m",
                      sleep=c.sleep, clock=c.now, execute=True)
    assert code == a.EXIT["REFUSED"] and log["refused"]["gate"] == "bus_lost" and log["bus_open_error"]["stage"] == "construct" and log["bus_open_error"]["errno"] == 2


def test_grip_open_failure_records_the_same_chain(tmp_path):
    f = failing(lambda: wrapped(OSError(5, "Input/output error")))
    c = Clock()
    code, log = a.grip("right", tmp_path / "g", torque_joints=["gripper"], open_bus=lambda port: f, owner=lambda p: "", mapping=mapping(), mapping_sha="m",
                       sleep=c.sleep, clock=c.now, execute=True)
    assert log["result"] == "REFUSED" and log["refused"]["gate"] == "bus_lost" and log["bus_open_error"]["errno"] == 5 and f.writes == []
