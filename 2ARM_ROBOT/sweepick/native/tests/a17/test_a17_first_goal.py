"""A17: saved out-of-range actual is valid input; fake bus only."""
import sys
from pathlib import Path
import pytest

REV = Path(__file__).resolve().parents[2] / 'revision'
sys.path.insert(0, str(REV))
import sweepick_pick01 as pick
from test_a16_close_binding import ACTUAL, fake, certificate
from test_sweepick_grip import go, sent


@pytest.mark.parametrize('opening,first', [(3344., 3344), (3501., 3431)])
@pytest.mark.parametrize('powered', [False, True])
def test_first_goal_and_all_close_goals_keep_configured_range(tmp_path, opening, first, powered):
    bus = fake(opening)
    bus.reg['Torque_Enable']['gripper'] = int(powered)
    checks = []
    def check(actual):
        assert bus.writes == []
        doc = pick.sweep_doc(actual, actual['gripper'], 2018.)
        goals = doc['trajectory']['goals']['gripper']
        assert goals[0] == opening and goals[-1] == 2018.
        assert 3431. in goals if opening == 3501. else True
        checks.append(actual)
        return certificate(actual)
    code, log = go(tmp_path, bus, level='model',
        cert=certificate(dict(ACTUAL, gripper=opening, wrist_flex=2837.)), start_recheck=check)
    assert code == 0 and log['result'] == 'GRIP_CONTACT_HOLDING'
    assert len(checks) == 1 and checks[0]['gripper'] == opening
    assert sent(bus)[0] == first and all(2001 <= v <= 3431 for v in sent(bus))
    assert all(j == 'gripper' for _, j, _ in bus.writes)
    assert log['certificate_binding']['matched'] and log['certificate_binding']['rechecked']
    assert log['write_attempts'] == log['writes']


@pytest.mark.parametrize('opening', [3344., 3501.])
def test_failed_recheck_cannot_reach_first_goal_or_torque(tmp_path, opening):
    bus = fake(opening)
    code, log = go(tmp_path, bus, level='model',
        cert=certificate(dict(ACTUAL, gripper=opening, wrist_flex=2837.)),
        start_recheck=lambda actual: certificate(actual, 'CONTACT'))
    assert code == 2 and log['writes'] == {'Goal_Position': 0, 'Torque_Enable': 0}
    assert log['write_attempts'] == log['writes'] and not bus.writes


@pytest.mark.parametrize('opening', [3344., 3501.])
def test_stop_after_recheck_still_prevents_all_writes(tmp_path, opening):
    bus, stop = fake(opening), []
    def check(actual):
        stop.append(True)
        return certificate(actual)
    code, log = go(tmp_path, bus, level='model',
        cert=certificate(dict(ACTUAL, gripper=opening, wrist_flex=2837.)),
        start_recheck=check, stop_requested=lambda: bool(stop))
    assert not bus.writes and log['writes'] == {'Goal_Position': 0, 'Torque_Enable': 0}
    assert (log.get('stopped_by') or log.get('refused'))['gate'] == 'operator_stop'
