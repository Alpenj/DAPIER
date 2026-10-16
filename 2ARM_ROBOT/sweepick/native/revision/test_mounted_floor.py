"""2026-10-16 MOUNTED_FLOOR setup: arms on the TurtleBot3, block on the floor. Offline on stored REAL frames of run
20261016_164052_9b3c13 where available; no device. The existing planner / analysis code runs unchanged on the mounted scene."""
import copy, importlib, json, os
from pathlib import Path

import numpy as np
import pytest

RUN = Path.home() / 'sweepick_console_runs/20261016_164052_9b3c13/session/looks'
needs_run = pytest.mark.skipif(not (RUN / '00_observe/SCENE.json').exists(), reason='stored run 20261016_164052_9b3c13 not on this machine')


@pytest.fixture(scope='module')
def p1():
    old = os.environ.get('SWEEPICK_SETUP'); os.environ['SWEEPICK_SETUP'] = 'MOUNTED_FLOOR'
    import sweepick_setup, sweepick_pick01
    importlib.reload(sweepick_setup); mod = importlib.reload(sweepick_pick01)
    yield mod
    if old is None:
        os.environ.pop('SWEEPICK_SETUP', None)
    else:
        os.environ['SWEEPICK_SETUP'] = old
    importlib.reload(sweepick_setup); importlib.reload(sweepick_pick01)


def test_the_setup_choice_is_explicit_and_bound(tmp_path, monkeypatch):
    import sweepick_setup as s
    assert json.loads(s.CHOICE_FILE.read_text())['setup'] in (s.DESK, s.MOUNTED_FLOOR)
    monkeypatch.setenv('SWEEPICK_SETUP', 'SOMETHING_ELSE')
    with pytest.raises(RuntimeError):
        s.current()


def test_mounted_world_is_the_robot_and_the_camera_does_not_need_the_board(p1):
    assert p1.MOUNTED and p1.SCENE.name == 'sweepick_mounted_floor_scene.mjb' and p1.BOARD_Z_IN_WORLD == 0.0
    T = p1.world_from_camera(None, p1.model())
    assert T[:3, 3][2] > 0.3                                                    # the camera stands above the robot
    assert np.allclose(T[:3, 0], [0, -1, 0], atol=1e-6)                         # image right = robot right
    assert T[:3, 2][0] > 0 and T[:3, 2][2] < 0                                   # looking forward and down


@needs_run
def test_the_stored_real_observation_finds_the_floor_block_and_both_arms(p1):
    import cv2
    s = json.loads((RUN / '00_observe/SCENE.json').read_text())
    r = p1.analyse(cv2.imread(str(RUN / '00_observe/top.png')), np.load(RUN / '00_observe/depth_mm.npy'), s['K'], s['ticks'])
    assert r['ok'] and r['setup'] == 'MOUNTED_FLOOR' and r['board']['role'] == 'DIAGNOSTIC_ONLY'
    o = r['object']
    assert o['class_match'] and abs(o['centre_xy'][0] - 0.325) < 0.01 and abs(o['centre_xy'][1] - 0.032) < 0.01
    desk = [0.65, 0.87, 0.90, 0.92, 0.77]                                       # left-arm share within 6 mm on the desk (runs 2026-10-14..15), for comparison only
    assert r['arms_on_model']['left']['share_within_6mm'] >= min(desk) and r['arms_on_model']['right']['share_within_6mm'] >= min(desk)


@needs_run
def test_the_planner_refuses_the_observed_out_of_reach_block_and_plans_one_inside_the_reach(p1):
    import cv2
    s = json.loads((RUN / '00_observe/SCENE.json').read_text())
    r = p1.analyse(cv2.imread(str(RUN / '00_observe/top.png')), np.load(RUN / '00_observe/depth_mm.npy'), s['K'], s['ticks']); r['ticks'] = s['ticks']
    assert p1.plan_pick(copy.deepcopy(r))['ok'] is False
    inside = copy.deepcopy(r); inside['object']['centre_world'][:2] = [0.20, 0.05]
    pl = p1.plan_pick(inside)
    assert pl['ok'] and pl['room_each_side_m'] > 0


@needs_run
def test_the_area_check_works_in_the_robot_frame(p1):
    import cv2
    s = json.loads((RUN / '00_observe/SCENE.json').read_text())
    r = p1.analyse(cv2.imread(str(RUN / '00_observe/top.png')), np.load(RUN / '00_observe/depth_mm.npy'), s['K'], s['ticks'])
    top = dict(depth_mm=np.load(RUN / '00_observe/depth_mm.npy'), K=s['K'])
    from sweepick_local_loop import real_area
    F = np.diag([1.0, -1.0, -1.0, 1.0]); T_cb = np.linalg.inv(F @ np.array(r['T_world_camera']))
    at_block = real_area(top['depth_mm'], top['K'], T_cb, 0.0, tuple(r['object']['centre_xy']), 0.03)
    empty = real_area(top['depth_mm'], top['K'], T_cb, 0.0, (0.25, 0.30), 0.03)
    assert at_block['state'] == 'OCCUPIED' and empty['state'] in ('CLEAR', 'UNKNOWN (not fully observed)')


def test_the_full_chain_is_refused_in_the_mounted_setup(p1, tmp_path):
    with pytest.raises(RuntimeError, match='MOUNTED_FLOOR'):
        p1.run_session(tmp_path / 's', execute=False, full_chain=True)
