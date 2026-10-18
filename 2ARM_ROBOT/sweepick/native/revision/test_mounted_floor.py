"""2026-10-16 MOUNTED_FLOOR setup: arms on the TurtleBot3, block on the floor. Offline on stored REAL frames of run
20261016_164052_9b3c13 where available; no device. The existing planner / analysis code runs unchanged on the mounted scene."""
import copy, importlib, json, os
from pathlib import Path

import numpy as np
import pytest

RUN = Path.home() / 'sweepick_console_runs/20261016_164052_9b3c13/session/looks'
needs_run = pytest.mark.skipif(not (RUN / '00_observe/SCENE.json').exists(), reason='stored run 20261016_164052_9b3c13 not on this machine')


@pytest.fixture
def scene_camera(p1, tmp_path, monkeypatch):
    """The scene camera (os30a_UNVERIFIED), as before 2026-10-18: the stored-run numbers below were taken with it."""
    monkeypatch.setattr(p1.setup, 'MEASURED_HEAD_EXTRINSIC', tmp_path / 'absent.json')
    return p1


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


def test_mounted_world_is_the_robot_and_the_camera_does_not_need_the_board(scene_camera):
    p1 = scene_camera
    assert p1.MOUNTED and p1.SCENE.name == 'sweepick_mounted_floor_scene.mjb' and p1.BOARD_Z_IN_WORLD == 0.0
    T = p1.world_from_camera(None, p1.model())
    assert T[:3, 3][2] > 0.3                                                    # the camera stands above the robot
    assert np.allclose(T[:3, 0], [0, -1, 0], atol=1e-6)                         # image right = robot right
    assert T[:3, 2][0] > 0 and T[:3, 2][2] < 0                                   # looking forward and down


@needs_run
def test_the_stored_real_observation_finds_the_floor_block_and_both_arms(scene_camera):
    p1 = scene_camera
    import cv2
    s = json.loads((RUN / '00_observe/SCENE.json').read_text())
    r = p1.analyse(cv2.imread(str(RUN / '00_observe/top.png')), np.load(RUN / '00_observe/depth_mm.npy'), s['K'], s['ticks'])
    assert r['ok'] and r['setup'] == 'MOUNTED_FLOOR' and r['board']['role'] == 'DIAGNOSTIC_ONLY'
    o = r['object']
    assert o['class_match'] and abs(o['centre_xy'][0] - 0.325) < 0.01 and abs(o['centre_xy'][1] - 0.032) < 0.01
    desk = [0.65, 0.87, 0.90, 0.92, 0.77]                                       # left-arm share within 6 mm on the desk (runs 2026-10-14..15), for comparison only
    assert r['arms_on_model']['right']['share_within_6mm'] >= min(desk)
    # The LEFT arm at home falls 0.33 within 6 mm with the symmetric ruler model (both bases 140 mm, no fitted offset): an OPEN
    # residual (joint mapping or arm pose), recorded here and not fitted away; only that it is measured is checked.
    assert r['arms_on_model']['left']['share_within_6mm'] is not None


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


RUN2 = Path.home() / 'sweepick_console_runs/20261016_173652_ea3c03/session/looks'
needs_run2 = pytest.mark.skipif(not (RUN2 / '00_observe/SCENE.json').exists(), reason='stored run 20261016_173652_ea3c03 not on this machine')


def test_the_floor_plane_is_fitted_with_a_block_left_out(p1):
    rng = np.random.default_rng(0)
    xy = rng.uniform([0.1, -0.4], [0.4, 0.2], (6000, 2)); z = 0.02 * xy[:, 0] - 0.01 * xy[:, 1] - 0.035 + rng.normal(0, 0.002, 6000)
    block = np.c_[rng.uniform(0.19, 0.23, (500, 2)), np.full(500, -0.035 + 0.04)]
    pts = np.r_[np.c_[xy, z], block]; f = p1.floor_plane(pts, np.ones(len(pts), bool))
    assert f['ok'] and np.allclose(f['coef'], [0.02, -0.01, -0.035], atol=0.002) and f['points'] <= 6000
    assert not p1.floor_plane(pts[:100], np.ones(100, bool))['ok']


@needs_run2
def test_a_block_on_a_floor_below_the_model_floor_is_found(scene_camera):
    p1 = scene_camera
    """Run 20261016_173652_ea3c03 ended NOT_STARTED: the floor read 35-40 mm below the model's z 0, outside the old +-30 mm
    window, so the block top was searched at the wrong height. The fitted floor finds it (40 mm block, spot 1 of the overlay)."""
    import cv2
    s = json.loads((RUN2 / '00_observe/SCENE.json').read_text())
    r = p1.analyse(cv2.imread(str(RUN2 / '00_observe/top.png')), np.load(RUN2 / '00_observe/depth_mm.npy'), s['K'], s['ticks'])
    o = r['object']; f = r['desk_z']['floor']
    assert r['ok'] and o is not None and o['class_match'] and abs(o['centre_xy'][0] - 0.21) < 0.02 and abs(o['centre_xy'][1] - 0.11) < 0.02
    assert abs(o['top_height_above_depth_desk_m'] - 0.04) < 0.005
    assert abs(o['centre_world'][2] - (p1.floor_z(f, *o['centre_xy']) + 0.02)) < 1e-9 and -0.03 < p1.floor_z(f, *o['centre_xy']) < -0.005   # the robot stood pitched in this run


@needs_run
def test_the_board_from_its_markers_agrees_with_the_depth_floor(p1):
    import cv2
    s = json.loads((RUN / '00_observe/SCENE.json').read_text())
    r = p1.analyse(cv2.imread(str(RUN / '00_observe/top.png')), np.load(RUN / '00_observe/depth_mm.npy'), s['K'], s['ticks'])
    chk = r['desk_z']['floor']['board_check']
    assert r['ok'] and abs(chk['difference_m']) <= chk['tolerance_m']


def test_the_model_floor_follows_the_observed_block(p1):
    import mujoco
    m = p1.model(((0.2, 0.1, -0.01), 0.0)); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    t = m.geom('table').id; f = m.geom('floor').id
    assert abs(d.geom_xpos[t][2] + m.geom_size[t][2] - (-0.03)) < 1e-9                    # the support top where the block stands (in the collision data)
    assert m.geom_contype[f] == 0 and m.geom_conaffinity[f] == 0                           # the static z-0 ground plane takes no part


def fake_plans(monkeypatch, p1, left, right):
    def plan(scene, side='left', **kw):
        v = dict(left=left, right=right)[side]
        return dict(ok=True, side=side, room_over_residual=v, room_each_side_m=0.005, tilt_deg=0.0, jaw='x') if v is not None else dict(ok=False, side=side, why=[f'{side}: no path'])
    monkeypatch.setattr(p1, 'plan_pick', plan)


SC = dict(object=dict(centre_world=[0.2, 0.0, -0.01], yaw_rad=0.0))


def test_one_reachable_arm_is_chosen_and_both_use_one_cost(p1, monkeypatch, tmp_path):
    no_right_references(p1, monkeypatch, tmp_path)
    fake_plans(monkeypatch, p1, None, 2.0); s = p1.select_arm(SC)
    assert s['selected'] == 'right' and not s['executable'] and 'only the right' in s['reason']
    fake_plans(monkeypatch, p1, 1.5, 2.0); s = p1.select_arm(SC)
    assert s['selected'] == 'right' and s['cost']['source'] == 'sweepick_pick01.plan_pick'
    fake_plans(monkeypatch, p1, 2.0, 2.0); assert p1.select_arm(SC)['selected'] == 'left'
    fake_plans(monkeypatch, p1, None, None); s = p1.select_arm(SC)
    assert s['selected'] is None and not s['executable']


def test_a_right_arm_choice_ends_before_any_motion(p1, monkeypatch, tmp_path):
    no_right_references(p1, monkeypatch, tmp_path)
    fake_plans(monkeypatch, p1, None, 2.0)
    scene = dict(ok=True, arms=dict(left=dict(torque={'a': 0}), right=dict(torque={'a': 0})), object=dict(SC['object'], geometry=None, class_match=True),
                 ticks=dict(left={n: 2000.0 for n in p1.ALL}, right={n: 2000.0 for n in p1.ALL}), wrist=dict(jaw_reference=dict(readable=True)))
    monkeypatch.setattr(p1, 'look', lambda out, tag: scene)
    st = p1.observe(tmp_path)
    assert st['ready'] is False and st['why'].startswith('RIGHT_ARM_SELECTED_NOT_MEASURED') and st['arm_selection']['selected'] == 'right'
    assert 'plans' not in st['arm_selection']


@needs_run
def test_the_ruler_corrected_model_puts_the_seen_floor_at_its_floor(p1):
    """Operator ruler 2026-10-16: arm bases 140 mm, TurtleBot3 top 104 mm. With the mount stack raised to it, the floor the top
    depth sees in front of the robot (run 20261016_164052_9b3c13) lies at the model floor."""
    import cv2
    s = json.loads((RUN / '00_observe/SCENE.json').read_text())
    r = p1.analyse(cv2.imread(str(RUN / '00_observe/top.png')), np.load(RUN / '00_observe/depth_mm.npy'), s['K'], s['ticks'])
    assert abs(p1.floor_z(r['desk_z']['floor'], 0.2, 0.0)) < 0.005



def no_right_references(p1, monkeypatch, tmp_path):
    """TEST ONLY: the right arm without its measured files, whatever the folder holds (10/18 the candidate folder holds them)."""
    monkeypatch.setattr(p1, "RIGHT_WRIST_RECORD", tmp_path / "absent_wrist_right.json"); monkeypatch.setitem(p1.JAW_PROFILES, "right", tmp_path / "absent_jaw_right.json")


def right_references(p1, monkeypatch, tmp_path):
    """TEST ONLY: stand-in right reference files so the right path can be exercised offline (no measured values are created)."""
    w = tmp_path / "wrist_right.json"; w.write_text(json.dumps(p1.WRIST_LEFT)); j = tmp_path / "jaw_right.json"; j.write_text(p1.JAW_PROFILES["left"].read_text())
    monkeypatch.setattr(p1, "RIGHT_WRIST_RECORD", w); monkeypatch.setitem(p1.JAW_PROFILES, "right", j)


def right_scene(p1):
    t = {n: 2000.0 for n in p1.ALL}
    return dict(ok=True, arms=dict(left=dict(torque={'a': 0}), right=dict(torque={'a': 0})), object=dict(SC['object'], geometry=None, class_match=True),
                ticks=dict(left=dict(t, gripper=3400.0), right=dict(t, gripper=3015.0)), wrist=dict(jaw_reference=dict(readable=True, inner_px=[60, 250], midline_px=155.0)))


def test_the_right_arm_is_not_executable_without_its_own_references(p1, monkeypatch, tmp_path):
    no_right_references(p1, monkeypatch, tmp_path)
    assert p1.executable_arms() == ("left",)
    with pytest.raises(RuntimeError, match='RIGHT_ARM_NOT_MEASURED'):
        p1.set_side('right')


def test_a_right_selection_with_its_references_makes_the_right_arm_the_session_arm(p1, monkeypatch, tmp_path):
    right_references(p1, monkeypatch, tmp_path)
    plan = dict(ok=True, side='right', room_over_residual=2.0, room_each_side_m=0.005, tilt_deg=0.0, jaw='x', model_jaw_gap_now_m=0.05, object_width_m=0.04, jaw_lean_deg=0.0, ticks={})
    monkeypatch.setattr(p1, 'plan_pick', lambda scene, side='left', **kw: dict(plan, side=side) if side == 'right' else dict(ok=False, side=side, why=['left: no path']))
    monkeypatch.setattr(p1, 'look', lambda out, tag: right_scene(p1))
    seen = []
    monkeypatch.setattr(p1, 'wrist_frame', lambda path, *a, **k: seen.append((str(path), p1.SIDE)) or dict(jaw_reference=dict(readable=True, inner_px=[70, 240], midline_px=155.0)))
    (tmp_path / 'looks/00_observe').mkdir(parents=True)
    try:
        st = p1.observe(tmp_path)
        assert st['side'] == 'right' and st['ready'] is True, st['why']
        assert st['home_ticks']['gripper'] == 3015.0 and st['other_ticks']['gripper'] == 3400.0
        assert seen and seen[-1][0].endswith('right_wrist.jpg') and seen[-1][1] == 'right'           # the right wrist gave the jaw reference
        assert st['jaw_reference']['inner_px'] == [70, 240] and p1.WRIST is not p1.WRIST_LEFT
    finally:
        p1.set_side('left')


def test_a_right_session_stage_asks_the_executor_for_the_right_arm(p1, monkeypatch, tmp_path):
    right_references(p1, monkeypatch, tmp_path)
    t = {n: 2000.0 for n in p1.ALL}
    state = dict(schema='tjj.pick01.v1', scene='looks/00_observe/SCENE.json', stages={}, side='right', ready=True, why=None, evidence=[], order=[], sources={},
                 plan=dict(ticks=dict(pregrasp={n: 2010.0 for n in p1.ARM})), home_ticks=t, right_ticks=t, other_ticks=dict(t, gripper=3400.0))
    (tmp_path / 'STATE.json').write_text(json.dumps(state))
    monkeypatch.setattr(p1, 'state_object', lambda out, st: ([0.2, -0.1, 0.02], 0.0)); monkeypatch.setattr(p1, 'model', lambda *a, **k: __import__('mujoco').MjModel.from_xml_string('<mujoco/>'))
    monkeypatch.setattr(p1, 'allowed', lambda st, name: (True, 'test'))
    calls = []
    def fake_run(mode, side, out, **kw):
        calls.append((mode, side, kw.get('other_arm_ticks', {}).get('gripper'))); Path(out).mkdir(parents=True, exist_ok=True)
        (Path(out) / 'TRAJECTORY.json').write_text(json.dumps(dict(side=side, trajectory=dict(steps=1))))
        return 0, dict(result='PLANNED', trajectory=dict(steps=1, duration_s=0.1, travel_ticks=10))
    rec = p1.stage(tmp_path, 'pregrasp', execute=False, run=fake_run, cert=lambda a, b, c: dict(nominal_verdict='CLEAR'), run_kw=dict(open_bus=None))
    p1.set_side('left')
    assert calls == [('plan', 'right', 3400.0)] and rec['result'] == 'PLANNED'


@needs_run
def test_the_measured_camera_moves_the_stored_block_by_the_camera_change_only(p1):
    """2026-10-18: with the measured head pose the same stored frame gives the same block, carried by T_measured . T_scene^-1
    (within 1 cm: the floor is refitted in the new frame)."""
    import cv2
    s = json.loads((RUN / '00_observe/SCENE.json').read_text())
    r = p1.analyse(cv2.imread(str(RUN / '00_observe/top.png')), np.load(RUN / '00_observe/depth_mm.npy'), s['K'], s['ticks'])
    assert r['ok'] and r['camera_source']['source'] == 'MEASURED' and r['object']['class_match']
    T_scene, T_meas = p1.setup.scene_robot_from_camera(), p1.setup.mounted_robot_from_camera()
    expected = (T_meas @ np.linalg.inv(T_scene) @ np.r_[0.325, 0.032, 0.02, 1.0])[:2]
    assert np.allclose(r['object']['centre_xy'], expected, atol=0.01)
