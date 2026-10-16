"""OFFLINE: where can the LEFT arm, mounted on the TurtleBot3 (A 'mobile' scene, USER_DIMENSIONS_UNVERIFIED), pick the A block
(a) from the floor and (b) from a shelf whose top is at the TurtleBot's own top height? The EXISTING SIM teacher planner
(the one plan_pick uses: IK + its path collision check, its own config) plans pregrasp -> grasp -> lift from the arm's
recorded home pose; nothing new is solved here. The support is the planner's required 'table' geom: the floor surface for
(a), a shelf box for (b). A grid of block centres is tried; a cell is feasible when all three moves plan without a via point.
No device; the shelf placement relative to the robot is a parameter, not a measurement."""
import json, sys
from pathlib import Path
import numpy as np
import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mobile_model as mm, integration_scenes
import mounted_poses as mp
sys.path.insert(0, str(mm.RT / 'source/production_source_g000'))
from sim_data_factory.teacher import HandoverTeacher, TeacherFailure, UP

HALF = 0.02


def scene(support, block_xy, shelf, yaw=0.0):
    spec = integration_scenes.build_scene('mobile', grippers='both')
    box = mujoco.mjtGeom.mjGEOM_BOX
    if support == 'floor':
        top = 0.0
        spec.worldbody.add_geom(name='table', type=box, pos=[block_xy[0], block_xy[1], -0.005], size=[0.15, 0.15, 0.005], rgba=[.5, .5, .5, 1])
    else:
        top = shelf['top_z_m']
        cx, cy, hx, hy = shelf['centre_xy'][0], shelf['centre_xy'][1], shelf['half_size_xy'][0], shelf['half_size_xy'][1]
        spec.worldbody.add_geom(name='table', type=box, pos=[cx, cy, top / 2], size=[hx, hy, top / 2], rgba=[.6, .45, .3, 1])
    b = spec.worldbody.add_body(name='red_block', pos=[block_xy[0], block_xy[1], top + HALF], quat=[np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]); b.add_freejoint()
    b.add_geom(name='red_block_geom', type=box, size=[HALF] * 3, rgba=[.8, .1, .1, 1])
    return spec.compile(), top


def start_pose(home):
    """As plan_pick does: the recorded home kept for FK, a command seed inside each servo's configured range
    (sweepick_pick01.home_command_ticks), and the grippers in the planner's gear units."""
    sys.path.insert(0, str(mp.NATIVE)); import sweepick_pick01 as p1
    L, R = home['left'], home['right']
    Lc, _ = p1.home_command_ticks(L, mp.MP, 'left'); Rc, _ = p1.home_command_ticks(R, mp.MP, 'right')
    q12 = mp.rc.ticks_to_q12(mp.MP, dict(left=dict(L, **Lc), right=dict(R, **Rc)))
    frac = {s: (c['gripper'] - float(mp.MP[s]['gripper']['lo'])) / (float(mp.MP[s]['gripper']['hi']) - float(mp.MP[s]['gripper']['lo'])) for s, c in (('left', Lc), ('right', Rc))}
    return q12, frac


def try_pick(m, home, block_xy, top, side='left', yaw=0.0):
    q12, frac = start_pose(home)
    t = HandoverTeacher(m); cfg = t.cfg; d = mujoco.MjData(m)
    mp.kin.set_q12(m, d, q12); mujoco.mj_forward(m, d)
    start = d.qpos[t.plan.adr].copy()
    start[5] = t.g_a[0] + t.g_b[0] * frac['left']; start[11] = t.g_a[1] + t.g_b[1] * frac['right']
    t.cmd = start.copy()
    centre = np.array([block_xy[0], block_xy[1], top + HALF])
    t.plan.pose(d.qpos, t.cmd); rl = t.plan.radial(side, centre)
    reasons = []
    for tilt in np.deg2rad(cfg.pick_tilt_deg):
        ap = np.cos(tilt) * -UP + np.sin(tilt) * rl
        for jname, jaw in (('x', np.array([np.cos(yaw), np.sin(yaw), 0])), ('y', np.array([-np.sin(yaw), np.cos(yaw), 0]))):
            try:
                def solve(s, pos, label, allow=()):
                    dd = mujoco.MjData(m); dd.qpos[:] = d.qpos; dd.qpos[t.plan.adr] = s; t.cmd = np.array(s, float).copy()
                    w, info = t.plan_move(dd, side, pos, ap, [jaw], allow=allow, label=label)
                    if len(w) != 1:
                        raise TeacherFailure(f'{label}: needs a via point')
                    return w[0]
                pre = solve(start, centre + [0, 0, cfg.pregrasp_height_m], 'pregrasp')
                des = solve(pre, centre + [0, 0, cfg.left_grasp_offset_m], 'grasp', allow=(side,))
                lift = solve(des, centre + [0, 0, cfg.left_grasp_offset_m + 0.06], 'lift', allow=(side,))
                return dict(ok=True, tilt_deg=float(np.degrees(tilt)), jaw=jname)
            except TeacherFailure as e:
                reasons.append(f'tilt {np.degrees(tilt):.0f} jaw {jname}: {str(e)[:90]}')
    return dict(ok=False, why=reasons[:3])


def run(home, grid_x, grid_y, shelf):
    out = dict(floor=[], shelf=[])
    for support in (('shelf',) if shelf else ('floor',)):
        for x in grid_x:
            for y in grid_y:
                if support == 'shelf':
                    # the block on the shelf: the shelf front edge stands a gap away from the robot side, the block inside it
                    if not (shelf['centre_xy'][0] - shelf['half_size_xy'][0] + HALF <= x <= shelf['centre_xy'][0] + shelf['half_size_xy'][0] - HALF
                            and shelf['centre_xy'][1] - shelf['half_size_xy'][1] + HALF <= y <= shelf['centre_xy'][1] + shelf['half_size_xy'][1] - HALF):
                        continue
                try:
                    m, top = scene(support, (x, y), shelf)
                    r = try_pick(m, home, (x, y), top)
                except Exception as e:
                    r = dict(ok=False, why=[f'{type(e).__name__}: {str(e)[:120]}'])
                out[support].append(dict(x=round(float(x), 3), y=round(float(y), 3), **r))
    return out
