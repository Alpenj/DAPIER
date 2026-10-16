"""OFFLINE status ladder for a few mounted floor candidate cells (LEFT arm; A's plan_pick is left-only):
  TEACHER_IK_ONLY        the work6 grid (teacher plan_move only; older scene without the left-mount fit, desk home start)
  A_PLAN_PICK_PASS       sweepick_pick01.plan_pick() in full (pad room, re-centring, tilt/jaw choice) on the mounted floor scene,
                         from the REAL mounted start (run 20261016_164052_9b3c13 observe ticks)
  NATIVE_STAGE_CERTIFIED sweepick_pick01.stage(execute=False) for pregrasp, descend_a, descend, close (closing sweep), lift:
                         the executor's planning read on a FAKE bus holding the previous target, the trajectory, and the
                         real sweepick_collision_cert.py on the saved scene. Observation gates are SYNTHETIC here (marked).
  REAL_VERIFIED          never set offline.
Plus model camera checks: block inside the top camera image at the observe pose, and inside the left wrist camera (model
camera, fovy 75) at the planned pre-grasp. No device; nothing is written to a motor."""
import copy, json, os, sys, tempfile, time
from pathlib import Path
import numpy as np

os.environ.setdefault('SWEEPICK_SETUP', 'MOUNTED_FLOOR')
H = Path.home(); NATIVE = H / 'sweepick_261016_floor_native/revision'
sys.path.insert(0, str(NATIVE))
import mujoco, cv2
import sweepick_pick01 as p1
from test_sweepick_pick02_actual import Arm
from test_sweepick_move_a import Clock

RUN = H / 'sweepick_console_runs/20261016_164052_9b3c13/session/looks/00_observe'
STAGES = ('pregrasp', 'descend_a', 'descend', 'close', 'lift')


def base_scene():
    s = json.loads((RUN / 'SCENE.json').read_text())
    r = p1.analyse(cv2.imread(str(RUN / 'top.png')), np.load(RUN / 'depth_mm.npy'), s['K'], s['ticks'])
    r['ticks'] = s['ticks']; r['K'] = s['K']; r['image_size'] = list(cv2.imread(str(RUN / 'top.png')).shape[1::-1])
    return r


def camera_checks(scene, plan):
    m = p1.model(); d = mujoco.MjData(m); c = np.array(scene['object']['centre_world'])
    p1.sweepick_kin.set_q12(m, d, np.array(scene['q12'])); mujoco.mj_forward(m, d)
    T = np.array(scene['T_world_camera']); pc = np.linalg.inv(T) @ np.r_[c, 1.0]
    K = np.array(scene['K']).reshape(3, 3); scale = scene['image_size'][0] / (2 * K[0, 2]); u, v = (K[:2, :2] @ (pc[:2] / pc[2]) + K[:2, 2]) * scale
    top = dict(in_image=bool(pc[2] > 0 and 0 <= u < scene['image_size'][0] and 0 <= v < scene['image_size'][1]), uv=[round(float(u)), round(float(v))])
    mp = p1.mapping(); q = p1.rc.ticks_to_q12(mp, dict(left=dict(scene['ticks']['left'], **plan['ticks']['pregrasp']), right=scene['ticks']['right']))
    p1.sweepick_kin.set_q12(m, d, q); mujoco.mj_forward(m, d)
    cam = m.camera('left_wrist_cam').id; R = d.cam_xmat[cam].reshape(3, 3); loc = R.T @ (c - d.cam_xpos[cam])     # MuJoCo camera: looks along -z
    fov = np.radians(m.cam_fovy[cam]); ang_v = np.degrees(np.arctan2(abs(loc[1]), -loc[2])) if loc[2] < 0 else 180.0
    ang_h = np.degrees(np.arctan2(abs(loc[0]), -loc[2])) if loc[2] < 0 else 180.0
    half_v = np.degrees(fov / 2); half_h = np.degrees(np.arctan(np.tan(fov / 2) * 320 / 240))
    wrist = dict(in_view=bool(loc[2] < 0 and ang_v <= half_v and ang_h <= half_h), off_axis_deg=[round(float(ang_h), 1), round(float(ang_v), 1)],
                 half_fov_deg=[round(float(half_h), 1), round(float(half_v), 1)], role='MODEL camera (left_wrist_cam, fovy 75, 320x240 aspect assumed); orientation not verified against the real wrist image')
    return dict(top=top, wrist_at_pregrasp=wrist)


def certify_stages(scene, plan):
    out = Path(tempfile.mkdtemp(prefix='mounted_cand_')); (out / 'looks/00').mkdir(parents=True)
    (out / 'looks/00/SCENE.json').write_text(json.dumps(scene, default=str))
    home = {n: float(v) for n, v in scene['ticks']['left'].items()}; right = {n: float(v) for n, v in scene['ticks']['right'].items()}
    st = dict(ready=True, why=None, scene='looks/00/SCENE.json', stages={}, evidence=[], order=[], plan=plan, home_ticks=home, right_ticks=right,
              jaw_reference=dict(readable=True, inner_px=[51, 257], midline_px=154.0, gap_px=206, gripper_tick=home['gripper']), height=None)
    (out / 'STATE.json').write_text(json.dumps(st, default=str))
    fake = Arm(dict(home)); c = Clock(); mp = p1.mapping()
    import hashlib; kw = dict(open_bus=lambda port: fake, owner=lambda p: '', mapping=mp, mapping_sha=hashlib.sha256(p1.CAND.read_bytes()).hexdigest(), sleep=c.sleep, clock=c.now)
    rows = []
    for name in STAGES:
        state = json.loads((out / 'STATE.json').read_text())
        if name in p1.GATE:
            state['evidence'].append(dict(what=p1.GATE[name], ok=True, synthetic='OFFLINE_CERTIFICATION_GATE: no real observation')); (out / 'STATE.json').write_text(json.dumps(state, default=str))
        t0 = time.time()
        rec = p1.stage(out, name, execute=False, run_kw=kw, torque_now=lambda: dict(fake.reg['Torque_Enable']))
        col = (rec.get('collision') or [{}])[-1]
        rows.append(dict(stage=name, result=rec.get('result'), controller_result=rec.get('controller_result'), why=rec.get('why'), verdict=col.get('verdict'),
                         first_contact=col.get('first_contact'), steps=col.get('steps'), seconds=round(time.time() - t0, 1)))
        if rec.get('result') != 'PLANNED':
            break
        state = json.loads((out / 'STATE.json').read_text()); rq = p1.stage_request(state, name, mp)
        target = rq.get('target') or {}
        for n, v in target.items():
            fake.reg['Present_Position'][n] = float(v); fake.reg['Goal_Position'][n] = float(v)
        state['stages'][name] = dict(stage=name, controller_result='GRIP_CONTACT_HOLDING' if name == 'close' else 'HOLDING', synthetic='OFFLINE: target / contact assumed', end=dict(ticks={n: float(fake.reg['Present_Position'][n]) for n in p1.ALL}))
        state['order'].append(name); state['height'] = {'pregrasp': 'pregrasp', 'descend_a': 'descend_a', 'descend': 'descend', 'lift': 'lift'}.get(name, state.get('height'))
        (out / 'STATE.json').write_text(json.dumps(state, default=str))
    return rows, str(out)


def evaluate(base, x, y, yaw_deg, stages=True):
    sc = copy.deepcopy(base); sc['object']['centre_world'] = [x, y, p1.BOARD_Z_IN_WORLD + 0.02]; sc['object']['yaw_rad'] = float(np.radians(yaw_deg))
    plan = p1.plan_pick(copy.deepcopy(sc))
    row = dict(x=x, y=y, yaw_deg=yaw_deg, A_PLAN_PICK_PASS=bool(plan['ok']), plan_why=None if plan['ok'] else (plan.get('why') or [])[:3])
    if plan['ok']:
        row.update(tilt_deg=plan['tilt_deg'], jaw=plan['jaw'], room_each_side_mm=round(plan['room_each_side_m'] * 1000, 2), pad_gaps_mm=[round(g, 2) for g in plan.get('pad_gaps_mm') or []],
                   cameras=camera_checks(sc, plan))
        if stages:
            rows, folder = certify_stages(sc, plan)
            row.update(stages=rows, NATIVE_STAGE_CERTIFIED=bool(len(rows) == len(STAGES) and all(r['result'] == 'PLANNED' for r in rows)), stage_folder=folder)
    row['REAL_VERIFIED'] = False
    return row


if __name__ == '__main__':
    base = base_scene(); grid = json.loads((Path(__file__).parent / 'PICK_GRID_W6_v2.json').read_text())
    teacher = {(c['x'], c['y'], c['yaw_deg']): c['ok'] for c in grid['cells'] if c['support'] == 'floor' and c['side'] == 'left'}
    centres = [(0.10, 0.30), (0.20, 0.05), (0.15, 0.20)]
    out = dict(schema='sweepick.mounted-candidates.v1', setup=p1.setup.SETUP, scene_sha256=json.loads((NATIVE / 'sweepick_mounted_floor_scene.json').read_text())['sha256'],
               start='REAL mounted ticks of run 20261016_164052_9b3c13 observe (left and right)', arm='left (A plan_pick is left-only)', candidates=[])
    for cx, cy in centres:
        cand = dict(centre=[cx, cy], rows=[])
        for yaw in (0, 45):
            r = evaluate(base, cx, cy, yaw); r['TEACHER_IK_ONLY'] = teacher.get((cx, cy, yaw)); cand['rows'].append(r)
            print(cx, cy, yaw, 'teacher', r['TEACHER_IK_ONLY'], 'plan', r['A_PLAN_PICK_PASS'], 'cert', r.get('NATIVE_STAGE_CERTIFIED'), [(s['stage'], s['result'], s['verdict']) for s in r.get('stages', [])], r.get('cameras', {}).get('wrist_at_pregrasp', {}).get('in_view'), flush=True)
        nb = []
        for dx in (-0.05, 0.0, 0.05):
            for dy in (-0.05, 0.0, 0.05):
                if dx == dy == 0.0:
                    continue
                r = evaluate(base, round(cx + dx, 3), round(cy + dy, 3), 0, stages=False); nb.append(dict(x=r['x'], y=r['y'], A_PLAN_PICK_PASS=r['A_PLAN_PICK_PASS'], TEACHER_IK_ONLY=teacher.get((r['x'], r['y'], 0))))
        cand['neighbours_yaw0'] = nb; out['candidates'].append(cand)
        print('  neighbours plan pass', sum(n['A_PLAN_PICK_PASS'] for n in nb), '/ 8', flush=True)
    Path(sys.argv[1] if len(sys.argv) > 1 else 'MOUNTED_CANDIDATES.json').write_text(json.dumps(out, indent=1, default=str) + '\n')
