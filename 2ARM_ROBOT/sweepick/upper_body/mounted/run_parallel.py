"""work6: the mounted pick feasibility grid in parallel (one process per cell). Same planner and scene as mounted_pick.py."""
import json, os, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import mounted_pick as mpk, mounted_poses as mp

STATE = Path('/home/robot/sweepick_console_runs/20261016_121735_b9241e/session/STATE.json')
st = json.loads(STATE.read_text())
HOME = dict(left={j: float(st['home_ticks'][j]) for j in mp.ARM}, right={j: float(st['right_ticks'][j]) for j in mp.ARM})
# Shelf placement from the A 'mobile' model (MODEL_FOOTPRINT.json): the mount rails reach y = +-0.200 m, the front camera
# x = +0.085 m. Each shelf stands 2 cm outside them (a scenario gap, not a measurement); top = base top 0.104 m.
RAIL_Y, FRONT_X, GAP, TOP = 0.200, 0.085, 0.02, 0.104
SHELVES = dict(LEFT_SIDE=dict(top_z_m=TOP, centre_xy=[0.0, RAIL_Y + GAP + 0.15], half_size_xy=[0.30, 0.15]),
               RIGHT_SIDE=dict(top_z_m=TOP, centre_xy=[0.0, -(RAIL_Y + GAP + 0.15)], half_size_xy=[0.30, 0.15]),
               FRONT=dict(top_z_m=TOP, centre_xy=[FRONT_X + GAP + 0.15, 0.0], half_size_xy=[0.15, 0.40]))
SUPPORTS = tuple((__import__('os').environ.get('SUPPORTS') or 'floor,shelf').split(','))


def inside(sh, x, y):
    (cx, cy), (hx, hy) = sh['centre_xy'], sh['half_size_xy']
    return cx - hx + mpk.HALF <= x <= cx + hx - mpk.HALF and cy - hy + mpk.HALF <= y <= cy + hy - mpk.HALF


def cell(args):
    support, shelf_name, side, yaw_deg, x, y = args
    try:
        m, top = mpk.scene(support, (x, y), SHELVES.get(shelf_name), np.deg2rad(yaw_deg))
        r = mpk.try_pick(m, HOME, (x, y), top, side=side, yaw=np.deg2rad(yaw_deg))
    except Exception as e:
        r = dict(ok=False, why=[f'{type(e).__name__}: {str(e)[:120]}'])
    return dict(support=support, shelf=shelf_name, side=side, yaw_deg=yaw_deg, x=round(x, 3), y=round(y, 3), **r)


if __name__ == '__main__':
    xs = np.round(np.arange(-0.20, 0.451, 0.05), 3); ys = np.round(np.arange(-0.50, 0.501, 0.05), 3)
    jobs = []
    for side in ('left', 'right'):
        for yaw in (0, 45):
            if 'floor' in SUPPORTS:
                jobs += [('floor', None, side, yaw, float(x), float(y)) for x in xs for y in ys]
            for name, sh in (SHELVES.items() if 'shelf' in SUPPORTS else ()):
                jobs += [('shelf', name, side, yaw, float(x), float(y)) for x in xs for y in ys if inside(sh, x, y)]
    t0 = time.time(); print('cells', len(jobs), flush=True)
    out = []
    with Pool(int(sys.argv[2]) if len(sys.argv) > 2 else 30) as pool:
        for i, r in enumerate(pool.imap_unordered(cell, jobs, chunksize=4)):
            out.append(r)
            if i % 500 == 0:
                print(i, round(time.time() - t0, 1), 's', flush=True)
    res = dict(home_source=str(STATE), shelves=SHELVES, grid=dict(x=xs.tolist(), y=ys.tolist()), elapsed_s=round(time.time() - t0, 1), cells=out)
    Path(sys.argv[1]).write_text(json.dumps(res) + '\n'); print('done', len(out), res['elapsed_s'], flush=True)
