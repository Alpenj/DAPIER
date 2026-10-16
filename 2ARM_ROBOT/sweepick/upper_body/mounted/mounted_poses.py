"""OFFLINE: put arm poses recorded on the REAL arms (desk runs) into the existing A 'mobile' scene (TurtleBot3 Waffle Pi +
aluminium mount + two SO-101 + camera stand, USER_DIMENSIONS_UNVERIFIED) and list every contact that is not a wheel on the
floor. A contact is a refusal (the existing rule: no new contact at all); there is no clearance number here.

Poses: each run's observed home ticks and the reached ticks of its left stages, the right arm at its observed home.
Ticks -> model angles through the existing calibrated mapping (sweepick_real_command.load_mapping). No device is opened."""
import json, sys
from pathlib import Path
import numpy as np
import mujoco

H = Path.home()
NATIVE = H / 'sweepick_261013_receive_alignment_restore/revision'
sys.path.insert(0, str(Path(__file__).resolve().parent)); sys.path.insert(1, str(NATIVE))
import mobile_model as mm
import sweepick_real_command as rc, sweepick_kin as kin
IN = H / 'sweepick_261007_commission/inputs'
MP = rc.load_mapping(IN / 'CANDIDATE_Q12_CONFIG.json', H / '.config/dapier/lerobot-calibration', IN / 'ASSEMBLED_ZERO_AND_PGRIPPER.json')
ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')


def contacts(m, d, q12):
    kin.set_q12(m, d, q12); mujoco.mj_forward(m, d)
    nm = lambda g: m.body(m.geom_bodyid[g]).name + ':' + (m.geom(g).name or f'g{g}')
    rows = []
    for c in d.contact[:d.ncon]:
        a, b = nm(c.geom1), nm(c.geom2)
        if {a.split(':')[0], b.split(':')[0]} & {'world'} and ('wheel' in a or 'wheel' in b):
            continue                                                             # the wheels stand on the floor
        rows.append(dict(a=a, b=b, depth_mm=round(-float(c.dist) * 1000, 2)))
    return rows


def poses(states):
    out = []
    for path, st in states:
        home = st.get('home_ticks'); right = st.get('right_ticks')
        if not home or not right:
            continue
        run = path.parent.parent.name if path.parent.name == 'session' else path.parent.name
        cand = [('home', home)] + [(k, {j: v['tick'] for j, v in (s.get('reached') or {}).items()}) for k, s in (st.get('stages') or {}).items() if s.get('reached')]
        for name, left in cand:
            left = {j: float(left.get(j, home.get(j))) for j in ARM}
            out.append(dict(run=run, pose=name, left=left, right={j: float(right[j]) for j in ARM}))
    return out


def evaluate(states):
    m = mm.build(); d = mujoco.MjData(m); rows = []
    for p in poses(states):
        q12 = rc.ticks_to_q12(MP, dict(left=p['left'], right=p['right']))
        c = contacts(m, d, q12)
        rows.append(dict(p, contacts=c, verdict='CLEAR' if not c else 'CONTACT'))
    return dict(model=m.names[:0].decode() or 'mobile_aluminum_USER_DIMENSIONS_UNVERIFIED', rows=rows)


if __name__ == '__main__':
    import glob
    paths = sorted(set(glob.glob(str(H / 'sweepick_console_runs/*/session/STATE.json'))))
    states = []
    for p in paths:
        try:
            states.append((Path(p), json.loads(Path(p).read_text())))
        except ValueError:
            pass
    res = evaluate(states)
    Path(sys.argv[1] if len(sys.argv) > 1 else 'MOUNTED_POSES.json').write_text(json.dumps(res, indent=1) + '\n')
    for r in res['rows']:
        print(r['run'], r['pose'], r['verdict'], [(c['a'], c['b'], c['depth_mm']) for c in r['contacts']][:3])
