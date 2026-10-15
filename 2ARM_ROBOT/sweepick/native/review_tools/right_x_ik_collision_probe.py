"""Offline feasibility of a rotation-free CURRENT_TOOL X move for the right arm, from saved REAL actual feedback.

Runs the unchanged A path: Owner.plan_observation_candidate -> HandoverTeacher.plan_move (A IK) -> Owner.segment_check
(teacher collision with the held block) -> sweepick_trajectory.validate -> plan_contact_suffix(normal_only) (normal reverse).
No device, bus, camera or executor is opened; reads are refused. The probe magnitudes are algebraic feasibility probes,
not proposed motions or limits.

    PYTHONDONTWRITEBYTECODE=1 ~/DAPIER/so101_imitation_learning/.venv/bin/python review_tools/right_x_ik_collision_probe.py OUT.json SESSION_DIR...
"""
import copy
import hashlib
import json
import sys
import tempfile
from pathlib import Path

import os

import numpy as np

HERE = Path(__file__).resolve().parents[1]
os.environ['SWEEPICK_RECEIVE_DIR'] = str(HERE / 'pin')   # as RUN_APPROVED_SESSION.py binds it
sys.path[:0] = [str(HERE / 'revision'), str(HERE)]
import sweepick_full_chain as fc  # noqa: E402
import sweepick_pick01 as p1      # noqa: E402

PROBES_M = (0.005, 0.010, 0.020)   # feasibility probes only
ARM = fc.ARM


def x_target(signed_m, basis=(1.0, 0.0, 0.0)):
    D = np.eye(4); D[:3, 3] = np.asarray(basis, float) * signed_m
    return D


def rotation_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def fk_right(model, mp, ticks_right, ticks_left):
    q = p1.rc.ticks_to_q12(mp, {'left': ticks_left, 'right': ticks_right})
    return fc.tool_pose(model, q, site='right_cube_grasp')[0]


def relative(T0, T1):
    D = np.linalg.inv(T0) @ T1
    return dict(translation_tool_m=D[:3, 3].tolist(), rotation_deg=rotation_angle_deg(D[:3, :3]))


def build_owner(session, tmp):
    planner = json.loads((session / 'RECEIVE_PLANNER_INPUT.json').read_text())
    held = json.loads((session / 'HELD_POSE.json').read_text())
    plan = json.loads((HERE / 'RECEIVE_FIELD_PLAN.json').read_text())
    fc.codex()
    from sim_data_factory.teacher import HandoverTeacher
    owner = fc.Owner(tmp, None, {}, p1=p1, read=lambda: (_ for _ in ()).throw(RuntimeError('device read refused in offline probe')))
    owner.planner_options = planner['planning_arguments']['planner_options']
    owner.teacher = HandoverTeacher(p1.model(), planner_options=owner.planner_options)
    owner.held = {k: held[k] for k in ('coarse_tool_from_object', 'seen_tool_from_object', 'tool_from_object') if held.get(k) is not None}
    if 'coarse_tool_from_object' not in owner.held:
        owner.held['coarse_tool_from_object'] = held.get('tool_from_object')
    owner.scene_look = planner['scene_look']
    owner.contact_plan = plan
    owner.scope = 'contact'
    owner.record_kind = 'SAVED_REAL_REPLAY_DIAGNOSTIC'
    owner.contact_close_executed = False                    # observation-only sessions never close the right jaw
    return owner


def view_history(session):
    rows = [json.loads(line) for line in (session / 'MOTION_HISTORY.jsonl').read_text().splitlines() if line.strip()]
    return [dict(side=r['side'], start_ticks={n: float(r['actual_start_ticks'][n]) for n in ARM}, insert_count=0, after_updated_pregrasp=False)
            for r in rows if r.get('kind') == 'terminal' and r.get('purpose') == 'OBSERVATION_VIEW' and r.get('actual_motion') and any((r.get('writes') or {}).values())]


def probe_state(session, stage_dir, label, with_views):
    obs = json.loads((stage_dir / 'OBSERVATION.json').read_text())
    raw = obs['actual_feedback']; prefix = json.loads((session / 'RECEIVE_APPROACH_PLAN.json').read_text())
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        owner = build_owner(session, tmp)
        model = p1.model(); mp = owner.mp
        start_T = fk_right(model, mp, raw['ticks']['right'], raw['ticks']['left'])
        for signed in [s * m for m in PROBES_M for s in (1, -1)]:
            if with_views:
                owner.contact_observation_history = view_history(session)
            else:
                owner.__dict__.pop('contact_observation_history', None)
            candidate = dict(id=f'RIGHT_X_PROBE_{signed * 1000:+.0f}mm', source_kind='SENSOR_SIM_CANDIDATE',
                             targets=[dict(side='right', reference_frame='CURRENT_TOOL', reference_from_target_tool=x_target(signed).tolist())],
                             expected_observed_components=['right wrist RGB at the X endpoint'], observation_policy='FINAL_TARGET_ONLY')
            result = owner.plan_observation_candidate(candidate, copy.deepcopy(raw), prefix)
            row = dict(probe_tool_x_m=signed, ready=result['ready'], why=result.get('why'), failure=result.get('failure'))
            if result['forward']:
                step = result['forward'][0]
                end_T = fc.tool_pose(model, np.asarray(step['target_q12']), site='right_cube_grasp')[0]
                traj = step['trajectory']; goals = traj.get('goals') or {}
                dev = []
                for k in range(int(traj.get('steps') or 0)):
                    sample = {n: float(goals[n][k]) if n in goals else float(raw['ticks']['right'][n]) for n in fc.ALL}
                    rel = relative(start_T, fk_right(model, mp, sample, raw['ticks']['left']))
                    t = np.asarray(rel['translation_tool_m'])
                    dev.append(dict(off_axis_m=float(np.hypot(t[1], t[2])), rotation_deg=rel['rotation_deg'], x_m=float(t[0])))
                end = relative(start_T, end_T)
                row.update(target_ticks=step['target_ticks'],
                           joint_delta_ticks={n: step['target_ticks'][n] - step['start_ticks'][n] for n in ARM},
                           endpoint_fk_from_command_ticks=end,
                           endpoint_x_error_m=end['translation_tool_m'][0] - signed,
                           path=dict(samples=len(dev), max_off_axis_m=max((d['off_axis_m'] for d in dev), default=0.0),
                                     max_rotation_deg=max((d['rotation_deg'] for d in dev), default=0.0),
                                     x_monotonic=all(abs(b['x_m']) + 1e-9 >= abs(a['x_m']) for a, b in zip(dev, dev[1:]))),
                           ik=_short(step.get('ik')),
                           collision=dict(verdict=step['certificate'].get('nominal_verdict'), samples=step['certificate'].get('nominal_steps_checked'),
                                          required=step['certificate'].get('samples_required'), first_contact=step['certificate'].get('first_nominal_contact')),
                           native_validation=dict(executable=step['native_validation'].get('executable'),
                                                  hard=step['native_validation'].get('hard'), provisional=step['native_validation'].get('provisional')),
                           planned_duration_s=traj.get('duration_s'))
            nr = result.get('normal_return') or {}
            row['normal_return'] = dict(ready=nr.get('ready'), why=nr.get('why'), labels=[r['label'] for r in nr.get('reverse', [])])
            rows.append(row)
    return dict(state=label, stage=stage_dir.name, observation_sha256=hashlib.sha256((stage_dir / 'OBSERVATION.json').read_bytes()).hexdigest(),
                bus_seq=raw.get('bus_seq'), right_start_ticks=raw['ticks']['right'],
                start_tool_axes_world=dict(x_approach=start_T[:3, 0].tolist(), y=start_T[:3, 1].tolist(), z_jaw=start_T[:3, 2].tolist()),
                prior_observation_views_reversed=len(view_history(session)) if with_views else 0, probes=rows)


def _short(info):
    if not isinstance(info, dict):
        return info
    return {k: v for k, v in info.items() if isinstance(v, (str, int, float, bool)) or v is None}


def main(out, *sessions):
    report = dict(schema='sweepick.right-x-ik-collision-probe.v1', device_access=False, motor_writes=0,
                  probe_role='algebraic feasibility probes of the A IK/collision/native path; not proposed motions, distances or limits',
                  source_sha256={n: hashlib.sha256((HERE / 'revision' / n).read_bytes()).hexdigest()
                                 for n in ('sweepick_full_chain.py', 'sweepick_pick01.py', 'sweepick_trajectory.py', 'current_view_candidate.py')},
                  states=[])
    for s in map(Path, sessions):
        decisions = sorted((s / 'decision_observations').iterdir())
        pick = lambda tag: next(d for d in decisions if d.name.endswith(tag))
        report['states'].append(dict(session=s.parent.name, **probe_state(s, pick('_RECEIVE_CONTACT'), 'AFTER_RIGHT_PREGRASP_BEFORE_VIEW', False)))
        report['states'].append(dict(session=s.parent.name, **probe_state(s, pick('_OBSERVATION_CANDIDATE'), 'AFTER_ADOPTED_OBSERVATION_VIEW', True)))
    Path(out).write_text(json.dumps(fc._plain(report), indent=1, default=str))


if __name__ == '__main__':
    main(*sys.argv[1:])
