"""Static RECEIVE preview using the existing teacher; no simulator stepping."""
from dataclasses import replace
import hashlib
from pathlib import Path
import time
from types import MethodType

import numpy as np

from sweepick_261008_receive_runtime import configure_runtime, field_module

PLANNER_CANDIDATE_FIELDS = ('handover_x_m', 'handover_z_m', 'handover_alpha_deg', 'handover_phi_deg')
PLANNING_SCOPES = ('FULL_TASK', 'RECEIVE_APPROACH', 'RECEIVE_CONTACT', 'SUPPORT_OBSERVATION')
SESSION05_CANDIDATE_POLICY = {
    'handover_x_m': (0.157,),
    'handover_z_m': (0.16,),
    'handover_alpha_deg': (85.0, 90.0),
    'handover_phi_deg': (-20.0,),
}


def _plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


def _session05_policy_requested(planner_options):
    if planner_options is None:
        return False
    candidate = planner_options['candidate']
    return all(np.isclose(float(candidate[name]), values[0])
               for name, values in SESSION05_CANDIDATE_POLICY.items())


def _candidate_coordinates(teacher, p, a_l, j_l):
    radial = teacher.plan.radial('left', p)
    tangent = np.cross(np.array([0.0, 0.0, 1.0]), radial)
    cross_axis = np.cross(a_l, tangent)
    return dict(
        handover_x_m=float(p[0]), handover_z_m=float(p[2]),
        handover_alpha_deg=float(np.degrees(np.arccos(np.clip(-a_l[2], -1.0, 1.0)))),
        handover_phi_deg=float(np.degrees(np.arctan2(np.dot(j_l, cross_axis), np.dot(j_l, tangent)))),
    )


def _clearance_detail(teacher, base, ctrl, block_qpos):
    """The same vertices-to-opposite-OBB distances used by Planner.gripper_distance, with provenance."""
    import mujoco
    from sim_data_factory.task_success import geom_distance, separation_lower_bound
    plan = teacher.plan
    data = plan.pose(base, ctrl, block_qpos)
    rows = []
    for geom1, geom2 in plan.clearance_pairs:
        distance = min(0.05, float(geom_distance(teacher.m, data, geom1, geom2)))
        witness = np.empty(6)
        native = float(mujoco.mj_geomDistance(teacher.m, data, geom1, geom2, 0.05, witness))
        transforms = []
        for geom in (geom1, geom2):
            transform = np.eye(4)
            transform[:3, 3] = data.geom_xpos[geom]
            transform[:3, :3] = data.geom_xmat[geom].reshape(3, 3)
            transforms.append(transform.tolist())
        rows.append(dict(
            geom_pair=[teacher.m.geom(geom1).name, teacher.m.geom(geom2).name],
            geom_types=[int(teacher.m.geom_type[geom1]), int(teacher.m.geom_type[geom2])],
            distance_m=distance, native_mujoco_distance_m=native,
            native_witness_model=witness.tolist(),
            certified_separation_lower_bound_m=float(separation_lower_bound(teacher.m, data, geom1, geom2)),
            T_model_geom=transforms,
        ))
    best = min(rows, key=lambda row: row['distance_m'])
    return dict(best, distance_definition='existing geom_distance vertices-to-opposite-OBB heuristic',
                pair_count=len(rows), pose='handover grasp endpoint')


def _path_preview(teacher, base, start, goal, *, stage, candidate=None, **kwargs):
    hit = teacher.plan.path_collisions(base, start, goal, **kwargs)
    diagnostic = getattr(teacher, '_session05_diagnostic', None)
    if diagnostic is not None:
        diagnostic['counts']['path_checks'] += 1
        key = 'candidate_path_checks' if candidate is not None else 'execution_path_checks'
        diagnostic['counts'][key] += 1
        diagnostic['counts']['path_checks_clear' if hit is None else 'path_checks_collision'] += 1
        hit_detail = None
        if hit is not None and any(len(pair) >= 3 and pair[-1] == 'clearance' for pair in hit[1]):
            u = float(hit[0])
            blend = 35*u**4 - 84*u**5 + 70*u**6 - 20*u**7
            ctrl = np.asarray(start, float) + (np.asarray(goal, float) - np.asarray(start, float)) * blend
            block = kwargs.get('block_qpos')
            if kwargs.get('carry') is not None:
                block = teacher.plan.carried_block(base, ctrl, kwargs['carry'])
            hit_detail = _clearance_detail(teacher, base, ctrl, block)
            hit_detail.update(pose='path collision sample', stage=stage, path_u=u)
        diagnostic['path_checks'].append(dict(
            stage=stage, candidate=candidate, samples=int(kwargs.get('samples', 24)),
            start_q12=teacher.to_canonical(np.asarray(start, float)).tolist(),
            goal_q12=teacher.to_canonical(np.asarray(goal, float)).tolist(),
            result='CLEAR' if hit is None else 'COLLISION', collision=_plain(hit),
            collision_clearance_detail=hit_detail,
        ))
    return hit


def _session05_select_handover(self, d, rel, base, carry, tried):
    """Existing teacher selection with A-local diagnostics; selection and rejection rules stay unchanged."""
    cfg, plan = self.cfg, self.plan
    started = time.monotonic()
    diagnostic = self._session05_diagnostic
    ranked, left_cache = [], {}
    analytic = self._handover_candidates(rel)[:cfg.handover_max_candidates]
    diagnostic['counts']['analytic_candidates'] = len(analytic)
    for analytic_index, (score, p, a_l, j_l, rot, a_r, n, beta) in enumerate(analytic):
        coordinates = _candidate_coordinates(self, p, a_l, j_l)
        record = dict(
            analytic_index=analytic_index, analytic_score=float(score), candidate=coordinates,
            beta_deg=float(beta), preferred_seed=all(np.isclose(coordinates[name], values[0])
                                                      for name, values in SESSION05_CANDIDATE_POLICY.items()),
            left_ik=None, depths=[], rejection=None,
        )
        diagnostic['candidates'].append(record)
        key = (tuple(np.round(p, 4)), tuple(np.round(a_l, 4)), tuple(np.round(j_l, 4)))
        if key not in left_cache:
            left_cache[key] = plan.solve(base, self.cmd, 'left', p - rot @ rel[:3, 3], a_l, [j_l], cfg)
            diagnostic['counts']['left_ik_evaluations'] += 1
        goal_l, ik = left_cache[key]
        record['left_ik'] = dict(status='SOLVED' if goal_l is not None else 'FAILED', residual=_plain(ik),
                                 gear_q12=None if goal_l is None else goal_l.tolist(),
                                 canonical_q12=None if goal_l is None else self.to_canonical(goal_l).tolist())
        if goal_l is None:
            record['rejection'] = 'left_ik'
            diagnostic['reasons']['left_ik'] = diagnostic['reasons'].get('left_ik', 0) + 1
            tried.append(('left_ik', p.tolist(), beta))
            continue
        for depth in cfg.right_grasp_depths_m:
            diagnostic['counts']['ik_depth_evaluations'] += 1
            grasp, ik_g = plan.solve(base, goal_l, 'right', p + depth * a_r, a_r, [n], cfg)
            depth_record = dict(
                depth_m=float(depth),
                right_ik=dict(status='SOLVED' if grasp is not None else 'FAILED', residual=_plain(ik_g),
                              gear_q12=None if grasp is None else grasp.tolist(),
                              canonical_q12=None if grasp is None else self.to_canonical(grasp).tolist()),
                endpoint_clearance=None, path=None, rejection=None,
            )
            record['depths'].append(depth_record)
            if grasp is None:
                depth_record['path'] = 'NOT_EVALUATED_RIGHT_IK'
                depth_record['rejection'] = 'right_grasp_ik'
                diagnostic['reasons']['right_grasp_ik'] = diagnostic['reasons'].get('right_grasp_ik', 0) + 1
                tried.append(('right_grasp_ik', p.tolist(), beta, depth))
                continue
            both = self.cmd.copy()
            both[0:5], both[6:11], both[11] = goal_l[0:5], grasp[6:11], self.gear_open
            detail = _clearance_detail(self, base, both, plan.carried_block(base, both, carry))
            depth_record['endpoint_clearance'] = detail
            diagnostic['counts']['endpoint_clearance_evaluations'] += 1
            ranked.append((detail['distance_m'], p, a_l, j_l, a_r, n, beta, depth,
                           goal_l, grasp, ik, ik_g, record, depth_record))
    ranked.sort(key=lambda row: -row[0])
    self.info['handover_clearance_top_mm'] = [round(row[0] * 1000, 1) for row in ranked[:5]]
    diagnostic['counts']['clearance_pass'] = sum(row[0] >= cfg.plan_clearance_m for row in ranked)
    diagnostic['ranked_endpoint_clearance'] = [dict(
        candidate=row[12]['candidate'], depth_m=float(row[7]), clearance_m_exact=float(row[0]),
        geom_pair=row[13]['endpoint_clearance']['geom_pair'],
    ) for row in ranked]
    try:
        for ranked_index, (clear, p, a_l, j_l, a_r, n, beta, depth, goal_l, grasp, ik, ik_g,
                           record, depth_record) in enumerate(ranked):
            if clear < cfg.plan_clearance_m:
                for rejected in ranked[ranked_index:]:
                    rejected_depth = rejected[13]
                    rejected_depth['path'] = 'NOT_EVALUATED_ENDPOINT_REJECTED'
                    rejected_depth['rejection'] = 'clearance'
                    diagnostic['reasons']['clearance'] = diagnostic['reasons'].get('clearance', 0) + 1
                tried.append(('clearance', p.tolist(), beta, round(clear * 1000, 1)))
                break
            diagnostic['counts']['path_candidate_evaluations'] += 1
            candidate_key = dict(candidate=record['candidate'], depth_m=float(depth))
            stage, ik_r = plan.solve(base, grasp, 'right', p - cfg.right_stage_back_m * a_r, a_r, [n], cfg)
            pre, ik_pre = plan.solve(base, grasp, 'right', p - cfg.right_pregrasp_back_m * a_r, a_r, [n], cfg)
            depth_record['right_stage_ik'] = dict(status='SOLVED' if stage is not None else 'FAILED', residual=_plain(ik_r))
            depth_record['right_pregrasp_ik'] = dict(status='SOLVED' if pre is not None else 'FAILED', residual=_plain(ik_pre))
            if stage is None or pre is None:
                depth_record['path'] = 'NOT_EVALUATED_RIGHT_STAGE_IK'
                depth_record['rejection'] = 'right_stage_ik'
                diagnostic['reasons']['right_stage_ik'] = diagnostic['reasons'].get('right_stage_ik', 0) + 1
                tried.append(('right_stage_ik', p.tolist(), beta))
                continue
            left_goal = self.cmd.copy(); left_goal[0:5] = goal_l[0:5]
            right_stage = self.cmd.copy(); right_stage[6:11] = stage[6:11]; right_stage[11] = self.gear_open
            both = np.r_[left_goal[:6], right_stage[6:]]
            pre_c = both.copy(); pre_c[6:11] = pre[6:11]
            grasp_c = both.copy(); grasp_c[6:11] = grasp[6:11]
            hit = _path_preview(self, base, both, pre_c, stage='right_approach_to_pregrasp',
                                candidate=candidate_key, carry=carry, allow_pad_block=('left',))
            if hit is None:
                hit = _path_preview(self, base, pre_c, grasp_c, stage='right_insert',
                                    candidate=candidate_key, carry=carry, allow_pad_block=('left',))
            if hit is not None:
                depth_record['path'] = 'COLLISION'
                depth_record['rejection'] = 'right_insert_collision'
                diagnostic['reasons']['right_insert_collision'] = diagnostic['reasons'].get('right_insert_collision', 0) + 1
                tried.append(('right_insert_collision', p.tolist(), beta))
                continue
            release_retreat_preview = dict(required=self.planning_scope == 'FULL_TASK')
            if self.planning_scope == 'FULL_TASK':
                released = grasp_c.copy()
                released[5] = self.cmd[5] + cfg.release_open_fraction_of_travel * (self.gear_open - self.cmd[5])
                if _path_preview(self, base, grasp_c, released, stage='left_release', candidate=candidate_key,
                                 carry=carry, allow_pad_block=('left', 'right'), samples=4) is not None:
                    depth_record['path'] = 'COLLISION'
                    depth_record['rejection'] = 'left_release_clearance'
                    diagnostic['reasons']['left_release_clearance'] = diagnostic['reasons'].get('left_release_clearance', 0) + 1
                    tried.append(('left_release_clearance', p.tolist(), beta))
                    continue
                block_h = plan.carried_block(base, grasp_c, carry)
                ok_retreat = False
                for direction in self.retreat_directions(base, released, a_r):
                    hops, _ = self.cartesian_hops(base, released, 'left', direction, cfg.left_retreat_m,
                                                  allow=('right',), step_m=0.02, block_qpos=block_h)
                    if hops is not None:
                        ok_retreat = True
                        break
                if not ok_retreat:
                    depth_record['path'] = 'NOT_EVALUATED_NO_LEFT_RETREAT'
                    depth_record['rejection'] = 'left_retreat_preview'
                    diagnostic['reasons']['left_retreat_preview'] = diagnostic['reasons'].get('left_retreat_preview', 0) + 1
                    tried.append(('left_retreat_preview', p.tolist(), beta))
                    continue
                release_retreat_preview['status'] = 'CLEAR'
            else:
                release_retreat_preview.update(status='NOT_REQUIRED_FOR_SCOPE', scope=self.planning_scope)
            left_with_right_open = left_goal.copy(); left_with_right_open[11] = self.gear_open
            order = None
            start_open = np.r_[self.cmd[:11], self.gear_open]
            for first, second in ((('right', right_stage), ('left', both)),
                                  (('left', left_with_right_open), ('right', both))):
                first_hit = _path_preview(self, base, start_open, first[1], stage='transfer_' + first[0],
                                          candidate=candidate_key, carry=carry, allow_pad_block=('left',))
                second_hit = None if first_hit is not None else _path_preview(
                    self, base, first[1], second[1], stage='transfer_' + second[0],
                    candidate=candidate_key, carry=carry, allow_pad_block=('left',))
                if first_hit is None and second_hit is None:
                    order = [first, second]
                    break
            if order is None:
                depth_record['path'] = 'COLLISION'
                depth_record['rejection'] = 'transfer_collision'
                diagnostic['reasons']['transfer_collision'] = diagnostic['reasons'].get('transfer_collision', 0) + 1
                tried.append(('transfer_collision', p.tolist(), beta))
                continue
            depth_record['path'] = 'CLEAR'
            diagnostic['counts']['path_valid_candidates'] += 1
            for not_evaluated in ranked[ranked_index + 1:]:
                later_clearance = not_evaluated[0]
                later_depth = not_evaluated[13]
                if later_depth['rejection'] is None:
                    if later_clearance < cfg.plan_clearance_m:
                        later_depth['path'] = 'NOT_EVALUATED_ENDPOINT_REJECTED'
                        later_depth['rejection'] = 'clearance'
                        diagnostic['reasons']['clearance'] = diagnostic['reasons'].get('clearance', 0) + 1
                    else:
                        later_depth['path'] = 'NOT_EVALUATED_AFTER_HIGHER_RANKED_SELECTION'
                        later_depth['rejection'] = 'higher_ranked_selection'
                        diagnostic['reasons']['higher_ranked_selection'] = (
                            diagnostic['reasons'].get('higher_ranked_selection', 0) + 1)
            diagnostic['selected'] = dict(candidate=record['candidate'], depth_m=float(depth),
                                          clearance_m_exact=float(clear), geom=depth_record['endpoint_clearance'])
            self.info.update(
                handover_point=p.tolist(), handover_left_pitch_deg=float(np.degrees(np.arccos(-a_l[2]))),
                planned_right_beta_deg=beta, planned_right_depth_m=depth,
                planned_gripper_clearance_mm=round(clear * 1000, 1),
                planned_gripper_clearance_m_exact=float(clear), left_handover_ik=ik,
                right_handover_grasp_ik=ik_g, right_stage_ik=ik_r,
                right_handover_pregrasp_ik=ik_pre, transfer_order=[side for side, _ in order],
                handover_candidates_tried=len(tried) + 1,
                left_release_retreat_preview=release_retreat_preview,
            )
            self.grip['right'] = dict(mode='open', target=self.gear_open)
            side, goal = order.pop(0)
            self.queue = order
            self.start_segment(goal, [side], 1.5)
            return
        raise self._session05_failure_type(f'no feasible handover candidate ({len(tried)} tried): {tried[:4]}')
    finally:
        diagnostic['planning_elapsed_s'] = float(time.monotonic() - started)


def validate_planner_options(planner_options):
    """Validate the optional receive IK branch selection without loading a model."""
    if planner_options is None:
        return None
    if not isinstance(planner_options, dict) or set(planner_options) != {'source', 'candidate', 'right_roll_seed'}:
        raise ValueError('planner_options requires source, candidate, and right_roll_seed only')
    if not isinstance(planner_options['source'], str) or not planner_options['source']:
        raise ValueError('planner_options.source must be a nonempty string')
    candidate = planner_options['candidate']
    if not isinstance(candidate, dict) or set(candidate) != set(PLANNER_CANDIDATE_FIELDS):
        raise ValueError(f'planner_options.candidate requires {PLANNER_CANDIDATE_FIELDS}')
    if any(isinstance(candidate[name], bool) or not isinstance(candidate[name], (int, float))
           or not np.isfinite(candidate[name]) for name in PLANNER_CANDIDATE_FIELDS):
        raise ValueError('planner_options.candidate values must be finite scalars')
    if planner_options['right_roll_seed'] != 'MODEL_LOWER_ENDPOINT':
        raise ValueError('planner_options.right_roll_seed must be MODEL_LOWER_ENDPOINT')
    return planner_options


def runtime():
    configure_runtime()
    import mujoco
    import integration_scenes
    from sim_data_factory.teacher import HandoverTeacher, TeacherFailure
    return mujoco, integration_scenes, HandoverTeacher, TeacherFailure, field_module("kin").set_q12


def build_planning_scene(state, planner_options=None):
    validate_planner_options(planner_options)
    mj, scenes, teacher_type, failure_type, set_q12 = runtime()
    scene = state["scene"]
    actual_source = hashlib.sha256(Path(scenes.__file__).read_bytes()).hexdigest()
    if scene["source_sha256"] != actual_source:
        raise ValueError("scene source SHA differs from the RECEIVE scene builder")
    spec = scenes.build_scene("desk", grippers="both")
    table = spec.geom("table")
    task_from_model = np.asarray(scene["T_task_from_model"])
    task_table = task_from_model @ np.asarray(scene["T_model_table"])
    if not np.isclose(task_table[2, 3], scene["table_top_z_m"]):
        raise ValueError("table top height differs from its task-frame transform")
    table.pos[:] = task_table[:3, 3] - task_table[:3, 2] * table.size[2]
    quat = np.empty(4)
    mj.mju_mat2Quat(quat, np.ascontiguousarray(task_table[:3, :3]).ravel())
    table.quat[:] = quat
    spec.geom("red_block_geom").size[:] = state["object_half_extents_m"]
    model = spec.compile()
    data = mj.MjData(model)
    measured = np.r_[state["left_measured_q"], state["right_measured_q"]]
    set_q12(model, data, measured)
    mj.mj_forward(model, data)
    for side in ("left", "right"):
        expected = task_from_model @ np.asarray(scene[f"T_model_{side}_base"])
        actual = np.eye(4)
        body = data.body(side + "_base")
        actual[:3, 3], actual[:3, :3] = body.xpos, body.xmat.reshape(3, 3)
        if not np.allclose(actual, expected):
            raise ValueError(f"{side} base transform differs; this desk scene cannot represent that assembly")
    pose = task_from_model @ np.asarray(state["object_pose_model"])
    teacher = teacher_type(model, planner_options=planner_options)
    if _session05_policy_requested(planner_options):
        teacher.cfg = replace(teacher.cfg, **SESSION05_CANDIDATE_POLICY)
        teacher._session05_diagnostic = dict(
            policy=dict(
                role='preferred singleton retained first; one bounded 5 degree alpha extension toward flat presentation',
                extension_basis=('alpha 90 is one half-step beyond the existing 75 to 85 degree spacing; '
                                 'same low-roll family, IK ranges and physical checks'),
                values={name: list(values) for name, values in SESSION05_CANDIDATE_POLICY.items()},
                right_grasp_depths_m=list(teacher.cfg.right_grasp_depths_m),
                plan_clearance_m=float(teacher.cfg.plan_clearance_m),
                ranking='descending endpoint clearance, then unchanged full path checks',
                random_search=False, joint_clipping=False, new_physical_thresholds=False,
            ),
            counts=dict(
                cartesian_grid_combinations=int(np.prod([len(values) for values in SESSION05_CANDIDATE_POLICY.values()])),
                analytic_candidates=0, left_ik_evaluations=0, ik_depth_evaluations=0,
                endpoint_clearance_evaluations=0, clearance_pass=0,
                path_candidate_evaluations=0, path_checks=0, candidate_path_checks=0,
                execution_path_checks=0, path_checks_clear=0, path_checks_collision=0,
                path_valid_candidates=0,
            ),
            candidates=[], ranked_endpoint_clearance=[], path_checks=[], reasons={}, selected=None,
            coverage=dict(endpoint='all right-depth IK solutions',
                          path='unchanged carry, approach and insert sampling for clearance-passing candidates'),
        )
        teacher.info['candidate_policy'] = teacher._session05_diagnostic
        teacher._session05_failure_type = failure_type
        teacher._select_handover = MethodType(_session05_select_handover, teacher)
    address = teacher.plan.cm.block_qpos
    quat = np.empty(4)
    mj.mju_mat2Quat(quat, np.ascontiguousarray(pose[:3, :3]).ravel())
    data.qpos[address:address + 7] = np.r_[pose[:3, 3], quat]
    mj.mj_forward(model, data)
    return mj, model, data, teacher, failure_type


def gear(teacher, canonical):
    result = np.array(canonical, dtype=float, copy=True)
    result[[5, 11]] = teacher.g_a + teacher.g_b * result[[5, 11]]
    return result


def plan_insert_hops(teacher, data, actual_q12, target_q12, left_hold_q):
    """Existing Cartesian IK hops from this observation; jaw commands retain their distinct provenance."""
    actual, target, hold = (np.asarray(value, float) for value in (actual_q12, target_q12, left_hold_q))
    if actual.shape != (12,) or target.shape != (12,) or hold.shape != (6,) or not all(np.isfinite(v).all() for v in (actual, target, hold)):
        raise ValueError('finite actual/target q12 and applied left hold q6 are required')
    start_q = actual.copy(); start_q[5] = hold[5]
    goal_q = start_q.copy(); goal_q[6:11] = target[6:11]
    start, goal = gear(teacher, start_q), gear(teacher, goal_q)
    base = data.qpos.copy(); address = teacher.cm.block_qpos
    block = base[address:address + 7].copy()
    teacher.plan.pose(base, start, block); begin, _ = teacher.plan.site_frame('right')
    teacher.plan.pose(base, goal, block); endpoint, _ = teacher.plan.site_frame('right')
    delta = endpoint - begin; distance = float(np.linalg.norm(delta))
    if distance <= np.finfo(float).eps:
        goals, failure = [goal], None
    else:
        goals, failure = teacher.cartesian_hops(base, start, 'right', delta / distance, distance,
                                               allow=('left',), block_qpos=block)
    if goals is None or not goals:
        raise ValueError(f'the existing Cartesian insertion planner returned no path: {failure}')
    if any(not np.array_equal(np.asarray(value)[[5, 11]], start[[5, 11]]) for value in goals):
        raise ValueError('a right insertion hop changed a native Jaw command')
    canonical = [teacher.to_canonical(np.asarray(value, float)).tolist() for value in goals]
    if any(not np.allclose(np.asarray(value)[:6], start_q[:6]) or not np.isclose(value[11], actual[11]) for value in canonical):
        raise ValueError('a right insertion hop changed the left command or the measured right opening')
    # Preserve the original command provenance instead of its lossy affine round trip.
    for value in canonical:
        value[5], value[11] = float(hold[5]), float(actual[11])
    return canonical


def plan_receive(state, *, planning_scope='FULL_TASK', planner_options=None):
    out = dict(status="NOT_READY", missing=list(state.get("missing", [])),
               real_execution_ready=False, motor_commands_sent=0, mj_step_calls=0,
               classification="PROVISIONAL model preview; no new physical HARD criteria",
               waypoints=[], left_gripper_hold_preserved=False, left_arm_frozen=False,
               geometric_start_q12=None, right_last_applied_q=None)
    try:
        validate_planner_options(planner_options)
        if planning_scope not in PLANNING_SCOPES:
            raise ValueError(f"unknown planning_scope {planning_scope!r}")
    except ValueError as exc:
        out.update(status="PLAN_REJECTED", failure=dict(stage="planner_options", cause=str(exc)))
        return out
    if state.get("status") != "INPUT_VALID":
        out["invalid"] = list(state.get("invalid", []))
        return out
    try:
        scene = (build_planning_scene(state) if planner_options is None
                 else build_planning_scene(state, planner_options=planner_options))
        mj, model, data, teacher, failure_type = scene
        if planning_scope != 'FULL_TASK' or planner_options is not None:
            teacher.set_planning_scope(planning_scope)
    except (ValueError, KeyError, RuntimeError, FileNotFoundError) as exc:
        out.update(status="NOT_READY", failure=dict(stage="scene", cause=str(exc)))
        return out
    measured = np.r_[state["left_measured_q"], state["right_measured_q"]]
    seed = np.r_[state["left_hold_q"], state["right_measured_q"]]
    seed[11] = np.clip(seed[11], 0.0, 1.0)  # Only the numerical command seed uses the EEPROM-normalized range.
    hold = float(state["left_hold_q"][5])
    out["geometric_start_q12"] = measured.tolist()
    out["command_seed_q12"] = seed.tolist()
    out["planning_seed_provenance"] = (dict(left="last_applied hold command", right="measured geometry, not applied command")
                                       if planner_options is None else dict(
        left="last_applied hold command seeds the proposed left waypoint; measured q is the geometric path start",
        right="measured geometry used as geometric trajectory start, not as the optional numerical wrist-roll seed",
    ))
    if planning_scope != 'FULL_TASK' or planner_options is not None:
        out["planning_scope"] = planning_scope
        out["planner_info"] = teacher.info
    carry = ("left", teacher.site_block_rel(data, "left"))
    current = gear(teacher, measured)
    base = data.qpos.copy()
    private_segments = []

    def waypoint(label, goal, trajectory=None):
        nonlocal current, base
        goal = np.asarray(goal, dtype=float)
        if not np.isclose(teacher.to_canonical(goal)[5], hold):
            raise ValueError("RECEIVE planner changed the left Jaw hold command")
        hit = _path_preview(teacher, base, current, goal, stage="execution_" + label,
                            carry=carry, allow_pad_block=("left",))
        if hit is not None:
            raise ValueError(f"{label}: collision preview {hit}")
        block = teacher.plan.carried_block(base, goal, carry)
        data.qpos[:] = teacher.plan.pose(base, goal, block).qpos
        mj.mj_forward(model, data)
        base, current = data.qpos.copy(), goal.copy()
        teacher.cmd = goal.copy()
        out["waypoints"].append(dict(label=label, q12=teacher.to_canonical(goal).tolist()))
        private_segments.append(trajectory)

    try:
        waypoint("LEFT_HOLD_SEED", gear(teacher, seed))
        opened = current.copy()
        opened[11] = teacher.gear_open
        waypoint("RIGHT_OPEN", opened)
        teacher._plan_transfer(data)
        first = teacher.segment
        queued = list(teacher.queue)
        waypoint("CARRY_" + teacher.info["transfer_order"][0].upper(), first["goal"], first["traj"])
        for side, goal in queued:
            teacher.start_segment(goal, [side])
            waypoint("CARRY_" + side.upper(), teacher.segment["goal"], teacher.segment["traj"])
        teacher.queue = []
        teacher._plan_right_grasp(data)
        first = teacher.segment
        queued = list(teacher.queue)
        waypoint("RIGHT_APPROACH", first["goal"], first["traj"])
        for side, goal in queued:
            teacher.start_segment(goal, [side])
            waypoint("RIGHT_INSERT", teacher.segment["goal"], teacher.segment["traj"])
    except (failure_type, ValueError) as exc:
        out.update(status="PLAN_REJECTED", failure=dict(stage="receive", cause=str(exc)))
        out["planner_info"] = teacher.info
        out["waypoints"] = []
        return out
    centre, rotation, _ = teacher.block_pose(data)
    predicted_pose = np.eye(4)
    predicted_pose[:3, 3], predicted_pose[:3, :3] = centre, rotation
    out.update(status="PLAN_PREVIEW", left_gripper_hold_preserved=True,
               planner_info=teacher.info,
               reused=["HandoverTeacher._plan_transfer", "HandoverTeacher._plan_right_grasp", "Planner.path_collisions"],
               scene=dict(state["scene"], compiled_table_top_z_m=state["scene"]["table_top_z_m"]),
               left_hold_q=list(state["left_hold_q"]),
               right_insert_q12=teacher.to_canonical(current).tolist(),
               predicted_right_object_rel=teacher.site_block_rel(data, "right").tolist(),
               predicted_carried_object_pose_task=predicted_pose.tolist(),
               object_pose_model=state["object_pose_model"],
               object_pose_uncertainty=state["object_pose_uncertainty"],
               pose_provenance="predicted carried geometry; actual RECEIVE feedback required downstream",
               _teacher=teacher, _model=model, _data=data, _carry=carry, _trajectories=private_segments)
    return out
