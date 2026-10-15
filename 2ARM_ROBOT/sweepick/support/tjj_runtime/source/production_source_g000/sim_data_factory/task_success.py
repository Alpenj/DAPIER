"""SIM-only same-block handover checker. Decides task stage from physics evidence only.

Not a hardware gate: it never authorizes robot execution and does not reuse the
hardware non-contact clearance rules. Intended contacts are named explicitly:
pad<->block (grasp) and block<->table (support). Everything else touching a robot
geom is forbidden (robot<->table/stand/camera, left<->right, same-arm non-adjacent).
"""
from dataclasses import dataclass, asdict
import numpy as np
import mujoco

SIDES = ('left', 'right')
BLOCK_BODY = 'red_block'
BLOCK_GEOM = 'red_block_geom'
TABLE_GEOM = 'table'


@dataclass(frozen=True)
class CheckerConfig:
    grip_force_min_N: float = 0.3        # per pad, both pads of one gripper
    grip_confirm_s: float = 0.2
    lift_clearance_m: float = 0.015      # block bottom above table top
    lift_confirm_s: float = 0.3
    release_force_max_N: float = 0.01    # donor pads "not touching"
    donor_clearance_m: float = 0.010     # donor gripper geoms (pads+housing) to block, after retreat
    hold_s: float = 1.0                  # dataset task definition, NOT a hardware hold time
    max_block_slip_m_s: float = 0.05     # block speed relative to recipient grasp site
    max_penetration_m: float = 0.002
    max_block_jump_m: float = 0.03       # per control step; larger = teleport/overwrite
    joint_limit_tol_rad: float = 0.02


def _points_to_box(points, centre, rot, half):
    """Signed distance of world points to an oriented box (negative inside)."""
    q = np.abs((np.asarray(points) - centre) @ rot) - half
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(axis=-1), 0)


def _geom_points_obb(model, data, g):
    """World vertices and oriented bounding box (geom frame) of a box or mesh geom."""
    rot, pos = data.geom_xmat[g].reshape(3, 3), data.geom_xpos[g]
    if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
        mid = model.geom_dataid[g]
        local = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
    else:
        local = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * model.geom_size[g]
    lo, hi = local.min(0), local.max(0)
    return local @ rot.T + pos, pos + rot @ ((lo + hi) / 2), rot, (hi - lo) / 2


def geom_distance(model, data, g1, g2):
    """Approximate separation of two box/mesh geoms: min(vertices->other OBB) both ways.
    Planner heuristic only; NOT a whole-surface separation certificate."""
    p1, c1, r1, h1 = _geom_points_obb(model, data, g1)
    p2, c2, r2, h2 = _geom_points_obb(model, data, g2)
    return float(min(_points_to_box(p1, c2, r2, h2).min(), _points_to_box(p2, c1, r1, h1).min()))

def separation_lower_bound(model, data, g1, g2):
    """Whole-shape separation proof, not the planner's vertex-distance heuristic."""
    from collision_guard import (certified_mesh_box_separation_lower_bound,
                                  certified_box_box_separation_lower_bound)
    return max(certified_mesh_box_separation_lower_bound(model,data,g1,g2),
               certified_box_box_separation_lower_bound(model,data,g1,g2))


def _geom_side(model, geom):
    name = model.body(int(model.geom_bodyid[geom])).name
    for side in SIDES:
        if name.startswith(side + '_'):
            return side
    return None


class ContactModel:
    """Static geom classification for one compiled model."""
    def __init__(self, model):
        self.model = model
        self.block_body = model.body(BLOCK_BODY).id
        self.block_geom = model.geom(BLOCK_GEOM).id
        self.table = model.geom(TABLE_GEOM).id
        self.side = [_geom_side(model, g) for g in range(model.ngeom)]
        self.pads = {(side, f): model.geom(f'{side}_pgripper_pad_{f}').id for side in SIDES for f in (1, 2)}
        self.pad_index = {g: (side, f) for (side, f), g in self.pads.items()}
        self.site = {side: model.site(f'{side}_cube_grasp').id for side in SIDES}
        self.block_dof = int(model.jnt_dofadr[model.body_jntadr[self.block_body]])
        self.block_qpos = int(model.jnt_qposadr[model.body_jntadr[self.block_body]])
        self.table_top_z = float(model.geom_pos[self.table][2] + model.geom_size[self.table][2])
        self.block_half = float(model.geom_size[self.block_geom][0])
        self.gripper_geoms = {s: [model.geom(f'{s}_pgripper_{n}').id for n in ('housing', 'pad_1', 'pad_2')]
                              for s in SIDES}

    def substep_failure(self, data):
        """Inspect contacts actually used by mj_step, without changing physics caches."""
        forbidden=[];depth=0.
        for c in data.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(depth,-float(c.dist))
            if self.block_geom not in (a,b) and (self.side[a] or self.side[b]):
                forbidden.append([self.model.geom(g).name or f'geom{g}' for g in (a,b)])
        warnings=int(sum(w.number for w in data.warning))
        if forbidden or depth>CheckerConfig().max_penetration_m or warnings:
            return dict(reason='physics_substep_fault',forbidden_contacts=forbidden,
                        max_penetration_m=depth,warnings=warnings,time=float(data.time),
                        contact_configuration_time=float(data.time-self.model.opt.timestep))
        return None

    def evidence(self, data):
        """Read-only per-state evidence. Requires data after mj_forward (contacts + efc_force)."""
        m = self.model
        pad = {side: np.zeros(2) for side in SIDES}
        support = 0.0
        forbidden, other_block = [], []
        depth = 0.0
        wrench = np.zeros(6)
        for i, c in enumerate(data.contact):
            g1, g2 = int(c.geom1), int(c.geom2)
            depth = max(depth, -float(c.dist))
            pair = {g1, g2}
            mujoco.mj_contactForce(m, data, i, wrench)
            normal = max(0.0, float(wrench[0]))
            if self.block_geom in pair:
                other = g2 if g1 == self.block_geom else g1
                if other in self.pad_index:
                    side, f = self.pad_index[other]
                    pad[side][f - 1] += normal
                elif other == self.table:
                    support += normal
                else:
                    other_block.append(m.geom(other).name or f'geom{other}@{m.body(int(m.geom_bodyid[other])).name}')
                continue
            s1, s2 = self.side[g1], self.side[g2]
            if s1 is None and s2 is None:
                continue
            forbidden.append(tuple(sorted(
                (m.geom(g).name or f'geom{g}@{m.body(int(m.geom_bodyid[g])).name}') for g in (g1, g2))))
        block_pos = data.xpos[self.block_body].copy()
        block_vel = data.qvel[self.block_dof:self.block_dof + 3].copy()
        rel_speed = {}
        for side in SIDES:
            vel = np.zeros(6)
            mujoco.mj_objectVelocity(m, data, mujoco.mjtObj.mjOBJ_SITE, self.site[side], vel, 0)
            rel_speed[side] = float(np.linalg.norm(block_vel - vel[3:]))
        # A minimum over vertices can miss edge/face contact. Require a valid
        # whole-shape lower bound for the unchanged 10 mm donor-release criterion.
        gripper_block = {}
        for s in SIDES:
            dist = np.inf
            for g in self.gripper_geoms[s]:
                dist = min(dist, separation_lower_bound(m,data,g,self.block_geom))
            gripper_block[s] = float(dist)
        return dict(pad_force_N={s: pad[s].tolist() for s in SIDES}, table_support_N=support,
                    gripper_block_distance_m=gripper_block,
                    block_pos=block_pos.tolist(), block_quat=data.xquat[self.block_body].tolist(),
                    block_bottom_clearance_m=float(block_pos[2] - self.block_half - self.table_top_z),
                    block_speed_rel_site=rel_speed, forbidden_contacts=sorted(set(forbidden)),
                    non_pad_block_contacts=sorted(set(other_block)), max_penetration_m=depth)


def evaluate_states(model, integration_states, q_state=None, config=CheckerConfig(), control_dt=None):
    """Replay-copy checker: mj_setState + mj_forward on a separate MjData per recorded state.

    Returns stage ('none'|'reach'|'pick_lift'|'handover'), per-step evidence and failure reasons.
    `reach` = both left pads touched the block (grasp contact), `pick_lift` = left-only
    grasp held the block off the table, `handover` = recipient-only hold with donor released.
    """
    cm = ContactModel(model)
    data = mujoco.MjData(model)
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    rows, failures = [], []
    times = []
    for k, state in enumerate(integration_states):
        mujoco.mj_setState(model, data, np.asarray(state), spec)
        mujoco.mj_forward(model, data)
        ev = cm.evidence(data)
        ev['time'] = float(data.time)
        rows.append(ev)
        times.append(float(data.time))
    dt = control_dt if control_dt is not None else (times[1] - times[0] if len(times) > 1 else 0.02)
    lo, hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
    arm_axes = [0, 1, 2, 3, 4, 6, 7, 8, 9, 10]
    events = {}
    left_grip = right_grip = lift = hold = 0.0
    worst = dict(penetration_m=0.0, block_jump_m=0.0, joint_limit_excess_rad=0.0)
    for k, ev in enumerate(rows):
        fl, fr = np.asarray(ev['pad_force_N']['left']), np.asarray(ev['pad_force_N']['right'])
        t = ev['time']
        if ev['forbidden_contacts']:
            failures.append(dict(step=k, time=t, reason='forbidden_contact', detail=ev['forbidden_contacts']))
        worst['penetration_m'] = max(worst['penetration_m'], ev['max_penetration_m'])
        if k:
            jump = float(np.linalg.norm(np.subtract(ev['block_pos'], rows[k - 1]['block_pos'])))
            worst['block_jump_m'] = max(worst['block_jump_m'], jump)
        if q_state is not None:
            q = np.asarray(q_state[k])
            excess = np.maximum(lo[arm_axes] - q[arm_axes], q[arm_axes] - hi[arm_axes]).max()
            worst['joint_limit_excess_rad'] = max(worst['joint_limit_excess_rad'], float(excess))
        if fl.min() >= config.grip_force_min_N:
            events.setdefault('reach', t)
        left_grip = left_grip + dt if fl.min() >= config.grip_force_min_N else 0.0
        if left_grip >= config.grip_confirm_s:
            events.setdefault('left_grasp', t)
        lifted = ('left_grasp' in events and fl.min() >= config.grip_force_min_N and fr.max() < config.release_force_max_N
                  and ev['table_support_N'] == 0.0 and ev['block_bottom_clearance_m'] >= config.lift_clearance_m)
        lift = lift + dt if lifted else 0.0
        if lift >= config.lift_confirm_s - 1e-9:
            events.setdefault('pick_lift', t)
        right_grip = right_grip + dt if 'pick_lift' in events and fr.min() >= config.grip_force_min_N else 0.0
        if right_grip >= config.grip_confirm_s:
            events.setdefault('right_grasp', t)
        recipient_only = ('right_grasp' in events and fr.min() >= config.grip_force_min_N
                          and fl.max() < config.release_force_max_N and ev['table_support_N'] == 0.0
                          and ev['gripper_block_distance_m']['left'] >= config.donor_clearance_m
                          and not ev['non_pad_block_contacts']
                          and ev['block_bottom_clearance_m'] >= config.lift_clearance_m
                          and ev['block_speed_rel_site']['right'] <= config.max_block_slip_m_s)
        hold = hold + dt if recipient_only else 0.0
        if hold >= config.hold_s - 1e-9:
            events.setdefault('handover', t)
    if worst['penetration_m'] > config.max_penetration_m:
        failures.append(dict(reason='penetration', value=worst['penetration_m']))
    if worst['block_jump_m'] > config.max_block_jump_m:
        failures.append(dict(reason='block_teleport_or_overwrite', value=worst['block_jump_m']))
    if worst['joint_limit_excess_rad'] > config.joint_limit_tol_rad:
        failures.append(dict(reason='joint_limit', value=worst['joint_limit_excess_rad']))
    stage = 'none'
    for name in ('reach', 'pick_lift', 'handover'):
        if name in events:
            stage = name
    valid = not failures
    return dict(stage=stage, valid=valid, handover_success=valid and stage == 'handover',
                pick_lift_success=valid and stage in ('pick_lift', 'handover'),
                events=events, failures=failures[:20], failure_count=len(failures), worst=worst,
                config=asdict(config), rows=rows)


def check_episode(path, model=None):
    """Load a recorder episode and check it on a fresh compiled scene copy."""
    from .recorder import load_episode
    meta, arrays, _, _ = load_episode(path)
    if model is None:
        from integration_scenes import task_env
        model = task_env('desk', grippers='both').model
    if int(meta['integration_state_spec']) != int(mujoco.mjtState.mjSTATE_INTEGRATION):
        raise ValueError('unexpected recorded state spec')
    return evaluate_states(model, arrays['integration_state'], arrays['q_state'],
                           control_dt=float(meta['control_dt']))
