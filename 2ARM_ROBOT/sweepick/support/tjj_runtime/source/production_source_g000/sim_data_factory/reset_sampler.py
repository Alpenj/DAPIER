"""SIM reset distribution: explicit or sampled dual-arm q0 + tabletop block XY/yaw.

Reset-only writes. core.make_episode sets the explicit arm q0; this module then places the
free block (qpos write at reset, before any recorded step) and re-snapshots the start state.
Sampled q0 is validated, never replaced by a home pose. Rejections keep their reason.
"""
from dataclasses import dataclass, asdict
import copy
import numpy as np
import mujoco
from .core import ACTION_NAMES, UNITS, decode_command, make_episode, snapshot
from .task_success import ContactModel


@dataclass(frozen=True)
class ResetDistribution:
    # Block region chosen from the desk scene: table top x[-.055,.646] y[-.6,.6]; the left
    # arm reaches top-down picks here (IK checked x .12-.25, |y| <= .06). Yaw is full square symmetry.
    block_x: tuple = (0.13, 0.24)
    block_y: tuple = (-0.06, 0.06)
    block_yaw: tuple = (-np.pi / 4, np.pi / 4)
    # Per-joint sub-boxes (rad) inside actuator ranges; product box is NOT assumed valid.
    pan: tuple = (-0.7, 0.7)
    lift: tuple = (-1.7, 0.2)
    elbow: tuple = (0.2, 1.65)
    wrist_flex: tuple = (-1.2, 1.4)
    wrist_roll: tuple = (-1.5, 1.5)
    gripper_open: tuple = (0.0, 1.0)
    min_site_height_m: float = 0.05        # grasp site above table top
    min_site_block_dist_m: float = 0.06
    dry_run_s: float = 0.3                 # controller-hold check on a COPY, not in the episode
    max_dry_run_drift_rad: float = 0.03


def _block_qpos(x, y, yaw, half):
    return np.array([x, y, half, np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])


def validate_start(model, q, block, dist=ResetDistribution()):
    """Check a candidate start on a private MjData. Returns (ok, reason, details)."""
    cm = ContactModel(model)
    try:
        ctrl = decode_command(q, model)
    except ValueError as e:
        return False, f'range: {e}', {}
    d = mujoco.MjData(model)
    adr = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    d.qpos[:] = model.qpos0
    d.qpos[adr] = ctrl
    d.qpos[cm.block_qpos:cm.block_qpos + 7] = block
    from pgripper import sync_kinematic_jaws
    sync_kinematic_jaws(model, d)
    d.ctrl[:] = ctrl
    mujoco.mj_forward(model, d)
    for c in d.contact:
        names = [model.geom(int(g)).name or model.body(int(model.geom_bodyid[g])).name for g in (c.geom1, c.geom2)]
        robot = [cm.side[int(g)] for g in (c.geom1, c.geom2)]
        if c.dist < 0 and any(robot):
            return False, f'initial contact {names}', {}
    info = {}
    for side in ('left', 'right'):
        p = d.site_xpos[cm.site[side]]
        info[f'{side}_site'] = p.tolist()
        if p[2] - cm.table_top_z < dist.min_site_height_m:
            return False, f'{side} grasp site too low ({p[2]:.3f})', info
        if np.linalg.norm(p - block[:3]) < dist.min_site_block_dist_m:
            return False, f'{side} gripper starts at the block', info
    # Controller initialization dry run on this copy: does the servo hold q0 under gravity?
    for _ in range(int(round(dist.dry_run_s / model.opt.timestep))):
        mujoco.mj_step(model, d)
    drift = np.abs(d.qpos[adr] - ctrl)
    drift[[5, 11]] = 0
    info['dry_run_max_drift_rad'] = float(drift.max())
    if not np.isfinite(d.qpos).all() or drift.max() > dist.max_dry_run_drift_rad:
        return False, f'controller does not hold q0 (drift {drift.max():.3f} rad)', info
    ev = cm.evidence(d)
    if ev['forbidden_contacts']:
        return False, f'contact during dry run {ev["forbidden_contacts"][:2]}', info
    return True, 'ok', info


def sample_start(model, rng, dist=ResetDistribution(), max_tries=200):
    """Rejection-sample (q0, block). Returns (q, block_qpos, record) with all rejections."""
    cm = ContactModel(model)
    rejections = []
    boxes = [dist.pan, dist.lift, dist.elbow, dist.wrist_flex, dist.wrist_roll, dist.gripper_open]
    for attempt in range(max_tries):
        q = np.array([rng.uniform(*b) for _ in range(2) for b in boxes])
        x, y, yaw = rng.uniform(*dist.block_x), rng.uniform(*dist.block_y), rng.uniform(*dist.block_yaw)
        block = _block_qpos(x, y, yaw, cm.block_half + cm.table_top_z)
        ok, reason, info = validate_start(model, q, block, dist)
        if ok:
            return q, block, dict(attempts=attempt + 1, rejections=rejections, validation=info,
                                  block_xy_yaw=[x, y, yaw])
        rejections.append(dict(q=q.tolist(), block_xy_yaw=[x, y, yaw], reason=reason))
    raise RuntimeError(f'no valid start in {max_tries} tries')


def make_task_episode(seed, q0, block_xy_yaw, scene_config=None, dist=ResetDistribution()):
    """core.make_episode(explicit q0) + reset-time block placement + start re-snapshot."""
    initial = dict(q=[float(v) for v in q0], names=list(ACTION_NAMES), units=list(UNITS))
    context = make_episode(seed, initial, scene_config or {'kind': 'desk'})
    m, d = context['model'], context['data']
    cm = ContactModel(m)
    block = _block_qpos(*block_xy_yaw, cm.block_half + cm.table_top_z)
    ok, reason, info = validate_start(m, np.asarray(q0, float), block, dist)
    if not ok:
        raise ValueError(f'rejected start: {reason}')
    d.qpos[cm.block_qpos:cm.block_qpos + 7] = block       # reset only; before the first recorded state
    d.qvel[cm.block_dof:cm.block_dof + 6] = 0
    mujoco.mj_forward(m, d)
    context['core_initial_before_block_placement'] = context['initial']
    context['initial'] = snapshot(context)
    context['reset'] = dict(requested_q0=initial, block_xy_yaw=[float(v) for v in block_xy_yaw],
                            block_qpos=block.tolist(), validation=info, distribution=asdict(dist))
    return context
