"""Shared SIM-only clocks, substep stepping and canonical physical task evidence."""
import platform
import subprocess
from pathlib import Path
import mujoco
import numpy as np

COMMAND_DT_S = .002


def require_unassisted_dynamics(model, data):
    """A no-weld label alone does not rule out an artificial grasp spring."""
    if np.any(data.xfrc_applied != 0) or np.any(data.qfrc_applied != 0):
        raise ValueError('external applied forces are forbidden in unassisted PGripper evidence')
    block = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'red_block')
    if block >= 0:
        joint = int(model.body_jntadr[block])
        if (model.body_mocapid[block] != -1 or model.body_jntnum[block] != 1
                or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE):
            raise ValueError('PGripper evidence requires a free, non-mocap block')
        spatial = np.isin(model.eq_type, [mujoco.mjtEq.mjEQ_CONNECT, mujoco.mjtEq.mjEQ_WELD])
        attached = (model.eq_obj1id == block) | (model.eq_obj2id == block)
        if np.any(data.eq_active & spatial & attached):
            raise ValueError('active block attachments are forbidden in unassisted PGripper evidence')


def configure_physics(model, physics_substeps=1, command_dt_s=COMMAND_DT_S):
    if (type(physics_substeps) is not int or physics_substeps not in (1, 2, 4)
            or not np.isfinite(command_dt_s) or command_dt_s <= 0):
        raise ValueError('invalid command clock or physics substeps')
    model.opt.timestep = command_dt_s / physics_substeps


def step_physics(model, data, physics_substeps=1, *, command_dt_s=COMMAND_DT_S, peaks=None, pairs=None):
    if (type(physics_substeps) is not int or physics_substeps not in (1, 2, 4)
            or not np.isfinite(command_dt_s) or command_dt_s <= 0
            or not np.isclose(model.opt.timestep * physics_substeps, command_dt_s, rtol=0, atol=1e-12)):
        raise ValueError('physics and command clocks disagree')
    for _ in range(physics_substeps):
        require_unassisted_dynamics(model, data)
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
            raise ValueError('invalid physics or MuJoCo warning')
        if peaks is not None:
            pad = float(target_pad_forces(model, data, pairs).max())
            depth = max((max(0., -float(c.dist)) for c in data.contact), default=0.)
            peaks['physics_steps'] = peaks.get('physics_steps', 0) + 1
            peaks['maximum_pad_force_N'] = max(peaks.get('maximum_pad_force_N', 0.), pad)
            peaks['maximum_penetration_m'] = max(peaks.get('maximum_penetration_m', 0.), depth)
    # Feedback and task progression stay at the command boundary, not at each substep.


def execution_metadata(model, physics_substeps, target_hz, command_dt_s=COMMAND_DT_S):
    try:
        revision = subprocess.check_output(['git', '-C', str(Path(__file__).parent),
            'rev-parse', 'HEAD'], text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        revision = None
    return dict(command_dt_s=command_dt_s, physics_dt_s=float(model.opt.timestep),
        physics_substeps=physics_substeps, policy_target_hz=target_hz, source_revision=revision,
        physics_evidence_contract='unassisted: no applied forces or active block attachments',
        versions={'python': platform.python_version(), 'mujoco': mujoco.__version__, 'numpy': np.__version__})


def target_pad_forces(model, data, pairs):
    """Count exact block/pad pairs; table or opposite-arm contacts cannot pass."""
    forces = np.zeros(4)
    for index, contact in enumerate(data.contact):
        pad = pairs.get(frozenset((contact.geom1, contact.geom2)))
        if pad is not None:
            wrench = np.zeros(6)
            mujoco.mj_contactForce(model, data, index, wrench)
            if not np.isfinite(wrench).all():
                raise ValueError("non-finite contact wrench")
            forces[pad] += max(0.0, wrench[0])
    return forces.reshape(2, 2)


def table_support_force(model, data):
    pair = {model.geom("table").id, model.geom("red_block_geom").id}
    total = 0.0
    for index, contact in enumerate(data.contact):
        if {contact.geom1, contact.geom2} == pair:
            wrench = np.zeros(6)
            mujoco.mj_contactForce(model, data, index, wrench)
            if not np.isfinite(wrench).all():
                raise ValueError("non-finite table support force")
            total += max(0.0, wrench[0])
    return total


class TaskProgress:
    """Verifier only: progression is established by physics, never policy/expert labels."""
    def __init__(self):
        self.events = {}
        self.left_hold = self.right_hold = self.recipient_hold = self.support_hold = self.placed_hold = 0.
        self.failure = None

    def update(self, dt, timestamp, forces, position, speed, support):
        values = np.r_[dt, timestamp, np.asarray(forces).ravel(), position, speed, support]
        if not np.isfinite(values).all() or dt <= 0:
            raise ValueError('invalid verifier evidence')
        left, right = np.asarray(forces)
        self.left_hold = self.left_hold + dt if left.min() >= 1 else 0.
        if self.left_hold >= .33 and 'left_grasp' not in self.events:
            self.events['left_grasp'] = timestamp
        if 'left_grasp' in self.events and left.min() >= .3 and position[2] > .06:
            self.events.setdefault('left_lift', timestamp)
        self.right_hold = self.right_hold + dt if 'left_lift' in self.events and right.min() >= 1 else 0.
        if self.right_hold >= .33:
            self.events.setdefault('right_grasp', timestamp)
        holding = ('right_grasp' in self.events and left.max() < .01 and right.min() >= .3 and position[2] > .06)
        self.recipient_hold = self.recipient_hold + dt if holding else 0.
        if self.recipient_hold >= 3 - 1e-9:
            self.events.setdefault('handover', timestamp)
        if 'left_lift' in self.events and 'right_grasp' not in self.events and left.min() < .3 and support < .05:
            self.failure = 'donor lost the object before verified recipient grip'
        if ('right_grasp' in self.events and 'table_support' not in self.events
                and left.max() < .01 and right.min() < .3 and support < .05):
            self.failure = 'recipient lost the object before table support'
        self.support_hold = self.support_hold + dt if 'handover' in self.events and support >= .05 else 0.
        if self.support_hold >= .2:
            self.events.setdefault('table_support', timestamp)
        placed = ('table_support' in self.events and np.asarray(forces).max() < .01 and support >= .1
            and .019 <= position[2] <= .022 and speed <= .01 and np.linalg.norm(np.asarray(position)[:2] - [.22, -.08]) <= .003)
        self.placed_hold = self.placed_hold + dt if placed else 0.
        if self.placed_hold >= 3 - 1e-9:
            self.events.setdefault('placed', timestamp)
        return 'placed' in self.events and self.failure is None


def evidence(model, data, pairs):
    dof = int(model.joint('red_block_free').dofadr[0])
    return (target_pad_forces(model, data, pairs), data.body('red_block').xpos.copy(),
            float(np.linalg.norm(data.qvel[dof:dof + 3])), table_support_force(model, data))


def contact_pairs(model):
    return {frozenset((model.geom('red_block_geom').id, model.geom(f'{side}_pgripper_pad_{finger}').id)): i * 2 + finger - 1
        for i, side in enumerate(('left', 'right')) for finger in (1, 2)}
