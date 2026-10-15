"""One physics clock and explicit state/action units for the SIM pilot."""
from pathlib import Path
import hashlib
import sys
import copy
import numpy as np
import mujoco
from integration_scenes import task_env, portable_model_sha256
from mobile_dual_so101 import ACTION_NAMES, actuator_targets_from_qpos, apply_control_as_pose
from pgripper import MOTOR_MAX_RAD
from shoe_task import DEFAULT_FRAME_SKIP
from pgripper_execution import require_unassisted_dynamics

REVISION = 'sim-data-r2'
UNITS = tuple('opening_fraction' if n.endswith('_gripper') else 'rad' for n in ACTION_NAMES)

def decode_command(q, model, *, names=ACTION_NAMES, units=UNITS):
    """Canonical opening fraction 0=closed/1=open -> PGripper gear target rad."""
    q = np.array(q, dtype=float, copy=True)
    if tuple(names) != ACTION_NAMES or tuple(units) != UNITS or q.shape != (12,) or not np.isfinite(q).all():
        raise ValueError('joint order/units/finite 12-vector required')
    if np.any(q[[5,11]] < 0) or np.any(q[[5,11]] > 1):
        raise ValueError('gripper opening must be in [0,1]')
    q[[5,11]] = q[[5,11]]*MOTOR_MAX_RAD
    if np.any(q < model.actuator_ctrlrange[:,0]) or np.any(q > model.actuator_ctrlrange[:,1]):
        raise ValueError('command outside model actuator limits; no clipping')
    return q

def encode_state(model, data):
    q = np.array(actuator_targets_from_qpos(model,data.qpos))
    q[[5,11]] = q[[5,11]]/MOTOR_MAX_RAD
    return q

def snapshot(context):
    m,d=context['model'],context['data']
    dofs=m.jnt_dofadr[m.actuator_trnid[:,0]]
    velocity=d.qvel[dofs].copy();velocity[[5,11]] /= MOTOR_MAX_RAD
    state=np.empty(mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(m,d,state,mujoco.mjtState.mjSTATE_INTEGRATION)
    return dict(q_state=encode_state(m,d), qvel=velocity, sim_time=float(d.time),
                qpos=d.qpos.copy(), sim_qvel=d.qvel.copy(),integration_state=state)

def make_episode(seed, initial_state, scene_config):
    """Explicit SIM reset only. No fallback home, settling, randomization or I/O."""
    if initial_state is None: raise ValueError('explicit initial_state required; sampler belongs to teacher')
    if set(scene_config)-{'kind'} or scene_config.get('kind','desk')!='desk':
        raise ValueError('pilot supports existing desk scene only')
    env=task_env('desk',grippers='both');m,d=env.model,env.data
    if tuple(m.actuator(i).name for i in range(m.nu)) != ACTION_NAMES:
        raise ValueError('actuator identity mismatch')
    # These are position servos: force=kp*ctrl-kp*q-kd*qvel, not torque commands.
    if (np.any(m.actuator_dyntype != 0) or np.any(m.actuator_gaintype != 0)
        or np.any(m.actuator_biastype != 1) or not np.allclose(m.actuator_gainprm[:,0],-m.actuator_biasprm[:,1])
        or np.any(m.actuator_gear[:,0]!=1)):
        raise ValueError('unsupported actuator semantics')
    ctrl=decode_command(initial_state['q'],m,names=initial_state['names'],units=initial_state['units'])
    apply_control_as_pose(m,d,ctrl,preserve_raw_pose=True)  # reset only; never in rollout
    require_unassisted_dynamics(m,d)
    for c in d.contact:
        bodies=[m.body(int(m.geom_bodyid[g])).name for g in (c.geom1,c.geom2)]
        if c.dist < 0 and any(n.startswith(('left_','right_')) for n in bodies):
            raise ValueError(f'initial forbidden robot contact: {bodies}, distance={c.dist}')
    context=dict(model=m,data=d,seed=int(seed),scene_config=dict(scene_config),
                 physics_steps=DEFAULT_FRAME_SKIP,control_dt=float(m.opt.timestep*DEFAULT_FRAME_SKIP))
    context['initial']=snapshot(context)
    context['compiled_model_sha256']=portable_model_sha256(m)
    return context

SOURCE_IDENTITY_SKIPS = {}

def source_identity():
    # Relative third-party synthetic modules (torch.ops) are not project files.
    root=Path(__file__).resolve().parents[1]
    result={};SOURCE_IDENTITY_SKIPS.clear()
    for name,mod in list(sys.modules.items()):
        raw=getattr(mod,'__file__',None)
        if not isinstance(raw,str):continue
        p=Path(raw)
        if not p.is_absolute():
            origin=getattr(getattr(mod,'__spec__',None),'origin',None)
            if isinstance(origin,str) and Path(origin).is_absolute():p=Path(origin)
            elif name.startswith('sim_data_factory'):
                raise ValueError('unresolved project source: '+name)
            else:
                SOURCE_IDENTITY_SKIPS[name]='relative synthetic/non-project __file__: '+raw
                continue
        p=p.resolve()
        if p.suffix=='.py' and p.is_relative_to(root):
            result[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    return result

def rollout(context, teacher, sensor, recorder, steps):
    m,d=context['model'],context['data']
    if context.get('terminal_failure'):return False
    for index in range(steps):
        obs=snapshot(context)
        cameras=copy.deepcopy(sensor.observe(m,d,context))
        command,diagnostics=teacher.next_command(context)
        command=np.array(command,copy=True);diagnostics=copy.deepcopy(diagnostics)
        if not np.array_equal(obs['integration_state'],snapshot(context)['integration_state']):
            raise ValueError('sensor/teacher mutated authoritative state')
        require_unassisted_dynamics(m,d)
        if getattr(teacher,'failure',None):
            context['terminal_failure']=dict(teacher.failure)
            context['terminal_state']=snapshot(context)
            return False
        ctrl=decode_command(command,m)
        d.ctrl[:]=ctrl
        for substep in range(context['physics_steps']):
            mujoco.mj_step(m,d)
            check=context.get('substep_check')
            fault=check(d) if check else None
            if fault:
                context['terminal_failure']={**fault,'substep':substep}
                context['terminal_state']=snapshot(context)
                # Partial intervals are terminal evidence, not fake 20ms transitions.
                return False
        mujoco.mj_forward(m,d)
        require_unassisted_dynamics(m,d)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
            raise ValueError('nonfinite physics state')
        recorder.append(obs,command,snapshot(context),dict(diagnostics),ctrl,cameras)
    return True
