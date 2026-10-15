"""Short controller/recorder test, explicitly NOT a handover demonstration."""
import argparse
from datetime import datetime,timezone
from pathlib import Path
import json
import hashlib
import os
import uuid
import numpy as np
import mujoco
from .core import (REVISION,ACTION_NAMES,UNITS,make_episode,snapshot,rollout,source_identity)
from .recorder import EpisodeRecorder,load_episode,student_observations

class NoCamera:
    def observe(self,model,data,context):return {}

class SmallCommand:
    """Contract stub: explicit small SIM command, no task-success logic."""
    def __init__(self,q):self.q=np.array(q,copy=True);self.index=0
    def next_command(self,context):
        cmd=self.q.copy();cmd[0]+=.006*min(self.index/5,1.)
        self.index+=1
        return cmd,{'phase':'NO_TASK_SMOKE','teacher_stub':True,'object_gt':context['data'].body('red_block').xpos.copy()}

def run(output_root,steps=20):
    output_root=Path(output_root)
    initial=json.loads((output_root/'initial_state_fixture_r2.json').read_text())
    context=make_episode(29,initial,{'kind':'desk'});m=context['model']
    identity=source_identity()
    model_path=Path(os.environ['DAPIER_SO101_MJCF']).resolve()
    metadata=dict(contract_revision=REVISION,seed=29,classification='synthetic_contract_test',
        model_source=str(model_path),model_source_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
        contract_sha256=hashlib.sha256((output_root/'integration_contract.md').read_bytes()).hexdigest(),
        physics_backend='mujoco_mj_step',task_scope='no_task_controller_recorder_smoke',
        task_success=False,task_validation='NOT_EVALUATED',hardware_access=False,
        hardware_qualification='NOT_EVALUATED',object_attachment=False,external_forces=False,
        requested_initial_state=initial,actual_initial_state=context['initial'],
        scene_config=context['scene_config'],compiled_model_sha256=context['compiled_model_sha256'],
        source_sha256=identity,mujoco_version=mujoco.__version__,physics_dt=float(m.opt.timestep),
        control_dt=context['control_dt'],physics_steps=context['physics_steps'],
        joint_names=ACTION_NAMES,units=UNITS,ctrl_units='arm rad / gripper gear rad',
        integration_state_spec=int(mujoco.mjtState.mjSTATE_INTEGRATION),
        sensor_status='NoCamera stub; no RGB/depth invented',teacher_status='SmallCommand stub',
        student_allowlist=['q_state','cameras.rgb','cameras.depth_m'],
        privileged_fields=['qpos','sim_qvel','integration_state','diagnostics','success'])
    name=datetime.now(timezone.utc).strftime('sim-smoke-%Y%m%dT%H%M%S-')+uuid.uuid4().hex[:8]
    recorder=EpisodeRecorder(output_root/'datasets/episodes'/name,metadata)
    rollout(context,SmallCommand(initial['q']),NoCamera(),recorder,steps)
    if source_identity()!=identity:raise ValueError('source revision changed during rollout')
    path=recorder.finish();meta,arrays,cameras,diagnostics=load_episode(path)
    np.testing.assert_allclose(arrays['q_state'][0],initial['q'],rtol=0,atol=1e-15)
    assert len(arrays['q_state'])==steps+1 and len(arrays['q_cmd'])==steps
    assert not np.array_equal(arrays['q_cmd'],arrays['q_state'][1:])
    assert np.max(abs(arrays['q_state'][-1]-arrays['q_state'][0]))>1e-6
    assert set(student_observations(arrays,cameras))=={'q_state','cameras'}
    # Replay the controller inputs in a separate scene; never overwrite rollout qpos.
    replay=make_episode(29,initial,{'kind':'desk'});rm,rd=replay['model'],replay['data']
    mujoco.mj_setState(rm,rd,arrays['integration_state'][0],mujoco.mjtState.mjSTATE_INTEGRATION)
    for i,ctrl in enumerate(arrays['ctrl_applied']):
        rd.ctrl[:]=ctrl
        for _ in range(context['physics_steps']):mujoco.mj_step(rm,rd)
        mujoco.mj_forward(rm,rd)
        np.testing.assert_allclose(rd.qpos,arrays['qpos'][i+1],rtol=0,atol=1e-12)
    result=dict(episode=str(path),transitions=steps,physics_steps=steps*context['physics_steps'],
        sim_duration_s=float(arrays['sim_time'][-1]-arrays['sim_time'][0]),
        max_state_change=float(np.max(abs(arrays['q_state'][-1]-arrays['q_state'][0]))),
        max_command_vs_next_state=float(np.max(abs(arrays['q_cmd']-arrays['q_state'][1:]))),
        physics_action_replay='PASS atol1e-12',training_eligible=False,task_success=False,hardware_access=False)
    print(json.dumps(result,indent=2));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,required=True)
    run(p.parse_args().output_root)
