"""Pilot transition recorder. Completion is not task success or dataset admission."""
from pathlib import Path
import json
import re
import numpy as np

STATE_KEYS=('q_state','qvel','sim_time','qpos','sim_qvel','integration_state')

def _json(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)

class EpisodeRecorder:
    def __init__(self, output_dir, metadata):
        self.path=Path(output_dir)
        self.path.mkdir(parents=True,exist_ok=False)
        self.metadata=json.loads(json.dumps(metadata,default=_json,allow_nan=False))
        self.states=[];self.commands=[];self.controls=[];self.diagnostics=[];self.cameras={}
        self.closed=False

    def append(self, obs_t, command_t, next_state, diagnostics, ctrl_applied, cameras):
        if self.closed:raise ValueError('episode already completed')
        for state in (obs_t,next_state):
            if any(not np.isfinite(state[k]).all() for k in STATE_KEYS):raise ValueError('nonfinite state')
        command=np.array(command_t,dtype=float,copy=True);ctrl=np.array(ctrl_applied,dtype=float,copy=True)
        if command.shape!=(12,) or ctrl.shape!=(12,) or not np.isfinite(command).all() or not np.isfinite(ctrl).all():
            raise ValueError('finite 12-vector command/control required')
        if not np.isclose(next_state['sim_time']-obs_t['sim_time'],self.metadata['control_dt'],rtol=0,atol=1e-10):
            raise ValueError('transition clock mismatch')
        if self.states and any(not np.array_equal(self.states[-1][k],obs_t[k]) for k in STATE_KEYS):
            raise ValueError('discontinuous observation; no state snapping')
        n=len(self.commands)
        if n and set(cameras)!=set(self.cameras):raise ValueError('camera set changed mid-episode')
        camera_copy={}
        for camera, fields in cameras.items():
            if not re.fullmatch(r'[A-Za-z0-9_-]+',camera):raise ValueError('unsafe camera ID')
            if set(fields)-{'rgb','depth_m','K','T_world_camera_optical','capture_time'}:
                raise ValueError('privileged or unsupported sensor field')
            if not {'rgb','K','T_world_camera_optical','capture_time'}<=set(fields):raise ValueError('camera provenance missing')
            arrays={k:np.array(v,copy=True) for k,v in fields.items()}
            if any(a.dtype.hasobject or not np.isfinite(a).all() for a in arrays.values()):raise ValueError('invalid camera array')
            rgb=arrays['rgb'];height,width=rgb.shape[:2]
            if rgb.shape!=(height,width,3) or rgb.dtype!=np.uint8:raise ValueError('RGB uint8 HxWx3 required')
            if arrays['K'].shape!=(3,3) or arrays['T_world_camera_optical'].shape!=(4,4):raise ValueError('camera transform shape')
            if float(arrays['capture_time'])!=float(obs_t['sim_time']):raise ValueError('camera time != observation time')
            if 'depth_m' in arrays and (arrays['depth_m'].shape!=(height,width) or np.any(arrays['depth_m']<0)):
                raise ValueError('metric depth shape/range')
            if n and (set(arrays)!=set(self.cameras[camera][0]) or any(arrays[k].shape!=self.cameras[camera][0][k].shape for k in arrays)):
                raise ValueError('camera fields/shape changed')
            camera_copy[camera]=arrays
        diagnostic_copy=json.loads(json.dumps(diagnostics,default=_json,allow_nan=False))
        if not self.states:self.states.append({k:np.array(obs_t[k],copy=True) for k in STATE_KEYS})
        self.states.append({k:np.array(next_state[k],copy=True) for k in STATE_KEYS})
        self.commands.append(command);self.controls.append(ctrl);self.diagnostics.append(diagnostic_copy)
        for camera,arrays in camera_copy.items():self.cameras.setdefault(camera,[]).append(arrays)

    def finish(self, extra_metadata=None):
        if self.closed or not self.commands:raise ValueError('empty or completed episode')
        if extra_metadata is not None:
            self.metadata.update(json.loads(json.dumps(extra_metadata,default=_json,allow_nan=False)))
        arrays={k:np.stack([s[k] for s in self.states]) for k in STATE_KEYS}
        arrays.update(q_cmd=np.stack(self.commands),ctrl_applied=np.stack(self.controls),control_index=np.arange(len(self.commands)))
        np.savez_compressed(self.path/'episode.npz',**arrays)
        for camera,frames in self.cameras.items():
            (self.path/'cameras').mkdir(exist_ok=True)
            np.savez_compressed(self.path/'cameras'/f'{camera}.npz',**{k:np.stack([f[k] for f in frames]) for k in frames[0]})
        (self.path/'diagnostics.json').write_text(json.dumps(self.diagnostics,indent=2,allow_nan=False))
        metadata={**self.metadata,'transitions':len(self.commands),'camera_ids':list(self.cameras),
                  'complete':True,'training_eligible':False}
        # Metadata is the last completion marker. Partial arrays alone never load.
        pending=self.path/'metadata.pending'
        pending.write_text(json.dumps(metadata,indent=2,allow_nan=False))
        pending.replace(self.path/'metadata.json');self.closed=True
        return self.path

def load_episode(path):
    path=Path(path);metadata=json.loads((path/'metadata.json').read_text())
    if not metadata.get('complete'):raise ValueError('incomplete episode')
    with np.load(path/'episode.npz',allow_pickle=False) as src:arrays={k:src[k].copy() for k in src.files}
    n=metadata['transitions']
    if any(len(arrays[k])!=n+1 for k in STATE_KEYS) or any(len(arrays[k])!=n for k in ('q_cmd','ctrl_applied','control_index')):
        raise ValueError('N+1/N causal shape mismatch')
    if arrays['q_state'].shape!=(n+1,12) or arrays['q_cmd'].shape!=(n,12):raise ValueError('joint shape mismatch')
    if any(not np.isfinite(a).all() for a in arrays.values()):raise ValueError('nonfinite episode')
    if not np.allclose(np.diff(arrays['sim_time']),metadata['control_dt'],rtol=0,atol=1e-10):raise ValueError('invalid timeline')
    cameras={}
    for camera in metadata['camera_ids']:
        if not re.fullmatch(r'[A-Za-z0-9_-]+',camera):raise ValueError('unsafe camera ID')
        with np.load(path/'cameras'/f'{camera}.npz',allow_pickle=False) as src:cameras[camera]={k:src[k].copy() for k in src.files}
        if any(len(a)!=n for a in cameras[camera].values()) or not np.array_equal(cameras[camera]['capture_time'],arrays['sim_time'][:-1]):
            raise ValueError('camera alignment mismatch')
    return metadata,arrays,cameras,json.loads((path/'diagnostics.json').read_text())

def student_observations(arrays,cameras):
    from .sensor import canonical_rgb
    return {'q_state':arrays['q_state'][:-1].copy(),
            'cameras':{name:{k:(np.stack([canonical_rgb(name,image) for image in v]) if k=='rgb' else v.copy()) for k,v in frame.items() if k in ('rgb','depth_m')}
                       for name,frame in cameras.items()}}
