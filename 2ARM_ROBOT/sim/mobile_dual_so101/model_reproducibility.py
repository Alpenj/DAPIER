"""Read-only desk compilation/fixture probe; writes evidence only to --output."""
import argparse, hashlib, importlib, json, os, platform, subprocess, sys
from pathlib import Path
import numpy as np
import mujoco

p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--overlay',action='store_true');a=p.parse_args()
repo=a.repo.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
sim=repo/'2ARM_ROBOT/sim/mobile_dual_so101';sys.path.insert(0,str(sim))
if a.overlay:
    import lerobot.envs
    lerobot.envs.__path__.insert(0,str(repo/'so101/integrations/lerobot_v0_6_so101_mujoco/overlay/src/lerobot/envs'))
from integration_scenes import task_env, portable_model_sha256, task_provenance
from collision_guard import certified_box_box_separation_lower_bound
from mobile_dual_so101 import resolve_so101_model
sha=lambda b:hashlib.sha256(b).hexdigest()
env=task_env();m,d=env.model,env.data
arrays={};fields={}
path_fields={'paths','npaths','nbuffer','mesh_pathadr','tex_pathadr','hfield_pathadr','skin_pathadr'}
def visit(obj,prefix=''):
    for n in sorted(dir(obj)):
        if n.startswith('_'):continue
        v=getattr(obj,n)
        if callable(v):continue
        k=prefix+n
        if isinstance(v,np.ndarray):
            arrays[k]=v.copy();fields[k]={'type':type(v).__name__,'dtype':v.dtype.str,'shape':v.shape,'sha256':sha(v.tobytes()),'portable_included':bool(prefix or n not in path_fields)}
        elif isinstance(v,bytes):fields[k]={'type':'bytes','hex':v.hex(),'sha256':sha(v),'portable_included':n not in path_fields}
        elif isinstance(v,(str,int,float)):fields[k]={'type':type(v).__name__,'value':v,'portable_included':n not in path_fields}
        elif k in {'opt','stat','vis','vis.global_','vis.headlight','vis.map','vis.quality','vis.rgba','vis.scale'}:visit(v,k+'.')
        else:raise TypeError(k)
visit(m)
np.savez_compressed(out/'compiled-fields.npz',**arrays)
mujoco.mj_saveModel(m,str(out/'desk.mjb'),None)
source=resolve_so101_model();arm=mujoco.MjSpec.from_file(str(source))
assets={str(source):sha(source.read_bytes())}
for mesh in arm.meshes:
    if mesh.file:
        path=(source.parent/arm.meshdir/mesh.file).resolve();assets[str(path)]=sha(path.read_bytes())
modules={}
for n in ('mujoco','numpy','lerobot','lerobot.envs','lerobot.envs.so101_mujoco.env','lerobot.envs.so101_mujoco.camera_profiles','integration_scenes','replay_recorded_episode','pgripper','camera_stand_cad','collision_guard','shoe_task'):
    mod=importlib.import_module(n);f=getattr(mod,'__file__',None);modules[n]={'file':f,'sha256':sha(Path(f).read_bytes()) if f else None}
libs={str(f):sha(f.read_bytes()) for f in Path(mujoco.__file__).parent.glob('*.so*')}
fixture=json.loads((sim/'test/fixtures/box_box_false_zero.json').read_text());d.qpos[:]=fixture['qpos'];d.qvel[:]=fixture['qvel'];d.ctrl[:]=fixture['target_q'];mujoco.mj_forward(m,d)
rows=[]
for g in fixture['pair']:
    b=int(m.geom_bodyid[g]);rows.append({'geom_id':g,'geom_name':m.geom(g).name,'body_id':b,'body_name':m.body(b).name,**{n:np.asarray(getattr(m,n)[g]).tolist() for n in ('geom_type','geom_size','geom_pos','geom_quat','geom_contype','geom_conaffinity','geom_margin','geom_gap','geom_condim','geom_friction','geom_solref','geom_solimp','geom_priority','geom_rbound')},**{n:np.asarray(getattr(m,n)[b]).tolist() for n in ('body_pos','body_quat','body_parentid')},**{n:np.asarray(getattr(d,n)[g]).tolist() for n in ('geom_xpos','geom_xmat')},**{n:np.asarray(getattr(d,n)[b]).tolist() for n in ('xpos','xmat','xquat')}})
pair=fixture['pair'];ray=np.zeros(6);native=float(mujoco.mj_geomDistance(m,d,*pair,2.,ray));cert=float(certified_box_box_separation_lower_bound(m,d,*pair))
from lerobot.envs.so101_mujoco.env import _FINGER_PAD_SPECS,FINGER_PAD_CUBE_CONTACT_SOLREF
report={'environment':{'python':sys.version,'executable':sys.executable,'platform':platform.platform(),'machine':platform.machine(),'env':{k:os.environ.get(k) for k in ('PYTHONPATH','LD_LIBRARY_PATH','MUJOCO_GL','PYOPENGL_PLATFORM','DAPIER_SO101_MJCF','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS')},'modules':modules,'libraries':libs,'loaded_mujoco_maps':[line for line in Path('/proc/self/maps').read_text().splitlines() if 'mujoco' in line],'git_sha':subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip()},'portable_model_sha256':portable_model_sha256(m),'mjb_sha256':sha((out/'desk.mjb').read_bytes()),'fields':fields,'source_assets':assets,'task_provenance':task_provenance(env),'imported_constants':{'finger_pad_specs':_FINGER_PAD_SPECS,'solref':FINGER_PAD_CUBE_CONTACT_SOLREF},'box_box_fixture':{'fixture_sha256':sha((sim/'test/fixtures/box_box_false_zero.json').read_bytes()),'pair':pair,'rows':rows,'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'ctrl':d.ctrl.tolist(),'native_distance_m':native,'native_fromto':ray.tolist(),'certificate_m':cert,'expected_certificate_m':fixture['expected_lower_bound_m'],'reverse_native_distance_m':float(mujoco.mj_geomDistance(m,d,*pair[::-1],2.,None))}}
from PIL import Image
report['environment']['versions']={'mujoco':mujoco.__version__,'numpy':np.__version__,'pillow':Image.__version__}
report['environment']['os_release']=platform.freedesktop_os_release()
report['environment']['cpu']={k.strip():v.strip() for line in Path('/proc/cpuinfo').read_text().splitlines()
    if ':' in line for k,v in [line.split(':',1)] if k.strip() in ('vendor_id','model name','flags')}
(out/'manifest.json').write_text(json.dumps(report,indent=2,default=lambda v:np.asarray(v).tolist())+'\n')
print(json.dumps({'output':str(out),'portable_model_sha256':report['portable_model_sha256'],'mjb_sha256':report['mjb_sha256'],'native_distance_m':native,'certificate_m':cert,'source':str(source)}))
