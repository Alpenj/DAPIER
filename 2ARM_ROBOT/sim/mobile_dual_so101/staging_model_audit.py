#!/usr/bin/env python3
"""Deliberate staging identity refresh; never bypass the teacher's identity gate."""
import argparse
import copy
import hashlib
import json
import tempfile
from pathlib import Path
import mujoco
import numpy as np
from controlled_contact_audit import array_hashes, options
from integration_scenes import task_env, task_provenance, portable_model_sha256

PATH_FIELDS={'paths','npaths','nbuffer','mesh_pathadr','tex_pathadr','hfield_pathadr','skin_pathadr'}


def compare_models(saved,current):
    a,b=array_hashes(saved),array_hashes(current)
    fields={}
    for name in sorted(a.keys()|b.keys()):
        x,y=getattr(saved,name,None),getattr(current,name,None)
        equal=(name in a and name in b and x.shape==y.shape and x.dtype==y.dtype and a[name]==b[name])
        row=dict(equal=equal,saved_sha256=a.get(name),current_sha256=b.get(name),
                 storage_only=name in PATH_FIELDS)
        if not equal and isinstance(x,np.ndarray) and isinstance(y,np.ndarray) and x.shape==y.shape and x.size:
            row['maximum_absolute_difference']=float(np.max(np.abs(x.astype(float)-y.astype(float))))
        fields[name]=row
    oa,ob=options(saved),options(current)
    # MJB loading clears mjSpec's compilation signature. Compare the same native
    # serialization form on both sides; retain fresh fingerprints as provenance.
    hashes=[];loaded_signatures=[]
    with tempfile.TemporaryDirectory() as directory:
        for i,model in enumerate((saved,current)):
            path=Path(directory)/f'{i}.mjb'
            mujoco.mj_saveModel(model,str(path),None)
            loaded=mujoco.MjModel.from_binary_path(str(path))
            hashes.append(portable_model_sha256(loaded));loaded_signatures.append(int(loaded.signature))
    ha,hb=hashes
    meshes=[]
    for m in (saved,current):
        rows=[]
        for i in range(m.nmesh):
            va,vn=int(m.mesh_vertadr[i]),int(m.mesh_vertnum[i])
            fa,fn=int(m.mesh_faceadr[i]),int(m.mesh_facenum[i])
            rows.append(dict(name=m.mesh(i).name,
                vertices_sha256=hashlib.sha256(m.mesh_vert[va:va+vn].tobytes()).hexdigest(),
                faces_sha256=hashlib.sha256(m.mesh_face[fa:fa+fn].tobytes()).hexdigest()))
        meshes.append(rows)
    # Portable identity excludes only path storage. No rounding/tolerance removes
    # physics differences; the ordinary teacher will still require the new exact hash.
    physics_differences=[k for k,v in fields.items() if not v['equal'] and not v['storage_only']]
    option_differences={k:[oa.get(k),ob.get(k)] for k in oa.keys()|ob.keys() if oa.get(k)!=ob.get(k)}
    scalar_differences={}
    for k in dir(saved):
        if k.startswith('_') or k in PATH_FIELDS or k=='signature':continue
        x,y=getattr(saved,k),getattr(current,k)
        if isinstance(x,(bytes,str,int,float)) and (type(x)!=type(y) or x!=y):
            scalar_differences[k]=[repr(x),repr(y)]
    equivalent=ha==hb and not (physics_differences or option_differences or scalar_differences)
    return dict(classification='A' if equivalent else 'B',saved_portable_sha256=ha,current_portable_sha256=hb,
        scalar_differences=scalar_differences,
        comparison_form='both native MJB round-tripped; compilation signature is not serialized',
        original_portable_sha256=[portable_model_sha256(saved),portable_model_sha256(current)],
        original_compilation_signatures=[int(saved.signature),int(current.signature)],
        serialized_compilation_signatures=loaded_signatures,
        fields=fields,physics_array_differences=physics_differences,
        storage_array_differences=[k for k,v in fields.items() if not v['equal'] and v['storage_only']],
        option_differences=option_differences,
        saved_options=oa,current_options=ob,mesh_hashes_equal=meshes[0]==meshes[1],mesh_hashes=meshes,
        counts={k:[int(getattr(saved,k)),int(getattr(current,k))]
                for k in ('nbody','ngeom','njnt','nu','neq','nmesh','ntendon','nq','nv')},
        compared_scope='All portable compiled fields incl scalars, names, physics/options/meshes; path storage excluded')


def refresh_reference(saved_report,saved_model,current_env,*,saved_raw_sha256):
    if saved_report['provenance']['model_sha256']!=saved_raw_sha256:
        raise ValueError('saved MJB does not match original staging evidence')
    audit=compare_models(saved_model,current_env.model)
    if audit['classification']!='A':
        raise ValueError('physical/model difference: rebuild staging; old reference cannot be refreshed')
    current=task_provenance(current_env)
    for k in ('scene_id','gripper_revision'):
        if saved_report['provenance'][k]!=current[k]:
            raise ValueError('staging scope changed: '+k)
    audit['saved_raw_sha256']=saved_raw_sha256
    audit['current_raw_sha256']=current['model_sha256']
    return dict(provenance=current,
        staging_search=dict(selected=dict(offset_m=saved_report['staging_search']['selected']['offset_m'])),
        provenance_refresh=dict(classification='A',original_provenance=copy.deepcopy(saved_report['provenance']),
            portable_sha256=audit['current_portable_sha256'],
            note='Reference offset only; no copied state, cached IK or old dynamics are execution evidence')),audit


def run(args):
    args.output.mkdir(parents=True,exist_ok=False)
    raw=args.saved_model.read_bytes()
    saved=mujoco.MjModel.from_binary_path(str(args.saved_model))
    current=task_env('desk')
    old=json.loads(args.staging.read_text())
    audit=compare_models(saved,current.model)
    (args.output/'compiled-comparison.json').write_text(json.dumps(audit,indent=2)+'\n')
    refreshed,audit=refresh_reference(old,saved,current,saved_raw_sha256=hashlib.sha256(raw).hexdigest())
    refreshed['provenance_refresh']['source_report_sha256']=hashlib.sha256(args.staging.read_bytes()).hexdigest()
    (args.output/'compiled-comparison.json').write_text(json.dumps(audit,indent=2)+'\n')
    (args.output/'staging-reference.json').write_text(json.dumps(refreshed,indent=2)+'\n')
    mujoco.mj_saveModel(current.model,str(args.output/'current-base-model.mjb'),None)
    print(json.dumps({k:v for k,v in audit.items() if k not in ('fields','mesh_hashes','saved_options','current_options')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--saved-model',type=Path,required=True)
    p.add_argument('--staging',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    run(p.parse_args())
