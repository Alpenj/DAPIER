"""Saved-state pad/box face audit. Kinematics only: never step a simulation."""
import json
from pathlib import Path
import numpy as np
import mujoco

MODE = 'ANALYTIC / KINEMATIC DIAGNOSTIC / NOT PHYSICS SUCCESS'
FACE_GROUP_TOL_M = 1e-7  # Existing supporting-face grouping precision, not a safety clearance.


def compiled_faces(model, saved):
    result=[]
    for side,ref in zip((-1,1),saved):
        gid=model.geom(ref['name']).id
        if model.geom_type[gid]!=mujoco.mjtGeom.mjGEOM_MESH:
            raise ValueError('audited PGripper face requires compiled mesh')
        mesh=int(model.geom_dataid[gid]);start=model.mesh_vertadr[mesh]
        vertices=model.mesh_vert[start:start+model.mesh_vertnum[mesh]].astype(float)
        prior=np.asarray(ref['polygon_mesh_m'])
        distances=np.linalg.norm(prior[:,None]-vertices[None,:],axis=2)
        indices=distances.argmin(axis=1);poly=vertices[indices]
        if distances.min(axis=1).max()>FACE_GROUP_TOL_M:
            raise ValueError('saved face is not in this compiled mesh frame')
        plane=np.asarray(ref['plane_mesh']);normal=plane[:3]
        residual=np.max(np.abs(poly@normal+plane[3]))
        if residual>FACE_GROUP_TOL_M or np.max(vertices@normal+plane[3])>FACE_GROUP_TOL_M:
            raise ValueError('selected polygon is not a supporting mesh face')
        result.append(dict(name=ref['name'],geom=gid,mesh=model.mesh(mesh).name,side=side,
            vertex_indices=indices.tolist(),vertices_mesh_m=poly.tolist(),normal_mesh=normal.tolist(),
            grouping_residual_m=float(residual),all_mesh_support_excess_m=float(np.max(vertices@normal+plane[3]))))
    return result


def clip_face(poly,axis,sign,limit):
    # Reuse the bench's four rectangle half-plane clips; carry 3D points to retain gap.
    out=[]
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        da=limit-sign*a[axis];db=limit-sign*b[axis]
        if da>=0:out.append(a)
        if (da>=0)!=(db>=0):out.append(a+(b-a)*da/(da-db))
    return np.asarray(out).reshape(-1,3)


def face_gap(poly_block,normal_block,half_size,side):
    poly=np.asarray(poly_block);half=np.asarray(half_size)
    # Block outward normal gives positive separation on BOTH sides; opposing pad
    # inward normal must align with its negative. Geom centers are not contact faces.
    gaps=side*poly[:,0]-half[0];patch=poly.copy()
    for axis in (1,2):
        for sign in (-1,1):patch=clip_face(patch,axis,sign,half[axis])
    area=(abs(np.sum(patch[:,1]*np.roll(patch[:,2],-1)-patch[:,2]*np.roll(patch[:,1],-1)))/2 if len(patch)>=3 else 0.)
    overlap=bool(area>0)
    extent=np.ptp(patch[:,1:3],axis=0).tolist() if overlap else None
    target=np.array([-side,0,0]);n=np.asarray(normal_block)
    return dict(min_gap_m=float(gaps.min()),mean_vertex_gap_m=float(gaps.mean()),max_gap_m=float(gaps.max()),
        wedge_m=float(np.ptp(gaps)),normal_angle_deg=float(np.degrees(np.arccos(np.clip(n@target/np.linalg.norm(n),-1,1)))),
        overlap_exists=overlap,overlap_extent_yz_m=extent,projected_overlap_area_m2=float(area),
        effective_min_gap_m=float(np.min(side*patch[:,0]-half[0])) if overlap else None,
        vertices_block_m=poly.tolist(),overlap_polygon_block_m=patch.tolist())


def observe(model,data,faces):
    block=model.geom('red_block_geom').id
    if model.geom_type[block]!=mujoco.mjtGeom.mjGEOM_BOX:raise ValueError('audited block must be BOX')
    B=data.geom_xmat[block].reshape(3,3);C=data.geom_xpos[block];half=model.geom_size[block]
    result=[]
    for face in faces:
        g=face['geom'];R=data.geom_xmat[g].reshape(3,3);o=data.geom_xpos[g]
        # mesh_vert is already compiler-centered; geom_xmat/xpos include the mesh offset.
        world=np.asarray(face['vertices_mesh_m'])@R.T+o
        poly=(world-C)@B;n=B.T@R@np.asarray(face['normal_mesh'])
        info=face_gap(poly,n,half,face['side'])
        tangent=world[1]-world[0];tangent/=np.linalg.norm(tangent);normal=R@np.asarray(face['normal_mesh'])
        info.update(name=face['name'],normal_block=n.tolist(),vertices_world_m=world.tolist(),face_vertex_centroid_world_m=world.mean(0).tolist(),
            normal_world=normal.tolist(),tangent_axes_world=[tangent.tolist(),np.cross(normal,tangent).tolist()],
            block_face_center_world_m=(C+face['side']*half[0]*B[:,0]).tolist(),block_outward_normal_world=(face['side']*B[:,0]).tolist())
        result.append(info)
    return result


def onset(times,gaps,tolerance=0.):
    # Bracket the observed samples; do not fabricate linear jaw travel between them.
    for i in range(1,len(times)):
        if gaps[i-1] is not None and gaps[i] is not None and gaps[i-1]>tolerance and gaps[i]<=tolerance:
            return dict(time_s=float(times[i]),bracket_s=[float(times[i-1]),float(times[i])],sample_index=i)
    return None


def trace_geometry(model,pre_qpos,rows,faces):
    data=mujoco.MjData(model)
    jaws=[model.joint(f'left_pgripper_jaw_{i}_slide').id for i in (1,2)]
    qa=model.jnt_qposadr[jaws]
    blockj=model.body_jntadr[model.body('red_block').id];ba=model.jnt_qposadr[blockj]
    tracks={'fixed_preclose':[],'actual':[],'recorded_gripper_frozen_block':[]}
    for row in rows:
        raw=np.asarray(row['raw_qpos'])
        # Both passive jaws are measured, independently. Never synthesize their
        # equality relation from the motor command, especially after contact.
        states={'fixed_preclose':np.array(pre_qpos), 'actual':raw.copy(), 'recorded_gripper_frozen_block':raw.copy()}
        states['fixed_preclose'][qa]=raw[qa]
        states['recorded_gripper_frozen_block'][ba:ba+7]=pre_qpos[ba:ba+7]
        for name,q in states.items():
            data.qpos[:]=q;mujoco.mj_kinematics(model,data)
            tracks[name].append([v['effective_min_gap_m'] for v in observe(model,data,faces)])
    return tracks


def audit(bench_model,bench_rows,arm_model,arm_summary,arm_rows,face_file,output):
    import hashlib
    from pgripper import jaw_gap_m
    output.mkdir(exist_ok=False)
    definitions=json.loads(face_file.read_text())['pads']
    m0=mujoco.MjModel.from_binary_path(str(bench_model));m2=mujoco.MjModel.from_binary_path(str(arm_model))
    f0=compiled_faces(m0,definitions);f2=compiled_faces(m2,definitions)
    saved0=json.loads(bench_rows.read_text());r0=next(r for r in saved0['rows'] if r['phase']=='CLOSE')
    q0=np.array(r0['raw_qpos_pre_step']);d0=mujoco.MjData(m0);d0.qpos[:]=q0;mujoco.mj_kinematics(m0,d0)
    saved2=json.loads(arm_summary.read_text());d2=mujoco.MjData(m2)
    mujoco.mj_setState(m2,d2,np.array(saved2['preclose_state']),mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_kinematics(m2,d2);q2=d2.qpos.copy();t0=float(d2.time)
    # Artifact traces contain measured motor AND passive slides at every step.
    rows=[json.loads(line) for line in arm_rows.read_text().splitlines()];rows=[r for r in rows if r['phase']=='CLOSE']
    times=[0.]+[r['time_s']-t0 for r in rows]
    initial={'raw_qpos':q2.tolist()};track=trace_geometry(m2,q2,[initial]+rows,f2)
    predicted={}
    for label,values in track.items():
        contacts=[onset(times,[p[i] for p in values]) for i in range(2)]
        bands=[[onset(times,[p[i] for p in values],tol) for tol in (FACE_GROUP_TOL_M,-FACE_GROUP_TOL_M)] for i in range(2)]
        predicted[label]=dict(onset=contacts,grouping_precision_onset_band=bands,
            delta_pad1_minus_pad2_s=contacts[0]['time_s']-contacts[1]['time_s'] if all(contacts) else None)
    actual={}
    for kind in ('contact_count','summed_normal_force_N'):
        found=[next(r for r in rows if r['normalized']['pads'][f'left_pgripper_pad_{i}'][kind]>0) for i in (1,2)]
        ts=[r['time_s']-t0 for r in found]
        actual[kind]=dict(times_s=ts,steps=[r['step'] for r in found],delta_pad1_minus_pad2_s=ts[0]-ts[1])
    state_info={}
    for label,m,d,faces in [('A0',m0,d0,f0),('A2',m2,d2,f2)]:
        metrics=observe(m,d,faces)
        assert all(x['normal_angle_deg']<90 and x['effective_min_gap_m'] is not None for x in metrics)
        for x in metrics:
            assert 0<x['effective_min_gap_m']<.04
        state_info[label]=dict(pads=metrics,compiled_faces=faces,jaw_opening_m=jaw_gap_m(m,d,'left'),
            block_half_size_m=m.geom_size[m.geom('red_block_geom').id].tolist(),
            min_gap_delta_pad1_minus_pad2_m=metrics[0]['effective_min_gap_m']-metrics[1]['effective_min_gap_m'])
    events=[]
    indices=sorted(set([0]+actual['contact_count']['steps']+actual['summed_normal_force_N']['steps']+[len(rows)]))
    for i in indices:
        act=track['actual'][i];frozen=track['recorded_gripper_frozen_block'][i]
        events.append(dict(step=i,t_s=times[i],actual_effective_gap_m=act,block_frozen_effective_gap_m=frozen,
            block_motion_gap_contribution_m=[a-b if a is not None and b is not None else None for a,b in zip(act,frozen)]))
    equalities=[i for i in range(m2.neq) if m2.equality(i).name.startswith('left_')]
    coupling=dict(ntendon=m2.ntendon,actuator_target_joint=m2.joint(int(m2.actuator_trnid[5,0])).name,
        actuator_transmission_type=int(m2.actuator_trntype[5]),equality=[dict(name=m2.equality(i).name,
        type=int(m2.eq_type[i]),joint1=m2.joint(int(m2.eq_obj1id[i])).name,joint2=m2.joint(int(m2.eq_obj2id[i])).name,
        data=m2.eq_data[i].tolist(),solref=m2.eq_solref[i].tolist(),solimp=m2.eq_solimp[i].tolist()) for i in equalities])
    result=dict(mode=MODE,new_physics_steps=0,measured_passive_jaw_trace=True,states=state_info,coupling=coupling,predicted=predicted,observed=actual,block_motion_events=events,
        sign_convention='n_block outward on each ±X face; gap>0 separated, pad inward normal compared with -n_block; YZ rectangle overlap required',
        precision=dict(existing_face_group_tolerance_m=FACE_GROUP_TOL_M,zero_crossing='strict g<=0; also report existing ±face-group precision timing band',sampling_s=float(m2.opt.timestep)),
        sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (bench_model,bench_rows,arm_model,arm_summary,arm_rows,face_file)},
        limitation='Recorded jaw trajectories after first contact include the historical contact/constraint response. This is a kinematic replay, not an open-loop counterfactual physics proof.')
    np.savez_compressed(output/'gap-traces.npz',times_s=times,**{k:np.asarray(v,float) for k,v in track.items()})
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(predicted=predicted,observed=actual,block_motion_events=events),indent=2))
    return result


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=MODE)
    for name in ('bench-model','bench-rows','arm-model','arm-summary','arm-rows','face-file','output'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();audit(**vars(a))
