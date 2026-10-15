"""Existing CAD inner contact triangles and approach-side edges; MODEL_PRIOR only."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree


_PREPARED = {}


def _paths_identity(paths):
    """Hash the actual files that define one immutable input category."""
    digest = hashlib.sha256()
    files = []
    for value in paths:
        path = Path(value).resolve()
        files.extend(sorted(p for p in path.rglob('*') if p.is_file()) if path.is_dir() else [path])
    for path in files:
        content = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''): content.update(chunk)
        digest.update(str(path).encode()); digest.update(b'\0'); digest.update(content.digest())
    return digest.hexdigest()


def _input_identity(paths, values=()):
    digest = hashlib.sha256(bytes.fromhex(_paths_identity(paths)))
    for value in values:
        digest.update(json.dumps(value, sort_keys=True, separators=(',', ':'), default=str).encode()); digest.update(b'\0')
    return digest.hexdigest()


def _geometry_source_identity(model, source_model_path):
    xml_path = Path(source_model_path).resolve()
    xml = ET.parse(xml_path).getroot()
    meshdir = xml_path.parent / xml.find('compiler').get('meshdir', '')
    paths = [xml_path]
    for index in (1, 2):
        geom = model.geom(f'right_pgripper_jaw_{index}_visual').id
        name = model.mesh(int(model.geom_dataid[geom])).name
        paths.append(meshdir / xml.find(f"asset/mesh[@name='{name}']").get('file'))
    return _paths_identity(paths)


def prepare_face_geometry(model, *, model_source_path, source_model_path, calibration_paths, config_paths,
                          config_values=()):
    """Prepare only immutable model/CAD inputs; current data/FK and observations stay uncached."""
    import mujoco
    identity = dict(model_sha256=_paths_identity([model_source_path]),
                    geometry_source_sha256=_geometry_source_identity(model, source_model_path),
                    calibration_sha256=_input_identity(calibration_paths),
                    config_sha256=_input_identity(config_paths, config_values))
    key = tuple(identity.values())
    if key in _PREPARED:
        return _PREPARED[key]
    data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    contracts = {index: inner_face_contract(model, data, index, source_model_path) for index in (1, 2)}
    meshes = {}
    for index in (1, 2):
        vertices, faces, body, _ = mesh_in_body(model, data, f'right_pgripper_jaw_{index}_visual')
        meshes[index] = dict(triangles_body_m=vertices[faces], body_id=body)
    from sweepick_261010_observed_robot_mask import prepare_robot_mask_geometry
    prepared = dict(identity=identity, model=model, contracts=contracts, meshes=meshes,
                    robot_mask=prepare_robot_mask_geometry(model))
    _PREPARED[key] = prepared
    return prepared


def mesh_in_body(model, data, geom):
    g = model.geom(geom).id
    body, mesh = int(model.geom_bodyid[g]), int(model.geom_dataid[g])
    start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
    world = model.mesh_vert[start:start+count] @ data.geom_xmat[g].reshape(3,3).T + data.geom_xpos[g]
    vertices = (world-data.xpos[body]) @ data.xmat[body].reshape(3,3)
    start, count = int(model.mesh_faceadr[mesh]), int(model.mesh_facenum[mesh])
    return vertices, model.mesh_face[start:start+count].copy(), body, g


def inner_face_contract(model, data, index, source_model_path):
    pad = f'right_pgripper_pad_{index}'
    jaw = f'right_pgripper_jaw_{index}_visual'
    vertices, faces, body, visual = mesh_in_body(model, data, jaw)
    pad_vertices, _, _, _ = mesh_in_body(model, data, pad)
    site = model.site(pad+'_inner').id
    datum = model.site_pos[site].copy()
    joint = model.joint(f'right_pgripper_jaw_{index}_slide').id
    normal = model.jnt_axis[joint].copy() * (1 if index == 1 else -1)
    normal /= np.linalg.norm(normal)
    approach = data.site_xmat[model.site('right_cube_grasp').id].reshape(3,3)[:,0]
    approach_body = data.xmat[body].reshape(3,3).T @ approach
    triangles = vertices[faces]
    cross = np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    norms = np.linalg.norm(cross,axis=1)
    unit = cross / np.maximum(norms[:,None],np.finfo(float).tiny)
    # Float32 CAD/compiled-vertex representation tolerance, never a sensor/physical error bound.
    xml_path=Path(source_model_path)
    xml=ET.parse(xml_path).getroot()
    mesh=xml.find(f"asset/mesh[@name='{model.mesh(int(model.geom_dataid[visual])).name}']")
    mesh_path=xml_path.parent/xml.find('compiler').get('meshdir','')/mesh.get('file')
    source_scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
    source_vertices=np.array([np.fromstring(line[2:],sep=' ')[:3] for line in mesh_path.read_text().splitlines() if line.startswith('v ')])
    scale = float(np.linalg.norm(source_vertices*source_scale,axis=1).max())
    precision = float(np.finfo(np.float32).eps * scale)
    distal = float((pad_vertices @ approach_body).min())
    selected = ((unit @ normal >= 1-np.finfo(np.float32).eps) &
                (np.abs((triangles-datum) @ normal).max(axis=1) <= precision) &
                ((triangles @ approach_body).min(axis=1) >= distal-precision) & (norms>0))
    ids = np.flatnonzero(selected)
    if len(ids)==0:
        raise ValueError(f'{pad}: existing CAD does not identify the inner datum plane')
    points = triangles[ids].reshape(-1,3)
    parent = list(range(len(points)))
    def find(i):
        while parent[i] != i:
            parent[i]=parent[parent[i]];i=parent[i]
        return i
    for a,b in cKDTree(points).query_pairs(precision):
        parent[find(b)]=find(a)
    unique={}; labels=[]
    for i in range(len(points)):
        key=find(i)
        if key not in unique: unique[key]=len(unique)
        labels.append(unique[key])
    coords=np.array([points[k] for k in unique])
    labels=np.array(labels).reshape(-1,3)
    count=Counter()
    for tri in labels:
        if len(set(tri))<3: continue
        for a,b in zip(tri,np.roll(tri,-1)):
            count[tuple(sorted((int(a),int(b))))]+=1
    boundary=np.array([coords[list(edge)] for edge,n in count.items() if n==1])
    if not len(boundary): raise ValueError(f'{pad}: CAD face boundary unavailable')
    front=float((points @ approach_body).max())
    leading=boundary[np.all(np.abs(boundary @ approach_body-front)<=precision,axis=1)]
    assert len(leading)>0, f'{pad}: no CAD approach-side boundary segment'
    return dict(role='MODEL_PRIOR', pad=pad,body=model.body(body).name,body_id=body,
        source_visual_geom=jaw, source_collision_geom=pad, site=pad+'_inner',
        datum_point_body_m=datum, normal_body=normal,approach_axis_body=approach_body,
        inner_triangles_body_m=triangles[ids],visual_triangle_ids=ids,
        boundary_edges_body_m=boundary,approach_side_edges_body_m=leading,
        CAD_precision_m=precision,precision_role='float32 geometry/topology representation only; not measurement or physical tolerance',
        source_mesh=str(mesh_path),source_vertex_coordinate_scale_m=scale,
        site_plane_residual_max_m=float(np.abs((triangles[ids]-datum)@normal).max()),
        basis='pgripper.py replace_gripper: distal vertices source Z<-0.065; jaw1 maximum-X/jaw2 minimum-X inner datum. teacher approach=grasp site +X.',
        physical_mount_identity_verified=False,observed_boundary=False)


def project_contract(contract, model, data, K, T_world_camera, shape):
    body=contract['body_id'];R=data.xmat[body].reshape(3,3);p=data.xpos[body]
    camera_from_world=np.linalg.inv(np.asarray(T_world_camera))
    def transform(points):
        world=np.asarray(points) @ R.T+p
        return world @ camera_from_world[:3,:3].T+camera_from_world[:3,3]
    triangles=transform(contract['inner_triangles_body_m'])
    normal_camera=camera_from_world[:3,:3]@R@contract['normal_body']
    datum_camera=transform(contract['datum_point_body_m'])
    mask=np.zeros(shape,np.uint8)
    projected=[]
    for tri in triangles:
        if np.any(tri[:,2]<=0): continue
        uv=tri @ np.asarray(K).T;uv=uv[:,:2]/uv[:,2:]
        projected.append(uv)
        # Evaluate the CAD triangle at depth pixel centres; rounding thin strips includes recess/background pixels.
        x0,y0=np.maximum(np.floor(uv.min(axis=0)).astype(int),0)
        x1,y1=np.minimum(np.ceil(uv.max(axis=0)).astype(int),[shape[1]-1,shape[0]-1])
        if x0>x1 or y0>y1: continue
        yy,xx=np.mgrid[y0:y1+1,x0:x1+1];a,b,c=uv
        denominator=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if denominator==0: continue
        w0=((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/denominator
        w1=((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/denominator
        w2=1-w0-w1
        mask[y0:y1+1,x0:x1+1] |= ((w0>=0)&(w1>=0)&(w2>=0)).astype(np.uint8)
    edges=transform(contract['approach_side_edges_body_m'])
    uv=edges @ np.asarray(K).T;edges_uv=uv[:,:,:2]/uv[:,:,2:]
    facing=float(np.dot(normal_camera,-datum_camera))
    return dict(role='MODEL_PRIOR',seed=mask!=0,triangles_uv=projected,
        approach_edges_uv=edges_uv,normal_camera=normal_camera,datum_camera_m=datum_camera,
        camera_side_signed_m=facing,
        visibility_prior='FRONT_FACING' if facing>0 else 'BACK_FACING' if facing<0 else 'EDGE_ON',
        visibility_is_observation=False)
