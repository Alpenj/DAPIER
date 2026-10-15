"""Refine the held-face perception mask against current robot mesh shape; collision paths are unchanged."""
import numpy as np


def prepare_robot_mask_geometry(model):
    """Extract immutable mesh/topology inputs; transforms and mask decisions remain per observation."""
    import mujoco
    import sweepick_pick01 as p1
    records = []
    for geom, body in p1.robot_boxes(model, None):
        record = dict(geom=int(geom), body=body, type=int(model.geom_type[geom]),
                      local_aabb=np.asarray(model.geom_aabb[geom], float))
        if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[geom]
            vertex_start, vertex_count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            face_start, face_count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
            vertices = np.asarray(model.mesh_vert[vertex_start:vertex_start + vertex_count], float)
            faces = np.asarray(model.mesh_face[face_start:face_start + face_count], int)
            record.update(vertices=vertices, faces=faces,
                          triangles=tuple(vertices[faces[:, i]] for i in range(3)))
        records.append(record)
    return records

def point_triangle_distance(P: np.ndarray, A: np.ndarray, B: np.ndarray, C: np.ndarray, point_chunk: int = 512) -> np.ndarray:
    """Return each point's minimum Euclidean distance to a triangle surface.

    The implementation is vectorized over all triangles and chunks only over
    points.  Vertex, edge, and interior barycentric candidates are evaluated;
    degenerate triangles safely fall back to their edge/vertex candidates.
    """
    P = np.asarray(P, float)
    out = np.full(len(P), np.inf, dtype=float)
    AB, AC, BC = B - A, C - A, C - B
    d00 = np.einsum("ij,ij->i", AB, AB)
    d01 = np.einsum("ij,ij->i", AB, AC)
    d11 = np.einsum("ij,ij->i", AC, AC)
    for start in range(0, len(P), point_chunk):
        X = P[start:start + point_chunk, None, :]
        AP = X - A
        BP = X - B
        CP = X - C
        best = np.minimum(np.sum(AP * AP, axis=-1), np.sum(BP * BP, axis=-1))
        best = np.minimum(best, np.sum(CP * CP, axis=-1))

        def edge_candidate(U, V, W):
            UW = W - U
            denom = np.einsum("ij,ij->i", UW, UW)
            t = np.divide(np.sum((X - U) * UW, axis=-1), denom[None, :],
                          out=np.zeros((len(X), len(U))), where=denom[None, :] > 0)
            t = np.clip(t, 0.0, 1.0)
            Q = U[None, :, :] + t[..., None] * UW[None, :, :]
            return np.sum((X - Q) * (X - Q), axis=-1)

        best = np.minimum(best, edge_candidate(A, B, B))
        best = np.minimum(best, edge_candidate(A, C, C))
        best = np.minimum(best, edge_candidate(B, C, C))

        d20 = np.sum((X - A) * AB[None, :, :], axis=-1)
        d21 = np.sum((X - A) * AC[None, :, :], axis=-1)
        denom = d00 * d11 - d01 * d01
        v = np.divide(d11[None, :] * d20 - d01[None, :] * d21,
                      denom[None, :], out=np.zeros_like(d20), where=denom[None, :] > 1e-20)
        w = np.divide(d00[None, :] * d21 - d01[None, :] * d20,
                      denom[None, :], out=np.zeros_like(d20), where=denom[None, :] > 1e-20)
        u = 1.0 - v - w
        interior = (denom[None, :] > 1e-20) & (u >= 0.0) & (v >= 0.0) & (w >= 0.0)
        Q = u[..., None] * A[None, :, :] + v[..., None] * B[None, :, :] + w[..., None] * C[None, :, :]
        interior_distance = np.sum((X - Q) * (X - Q), axis=-1)
        best = np.where(interior, interior_distance, best)
        out[start:start + len(X)] = np.sqrt(np.min(best, axis=1))
    return out

def ray_inside_axis(P: np.ndarray, A: np.ndarray, B: np.ndarray, C: np.ndarray,
                    direction: np.ndarray, point_chunk: int = 512) -> np.ndarray:
    """Moller-Trumbore odd/even parity for a closed triangle mesh."""
    inside = np.zeros(len(P), dtype=bool)
    direction = np.asarray(direction, float)
    E1, E2 = B - A, C - A
    H = np.cross(np.broadcast_to(direction, E2.shape), E2)
    det = np.einsum("ij,ij->i", E1, H)
    usable = np.abs(det) > 1e-12
    inv_det = np.zeros_like(det)
    inv_det[usable] = 1.0 / det[usable]
    for start in range(0, len(P), point_chunk):
        X = P[start:start + point_chunk, None, :]
        S = X - A[None, :, :]
        uu = inv_det[None, :] * np.einsum("pj,cpj->cp", H, S)
        Q = np.cross(S, E1[None, :, :])
        vv = inv_det[None, :] * np.einsum("pj,cpj->cp", np.broadcast_to(direction, E1.shape), Q)
        tt = inv_det[None, :] * np.einsum("pj,cpj->cp", E2, Q)
        hit = (usable[None, :] & (uu >= -1e-10) & (vv >= -1e-10) &
               (uu + vv <= 1.0 + 1e-10) & (tt > 1e-10))
        intersections = np.sort(np.where(hit, tt, np.inf), axis=1)
        unique = np.isfinite(intersections)
        if intersections.shape[1] > 1:
            previous, current = intersections[:, :-1], intersections[:, 1:]
            difference = np.full(current.shape, np.inf)
            finite = np.isfinite(previous) & np.isfinite(current)
            np.subtract(current, previous, out=difference, where=finite)
            # A shared mesh edge intersects the same surface twice; float roundoff is not a second crossing.
            roundoff = 32 * np.finfo(float).eps * np.maximum(1.0, np.where(finite, np.abs(current), 0.0))
            unique[:, 1:] &= difference > roundoff
        inside[start:start + len(X)] = (np.count_nonzero(unique, axis=1) % 2) == 1
    return inside

def closed_mesh_membership(local_points_to_test: np.ndarray, vertices: np.ndarray, faces: np.ndarray):
    """Return majority-of-three-axis parity and disagreement diagnostics."""
    if len(local_points_to_test) == 0:
        return np.zeros(0, bool), np.zeros(0, np.uint8)
    A, B, C = vertices[faces[:, 0]], vertices[faces[:, 1]], vertices[faces[:, 2]]
    axis_results = np.column_stack([
        ray_inside_axis(local_points_to_test, A, B, C, np.array([1.0, 0.0, 0.0])),
        ray_inside_axis(local_points_to_test, A, B, C, np.array([0.0, 1.0, 0.0])),
        ray_inside_axis(local_points_to_test, A, B, C, np.array([0.0, 0.0, 1.0])),
    ])
    return np.count_nonzero(axis_results, axis=1) >= 2, np.count_nonzero(axis_results, axis=1).astype(np.uint8)


def observed_robot_mask(model, data, points, search_mask, margin, diagnostics=None, prepared=None):
    """Use AABB broadphase and the same margin on mesh surfaces in the current held search.

    Outside the held search, the existing conservative AABB exclusion remains. Non-mesh
    shapes also retain that exclusion. No collision checker uses this perception mask.
    """
    import mujoco
    points, search = np.asarray(points, float), np.asarray(search_mask, bool)
    if search.shape != (len(points),) or not np.isfinite(points).all() or not np.isfinite(margin) or margin < 0:
        raise ValueError('finite points, matching search mask and nonnegative margin required')
    removed = np.zeros(len(points), bool)
    records = []
    geometry = prepared
    if geometry is None:
        geometry = prepare_robot_mask_geometry(model)
    for static in geometry:
        geom, body = static['geom'], static['body']
        rotation, origin = data.geom_xmat[geom].reshape(3, 3), data.geom_xpos[geom]
        aabb = static['local_aabb']
        local = (points - origin) @ rotation
        broad = (np.abs(local - aabb[:3]) <= aabb[3:] + margin).all(axis=1)
        hit = broad.copy()
        indices = np.flatnonzero(broad & search)
        if len(indices) and static['type'] == mujoco.mjtGeom.mjGEOM_MESH:
            vertices, faces = static['vertices'], static['faces']
            a, b, c = static['triangles']
            surface = point_triangle_distance(local[indices], a, b, c) <= margin
            interior = np.zeros(len(indices), bool)
            interior[~surface], _votes = closed_mesh_membership(local[indices][~surface], vertices, faces)
            hit[indices] = surface | interior
        first = hit & search & ~removed
        removed |= hit
        if len(indices):
            T = np.eye(4); T[:3, :3], T[:3, 3] = rotation, origin
            records.append(dict(name=model.geom(geom).name, body=body, geom_id=int(geom),
                type=static['type'], T_world_geom=T.tolist(), local_aabb=aabb.tolist(), margin_m=float(margin),
                broad_search_points=len(indices), shape_search_points=int((hit & search).sum()),
                first_removed_points=int(first.sum()), _broad_search=broad & search,
                _shape_search=hit & search, _first_removed=first))
    if diagnostics is not None:
        diagnostics.update(role='held perception only; current mesh shape + unchanged margin; AABB outside search',
            geoms=records, removed_search_points=int((removed & search).sum()))
    return removed
