"""Read-only SIM diagnostics; independent geometry never authorizes control."""
import mujoco
import numpy as np

from collision_guard import minimum_protected_clearance, protected_geom_pairs


def describe_pair(model, data, first, second, required_clearance_m):
    details = []
    minimum_protected_clearance(model, data, [(first, second)],
        distance_cap_m=required_clearance_m, diagnostics=details)
    result = details[0]
    result["required_clearance_m"] = required_clearance_m
    result["time_s"] = float(data.time)
    result["geoms"] = []
    for g in (first, second):
        body = int(model.geom_bodyid[g])
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, data.geom_xmat[g])
        mesh = int(model.geom_dataid[g])
        chain = []
        ancestor = body
        while ancestor:
            chain.append(dict(body=model.body(ancestor).name,
                              joint_count=int(model.body_jntnum[ancestor]),
                              joints=[dict(name=model.joint(j).name,
                                  world_axis=data.xaxis[j].tolist())
                                  for j in range(int(model.body_jntadr[ancestor]),
                                      int(model.body_jntadr[ancestor]+model.body_jntnum[ancestor]))]))
            ancestor = int(model.body_parentid[ancestor])
        result["geoms"].append(dict(id=g, name=model.geom(g).name or None,
            body_name=model.body(body).name,
            type=mujoco.mjtGeom(int(model.geom_type[g])).name,
            mesh_id=mesh if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH else None,
            mesh_asset=(model.mesh(mesh).name
                        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH else None),
            world_position_m=data.geom_xpos[g].tolist(),
            world_quaternion_wxyz=quat.tolist(),
            world_rotation_matrix=data.geom_xmat[g].reshape(3, 3).tolist(),
            size=model.geom_size[g].tolist(),
            contype=int(model.geom_contype[g]), conaffinity=int(model.geom_conaffinity[g]),
            body_weldid=int(model.body_weldid[body]), ancestor_chain=chain))
    result["protected_pair"] = (first, second) in protected_geom_pairs(model)
    if (model.names.startswith(b"desk_learning_OS30A_UNVERIFIED\x00")
            and model.geom(second).name == "table"
            and model.body(int(model.geom_bodyid[first])).name.startswith(("left_", "right_"))
            and model.geom_contype[first] and model.geom_conaffinity[first]):
        result["protected_pair_reason"] = (
            "collision_guard.protected_geom_pairs: integration desk branch explicitly "
            "adds every active left_/right_ arm geom x table obstacle. "
            "No mounting-interface exception; table belongs to world body.")
    else:
        result["protected_pair_reason"] = "See protected_geom_pairs; no desk arm/table rule matched."
    result["pair_contacts"] = [dict(distance_m=float(c.dist))
        for c in data.contact if {int(c.geom1), int(c.geom2)} == {first, second}]

    # Specific independent mesh/box-top witness, not a new runtime distance bound.
    # If a lowest mesh vertex projects inside the box top, its vertical gap is
    # both a separation lower bound and an attained upper bound (exact distance).
    if (model.geom_type[first] == mujoco.mjtGeom.mjGEOM_MESH
            and model.geom_type[second] == mujoco.mjtGeom.mjGEOM_BOX):
        mesh = int(model.geom_dataid[first])
        start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
        raw = model.mesh_vert[start:start+count].astype(float)
        graph_start = int(model.mesh_graphadr[mesh])
        hull = raw
        if graph_start >= 0 and count >= 10:
            graph = model.mesh_graph[graph_start:]
            n = int(graph[0])
            hull = raw[graph[2+n:2+2*n]]
        rotation = data.geom_xmat[second].reshape(3, 3)
        evidence = dict(axis_world=rotation[:, 2].tolist(),
                        box_top_projection_m=float(data.geom_xpos[second] @ rotation[:, 2]
                                                   + model.geom_size[second, 2]))
        for label, vertices in (("compiled_raw", raw), ("compiled_hull", hull)):
            world = vertices @ data.geom_xmat[first].reshape(3, 3).T + data.geom_xpos[first]
            local = (world-data.geom_xpos[second]) @ rotation
            k = int(np.argmin(local[:, 2]))
            gap = float(local[k, 2] - model.geom_size[second, 2])
            inside = bool(np.all(np.abs(local[k, :2]) <= model.geom_size[second, :2]))
            top = local[k].copy()
            top[2] = model.geom_size[second, 2]
            evidence[label] = dict(vertex_count=len(vertices), gap_m=gap,
                lowest_vertex_world_m=world[k].tolist(),
                box_top_witness_world_m=(top @ rotation.T + data.geom_xpos[second]).tolist(),
                projection_inside_box_top=inside,
                exact_positive_distance_m=gap if inside and gap > 0 else None)
        result["independent_geometry"] = evidence
        distance = evidence["compiled_hull"]["exact_positive_distance_m"]
        if distance is not None:
            result["physical_clearance_class"] = (
                "B" if distance < required_clearance_m else "clearance_satisfied")
            result["distance_computation_class"] = (
                "C" if abs(result["native_signed_distance_m"] - distance) > 1e-6 else "consistent")
            result["classification"] = ("C" if result["distance_computation_class"] == "C"
                                        else result["physical_clearance_class"])
            result["interpretation"] = (
                "Native/API mismatch and physical clearance are separate: "
                "independent distance does NOT replace guard distance or authorize motion.")
    return result
