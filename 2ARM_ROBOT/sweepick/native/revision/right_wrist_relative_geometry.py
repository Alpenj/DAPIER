"""Current wrist rays and top planes, with cuboid/pad assumptions kept as MODEL_PRIOR."""
import itertools

import cv2
import numpy as np


def transform(value, name):
    matrix = np.asarray(value, float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1]) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3)) or
            not np.isclose(np.linalg.det(matrix[:3, :3]), 1)):
        raise ValueError(f"{name}: finite rigid 4x4 transform required")
    return matrix


def _real_optical_binding(binding):
    """Compare the physical route/mount while leaving boot-local USB enumeration to the route owner."""
    if not isinstance(binding, dict):
        raise ValueError("REAL optical binding must be a mapping")
    pairing = binding.get("physical_pairing")
    mode = binding.get("mode")
    connection = pairing.get("connection") if isinstance(pairing, dict) else None
    negotiated = mode.get("negotiated") if isinstance(mode, dict) else None
    required = (
        binding.get("ref"),
        *(pairing.get(key) if isinstance(pairing, dict) else None for key in
          ("device", "paired_arm_serial", "paired_arm_label", "evidence", "evidence_file", "binding_status")),
        connection.get("usb_sysfs") if isinstance(connection, dict) else None,
        mode.get("requested") if isinstance(mode, dict) else None,
        *(negotiated.get(key) if isinstance(negotiated, dict) else None for key in
          ("fourcc", "width", "height", "fps")),
    )
    if any(value is None or value == "" for value in required):
        raise ValueError("REAL optical binding is missing a stable route/mount/mode field")
    stable = dict(binding)
    stable_pairing = dict(pairing)
    stable_connection = dict(connection)
    stable_connection.pop("boot_id", None)
    stable_connection.pop("devnum", None)
    stable_pairing["connection"] = stable_connection
    stable_pairing.pop("previous_connection", None)
    stable["physical_pairing"] = stable_pairing
    return stable


def optical_inputs(calibration, frame, source_kind):
    if calibration is None:
        raise ValueError("RIGHT_WRIST_K_D_T_TOOL_CAMERA_MISSING")
    if calibration.get("schema") != "sweepick.wrist-optical.v1" or calibration.get("camera_key") != "right_wrist":
        raise ValueError("right wrist optical identity mismatch")
    if calibration.get("source_kind") != source_kind or frame.get("source_kind") != source_kind:
        raise ValueError("optical/input source mismatch; SIM calibration is not REAL calibration")
    calibration_binding, frame_binding = calibration.get("binding"), frame.get("binding")
    binding_matches = (_real_optical_binding(calibration_binding) == _real_optical_binding(frame_binding)
                       if source_kind == "REAL" else calibration_binding == frame_binding)
    if calibration.get("tool_frame") != "right_cube_grasp" or not binding_matches:
        raise ValueError("optical mount/mode/tool binding mismatch")
    image = np.asarray(frame["frame"])
    if list(image.shape[:2]) != calibration.get("image_size_hw"):
        raise ValueError("raw wrist resolution differs from optical calibration; no implicit resize")
    K = np.asarray(calibration["K"], float); D = np.asarray(calibration["D"], float)
    if K.shape != (3, 3) or not np.isfinite(K).all() or min(K[0, 0], K[1, 1]) <= 0 or not np.isfinite(D).all():
        raise ValueError("finite wrist K/D required")
    return K, D, transform(calibration["T_tool_camera"], "T_tool_camera")


def lift_pixels_to_plane(pixels, K, D, T_world_camera, plane_centre, plane_normal):
    pixels = np.asarray(pixels, float).reshape(-1, 2)
    if not np.isfinite(pixels).all():
        raise ValueError("finite observed pixels required")
    pose = transform(T_world_camera, "T_world_camera")
    centre = np.asarray(plane_centre, float); normal = np.asarray(plane_normal, float)
    if centre.shape != (3,) or normal.shape != (3,) or not np.isfinite(np.r_[centre, normal]).all() or np.linalg.norm(normal) == 0:
        raise ValueError("finite current plane required")
    normal = normal / np.linalg.norm(normal)
    uv = cv2.undistortPoints(pixels.reshape(-1, 1, 2), K, D).reshape(-1, 2)
    rays = np.column_stack([uv, np.ones(len(uv))]) @ pose[:3, :3].T
    denominator = rays @ normal
    if np.any(np.abs(denominator) <= np.finfo(float).eps * np.linalg.norm(rays, axis=1)):
        raise ValueError("ray/plane intersection numerically unobservable")
    distance = ((centre - pose[:3, 3]) @ normal) / denominator
    if not np.isfinite(distance).all() or np.any(distance <= 0):
        raise ValueError("observed rays do not meet the current plane in front of the camera")
    return pose[:3, 3] + distance[:, None] * rays, dict(
        minimum_abs_ray_normal_dot=float(np.min(np.abs(denominator))),
        classification="ESTIMATED_CURRENT_PIXEL_RAY_CURRENT_MEASURED_PLANE_INTERSECTION",
        physical_error_bound_m=None)


def project_points(points_world, K, D, T_world_camera):
    camera_from_world = np.linalg.inv(transform(T_world_camera, "T_world_camera"))
    points = np.asarray(points_world) @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
    if np.any(points[:, 2] <= 0):
        raise ValueError("model feature behind camera")
    return cv2.projectPoints(points, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)


def reacquire_target_component(image, seed):
    """Grow the existing target seed from current RGB, retaining only its connected component."""
    image = np.asarray(image)
    seed = np.asarray(seed, bool)
    if image.ndim != 3 or image.shape[2] < 3 or image.dtype != np.uint8 or seed.shape != image.shape[:2] or not seed.any():
        raise ValueError("CURRENT_RGB_TARGET_SEED_MALFORMED")
    grab = np.full(seed.shape, cv2.GC_PR_BGD, np.uint8)
    grab[seed] = cv2.GC_FGD
    border = np.zeros(seed.shape, bool)
    border[[0, -1], :] = True; border[:, [0, -1]] = True
    grab[border & ~seed] = cv2.GC_BGD
    cv2.grabCut(image[:, :, :3], grab, None, np.zeros((1, 65), float), np.zeros((1, 65), float),
                5, cv2.GC_INIT_WITH_MASK)
    foreground = np.isin(grab, (cv2.GC_FGD, cv2.GC_PR_FGD)).astype(np.uint8)
    count, labels = cv2.connectedComponents(foreground, 8)[:2]
    seed_labels = np.unique(labels[seed]); seed_labels = seed_labels[seed_labels != 0]
    if count < 2 or len(seed_labels) != 1:
        raise ValueError("CURRENT_RGB_TARGET_SEED_COMPONENT_AMBIGUOUS")
    return labels == seed_labels[0]


def _fragment_union_connects(a, b, edge_mask):
    """Accept a straight observed edge assembled from Canny/LSD fragments in the existing sqrt(8) neighbourhood."""
    a = np.asarray(a, float); b = np.asarray(b, float)
    pixels = np.column_stack(np.nonzero(edge_mask)[::-1]).astype(float)
    direction = b-a
    length = float(np.linalg.norm(direction))
    if not len(pixels) or length == 0:
        return False
    unit = direction/length
    along = (pixels-a) @ unit
    nearest = a + np.clip(along, 0, length)[:, None]*unit
    supported = np.sort(along[(along >= 0) & (along <= length) &
                              (np.sum((pixels-nearest)**2, axis=1) <= 8)])
    raster_neighbourhood = np.sqrt(8.0)
    return bool(len(supported) > 1 and supported[0] <= raster_neighbourhood and
                length-supported[-1] <= raster_neighbourhood and
                np.max(np.diff(supported)) <= raster_neighbourhood)


def observed_face_graph(image, mask):
    """Current RGB target contour and crease graph; no corner or physical identity is predicted."""
    from sweepick_261009_offsite_relative_features import observed_rgb_regions
    rgb = observed_rgb_regions(image)
    target = reacquire_target_component(image, mask)
    contours, _ = cv2.findContours(target.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        raise ValueError("CURRENT_RGB_REACQUIRED_TARGET_HAS_NO_EXTERIOR")
    contour = max(contours, key=cv2.contourArea)
    # sqrt(8) is the existing pixel-neighbourhood bound used by the RGB edge extractor, not a physical tolerance.
    contour_points = cv2.approxPolyDP(contour, np.sqrt(8.0), True).reshape(-1, 2).astype(float)
    if len(contour_points) < 3:
        raise ValueError("CURRENT_RGB_HAS_NO_FACE_JUNCTION_GRAPH")
    target_edge_mask = rgb["edge_mask"] & (cv2.dilate(target.astype(np.uint8), np.ones((3, 3), np.uint8)) != 0)
    internal_points = []
    corners = cv2.goodFeaturesToTrack(rgb["gray"], 0, .01, 3, mask=target.astype(np.uint8))
    for point in ([] if corners is None else corners.reshape(-1, 2)):
        if np.min(np.sum((contour_points-point)**2, axis=1)) <= 8:
            continue
        directions = []
        for segment in rgb["segments"]:
            distance = np.sum((segment-point)**2, axis=1); endpoint = int(np.argmin(distance))
            if distance[endpoint] <= 8:
                directions.append(segment[1-endpoint]-segment[endpoint])
        if len(directions) < 3 or np.linalg.matrix_rank(directions) != 2:
            continue
        branches = [other for other in contour_points
                    if _fragment_union_connects(point, other, target_edge_mask)]
        if len(branches) >= 3:
            internal_points.append(point)
    points = np.vstack([contour_points, np.asarray(internal_points, float).reshape(-1, 2)])
    contour_count = len(contour_points)
    graph_edges = []
    edge_roles = []
    neighbours = [set() for _ in points]
    boundary = {tuple(sorted((index, (index+1) % contour_count))) for index in range(contour_count)}
    for first, second in itertools.combinations(range(len(points)), 2):
        nodes = (first, second)
        boundary_edge = tuple(sorted(nodes)) in boundary
        if not boundary_edge and not _fragment_union_connects(points[first], points[second], target_edge_mask):
            continue
        graph_edges.append(nodes); neighbours[first].add(second); neighbours[second].add(first)
        edge_roles.append(dict(nodes=nodes, source=("CURRENT_RGB_REACQUIRED_MASK_EXTERIOR_POLYLINE"
                                                    if boundary_edge else "CURRENT_CANNY_MULTI_LSD_FRAGMENT_UNION")))
    exterior = [index < contour_count for index in range(len(points))]
    return points, graph_edges, neighbours, exterior, dict(
        reacquisition="CURRENT_RGB_GRABCUT_ORIGINAL_TARGET_SEED_CONNECTED_COMPONENT",
        target_seed_area_px=int(np.count_nonzero(mask)), reacquired_component_area_px=int(target.sum()),
        contour_simplification_px=float(np.sqrt(8.0)),
        junction_roles=[("CURRENT_RGB_REACQUIRED_EXTERIOR_CONTOUR_PIXEL" if value else
                         "CURRENT_RGB_GFTT_MULTI_FRAGMENT_THREE_BRANCH_JUNCTION_PIXEL") for value in exterior],
        edge_roles=edge_roles, physical_identity_verified=False)


def _face_cycles(points, neighbours, exterior_junction):
    cycles = {}
    for a in range(len(points)):
        for b in neighbours[a]:
            for c in neighbours[b]-{a}:
                for d in (neighbours[c] & neighbours[a])-{b}:
                    if len({a, b, c, d}) != 4:
                        continue
                    cycle = (a, b, c, d)
                    variants = [cycle[i:]+cycle[:i] for i in range(4)]
                    reverse = tuple(reversed(cycle)); variants += [reverse[i:]+reverse[:i] for i in range(4)]
                    key = min(variants)
                    polygon = points[list(key)]
                    edges = np.roll(polygon, -1, axis=0)-polygon
                    following = np.roll(edges, -1, axis=0)
                    cross = edges[:, 0]*following[:, 1]-edges[:, 1]*following[:, 0]
                    if not (np.all(cross > 0) or np.all(cross < 0)):
                        continue
                    if sum(exterior_junction[i] for i in key) < 2:
                        continue
                    if any(not exterior_junction[i] and len(neighbours[i]) < 3 for i in key):
                        continue
                    area = abs(float(cv2.contourArea(polygon.astype(np.float32))))
                    cycles[key] = area
    return cycles


def match_visible_face_edges(image, mask, K, D, T_world_camera, search_pose, half):
    """Associate two measured exterior edges to one searched cuboid face."""
    search = transform(search_pose, "SEARCH_PRIOR object pose")
    model_edges = []
    for along in range(3):
        across = [index for index in range(3) if index != along]
        for signs in itertools.product((-1, 1), repeat=2):
            local = np.zeros((2, 3)); local[:, along] = [-half, half]
            for axis, sign in zip(across, signs):
                local[:, axis] = sign * half
            world = local @ search[:3, :3].T + search[:3, 3]
            model_edges.append(dict(along=along, fixed=dict(zip(across, signs)),
                                    faces=list(zip(across, signs)),
                                    uv=project_points(world, K, D, T_world_camera)))
    points, graph_edges, neighbours, exterior, graph_evidence = observed_face_graph(image, mask)
    edge_sources = {tuple(role["nodes"]): role["source"] for role in graph_evidence["edge_roles"]}
    candidates = []
    for nodes in graph_edges:
        segment = points[list(nodes)]
        x, y = segment[:, 0], segment[:, 1]
        if ((np.all(x == 0) or np.all(x == mask.shape[1]-1)) or
                (np.all(y == 0) or np.all(y == mask.shape[0]-1))):
            continue
        length = np.linalg.norm(segment[1]-segment[0])
        if length == 0:
            continue
        costs = []
        for edge in model_edges:
            a, b = edge["uv"]; direction = b-a; squared = direction @ direction
            if squared == 0:
                costs.append(np.inf); continue
            fraction = np.clip((segment-a) @ direction / squared, 0, 1)
            nearest = a + fraction[:, None] * direction
            costs.append(float(np.mean(np.sum((segment-nearest)**2, axis=1))))
        order = np.argsort(costs)
        if not np.isfinite(costs[order[0]]) or np.isclose(costs[order[0]], costs[order[1]], rtol=0, atol=np.finfo(float).eps):
            continue
        selected = model_edges[int(order[0])]
        candidates.append(dict(model_edge=int(order[0]), along=selected["along"], fixed=selected["fixed"],
                               faces=selected["faces"], pixels=segment.tolist(), graph_nodes=nodes,
                               observed_edge_source=edge_sources[nodes],
                               endpoint_is_frame_clipped=[bool(px <= 0 or px >= mask.shape[1]-1 or
                                                               py <= 0 or py >= mask.shape[0]-1)
                                                          for px, py in segment],
                               association_cost_px2=costs[order[0]],
                               semantics="CURRENT_RGB_CONNECTED_EDGE_MODEL_ASSOCIATION_CANDIDATE"))
    pairs = []
    for a, b in itertools.combinations(candidates, 2):
        junction = set(a["graph_nodes"]) & set(b["graph_nodes"])
        shared = set(a["faces"]) & set(b["faces"])
        if a["along"] == b["along"] or len(shared) != 1 or len(junction) != 1:
            continue
        node = next(iter(junction))
        if not exterior[node] and len(neighbours[node]) < 3:
            continue
        directions = np.asarray([np.diff(a["pixels"], axis=0)[0], np.diff(b["pixels"], axis=0)[0]])
        if np.linalg.matrix_rank(directions) != 2:
            continue
        face_axis, face_sign = shared.pop()
        selected = [dict(a), dict(b)]
        for edge in selected:
            edge["across"] = next(axis for axis in edge["fixed"] if axis != face_axis)
            edge["side"] = edge["fixed"][edge["across"]]
        pairs.append((selected[0], selected[1], face_axis, face_sign))
    if not pairs:
        raise ValueError("NO_TWO_INDEPENDENT_CURRENT_CUBE_EDGE_CANDIDATES")
    ranked = sorted(pairs, key=lambda pair: pair[0]["association_cost_px2"] + pair[1]["association_cost_px2"])
    if len(ranked) > 1 and np.isclose(sum(e["association_cost_px2"] for e in ranked[0][:2]),
                                    sum(e["association_cost_px2"] for e in ranked[1][:2]), rtol=0, atol=np.finfo(float).eps):
        raise ValueError("COMPETING_RGB_EDGE_ASSOCIATIONS")
    a, b, axis, sign = ranked[0]
    return [a, b], dict(face_axis=axis, face_sign=sign, candidate_count=len(candidates),
                                pair_count=len(pairs), assignment_role="MODEL_PRIOR_ASSOCIATION_CANDIDATE",
                                evidence="CURRENT_RGB_REACQUIRED_CONTOUR_OR_MULTI_FRAGMENT_CONNECTED_LINES",
                                observed_graph=graph_evidence,
                                physical_edge_identity_verified=False, association_cost_is_physical_bound=False)


def pose_from_plane_edges(edges, plane, T_world_camera, K, D, search_pose, half, face):
    normal = np.asarray(plane["normal_world"], float); normal /= np.linalg.norm(normal)
    search = transform(search_pose, "SEARCH_PRIOR object pose")
    points = [lift_pixels_to_plane(e["pixels"], K, D, T_world_camera, plane["centre_world_m"], normal)[0] for e in edges]
    axis, sign = face["face_axis"], face["face_sign"]
    R = np.zeros((3, 3)); R[:, axis] = normal * sign
    first = edges[0]["along"]; line = points[0][1]-points[0][0]
    line -= normal * (line @ normal)
    if np.linalg.norm(line) <= np.finfo(float).eps:
        raise ValueError("CURRENT_EDGE_DIRECTION_UNOBSERVABLE")
    R[:, first] = line / np.linalg.norm(line)
    if R[:, first] @ search[:3, first] < 0:
        R[:, first] *= -1
    remaining = next(i for i in range(3) if i not in (axis, first))
    R[:, remaining] = np.cross(R[:, (remaining+1)%3], R[:, (remaining+2)%3])
    rows = [normal]; values = [normal @ np.asarray(plane["centre_world_m"])]
    for edge, measured in zip(edges, points):
        row = R[:, edge["across"]]
        rows.append(row); values.append(float(row @ measured.mean(axis=0)-edge["side"]*half))
    if np.linalg.matrix_rank(rows) != 3:
        raise ValueError("FACE_CENTRE_TANGENT_COMPONENTS_UNOBSERVABLE")
    face_centre = np.linalg.solve(rows, values)
    pose = np.eye(4); pose[:3, :3] = R; pose[:3, 3] = face_centre-normal*half
    diagnostics = dict(second_edge_direction_residual=float(np.linalg.norm(np.cross(
        points[1][1]-points[1][0], R[:, edges[1]["along"]]))),
        physical_error_bound_m=None, residual_is_physical_bound=False)
    return pose, points, diagnostics


def cuboid_pnp_from_current_corners(image, mask, K, D, T_world_camera, search_pose, half, evidence=None):
    """PnP from one complete current edge-connected face; mixed red surfaces may contain several faces."""
    search=transform(search_pose,"SEARCH_PRIOR object pose")
    vertices=np.asarray(list(itertools.product((-half,half),repeat=3)))
    world=vertices@search[:3,:3].T+search[:3,3]
    projected=project_points(world,K,D,T_world_camera)
    H,W=mask.shape
    evidence = {} if evidence is None else evidence
    observed, graph_edges, neighbours, exterior, graph_evidence = observed_face_graph(image, mask)
    evidence.update(classification='CURRENT_RGB_FACE_GRAPH',
        current_junction_pixels=observed.tolist(),
        current_connected_face_edges=[[observed[a].tolist(),observed[b].tolist()] for a,b in graph_edges],
        junction_roles=[dict(exterior_envelope=bool(exterior[i]),connected_edge_count=len(neighbours[i]))
                        for i in range(len(observed))],
        observed_graph=graph_evidence, physical_face_identity_verified=False)
    cycles = _face_cycles(observed, neighbours, exterior)
    evidence['observed_face_cycle_count']=len(cycles)
    if not cycles:
        raise ValueError('CURRENT_RGB_EDGES_DO_NOT_CLOSE_ONE_OBSERVED_FACE_CYCLE')
    complete_cycles={cycle:area for cycle,area in cycles.items() if not np.any(
        (observed[list(cycle),0]<=0)|(observed[list(cycle),0]>=W-1)|
        (observed[list(cycle),1]<=0)|(observed[list(cycle),1]>=H-1))}
    if not complete_cycles:
        raise ValueError('OBSERVED_FACE_CYCLES_ARE_FRAME_CLIPPED')
    ranked_cycles=sorted(complete_cycles.items(),key=lambda item:item[1],reverse=True)
    if len(ranked_cycles)>1 and np.isclose(ranked_cycles[0][1],ranked_cycles[1][1],rtol=0,atol=np.finfo(float).eps):
        raise ValueError('COMPETING_CURRENT_RGB_FACE_CYCLES')
    cycle, cycle_area = ranked_cycles[0]
    pixels=observed[list(cycle)]
    evidence.update(selected_face_cycle_pixels=pixels.tolist(),selected_face_cycle_area_px2=cycle_area,
                    selected_face_cycle_clipped=False)
    hypotheses=[]
    camera_in_object=(np.linalg.inv(search)@np.r_[T_world_camera[:3,3],1])[:3]
    index={tuple(point):i for i,point in enumerate(vertices)}
    for axis in range(3):
        other=[i for i in range(3) if i!=axis]
        for sign in (-1,1):
            if sign*camera_in_object[axis]<=half:
                continue
            ordered=[]
            for a,b in ((-1,-1),(1,-1),(1,1),(-1,1)):
                local=np.zeros(3);local[axis]=sign*half;local[other]=np.asarray([a,b])*half
                ordered.append(index[tuple(local)])
            ids=np.asarray(ordered)
            if np.any((projected[ids,0]<=0)|(projected[ids,0]>=W-1)|(projected[ids,1]<=0)|(projected[ids,1]>=H-1)):
                continue
            points=vertices[ids].astype(np.float64)
            mappings=[]
            for offset in range(4):
                mappings.append(np.roll(pixels,offset,axis=0))
                mappings.append(np.roll(pixels[::-1],offset,axis=0))
            for mapped in mappings:
                association=float(np.sum((projected[ids]-mapped)**2))
                result=cv2.solvePnPGeneric(points,mapped,K,D,flags=cv2.SOLVEPNP_SQPNP)
                for rvec,tvec in zip(result[1],result[2]):
                    pose=np.eye(4);pose[:3,:3]=cv2.Rodrigues(rvec)[0];pose[:3,3]=np.asarray(tvec).reshape(3)
                    if np.any((vertices@pose[:3,:3].T+pose[:3,3])[:,2]<=0):continue
                    local_face_centre=np.zeros(3);local_face_centre[axis]=sign*half
                    face_centre=pose[:3,:3]@local_face_centre+pose[:3,3]
                    outward=sign*pose[:3,axis]
                    if outward@(-face_centre)<=0:continue
                    fit=cv2.projectPoints(points,rvec,tvec,K,D)[0].reshape(-1,2)
                    hypotheses.append(dict(pose=T_world_camera@pose,association=association,
                        residual=float(np.sqrt(np.mean(np.sum((fit-mapped)**2,axis=1)))),pixels=mapped,
                        ids=ids,points=points,face_axis=axis,face_sign=sign))
    if not hypotheses:raise ValueError('CUBOID_PNP_HAS_NO_FINITE_FRONT_CAMERA_SOLUTION')
    hypotheses.sort(key=lambda item:(item['association'],item['residual']))
    if len(hypotheses)>1 and np.isclose(hypotheses[0]['association'],hypotheses[1]['association'],rtol=0,atol=np.finfo(float).eps):
        raise ValueError('CUBOID_PNP_COMPETING_SOLUTIONS')
    best=hypotheses[0]
    delta=np.linalg.inv(search)@best['pose']
    rotation_deg=float(np.degrees(np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1))))
    return best['pose'],dict(method='CALIBRATED_SINGLE_FACE_CUBOID_PNP',pixel_corners=best['pixels'].tolist(),
        model_vertex_ids=best['ids'].tolist(),model_vertex_coordinates_m=best['points'].tolist(),
        face_axis=best['face_axis'],face_sign=best['face_sign'],
        corner_source='CURRENT_RGB_EDGE_CONNECTED_FACE_CYCLE',association_source='MODEL_PRIOR_FACE_TOPOLOGY',
        model_prior_face_label_gauge=dict(face_axis=best['face_axis'],face_sign=best['face_sign'],
            projected_corner_association_cost_px2=best['association'],association_cost_is_physical_bound=False),
        fitted_pose_delta_from_search_prior=dict(translation_m=float(np.linalg.norm(delta[:3,3])),
            rotation_deg=rotation_deg,diagnostic_only=True),
        selected_cycle_area_px2=cycle_area,cycle_area_is_physical_bound=False,
        exterior_junction_count=sum(exterior[i] for i in cycle),
        internal_three_edge_junction_count=sum(not exterior[i] and len(neighbours[i])>=3 for i in cycle),
        predicted_or_clipped_corner_used_as_observed=False,physical_corner_identity_verified=False,
        reprojection_rms_px=best['residual'],hypotheses=len(hypotheses),residual_is_physical_bound=False)


def produce_relative_geometry(*, observation, frames, calibration, plane, search_pose, T_world_tool, half=0.02):
    """Separate model-assisted planning candidate; never changes held_object or existing ready flags."""
    frame = frames.get("right_wrist") or {}
    identity = {k: frame.get(k) for k in ("n", "recv_monotonic_s")}
    feedback = {k: observation.get(k) for k in ("feedback_stamp", "seq")}
    result = dict(status="MISSING", classification="MODEL_ASSISTED_RELATIVE", source_kind=observation.get("source_kind"),
                  camera=identity, feedback_identity=feedback, actual_q12=observation.get("q12"),
                  model_pose_available=False, model_path_ready=False, live_execution_permitted=False,
                  physical_error_bound_m=None)
    try:
        if frame.get("reader_valid") is not True or frame.get("fresh") is not True:
            raise ValueError("CURRENT_RIGHT_WRIST_FRAME_UNAVAILABLE")
        q = np.asarray(observation.get("q12"), float)
        if q.shape != (12,) or not np.isfinite(q).all():
            raise ValueError("CURRENT_ACTUAL_Q12_MALFORMED")
        K, D, T_tool_camera = optical_inputs(calibration, frame, observation["source_kind"])
        T_world_tool = transform(T_world_tool, "current actual FK T_world_tool")
        T_world_camera = T_world_tool @ T_tool_camera
        import sweepick_pick01 as p1
        surfaces = p1.wrist_object_surfaces(frame["frame"])
        if not surfaces.get("visible"):
            raise ValueError("CURRENT_WRIST_OBJECT_APPEARANCE_" + surfaces["status"])
        def independent_wrist_pose(reason, relation=None):
            result.update(top_plane_used=False, top_missing_reason=reason)
            if relation is not None:
                result['top_plane_face_relation'] = relation
            face_evidence = {}
            try:
                pose, diagnostics = cuboid_pnp_from_current_corners(
                    frame['frame'], surfaces['_mask'], K, D, T_world_camera, search_pose, half, face_evidence)
            except ValueError as exc:
                if face_evidence:
                    result['wrist_face_evidence'] = face_evidence
                raise ValueError(f"{reason}; INDEPENDENT_WRIST_PNP_UNAVAILABLE: {exc}") from exc
            result['wrist_face_evidence'] = face_evidence
            result.update(status='CANDIDATE', model_pose_available=True, object_pose_model=pose.tolist(),
                object_pose_in_receiver_tool=(np.linalg.inv(T_world_tool)@pose).tolist(),
                observed_components=['current RGB edge-connected face junctions','current RGB edges','current actual q'],
                estimated_components=['calibrated PnP pose','single cuboid face topology association'],
                model_prior_components=['40mm cuboid','pad/TCP geometry','candidate A FK mapping'],
                unobserved_components=['physical corner identity','physical relative-error bound','pad compliance'],
                source_fields=dict(object_pose_in_receiver_tool='MODEL_ASSISTED_RELATIVE',raw_pixels='CURRENT_MEASUREMENT',
                    hidden_cube_corners='MODEL_PRIOR',pad_pose='MODEL_PRIOR'),fit_diagnostics=diagnostics,
                top_plane_used=False,top_missing_reason=reason)
            return result
        if (not isinstance(plane, dict) or plane.get("status") != "CURRENT_OBJECT_PLANE" or plane.get('competing_planes')):
            return independent_wrist_pose('no unambiguous current plane; independent wrist single-face PnP candidate')
        if plane.get("feedback_identity") != feedback or plane.get("source_kind") != observation["source_kind"]:
            raise ValueError("TOP_PLANE_CURRENT_Q_SOURCE_MISMATCH")
        top = frames.get("top") or {}
        if (top.get("reader_valid") is not True or top.get("fresh") is not True or
                any((plane.get("camera") or {}).get(k) != top.get(k) for k in ("n", "recv_monotonic_s"))):
            raise ValueError("TOP_PLANE_CURRENT_FRAME_MISMATCH")
        plane = dict(plane)
        normal = np.asarray(plane["normal_world"], float)
        outward = np.asarray(plane["observer_world_m"], float)-np.asarray(plane["centre_world_m"], float)
        if not np.isfinite(np.r_[normal, outward]).all() or np.linalg.norm(normal) == 0:
            raise ValueError("CURRENT_PLANE_NORMAL_SIGN_REFERENCE_MALFORMED")
        if float(normal @ outward) == 0:
            raise ValueError("CURRENT_PLANE_NORMAL_SIGN_UNOBSERVABLE")
        if normal @ outward < 0:
            normal = -normal
        normal = normal/np.linalg.norm(normal)
        plane["normal_world"] = normal.tolist()
        search = transform(search_pose, "SEARCH_PRIOR object pose")
        local_normal = search[:3, :3].T @ normal
        plane_axis = int(np.argmax(np.abs(local_normal)))
        plane_sign = 1 if local_normal[plane_axis] > 0 else -1
        wrist_outward = T_world_camera[:3, 3]-np.asarray(plane["centre_world_m"], float)
        if not np.isfinite(wrist_outward).all() or np.linalg.norm(wrist_outward) == 0:
            raise ValueError("CURRENT_PLANE_WRIST_VIEW_DIRECTION_UNOBSERVABLE")
        wrist_view_dot = float(normal @ wrist_outward)
        relation = dict(
            classification="MODEL_ASSISTED_CURRENT_FACE_ASSOCIATION",
            plane_face_axis=plane_axis, plane_face_sign=plane_sign,
            outward_normal_dot_wrist_camera_vector_m=wrist_view_dot,
            outward_normal_dot_wrist_camera_unit_vector=float(wrist_view_dot/np.linalg.norm(wrist_outward)),
            plane_normal_sign="toward this current top plane's observing camera",
            same_face_confirmed=False, physical_face_identity_verified=False)
        if wrist_view_dot <= 0:
            relation['reason'] = 'top-observer outward face is back-facing from the current wrist camera'
            return independent_wrist_pose(relation['reason'], relation)
        try:
            edges, face = match_visible_face_edges(frame["frame"], surfaces["_mask"], K, D,
                                                   T_world_camera, search_pose, half)
        except ValueError as exc:
            relation['reason'] = 'current exterior edges do not identify one wrist-visible cube face: ' + str(exc)
            return independent_wrist_pose(relation['reason'], relation)
        relation.update(wrist_edge_face_axis=face['face_axis'], wrist_edge_face_sign=face['face_sign'])
        if (face['face_axis'], face['face_sign']) != (plane_axis, plane_sign):
            relation['reason'] = 'current wrist exterior edge topology identifies a different cube face'
            return independent_wrist_pose(relation['reason'], relation)
        relation.update(same_face_confirmed=True,
                        reason='top-plane face and current wrist exterior two-edge topology share one model-associated face')
        pose, points, diagnostics = pose_from_plane_edges(edges, plane, T_world_camera, K, D, search_pose, half, face)
        result.update(status="CANDIDATE", model_pose_available=True, object_pose_model=pose.tolist(),
                      object_pose_in_receiver_tool=(np.linalg.inv(T_world_tool) @ pose).tolist(),
                      observed_components=["current top depth plane", "current wrist RGB line pixels", "current actual q"],
                      estimated_components=["wrist-ray/plane edge intersection", "cuboid edge association", "face tangent/centre"],
                      model_prior_components=["40mm cuboid", "pad/TCP geometry", "candidate A FK mapping"],
                      unobserved_components=["physical relative-error bound", "pad compliance", "physical edge/occlusion identity"],
                      edge_candidates=edges, points_world=[p.tolist() for p in points], face_association=face,
                      fit_diagnostics=diagnostics, optical_source=dict(schema=calibration["schema"], binding=calibration["binding"]),
                      top_plane_used=True, top_plane_face_relation=relation,
                      plane_normal_sign="toward this current top plane's observing camera",
                      source_fields=dict(object_pose_in_receiver_tool="MODEL_ASSISTED_RELATIVE",
                                         raw_plane="CURRENT_MEASUREMENT", raw_pixels="CURRENT_MEASUREMENT",
                                         hidden_cube_corners="MODEL_PRIOR", pad_pose="MODEL_PRIOR"))
    except (ValueError, KeyError, cv2.error) as exc:
        result["missing_reason"] = str(exc)
    return result


def pad_projection(model, data, contracts, K, D, T_world_camera):
    """Actual-q CAD projection; a hidden pad remains a model prediction."""
    output = []
    for index, contract in contracts.items():
        body = contract["body_id"]
        world = contract["boundary_edges_body_m"] @ data.xmat[body].reshape(3, 3).T + data.xpos[body]
        row = dict(pad=index, source="MODEL_PRIOR_ACTUAL_Q_CAD", boundary_world_m=world.tolist(),
                   currently_measured=False, physical_identity_verified=False)
        if "datum_point_body_m" in contract:
            R = data.xmat[body].reshape(3, 3)
            row.update(datum_world_m=(R @ contract["datum_point_body_m"] + data.xpos[body]).tolist(),
                       normal_world=(R @ contract["normal_body"]).tolist(),
                       source_visual_geom=contract["source_visual_geom"],
                       source_collision_geom=contract["source_collision_geom"])
        try:
            row.update(projection_status="PROJECTED_MODEL_PRIOR",
                       pixels=project_points(world.reshape(-1, 3), K, D, T_world_camera).reshape(-1, 2, 2).tolist())
        except ValueError as exc:
            row.update(projection_status="NOT_PROJECTABLE", pixels=None, reason=str(exc))
        output.append(row)
    return output
