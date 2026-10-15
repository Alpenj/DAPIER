#!/usr/bin/env python3
"""Depth-based surface perception for the TJJ task manager: where an object stands on a known surface, whether a region
of that surface is empty, and whether an object was placed (resting in a region with the gripper away from it).

Inputs are what the top camera gives: a metric depth image, its intrinsics K and the camera pose T_world_camera_optical
(+X right, +Y down, +Z forward), plus joint feedback for the gripper. No simulator state, segmentation or object pose is
read. With the SIM camera the geometry is exact by construction; with the real OS30A the K / T / depth registration must be
verified first, so every result carries `calibrated=False` until that is done (SIM_ONLY / REAL_MODALITY_UNVERIFIED).
"""
from __future__ import annotations

import numpy as np

from tjj_task_manager import AreaObservation, ObjectObservation, PlaceVerification, Validity


def back_project(depth_m, K, T_world_cam, min_depth=0.05, max_depth=2.5):
    depth = np.asarray(depth_m, dtype=np.float64)
    valid = np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
    v, u = np.nonzero(valid)
    z = depth[v, u]
    pts = np.column_stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])
    return pts @ np.asarray(T_world_cam)[:3, :3].T + np.asarray(T_world_cam)[:3, 3]


def clusters_on_surface(points_world, surface_z, *, min_height=0.008, max_height=0.12, roi=None, cell=0.005, min_points=6):
    """Points standing between min_height and max_height above the surface, grouped by 8-connected cells of an XY grid."""
    p = np.asarray(points_world)
    keep = (p[:, 2] >= surface_z + min_height) & (p[:, 2] <= surface_z + max_height)
    if roi is not None:
        (x0, x1), (y0, y1) = roi
        keep &= (p[:, 0] >= x0) & (p[:, 0] <= x1) & (p[:, 1] >= y0) & (p[:, 1] <= y1)
    p = p[keep]
    if not len(p):
        return []
    ij = np.floor(p[:, :2] / cell).astype(np.int64)
    cells = {}
    for n, key in enumerate(map(tuple, ij)):
        cells.setdefault(key, []).append(n)
    seen, out = set(), []
    for start in cells:
        if start in seen:
            continue
        stack, members = [start], []
        seen.add(start)
        while stack:
            c = stack.pop()
            members += cells[c]
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (c[0] + di, c[1] + dj)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        if len(members) < min_points:
            continue
        q = p[members]
        top = q[q[:, 2] >= q[:, 2].max() - 0.004]                         # the visible top face
        out.append(dict(centroid_xyz=[float(top[:, 0].mean()), float(top[:, 1].mean()), float(q[:, 2].max())], points=int(len(q)),
                        extent_xy_m=[float(np.ptp(q[:, 0])), float(np.ptp(q[:, 1]))], top_height_m=float(q[:, 2].max() - surface_z)))
    return sorted(out, key=lambda c: -c["points"])


def validity(frame_id, stamp_s, ok, reason="", calibrated=False):
    return Validity(frame_id=frame_id, stamp_s=float(stamp_s), valid=bool(ok), source="top depth + K + T_world_camera", reason=reason, calibrated=calibrated)


def observe_object(depth_m, K, T, stamp_s, *, surface_z, roi, frame_id="world", max_extent_m=0.09, calibrated=False, object_id="object-0"):
    """The largest cluster in the ROI that is small enough to be a graspable object (an arm in view is longer than that)."""
    cs = [c for c in clusters_on_surface(back_project(depth_m, K, T), surface_z, roi=roi) if max(c["extent_xy_m"]) <= max_extent_m]
    if not cs:
        return None, cs
    c = cs[0]
    return ObjectObservation(object_id, tuple(c["centroid_xyz"]), tuple(c["extent_xy_m"]), validity(frame_id, stamp_s, True, calibrated=calibrated)), cs


def unseen_on_surface(depth_m, K, T, region, min_depth=0.05, max_depth=2.5):
    """Pixels WITHOUT a depth value whose viewing ray meets the region's surface plane inside the region. Such a pixel
    is not an observation of an empty surface: whatever lies along that ray was not measured."""
    depth = np.asarray(depth_m, dtype=np.float64)
    v, u = np.nonzero(~(np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)))
    if not len(u):
        return 0
    T = np.asarray(T, dtype=np.float64)
    rays = np.column_stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones(len(u))]) @ T[:3, :3].T
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (region.surface_z_m - T[2, 3]) / rays[:, 2]
    hit = T[:3, 3] + rays * s[:, None]
    return int((np.isfinite(s) & (s > 0) & (np.hypot(hit[:, 0] - region.center_xy[0], hit[:, 1] - region.center_xy[1]) <= region.radius_m)).sum())


def observe_area(depth_m, K, T, stamp_s, region, *, calibrated=False, min_height=0.008, unseen_is_unknown=False):
    """unseen_is_unknown (closeout, for a sparse real depth): a region with pixels that carry no depth is NOT reported
    clear and its observation is not valid ("not observed" is not "empty"). Off by default: the SIM depth is dense and
    the existing SIM behaviour is unchanged."""
    pts = back_project(depth_m, K, T)
    d = np.hypot(pts[:, 0] - region.center_xy[0], pts[:, 1] - region.center_xy[1])
    inside = pts[(d <= region.radius_m)]
    seen = len(inside)
    above = int((inside[:, 2] >= region.surface_z_m + min_height).sum()) if seen else 0
    ok = seen > 0                                                            # the region must be in view at all
    if unseen_is_unknown and above == 0:
        unseen = unseen_on_surface(depth_m, K, T, region)
        if unseen:
            return AreaObservation(region.region_id, clear=False, points_above_surface=above,
                                   validity=validity(region.frame_id, stamp_s, False, f"{unseen} pixels of the region carry no depth ({seen} do): not observed is not empty", calibrated))
    return AreaObservation(region.region_id, clear=ok and above == 0, points_above_surface=above,
                           validity=validity(region.frame_id, stamp_s, ok, "" if ok else "region not in the depth image", calibrated))


def verify_place(depth_m, K, T, stamp_s, region, *, gripper_opening, gripper_tcp_xyz, opening_released=0.6, tcp_clear_m=0.03, object_top_height_range=(0.015, 0.08),
                 calibrated=False):
    """PLACE from what can be sensed: an object-sized cluster rests in the target region (top depth), the gripper is open
    (joint feedback) and its tool point is away from that object (forward kinematics of the measured joints)."""
    # The clusters are formed in a window wider than the region by an object size: a window cut at the region's edge would
    # crop an object lying across it and pull its centroid inside (seen in R3: 13 mm, a false 'in region').
    grow = region.radius_m + 0.05
    cs = [c for c in clusters_on_surface(back_project(depth_m, K, T), region.surface_z_m, roi=((region.center_xy[0] - grow, region.center_xy[0] + grow),
                                                                                             (region.center_xy[1] - grow, region.center_xy[1] + grow)))
          if max(c["extent_xy_m"]) <= 0.09]
    cs = [c for c in cs if np.hypot(c["centroid_xyz"][0] - region.center_xy[0], c["centroid_xyz"][1] - region.center_xy[1]) <= region.radius_m]
    pts = back_project(depth_m, K, T)
    seen = int((np.hypot(pts[:, 0] - region.center_xy[0], pts[:, 1] - region.center_xy[1]) <= region.radius_m).sum())
    in_view = seen > 0                                                       # a target the top camera does not see cannot be verified by it
    in_region = bool(cs)
    supported = in_region and object_top_height_range[0] <= cs[0]["top_height_m"] <= object_top_height_range[1]
    distance = None if not in_region else float(np.linalg.norm(np.asarray(gripper_tcp_xyz) - np.asarray(cs[0]["centroid_xyz"])))
    separated = bool(in_region and gripper_opening >= opening_released and distance >= tcp_clear_m)
    return PlaceVerification(object_in_region=in_region, object_supported=bool(supported), gripper_separated=separated,
                             evidence=dict(clusters_in_region=cs[:3], region_points_in_view=seen, gripper_opening=float(gripper_opening), tcp_to_object_m=distance,
                                           parameters=dict(opening_released=opening_released, tcp_clear_m=tcp_clear_m, object_top_height_range=list(object_top_height_range),
                                                           note="first-demo values for the 40 mm block of the SIM scene; to be set from the real object and sensor error, not tuned to pass")),
                             validity=validity(region.frame_id, stamp_s, in_view, "" if in_view else "target region not in the depth image", calibrated))


# --------------------------------------------------------------------------------------------------
# v2 object selection: table plane removal + workspace ROI + robot self exclusion + a robust top face
# --------------------------------------------------------------------------------------------------

def robust_top(points, surface_z, bin_m=0.002, min_share=0.15, band_m=0.003):
    """Top face of a cluster: the highest height bin that holds a real share of the points (single high pixels at an edge
    do not count). Returns (top height above the surface, top-face points)."""
    z = points[:, 2] - surface_z
    bins = np.floor(z / bin_m).astype(np.int64)
    values, counts = np.unique(bins, return_counts=True)
    ok = values[counts >= max(3, min_share * len(z))]
    top = (float(ok.max()) + 0.5) * bin_m if len(ok) else float(np.median(z))
    face = points[np.abs(z - top) <= band_m]
    return top, (face if len(face) >= 3 else points)


def select_object(depth_m, K, T, *, surface_z, roi, robot_mask=None, min_height=0.008, max_height=0.12, cell=0.005, min_points=12, max_extent_m=0.09):
    """Object candidates on the surface from the top depth. `robot_mask(points) -> bool array` marks points that belong to
    the robot itself (kinematic self model); those are removed before clustering. Returns (best or None, candidates, counts)."""
    pts = back_project(depth_m, K, T)
    above = (pts[:, 2] >= surface_z + min_height) & (pts[:, 2] <= surface_z + max_height + 0.05)
    (x0, x1), (y0, y1) = roi
    inside = (pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1)
    keep = above & inside
    robot = np.zeros(len(pts), dtype=bool) if robot_mask is None else np.asarray(robot_mask(pts), dtype=bool)
    p = pts[keep & ~robot]
    counts = dict(points=int(len(pts)), above_surface_in_roi=int(keep.sum()), removed_as_robot=int((keep & robot).sum()), left=int(len(p)))
    if not len(p):
        return None, [], counts
    ij = np.floor(p[:, :2] / cell).astype(np.int64)
    cells = {}
    for n, key in enumerate(map(tuple, ij)):
        cells.setdefault(key, []).append(n)
    seen, out = set(), []
    for start in cells:
        if start in seen:
            continue
        stack, members = [start], []
        seen.add(start)
        while stack:
            c = stack.pop()
            members += cells[c]
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (c[0] + di, c[1] + dj)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        if len(members) < min_points:
            continue
        q = p[members]
        top, face = robust_top(q, surface_z)
        extent = [float(np.ptp(q[:, 0])), float(np.ptp(q[:, 1]))]
        out.append(dict(centroid_xyz=[float(face[:, 0].mean()), float(face[:, 1].mean()), float(surface_z + top)], points=int(len(q)), top_points=int(len(face)), extent_xy_m=extent,
                        top_height_m=float(top), truncated=bool(top >= max_height), object_sized=bool(max(extent) <= max_extent_m and top < max_height)))
    out.sort(key=lambda c: (not c["object_sized"], -c["points"]))
    best = out[0] if out and out[0]["object_sized"] else None
    return best, out, counts


# --------------------------------------------------------------------------------------------------
# R3: orientation of an object on the surface, a box held between two jaws, and the object colour in a wrist image
# --------------------------------------------------------------------------------------------------

def top_face_yaw(depth_m, K, T, centroid_xyz, *, surface_z, robot_mask=None, reach_m=0.045, band_m=0.004):
    """Yaw (mod 90 deg) of a box-like object from the points of its top face (minimum-area rectangle). Returns
    (yaw rad or None, points used)."""
    import cv2
    pts = back_project(depth_m, K, T)
    near = (np.hypot(pts[:, 0] - centroid_xyz[0], pts[:, 1] - centroid_xyz[1]) <= reach_m) & (np.abs(pts[:, 2] - centroid_xyz[2]) <= band_m)
    if robot_mask is not None:
        near &= ~np.asarray(robot_mask(pts), dtype=bool)
    face = pts[near]
    if len(face) < 12:
        return None, int(len(face))
    (_, _), (_, _), angle = cv2.minAreaRect(face[:, :2].astype(np.float32))
    yaw = np.deg2rad(angle)
    return float((yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4), int(len(face))


def box_sdf(points, half):
    q = np.abs(points) - half
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)


def fit_held_box(points, site_pos, site_rot, half, prior_t=None, *, view=None, reach_m=0.03, step_m=0.003, step_deg=3.0, tolerance_m=0.0004):
    """A box pinched between two parallel jaws has three free parameters in the jaw frame: the rotation about the jaw axis
    (site Z) and the position in the plane of the pads; along the jaw axis it is centred. Grid search of those three.
    Cost = how far the depth points (not the robot) are from the box surface + (with `view` = (depth_m, K, T)) how much of
    the box surface that faces the camera has no depth point on it although nothing is in front of it. Among the fits
    within `tolerance_m` of the best cost the one nearest to `prior_t` is returned; the spread of those fits is the
    ambiguity. Returns None with fewer than 30 points."""
    if len(points) < 30:
        return None
    loc = (np.asarray(points) - site_pos) @ site_rot
    grid = np.arange(-reach_m, reach_m + 1e-9, step_m)
    u = np.linspace(-0.8, 0.8, 5)
    faces = []                                                               # sample points of the six faces, box frame, with their outward normals
    for axis in range(3):
        for sign in (-1.0, 1.0):
            a, b = [i for i in range(3) if i != axis]
            pts = np.zeros((25, 3))
            pts[:, axis] = sign * half[axis]
            pts[:, a], pts[:, b] = [g.ravel() * half[i] for g, i in zip(np.meshgrid(u, u), (a, b))]
            n = np.zeros(3)
            n[axis] = sign
            faces.append((pts, n))
    if view is not None:
        depth, K, T = view
        Tinv = np.linalg.inv(T)
        cam = T[:3, 3]
        hgt, wid = depth.shape
        world = back_project(depth, K, T).reshape(hgt, wid, 3)
    rows = []
    for th in np.deg2rad(np.arange(-45.0, 45.0, step_deg)):
        c, s = np.cos(th), np.sin(th)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        pr = loc @ R
        Rw = site_rot @ R                                                    # box axes in the world
        for tx in grid:
            for ty in grid:
                t = np.array([tx, ty, 0.0])
                cost = float(np.minimum(np.abs(box_sdf(pr - R.T @ t, half)), 0.01).mean())
                if view is not None:
                    centre = site_pos + site_rot @ t
                    seen = missing = 0
                    for pts, n in faces:
                        if (Rw @ n) @ (cam - centre) <= 0:                   # this face looks away from the camera
                            continue
                        w = centre + pts @ Rw.T
                        pc = (w - T[:3, 3]) @ Tinv[:3, :3].T
                        px = np.round(K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]).astype(int)
                        py = np.round(K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]).astype(int)
                        ok = (px >= 0) & (px < wid) & (py >= 0) & (py < hgt)
                        m_ = world[py[ok], px[ok]]
                        d = np.linalg.norm(m_ - w[ok], axis=1)
                        in_front = np.linalg.norm(m_ - cam, axis=1) < np.linalg.norm(w[ok] - cam, axis=1) - 0.008
                        seen += int((~in_front).sum())
                        missing += int(((d > 0.008) & ~in_front).sum())
                    cost += 0.004 * (missing / seen if seen >= 10 else 1.0)
                rows.append((cost, float(th), float(tx), float(ty)))
    best = min(r[0] for r in rows)
    good = [r for r in rows if r[0] <= best + tolerance_m]
    ref = np.zeros(2) if prior_t is None else np.asarray(prior_t)[:2]
    cost, th, tx, ty = min(good, key=lambda r: (np.hypot(r[2] - ref[0], r[3] - ref[1]), r[0])) if view is None else min(good)
    c, s = np.cos(th), np.sin(th)
    return dict(t=[tx, ty, 0.0], R=[[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], theta_deg=float(np.degrees(th)), cost_mm=1000 * cost, best_cost_mm=1000 * best, points=int(len(points)),
                ambiguity_mm=[1000 * float(np.ptp([r[2] for r in good])), 1000 * float(np.ptp([r[3] for r in good]))])


def red_fraction(rgb):
    """Share of the pixels of an RGB image that have the colour of the task object ("red block" in the task's object
    hint), and their centroid in pixels."""
    r, g, b = [rgb[..., i].astype(float) for i in range(3)]
    mask = (r > 90) & (r > 1.6 * g) & (r > 1.6 * b)
    ys, xs = np.nonzero(mask)
    return float(mask.mean()), (None if not len(xs) else (float(xs.mean()), float(ys.mean())))
