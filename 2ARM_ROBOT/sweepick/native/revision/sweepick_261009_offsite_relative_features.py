"""File-only right-inner-pad geometry from marked registered RGB-D patches."""

from __future__ import annotations

import cv2
import numpy as np


CLASSIFICATION = "CAMERA_DERIVED_RELATIVE_ESTIMATE"


def _missing(source_kind, source_image, timestamps, reason):
    return dict(classification=CLASSIFICATION, status="MISSING", source_kind=source_kind,
                source_image=source_image, timestamps=timestamps, missing_reason=reason,
                independent_physical_truth=False, fit_residual_is_physical_bound=False,
                input_depth_units="mm", features_complete=False, reference=None, ready=False,
                runtime_convention=dict(name="UNKNOWN", transform=None))


def _patch(depth, K, polygon, scale):
    poly = np.asarray(polygon, float)
    if poly.ndim != 2 or poly.shape[0] < 3 or poly.shape[1] != 2 or not np.isfinite(poly).all():
        return None
    pixel_poly = np.rint(poly * scale).astype(np.int32)
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillPoly(mask, [pixel_poly], 1)
    valid = (mask != 0) & np.isfinite(depth) & (depth > 0)
    v, u = np.nonzero(valid)
    if len(u) < 3:
        return None
    z = depth[v, u] / 1000.0
    points = np.column_stack([(u - K[0, 2]) * z / K[0, 0],
                              (v - K[1, 2]) * z / K[1, 1], z])
    centre = points.mean(0)
    _u, singular, vh = np.linalg.svd(points - centre, full_matrices=False)
    if len(singular) < 3 or singular[1] <= np.finfo(float).eps * max(singular[0], 1.0) * len(points):
        return None
    normal = vh[-1] / np.linalg.norm(vh[-1])
    residual = (points - centre) @ normal
    return dict(centre=centre, normal=normal, points=len(points), missing_pixels=int(mask.sum() - valid.sum()),
                singular=singular, rms=float(np.sqrt(np.mean(residual ** 2))),
                p95=float(np.percentile(np.abs(residual), 95)))


def _plane_ray(K, uv, scale, plane):
    pixel = np.asarray(uv, float) * scale
    ray = np.linalg.inv(K) @ np.r_[pixel, 1.0]
    denominator = float(np.dot(plane["normal"], ray))
    if abs(denominator) <= np.finfo(float).eps:
        return None
    distance = float(np.dot(plane["normal"], plane["centre"]) / denominator)
    return None if not np.isfinite(distance) or distance <= 0 else ray * distance


def visible_face_relation(depth_mm, K, image_shape, pad_patch_polygon, object_face_polygon, ordered_pad_edge_uv, *,
                          source_kind, source_image, timestamps):
    """Observe one visible inner-pad/visible object face; no hidden face, volume pose or runtime datum is inferred."""
    if source_kind not in ("REAL", "SYNTHETIC"):
        raise ValueError("source_kind must preserve REAL/SYNTHETIC provenance")
    depth = np.asarray(depth_mm, float)
    K = np.asarray(K, float).reshape(3, 3).copy()
    if depth.ndim != 2 or len(image_shape) < 2 or min(image_shape[:2]) <= 0 or not np.isfinite(K).all() or min(K[0, 0], K[1, 1]) <= 0:
        raise ValueError("registered depth, source image dimensions and finite intrinsics are required")
    scale = np.array([depth.shape[1] / image_shape[1], depth.shape[0] / image_shape[0]], float)
    K[0] *= scale[0]; K[1] *= scale[1]
    pad, obj = [_patch(depth, K, polygon, scale) for polygon in (pad_patch_polygon, object_face_polygon)]
    if pad is None or obj is None:
        return _missing(source_kind, source_image, timestamps, "a visible pad/object face has no identifiable registered-depth plane")
    edge = np.asarray(ordered_pad_edge_uv, float)
    if edge.shape != (2, 2) or not np.isfinite(edge).all():
        return _missing(source_kind, source_image, timestamps, "the actual visible pad edge direction is missing")
    edge_points = [_plane_ray(K, uv, scale, pad) for uv in edge]
    corners = [_plane_ray(K, uv, scale, obj) for uv in object_face_polygon]
    pad_boundary = [_plane_ray(K, uv, scale, pad) for uv in pad_patch_polygon]
    if any(p is None for p in edge_points + corners + pad_boundary):
        return _missing(source_kind, source_image, timestamps, "an observed feature ray does not meet its face plane")
    n = pad["normal"].copy()
    if np.dot(n, -pad["centre"]) < 0:
        n *= -1  # a visible INNER face points toward the observing camera; never toward the object by construction
    tangent = edge_points[1] - edge_points[0]
    tangent -= n * np.dot(n, tangent)
    if np.linalg.norm(tangent) <= np.finfo(float).eps:
        return _missing(source_kind, source_image, timestamps, "the observed edge does not identify a pad tangent")
    tangent /= np.linalg.norm(tangent)
    second = np.cross(n, tangent)
    relative = np.asarray(corners) - pad["centre"]
    gap = relative @ n
    overlap = []
    for axis in (tangent, second):
        interval_pad = (np.asarray(pad_boundary) - pad["centre"]) @ axis
        interval_object = relative @ axis
        overlap.append(dict(pad_observed_interval_m=[float(interval_pad.min()), float(interval_pad.max())],
                            object_observed_interval_m=[float(interval_object.min()), float(interval_object.max())],
                            signed_overlap_m=float(min(interval_pad.max(), interval_object.max()) - max(interval_pad.min(), interval_object.min()))))
    return dict(classification="CAMERA_DERIVED_VISIBLE_FACE_RELATION", status="OK", source_kind=source_kind,
                source_image=source_image, timestamps=timestamps, input_depth_units="mm",
                pad_plane_normal_camera=n.tolist(), normal_sign="toward camera viewing the identified inner face; requires actual INNER-face identity",
                observed_object_face_signed_gap_m=[float(gap.min()), float(gap.max())],
                tangent_overlap=overlap, relation_scope="only marked visible face/edge regions; not whole-object clearance or opposite-pad gap",
                raw_marks=dict(pad_patch_polygon=pad_patch_polygon, object_face_polygon=object_face_polygon, ordered_pad_edge_uv=edge.tolist()),
                fit_diagnostics=[dict(plane_rms_residual_m=p["rms"], plane_p95_abs_residual_m=p["p95"]) for p in (pad, obj)],
                fit_residual_is_physical_bound=False, independent_physical_truth=False,
                whole_object_minimum_gap_m=None, opposite_inner_pad_gap_m=None, T_runtime_object=None,
                reference=None, ready=False)


def observed_rgb_regions(color):
    """Extract current RGB edges/segments without assigning a target or physical identity."""
    image = np.asarray(color)
    if (image.ndim not in (2, 3) or not image.shape[0] or not image.shape[1]
            or (image.ndim == 3 and image.shape[2] < 3) or image.dtype != np.uint8):
        raise ValueError("an 8-bit current RGB image is required")
    gray = image if image.ndim == 2 else cv2.cvtColor(image[:, :, :3], cv2.COLOR_BGR2GRAY)
    level = float(np.median(gray))
    edges = cv2.Canny(gray, int(max(0.0, 0.66 * level)), int(max(1.0, min(255.0, 1.33 * level))))
    detected = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD).detect(gray)[0]
    segments = [] if detected is None else [line.reshape(2, 2).astype(float) for line in detected]
    segment_mask = np.zeros(gray.shape, np.uint8)
    for start, end in segments:
        cv2.line(segment_mask, tuple(np.rint(start).astype(int)), tuple(np.rint(end).astype(int)), 1, 1)
    # Join only detector endpoints that are already within the same small raster neighbourhood.  A spatial hash keeps
    # the exact distance test while avoiding the quadratic Python loop on full camera frames.
    endpoints = np.asarray([point for segment in segments for point in segment], float)
    if len(endpoints):
        bucket_width = np.sqrt(8.0)
        buckets = {}
        for index, point in enumerate(endpoints):
            bucket = tuple(np.floor(point / bucket_width).astype(int))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for other in buckets.get((bucket[0] + dx, bucket[1] + dy), ()):
                        if np.dot(point - endpoints[other], point - endpoints[other]) <= 8.0:
                            cv2.line(segment_mask, tuple(np.rint(point).astype(int)),
                                     tuple(np.rint(endpoints[other]).astype(int)), 1, 1)
            buckets.setdefault(bucket, []).append(index)
    return dict(classification="CURRENT_RGB_FEATURES_UNIDENTIFIED", gray=gray,
                edge_mask=(edges != 0) | (segment_mask != 0), canny_edge_mask=edges != 0,
                segments=segments, physical_identity="UNKNOWN")


def _fit_plane(points, minimum_points):
    keep = np.ones(len(points), bool)
    cutoff = None
    for _ in range(3):
        face = points[keep]
        if len(face) < minimum_points:
            return None
        centre = face.mean(0)
        _unused, singular, vh = np.linalg.svd(face - centre, full_matrices=False)
        if len(singular) < 3 or singular[1] <= np.finfo(float).eps * max(singular[0], 1.0) * len(face):
            return None
        residual = np.abs((points - centre) @ vh[-1])
        median = float(np.median(residual[keep]))
        mad = float(np.median(np.abs(residual[keep] - median)))
        cutoff = max(median + 3.0 * 1.4826 * mad,
                     np.finfo(float).eps * max(float(np.linalg.norm(centre)), 1.0))
        new_keep = residual <= cutoff
        if new_keep.sum() < minimum_points or np.array_equal(new_keep, keep):
            break
        keep = new_keep
    face = points[keep]
    centre = face.mean(0)
    _unused, singular, vh = np.linalg.svd(face - centre, full_matrices=False)
    return dict(centre=centre, normal=vh[-1] / np.linalg.norm(vh[-1]), singular=singular,
                cutoff=cutoff, keep=keep, residual=(face - centre) @ vh[-1])


def _plane_sse(points):
    centre = points.mean(axis=0)
    _unused, singular, vh = np.linalg.svd(points - centre, full_matrices=False)
    if len(singular) < 3 or singular[1] <= np.finfo(float).eps * max(singular[0], 1.0) * len(points):
        return None
    residual = (points - centre) @ vh[-1]
    return float(np.dot(residual, residual))


def _has_competing_depth_planes(points, camera_depth, pixel_uv, minimum_points):
    return competing_depth_surface_partition(points, camera_depth, pixel_uv, minimum_points) is not None


def competing_depth_surface_partition(points, camera_depth, pixel_uv, minimum_points):
    """Return the existing evidence-supported depth partition, or None; no new threshold."""
    z = np.asarray(camera_depth, float)
    if len(points) < 2 * minimum_points or not np.isfinite(z).all() or float(z.min()) == float(z.max()):
        return None
    centres = np.array([z.min(), z.max()], float)
    labels = None
    while True:
        updated = np.argmin(np.abs(z[:, None] - centres[None, :]), axis=1)
        if labels is not None and np.array_equal(updated, labels):
            break
        labels = updated
        counts = np.bincount(labels, minlength=2)
        if np.any(counts < minimum_points):
            return None
        centres = np.array([z[labels == group].mean() for group in range(2)])
    # Depth clustering alone splits ordinary zero-mean sensor noise into low/high halves.  Require a spatial depth
    # discontinuity whose cross-cluster neighbour jumps exceed this frame's robust within-surface variation.
    locations = {tuple(pixel): index for index, pixel in enumerate(np.asarray(pixel_uv, int))}
    within = []
    across = []
    for index, (u, v) in enumerate(np.asarray(pixel_uv, int)):
        for neighbour in ((u + 1, v), (u, v + 1)):
            other = locations.get(neighbour)
            if other is None:
                continue
            target = within if labels[index] == labels[other] else across
            target.append(abs(float(z[index] - z[other])))
    if not within or not across:
        return None
    within = np.asarray(within, float)
    median = float(np.median(within))
    mad = float(np.median(np.abs(within - median)))
    discontinuity_floor = median + 3.0 * 1.4826 * mad
    if min(across) <= discontinuity_floor:
        return None
    whole_sse = _plane_sse(points)
    split_sse_parts = [_plane_sse(points[labels == group]) for group in range(2)]
    if whole_sse is None or any(value is None for value in split_sse_parts):
        return None
    split_sse = float(sum(split_sse_parts))
    scale = max(float(np.square(points).sum()), 1.0)
    floor = np.finfo(float).eps * scale
    samples = len(points)
    one_bic = samples * np.log(max(whole_sse, floor) / samples) + 3.0 * np.log(samples)
    two_bic = samples * np.log(max(split_sse, floor) / samples) + 7.0 * np.log(samples)
    return labels if two_bic < one_bic else None


def _components_share_plane(first, second):
    """Compare RGB-component plane models whose grouping was chosen independently of depth noise."""
    whole_sse = _plane_sse(np.vstack([first, second]))
    first_sse = _plane_sse(first)
    second_sse = _plane_sse(second)
    if whole_sse is None or first_sse is None or second_sse is None:
        return False
    samples = len(first) + len(second)
    scale = max(float(np.square(np.vstack([first, second])).sum()), 1.0)
    floor = np.finfo(float).eps * scale
    one_bic = samples * np.log(max(whole_sse, floor) / samples) + 3.0 * np.log(samples)
    two_bic = samples * np.log(max(first_sse + second_sse, floor) / samples) + 6.0 * np.log(samples)
    return bool(one_bic <= two_bic)


def _longest_true_run(values):
    best = None
    start = None
    for index, value in enumerate(np.r_[values, False]):
        if value and start is None:
            start = index
        elif not value and start is not None:
            if best is None or index - start > best[1] - best[0]:
                best = (start, index)
            start = None
    return best


def _plane_points_at_uv(uv, K, transform, plane):
    """Intersect depth-domain camera pixels with a fitted plane in the transform's output frame."""
    pixels = np.asarray(uv, float).reshape(-1, 2)
    rays_camera = np.column_stack([(pixels[:, 0] - K[0, 2]) / K[0, 0],
                                   (pixels[:, 1] - K[1, 2]) / K[1, 1], np.ones(len(pixels))])
    rays = rays_camera @ transform[:3, :3].T
    denominator = rays @ plane["normal"]
    usable = np.abs(denominator) > np.finfo(float).eps
    distance = np.full(len(pixels), np.nan, float)
    distance[usable] = ((plane["centre"] - transform[:3, 3]) @ plane["normal"]) / denominator[usable]
    usable &= np.isfinite(distance) & (distance > 0)
    points = transform[:3, 3] + rays * distance[:, None]
    return points, usable, distance


def _plane_polygon_centroid(points, normal):
    """Area centroid of one ordered observed plane contour; does not invent fitted corners."""
    polygon = np.asarray(points, float)
    if polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 3:
        return None
    origin = polygon[0]
    first = polygon[1:] - origin
    lengths = np.linalg.norm(first, axis=1)
    if not np.any(lengths > np.finfo(float).eps):
        return None
    x_axis = first[int(np.argmax(lengths))] / lengths.max()
    x_axis -= normal * np.dot(normal, x_axis)
    length = np.linalg.norm(x_axis)
    if length <= np.finfo(float).eps:
        return None
    x_axis /= length
    y_axis = np.cross(normal, x_axis)
    coordinates = np.column_stack([(polygon - origin) @ x_axis, (polygon - origin) @ y_axis])
    following = np.roll(coordinates, -1, axis=0)
    cross = coordinates[:, 0] * following[:, 1] - following[:, 0] * coordinates[:, 1]
    area2 = float(cross.sum())
    if abs(area2) <= np.finfo(float).eps:
        return None
    centre2 = ((coordinates + following) * cross[:, None]).sum(axis=0) / (3.0 * area2)
    return origin + centre2[0] * x_axis + centre2[1] * y_axis


def observed_face_from_seed(depth_mm, K_depth, color, seed, *, image_shape=None, xyz_image=None,
                            allowed_mask=None, minimum_points=3, T_output_camera=None, rgb_features=None,
                            diagnostics=None):
    """Reacquire one visible RGB-D face from a projected seed; the seed never becomes its boundary."""
    if diagnostics is not None and not isinstance(diagnostics, dict):
        raise ValueError("diagnostics must be a mutable dict")
    diagnostic = diagnostics if diagnostics is not None else {}
    diagnostic.update(status="MISSING", missing_reason=None, minimum_points=int(minimum_points))
    diagnostic["_masks"] = {}
    def unavailable(reason):
        diagnostic.update(status="MISSING", missing_reason=reason)
        return None, reason
    depth = np.asarray(depth_mm, float)
    K = np.asarray(K_depth, float).reshape(3, 3)
    if depth.ndim != 2 or not np.isfinite(K).all() or min(K[0, 0], K[1, 1]) <= 0:
        raise ValueError("registered depth and finite depth-domain intrinsics are required")
    image = np.asarray(color)
    resized = image if image.shape[:2] == depth.shape else cv2.resize(
        image, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_AREA)
    rgb = observed_rgb_regions(resized) if rgb_features is None else rgb_features
    if (not isinstance(rgb, dict) or np.asarray(rgb.get("edge_mask")).shape != depth.shape
            or np.asarray(rgb.get("gray")).shape != depth.shape or not isinstance(rgb.get("segments"), list)):
        raise ValueError("rgb_features must describe this registered-depth pixel domain")
    raw_valid = np.isfinite(depth) & (depth > 0)
    valid = raw_valid.copy()
    if allowed_mask is not None:
        allowed = np.asarray(allowed_mask, bool)
        if allowed.shape != depth.shape:
            raise ValueError("allowed_mask must use the registered-depth pixel domain")
        valid &= allowed
    diagnostic.update(raw_valid_depth_pixels=int(np.count_nonzero(raw_valid)),
                      allowed_valid_depth_pixels=int(np.count_nonzero(valid)))
    diagnostic["_masks"].update(raw_valid=raw_valid, allowed_valid=valid.copy())

    source_shape = tuple(image_shape or image.shape[:2])
    scale = np.array([depth.shape[1] / source_shape[1], depth.shape[0] / source_shape[0]], float)
    if isinstance(seed, np.ndarray) and seed.dtype == bool and seed.shape == depth.shape:
        seed_mask = seed.copy()
    else:
        polygon = np.asarray(seed, float)
        if polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2 or not np.isfinite(polygon).all():
            return unavailable("candidate seed is malformed")
        seed_mask = np.zeros(depth.shape, np.uint8)
        cv2.fillPoly(seed_mask, [np.rint(polygon * scale).astype(np.int32)], 1)
        seed_mask = seed_mask != 0
    seed_valid = seed_mask & valid
    diagnostic.update(seed_pixels=int(np.count_nonzero(seed_mask)),
                      seed_valid_depth_pixels=int(np.count_nonzero(seed_valid)))
    diagnostic["_masks"].update(seed=seed_mask, seed_valid=seed_valid.copy())
    if np.count_nonzero(seed_valid) < minimum_points:
        return unavailable("candidate seed has too few current registered-depth pixels")

    if xyz_image is None:
        v, u = np.indices(depth.shape)
        z = depth / 1000.0
        xyz = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], axis=-1)
    else:
        xyz = np.asarray(xyz_image, float)
        if xyz.shape != (*depth.shape, 3):
            raise ValueError("xyz_image must be HxWx3 in the requested output frame")
    valid &= np.isfinite(xyz).all(axis=2)
    seed_valid &= valid
    diagnostic.update(xyz_valid_depth_pixels=int(np.count_nonzero(valid)),
                      seed_valid_xyz_pixels=int(np.count_nonzero(seed_valid)))
    diagnostic["_masks"].update(xyz_valid=valid.copy(), seed_valid=seed_valid.copy())

    # RGB defines the 2-D observed region.  Canny commonly produces nested inner/outer contours for one edge, so
    # filled edge contours are not regions.  Select the unique non-edge component that contains current seed depth.
    # The selected component may be open at an occlusion or image boundary; those boundaries are never fabricated as
    # RGB edges below.
    open_region = ~rgb["edge_mask"]
    count, labels = cv2.connectedComponents(open_region.astype(np.uint8), connectivity=4)
    candidates = [label for label in range(1, count)
                  if np.count_nonzero(seed_valid & (labels == label)) >= minimum_points]
    diagnostic.update(rgb_components=int(count - 1), seed_supported_rgb_components=len(candidates))
    diagnostic["_masks"]["rgb_edge"] = np.asarray(rgb["edge_mask"], bool)
    if not candidates:
        return unavailable("candidate seed has no coherent current RGB region with enough registered depth")
    seed_v, seed_u = np.nonzero(seed_valid)
    seed_centre = np.array([seed_u.mean(), seed_v.mean()])
    distances = []
    for label in candidates:
        label_v, label_u = np.nonzero(seed_valid & (labels == label))
        distance2 = np.square(label_u - seed_centre[0]) + np.square(label_v - seed_centre[1])
        distances.append(float(distance2.min()))
    closest = min(distances)
    selected = [label for label, distance in zip(candidates, distances) if distance == closest]
    seed_surface = seed_valid & np.isin(labels, selected)
    diagnostic.update(seed_selected_rgb_pixels=int(np.count_nonzero(seed_surface)))
    diagnostic["_masks"]["seed_selected_rgb"] = seed_surface.copy()
    surface_v, surface_u = np.nonzero(seed_surface)
    surface_uv = np.column_stack([surface_u, surface_v])
    if _has_competing_depth_planes(xyz[seed_surface], depth[seed_surface] / 1000.0, surface_uv, minimum_points):
        return unavailable("candidate seed's RGB region contains competing depth-supported planes; face is ambiguous")
    # A colour/occlusion boundary can split one physical plane into several RGB components.  Merge only components
    # whose combined seed depth still selects one plane; a different-depth background or second face stays excluded.
    for label in candidates:
        if label in selected:
            continue
        current = seed_valid & np.isin(labels, selected)
        candidate = seed_valid & (labels == label)
        if _components_share_plane(xyz[current], xyz[candidate]):
            selected.append(label)
    candidates = selected
    selected_labels = np.isin(labels, selected)
    seed_surface = seed_valid & selected_labels
    diagnostic.update(selected_rgb_components=len(selected),
                      seed_selected_rgb_pixels=int(np.count_nonzero(seed_surface)))
    diagnostic["_masks"].update(selected_rgb_region=selected_labels, seed_selected_rgb=seed_surface.copy())
    surface_v, surface_u = np.nonzero(seed_surface)
    surface_uv = np.column_stack([surface_u, surface_v])
    if _has_competing_depth_planes(xyz[seed_surface], depth[seed_surface] / 1000.0, surface_uv, minimum_points):
        return unavailable("candidate seed's RGB region contains competing depth-supported planes; face is ambiguous")
    fit = _fit_plane(xyz[seed_surface], minimum_points)
    if fit is None:
        return unavailable("candidate seed does not identify a non-collinear face plane")
    region = selected_labels
    # Include observed edge pixels adjacent to the selected region.  This preserves sparse samples that happen to lie
    # on a rasterised RGB edge without filling missing depth or growing the face into another RGB region.
    adjacent_edge = rgb["edge_mask"] & (cv2.dilate(region.astype(np.uint8), np.ones((3, 3), np.uint8)) != 0)
    region_support = region | adjacent_edge
    observed_rgb_region_support = region_support.copy()
    residual = np.full(depth.shape, np.inf, float)
    residual[valid] = np.abs((xyz[valid] - fit["centre"]) @ fit["normal"])
    plane = valid & (residual <= fit["cutoff"])
    mask = plane & region_support
    _count, support_labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    linked_labels = np.unique(support_labels[mask & seed_surface])
    linked_labels = linked_labels[linked_labels != 0]
    linked_support = np.isin(support_labels, linked_labels)
    unsupported_support = mask & ~linked_support
    mask &= linked_support
    diagnostic.update(seed_linked_plane_pixels=int(np.count_nonzero(mask)),
                      unsupported_same_plane_pixels=int(np.count_nonzero(unsupported_support)))
    diagnostic["_masks"].update(fitted_plane=plane, seed_linked_plane_support=mask.copy(),
                                unsupported_same_plane_support=unsupported_support)
    if np.count_nonzero(mask) < minimum_points:
        return unavailable("the reacquired current face has too few seed-linked depth pixels")
    fit = _fit_plane(xyz[mask], minimum_points)
    if fit is None:
        return unavailable("the reacquired current face is not a non-collinear plane")
    residual[valid] = np.abs((xyz[valid] - fit["centre"]) @ fit["normal"])
    plane = valid & (residual <= fit["cutoff"])

    # A shared RGB component or a coincident plane elsewhere in the image does not establish object identity.
    # Only the plane support connected to current seed depth may contribute a face boundary or orientation.
    region = mask & ~rgb["edge_mask"]
    adjacent_edge = rgb["edge_mask"] & (cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8)) != 0)
    region_support = mask | adjacent_edge

    # Keep only contour samples backed by a current RGB edge.  In particular, an open component's image border is
    # not reported as a face corner or edge.
    contours, _hierarchy = cv2.findContours(region_support.astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    external, _hierarchy = cv2.findContours(observed_rgb_region_support.astype(np.uint8), cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_NONE)
    region_contour = max(external, key=cv2.contourArea).reshape(-1, 2).astype(float) if external else np.empty((0, 2))
    near_rgb_edge = cv2.dilate(rgb["edge_mask"].astype(np.uint8), np.ones((3, 3), np.uint8)) != 0
    contour_runs = []
    for candidate in contours:
        points = candidate.reshape(-1, 2)
        supported = near_rgb_edge[points[:, 1], points[:, 0]]
        if np.any(supported):
            contour_runs.append(points[supported].astype(float))
    contour = max(contour_runs, key=len) if contour_runs else np.empty((0, 2))

    region_boundary = adjacent_edge
    supported_boundary = cv2.dilate(region_boundary.astype(np.uint8), np.ones((3, 3), np.uint8)) != 0
    T = np.eye(4) if T_output_camera is None else np.asarray(T_output_camera, float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError("T_output_camera must be a finite 4x4 transform")
    grid_v, grid_u = np.indices(depth.shape)
    grid_uv = np.column_stack([grid_u.ravel(), grid_v.ravel()])
    _grid_points, grid_usable, predicted_depth = _plane_points_at_uv(grid_uv, K, T, fit)
    grid_usable = grid_usable.reshape(depth.shape)
    predicted_depth = predicted_depth.reshape(depth.shape)
    same_plane = plane
    nearer_than_face = valid & grid_usable & ~same_plane & (depth / 1000.0 < predicted_depth)
    farther_than_face = valid & grid_usable & ~same_plane & (depth / 1000.0 > predicted_depth)
    outside_region = ~np.isin(labels, candidates) & (labels != 0)
    visible_edges = []
    rejected_edges = []
    for segment in rgb["segments"]:
        start, end = segment
        samples = max(2, int(np.ceil(np.max(np.abs(end - start)))) + 1)
        line = np.linspace(start, end, samples)
        pixel = np.rint(line).astype(int)
        inside = ((pixel[:, 0] >= 0) & (pixel[:, 0] < depth.shape[1]) &
                  (pixel[:, 1] >= 0) & (pixel[:, 1] < depth.shape[0]))
        support = np.zeros(samples, bool)
        support[inside] = supported_boundary[pixel[inside, 1], pixel[inside, 0]]
        run = _longest_true_run(support)
        if run is None or run[1] - run[0] < 3:
            continue
        supported_uv = pixel[run[0]:run[1]]
        line_mask = np.zeros(depth.shape, np.uint8)
        cv2.line(line_mask, tuple(supported_uv[0]), tuple(supported_uv[-1]), 1, 1)
        near_line = cv2.dilate(line_mask, np.ones((5, 5), np.uint8)) != 0
        exterior = outside_region & near_line
        counts = dict(coplanar=int(np.count_nonzero(same_plane & exterior)),
                      nearer=int(np.count_nonzero(nearer_than_face & exterior)),
                      farther=int(np.count_nonzero(farther_than_face & exterior)))
        reason = None
        if counts["nearer"]:
            reason = "CLOSER_FOREGROUND_OCCLUSION"
        elif counts["farther"] <= counts["coplanar"]:
            reason = "COPLANAR_RGB_DIVIDER" if counts["coplanar"] else "MISSING_EXTERIOR_DEPTH"
        elif not counts["farther"]:
            reason = "MISSING_EXTERIOR_DEPTH"
        if reason is not None:
            rejected_edges.append(dict(uv=supported_uv[[0, -1]].astype(float), reason=reason,
                                       outside_depth_pixels=counts))
            continue
        endpoints, usable, _distance = _plane_points_at_uv(supported_uv[[0, -1]], K, T, fit)
        if not np.all(usable):
            continue
        direction = endpoints[1] - endpoints[0]
        length = float(np.linalg.norm(direction))
        if length <= np.finfo(float).eps:
            continue
        direction /= length
        midpoint_uv = supported_uv[[0, -1]].mean(axis=0)
        pixel_direction = supported_uv[-1] - supported_uv[0]
        pixel_normal = np.array([-pixel_direction[1], pixel_direction[0]], float)
        pixel_normal_length = np.linalg.norm(pixel_normal)
        inward = None
        if pixel_normal_length > np.finfo(float).eps:
            pixel_normal /= pixel_normal_length
            region_v, region_u = np.nonzero(region & near_line)
            side = (np.column_stack([region_u, region_v]) - midpoint_uv) @ pixel_normal
            side = side[np.abs(side) > np.finfo(float).eps]
            if len(side) and abs(float(side.sum())) > np.finfo(float).eps:
                inward_uv = midpoint_uv + np.sign(side.sum()) * pixel_normal
                inward_points, inward_usable, _distance = _plane_points_at_uv(
                    np.vstack([midpoint_uv, inward_uv]), K, T, fit)
                if np.all(inward_usable):
                    inward = inward_points[1] - inward_points[0]
                    inward -= direction * np.dot(inward, direction)
                    inward_length = np.linalg.norm(inward)
                    inward = None if inward_length <= np.finfo(float).eps else inward / inward_length
        visible_edges.append(dict(uv=supported_uv[[0, -1]].astype(float), points=endpoints,
                                  direction=direction, inward_direction=inward, length_m=length,
                                  pixels=int(len(supported_uv)), outside_depth_pixels=counts,
                                  boundary_evidence="FARTHER_DEPTH_OUTSIDE"))

    pairs = []
    for first in range(len(visible_edges)):
        for second in range(first + 1, len(visible_edges)):
            dot = abs(float(np.dot(visible_edges[first]["direction"], visible_edges[second]["direction"])))
            if dot <= 0.25 and visible_edges[first]["inward_direction"] is not None and visible_edges[second]["inward_direction"] is not None:
                pairs.append((dot, -min(visible_edges[first]["length_m"], visible_edges[second]["length_m"]), first, second))
    face_edges = []
    ordered = None
    if pairs:
        _dot, _length, first, second = min(pairs)
        face_edges = [visible_edges[first], visible_edges[second]]
    elif visible_edges:
        face_edges = [max(visible_edges, key=lambda edge: edge["length_m"])]
    if face_edges:
        ordered = max(face_edges, key=lambda edge: edge["length_m"])
    axes = [edge["direction"] for edge in face_edges]
    unclassified = (visible_edges if not axes else
                    [edge for edge in visible_edges
                     if max(abs(float(np.dot(edge["direction"], axis))) for axis in axes) < 0.9])
    image_boundary_touch = bool(len(region_contour) and
                                ((region_contour[:, 0] <= 1).any() or (region_contour[:, 0] >= depth.shape[1] - 2).any()
                                 or (region_contour[:, 1] <= 1).any() or (region_contour[:, 1] >= depth.shape[0] - 2).any()))
    # A full visible outline needs farther measured depth outside every actual contour sample.  Missing exterior depth,
    # a coplanar colour split, and a closer foreground aperture all leave the face centre/outline incomplete.
    complete_boundary = False
    visible_face_centroid = None
    if len(region_contour) >= 3 and not image_boundary_touch:
        kernel = np.ones((5, 5), np.uint8)
        far_near = cv2.boxFilter((farther_than_face & outside_region).astype(np.uint8), cv2.CV_32S,
                                 (5, 5), normalize=False, borderType=cv2.BORDER_CONSTANT)
        same_near = cv2.boxFilter((same_plane & outside_region).astype(np.uint8), cv2.CV_32S,
                                  (5, 5), normalize=False, borderType=cv2.BORDER_CONSTANT)
        closer_near = cv2.boxFilter((nearer_than_face & outside_region).astype(np.uint8), cv2.CV_32S,
                                    (5, 5), normalize=False, borderType=cv2.BORDER_CONSTANT)
        contour_index = region_contour.astype(int)
        complete_boundary = bool(np.all(near_rgb_edge[contour_index[:, 1], contour_index[:, 0]])
                                 and np.all(far_near[contour_index[:, 1], contour_index[:, 0]] > 0)
                                 and far_near[contour_index[:, 1], contour_index[:, 0]].sum()
                                     > same_near[contour_index[:, 1], contour_index[:, 0]].sum()
                                 and not np.any(closer_near[contour_index[:, 1], contour_index[:, 0]])
                                 and not rejected_edges)
        if complete_boundary:
            contour_points, usable, _distance = _plane_points_at_uv(region_contour, K, T, fit)
            if np.all(usable):
                visible_face_centroid = _plane_polygon_centroid(contour_points, fit["normal"])
            complete_boundary = visible_face_centroid is not None
    if complete_boundary:
        contour = region_contour
    visibility = ("VISIBLE_FACE_BOUNDARY" if complete_boundary else
                  "PARTIAL_OCCLUDED" if image_boundary_touch or rejected_edges else "PARTIAL_VISIBLE_BOUNDARY")
    output_scale = np.array([1.0 / scale[0], 1.0 / scale[1]])
    polygon = (contour * output_scale).tolist()
    ordered_uv = None if ordered is None else (ordered["uv"] * output_scale).tolist()
    pixel_provenance = dict(seed_valid_depth_pixels=int(np.count_nonzero(seed_valid)),
                            seed_selected_rgb_pixels=int(np.count_nonzero(seed_surface)),
                            seed_linked_plane_pixels=int(np.count_nonzero(mask)),
                            unsupported_same_plane_pixels=int(np.count_nonzero(unsupported_support)),
                            linkage="seed depth -> current RGB component -> fitted plane -> connected plane support -> edge")
    diagnostic_masks = dict(seed_valid=seed_valid, selected_rgb_region=selected_labels,
                            seed_linked_plane_support=mask,
                            unsupported_same_plane_support=unsupported_support,
                            selected_edge_support=adjacent_edge)
    diagnostic.update(status="OK", missing_reason=None, pixel_provenance=pixel_provenance,
                      orientation_identified=ordered is not None,
                      observed_exterior_edges=len(visible_edges), selected_face_edges=len(face_edges))
    diagnostic["_masks"].update(diagnostic_masks)
    def public_edge(edge):
        return dict(uv=(edge["uv"] * output_scale).tolist(), points=edge["points"].tolist(),
                    direction=edge["direction"].tolist(),
                    inward_direction=None if edge["inward_direction"] is None else edge["inward_direction"].tolist(),
                    length_m=edge["length_m"], pixels=edge["pixels"],
                    outside_depth_pixels=edge["outside_depth_pixels"], boundary_evidence=edge["boundary_evidence"])
    return dict(
        polygon=polygon,
        ordered_edge=ordered_uv,
        observed_exterior_edges=[public_edge(edge) for edge in visible_edges],
        visible_edges=[public_edge(edge) for edge in face_edges],
        rejected_boundary_edges=[dict(uv=(edge["uv"] * output_scale).tolist(), reason=edge["reason"],
                                      outside_depth_pixels=edge["outside_depth_pixels"])
                                 for edge in rejected_edges],
        visibility=visibility,
        complete_face_boundary=complete_boundary,
        visible_face_centroid=None if visible_face_centroid is None else visible_face_centroid.tolist(),
        visible_face_centroid_classification=("OBSERVED_FULL_RGB_DEPTH_BOUNDARY" if complete_boundary else "UNIDENTIFIED"),
        orientation_identified=ordered is not None,
        edge_missing_reason=None if ordered is not None else "no actual RGB/depth exterior edge identifies an in-plane face axis",
        occlusion=dict(image_boundary_touch=image_boundary_touch,
                       unclassified_exterior_segments=len(unclassified) + len(rejected_edges),
                       closer_foreground_segments=sum(edge["reason"] == "CLOSER_FOREGROUND_OCCLUSION" for edge in rejected_edges),
                       coplanar_divider_segments=sum(edge["reason"] == "COPLANAR_RGB_DIVIDER" for edge in rejected_edges),
                       missing_exterior_depth_segments=sum(edge["reason"] == "MISSING_EXTERIOR_DEPTH" for edge in rejected_edges)),
        depth_pixels=int(np.count_nonzero(mask)),
        rgb_edge_pixels=int(sum(edge["pixels"] for edge in face_edges)),
        observed_exterior_edge_pixels=int(sum(edge["pixels"] for edge in visible_edges)),
        rgb_boundary_coverage=("complete actual RGB outline with farther measured exterior depth" if complete_boundary
                               else "partial actual RGB boundary; only farther-depth-certified exterior segments retained"),
        plane_centre=fit["centre"].tolist(), plane_normal=fit["normal"].tolist(),
        fit_diagnostics=dict(plane_singular_values=fit["singular"].tolist(),
                             plane_rms_residual_m=float(np.sqrt(np.mean(fit["residual"] ** 2))),
                             plane_p95_abs_residual_m=float(np.percentile(np.abs(fit["residual"]), 95)),
                             residual_role="fit diagnostic only; not a physical error bound"),
        pixel_provenance=pixel_provenance,
        _diagnostic_masks=diagnostic_masks,
        _diagnostics=dict(pixel_provenance=pixel_provenance, masks=diagnostic_masks),
        _points_xyz=xyz[mask], _edge_points_xyz=None if ordered is None else ordered["points"], _mask=mask,
    ), None


def _relation_from_observed_features(pad, obj, *, source_kind, source_image, timestamps):
    pad_points = np.asarray(pad["_points_xyz"], float)
    obj_points = np.asarray(obj["_points_xyz"], float)
    centre = np.asarray(pad["plane_centre"], float)
    normal = np.asarray(pad["plane_normal"], float)
    if np.dot(normal, -centre) < 0:
        normal *= -1
    edge = np.asarray(pad["_edge_points_xyz"], float)
    tangent = edge[1] - edge[0]
    tangent -= normal * np.dot(normal, tangent)
    if np.linalg.norm(tangent) <= np.finfo(float).eps:
        return _missing(source_kind, source_image, timestamps, "the observed actual edge does not identify a pad tangent")
    tangent /= np.linalg.norm(tangent)
    second = np.cross(normal, tangent)
    relative = obj_points - centre
    gap = relative @ normal
    overlap = []
    for axis in (tangent, second):
        interval_pad = (pad_points - centre) @ axis
        interval_object = relative @ axis
        overlap.append(dict(pad_observed_interval_m=[float(interval_pad.min()), float(interval_pad.max())],
                            object_observed_interval_m=[float(interval_object.min()), float(interval_object.max())],
                            signed_overlap_m=float(min(interval_pad.max(), interval_object.max()) - max(interval_pad.min(), interval_object.min()))))
    return dict(classification="CAMERA_DERIVED_VISIBLE_FACE_RELATION", status="OK", source_kind=source_kind,
                source_image=source_image, timestamps=timestamps, input_depth_units="mm",
                pad_plane_normal_camera=normal.tolist(), normal_sign="toward camera from an unidentified observed patch",
                observed_object_face_signed_gap_m=[float(gap.min()), float(gap.max())], tangent_overlap=overlap,
                relation_scope="only current observed RGB-D face pixels and actual visible edges; no hidden corners or opposite face",
                raw_marks=dict(pad_patch_polygon=pad["polygon"], object_face_polygon=obj["polygon"], ordered_pad_edge_uv=pad["ordered_edge"]),
                fit_diagnostics=[pad["fit_diagnostics"], obj["fit_diagnostics"]],
                fit_residual_is_physical_bound=False, independent_physical_truth=False,
                whole_object_minimum_gap_m=None, opposite_inner_pad_gap_m=None, T_runtime_object=None,
                reference=None, ready=False)


def current_frame_visible_face_relation(depth_mm, K, color, pad_roi_polygon, object_roi_polygon, *,
                                        source_kind, source_image, timestamps, frame_identity, diagnostics=None,
                                        object_seed_mask=None):
    """Observe one current visible pad/object face pair from projected candidate ROIs.

    The ROIs may come from actual joints plus a nominal model or a held-object prediction.  They select where to
    search; all returned marks are rebuilt from this frame's RGB edges and registered-depth masks.  The physical pad
    mounting identity therefore remains unknown. A supplied held-face mask seeds the object instead of its polygon;
    its current FrameSet/feedback binding is checked by the owner.
    """
    if source_kind not in ("REAL", "SYNTHETIC"):
        raise ValueError("source_kind must preserve REAL/SYNTHETIC provenance")
    if diagnostics is not None and not isinstance(diagnostics, dict):
        raise ValueError("diagnostics must be a mutable dict")
    diagnostic = diagnostics if diagnostics is not None else {}
    diagnostic.update(status="MISSING", missing_reason=None)
    depth = np.asarray(depth_mm, float)
    image = np.asarray(color)
    K_source = np.asarray(K, float).reshape(3, 3).copy()
    identity = dict(frame_identity) if isinstance(frame_identity, dict) else {}
    missing = None
    if (depth.ndim != 2 or image.ndim not in (2, 3) or min(image.shape[:2], default=0) <= 0
            or (image.ndim == 3 and image.shape[2] < 3) or image.dtype != np.uint8):
        raise ValueError("current registered depth and RGB image are required")
    if not np.isfinite(K_source).all() or min(K_source[0, 0], K_source[1, 1]) <= 0:
        raise ValueError("finite current-frame intrinsics are required")
    frame_number = identity.get("frame_number")
    same_frameset = ((isinstance(frame_number, dict) and "color" in frame_number and "depth" in frame_number
                      and frame_number["color"] == frame_number["depth"])
                     or (isinstance(frame_number, (list, tuple)) and len(frame_number) == 2
                         and frame_number[0] == frame_number[1]))
    if identity.get("fresh") is not True or identity.get("reader_valid") is not True:
        missing = "frame identity is not fresh and reader-valid"
    elif not isinstance(identity.get("n"), int) or isinstance(identity.get("n"), bool):
        missing = "current reader sequence identity is missing"
    elif not isinstance(identity.get("recv_monotonic_s"), (int, float)) or isinstance(identity.get("recv_monotonic_s"), bool) or not np.isfinite(identity["recv_monotonic_s"]):
        missing = "current reader receipt timestamp is missing"
    elif not same_frameset:
        missing = "current color and depth frame numbers do not identify the same FrameSet"
    elif any(k in identity and identity[k] != (timestamps or {}).get(k)
             for k in ("sdk_color_stamp_s", "sdk_depth_stamp_s", "helper_receipt_monotonic_s", "recv_monotonic_s")):
        missing = "frame identity and supplied current-frame timestamps disagree"

    candidate_rois = dict(pad=np.asarray(pad_roi_polygon).tolist(), object=np.asarray(object_roi_polygon).tolist())
    if object_seed_mask is not None and (not isinstance(object_seed_mask, np.ndarray)
            or object_seed_mask.dtype != bool or object_seed_mask.shape != depth.shape):
        raise ValueError("object_seed_mask must be a boolean mask in this registered-depth domain")
    diagnostic.update(frame_identity=identity, candidate_rois=candidate_rois,
                      depth_shape=list(depth.shape), rgb_shape=list(image.shape[:2]))
    role = dict(candidate_role="actual-joint nominal-model/held prediction used only as image ROI",
                roi_coordinate_domain="original RGB pixels; mapped to registered depth by frame dimensions",
                physical_identity="UNKNOWN", inner_face_identity_verified=False,
                observed_face="UNIDENTIFIED_VISIBLE_PATCH",
                unknown=["physical pad mounting/number identity", "opposite inner-pad gap",
                         "hidden housing clearance", "complete object rotation"])
    if missing is None:
        scale = np.array([depth.shape[1] / image.shape[1], depth.shape[0] / image.shape[0]], float)
        K_depth = K_source.copy()
        K_depth[0] *= scale[0]
        K_depth[1] *= scale[1]
        registered_color = image if image.shape[:2] == depth.shape else cv2.resize(
            image, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_AREA)
        rgb_features = observed_rgb_regions(registered_color)
        pad_diagnostic = {}
        diagnostic["pad"] = pad_diagnostic
        pad, missing = observed_face_from_seed(
            depth, K_depth, registered_color, pad_roi_polygon, image_shape=image.shape[:2],
            rgb_features=rgb_features, diagnostics=pad_diagnostic)
        if missing is None and not pad["orientation_identified"]:
            missing = pad["edge_missing_reason"]
        if missing is not None:
            missing = "pad candidate ROI: " + missing
        obj = None
        if missing is None:
            object_diagnostic = {}
            diagnostic["object"] = object_diagnostic
            obj, missing = observed_face_from_seed(
                depth, K_depth, registered_color,
                object_roi_polygon if object_seed_mask is None else object_seed_mask, image_shape=image.shape[:2],
                rgb_features=rgb_features, diagnostics=object_diagnostic)
            if missing is None and not obj["orientation_identified"]:
                missing = obj["edge_missing_reason"]
            if missing is None and np.any(pad["_mask"] & obj["_mask"]):
                missing = "pad/object seeds reacquired the same observed patch; target identities are ambiguous"
            if missing is not None:
                missing = "object candidate ROI: " + missing
    else:
        pad = obj = None

    if missing is not None:
        diagnostic.update(status="MISSING", missing_reason=missing)
        result = _missing(source_kind, source_image, timestamps, missing)
        result.update(role, classification="CAMERA_DERIVED_CURRENT_VISIBLE_FACE_RELATION",
                      frame_identity=identity, observed_object_face_signed_gap_m=None, tangent_overlap=None,
                      whole_object_minimum_gap_m=None, opposite_inner_pad_gap_m=None, T_runtime_object=None,
                      candidate_rois=candidate_rois)
        return result

    relation = _relation_from_observed_features(
        pad, obj, source_kind=source_kind, source_image=source_image, timestamps=timestamps)
    if relation["status"] != "OK":
        diagnostic.update(status="MISSING", missing_reason=relation.get("missing_reason"))
        relation.update(role, frame_identity=identity,
                        observed_object_face_signed_gap_m=None, tangent_overlap=None,
                        whole_object_minimum_gap_m=None, opposite_inner_pad_gap_m=None, T_runtime_object=None,
                        candidate_rois=candidate_rois)
        return relation
    relation.update(role, classification="CAMERA_DERIVED_CURRENT_VISIBLE_PATCH_RELATION", frame_identity=identity,
                    normal_sign="toward camera from an unidentified observed candidate patch; not verified as an inner face",
                    signed_gap_reference="camera-facing normal of the unidentified observed patch",
                    relation_scope="signed gap/overlap of this frame's two observed visible patches; not a verified inner-pad clearance",
                    candidate_rois=candidate_rois,
                    observed_masks=dict(pad={k: pad[k] for k in ("depth_pixels", "rgb_edge_pixels", "visibility", "occlusion")},
                                        object={k: obj[k] for k in ("depth_pixels", "rgb_edge_pixels", "visibility", "occlusion")}),
                    pixel_provenance=dict(pad=pad["pixel_provenance"], object=obj["pixel_provenance"]),
                    _diagnostic_masks=dict(pad=pad["_diagnostic_masks"], object=obj["_diagnostic_masks"]),
                    _diagnostics=dict(pad=pad["_diagnostics"], object=obj["_diagnostics"]),
                    producer="current_frame_visible_face_relation")
    diagnostic.update(status="OK", missing_reason=None,
                      pixel_provenance=relation["pixel_provenance"])
    return relation


def calculate(depth_mm, K, image_shape, inner_pad_polygons, right_long_edge_uv, object_pose_camera, *,
              source_kind, source_image, timestamps, object_axes_observed=()):
    """Use both complete four-corner inner faces; ordered right-pad edge identifies +Y."""
    if source_kind not in ("REAL", "SYNTHETIC"):
        raise ValueError("source_kind must preserve REAL/SYNTHETIC provenance")
    depth = np.asarray(depth_mm, float)
    if depth.ndim != 2 or image_shape is None or not hasattr(image_shape, "__len__") or len(image_shape) < 2:
        raise ValueError("registered depth and source image shape are required")
    K = np.asarray(K, float).reshape(3, 3).copy()
    h, w = int(image_shape[0]), int(image_shape[1])
    if h <= 0 or w <= 0 or not np.isfinite(K).all() or K[0, 0] == 0 or K[1, 1] == 0:
        raise ValueError("finite intrinsics and source image dimensions are required")
    scale = np.array([depth.shape[1] / w, depth.shape[0] / h], float)
    K[0] *= scale[0]
    K[1] *= scale[1]
    if not isinstance(inner_pad_polygons, (list, tuple)) or len(inner_pad_polygons) != 2:
        return _missing(source_kind, source_image, timestamps, "both marked right inner-pad face polygons are required")
    if any(np.asarray(polygon).shape != (4, 2) for polygon in inner_pad_polygons):
        return _missing(source_kind, source_image, timestamps, "both physical inner faces need their four observed corners")
    planes = [_patch(depth, K, polygon, scale) for polygon in inner_pad_polygons]
    if any(plane is None for plane in planes):
        return _missing(source_kind, source_image, timestamps, "a marked inner-pad patch has fewer than three valid non-collinear depth pixels")
    edge = np.asarray(right_long_edge_uv, float)
    if edge.shape != (2, 2) or not np.isfinite(edge).all():
        return _missing(source_kind, source_image, timestamps, "the ordered right long-pad edge mark is missing")
    corners = [[_plane_ray(K, uv, scale, plane) for uv in polygon]
               for plane, polygon in zip(planes, inner_pad_polygons)]
    if any(point is None for face in corners for point in face):
        return _missing(source_kind, source_image, timestamps, "an observed inner-face corner does not meet its fitted plane")
    # Partial depth coverage must not move the physical pad origin toward the visible patch.
    c0, c1 = [np.mean(face, axis=0) for face in corners]
    separation = c1 - c0
    n0, n1 = planes[0]["normal"].copy(), planes[1]["normal"].copy()
    if np.dot(n0, separation) < 0:
        n0 *= -1
    if np.dot(n1, -separation) < 0:
        n1 *= -1
    x = n0 - n1
    if np.linalg.norm(x) <= np.finfo(float).eps:
        return _missing(source_kind, source_image, timestamps, "opposing inner-plane normals do not identify the jaw normal")
    x /= np.linalg.norm(x)
    if np.dot(x, separation) < 0:
        x *= -1
    midpoint = (c0 + c1) / 2
    denominators = [float(np.dot(n, x)) for n in (n0, n1)]
    if min(map(abs, denominators)) <= np.finfo(float).eps:
        return _missing(source_kind, source_image, timestamps, "the midpoint cross-section is parallel to an inner plane")
    crossings = [midpoint + x * (np.dot(n, c - midpoint) / denominator)
                 for n, c, denominator in zip((n0, n1), (c0, c1), denominators)]
    edge_points = [_plane_ray(K, uv, scale, planes[1]) for uv in edge]
    if any(point is None for point in edge_points):
        return _missing(source_kind, source_image, timestamps, "the marked long-pad edge ray does not meet the fitted right inner plane")
    y = edge_points[1] - edge_points[0]
    y -= x * np.dot(y, x)
    if np.linalg.norm(y) <= np.finfo(float).eps:
        return _missing(source_kind, source_image, timestamps, "the long-pad edge does not identify an axis orthogonal to the jaw normal")
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    T_camera_pad = np.eye(4)
    T_camera_pad[:3, :3] = np.column_stack([x, y, z])
    T_camera_pad[:3, 3] = (crossings[0] + crossings[1]) / 2
    T_camera_object = np.asarray(object_pose_camera, float)
    if T_camera_object.shape != (4, 4) or not np.isfinite(T_camera_object).all():
        return _missing(source_kind, source_image, timestamps, "the partial object pose in the same camera frame is missing")
    R = T_camera_object[:3, :3]
    if (not np.allclose(T_camera_object[3], [0, 0, 0, 1])
            or not np.allclose(R.T @ R, np.eye(3)) or not np.isclose(np.linalg.det(R), 1)):
        return _missing(source_kind, source_image, timestamps, "the observed object pose is not a rigid transform")
    T_pad_object = np.linalg.inv(T_camera_pad) @ T_camera_object
    axes = set(object_axes_observed)
    return dict(classification=CLASSIFICATION, status="OK", source_kind=source_kind, source_image=source_image, timestamps=timestamps,
                input_depth_units="mm", independent_physical_truth=False,
                T_camera_pad=T_camera_pad.tolist(), T_camera_object=T_camera_object.tolist(),
                T_pad_object=T_pad_object.tolist(), T_object_pad=np.linalg.inv(T_pad_object).tolist(),
                registration=dict(source_image_shape=[h, w], depth_shape=list(depth.shape), K_at_depth_shape=K.tolist()),
                marked_features=dict(inner_pad_polygons=np.asarray(inner_pad_polygons, float).tolist(), right_long_edge_uv=edge.tolist()),
                raw_source_points=dict(depth_units="mm", valid_points_per_patch=[p["points"] for p in planes],
                                       inner_face_corners_camera_m=[np.asarray(face).tolist() for face in corners],
                                       invalid_or_zero_depth_pixels_per_patch=[p["missing_pixels"] for p in planes]),
                inner_pad_gap_m=float(abs(np.dot(crossings[1] - crossings[0], x))),
                gap_definition="distance between the two fitted inner planes through the midpoint of their observed full-face centres; not object-to-pad distance",
                axes=dict(x="inner_pad_polygons[0] toward inner_pad_polygons[1], from opposing plane normals",
                          y="ordered observed long edge on inner_pad_polygons[1], orthogonalized", z="cross(x,y)"),
                object_axes_observed=sorted(axes), object_axes_missing=sorted({"x", "y", "z"} - axes),
                inner_normal_tilt_error_rad=float(np.arccos(np.clip(np.dot(n0, -n1), -1.0, 1.0))),
                fit_diagnostics=[dict(points=p["points"], invalid_or_zero_depth_pixels=p["missing_pixels"],
                                      singular_values=p["singular"].tolist(), plane_rms_residual_m=p["rms"],
                                      plane_p95_abs_residual_m=p["p95"], residual_role="diagnostic only; not a physical bound") for p in planes],
                fit_residual_is_physical_bound=False, features_complete=not bool({"x", "y", "z"} - axes),
                reference=None, runtime_convention=dict(name="UNKNOWN", transform=None), ready=False)
