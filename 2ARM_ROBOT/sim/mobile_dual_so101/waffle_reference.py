"""Data-derived TurtleBot3 Waffle Pi mounting reference.

The values in this module come from the preserved ROBOTIS Jazzy URDF/base STL
and the official TB3_WAFFLE_PLATE-IPL-01 drawing/STEP.  They describe geometry
only and never open or command physical hardware.
"""

from __future__ import annotations

from typing import Sequence


OFFICIAL_TURTLEBOT3_COMMIT = "0c0be84e3f5c3194fb2adea8426a58a96060eab5"
OFFICIAL_WAFFLE_URDF_SHA256 = (
    "33c201d21492246e9eba8ecd7f1ca9ae4bd7c88a1da5bc360584002ba61ab9ce"
)
OFFICIAL_WAFFLE_BASE_STL_SHA256 = (
    "706230121a287ff953c4c84d344014a147fc27615b0e95221aaf04cb0dca7e42"
)
OFFICIAL_WAFFLE_PLATE_PDF_SHA256 = (
    "f4568fa9e642e54998a2be0224c906b1dfec823d5bb9c10a8450833eb00e8105"
)
OFFICIAL_WAFFLE_PLATE_STEP_SHA256 = (
    "966d4e1a0236ef69674870bbb130ae7ce7fceb2168eb5eb7847e4b01c8a268d3"
)

# REP-103/mobile-base convention confirmed by the official URDF: the original
# camera is at +X, the left wheel is at +Y and base_joint raises +Z.
BASE_LINK_X_FORWARD = (1.0, 0.0, 0.0)
BASE_LINK_Y_LEFT = (0.0, 1.0, 0.0)
BASE_LINK_Z_UP = (0.0, 0.0, 1.0)

# The official visual mesh is translated -64 mm along base_link X.  Therefore
# mesh point (64, 0, 0) coincides with the wheel-axis base_link origin.
OFFICIAL_VISUAL_MESH_ORIGIN_M = (-0.064, 0.0, 0.0)
OFFICIAL_WAFFLE_PLATE_SIZE_M = (0.128, 0.064, 0.009)

# The assembled official STL has its upper Waffle plate plane at 91.5 mm.
# URDF's simplified base collision box reaches 94 mm; it is a collision proxy,
# not the physical plate datum.
WAFFLE_TOP_MOUNT_PLANE_Z_M = 0.0915
WAFFLE_BASE_COLLISION_PROXY_TOP_Z_M = 0.094
WAFFLE_TOP_REFERENCE_ORIGIN_M = (-0.064, 0.0, WAFFLE_TOP_MOUNT_PLANE_Z_M)

# Circular top openings extracted from the official assembled STL.  The full
# Waffle feature is a compound M3 bolt/nut profile, so these are center datums,
# not drill diameters.  The six selected points form a symmetric, wide deck
# attachment pattern and are all visibly open in the official top layer.
TOWER_DECK_ANCHOR_POINTS_MESH_MM = (
    (-88.0, -64.0, 91.5),
    (-88.0, 64.0, 91.5),
    (0.0, -88.0, 91.5),
    (0.0, 88.0, 91.5),
    (40.0, -64.0, 91.5),
    (40.0, 64.0, 91.5),
)
TOWER_DECK_ANCHOR_POINTS_M = (
    (-0.152, -0.064, WAFFLE_TOP_MOUNT_PLANE_Z_M),
    (-0.152, 0.064, WAFFLE_TOP_MOUNT_PLANE_Z_M),
    (-0.064, -0.088, WAFFLE_TOP_MOUNT_PLANE_Z_M),
    (-0.064, 0.088, WAFFLE_TOP_MOUNT_PLANE_Z_M),
    (-0.024, -0.064, WAFFLE_TOP_MOUNT_PLANE_Z_M),
    (-0.024, 0.064, WAFFLE_TOP_MOUNT_PLANE_Z_M),
)

# The source upper is the custom SO-101 socket. It replaces the three static
# printed base parts from each stock arm while retaining the base servo and the
# complete shoulder-and-downstream kinematic chain. The source upper's large
# socket axes remain at Y +/-127 mm, which places arm frames at +/-93.4 mm.
TOWER_DECK_CENTER_X_M = -0.064
TOWER_DECK_HALF_SIZE_X_M = 0.096
TOWER_DECK_HALF_SIZE_Y_M = 0.128
TOWER_CENTER_X_M = -0.064
TOWER_CENTER_Y_ABS_M = 0.0934

# Teammate split-print CAD revision: assem_base.step plus two STL parts.
# Autodesk STEP/STL coordinates are millimetres. The upper STL has a local
# Z=0..156 mm range and is translated +160 mm in the assembly, giving a 10 mm
# overlap with the lower Z=0..170 mm part.
ASSEMBLED_SUPPORT_STEP_SHA256 = (
    "f9f77f71a77f962aac3c7a3898bf5df7232b12982be3f16e3e2fc20d39c1bb3b"
)
ASSEMBLED_SUPPORT_UNDER_STL_SHA256 = (
    "f91c58b14bd9932757787d9fea1f72104bae537d38b003d2a430576fecc70076"
)
ASSEMBLED_SUPPORT_UPPER_SOURCE_STL_SHA256 = (
    "ed6218f3ba83459fc7c436416ef708130e2622a27df6772d4d62bf6a9d822bf7"
)
ASSEMBLED_SUPPORT_UPPER_STL_SHA256 = (
    "ed6218f3ba83459fc7c436416ef708130e2622a27df6772d4d62bf6a9d822bf7"
)
ASSEMBLED_SUPPORT_UPPER_Z_OFFSET_M = 0.160
SO101_SOCKET_AXIS_Y_ABS_M = 0.127
SEMI_SUPPORT_BASE_SIZE_M = (0.160, 0.180, 0.025)
SEMI_SUPPORT_COLUMN_SIZE_M = (0.090, 0.100, 0.275)
SEMI_SUPPORT_COLUMN_TOP_LOCAL_Z_M = 0.300
SEMI_SUPPORT_LOCAL_MAX_Z_M = 0.31600000010259134
SEMI_SUPPORT_BIG_HOLE_RADIUS_M = 0.0085
SEMI_SUPPORT_BIG_HOLE_CENTERS_LOCAL_M = (
    (0.0, +SO101_SOCKET_AXIS_Y_ABS_M, 0.302550896),
    (0.0, -SO101_SOCKET_AXIS_Y_ABS_M, 0.302550896),
)
SEMI_SUPPORT_BOTTOM_HOLES_LOCAL_M = (
    (-0.077, -0.087, 0.0),
    (-0.077, +0.087, 0.0),
    (+0.077, -0.087, 0.0),
    (+0.077, +0.087, 0.0),
)


def mesh_mm_to_base_link_m(point_mm: Sequence[float]) -> tuple[float, float, float]:
    """Apply the official URDF visual transform to one mesh-space point."""

    if len(point_mm) != 3:
        raise ValueError("mesh point must contain exactly three coordinates")
    return tuple(
        float(value) / 1000.0 + offset
        for value, offset in zip(point_mm, OFFICIAL_VISUAL_MESH_ORIGIN_M)
    )


def validate_reference() -> None:
    """Fail closed if edited reference axes or anchor symmetry are invalid."""

    x_axis = BASE_LINK_X_FORWARD
    y_axis = BASE_LINK_Y_LEFT
    z_axis = BASE_LINK_Z_UP
    cross_xy = (
        x_axis[1] * y_axis[2] - x_axis[2] * y_axis[1],
        x_axis[2] * y_axis[0] - x_axis[0] * y_axis[2],
        x_axis[0] * y_axis[1] - x_axis[1] * y_axis[0],
    )
    if cross_xy != z_axis:
        raise RuntimeError("Waffle base_link axes must remain right-handed")
    anchors = set(TOWER_DECK_ANCHOR_POINTS_M)
    for x_value, y_value, z_value in anchors:
        if (x_value, -y_value, z_value) not in anchors:
            raise RuntimeError("Waffle tower-deck anchors must remain Y-symmetric")
        if z_value != WAFFLE_TOP_MOUNT_PLANE_Z_M:
            raise RuntimeError("Waffle tower-deck anchors left the top mount plane")


validate_reference()
