from __future__ import annotations

import hashlib
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
TURTLEBOT_DIR = PROJECT_DIR.parent / "turtlebot3_waffle_pi"
sys.path.insert(0, str(PROJECT_DIR))

from waffle_reference import (
    BASE_LINK_X_FORWARD,
    BASE_LINK_Y_LEFT,
    BASE_LINK_Z_UP,
    OFFICIAL_WAFFLE_BASE_STL_SHA256,
    OFFICIAL_WAFFLE_URDF_SHA256,
    TOWER_CENTER_X_M,
    TOWER_CENTER_Y_ABS_M,
    TOWER_DECK_ANCHOR_POINTS_M,
    TOWER_DECK_ANCHOR_POINTS_MESH_MM,
    TOWER_DECK_CENTER_X_M,
    TOWER_DECK_HALF_SIZE_X_M,
    TOWER_DECK_HALF_SIZE_Y_M,
    WAFFLE_BASE_COLLISION_PROXY_TOP_Z_M,
    WAFFLE_TOP_MOUNT_PLANE_Z_M,
    mesh_mm_to_base_link_m,
)


class WaffleReferenceTest(unittest.TestCase):
    def test_preserved_official_urdf_and_base_mesh_match_recorded_hashes(self) -> None:
        urdf = (
            TURTLEBOT_DIR
            / "upstream/turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf"
        )
        mesh = (
            TURTLEBOT_DIR
            / "upstream/turtlebot3_description/meshes/bases/waffle_pi_base.stl"
        )
        self.assertEqual(
            hashlib.sha256(urdf.read_bytes()).hexdigest(),
            OFFICIAL_WAFFLE_URDF_SHA256,
        )
        self.assertEqual(
            hashlib.sha256(mesh.read_bytes()).hexdigest(),
            OFFICIAL_WAFFLE_BASE_STL_SHA256,
        )

    def test_official_mesh_transform_places_wheel_axis_at_base_link_origin(
        self,
    ) -> None:
        self.assertEqual(mesh_mm_to_base_link_m((64.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        self.assertEqual(
            mesh_mm_to_base_link_m((64.0, 0.0, 91.5)),
            (0.0, 0.0, WAFFLE_TOP_MOUNT_PLANE_Z_M),
        )

    def test_base_link_axes_are_right_handed_x_forward_y_left_z_up(self) -> None:
        self.assertEqual(BASE_LINK_X_FORWARD, (1.0, 0.0, 0.0))
        self.assertEqual(BASE_LINK_Y_LEFT, (0.0, 1.0, 0.0))
        self.assertEqual(BASE_LINK_Z_UP, (0.0, 0.0, 1.0))

    def test_selected_official_mesh_holes_map_to_deck_anchors(self) -> None:
        transformed = tuple(
            mesh_mm_to_base_link_m(point)
            for point in TOWER_DECK_ANCHOR_POINTS_MESH_MM
        )
        self.assertEqual(transformed, TOWER_DECK_ANCHOR_POINTS_M)
        for x_value, y_value, z_value in TOWER_DECK_ANCHOR_POINTS_M:
            self.assertLessEqual(
                abs(x_value - TOWER_DECK_CENTER_X_M),
                TOWER_DECK_HALF_SIZE_X_M,
            )
            self.assertLessEqual(abs(y_value), TOWER_DECK_HALF_SIZE_Y_M)
            self.assertEqual(z_value, WAFFLE_TOP_MOUNT_PLANE_Z_M)
            self.assertIn(
                (x_value, -y_value, z_value),
                TOWER_DECK_ANCHOR_POINTS_M,
            )

    def test_tower_and_camera_reference_remain_centered_on_base(self) -> None:
        self.assertEqual(TOWER_CENTER_X_M, TOWER_DECK_CENTER_X_M)
        self.assertEqual(TOWER_CENTER_Y_ABS_M, 0.100)
        self.assertLess(
            WAFFLE_TOP_MOUNT_PLANE_Z_M,
            WAFFLE_BASE_COLLISION_PROXY_TOP_Z_M,
        )


if __name__ == "__main__":
    unittest.main()
