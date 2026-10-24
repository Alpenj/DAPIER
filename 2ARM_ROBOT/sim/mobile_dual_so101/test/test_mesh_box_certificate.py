import json
from pathlib import Path
import unittest

import mujoco
import numpy as np

from collision_guard import (certified_mesh_box_separation_lower_bound,
                             minimum_protected_clearance, check_bimanual_path)
from integration_scenes import task_env
from mobile_dual_so101 import actuator_targets_from_qpos


class MeshBoxCertificateTest(unittest.TestCase):
    def test_saved_native_false_zero_stays_below_general_30mm(self):
        fixture = json.loads((Path(__file__).parent/"fixtures/mesh_box_false_zero.json").read_text())
        env = task_env()
        m, d = env.model, env.data
        d.qpos[:] = fixture["qpos"]
        d.qvel[:] = fixture["qvel"]
        d.ctrl[:] = fixture["ctrl"]
        mujoco.mj_forward(m, d)
        for pair in (fixture["pair"], fixture["pair"][::-1]):
            native = mujoco.mj_geomDistance(m, d, *pair, .03, None)
            self.assertEqual(native, fixture["native_distance_m"])
            details = []
            gap = minimum_protected_clearance(m, d, [pair], distance_cap_m=.03,
                                               diagnostics=details)[0]
            self.assertGreaterEqual(gap, fixture["expected_positive_separation_lower_bound_min_m"])
            self.assertLessEqual(gap, fixture["independent_geometry"]["compiled_raw"]["gap_m"])
            self.assertLess(gap, .03)
            json.dumps(details)  # viewer/report must not receive numpy bool scalars
            self.assertTrue(details[0]["mesh_box_certificate_eligible"])
            self.assertFalse(details[0]["mesh_certificate_eligible"])
        q = actuator_targets_from_qpos(m, d.qpos)
        self.assertTrue(check_bimanual_path(m, q, q).safe)  # exact SIM near-support policy; distance still <30mm

    def test_rotated_box_separated_touching_penetrating(self):
        m = mujoco.MjModel.from_xml_string("""
        <mujoco><asset><mesh name="cube" vertex="
          -.1 -.1 -.1  -.1 -.1 .1  -.1 .1 -.1  -.1 .1 .1
           .1 -.1 -.1   .1 -.1 .1   .1 .1 -.1   .1 .1 .1"/></asset>
        <worldbody><geom name="box" type="box" size=".1 .1 .1"
          quat=".9238795325112867 0 .3826834323650898 0"/>
          <body><freejoint/><geom type="mesh" mesh="cube" mass="1"/></body>
        </worldbody></mujoco>""")
        d = mujoco.MjData(m)
        d.qpos[3:7] = m.geom_quat[0]
        mujoco.mj_forward(m, d)
        axis = d.geom_xmat[0].reshape(3,3)[:,0]
        mesh = int(m.geom_dataid[1])
        v = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
        world = v @ d.geom_xmat[1].reshape(3,3).T + d.geom_xpos[1]
        lower = float((world @ axis).min())
        for gap in (.04, 0., -.01):
            d.qpos[:3] = axis * (.1 - lower + gap)
            mujoco.mj_forward(m, d)
            for pair in ((1,0), (0,1)):
                with self.subTest(gap=gap, pair=pair):
                    cert = certified_mesh_box_separation_lower_bound(m, d, *pair)
                    final = minimum_protected_clearance(m, d, [pair])[0]
                    if gap > 0:
                        self.assertAlmostEqual(cert, gap, places=12)
                    else:
                        self.assertEqual(cert, 0.)
                        self.assertLessEqual(final, 1e-12)
                        self.assertLess(final, .03)
                    if gap < 0:
                        self.assertTrue(any(c.dist < 0 for c in d.contact))
        # Existing mesh-only certificate is not repurposed for unsupported types.
        self.assertEqual(certified_mesh_box_separation_lower_bound(m, d, 0, 0), 0.)


if __name__ == "__main__":
    unittest.main()
