from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from shoe_task import ShoeTaskConfig, ShoeTaskEnv, MOBILE_BLOCK_CONFIG, block_metrics, UnsafeActionError
from mobile_dual_so101 import actuator_targets_from_qpos, HUMANOID_HOME_ACTION
from parallel_shoe_rollout import ParallelRolloutConfig, run_parallel_rollouts

BOUNDED_CONFIG = MOBILE_BLOCK_CONFIG


class SeededResetTest(unittest.TestCase):
    def test_current_mobile_pose_rejects_arm_floor_and_blocks_physics(self):
        env = ShoeTaskEnv(replace(BOUNDED_CONFIG, initial_home_pose=False))
        # Repaired common plane guard rejects before the later reset-only check.
        with self.assertRaises(UnsafeActionError) as caught:
            env.reset(seed=0)
        self.assertEqual(caught.exception.assessment.second_geom_id, env.model.geom("floor").id)
        self.assertLess(caught.exception.assessment.minimum_clearance_m, 0)
        with self.assertRaisesRegex(ValueError, "successfully validated reset"):
            env.apply_action([0.] * 12, physics_steps=1)
        self.assertEqual(env.data.time, 0)

    def test_home_measured_and_controller_targets_match(self):
        env = ShoeTaskEnv(BOUNDED_CONFIG)
        env.reset(seed=0)
        np.testing.assert_allclose(actuator_targets_from_qpos(env.model, env.data.qpos),
                                   HUMANOID_HOME_ACTION, atol=1e-12, rtol=0)
        np.testing.assert_array_equal(env.data.ctrl,
                                      actuator_targets_from_qpos(env.model, env.data.qpos))
        np.testing.assert_array_equal(env.data.qvel, 0)

    def test_settle_keeps_home_target_and_records_floor_reference(self):
        env = ShoeTaskEnv(BOUNDED_CONFIG)
        env.reset(seed=0)
        target = env.data.ctrl.copy()
        info = env.settle()
        self.assertEqual(info["phase"], "PREGRASP")
        self.assertGreaterEqual(info["stable_steps"], 10)
        self.assertGreaterEqual(info["arm_floor_clearance_m"], .03)
        self.assertGreaterEqual(info["arm_block_clearance_m"], .03)
        self.assertAlmostEqual(env.data.time, .2)
        np.testing.assert_array_equal(target, env.data.ctrl)
        self.assertFalse(env.metrics()["success"])
        env.reset(seed=0)
        self.assertIsNone(env.settle_info)

    def test_only_target_geometry_changes_and_block_parameters_match(self):
        env = ShoeTaskEnv(BOUNDED_CONFIG)
        legacy = ShoeTaskEnv(ShoeTaskConfig())
        m, old = env.model, legacy.model
        geom = m.geom("block_geom")
        self.assertEqual(int(m.geom_type[geom.id]), int(mujoco.mjtGeom.mjGEOM_BOX))
        np.testing.assert_array_equal(geom.size, [.02,.02,.02])
        np.testing.assert_allclose(m.body("shoe").mass, [.02])
        np.testing.assert_allclose(m.body("shoe").inertia, np.full(3, .02 * .04**2 / 6))
        np.testing.assert_array_equal(geom.friction, [1.6,.02,.001])
        np.testing.assert_array_equal(geom.solref, [-200000.,-400.])
        np.testing.assert_array_equal(geom.solimp, [.95,.99,.001,.5,2.])
        self.assertEqual(m.ngeom, old.ngeom - 1)
        for field in ("geom_type","geom_pos","geom_quat","geom_contype","geom_conaffinity"):
            np.testing.assert_array_equal(getattr(m,field)[:geom.id], getattr(old,field)[:geom.id])
        for field in ("body_pos","body_quat","jnt_range","actuator_ctrlrange","cam_pos","cam_quat"):
            np.testing.assert_array_equal(getattr(m,field)[:-1] if field in ("body_pos","body_quat") else getattr(m,field),
                                          getattr(old,field)[:-1] if field in ("body_pos","body_quat") else getattr(old,field))
        for field in ("timestep","integrator","disableflags","enableflags","ccd_iterations","ccd_tolerance"):
            self.assertEqual(getattr(m.opt,field), getattr(old.opt,field))
        self.assertEqual(m.joint("shoe_free").type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(m.body("shoe").mocapid[0], -1)

    def test_block_success_requires_supported_continuous_lift(self):
        env = ShoeTaskEnv(BOUNDED_CONFIG)
        env.reset(seed=0)
        adr = env.model.joint("shoe_free").qposadr[0]
        env.data.qpos[adr + 2] = .15
        mujoco.mj_forward(env.model,env.data)
        snapshot = block_metrics(env.model,env.data)
        self.assertTrue(snapshot["lifted"])
        self.assertFalse(snapshot["success"])
        self.assertFalse(snapshot["grasp_supported"])
        env.reset(seed=0)
        hold = actuator_targets_from_qpos(env.model, env.data.qpos)
        evidence = {"target_object":"block","lift_supported":True,"success":False,"reward":0}
        with patch("shoe_task.block_metrics", return_value=evidence), patch("shoe_task.mujoco.mj_step"):
            env.apply_action(hold, physics_steps=1000)
            self.assertFalse(env.metrics()["success"])
            env.apply_action(hold, physics_steps=501)
            self.assertTrue(env.metrics()["success"])
            evidence["lift_supported"] = False
            env.apply_action(hold, physics_steps=1)
            self.assertFalse(env.metrics()["success"])
            self.assertEqual(env.metrics()["continuous_hold_s"], 0)
        env.reset(seed=0)
        self.assertEqual(env.metrics()["continuous_hold_s"], 0)

    def test_seed_reproducibility_diversity_and_100_safe_resets(self):
        env = ShoeTaskEnv(BOUNDED_CONFIG)
        initial = []
        for seed in range(100):
            obs, info = env.reset(seed=seed)
            self.assertTrue(info["randomized"])
            self.assertTrue(info["validation"]["collision_guard"]["safe"])
            self.assertTrue(info["validation"]["joint_limits"])
            self.assertGreaterEqual(info["validation"]["target_clearance_m"], .03)
            self.assertLess(info["validation"]["reachability"]["shoulder_distance_m"]["left"], .4)
            self.assertTrue(info["validation"]["floor_nonpenetrating"])
            self.assertEqual(env.data.time, 0)
            self.assertTrue(np.all(env.data.qvel == 0))
            self.assertTrue(np.all(np.abs(np.array(obs["shoe"]["position_map_m"])[:2] - [-.22,.22]) <= .002))
            self.assertLessEqual(abs(obs["shoe"]["yaw_map_rad"]), math.radians(.5))
            self.assertEqual(obs["shoe"]["position_map_m"][2], .021)
            initial.append(tuple(obs["shoe"]["position_map_m"]) + (obs["shoe"]["yaw_map_rad"],))
            json.dumps(info, allow_nan=False)
        self.assertEqual(len(set(initial)), 100)
        for seed in (0,1,2,100,101,1000):
            obs, info = env.reset(seed=seed)
            qpos, qvel = env.data.qpos.copy(), env.data.qvel.copy()
            env.data.qpos[:] += .001
            obs2, info2 = env.reset(seed=seed)
            self.assertTrue(np.array_equal(qpos, env.data.qpos))
            self.assertTrue(np.array_equal(qvel, env.data.qvel))
            self.assertEqual((obs, info), (obs2, info2))

    def test_invalid_ranges_seeds_and_unreachable_reset_fail_closed(self):
        for change in (dict(shoe_xy_range_m=-1), dict(shoe_yaw_range_rad=float("nan"))):
            with self.assertRaises(ValueError):
                ShoeTaskEnv(replace(BOUNDED_CONFIG, **change))
        env = ShoeTaskEnv(BOUNDED_CONFIG)
        for seed in (-1, 1.5, True):
            with self.assertRaises(ValueError):
                env.reset(seed=seed)
        with self.assertRaisesRegex(ValueError, "reach envelope"):
            ShoeTaskEnv(ShoeTaskConfig(shoe_xy_range_m=.002, initial_home_pose=True)).reset(seed=0)
        with self.assertRaisesRegex(ValueError, "collision clearance"):
            ShoeTaskEnv(replace(BOUNDED_CONFIG, shoe_position_m=(-.1,.1,.015))).reset(seed=0)
        with self.assertRaisesRegex(ValueError, "collision clearance"):
            ShoeTaskEnv(replace(BOUNDED_CONFIG, shoe_position_m=(-.22,.22,.015))).reset(seed=0)
        with self.assertRaisesRegex(ValueError, "object_kind"):
            ShoeTaskEnv(model=env.model)

    def test_world_weld_cannot_count_as_free_block_grasp(self):
        model = mujoco.MjModel.from_xml_string("""
        <mujoco><worldbody>
          <body name="left_gripper" pos="-1 0 0"><geom size=".01"/></body>
          <body name="right_gripper" pos="1 0 0"><geom size=".01"/></body>
          <body name="shoe" pos="0 0 .1"><freejoint name="shoe_free"/>
            <geom name="block_geom" type="box" size=".02 .02 .02" mass=".02"/>
          </body></worldbody><equality><weld body1="shoe"/></equality></mujoco>
        """)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model,data)
        metrics = block_metrics(model,data)
        self.assertTrue(metrics["attachment_active"])
        self.assertFalse(metrics["grasp_supported"])
        self.assertFalse(metrics["success"])

    def test_block_contact_evidence_requires_both_left_fingers(self):
        env = ShoeTaskEnv(BOUNDED_CONFIG)
        env.reset(seed=0)
        m,d = env.model,env.data
        d.qpos[m.joint("shoe_free").qposadr[0] + 2] = .15
        mujoco.mj_forward(m,d)
        def collision_geom(mesh_name):
            return next(g for g in range(m.ngeom) if m.geom_contype[g]
                        and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                        and m.mesh(int(m.geom_dataid[g])).name == mesh_name)
        fixed = collision_geom("left_wrist_roll_follower_so101_v1")
        moving = collision_geom("left_moving_jaw_so101_v1")
        motor = collision_geom("left_sts3215_03a_no_horn_v1")
        def contact_force(model,data,index,wrench):
            wrench[0] = 1
        for others,expected in (([fixed],False),([fixed,moving],True),([motor,moving],False)):
            fake = SimpleNamespace(
                contact=[SimpleNamespace(geom1=m.geom("block_geom").id,geom2=g,dist=-.0001) for g in others],
                geom_xmat=d.geom_xmat,geom_xpos=d.geom_xpos,xpos=d.xpos,eq_active=d.eq_active)
            with patch("shoe_task.mujoco.mj_contactForce", side_effect=contact_force):
                result = block_metrics(m,fake)
            self.assertEqual(result["grasp_supported"],expected)
            self.assertFalse(result["success"])  # Snapshot cannot prove a hold.

    def test_worker_partition_independent_initial_conditions(self):
        results = []
        for workers in (1,4):
            result = run_parallel_rollouts(
                ParallelRolloutConfig(workers=workers, episodes=8, steps_per_episode=3, master_seed=100),
                shoe_config=BOUNDED_CONFIG,
            )
            rows = sorted((row for worker in result["workers"] for row in worker["initial_states"]),
                          key=lambda row: row["episode_id"])
            self.assertEqual([row["reset"]["seed"] for row in rows], list(range(100,108)))
            results.append(rows)
        self.assertEqual(results[0], results[1])

if __name__ == "__main__":
    unittest.main()
