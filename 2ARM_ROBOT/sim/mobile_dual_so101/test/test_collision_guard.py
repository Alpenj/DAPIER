from __future__ import annotations

from pathlib import Path
import json
import sys
import unittest
from unittest.mock import patch

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

from collision_guard import (
    _collision_geoms_for_arm,
    bimanual_geom_pairs,
    check_bimanual_path,
    certified_separation_lower_bound,
    minimum_protected_clearance,
    evaluate_pair_clearance_evidence, evaluate_clearance_set, ClearanceStatus,
    protected_geom_pairs,
    _carried_object_attachment, _apply_carried_object, _carried_support_clearance,
)
from mobile_dual_so101 import (
    HUMANOID_HOME_ACTION,
    TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
    apply_control_as_pose,
    build_model,
)


UNSAFE_BIMANUAL_TARGET = (
    0.195731791341601,
    1.6758398119082343,
    -0.1950753295290686,
    -0.5266413229575377,
    0.3401861733712628,
    0.4411373258803475,
    -0.485894097852944,
    0.5940617023869152,
    0.4596671975241349,
    0.45149208438578126,
    -1.875765584091561,
    1.0153142196339973,
)
RESTORED_BASE_CAMERA_CLEARANCE_M = 0.10


class ClearanceEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.model=mujoco.MjModel.from_xml_string('<mujoco><worldbody>'
            '<geom name="a" type="sphere" size=".01"/>'
            '<geom name="b" type="sphere" size=".01" pos=".1 0 0"/>'
            '<geom name="c" type="sphere" size=".01" pos=".2 0 0"/>'
            '</worldbody></mujoco>')
        self.data=mujoco.MjData(self.model);mujoco.mj_forward(self.model,self.data)

    def test_native_contract_failure_cannot_hide_in_legacy_minimum(self):
        for native in (float('nan'),float('inf'),float('-inf'),.051):
            with self.subTest(native=native),patch.object(mujoco,'mj_geomDistance',side_effect=[native,.05]):
                gap,a,b=minimum_protected_clearance(self.model,self.data,[(0,1),(0,2)],distance_cap_m=.05)
                self.assertLess(gap,0);self.assertEqual((a,b),(0,1))
            with patch.object(mujoco,'mj_geomDistance',return_value=native):
                evidence=evaluate_pair_clearance_evidence(self.model,self.data,(0,1),
                    required_clearance_m=.03,query_cap_m=.05)
                self.assertFalse(evidence.safe)
                self.assertEqual(evidence.status,ClearanceStatus.ENGINE_CONTRACT_FAILURE)

    def test_finite_inputs_empty_pairs_and_cutoff_semantics(self):
        for required,cap in ((float('nan'),.05),(.03,float('inf')),(.03,.02),(.03,.03)):
            with self.subTest(required=required,cap=cap),self.assertRaises(ValueError):
                evaluate_pair_clearance_evidence(self.model,self.data,(0,1),
                    required_clearance_m=required,query_cap_m=cap)
        with self.assertRaises(ValueError):
            evaluate_clearance_set(self.model,self.data,[],required_clearance_m=.03,query_cap_m=.05)
        clipped=evaluate_pair_clearance_evidence(self.model,self.data,(0,1),
            required_clearance_m=.03,query_cap_m=.05)
        self.assertTrue(clipped.safe);self.assertIsNone(clipped.exact_distance_m)
        self.assertEqual(clipped.provable_clearance_m,.05)
        # A native value just below the cap is still exact, never rounded up to pass.
        self.data.geom_xpos[1]=[.049,0,0]
        with patch.object(mujoco,'mj_geomDistance',return_value=.03-1e-15):
            evidence=evaluate_pair_clearance_evidence(self.model,self.data,(0,1),
                required_clearance_m=.03,query_cap_m=.03+1e-15)
            self.assertFalse(evidence.safe);self.assertEqual(evidence.status,ClearanceStatus.EXACT_FAIL)

    def test_false_zero_needs_certificate_and_penetration_cannot_be_overridden(self):
        with patch.object(mujoco,'mj_geomDistance',return_value=0.):
            evidence=evaluate_pair_clearance_evidence(self.model,self.data,(0,1),
                required_clearance_m=.03,query_cap_m=.05)
            self.assertTrue(evidence.safe);self.assertIsNone(evidence.exact_distance_m)
            self.assertEqual(evidence.status,ClearanceStatus.CERTIFIED_SAFE)
        with patch.object(mujoco,'mj_geomDistance',return_value=-.001):
            evidence=evaluate_pair_clearance_evidence(self.model,self.data,(0,1),
                required_clearance_m=.03,query_cap_m=.05)
            self.assertFalse(evidence.safe)
        self.data.geom_xpos[1]=[.02,0,0]
        with patch.object(mujoco,'mj_geomDistance',return_value=0.):
            evidence=evaluate_pair_clearance_evidence(self.model,self.data,(0,1),
                required_clearance_m=.03,query_cap_m=.05)
            self.assertFalse(evidence.safe);self.assertIsNone(evidence.exact_distance_m)
            self.assertEqual(evidence.status,ClearanceStatus.UNVERIFIABLE_FAIL)


class CarriedObjectPathTest(unittest.TestCase):
    @staticmethod
    def model(joint_type="hinge", extra="", masks="0 1"):
        contype, affinity=masks.split()
        bodies=''.join(f'<body name="j{i}" pos="{i+2} 0 0"><joint name="j{i}"/>'
                       '<geom type="sphere" size=".01" mass="1" contype="0" conaffinity="0"/></body>'
                       for i in range(1,12))
        actuators=''.join(f'<position joint="j{i}" ctrlrange="-2 2"/>' for i in range(12))
        return mujoco.MjModel.from_xml_string(f'''<mujoco model="desk_learning_OS30A_UNVERIFIED">
          <worldbody><body name="hand"><joint name="j0" type="{joint_type}" axis="0 0 1"/>
            <geom type="sphere" size=".01" mass="1" contype="0" conaffinity="0"/>
            <site name="left_cube_grasp" pos=".1 0 .3"/></body>{bodies}
            <geom name="table" type="box" size="1 1 .1" pos="0 0 -.1"/>
            <geom name="obstacle" type="sphere" size=".01" pos="0 .15 .3" contype="{contype}" conaffinity="{affinity}"/>
            <body name="payload" pos=".15 0 .3"><freejoint name="red_block_free"/>
              <geom name="red_block_geom" type="box" size=".02 .02 .02" mass="1"/>{extra}</body>
          </worldbody><actuator>{actuators}</actuator></mujoco>''')

    def check_path(self, model, data, goal):
        # Isolate new payload traversal from robot-specific CAD policy. Distances,
        # FK, payload transforms and every interpolation sample are real MuJoCo.
        with (patch("collision_guard.protected_geom_pairs", return_value=()),
              patch("collision_guard._collision_geoms_for_arm", return_value=()),
              patch("collision_guard.manipulation_pair_status", return_value=[]),
              patch("collision_guard.minimum_protected_clearance", side_effect=lambda m,d,p,**kw:
                    minimum_protected_clearance(m,d,p,**kw) if p else (.05,0,1))):
            return check_bimanual_path(model, np.zeros(12), goal, reference_data=data,
                task_phase="LIFT" if goal[0]>0 else "PLACE", carried_object=True,
                allow_sim_near_support=False)

    def test_rotation_moves_offset_cube_and_preserves_original_reference(self):
        model=self.model(); reference=mujoco.MjData(model)
        original=reference.qpos.copy()
        preview=mujoco.MjData(model); preview.qpos[:]=reference.qpos
        attachment=_carried_object_attachment(model,preview,np.zeros(12))
        goal=np.zeros(12); goal[0]=np.pi/2
        apply_control_as_pose(model,preview,goal,preserve_raw_pose=True)
        _apply_carried_object(model,preview,attachment)
        np.testing.assert_allclose(preview.geom("red_block_geom").xpos,[0,.15,.3],atol=1e-12)
        np.testing.assert_allclose(preview.geom("red_block_geom").xmat.reshape(3,3),
            [[0,-1,0],[1,0,0],[0,0,1]],atol=1e-12)
        np.testing.assert_array_equal(reference.qpos,original)

    def test_swept_cube_hits_one_way_mask_obstacle_that_static_cube_misses(self):
        for masks in ("0 1", "1 0"):
            model=self.model(masks=masks); data=mujoco.MjData(model); mujoco.mj_forward(model,data)
            initial=data.qpos.copy()
            block, obstacle=model.geom("red_block_geom").id,model.geom("obstacle").id
            self.assertGreater(mujoco.mj_geomDistance(model,data,block,obstacle,.5,None),.03)
            near_goal=np.zeros(12); near_goal[0]=.2
            clear=self.check_path(model,data,near_goal)
            self.assertTrue(clear.safe)
            self.assertGreaterEqual(clear.minimum_clearance_m,.03)
            self.assertFalse(clear.hardware_dispatch_authorized)
            goal=np.zeros(12); goal[0]=np.pi/2
            assessment=self.check_path(model,data,goal)
            self.assertFalse(assessment.safe)
            self.assertGreater(assessment.checked_samples,1)
            self.assertEqual({assessment.first_geom_id,assessment.second_geom_id},{block,obstacle})
            np.testing.assert_array_equal(data.qpos,initial)

    def test_support_intersection_and_extra_payload_shape_are_rejected(self):
        model=self.model(joint_type="slide"); data=mujoco.MjData(model)
        goal=np.zeros(12); goal[0]=-.3
        assessment=self.check_path(model,data,goal)
        self.assertFalse(assessment.safe)
        self.assertIn("support half-space",assessment.reason)
        self.assertLess(assessment.minimum_clearance_m,0.)
        # A tilted box must use its projected extent, not just nominal half-height.
        address=int(model.joint("red_block_free").qposadr[0])
        data.qpos[address+2]=.025
        data.qpos[address+3:address+7]=[np.cos(np.pi/8),0,np.sin(np.pi/8),0]
        mujoco.mj_forward(model,data)
        self.assertAlmostEqual(_carried_support_clearance(model,data,model.geom("red_block_geom").id,
            model.geom("table").id),.025-.02*np.sqrt(2))
        data.qpos[address+3:address+7]=0.
        with self.assertRaisesRegex(ValueError,"unit quaternion"):
            self.check_path(model,data,goal)
        data.qpos[address]=float("nan")
        with self.assertRaisesRegex(ValueError,"reference state must be finite"):
            self.check_path(model,data,goal)
        for extra in ('<geom type="sphere" size=".01" contype="0" conaffinity="1"/>',
                      '<body><geom type="sphere" size=".01" contype="1" conaffinity="0"/></body>'):
            model=self.model(extra=extra)
            with self.assertRaisesRegex(ValueError,"additional payload"):
                _carried_object_attachment(model,mujoco.MjData(model),np.zeros(12))


class CollisionGuardTest(unittest.TestCase):
    def test_infinite_plane_signed_distance_ignores_lateral_offset(self) -> None:
        model = mujoco.MjModel.from_xml_string("""
          <mujoco><worldbody>
            <geom name="plane" type="plane" size="0 0 .1"/>
            <body pos="10 20 .11"><freejoint/>
              <geom name="box" type="box" size=".1 .1 .1" mass="1"/>
            </body>
          </worldbody></mujoco>""")
        data = mujoco.MjData(model)
        for z, expected in ((.11, .01), (.10, 0.), (.09, -.01)):
            data.qpos[2] = z
            mujoco.mj_forward(model, data)
            for pair in ((0, 1), (1, 0)):
                with self.subTest(z=z, pair=pair):
                    native = mujoco.mj_geomDistance(model, data, *pair, 2., None)
                    self.assertAlmostEqual(native, expected, places=12)
                    details = []
                    minimum_protected_clearance(model, data, [pair], diagnostics=details)
                    self.assertFalse(details[0]["bounding_sphere_eligible"])
                    self.assertIsNone(details[0]["bounding_sphere_lower_bound_m"])
                    self.assertFalse(details[0]["mesh_certificate_eligible"])
                    self.assertAlmostEqual(details[0]["final_distance_m"], expected, places=12)
                    self.assertAlmostEqual(
                        minimum_protected_clearance(model, data, [pair])[0],
                        native, places=12)

    def test_saved_false_zero_poses(self) -> None:
        from shoe_task import ShoeTaskEnv
        fixture = json.loads((Path(__file__).parent / "fixtures/mesh_false_zero.json").read_text())
        model = ShoeTaskEnv().model
        for pose in fixture["poses"]:
            with self.subTest(time=pose["time_s"]):
                data = mujoco.MjData(model)
                data.qpos[:] = pose["qpos"]
                mujoco.mj_forward(model, data)
                pair = pose["pair"]
                native = mujoco.mj_geomDistance(model, data, *pair, .03, None)
                self.assertEqual(native, pose["native_distance_m"])
                bound = certified_separation_lower_bound(model, data, *pair)
                self.assertGreaterEqual(bound, pose["world_z_separation_m"] - 1e-12)
                self.assertGreater(minimum_protected_clearance(model, data, [pair])[0], 0)

    @classmethod
    def setUpClass(cls) -> None:
        cls.model, _ = build_model(arm_mount_height_m=0.30)

    def test_certified_separated_legacy_home_is_safe(self) -> None:
        data = mujoco.MjData(self.model)
        apply_control_as_pose(self.model, data, HUMANOID_HOME_ACTION)
        zero_pairs = [pair for pair in protected_geom_pairs(self.model)
                      if mujoco.mj_geomDistance(self.model, data, *pair, .03, None) == 0]
        self.assertTrue(zero_pairs)
        for pair in zero_pairs:
            self.assertGreater(certified_separation_lower_bound(self.model, data, *pair), 0)
        result = check_bimanual_path(
            self.model,
            HUMANOID_HOME_ACTION,
            HUMANOID_HOME_ACTION,
        )
        self.assertTrue(result.safe)
        self.assertGreaterEqual(result.minimum_clearance_m, .03)
        self.assertFalse(result.control_authorized)
        self.assertFalse(result.hardware_execution)

    def test_interpolated_arm_arm_collision_is_rejected(self) -> None:
        result = check_bimanual_path(
            self.model,
            HUMANOID_HOME_ACTION,
            UNSAFE_BIMANUAL_TARGET,
        )
        self.assertFalse(result.safe)
        self.assertLess(result.minimum_clearance_m, 0.03)
        self.assertFalse(result.hardware_dispatch_authorized)
        self.assertFalse(result.executed_action)

        data = mujoco.MjData(self.model)
        apply_control_as_pose(self.model, data, UNSAFE_BIMANUAL_TARGET)
        clearance, first, second = minimum_protected_clearance(
            self.model,
            data,
            bimanual_geom_pairs(self.model),
        )
        self.assertLess(clearance, 0.03)
        first_body = self.model.body(
            int(self.model.geom_bodyid[first])
        ).name
        second_body = self.model.body(
            int(self.model.geom_bodyid[second])
        ).name
        self.assertNotEqual(
            first_body.startswith("left_"),
            second_body.startswith("left_"),
        )

    def test_exact_same_arm_contact_is_rejected(self) -> None:
        calls = 0

        def clearance(*_args, **_kwargs):
            nonlocal calls
            calls += 1
            return ((0.0 if calls % 2 else 1.0), 1, 2)

        with patch(
            "collision_guard.minimum_protected_clearance",
            side_effect=clearance,
        ):
            result = check_bimanual_path(
                self.model,
                HUMANOID_HOME_ACTION,
                HUMANOID_HOME_ACTION,
            )

        self.assertFalse(result.safe)
        self.assertEqual(result.minimum_clearance_m, 0.0)
        self.assertIn("same-arm collision", result.reason)

    def test_saved_postphysics_wrist_gap_does_not_hide_invalid_floor_pose(self) -> None:
        from shoe_task import ShoeTaskEnv, UnsafeActionError
        from mobile_dual_so101 import actuator_targets_from_qpos

        env = ShoeTaskEnv()
        env.reset(seed=100)
        hold = actuator_targets_from_qpos(env.model, env.data.qpos)
        # This historical zero pose is not a valid physics start. Preserve the
        # original postphysics evidence without bypassing the repaired plane gate.
        with self.assertRaises(UnsafeActionError):
            env.apply_action(hold, physics_steps=34)
        self.assertEqual(env.data.time, 0)
        fixture = json.loads((Path(__file__).parent / "fixtures/mesh_false_zero.json").read_text())
        posed = mujoco.MjData(env.model)
        posed.qpos[:] = next(p["qpos"] for p in fixture["poses"] if abs(p["time_s"] - .068) < 1e-12)
        mujoco.mj_forward(env.model, posed)
        measured = actuator_targets_from_qpos(env.model, posed.qpos)
        geoms = []
        for mesh_name in ("left_sts3215_03a_no_horn_v1", "left_moving_jaw_so101_v1"):
            mesh_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_MESH, mesh_name)
            candidates = [i for i in _collision_geoms_for_arm(env.model, "left")
                          if env.model.geom_dataid[i] == mesh_id
                          and env.model.body(int(env.model.geom_bodyid[i])).name
                          in ("left_wrist", "left_moving_jaw_so101_v1")]
            self.assertEqual(len(candidates), 1)
            geoms.append(candidates[0])
        vertices = []
        for geom_id in geoms:
            mesh_id = int(env.model.geom_dataid[geom_id])
            start = env.model.mesh_vertadr[mesh_id]
            end = start + env.model.mesh_vertnum[mesh_id]
            vertices.append(env.model.mesh_vert[start:end] @ posed.geom_xmat[geom_id].reshape(3, 3).T
                            + posed.geom_xpos[geom_id])
        # Independent geometry evidence: every wrist vertex is above every jaw
        # vertex, even though MuJoCo 3.3.7 sometimes reports zero distance.
        separation = float(vertices[0][:, 2].min() - vertices[1][:, 2].max())
        self.assertGreater(separation, 0.011)
        clearance, _, _ = minimum_protected_clearance(env.model, posed, [tuple(geoms)])
        self.assertGreater(clearance, 0.0)
        self.assertFalse(check_bimanual_path(env.model, measured, hold).safe)

    def test_mesh_bound_does_not_erase_touching_or_penetration(self) -> None:
        model = mujoco.MjModel.from_xml_string("""
          <mujoco><asset><mesh name="cube" maxhullvert="4" vertex="
            -1 -1 -1  -1 -1 1  -1 1 -1  -1 1 1
             1 -1 -1   1 -1 1   1 1 -1   1 1 1"/></asset>
            <worldbody><geom type="mesh" mesh="cube"/>
              <body><freejoint/><geom type="mesh" mesh="cube"/></body>
            </worldbody></mujoco>""")
        data = mujoco.MjData(model)
        for x, returned in ((3.0, 0.0), (2.0, 0.0), (1.5, -0.5)):
            with self.subTest(x=x):
                data.qpos[0] = x
                mujoco.mj_forward(model, data)
                if x <= 2:
                    self.assertEqual(certified_separation_lower_bound(model, data, 0, 1), 0)
                    self.assertLessEqual(
                        minimum_protected_clearance(model, data, [(0, 1)])[0], 0)
                with patch("collision_guard.mujoco.mj_geomDistance", return_value=returned):
                    distance, _, _ = minimum_protected_clearance(model, data, [(0, 1)])
                if x > 2:
                    self.assertGreater(distance, 0.99)
                    self.assertLessEqual(distance, 1.0)
                else:
                    self.assertEqual(distance, returned)

    def test_floor_is_protected_from_every_arm_geom(self) -> None:
        floor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
        )
        pairs = {frozenset(pair) for pair in protected_geom_pairs(self.model)}
        for side in ("left", "right"):
            for arm_id in _collision_geoms_for_arm(self.model, side):
                self.assertIn(frozenset((arm_id, floor_id)), pairs)

    def test_every_moving_arm_geom_is_protected_from_every_base_geom(self) -> None:
        pairs = set(protected_geom_pairs(self.model))
        body_names = tuple(
            self.model.body(int(self.model.geom_bodyid[geom_id])).name
            for geom_id in range(self.model.ngeom)
        )
        arm_geoms = tuple(
            geom_id
            for geom_id, body_name in enumerate(body_names)
            if self.model.geom_contype[geom_id]
            and body_name.startswith(("left_", "right_"))
        )
        base_geoms = tuple(
            geom_id
            for geom_id, body_name in enumerate(body_names)
            if self.model.geom_contype[geom_id] and body_name.startswith("tb3_")
        )
        arm_body_names = {body_names[geom_id] for geom_id in arm_geoms}
        for side in ("left", "right"):
            for link in (
                "shoulder",
                "upper_arm",
                "lower_arm",
                "wrist",
                "gripper",
                "moving_jaw_so101_v1",
            ):
                self.assertIn(f"{side}_{link}", arm_body_names)
        self.assertEqual(len(arm_geoms), 26)
        self.assertEqual(len(base_geoms), 5)
        self.assertFalse(
            {
                (arm_geom, base_geom)
                for arm_geom in arm_geoms
                for base_geom in base_geoms
            }
            - pairs
        )

    def test_same_arm_non_adjacent_pairs_are_protected(self) -> None:
        body_pairs = {
            frozenset(
                self.model.body(int(self.model.geom_bodyid[geom_id])).name
                for geom_id in pair
            )
            for pair in protected_geom_pairs(self.model)
        }
        omitted_body_pairs = (
            ("left_shoulder", "left_wrist"),
            ("left_upper_arm", "left_gripper"),
            ("left_lower_arm", "left_moving_jaw_so101_v1"),
            ("right_shoulder", "right_wrist"),
            ("right_upper_arm", "right_gripper"),
            ("right_lower_arm", "right_moving_jaw_so101_v1"),
        )
        for pair in omitted_body_pairs:
            self.assertIn(frozenset(pair), body_pairs)
        for side in ("left", "right"):
            self.assertIn(
                frozenset((f"{side}_shoulder", f"{side}_gripper")),
                body_pairs,
            )
            self.assertNotIn(
                frozenset((f"{side}_shoulder", f"{side}_upper_arm")),
                body_pairs,
            )

    def test_step_support_home_protects_base_column_and_bare_camera(self) -> None:
        tower_model, _ = build_model(
            arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
            mount_layout="tower",
        )
        pairs = protected_geom_pairs(tower_model)
        protected_names = {
            tower_model.geom(geom_id).name
            for pair in pairs
            for geom_id in pair
            if tower_model.geom(geom_id).name
        }
        self.assertTrue(
            {
                "tb3_base_link_collision",
                "workspace_depth_camera_collision",
                "front_slam_depth_camera_collision",
                "semi_support_column_collision",
                "tower_camera_mast_collision",
                "tower_camera_interface_plate_collision",
            }.issubset(protected_names)
        )
        self.assertNotIn("semi_support_camera_post_collision", protected_names)
        self.assertNotIn("tower_camera_boom_collision", protected_names)
        result = check_bimanual_path(
            tower_model,
            HUMANOID_HOME_ACTION,
            HUMANOID_HOME_ACTION,
        )
        self.assertTrue(result.safe)
        self.assertGreaterEqual(result.minimum_clearance_m, 0.03)
        camera_id = mujoco.mj_name2id(
            tower_model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "workspace_depth_camera_collision",
        )
        camera_pairs = tuple(pair for pair in pairs if camera_id in pair)
        camera_data = mujoco.MjData(tower_model)
        apply_control_as_pose(
            tower_model, camera_data, HUMANOID_HOME_ACTION
        )
        camera_clearance, _, _ = minimum_protected_clearance(
            tower_model,
            camera_data,
            camera_pairs,
            distance_cap_m=2.0,
        )
        # The restored SO-101 Waveshare mounting plate is the intentional
        # nearest camera pair. Preserve at least a 100 mm nominal envelope;
        # the operational protected-path gate remains 30 mm above.
        self.assertGreaterEqual(
            camera_clearance,
            RESTORED_BASE_CAMERA_CLEARANCE_M,
        )
        self.assertFalse(result.hardware_execution)

    def test_printed_mount_proxies_are_protected_and_fail_closed(self) -> None:
        names = (
            "printed_mount_deck_collision",
            "printed_torso_collision",
            "printed_camera_mount_collision",
        )
        pairs = protected_geom_pairs(self.model)
        protected_names = {
            self.model.geom(geom_id).name
            for pair in pairs
            for geom_id in pair
        }
        self.assertTrue(set(names).issubset(protected_names))

        for name in names:
            geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, name
            )
            with self.subTest(inactive_geom=name):
                self.model.geom_contype[geom_id] = 0
                self.model.geom_conaffinity[geom_id] = 0
                try:
                    with self.assertRaisesRegex(
                        RuntimeError, "collision geometry is missing or inactive"
                    ):
                        protected_geom_pairs(self.model)
                finally:
                    self.model.geom_contype[geom_id] = 1
                    self.model.geom_conaffinity[geom_id] = 1

    def test_each_required_collision_geometry_is_fail_closed(self) -> None:
        model, _ = build_model(
            arm_mount_height_m=TOWER_RECOMMENDED_ARM_MOUNT_HEIGHT_M,
            mount_layout="tower",
        )
        required_ids = [
            *_collision_geoms_for_arm(model, "left"),
            *_collision_geoms_for_arm(model, "right"),
        ]
        for name in (
            "floor",
            "tb3_base_link_collision",
            "tb3_wheel_left_link_collision",
            "tb3_wheel_right_link_collision",
            "tb3_caster_back_right_link_collision",
            "tb3_caster_back_left_link_collision",
            "workspace_depth_camera_collision",
            "front_slam_depth_camera_collision",
            "semi_support_base_collision",
            "semi_support_column_collision",
            "tower_camera_mast_collision",
            "tower_camera_interface_plate_collision",
        ):
            required_ids.append(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            )

        for geom_id in required_ids:
            body_name = model.body(int(model.geom_bodyid[geom_id])).name
            geom_name = model.geom(geom_id).name or f"{body_name}[{geom_id}]"
            with self.subTest(inactive_geom=geom_name):
                contype = int(model.geom_contype[geom_id])
                conaffinity = int(model.geom_conaffinity[geom_id])
                model.geom_contype[geom_id] = 0
                model.geom_conaffinity[geom_id] = 0
                try:
                    with self.assertRaisesRegex(
                        RuntimeError,
                        "collision geometry is missing or inactive",
                    ):
                        protected_geom_pairs(model)
                finally:
                    model.geom_contype[geom_id] = contype
                    model.geom_conaffinity[geom_id] = conaffinity

    def test_invalid_clearance_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            check_bimanual_path(
                self.model,
                HUMANOID_HOME_ACTION,
                HUMANOID_HOME_ACTION,
                required_clearance_m=0.0,
            )


if __name__ == "__main__":
    unittest.main()
