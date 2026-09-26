"""Native joint-tube clearance certificate on an observed-support scene (no hardware)."""
import math
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collision_guard import (NATIVE_COMMAND_HORIZON_S, _collision_geoms_for_arm, _same_arm_geom_pairs,
    check_tracking_envelope, minimum_protected_clearance, protected_geom_pairs)
from evaluate_single_shot_ik import bind_observed_block, observed_task_env
from mobile_dual_so101 import HUMANOID_HOME_ACTION, apply_control_as_pose


class TrackingEnvelopeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        datum = [.0388353, 0., .0254]
        block = {"scene_support":{"frame":"left_motor1_datum", "normal_xyz":[0., 0., 1.],
                                  "revision":"MOCK measured support", "top_z_m":-.04},
                 "scene_object":{"frame":"left_motor1_datum", "position_semantics":"object_center",
                                 "center_xyz_m":[.24, -.14, -.02], "size_m":[.04]*3,
                                 "quaternion_wxyz":[1., 0., 0., 0.]}}
        env, _ = observed_task_env(block, datum)
        cls.model, cls.data = env.model, env.data
        bind_observed_block(cls.model, cls.data, block, datum)
        mujoco.mj_forward(cls.model, cls.data)
        cls.start = np.array(HUMANOID_HOME_ACTION, dtype=float)
        cls.start[5], cls.start[11] = .3, 0.
        cls.goal = cls.start.copy()
        cls.goal[:6] += [.15, .1, -.1, .1, .2, .4]
        cls.report = check_tracking_envelope(cls.model, cls.start, cls.goal,
            reference_data=cls.data, max_velocity_rad_s=[.1]*6)

    def test_random_tube_configurations_respect_the_certified_bound(self):
        report, model = self.report, self.model
        self.assertTrue(report["model_tube_verified"], report)
        # The model result never becomes the execution prerequisite by itself.
        self.assertFalse(report["verified"])
        self.assertTrue(report["execution_prerequisites_unmet"])
        self.assertGreaterEqual(report["minimum_clearance_m"], .030)
        self.assertEqual(report["segment_goal_rad"], self.goal[:6].tolist())
        bottleneck = report["bottleneck"]
        self.assertEqual(bottleneck["pair"], report["minimum_clearance_pair"])
        self.assertTrue(0. <= bottleneck["path_fraction"] <= 1.)
        tolerance = report["tracking_tolerance_rad"]
        delta = self.goal[:6] - self.start[:6]
        # Independent recomputation of the native tube: tolerance + limiter lead + sample gap.
        lead = np.abs(delta) * min(1., np.min(.1*NATIVE_COMMAND_HORIZON_S/np.abs(delta)))
        error = tolerance + lead + np.abs(delta)/(2*(report["checked_samples"]-1))
        left, right = _collision_geoms_for_arm(model, "left"), _collision_geoms_for_arm(model, "right")
        same_left = list(_same_arm_geom_pairs(model, left))
        same = set(same_left) | set(_same_arm_geom_pairs(model, right))
        block = model.geom("red_block_geom").id
        general = [p for p in dict.fromkeys([*protected_geom_pairs(model), *((g, block) for g in (*left, *right))])
                   if p not in same and (p[0] in left or p[1] in left)]
        data = mujoco.MjData(model)
        data.qpos[:] = self.data.qpos
        rng = np.random.default_rng(20260926)
        low, high = model.actuator_ctrlrange[:6].T
        worst_general = worst_self = math.inf
        for _ in range(150):
            action = self.start + rng.uniform() * (self.goal - self.start)
            action[:6] = np.clip(action[:6] + rng.uniform(-error, error), low, high)
            apply_control_as_pose(model, data, action, preserve_raw_pose=True)
            worst_general = min(worst_general, minimum_protected_clearance(model, data, general, distance_cap_m=.2)[0])
            worst_self = min(worst_self, minimum_protected_clearance(model, data, same_left, distance_cap_m=.2)[0])
        self.assertGreaterEqual(worst_general, report["minimum_clearance_m"] - 1e-9)
        self.assertGreater(worst_self, 0.)

    def test_faster_velocity_cap_never_widens_the_certified_tube(self):
        fast = check_tracking_envelope(self.model, self.start, self.goal,
                                       reference_data=self.data, max_velocity_rad_s=[.3]*6)
        self.assertFalse(fast["verified"])
        if fast["model_tube_verified"]:
            self.assertLessEqual(fast["tracking_tolerance_rad"], self.report["tracking_tolerance_rad"])
        self.assertFalse(fast["hardware_execution"])

    def test_refuses_moving_passive_arm_contact_phase_and_uncapped_velocity(self):
        moved = self.goal.copy()
        moved[6] += .01
        cases = ((moved, {}), (self.goal, {"task_phase":"LIFT"}), (self.goal, {"max_velocity_rad_s":[.31]*6}),
                 # An allowance below the supervised-trial minimum would look like a trial certificate.
                 (self.goal, {"excursion_allowance_rad":.02}), (self.goal, {"excursion_allowance_rad":.2}))
        for goal, extra in cases:
            kwargs = {"reference_data":self.data, "max_velocity_rad_s":[.1]*6, **extra}
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                check_tracking_envelope(self.model, self.start, goal, **kwargs)


    def test_trial_allowance_changes_the_prerequisite_not_the_monitor(self):
        trial = check_tracking_envelope(self.model, self.start, self.start + .3*(self.goal-self.start),
            reference_data=self.data, max_velocity_rad_s=[.15]*6, excursion_allowance_rad=.05)
        self.assertFalse(trial["verified"])
        self.assertEqual(trial["excursion_allowance_rad"], .05)
        self.assertEqual(trial["execution_prerequisites_unmet"],
                         ["supervised trial field conditions not confirmed by an operator record"])
        self.assertIn("firmware speed-cap adherence unverified", trial["accepted_assumptions"][0])
        self.assertLessEqual(trial.get("tracking_tolerance_rad", .004), .004)


if __name__ == "__main__":
    unittest.main()
