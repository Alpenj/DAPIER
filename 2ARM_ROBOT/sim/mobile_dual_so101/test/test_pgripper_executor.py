"""The expert and policy must share target hold, units and slew semantics."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import numpy as np
import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pgripper_handover import PolicyTargetExecutor, contract_vector
from pgripper_execution import configure_physics, step_physics


class ExecutorTest(unittest.TestCase):
    def test_three_clocks_reset_and_explicit_raw_range_policy(self):
        model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint name="j" type="slide"/><geom size=".1"/></body></worldbody></mujoco>')
        configure_physics(model, 2)
        data = mujoco.MjData(model)
        stub = SimpleNamespace(opt=model.opt, actuator_ctrlrange=np.tile([0., 2.], (12, 1)))
        executor = PolicyTargetExecutor(stub, 25)
        current = np.zeros(12)
        for _ in range(20):
            previous = current.copy()
            current = executor.step(current, np.ones(12))
            self.assertLessEqual(np.max(np.abs(current-previous)), .0012 + 1e-12)
            step_physics(model, data, 2)
        self.assertAlmostEqual(data.time, .04, places=12)
        self.assertEqual(executor.steps, 20)
        self.assertEqual(model.opt.timestep, .001)
        executor.hold(current)
        executor.reset()
        self.assertEqual(executor.steps, 0)
        self.assertIsNone(executor.target)
        self.assertEqual(executor.hold_events, [])
        for policy in ('reject', 'project'):
            executor = PolicyTargetExecutor(stub, 25, range_policy=policy)
            raw = np.zeros(12); raw[5] = 1.2
            if policy == 'reject':
                with self.assertRaises(ValueError):
                    executor.step(np.zeros(12), raw)
                self.assertEqual(executor.steps, 0)
            else:
                executor.step(np.zeros(12), raw)
                self.assertEqual(executor.target[5], 2.)
            self.assertEqual(executor.range_events[0]['axes'], [5])
            for value in (np.nan, np.inf, -np.inf):
                before = executor.steps
                with self.assertRaises(ValueError):
                    executor.step(current, np.full(12, value))
                self.assertEqual(executor.steps, before)
        for bad in (0, 3, True):
            with self.assertRaises(ValueError):
                configure_physics(model, bad)
        with self.assertRaises(ValueError):
            step_physics(model, data, 1)
        with self.assertRaises(ValueError):
            PolicyTargetExecutor(stub, 25, command_dt_s=.0015)

    def test_small_policy_target_is_ramped_over_one_frame(self):
        model = SimpleNamespace(opt=SimpleNamespace(timestep=.002),
            actuator_ctrlrange=np.tile([0., 2.], (12, 1)))
        executor = PolicyTargetExecutor(model, 25)
        goal = np.full(12, .01)
        current = np.zeros(12)
        for step in range(20):
            current = executor.step(current, contract_vector(model, goal))
            np.testing.assert_allclose(current, goal * (step + 1) / 20, atol=1e-9)
        dense = PolicyTargetExecutor(model, 500)
        np.testing.assert_allclose(dense.step(np.zeros(12), contract_vector(model, goal)), .0012)

    def test_safety_hold_cancels_cached_approach_mid_interval(self):
        model = SimpleNamespace(opt=SimpleNamespace(timestep=.002),
            actuator_ctrlrange=np.tile([0., 2.], (12, 1)))
        executor = PolicyTargetExecutor(model, 25)
        current = np.zeros(12)
        for _ in range(7):
            current = executor.step(current, np.ones(12))
        frozen = current.copy()
        executor.hold(current)
        self.assertEqual(executor.steps, 7)
        for _ in range(13):
            current = executor.step(current, contract_vector(model, frozen))
            np.testing.assert_array_equal(current, frozen)
        with self.assertRaises(ValueError):
            executor.hold(np.full(12, np.nan))

    def test_policy_clock_and_rate_limit(self):
        model = SimpleNamespace(opt=SimpleNamespace(timestep=.002),
            actuator_ctrlrange=np.tile([-2., 2.], (12, 1)))
        model.actuator_ctrlrange[[5, 11]] = [0., 2.2028]
        executor = PolicyTargetExecutor(model, 25)
        ctrl = np.zeros(12)
        proposal = np.full(12, .5)
        for i in range(20):
            previous = ctrl.copy()
            ctrl = executor.step(ctrl, proposal if i == 0 else np.zeros(12))
            np.testing.assert_allclose(executor.target, contract_vector(model, proposal, inverse=True))
            self.assertLessEqual(float(np.max(np.abs(ctrl - previous))), .0012 + 1e-12)
        ctrl = executor.step(ctrl, np.zeros(12))
        np.testing.assert_array_equal(executor.target, np.zeros(12))
        self.assertAlmostEqual(ctrl[0], .0228)
        before = executor.steps
        with self.assertRaises(ValueError):
            executor.step(ctrl, np.full(12, np.nan))
        self.assertEqual(executor.steps, before)
        with self.assertRaises(ValueError):
            PolicyTargetExecutor(model, 30)


if __name__ == '__main__':
    unittest.main()
