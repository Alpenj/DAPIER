"""The read-only native adapter: what it reports, and that it never moves or opens a device. No network unless injected."""
import ast
import unittest
from pathlib import Path

import native_adapter as na
import sweepick_upper_body as ub
import test_sweepick_upper_body as T

IDLE = dict(state='IDLE', owner_alive=False, can_confirm_recovery=False, block_reason=None)
FAULT = dict(state='FAULT', owner_alive=False, can_confirm_recovery=True, block_reason='closing-sweep certificate mismatch')
ALL_LINKS = lambda path: True


class NativeAdapter(unittest.TestCase):
    def adapter(self, console, **k):
        return na.NativeArmAdapter(get=lambda url: console, exists=k.pop('exists', ALL_LINKS), **k)

    def test_a_fault_on_the_console_refuses_both_arms_mechanically(self):
        ev = self.adapter(FAULT).evaluate_arms({})
        self.assertIs(ev['left']['mechanical_state']['ok'], False)
        self.assertIn('closing-sweep', ev['left']['mechanical_state']['why'])

    def test_without_a_mount_record_or_console_the_state_is_unknown_not_ok(self):
        self.assertIsNone(self.adapter(IDLE).evaluate_arms({})['left']['mechanical_state']['ok'])
        self.assertIsNone(self.adapter(None).evaluate_arms({})['left']['mechanical_state']['ok'])
        ok = self.adapter(IDLE, mount_verification=dict(by='operator', ref='mount#1')).evaluate_arms({})
        self.assertIs(ok['left']['mechanical_state']['ok'], True)

    def test_a_missing_device_link_is_a_refusal(self):
        ev = self.adapter(IDLE, exists=lambda p: not p.endswith('right_wrist_rgb'), mount_verification=dict(ref='m')).evaluate_arms({})
        self.assertIs(ev['right']['mechanical_state']['ok'], False); self.assertIs(ev['left']['mechanical_state']['ok'], True)

    def test_unrun_planner_checks_stay_unknown_and_the_right_arm_is_not_real_eligible(self):
        ev = self.adapter(IDLE, mount_verification=dict(ref='m')).evaluate_arms({})
        self.assertTrue(all(ev['left'][k]['ok'] is None for k in ('approach_ik', 'collision_and_view', 'grasp_room', 'transport_pose', 'place_pose')))
        self.assertIs(ev['right']['verified_capability']['eligible_for_REAL'], False)
        self.assertEqual(ub.select_arm(ev, 'REAL')['decision'], ub.ARM_NOT_READY)

    def test_every_motion_is_refused_without_moving_and_the_mission_ends_not_ready(self):
        mounted = dict(left={k: dict(ok=True, cost=dict(value=1, definition='d', source='s')) for k in ('approach_ik', 'collision_and_view', 'grasp_room', 'transport_pose', 'place_pose')})
        a = self.adapter(IDLE, mount_verification=dict(ref='m'), mounted=mounted)
        for r in (a.pick('left', lambda: True), a.transport_pose('left', lambda: True), a.place_plan('left', {}), a.approach_place('left', lambda: True, {}), a.release('left', lambda: True), a.retreat('left', lambda: True)):
            self.assertEqual((r['moved'], r['refused'][:10]), (False, 'NOT_WIRED:'))
        m = ub.UpperBodyMission(T.TASK, a, mode='REAL', native_manifest_sha256=T.MANIFEST, clock_id=T.CLOCK)
        m.receive(T.base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
        out = m.pick_at_stop(T.auth(source_kind='REAL'), T.look(), 12.0)
        self.assertEqual(out[0]['kind'], ub.ARM_NOT_READY); self.assertIn('NOT_WIRED', out[0]['reason'])
        self.assertEqual(a.calls, ['pick', 'transport_pose', 'place_plan', 'approach_place', 'release', 'retreat', 'evaluate_arms', 'pick'])

    def test_the_adapter_imports_no_device_or_motor_code(self):
        tree = ast.parse(Path(na.__file__).read_text())
        names = {a.name.split('.')[0] for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom)) for a in n.names}
        names |= {n.module.split('.')[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
        self.assertFalse(names & {'serial', 'lerobot', 'scservo_sdk', 'sweepick_move_a', 'sweepick_pick01', 'sweepick_real_command', 'cv2'}, names)
        src = Path(na.__file__).read_text()
        self.assertNotIn('open(', src.replace('urlopen(', ''))


if __name__ == '__main__':
    unittest.main()
