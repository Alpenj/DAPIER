"""MOCK / OFFLINE regression of the upper-body mission contract. No device, no native session, no base."""
import copy
import unittest

import sweepick_upper_body as ub

TASK, CLOCK, MANIFEST = 'task-1', 'robot-host-mono', 'a' * 64


def base_msg(kind, seq, stamp, **extra):
    fields = dict(stop_location_id='PICKUP_1', base_stopped_proof='odom-zero-twist#12', controller_idle_proof='nav2-idle#12')
    if kind == ub.BASE_PARKED_AT_DISPOSAL:
        fields.update(stop_location_id='DISPOSAL_2', disposal_area_id='BIN_A', local_observation_permitted=True)
    if kind == ub.BASE_MOVING:
        fields = {}
    if kind == ub.BASE_STOP_OR_FAULT:
        fields = dict(reason='nav2 abort')
    fields.update(extra)
    status = 'PARKED' if kind.startswith('BASE_PARKED') else 'MOVING' if kind == ub.BASE_MOVING else 'STOPPED'
    return ub.message(kind, TASK, seq, 'none', source_kind='MOCK', source_identity='base_team_mock', stamp=stamp, clock_id=CLOCK,
                      status=status, proof_ref=f'base#{seq}', **fields)


def auth(**over):
    a = dict(scope=ub.AUTH_SCOPE, native_manifest_sha256=MANIFEST, supervised=True, recovery_state='NO_OPEN_FAULT', robot_state_ref='console-status#1')
    a.update(over)
    msg = dict(schema=ub.SCHEMA, kind=ub.OPERATOR_AUTHORIZATION, task_id=a.pop('task_id', TASK), sequence_id=1, side='none', source_kind=a.pop('source_kind', 'MOCK'),
               source_identity=a.pop('source_identity', 'operator'), stamp=1.0, clock_id=CLOCK, status='APPROVED', proof_ref='approval#1', **a)
    return msg                                                                       # built raw: a malformed approval must reach the mission's own check


def check(ok=True, **k):
    return dict(ok=ok, **k)


def arm(capable=True, real=True, travel=100, **over):
    c = dict(verified_capability=check(capable, eligible_for_REAL=real), mechanical_state=check(), approach_ik=check(joint_travel_ticks=travel),
             collision_and_view=check(), grasp_room=check(), transport_pose=check(), place_pose=check())
    c.update(over)
    return c


class Mock:
    def __init__(self, evaluations=None, **results):
        self.ev = evaluations or dict(left=arm(), right=arm(real=False))
        self.r = dict(pick=dict(held='CONFIRMED', lifted=True, moved=True, evidence=dict(stamp=12.0, clock_id=CLOCK, ref='pick-look-04')),
                      transport=dict(reached=True, readback_ok=True, gripper_hold='CONFIRMED', interference_review='PASS', torque='ON', evidence=dict(stamp=13.0, clock_id=CLOCK, ref='transport#1')),
                      hold=dict(readback_ok=True, gripper_hold='CONFIRMED'),
                      place=dict(approached=True, support_and_space='CONFIRMED', opened_ack=True, separated='OBSERVED', in_target='OBSERVED', retreated=True, evidence=dict(stamp=31.0, clock_id=CLOCK, ref='release#1')),
                      safe_end=dict(torque_state='OFF_CONFIRMED', holding_state='EMPTY'))
        self.r.update(results); self.calls = []; self.before_motion = None

    def evaluate_arms(self, observation):
        self.calls.append('evaluate'); return copy.deepcopy(self.ev)

    def _motion(self, name, still_allowed):
        self.calls.append(name)
        if self.before_motion:
            self.before_motion(name)
        if not still_allowed():                                                    # the REAL adapter checks right before any write
            return dict(refused='still_allowed() false at the time of use', moved=False)
        return copy.deepcopy(self.r[name])

    def pick(self, side, still_allowed): return self._motion('pick', still_allowed)
    def transport_pose(self, side, still_allowed): return self._motion('transport', still_allowed)
    def hold_readback(self, side): self.calls.append('hold'); return copy.deepcopy(self.r['hold'])
    def place(self, side, still_allowed, target): return self._motion('place', still_allowed)
    def safe_end(self, side): self.calls.append('safe_end'); return copy.deepcopy(self.r['safe_end'])


def look(stamp=11.0, **over):
    o = dict(stamp=stamp, clock_id=CLOCK, stop_location_id='PICKUP_1', camera_fresh=True)
    o.update(over); return o


def disposal_look(stamp=30.0, **over):
    o = dict(stamp=stamp, clock_id=CLOCK, stop_location_id='DISPOSAL_2', disposal_area_id='BIN_A', camera_fresh=True, target_area='OBSERVED')
    o.update(over); return o


def mission(mock=None, mode='MOCK'):
    m = ub.UpperBodyMission(TASK, mock or Mock(), mode=mode, native_manifest_sha256=MANIFEST, clock_id=CLOCK)
    return m, m.a


def run_to_transport(m):
    m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
    m.pick_at_stop(auth(), look(), 12.5)
    return m.secure_transport(13.5)


class Schema(unittest.TestCase):
    def test_every_message_carries_the_required_identity_and_a_bare_arrived_is_refused(self):
        good = base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0)
        self.assertEqual(ub.validate(good, inbound=True), [])
        bare = dict(kind=ub.BASE_PARKED_AT_PICKUP, task_id=TASK, arrived=True)
        errs = ub.validate(bare, inbound=True)
        for field in ('sequence_id', 'stamp', 'clock_id', 'source_identity', 'proof_ref'):
            self.assertTrue(any(field in e for e in errs), field)
        no_proof = dict(good); no_proof.pop('controller_idle_proof')
        self.assertIn('BASE_PARKED_AT_PICKUP needs controller_idle_proof', ub.validate(no_proof, inbound=True))
        self.assertTrue(ub.validate(dict(good, stamp=float('nan'))))
        self.assertTrue(ub.validate(dict(good, kind=ub.ARM_TRANSPORT_READY), inbound=True))           # an outbound kind is not accepted inbound

    def test_base_view_ignores_other_tasks_old_sequences_and_clock_changes(self):
        v = ub.BaseView(TASK)
        self.assertTrue(v.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 2, 10.0)))
        self.assertFalse(v.receive(base_msg(ub.BASE_MOVING, 1, 11.0)))                                  # older sequence
        other = base_msg(ub.BASE_MOVING, 3, 11.0); other['task_id'] = 'task-2'
        self.assertFalse(v.receive(other))
        clock = base_msg(ub.BASE_MOVING, 4, 11.0); clock['clock_id'] = 'other-clock'
        self.assertFalse(v.receive(clock))
        self.assertIsNone(v.blocks_arm_motion()); self.assertEqual(len(v.rejected), 3)


class SelectArm(unittest.TestCase):
    def test_left_fit_right_unfit(self):
        d = ub.select_arm(dict(left=arm(), right=arm(collision_and_view=check(False, why='right wrist sees the block cut'))), 'MOCK')
        self.assertEqual((d['selected'], d['arms']['right']['decided_by']), ('left', 'collision_and_view'))

    def test_right_fit_left_unfit_in_mock_and_right_is_never_REAL_eligible_yet(self):
        ev = dict(left=arm(mechanical_state=check(False, why='left FAULT not recovered')), right=arm(real=False))
        self.assertEqual(ub.select_arm(ev, 'MOCK')['selected'], 'right')
        real = ub.select_arm(ev, 'REAL')
        self.assertIsNone(real['selected']); self.assertEqual(real['decision'], ub.ARM_NOT_READY)
        self.assertIn('eligible_for_REAL=false', real['arms']['right']['why'])

    def test_both_fit_are_compared_by_planner_travel_not_guessed(self):
        d = ub.select_arm(dict(left=arm(travel=300), right=arm(travel=120)), 'MOCK')
        self.assertEqual((d['selected'], d['comparison']), ('right', dict(left=300, right=120)))
        no_travel = ub.select_arm(dict(left=arm(travel=None), right=arm(travel=120)), 'MOCK')
        self.assertIsNone(no_travel['selected'])
        tie = ub.select_arm(dict(left=arm(travel=120), right=arm(travel=120)), 'MOCK')
        self.assertEqual(tie['selected'], 'left')

    def test_both_refused_or_unknown_is_arm_not_ready(self):
        d = ub.select_arm(dict(left=arm(approach_ik=check(False, why='IK: no solution')), right=dict()), 'MOCK')
        self.assertEqual(d['decision'], ub.ARM_NOT_READY)
        self.assertEqual((d['arms']['left']['status'], d['arms']['right']['status']), ('REFUSED', 'UNKNOWN'))


class Mission(unittest.TestCase):
    def test_the_whole_mock_task_and_no_mission_completion_by_the_upper_body(self):
        m, a = mission()
        self.assertEqual([x['kind'] for x in run_to_transport(m)], [ub.ARM_TRANSPORT_READY])
        m.receive(base_msg(ub.BASE_MOVING, 2, 20.0))
        self.assertEqual(m.hold_check(21.0), [])
        m.receive(base_msg(ub.BASE_PARKED_AT_DISPOSAL, 3, 29.0))
        out = m.place_at_stop(disposal_look(), 32.0)
        self.assertEqual([x['kind'] for x in out], [ub.ARM_DISPOSAL_CONFIRMED])
        s = m.summary()
        self.assertEqual(s['sent'], [ub.ARM_PICK_CONFIRMED, ub.ARM_TRANSPORT_READY, ub.ARM_DISPOSAL_CONFIRMED])
        self.assertIs(s['mission_completed_by_upper_body'], False)
        self.assertTrue(all(x['source_kind'] == 'MOCK' and not ub.validate(x, inbound=False) for x in m.sent))

    def test_base_moving_stop_or_fault_blocks_pick_place_and_arm_motion(self):
        for kind in (ub.BASE_MOVING, ub.BASE_STOP_OR_FAULT):
            m, a = mission(); m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0)); m.receive(base_msg(kind, 2, 10.5))
            out = m.pick_at_stop(auth(), look(), 11.5)
            self.assertEqual(out[0]['kind'], ub.ARM_NOT_READY); self.assertNotIn('pick', a.calls)
        m, a = mission(); run_to_transport(m); m.receive(base_msg(ub.BASE_STOP_OR_FAULT, 2, 20.0))
        out = m.place_at_stop(disposal_look(), 32.0)
        self.assertEqual(out[0]['kind'], ub.ARM_NOT_READY); self.assertNotIn('place', a.calls)
        self.assertEqual(out[-1]['kind'], ub.BASE_BLOCK_REQUEST)

    def test_time_of_check_vs_time_of_use(self):
        m, a = mission()
        def base_starts(name):
            if name == 'pick':
                m.receive(base_msg(ub.BASE_MOVING, 2, 11.8))
        a.before_motion = base_starts
        m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
        out = m.pick_at_stop(auth(), look(), 12.0)
        self.assertEqual(out[0]['kind'], ub.ARM_NOT_READY); self.assertIn('still_allowed', out[0]['reason'])

    def test_transport_readback_loss_is_never_success_and_asks_the_base_to_block(self):
        m, a = mission(Mock(hold=dict(readback_ok=False, gripper_hold='UNKNOWN')))
        run_to_transport(m); m.receive(base_msg(ub.BASE_MOVING, 2, 20.0))
        out = m.hold_check(21.0)
        self.assertEqual([x['kind'] for x in out], [ub.ARM_FAULT, ub.BASE_BLOCK_REQUEST])
        self.assertEqual(m.place_at_stop(disposal_look(), 32.0)[0]['kind'], ub.ARM_NOT_READY)
        self.assertNotIn(ub.ARM_DISPOSAL_CONFIRMED, m.summary()['sent'])

    def test_observations_and_parks_of_another_place_or_time_are_not_reused(self):
        m, a = mission(); m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
        self.assertEqual(m.pick_at_stop(auth(), look(stamp=9.0), 12.0)[0]['kind'], ub.ARM_NOT_READY)          # before the stop
        m, a = mission(); m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
        self.assertEqual(m.pick_at_stop(auth(), look(clock_id='other'), 12.0)[0]['kind'], ub.ARM_NOT_READY)   # other clock domain
        m, a = mission(); m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
        self.assertEqual(m.pick_at_stop(auth(), look(camera_fresh=False), 12.0)[0]['kind'], ub.ARM_NOT_READY)  # camera stale
        m, a = mission(); run_to_transport(m); m.receive(base_msg(ub.BASE_PARKED_AT_DISPOSAL, 2, 29.0))
        self.assertEqual(m.place_at_stop(disposal_look(stamp=12.0), 32.0)[0]['kind'], ub.ARM_NOT_READY)       # the pickup-time look
        m, a = mission(); run_to_transport(m); m.receive(base_msg(ub.BASE_PARKED_AT_DISPOSAL, 2, 29.0, stop_location_id='PICKUP_1'))
        self.assertIn('pickup stop', m.place_at_stop(disposal_look(stop_location_id='PICKUP_1'), 32.0)[0]['reason'])

    def test_lift_alone_is_not_completion_and_an_open_ack_alone_is_not_a_disposal(self):
        m, a = mission(); m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
        out = m.pick_at_stop(auth(), look(), 12.5)
        self.assertEqual(out[0]['kind'], ub.ARM_PICK_CONFIRMED); self.assertIn('not a mission completion', out[0]['note'])
        for res in (dict(separated='UNKNOWN', in_target='UNKNOWN'), dict(separated='OBSERVED', in_target='UNKNOWN')):
            m, a = mission(Mock(place=dict(Mock().r['place'], **res))); run_to_transport(m); m.receive(base_msg(ub.BASE_PARKED_AT_DISPOSAL, 2, 29.0))
            out = m.place_at_stop(disposal_look(), 32.0)
            self.assertEqual(out[0]['kind'], ub.ARM_RECOVERY_REQUIRED); self.assertIn('open command/ACK alone', out[0]['reason'])

    def test_a_pick_from_the_past_is_not_a_held_confirmation(self):
        m, a = mission(Mock(pick=dict(held='CONFIRMED', lifted=True, moved=True, evidence=dict(stamp=5.0, clock_id=CLOCK, ref='old HELD_CONFIRMED'))))
        m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
        self.assertEqual(m.pick_at_stop(auth(), look(), 12.0)[0]['kind'], ub.ARM_RECOVERY_REQUIRED)

    def test_stop_and_faults_at_each_stage_keep_unknown_and_block_the_base(self):
        cases = dict(pick=dict(stopped=True), transport=dict(faults={'ID3': 'status 0x20'}), hold=dict(owner_conflict='another console owner'))
        for stage, bad in cases.items():
            m, a = mission(Mock(**{stage: bad})); m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
            out = m.pick_at_stop(auth(), look(), 12.0)
            if stage != 'pick':
                out = m.secure_transport(13.0)
            if stage == 'hold':
                m.receive(base_msg(ub.BASE_MOVING, 2, 20.0)); out = m.hold_check(21.0)
            self.assertIn(out[0]['kind'], (ub.ARM_STOPPED, ub.ARM_RECOVERY_REQUIRED, ub.ARM_FAULT), stage)
            self.assertEqual(out[-1]['kind'], ub.BASE_BLOCK_REQUEST, stage)
            self.assertEqual(out[0].get('holding_state'), ub.UNKNOWN, stage)
        m, a = mission(); run_to_transport(m)
        out = m.stop(20.0)
        self.assertEqual([x['kind'] for x in out], [ub.ARM_STOPPED, ub.BASE_BLOCK_REQUEST]); self.assertFalse(m.still_allowed())

    def test_ik_or_collision_refusal_and_readback_mismatch(self):
        m, a = mission(Mock(pick=dict(refused='plan_pick: IK no solution', moved=False)))
        m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
        self.assertEqual(m.pick_at_stop(auth(), look(), 12.0)[0]['kind'], ub.ARM_NOT_READY)
        m, a = mission(Mock(transport=dict(refused='collision certificate CONTACT', moved=False)))
        self.assertEqual(run_to_transport(m)[0]['kind'], ub.ARM_RECOVERY_REQUIRED)                         # already holding: a person decides
        m, a = mission(Mock(transport=dict(readback_mismatch='ID2 actual 2310 vs receipt 2352')))
        self.assertIn('readback', run_to_transport(m)[0]['reason'])

    def test_unverified_transport_pose_is_not_transport_ready(self):
        for missing in ('reached', 'interference_review', 'gripper_hold'):
            t = dict(Mock().r['transport']); t[missing] = None
            m, a = mission(Mock(transport=t))
            out = run_to_transport(m)
            self.assertEqual(out[0]['kind'], ub.ARM_RECOVERY_REQUIRED, missing); self.assertNotIn(ub.ARM_TRANSPORT_READY, m.summary()['sent'])

    def test_authorization_is_the_operators_for_this_task_and_this_source(self):
        bad = dict(other_task=auth(task_id='task-9'), other_manifest=auth(native_manifest_sha256='b' * 64), unsupervised=auth(supervised=False),
                   open_fault=auth(recovery_state='FAULT_OPEN_121735'), from_base=auth(source_identity='base_team_mock'), wrong_scope=auth(scope='LEFT_START_TO_LIFT'))
        for name, a_ in bad.items():
            m, a = mission(); m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
            out = m.pick_at_stop(a_, look(), 12.0)
            self.assertEqual(out[0]['kind'], ub.ARM_NOT_READY, name); self.assertNotIn('pick', a.calls)
        m, a = mission(mode='REAL'); m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
        self.assertIn('REAL operator authorization', m.pick_at_stop(auth(), look(), 12.0)[0]['reason'])

    def test_real_mode_with_only_the_unverified_right_arm_fit_never_moves(self):
        ev = dict(left=arm(approach_ik=check(False, why='IK')), right=arm(real=False))
        m, a = mission(Mock(evaluations=ev), mode='REAL'); m.receive(base_msg(ub.BASE_PARKED_AT_PICKUP, 1, 10.0))
        out = m.pick_at_stop(auth(source_kind='REAL'), look(), 12.0)
        self.assertEqual(out[0]['kind'], ub.ARM_NOT_READY); self.assertNotIn('pick', a.calls)


if __name__ == '__main__':
    unittest.main()
