"""2026-10-15: an accepted OBSERVATION_VIEW that settled short (20261015_140844_3cc05c observe_0_0/0_1, elbow +26/+23)
is recorded as a diagnostic, not the run's first error. Other TARGET_NOT_REACHED results stay faults."""
import json
from pathlib import Path
import unittest

import test_records
from records import atomic_json, run_record, save_review
from replay import settled_short_view

VIEW = {'nominal_target_reached': False, 'command_completed': True, 'actual_settled': True, 'command_state_resolved': True}
ZERO = {n: 0.0 for n in ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')}


class SettledShortView(unittest.TestCase):
    setUp = test_records.RunEvidence.setUp

    def stage(self, name, view, status=ZERO, result='TARGET_NOT_REACHED_HOLDING'):
        folder = self.root / 'handoff' / name / 'run'; folder.mkdir(parents=True)
        move = {'result': result, 'end': {'ticks': dict(ZERO), 'status': status}, 'rows': [],
                'reached': {'elbow_flex': {'minus_target': 26.0, 'within_tolerance': False, 'settled': True, 'reached': False}}}
        if view is not None:
            move['observation_view'] = view
        atomic_json(folder / 'move.json', move)
        return {'stage': 'handoff_' + name, 'side': 'right', 'result': result, 't0_host': 101.0, 't_host_s': 101.2, 'folder': str(folder.parent)}

    def review(self, stages):
        ep = json.loads((self.root / 'episode/EPISODE.json').read_text()) if (self.root / 'episode/EPISODE.json').exists() else {'data_quality': {}}
        ep['stages'] = list(ep.get('stages', [])) + stages; atomic_json(self.root / 'episode/EPISODE.json', ep)
        record, evidence = run_record(self.directory)
        save_review(self.directory, record, evidence)
        return json.loads((self.directory / 'RUN_RECORD.json').read_text()), json.loads((self.directory / 'AI_REVIEW.json').read_text())

    def test_predicate_matches_the_owner_acceptance(self):
        self.assertTrue(settled_short_view({'observation_view': VIEW, 'end_status': ZERO}))
        for view, status in ((dict(VIEW, command_state_resolved=False), ZERO), (dict(VIEW, actual_settled=False), ZERO),
                             (VIEW, dict(ZERO, elbow_flex=32.0)), (None, ZERO), (VIEW, None)):
            self.assertFalse(settled_short_view({'observation_view': view, 'end_status': status}))

    def test_accepted_view_is_recorded_not_first_error(self):
        record, review = self.review([self.stage('observe_0_0', VIEW)])
        gates = [e and e.get('gate') for e in (record['first_error'],)]
        self.assertNotIn('TARGET_NOT_REACHED', gates)
        self.assertEqual([v['gate'] for v in review['observation_views_settled_short']], ['OBSERVATION_VIEW_SETTLED_SHORT'])
        self.assertEqual(review['observation_views_settled_short'][0]['short_ticks'], {'elbow_flex': 26.0})

    def test_unresolved_view_or_other_stage_still_faults(self):
        for name, view in (('observe_0_0', dict(VIEW, command_state_resolved=False)), ('retreat', None)):
            with self.subTest(name=name):
                self.setUp()
                record, review = self.review([self.stage(name, view)])
                self.assertEqual(record['first_error']['gate'], 'TARGET_NOT_REACHED')
                self.assertEqual(review.get('observation_views_settled_short'), [])


if __name__ == '__main__':
    unittest.main()
