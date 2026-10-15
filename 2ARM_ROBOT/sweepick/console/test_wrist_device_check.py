"""2026-10-15 WRIST UDEV & OWNER P0: real device resolution for the occupancy check (fake filesystem, FakeRunner)."""
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import control
import test_control
from control import DeviceCheckError, resolve_devices

BY = '/dev/v4l/by-path/pci-0000:65:00.3-usb-0:'
PLAN = {'device_routes': {'left_arm': '/dev/dapier/left_arm', 'right_arm': '/dev/dapier/right_arm', 'top': '/dev/dapier/workspace_rgbd',
                          'left_wrist': BY + '1.3:1.0-video-index0', 'right_wrist': BY + '1.1:1.0-video-index0'}}
PAIR = {'left_wrist': {'device': BY + '1.3:1.0-video-index0', 'connection': {'usb_sysfs': '/sys/bus/usb/devices/1-1.3'}},
        'right_wrist': {'device': BY + '1.1:1.0-video-index0', 'connection': {'usb_sysfs': '/sys/bus/usb/devices/1-1.1'}}}


def fs(**over):
    links = {'/dev/dapier/left_arm': '/dev/ttyACM1', '/dev/dapier/right_arm': '/dev/ttyACM0', '/dev/dapier/workspace_rgbd': '/dev/bus/usb/005/002',
             PLAN['device_routes']['left_wrist']: '/dev/video10', PLAN['device_routes']['right_wrist']: '/dev/video8',
             '/dev/dapier/left_wrist_rgb': '/dev/video10', '/dev/dapier/right_wrist_rgb': '/dev/video8',
             '/sys/class/video4linux/video10/device': '/sys/devices/pci0000:00/usb1/1-1/1-1.3/1-1.3:1.0',
             '/sys/class/video4linux/video8/device': '/sys/devices/pci0000:00/usb1/1-1/1-1.1/1-1.1:1.0',
             '/sys/bus/usb/devices/1-1.3': '/sys/devices/pci0000:00/usb1/1-1/1-1.3', '/sys/bus/usb/devices/1-1.1': '/sys/devices/pci0000:00/usb1/1-1/1-1.1'}
    links.update(over)
    present = {k for k, v in links.items() if v is not None}
    return dict(realpath=lambda p: links.get(p) or p, exists=lambda p: p in present or p in set(v for v in links.values() if v))


class Resolution(unittest.TestCase):
    def test_normal_left_right_mapping(self):
        nodes = resolve_devices(PLAN, PAIR, **fs())
        self.assertEqual((nodes['left_wrist'], nodes['right_wrist']), ('/dev/video10', '/dev/video8'))

    def check(self, words, plan=PLAN, pairing=PAIR, **over):
        with self.assertRaises(DeviceCheckError) as raised:
            resolve_devices(plan, pairing, **fs(**over))
        self.assertIn(words, str(raised.exception))

    def test_left_alias_missing(self):
        self.check('left_wrist: alias /dev/dapier/left_wrist_rgb missing', **{'/dev/dapier/left_wrist_rgb': None})

    def test_right_alias_points_to_the_left_camera(self):
        self.check('right_wrist: alias /dev/dapier/right_wrist_rgb -> /dev/video10', **{'/dev/dapier/right_wrist_rgb': '/dev/video10'})

    def test_both_routes_resolve_to_the_same_node(self):
        self.check('both wrists resolve to the same capture node', **{PLAN['device_routes']['right_wrist']: '/dev/video10', '/dev/dapier/right_wrist_rgb': '/dev/video10',
                                                                      '/sys/class/video4linux/video10/device': '/sys/devices/pci0000:00/usb1/1-1/1-1.3/1-1.3:1.0'})

    def test_usb_route_changed(self):
        self.check('right_wrist: /dev/video8 is not on the confirmed USB route', **{'/sys/class/video4linux/video8/device': '/sys/devices/pci0000:00/usb1/1-1/1-1.2/1-1.2:1.0'})

    def test_pairing_differs_from_plan(self):
        pairing = dict(PAIR, left_wrist=dict(PAIR['left_wrist'], device=BY + '1.4:1.0-video-index0'))
        self.check('left_wrist: plan route', pairing=pairing)

    def test_missing_arm_node(self):
        self.check('right_arm: /dev/dapier/right_arm missing', **{'/dev/dapier/right_arm': None, '/dev/ttyACM0': None})


class Occupancy(unittest.TestCase):
    setUp = test_control.ControlContract.setUp

    def test_real_right_camera_occupancy_is_reported_on_its_canonical_node(self):
        def run(argv, **k):
            self.runner.calls.append(argv)
            if argv[0] == 'fuser':
                return SimpleNamespace(returncode=0 if argv[1] == '/dev/video8' else 1, stdout='4242' if argv[1] == '/dev/video8' else '', stderr='')
            return self.runner(argv, **k)
        self.control.run = run
        self.assertEqual(self.control.owners(), {'/dev/video8': '4242'})
        with self.assertRaisesRegex(control.ControlError, 'owner'):
            self.control.prepare()

    def test_fuser_error_or_permission_problem_is_not_free(self):
        for rc, err in ((1, 'Cannot stat /dev/video8: Permission denied'), (2, '')):
            with self.subTest(rc=rc):
                self.control.run = lambda argv, **k: (SimpleNamespace(returncode=rc, stdout='', stderr=err) if argv[0] == 'fuser' else self.runner(argv, **k))
                with self.assertRaises(DeviceCheckError):
                    self.control.owners()
                contract = self.control.operation_contract()
                self.assertEqual(contract['owner']['device_occupancy'], 'UNKNOWN')
                self.assertFalse(any(s['can_prepare'] for s in contract['scopes']))

    def test_unconfirmed_mapping_refuses_prepare_and_start(self):
        job = self.control.prepare()['active']
        with patch('control.resolve_devices', side_effect=DeviceCheckError('장치 경로 확인 실패: left_wrist: alias /dev/dapier/left_wrist_rgb missing')):
            with self.assertRaisesRegex(DeviceCheckError, 'left_wrist_rgb'):
                self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True)
            self.assertFalse(any(c[:2] == ['tmux', 'new-session'] for c in self.runner.calls))
            self.assertFalse(json.loads(self.control.state_file.read_text())['consumed'])

    def test_normal_mapping_matches_the_existing_behaviour(self):
        self.assertEqual(self.control.owners(), {})
        job = self.control.prepare()['active']
        self.control.start(job['id'], field_ready=True, mounts_unchanged=True, supervised=True)
        self.assertEqual(len([c for c in self.runner.calls if c[:2] == ['tmux', 'new-session']]), 1)
        fusers = sorted(c[1] for c in self.runner.calls if c[0] == 'fuser')
        self.assertIn('/dev/video8', fusers); self.assertIn('/dev/video10', fusers)
        self.assertFalse(any('/dev/dapier/' in c for c in fusers))       # canonical nodes only


if __name__ == '__main__':
    unittest.main()
