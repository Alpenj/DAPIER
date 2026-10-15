"""Wrist camera identity: cover-test tool and the console's re-enumeration guard (fake frames, scripted answers)."""
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import control
import wrist_cover_check as wc
from control import DeviceCheckError, resolve_devices
from test_wrist_device_check import PLAN, PAIR, fs

BOOT = Path('/proc/sys/kernel/random/boot_id').read_text().strip()


class Fixture(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup); root = Path(self.tmp.name)
        self.usb = {n: root / n for n in ('1-1.3', '1-1.1')}
        for n, d in self.usb.items():
            d.mkdir(); (d / 'devnum').write_text({'1-1.3': '34', '1-1.1': '32'}[n])
        pairing = {'wrists': {'left_wrist': dict(PAIR['left_wrist'], connection={'usb_sysfs': str(self.usb['1-1.3'])}, paired_arm_serial='L'),
                              'right_wrist': dict(PAIR['right_wrist'], connection={'usb_sysfs': str(self.usb['1-1.1'])}, paired_arm_serial='R')}}
        (root / 'WRIST_PAIRING.json').write_text(json.dumps(pairing)); (root / 'RECEIVE_FIELD_PLAN.json').write_text(json.dumps(PLAN))
        for p in (patch.object(control, 'A', root), patch('control.resolve_devices', return_value={'left_wrist': '/dev/video10', 'right_wrist': '/dev/video8'})):
            p.start(); self.addCleanup(p.stop)
        self.root = root
        self.run = lambda argv, **k: SimpleNamespace(returncode=1, stdout='', stderr='')

    def grab(self, dark):
        state = {'covered': False}
        def g(node):
            v = 10 if state['covered'] and node == dark else 120
            return np.full((24, 32, 3), v, np.uint8)
        return g, state

    def answers(self, state, replies):
        it = iter(replies)
        def ask(prompt):
            if 'Enter' in prompt:
                state['covered'] = True
                return ''
            return next(it)
        return ask


class CoverTest(Fixture):
    def test_left_cover_confirms_and_records_the_current_connection(self):
        g, st = self.grab('/dev/video10')
        rec = wc.confirm(SimpleNamespace(out=str(self.root / 'ev')), grab=g, ask=self.answers(st, ['y', 'y']), say=lambda *a: None, run=self.run)
        saved = json.loads((self.root / control.IDENTITY).read_text())
        self.assertEqual(saved['wrists']['left_wrist']['capture_node'], '/dev/video10'); self.assertEqual(saved['wrists']['left_wrist']['devnum'], '34')
        self.assertEqual(saved['boot_id'], BOOT); self.assertEqual(rec['motor_writes'], 0); self.assertEqual(len(saved['evidence']['images']), 4)

    def test_swapped_cameras_are_refused_without_a_record(self):
        g, st = self.grab('/dev/video8')                                  # the LEFT lens is physically on the camera mapped to RIGHT
        with self.assertRaisesRegex(SystemExit, '좌우 케이블이 바뀌었을 수'):
            wc.confirm(SimpleNamespace(out=str(self.root / 'ev')), grab=g, ask=self.answers(st, ['y', 'y']), say=lambda *a: None, run=self.run)
        self.assertFalse((self.root / control.IDENTITY).exists())

    def test_operator_must_confirm_images_and_workspace(self):
        for replies in (['n'], ['y', 'n']):
            with self.subTest(replies=replies):
                g, st = self.grab('/dev/video10')
                with self.assertRaises(SystemExit):
                    wc.confirm(SimpleNamespace(out=str(self.root / 'ev')), grab=g, ask=self.answers(st, replies), say=lambda *a: None, run=self.run)
                self.assertFalse((self.root / control.IDENTITY).exists())

    def test_indistinguishable_change_is_refused(self):
        st = {'covered': False}
        with self.assertRaisesRegex(SystemExit, '구분되지 않음'):
            wc.confirm(SimpleNamespace(out=str(self.root / 'ev')), grab=lambda n: np.full((4, 4, 3), 50, np.uint8), ask=self.answers(st, ['y', 'y']), say=lambda *a: None, run=self.run)

    def test_occupied_camera_or_owner_session_stops_before_opening(self):
        opened = []
        for run in (lambda argv, **k: SimpleNamespace(returncode=0, stdout='999' if argv[0] == 'fuser' else '', stderr=''),
                    lambda argv, **k: SimpleNamespace(returncode=0 if argv[0] == 'tmux' else 1, stdout='', stderr='')):
            with self.assertRaises(SystemExit):
                wc.confirm(SimpleNamespace(out=str(self.root / 'ev')), grab=lambda n: opened.append(n), ask=lambda p: 'y', say=lambda *a: None, run=run)
        self.assertEqual(opened, [])

    def test_discover_after_a_port_change_writes_only_a_proposal(self):
        found = {'2-1.4': dict(node='/dev/video2', usb_sysfs='/sys/bus/usb/devices/2-1.4', devnum='7', by_path='/dev/v4l/by-path/pci-0000:67:00.4-usb-0:1.4:1.0-video-index0'),
                 '2-1.2': dict(node='/dev/video4', usb_sysfs='/sys/bus/usb/devices/2-1.2', devnum='8', by_path='/dev/v4l/by-path/pci-0000:67:00.4-usb-0:1.2:1.0-video-index0')}
        g, st = self.grab('/dev/video4')
        prop = wc.discover(SimpleNamespace(out=str(self.root / 'disc')), grab=g, ask=self.answers(st, ['y', 'y']), say=lambda *a: None, run=self.run, nodes=lambda: found)
        self.assertEqual(prop['left']['usb'], '2-1.2'); self.assertIn('ENV{ID_PATH}=="pci-0000:67:00.4-usb-0:1.2:1.0", SYMLINK+="dapier/left_wrist_rgb"', prop['udev_lines'][0])
        self.assertFalse((self.root / control.IDENTITY).exists())
        with self.assertRaisesRegex(SystemExit, '2개가 아님'):
            wc.discover(SimpleNamespace(out=str(self.root / 'd2')), nodes=lambda: {'x': found['2-1.4']}, run=self.run)


class Guard(unittest.TestCase):
    def identity(self, **over):
        rec = dict(boot_id=BOOT, wrists={'left_wrist': dict(usb_sysfs='/sys/bus/usb/devices/1-1.3', devnum='34', capture_node='/dev/video10'),
                                         'right_wrist': dict(usb_sysfs='/sys/bus/usb/devices/1-1.1', devnum='32', capture_node='/dev/video8')})
        rec.update(over); return rec

    def read(self, **over):
        values = {'/proc/sys/kernel/random/boot_id': BOOT, '/sys/bus/usb/devices/1-1.3/devnum': '34', '/sys/bus/usb/devices/1-1.1/devnum': '32',
                  '/sys/class/tty/ttyACM1/device/../serial': 'L', '/sys/class/tty/ttyACM0/device/../serial': 'R'}
        values.update(over); return lambda p: values.get(p)

    def pairing(self):
        return {n: dict(PAIR[n], paired_arm_serial={'left_wrist': 'L', 'right_wrist': 'R'}[n]) for n in PAIR}

    def test_current_confirmation_passes(self):
        resolve_devices(PLAN, self.pairing(), read=self.read(), identity=self.identity(), **fs())

    def test_missing_confirmation_reboot_or_reenumeration_refuses(self):
        for identity, read, words in ((None and {}, self.read(), 'not confirmed'), ({}, self.read(), 'not confirmed'),
                                      (self.identity(boot_id='old'), self.read(), 'rebooted'),
                                      (self.identity(), self.read(**{'/sys/bus/usb/devices/1-1.1/devnum': '40'}), 'right_wrist: camera re-enumerated')):
            if identity is None:
                continue
            with self.subTest(words=words), self.assertRaisesRegex(DeviceCheckError, words):
                resolve_devices(PLAN, self.pairing(), read=read, identity=identity, **fs())

    def test_arm_alias_on_the_other_arms_adapter_refuses(self):
        with self.assertRaisesRegex(DeviceCheckError, 'left_arm: /dev/ttyACM1 has USB serial R'):
            resolve_devices(PLAN, self.pairing(), read=self.read(**{'/sys/class/tty/ttyACM1/device/../serial': 'R'}), identity=self.identity(), **fs())


if __name__ == '__main__':
    unittest.main()
