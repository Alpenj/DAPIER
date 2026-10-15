"""Publish the existing owner's reads/ACKs to a file; no ports or control commands."""
import functools
import json
import math
import os
from pathlib import Path
import threading
import time

REGISTERS = {'Goal_Position': 'goal', 'Present_Position': 'actual', 'Torque_Enable': 'torque',
             'Status': 'status', 'Moving': 'moving', 'Present_Velocity': 'velocity'}


class Feed:
    def __init__(self, out):
        self.path = Path(out) / 'CURRENT_FEEDBACK.json'
        self.lock = threading.RLock()
        self.data = {'pid': os.getpid(), 'sides': {}, 'commands': {}, 'last_write_s': {}, 'first_error': None}
        self.disabled = False

    def side(self, bus):
        for side in ('left', 'right'):
            if os.path.realpath(bus.port) == os.path.realpath('/dev/dapier/' + side + '_arm'):
                return side

    def save(self):
        if self.disabled:
            return
        try:
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(self.data, allow_nan=False))
            temporary.replace(self.path)
        except (OSError, ValueError):
            # Optional UI producer failure never changes the native command/STOP contract.
            self.disabled = True

    def read(self, side, name, values):
        if side is None or name not in REGISTERS:
            return
        with self.lock:
            state = self.data['sides'].setdefault(side, {})
            state[REGISTERS[name]] = {'time_s': time.time(), 'values': {
                key: float(value) if math.isfinite(float(value)) else 'UNKNOWN' for key, value in values.items()}}
            if name == 'Status' and any(values.values()) and self.data['first_error'] is None:
                self.data['first_error'] = {'gate': 'SERVO_FAULT', 'side': side, 'time_s': time.time(), 'detail': str(values)}
            if name == 'Present_Position' or name == 'Status' and any(values.values()):
                self.save()

    def command(self, side, motor, value, ack, servo_id=None):
        if side is None:
            return
        with self.lock:
            self.data['commands'].setdefault(side, {})[motor] = {'id': servo_id, 'requested_goal': value, 'ack': ack, 'time_s': time.time()}
            if ack == 'ACK':
                self.data['last_write_s'][side] = time.time()
            # Publish successful ACKs with the next existing position read, avoiding per-axis file writes.

    def wrote(self, side):
        if side:
            with self.lock:
                self.data['last_write_s'][side] = time.time()
                self.save()

    def failed(self, side, detail, servo_id=None, motor=None):
        with self.lock:
            if side:
                self.data['sides'][side] = {}
            if self.data['first_error'] is None:
                self.data['first_error'] = {'gate': 'BUS_LOST', 'side': side, 'id': servo_id, 'joint': motor,
                                            'ack': 'UNKNOWN', 'application': 'UNKNOWN', 'detail': detail, 'time_s': time.time()}
            self.save()


def install(out):
    from lerobot.motors.feetech import FeetechMotorsBus
    feed = Feed(out)
    cls = FeetechMotorsBus
    original_read, original_write = cls.sync_read, cls.write

    def observe(method, bus, *args):
        if not feed.disabled:
            try:
                method(feed.side(bus), *args)
            except Exception:
                feed.disabled = True

    @functools.wraps(original_read)
    def read(bus, data_name, *args, **kwargs):
        try:
            values = original_read(bus, data_name, *args, **kwargs)
        except Exception as exc:
            observe(feed.failed, bus, f'{data_name}: {type(exc).__name__}: {exc}')
            raise
        if kwargs.get('normalize', True) is False:
            observe(feed.read, bus, data_name, values)
        return values

    @functools.wraps(original_write)
    def write(bus, data_name, motor, value, *args, **kwargs):
        try:
            servo_id = bus.motors[motor].id if isinstance(motor, str) else motor
        except (AttributeError, KeyError, TypeError):
            servo_id = None
        try:
            result = original_write(bus, data_name, motor, value, *args, **kwargs)
        except Exception as exc:
            if data_name == 'Goal_Position':
                observe(feed.command, bus, motor, value, 'UNKNOWN', servo_id)
            observe(feed.failed, bus, f'{data_name} {motor}: {type(exc).__name__}: {exc}', servo_id, motor)
            raise
        if data_name == 'Goal_Position' and kwargs.get('normalize', True) is False:
            observe(feed.command, bus, motor, value, 'ACK', servo_id)
        if data_name == 'Torque_Enable':
            observe(feed.wrote, bus)
        return result

    cls.sync_read, cls.write = read, write
    return feed
