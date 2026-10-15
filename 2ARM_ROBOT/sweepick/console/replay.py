"""Read archived A recorder files only. This module never imports the robot runtime."""
from bisect import bisect_right
from datetime import datetime
from functools import lru_cache
from io import BytesIO
import json
import math
from pathlib import Path
from statistics import median
from urllib.parse import quote

import numpy as np
from PIL import Image

HOME = Path('/home/robot')
# shortcut: curated review sessions; current/last new jobs remain available through the owner view.
ROOTS = {
    'session_18': HOME / 'sweepick_261012_receive_session_18',
    'observation_08': HOME / 'sweepick_261013_right_receive_observation_08',
    'observation_09': HOME / 'sweepick_261013_right_receive_observation_09',
    'field_20261014': HOME / 'sweepick_console_runs/20261014_155824_d4017e/session',
}
LAUNCHER = HOME / 'sweepick_261013_receive_alignment_restore/RUN_APPROVED_SESSION.py'
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
CAMERAS = {'top_rgb': ('top', 'OS30A RGB'), 'top_depth': ('top', 'OS30A Depth'),
           'left_wrist': ('left_wrist', '왼손목 RGB'), 'right_wrist': ('right_wrist', '오른손목 RGB')}
UNKNOWN = 'UNKNOWN'


def execution_plan(source_session=None):
    return {'source_session': source_session, 'execution_connected': False, 'approved': False,
            'scope': 'Fresh LEFT PICK/LIFT/CARRY/RIGHT PREGRASP → 비접촉 관측 → 조건이 충족된 정상 회수/home/OFF',
            'existing_launcher': str(LAUNCHER), 'launcher_available': LAUNCHER.is_file(),
            'requirements': ['기존 owner 단일 소유 유지', 'fresh actual/관측으로 기존 실행기에서 재계획',
                             '현재 A 실행본·source manifest 및 웹 START 전달 경로의 별도 검증',
                             '범위·실패 시 복구 절차를 제시하고 새 현장 승인 받기',
                             'BUS_LOST/ACK UNKNOWN에서 후속 Goal·실패 Goal 재전송 금지'],
            'prohibited': ['저장된 모터값 직접 재전송', '브라우저 모터 직접 제어', '이전 1회 승인 재사용',
                           '승인 없는 RIGHT INSERT/CLOSE/LEFT RELEASE'],
            'approval_state': 'NOT_CONNECTED_AWAITING_VALIDATION_AND_FIELD_APPROVAL'}


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def clean(value):
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    return UNKNOWN if isinstance(value, float) and not math.isfinite(value) else value


def read_json(path):
    return json.loads(path.read_text()) if path.exists() else {}


def terminal_evidence(root):
    """Extract child outcome before the wrapper/owner's generic exit status."""
    root = Path(root).resolve()
    lifecycle = []
    path = root / 'review/lifetime.jsonl'
    if path.is_file():
        try:
            raw_lifecycle = path.read_text(errors='replace').splitlines()
        except OSError:
            raw_lifecycle = []
        for line in raw_lifecycle:
            try:
                row = json.loads(line)
                if isinstance(row, dict):
                    lifecycle.append(row)
            except ValueError:
                continue
    parent = next((row for row in reversed(lifecycle) if row.get('event') == 'owner_end'), {})
    try:
        ep = read_json(root / 'episode/EPISODE.json')
    except (OSError, ValueError):
        ep = {}
    stages = ep.get('stages', []) if isinstance(ep, dict) else []
    event_stages = []
    events_path = root / 'episode/events.jsonl'
    if events_path.is_file():
        try:
            for line in events_path.read_text(errors='replace').splitlines():
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if isinstance(event, dict) and event.get('kind') == 'stage_record':
                    event_stages.append(event)
        except OSError:
            pass
    if not isinstance(stages, list) or not stages:
        stages = event_stages
    terminal = {'parent_exit_code': parent.get('exit_code'), 'child_exit_code': None,
                'refused': None, 'stopped_by': ep.get('stopped_by') if isinstance(ep, dict) else None,
                'exception': None, 'failed_stage': None, 'move_file': None,
                'command_state': 'APPLICATION_UNKNOWN', 'evidence_errors': []}
    terminal_failures = {'REFUSED', 'BUS_LOST', 'FAILED', 'EXCEPTION', 'EXECUTOR_EXCEPTION', 'NO_PROGRESS_HOLDING', 'RELEASE_UNCONFIRMED', 'RETURN_NOT_CONFIRMED_HOLDING'}
    candidates = []
    for order, stage in enumerate(stages if isinstance(stages, list) else []):
        if not isinstance(stage, dict):
            continue
        result = stage.get('result')
        if isinstance(result, str) and (result in terminal_failures or result.startswith(('FAULT_', 'STOPPED_'))):
            candidates.append((stage.get('t_host_s') if finite(stage.get('t_host_s')) else float('inf'), order, stage))
    if candidates:
        stage = min(candidates, key=lambda item: (item[0], item[1]))[2]
        terminal['failed_stage'] = {key: stage.get(key) for key in ('stage', 'side', 'result', 't_host_s')}
        name = stage.get('stage')
        stage_path = (root / f'stage_{name}.json').resolve() if isinstance(name, str) else None
        stage_doc = {}
        if stage_path is not None and stage_path.is_relative_to(root) and stage_path.is_file():
            try:
                stage_doc = read_json(stage_path)
            except (OSError, ValueError) as exc:
                terminal['evidence_errors'].append(f'stage record: {exc}')
        if not isinstance(stage_doc, dict):
            stage_doc = {}
        folder_value = stage.get('folder')
        folder = Path(folder_value).resolve() if isinstance(folder_value, str) else (root / str(name or '')).resolve()
        move = {}
        if folder.is_relative_to(root):
            moves = sorted((path for path in folder.glob('run*/move.json')
                            if path.resolve().is_relative_to(root)),
                           key=lambda path: int(path.parent.name[3:]) if path.parent.name[3:].isdigit() else -1)
            if moves:
                move_path = moves[-1]
                terminal['move_file'] = move_path.relative_to(root).as_posix()
                try:
                    move = read_json(move_path)
                except (OSError, ValueError) as exc:
                    terminal['evidence_errors'].append(f'move record: {exc}')
        if not isinstance(move, dict):
            terminal['evidence_errors'].append('move record is not an object')
            move = {}
        details = {**stage_doc, **move}
        terminal.update(child_exit_code=details.get('exit_code'), refused=details.get('refused'),
                        stopped_by=details.get('stopped_by') or terminal['stopped_by'],
                        exception=details.get('exception'))
        writes, attempts, events = details.get('writes'), details.get('write_attempts'), details.get('events')
        positive_writes = (isinstance(writes, dict) and any(finite(value) and value > 0 for value in writes.values())) or (isinstance(writes, list) and bool(writes))
        attempt_count = (attempts if finite(attempts) else
                         sum(value for value in attempts.values() if finite(value)) if isinstance(attempts, dict) else
                         len(attempts) if isinstance(attempts, list) else None)
        event_attempt = any(isinstance(event, dict) and any('write_acknowledgement' in str(key) for key in event) or
                            isinstance(event, str) and 'write_acknowledgement' in event
                            for event in (events if isinstance(events, list) else []))
        attempted = bool(details.get('last_goal_write_attempt') or positive_writes or
                         finite(attempt_count) and attempt_count > 0 or event_attempt)
        zero_counter_recorded = (isinstance(writes, dict) and {'Goal_Position', 'Torque_Enable'} <= writes.keys()
                                 and all(finite(value) and value == 0 for value in writes.values())) or attempt_count == 0
        explicit_zero = zero_counter_recorded and not attempted
        terminal['command_state'] = ('NOT_EXECUTED' if terminal['refused'] and explicit_zero else
                                     'FAILED' if details.get('application_status') == 'FAILED' else
                                     'APPLICATION_UNKNOWN')
    terminal['error'] = terminal['refused'] or terminal['stopped_by'] or terminal['exception']
    if terminal['error'] is not None and not isinstance(terminal['error'], dict):
        terminal['error'] = {'gate': 'CHILD_TERMINAL_ERROR', 'detail': str(terminal['error'])}
    if not terminal['error'] and terminal['child_exit_code'] not in (None, 0):
        terminal['error'] = {'gate': 'CHILD_EXIT', 'detail': f"child exit {terminal['child_exit_code']}"}
    if not terminal['error'] and parent.get('error'):
        terminal['error'] = {'gate': 'EXECUTOR_ERROR', 'detail': parent['error']}
    return clean(terminal)


def application_state(*, attempted=True, ack=None, requested=None, goal=None, actual=None, start_actual=None):
    if not attempted:
        return {'state': 'NOT_EXECUTED', 'proof': None}
    if ack in ('FAILED', 'REFUSED', 'NACK'):
        return {'state': 'FAILED', 'proof': None}
    if finite(requested) and finite(goal) and requested == goal:
        proof = 'PHYSICAL_MOTION' if finite(actual) and finite(start_actual) and (actual - start_actual) * (requested - start_actual) > 0 else 'GOAL_REGISTER_MATCH'
        return {'state': 'APPLICATION_CONFIRMED', 'proof': proof}
    return {'state': 'APPLICATION_UNKNOWN', 'proof': None}


def iso_time(value):
    return datetime.fromisoformat(value).timestamp() if value else None


def latest(rows, times, at):
    i = bisect_right(times, at) - 1
    return (rows[i], i) if i >= 0 else (None, -1)


@lru_cache(maxsize=32)
def depth_png(path_string):
    path = Path(path_string)
    depth = np.load(path, allow_pickle=False) if path.suffix == '.npy' else np.asarray(Image.open(path))
    if depth.ndim != 2 or not np.issubdtype(depth.dtype, np.number):
        raise ValueError('depth must be a numeric 2D millimetre array')
    valid = np.isfinite(depth) & (depth > 0)
    colour = np.zeros((*depth.shape, 3), dtype=np.uint8)
    bounds = None
    if valid.any():
        lo, hi = float(depth[valid].min()), float(depth[valid].max())
        bounds = [lo, hi]
        scale = np.zeros(depth.shape, dtype=float)
        if hi > lo:
            scale[valid] = (depth[valid] - lo) / (hi - lo)
        colour[valid] = np.column_stack((255 * (1 - scale[valid]),
                                       255 * (1 - np.abs(2 * scale[valid] - 1)),
                                       255 * scale[valid])).astype(np.uint8)
    output = BytesIO()
    Image.fromarray(colour).save(output, format='PNG')
    return output.getvalue(), bounds


def settled_short_view(stage):
    """An OBSERVATION_VIEW that ended TARGET_NOT_REACHED_HOLDING with the same conditions the full-chain owner accepts
    (sweepick_full_chain Owner.segment missed_view): command completed, actual settled, command state resolved, Status 0."""
    view, status = stage.get('observation_view'), stage.get('end_status')
    return bool(isinstance(view, dict) and view.get('command_completed') is True and view.get('actual_settled') is True
                and view.get('command_state_resolved') is True and isinstance(status, dict) and status and not any(status.values()))


class ReplaySession:
    def __init__(self, session_id, root):
        self.id, self.root = session_id, Path(root).resolve()
        self.warnings, self.sources = [], []
        self.ep = self.load('episode/EPISODE.json')
        self.state = self.load('STATE.json')
        self.events = list(self.lines('episode/events.jsonl'))
        self.frames = {key: [] for key in CAMERAS}
        self.feedback = {side: [] for side in ('left', 'right')}
        self.media = set()
        self.offset = None
        self.stages, self.decisions, self.bookmarks = [], [], []
        self.packet_writes = []
        self.fault = None
        self.settled_short_views = []                                   # accepted observation views that settled short: recorded, not a fault
        self.untimed_errors = []                                       # Console-17: recorded errors without a host time; never ordered as "first"
        self.index_frames()
        self.index_feedback()
        self.index_stages()
        self.index_decisions()
        self.index_faults()
        self.index_later_reads()
        for key, frames in self.frames.items():
            frames.sort(key=lambda row: row['t'])
        for rows in self.feedback.values():
            rows.sort(key=lambda row: row['t'])
        self.frame_times = {key: [row['t'] for row in frames] for key, frames in self.frames.items()}
        self.feedback_times = {side: [row['t'] for row in rows] for side, rows in self.feedback.items()}
        times = [e['t_host_s'] for e in self.events if finite(e.get('t_host_s'))]
        times += [t for values in self.frame_times.values() for t in values]
        times += [t for values in self.feedback_times.values() for t in values]
        if not times:
            raise ValueError('session has no recorded timestamps')
        self.start, self.end = min(times), max(times)
        self.trial_end = max((e['t_host_s'] for e in self.events if e.get('kind') == 'episode_end'), default=self.end)
        self.bookmarks.insert(0, {'name': '기록 시작', 't_s': self.start})
        self.bookmarks.append({'name': '세션 종료', 't_s': self.trial_end})
        self.default = next((b['t_s'] for b in self.bookmarks if b['name'] == 'BEFORE_RIGHT_PREGRASP'),
                            self.fault['time_s'] if self.fault else self.start)
        self.category = 'success' if self.id == 'session_18' else 'failure' if self.id == 'observation_09' else 'partial'
        if self.id == 'session_18':
            self.scope = 'PICK/LIFT/CARRY/RIGHT PREGRASP 시연 범위 성공 · 전체 RECEIVE는 미완료'
            self.warnings.append('성공 분류는 지정된 접근 시연 범위입니다. 전체 handover 성공이 아니며 OBSERVE·retreat의 TARGET_NOT_REACHED 기록도 보존합니다.')
        elif self.id == 'observation_09':
            self.scope = '왼팔 초기 PREGRASP 중 BUS_LOST · 오른팔 접근 전 중단'
            self.warnings.append('이 세션의 원본 영상·frames/joints 인덱스는 이전 데이터 정리에서 삭제되었습니다. 이후 환경 점검 영상을 대신 사용하지 않습니다.')
        elif self.id == 'field_20261014':
            self.scope = '웹 PICK/LIFT/CARRY/RIGHT PREGRASP 완료 · 관측 승인 대기에서 왼 영상 갱신 공백으로 중단'
            self.warnings.append('정상 회수/HOME은 수행하지 않았습니다. 이후 사용자 요청 OFF 확인은 별도 기록이며, 프레임 공백을 물체 낙하로 판정하지 않습니다.')
            self.default = self.fault['time_s'] if self.fault else self.default
        elif self.id == 'observation_08':
            self.scope = 'PICK/LIFT/CARRY/오른팔 접근 완료 · 수신면 관측은 카메라 오류로 미완료'
        else:
            self.scope = '새 실물 세션 · 단계별 완료와 최초 오류는 원본 기록 기준'
            self.category = 'failure' if self.fault or self.untimed_errors else 'partial'
        if not self.ep.get('data_quality', {}).get('complete_three_view_coverage', False):
            self.warnings.append('원본의 연속 3-view coverage는 INCOMPLETE입니다. 영상이 있는 구간에서도 누락 구간을 정상 연속 수집으로 표시하지 않습니다.')

    def load(self, relative):
        path = self.root / relative
        if not path.exists():
            return {}
        self.sources.append(relative)
        try:
            return read_json(path)
        except (ValueError, OSError) as exc:
            self.warnings.append(f'{relative}: 읽기 실패 {exc}')
            return {}

    def lines(self, relative):
        path = self.root / relative
        if not path.exists():
            return
        self.sources.append(relative)
        with path.open() as stream:
            for n, line in enumerate(stream, 1):
                try:
                    row = json.loads(line)
                    if isinstance(row, dict):
                        yield row
                except ValueError:
                    self.warnings.append(f'{relative}:{n}: 손상된 행 제외')

    def allow_media(self, path):
        path = Path(path).resolve()
        if path.is_relative_to(self.root) and path.suffix.lower() in ('.jpg', '.jpeg', '.png', '.npy') and path.is_file():
            relative = path.relative_to(self.root).as_posix()
            self.media.add(relative)
            return relative
        return None

    def index_frames(self):
        for row in self.lines('episode/frames.jsonl'):
            t = row.get('recv_host_s')
            if not finite(t):
                continue
            if self.offset is None and finite(row.get('recv_monotonic_s')):
                self.offset = t - row['recv_monotonic_s']
            camera = row.get('camera')
            key = 'top_rgb' if camera == 'top' else camera
            if key not in self.frames:
                continue
            file = row.get('file')
            if file:
                self.frames[key].append({'t': t, 'n': row.get('n'),
                    'file': self.allow_media(self.root / 'episode' / file)})
            if camera == 'top' and row.get('depth_file'):
                self.frames['top_depth'].append({'t': t, 'n': row.get('n'),
                    'file': self.allow_media(self.root / 'episode' / row['depth_file'])})

    def sample(self, side, at, raw, source, requested=None):
        if not finite(at) or side not in self.feedback:
            return
        self.feedback[side].append({'t': at, 'source': source,
            'actual': raw.get('ticks', raw.get('present', {})),
            'goal': raw.get('goal_register', raw.get('goal', {})),
            'torque': raw.get('torque', {}), 'status': raw.get('status', {}), 'moving': raw.get('moving', {}),
            'requested': requested or raw.get('commanded', {})})

    def index_feedback(self):
        for row in self.lines('episode/joints.jsonl'):
            t = row.get('t_host_s')
            if finite(t):
                self.sample(row.get('side'), t + row.get('read_s', 0), row, 'episode/joints.jsonl')

    def index_stages(self):
        phases = [e for e in self.events if e.get('kind') == 'phase']
        for stage in self.ep.get('stages', []):
            name, side = stage['stage'], stage.get('side', 'left')
            folder = Path(stage['folder']).resolve()
            if not folder.is_relative_to(self.root):
                self.warnings.append(f'{name}: 세션 밖 경로 제외')
                continue
            moves = sorted(folder.glob('run*/move.json'))
            ended = stage.get('t_host_s')
            matching = [e['t_host_s'] for e in phases if e.get('step') in (name, name + '_' + side) and e['t_host_s'] <= ended]
            started = max(matching) if matching else self.state.get('stages', {}).get(name, {}).get('t')
            record = {'name': name, 'side': side, 'start': started, 'end': ended,
                      'result': stage.get('result', UNKNOWN), 'last_attempt': None, 'acknowledged': {}, 'release': UNKNOWN}
            for path in moves:
                relative = path.relative_to(self.root).as_posix()
                move = self.load(relative)
                for row in move.get('rows', []):
                    t = row.get('t')
                    if finite(t):
                        self.sample(side, t + row.get('read_s', 0), row, relative)
                        if started is None:
                            started = t
                # The terminal record is only made visible at its recorded stage end.
                if isinstance(move.get('end'), dict) and isinstance(move['end'].get('ticks'), dict):
                    self.sample(side, ended, move['end'], relative + ':end', move.get('last_acknowledged_goal'))
                record['observation_view'] = move.get('observation_view') if isinstance(move.get('observation_view'), dict) else None
                record['end_status'] = (move.get('end') or {}).get('status') if isinstance(move.get('end'), dict) else None
                record['reached'] = move.get('reached') if isinstance(move.get('reached'), dict) else None
                record.update(last_attempt=move.get('last_goal_write_attempt'),
                              acknowledged=move.get('last_acknowledged_goal', {}),
                              release=move.get('outcome', {}).get('torque_released', UNKNOWN), start=started,
                              ack_source=relative + ':end')
                if move.get('result') == 'BUS_LOST':
                    self.fault = {'gate': 'BUS_LOST', 'time_s': ended, 'side': side,
                        'detail': move.get('stopped_by', {}).get('detail', 'bus feedback lost'), 'source': relative}
            self.stages.append(record)

    def index_decisions(self):
        if self.offset is None:
            return
        seen = {key: {row['n'] for row in frames} for key, frames in self.frames.items()}
        for path in sorted((self.root / 'decision_observations').glob('*/OBSERVATION.json')):
            relative = path.relative_to(self.root).as_posix()
            observation = self.load(relative)
            raw = observation.get('actual_feedback', {})
            mono = raw.get('t_bus_monotonic')
            if not finite(mono):
                continue
            t = mono + self.offset
            name = observation.get('stage', path.parent.name)
            self.decisions.append({'name': name, 't': t, 'frames': observation.get('camera_frames', {})})
            self.bookmarks.append({'name': name, 't_s': t})
            for side in self.feedback:
                per_side = {key: values.get(side, {}) for key, values in raw.items() if isinstance(values, dict)}
                self.sample(side, t, per_side, relative + ':actual_feedback')
            for key, (camera, _) in CAMERAS.items():
                frame = observation.get('camera_frames', {}).get(camera, {})
                receipt = frame.get('recv_monotonic_s')
                filekey = 'raw_depth_mm' if key == 'top_depth' else 'rgb'
                file = observation.get('files', {}).get(camera, {}).get(filekey)
                if finite(receipt) and file and frame.get('n') not in seen[key]:
                    self.frames[key].append({'t': receipt + self.offset, 'n': frame.get('n'),
                        'file': self.allow_media(path.parent / file)})
                    seen[key].add(frame.get('n'))
            per_camera = observation.get('camera_frames', {})
            if observation.get('observation', {}).get('fresh') is True and any(f.get('fresh') is False for f in per_camera.values()):
                self.warnings.append(f'{name}: 상위 fresh=true와 카메라별 fresh=false가 불일치합니다. 카메라별 기록을 표시합니다.')

    def index_faults(self):
        terminal = terminal_evidence(self.root)
        if terminal.get('error'):
            error = dict(terminal['error'])
            stage = terminal.get('failed_stage') or {}
            # Console-17: a final exception with no failed-stage host time keeps time_s=None. It was given 0 before, which
            # made it "earlier" than the real servo error and moved the failure frames to the session start.
            when = stage.get('t_host_s')
            error.update(time_s=when if finite(when) else None, source=terminal.get('move_file') or 'review/lifetime.jsonl',
                         level=error.get('level', UNKNOWN))
            if not finite(error['time_s']):
                error['time_basis'] = 'NOT_RECORDED_FOR_THIS_ERROR'
                self.untimed_errors.append(error)
            elif not self.fault or error['time_s'] < self.fault['time_s']:
                self.fault = error
        packets = self.load('review/BUS_PACKET_CURRENT_PID.json')
        if isinstance(packets, list):
            self.packet_writes = [p for p in packets if p.get('kind') == 'write' and p.get('expected', {}).get('address') == 42]
            failed = next((p for p in self.packet_writes if p.get('error_kind')), None)
            if self.fault and failed:
                expected = failed.get('expected', {})
                self.fault.update(id=expected.get('expected_id'), requested_goal=expected.get('value'),
                    transport_error=failed.get('error_kind'), sdk_result=[r.get('result') for r in failed.get('sdk_rx_results', [])],
                    tx=failed.get('tx'), rx_bytes=failed.get('rx_total_bytes'),
                    latency_ms=failed['latency_ns'] / 1e6 if finite(failed.get('latency_ns')) else UNKNOWN,
                    attempts=failed.get('actual_attempts'), retries=failed.get('actual_retries'),
                    ack=failed.get('ack_status', UNKNOWN), application=failed.get('application_status', UNKNOWN))
        self.reader_errors = {}
        for camera, record in self.ep.get('readers_closed', {}).items():
            if record.get('ended_before_owner_stop') and self.offset is not None:
                mono = record.get('reader_ended_monotonic_s')
                if finite(mono):
                    key = 'top' if camera == 'SdkTop' else camera
                    error = {'gate': 'CAMERA_READER_ERROR', 'time_s': mono + self.offset,
                             'detail': str(record.get('reader_error') or 'reader ended before owner stop'),
                             'camera': key, 'source': 'episode/EPISODE.json readers_closed'}
                    self.reader_errors[key] = error
                    if not self.fault or error['time_s'] < self.fault['time_s']:
                        self.fault = error
        # Controller failures after the successful approach are retained in their own scope.
        for stage in self.stages:
            if stage['result'] == 'TARGET_NOT_REACHED_HOLDING' and settled_short_view(stage):
                short = {n: r.get('minus_target') for n, r in (stage.get('reached') or {}).items() if isinstance(r, dict) and r.get('within_tolerance') is False}
                self.settled_short_views.append({'gate': 'OBSERVATION_VIEW_SETTLED_SHORT', 'time_s': stage['end'], 'detail': stage['name'],
                    'short_ticks': short, 'source': 'episode/EPISODE.json stages + run/move.json observation_view',
                    'meaning': 'command completed, actual settled and resolved, target not reached; the owner observed from the measured pose'})
                self.bookmarks.append({'name': '관측 자세 미도달(정착·수용) · ' + stage['name'], 't_s': stage['end']})
                continue
            if stage['result'] == 'TARGET_NOT_REACHED_HOLDING':
                error = {'gate': 'TARGET_NOT_REACHED', 'time_s': stage['end'], 'detail': stage['name'],
                         'source': 'episode/EPISODE.json stages'}
                if not self.fault or error['time_s'] < self.fault['time_s']:
                    self.fault = error
        for event in self.events:
            if event.get('kind') == 'awaiting_observation_move_approval':
                self.bookmarks.append({'name': '추가 관측 이동 승인 대기', 't_s': event['t_host_s']})
            if event.get('kind') in ('observation_plan_authorization_failed', 'owner_stop'):
                detail = event.get('reason') or '; '.join(event.get('reasons', [])) or 'existing owner stopped'
                error = {'gate': 'LEFT_HOLD_VISION_UNCONFIRMED' if 'fresh left wrist' in detail else 'OWNER_STOP',
                         'time_s': event['t_host_s'], 'detail': detail, 'source': 'episode/events.jsonl'}
                if not self.fault or error['time_s'] < self.fault['time_s']:
                    self.fault = error
        if self.fault:
            self.bookmarks.append({'name': '최초 오류 · ' + self.fault['gate'], 't_s': self.fault['time_s']})

    def index_later_reads(self):
        self.recovery = None
        for relative in ('review/ENDING_ACTUAL.json', 'review/TORQUE_OFF_RESULT.json'):
            record = self.load(relative)
            at = iso_time(record.get('timestamp'))
            if not finite(at):
                continue
            self.bookmarks.append({'name': '별도 사후 조회' if 'ENDING' in relative else '승인 후 왼팔 OFF 확인', 't_s': at})
            if 'ENDING' in relative:
                for side in self.feedback:
                    raw = {key: values.get(side, {}) for key, values in record.get('raw', {}).items() if isinstance(values, dict)}
                    self.sample(side, at, raw, relative + ':later_separate_read')
            else:
                after = record.get('after', {})
                self.sample('left', at, {'torque': after.get('Torque_Enable', {}),
                    'status': after.get('Status', {}), 'moving': after.get('Moving', {})}, relative + ':later_approved_cleanup')
                self.recovery = {'time_s': at, 'scope': record.get('scope'), 'all_left_off': record.get('all_left_off'),
                                 'right': record.get('right_unchanged'), 'source': relative}
        relative = 'review/TORQUE_OFF_OPERATOR_REQUEST.json'
        record = self.load(relative)
        at = iso_time(record.get('finished'))
        if finite(at):
            self.bookmarks.append({'name': '별도 사용자 요청 양팔 OFF 확인', 't_s': at})
            for side in self.feedback:
                after = record.get('sides', {}).get(side, {}).get('after') or {}
                if after.get('answered') is True:
                    self.sample(side, at, after.get('state', {}), relative + ':read_at_or_before_finish')
            self.recovery = {'time_s': at, 'scope': record.get('scope'), 'source': relative,
                             'all_left_off': record.get('all_off_confirmed'),
                             'right': '양팔 OFF 사후 확인 · HOME/정상 회수 확인 아님'}

    def catalog(self):
        return {'id': self.id, 'label': self.id, 'start_s': self.start, 'end_s': self.end,
                'default_t_s': self.default, 'bookmarks': sorted(self.bookmarks, key=lambda b: b['t_s']),
                'category': self.category, 'classification_scope': self.scope,
                'outcome': self.ep.get('task_outcome', UNKNOWN), 'warning': ' '.join(self.warnings)}

    def camera(self, key, at):
        camera, label = CAMERAS[key]
        rows, times = self.frames[key], self.frame_times[key]
        row, index = latest(rows, times, at)
        error = self.reader_errors.get(camera)
        failed = error and at >= error['time_s']
        result = {'id': key, 'label': label, 'status': 'NOT_AVAILABLE', 'connection': 'NOT_AVAILABLE',
                  'image_url': None, 'age_s': UNKNOWN, 'fps': 'NOT_AVAILABLE', 'last_received_s': UNKNOWN,
                  'n': UNKNOWN, 'reason': '이 시각에 원본 프레임 없음', 'depth_range_mm': None, 'saved_fps': 'NOT_AVAILABLE'}
        if not row:
            return result
        age = max(0, at - row['t'])
        coverage = self.ep.get('data_quality', {}).get('camera_coverage', {}).get('views', {}).get(camera, {})
        allowance = coverage.get('cadence_allowance_s')
        intervals = [b - a for a, b in zip(times[max(0, index - 60):index], times[max(0, index - 60) + 1:index + 1]) if b > a]
        if intervals:
            result['fps'] = len(intervals) / sum(intervals)
            if not finite(allowance):
                allowance = 2 * median(intervals)
        # Depth is saved at a lower cadence; its own receipt and age stay visible.
        if key == 'top_depth' and intervals:
            allowance = 2 * median(intervals)
            result['saved_fps'] = result['fps']
            rgb, rgb_index = latest(self.frames['top_rgb'], self.frame_times['top_rgb'], at)
            rgb_times = self.frame_times['top_rgb'][max(0, rgb_index - 60):rgb_index + 1]
            if rgb and len(rgb_times) > 1 and rgb_times[-1] > rgb_times[0]:
                result['fps'] = (len(rgb_times) - 1) / (rgb_times[-1] - rgb_times[0])
        status = 'STALE' if failed or finite(allowance) and age > allowance else 'REPLAY_FRESH' if finite(allowance) else UNKNOWN
        result.update(status=status if row['file'] else 'NOT_AVAILABLE',
            connection='RECORDED_ERROR' if failed else 'RECORDED_STREAM',
            age_s=age, last_received_s=row['t'], n=row['n'], freshness_limit_s=allowance,
            reason=error['detail'] if failed else '기록 수신 간격 초과' if status == 'STALE' else '')
        if row['file']:
            result['image_url'] = f'/api/media/{self.id}/{quote(row["file"])}' + ('?depth=1' if key == 'top_depth' else '')
            if key == 'top_depth':
                try:
                    result['depth_range_mm'] = depth_png(str(self.root / row['file']))[1]
                except (ValueError, OSError) as exc:
                    result.update(status='NOT_AVAILABLE', image_url=None, reason=f'depth 읽기 실패: {exc}')
        else:
            result['reason'] = '메타데이터만 남고 원본 파일 없음'
        return result

    def joint_rows(self, at):
        output = []
        for side in self.feedback:
            row, _ = latest(self.feedback[side], self.feedback_times[side], at)
            bus_lost = self.fault and self.fault['gate'] == 'BUS_LOST' and at >= self.fault['time_s']
            if bus_lost and row and row['t'] <= self.fault['time_s']:
                row = None
            completed = [s for s in self.stages if s['side'] == side and s['end'] <= at]
            last_stage = completed[-1] if completed else {}
            packet_by_id = {p['expected']['expected_id']: p for p in self.packet_writes} if bus_lost and side == 'left' else {}
            for i, name in enumerate(JOINTS, 1):
                r = {'side': side, 'id': i, 'name': name, 'actual': UNKNOWN, 'goal': UNKNOWN,
                     'torque': UNKNOWN, 'moving': UNKNOWN, 'status': UNKNOWN, 'feedback_time_s': UNKNOWN,
                     'age_s': UNKNOWN, 'feedback_status': UNKNOWN, 'source': 'NOT_AVAILABLE',
                     'requested_goal': UNKNOWN, 'ack': 'NOT_AVAILABLE', 'application': UNKNOWN}
                if row:
                    r.update({key: row[key].get(name, UNKNOWN) if isinstance(row[key], dict) else UNKNOWN
                              for key in ('actual', 'goal', 'torque', 'moving', 'status')})
                    r.update({key: UNKNOWN for key in ('actual', 'goal', 'torque', 'moving', 'status') if r[key] is None})
                    r.update(feedback_time_s=row['t'], age_s=max(0, at - row['t']),
                             feedback_status='MEASURED' if at == row['t'] else 'HISTORICAL', source=row['source'],
                             requested_goal=row['requested'].get(name, UNKNOWN))
                    application = application_state(requested=r['requested_goal'], goal=r['goal'], actual=r['actual'])
                    r['command_state'], r['application_proof'] = application['state'], application['proof']
                    if application['state'] == 'APPLICATION_CONFIRMED':
                        r['application'] = 'READBACK_MATCH'
                # ACK evidence is separate from readback and does not certify motion/application.
                if name in last_stage.get('acknowledged', {}):
                    r['last_acknowledged'] = {'requested_goal': last_stage['acknowledged'][name],
                        'stage': last_stage['name'], 'recorded_at_s': last_stage['end'], 'ack': 'ACK'}
                    r['ack'] = UNKNOWN
                    if row and row['source'] == last_stage.get('ack_source') and r['requested_goal'] == last_stage['acknowledged'][name]:
                        r.update(ack='ACK', ack_source=last_stage['ack_source'], ack_time_s=last_stage['end'])
                if i in packet_by_id:
                    packet = packet_by_id[i]
                    r.update(requested_goal=packet['expected']['value'],
                             ack='ACK' if packet.get('ack_status') == 'CONFIRMED' else UNKNOWN,
                             application=UNKNOWN, command_state='APPLICATION_UNKNOWN', application_proof=None,
                             ack_source='review/BUS_PACKET_CURRENT_PID.json', ack_time_s=UNKNOWN)
                elif bus_lost and side == 'left':
                    r.update(ack='NOT_SENT', application=UNKNOWN, command_state='NOT_EXECUTED', application_proof=None)
                output.append(r)
        return clean(output)

    def stage_results(self, at):
        groups = {
            'PICK': [s for s in self.stages if s['name'] in ('pregrasp', 'align', 'descend_a', 'descend', 'close')],
            'LIFT': [s for s in self.stages if s['name'] == 'lift'],
            'CARRY': [s for s in self.stages if s['name'].startswith('handoff_approach_carry_')],
            'PREGRASP': [s for s in self.stages if s['name'] == 'handoff_approach_right_approach'],
            'OBSERVE': [s for s in self.stages if s['name'].startswith('handoff_observe_')],
        }
        output = []
        for name, stages in groups.items():
            started = min((s['start'] for s in stages if finite(s['start'])), default=None)
            ended = max((s['end'] for s in stages), default=None)
            complete = [s for s in stages if s['end'] <= at]
            status = 'NOT_REACHED' if not stages else 'PENDING' if started is None or at < started else 'RUNNING'
            if complete and len(complete) == len(stages):
                status = complete[-1]['result']
            errors = [s for s in complete if s['result'] in ('BUS_LOST', 'TARGET_NOT_REACHED_HOLDING') and not settled_short_view(s)]
            if errors:
                status = errors[0]['result']
            detail = '; '.join(s['name'] + ': ' + (s['result'] if s['end'] <= at else 'PENDING/RUNNING') for s in stages)
            if name == 'PICK' and status == 'BUS_LOST':
                detail = '왼팔 초기 pregrasp 중 실패 · PICK/CLOSE 미완료; ' + detail
            if status == 'REFUSED':
                detail = 'BLOCKED_NOT_COMPLETED · command refused before completion; ' + detail
            if name == 'OBSERVE':
                decisions = [d for d in self.decisions if 'RECEIVE_CONTACT' in d['name']]
                if decisions:
                    started = decisions[0]['t'] if started is None else min(started, decisions[0]['t'])
                    ended = max([d['t'] for d in decisions] + ([ended] if finite(ended) else []))
                    if at >= started:
                        bad_frames = [cam for d in decisions if d['t'] <= at for cam, frame in d['frames'].items()
                                      if frame.get('fresh') is False or frame.get('reader_valid') is False]
                        status = 'NOT_READY' if bad_frames else 'RECORDED_OBSERVATION' if not errors else errors[0]['result']
                        detail = '기록된 수신면 관측. ' + detail
                        if bad_frames:
                            detail += ' 카메라 fresh/reader_valid=false: ' + ', '.join(sorted(set(bad_frames)))
                        handoff_end = next((e for e in self.events if e.get('kind') == 'trail' and e.get('step') == 'handoff'), None)
                        if handoff_end and at >= handoff_end['t_host_s']:
                            status = 'NOT_READY'
                            detail += ' ' + handoff_end.get('result', UNKNOWN)
                    elif not stages:
                        status = 'PENDING'
                requests = [e for e in self.events if e.get('kind') == 'awaiting_observation_move_approval' and e['t_host_s'] <= at]
                consumed = [e for e in self.events if e.get('kind') == 'observation_move_approval_consumed' and e['t_host_s'] <= at]
                if requests and not consumed:
                    status, detail = 'AWAITING_APPROVAL', '현재 actual 기반 추가 비접촉 관측 이동 승인 대기'
                if self.fault and self.fault['gate'] in ('LEFT_HOLD_VISION_UNCONFIRMED', 'OWNER_STOP') and at >= self.fault['time_s']:
                    status, detail = 'FAILED', self.fault['detail']
            output.append({'name': name, 'label': name, 'status': status,
                           'completion': 'BLOCKED_NOT_COMPLETED' if status == 'REFUSED' else 'COMPLETED' if status in ('HOLDING', 'GRIP_CONTACT_HOLDING') else UNKNOWN,
                           'detail': detail or '이 단계까지 실행하지 않음', 'started_s': started, 'ended_s': ended})
        return output

    def snapshot(self, at=None):
        if at is None:
            at = self.default
        if not finite(at):
            raise ValueError('time must be finite')
        at = min(self.end, max(self.start, at))
        joints = self.joint_rows(at)
        past_phases = [e for e in self.events if e.get('kind') == 'phase' and e['t_host_s'] <= at]
        current = past_phases[-1].get('step', UNKNOWN) if past_phases else 'start'
        if at > self.trial_end:
            current = '별도 사후 조회 / 승인 후 종료 확인'
        completed = [s for s in self.stages if s['end'] <= at]
        last_stage = completed[-1] if completed else {}
        first_error = self.fault if self.fault and at >= self.fault['time_s'] else None
        if first_error and at <= self.trial_end and first_error['gate'] in ('LEFT_HOLD_VISION_UNCONFIRMED', 'OWNER_STOP'):
            current = 'STOPPED_NEEDS_OPERATOR'
        safe = UNKNOWN
        if (self.id in ROOTS or getattr(self, 'native_release_confirmed', False)) and not (self.fault and self.fault['gate'] in ('BUS_LOST', 'LEFT_HOLD_VISION_UNCONFIRMED', 'OWNER_STOP')) and at >= self.trial_end:
            if all(r['torque'] == 0 and r['status'] == 0 and r['moving'] == 0 for r in joints):
                safe = 'RECORDED_BOTH_ARMS_OFF'
        collision = []
        for name, stage in self.state.get('stages', {}).items():
            if finite(stage.get('t')) and stage['t'] > at:
                continue
            for verdict in stage.get('collision', []):
                collision.append({'stage': name, 'verdict': verdict.get('verdict', UNKNOWN),
                                  'scene': verdict.get('scene'), 'allowed': verdict.get('allowed')})
        terminal = terminal_evidence(self.root)
        terminal_time = (terminal.get('failed_stage') or {}).get('t_host_s')
        if finite(terminal_time) and terminal_time > at:
            terminal = {'parent_exit_code': None, 'child_exit_code': None, 'refused': None,
                        'stopped_by': None, 'exception': None, 'failed_stage': None, 'move_file': None,
                        'command_state': 'NOT_EXECUTED', 'error': None}
        if terminal.get('failed_stage', {}) and terminal['failed_stage'].get('stage') == 'close':
            current = 'BLOCKED_NOT_COMPLETED_AFTER_CLOSE'
        confirmed = [row for row in joints if row.get('command_state') == 'APPLICATION_CONFIRMED']
        command_state = (terminal['command_state'] if terminal.get('failed_stage') else
                         'APPLICATION_CONFIRMED' if confirmed else
                         'APPLICATION_UNKNOWN' if last_stage.get('last_attempt') or last_stage.get('acknowledged') else 'NOT_EXECUTED')
        command = {'ack': UNKNOWN, 'application': 'READBACK_MATCH' if command_state == 'APPLICATION_CONFIRMED' else UNKNOWN,
                   'state': command_state,
                   'application_proof': sorted({row.get('application_proof') for row in confirmed if row.get('application_proof')}),
                   'terminal': terminal, 'safe_shutdown': safe,
                   'last_attempt': last_stage.get('last_attempt'), 'first_error': first_error}
        if command['last_attempt']:
            command['ack'] = command['last_attempt'].get('acknowledgement', UNKNOWN)
        if terminal.get('failed_stage'):
            command['historical_last_command'] = command['last_attempt']
            command['last_attempt'] = None
            command['ack'] = 'NOT_SENT' if command_state == 'NOT_EXECUTED' else UNKNOWN
            command['application_proof'] = []
        if self.recovery and at >= self.recovery['time_s']:
            command['later_recovery'] = self.recovery
        return clean({'mode': 'REPLAY', 'session': {'id': self.id, 'label': self.id,
            'outcome': self.ep.get('task_outcome', UNKNOWN)}, 'time_s': at, 'start_s': self.start, 'end_s': self.end,
            'current_stage': current, 'cameras': [self.camera(key, at) for key in CAMERAS], 'joints': joints,
            'stage_results': self.stage_results(at),
            'ik': {'status': 'RECORDED_PLAN' if self.state.get('plan', {}).get('ik') else UNKNOWN,
                   'details': self.state.get('plan', {}).get('ik', {}), 'source': 'STATE.json plan.ik'},
            'collision': {'status': 'CLEAR' if collision and all(c['verdict'] == 'CLEAR' for c in collision) else UNKNOWN,
                          'classification': 'MODEL_CANDIDATE_NOT_PHYSICAL_CERTIFICATION', 'details': collision},
            'command': command, 'warnings': self.warnings, 'sources': self.sources,
            'controls': {'read_only': True, 'start_connected': False, 'stop_connected': False, 'device_access': False}})

    def rerun_plan(self):
        return {**execution_plan(self.id), 'demonstration_scope': self.scope,
                'raw_outcome': self.ep.get('task_outcome', UNKNOWN)}


@lru_cache(maxsize=8)
def session(session_id, root=None, revision=None):
    if session_id not in ROOTS and root is None:
        raise KeyError(session_id)
    return ReplaySession(session_id, ROOTS[session_id] if root is None else root)
