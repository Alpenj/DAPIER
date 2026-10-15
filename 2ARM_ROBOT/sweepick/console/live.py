"""Display files published by the one existing robot owner, without reading devices."""
import time
from statistics import median
from urllib.parse import quote
from pathlib import Path

from replay import (CAMERAS, JOINTS, ReplaySession, UNKNOWN, application_state, clean,
                    depth_png, finite, read_json, iso_time, terminal_evidence)
from control import ControlError


class LiveView:
    def __init__(self, controller):
        self.controller = controller
        self.allowed_media = set()
        self.cached_state = None
        self.state_mtime = None

    def snapshot(self):
        status = self.controller.status()
        job = status['active']
        if not job:
            raise ControlError('이 콘솔의 준비/실행 세션 없음')
        root, now = Path(job['session_out']), time.time()
        frame_rows = self.controller.tail(root / 'episode/frames.jsonl')
        try:
            ep = read_json(root / 'episode/EPISODE.json')
        except (OSError, ValueError):
            ep = {}
        camera_errors = []
        groups = {key: [] for key in CAMERAS}
        for row in frame_rows:
            key = 'top_rgb' if row.get('camera') == 'top' else row.get('camera')
            if key in groups and finite(row.get('recv_host_s')):
                groups[key].append(dict(row, file='episode/' + row['file']))
                if key == 'top_rgb' and row.get('depth_file'):
                    groups['top_depth'].append(dict(row, file='episode/' + row['depth_file']))
        cameras = []
        for key, (_, label) in CAMERAS.items():
            rows = groups[key]
            camera = {'id': key, 'label': label, 'status': 'NOT_AVAILABLE', 'connection': 'NOT_AVAILABLE',
                      'image_url': None, 'age_s': UNKNOWN, 'fps': 'NOT_AVAILABLE', 'saved_fps': 'NOT_AVAILABLE',
                      'last_received_s': UNKNOWN, 'n': UNKNOWN, 'reason': 'owner 원본 프레임 없음', 'depth_range_mm': None}
            if rows:
                row = rows[-1]
                recent = rows[-61:]
                intervals = [b['recv_host_s'] - a['recv_host_s'] for a, b in zip(recent, recent[1:])
                             if b['recv_host_s'] > a['recv_host_s']]
                cadence = 2 * median(intervals) if intervals else None
                age = now - row['recv_host_s']
                camera.update(status='LIVE' if status['state'] in ('RUNNING', 'STOP_REQUESTED') and cadence and age <= cadence else 'STALE',
                              connection='EXISTING_OWNER_FILE_FEED', age_s=age if age >= 0 else UNKNOWN, n=row.get('n', UNKNOWN),
                              last_received_s=row['recv_host_s'], fps=len(intervals) / sum(intervals) if intervals else 'NOT_AVAILABLE',
                              freshness_limit_s=cadence or UNKNOWN,
                              reason='마지막 owner 프레임 경과 시간 표시')
                path = (root / row['file']).resolve()
                if age < 0:
                    camera.update(status=UNKNOWN, reason='수신 시각이 현재 host 시각보다 미래임')
                if path.is_relative_to(root.resolve()) and path.is_file():
                    self.allowed_media.add((job['id'], row['file']))
                    camera['image_url'] = f'/api/control/media/{job["id"]}/{quote(row["file"])}' + ('?depth=1' if key == 'top_depth' else '')
                    if key == 'top_depth':
                        camera['saved_fps'] = camera['fps']
                        camera['fps'] = cameras[0]['fps']
                        try:
                            camera['depth_range_mm'] = depth_png(str(path))[1]
                        except (OSError, ValueError) as exc:
                            camera.update(status='NOT_AVAILABLE', image_url=None, reason='Depth 원본 미완성/읽기 오류: ' + str(exc))
                else:
                    camera.update(status='NOT_AVAILABLE', reason='원본 파일 미수신/없음')
            closed = ep.get('readers_closed', {}).get('SdkTop' if key.startswith('top_') else key, {})
            if closed.get('ended_before_owner_stop'):
                camera.update(status='STALE' if camera['image_url'] else 'NOT_AVAILABLE', connection='RECORDED_ERROR',
                              reason=str(closed.get('reader_error') or 'reader ended before owner stop'))
                receipt = rows[-1] if rows else {}
                mono = closed.get('reader_ended_monotonic_s')
                at = mono + receipt['recv_host_s'] - receipt['recv_monotonic_s'] if finite(mono) and finite(receipt.get('recv_monotonic_s')) else UNKNOWN
                camera_errors.append({'gate': 'CAMERA_READER_ERROR', 'camera': key, 'time_s': at,
                                      'detail': camera['reason'], 'source': 'episode/EPISODE.json readers_closed'})
            cameras.append(camera)
        feed = read_json(root / 'CURRENT_FEEDBACK.json')
        recovery_path = root / 'review/TORQUE_OFF_OPERATOR_REQUEST.json'
        recovery = read_json(recovery_path)
        recovery_at = iso_time(recovery.get('finished'))
        joints = []
        for side in ('left', 'right'):
            state = feed.get('sides', {}).get(side, {})
            for i, name in enumerate(JOINTS, 1):
                command = feed.get('commands', {}).get(side, {}).get(name, {})
                received = state.get('actual', {}).get('time_s')
                r = {'side': side, 'id': i, 'name': name,
                     **{key: state.get(key, {}).get('values', {}).get(name, UNKNOWN) for key in ('actual', 'goal', 'torque', 'status', 'moving')},
                     'feedback_time_s': received or UNKNOWN, 'age_s': now - received if finite(received) and now >= received else UNKNOWN,
                     'feedback_status': 'HISTORICAL' if finite(received) else UNKNOWN, 'source': 'CURRENT_FEEDBACK.json existing owner',
                     'register_times_s': {key: val.get('time_s', UNKNOWN) for key, val in state.items()},
                     'requested_goal': command.get('requested_goal', UNKNOWN), 'ack': command.get('ack', 'NOT_AVAILABLE'),
                     'ack_time_s': command.get('time_s', UNKNOWN), 'ack_source': 'existing unicast writer contract', 'application': 'NOT_EXECUTED'}
                applied = application_state(attempted=bool(command), ack=r['ack'], requested=r['requested_goal'],
                    goal=r['goal'], actual=r['actual'])
                if not (r['ack'] == 'ACK' and state.get('goal', {}).get('time_s', 0) >= command.get('time_s', float('inf'))):
                    applied = {'state': 'APPLICATION_UNKNOWN', 'proof': None} if command else {'state': 'NOT_EXECUTED', 'proof': None}
                r['command_state'], r['application_proof'] = applied['state'], applied['proof']
                r['application'] = 'READBACK_MATCH' if applied['state'] == 'APPLICATION_CONFIRMED' else UNKNOWN
                after = recovery.get('sides', {}).get(side, {}).get('after') or {}
                if after.get('answered') is True and finite(recovery_at):
                    raw = after.get('state', {})
                    r.update({key: raw.get('present' if key == 'actual' else key, {}).get(name, UNKNOWN)
                              for key in ('actual', 'goal', 'torque', 'status', 'moving')})
                    r.update(feedback_time_s=recovery_at, age_s=now - recovery_at,
                             feedback_status='POST_RECOVERY_READ_AT_OR_BEFORE_RECORD_TIME', source=str(recovery_path.relative_to(root)),
                             application=UNKNOWN, command_state='APPLICATION_UNKNOWN', application_proof=None)
                joints.append(r)
        events = self.controller.episode_events(root)
        phases = [e for e in events if e.get('kind') == 'phase']
        slim = ReplaySession.__new__(ReplaySession)
        slim.events, slim.decisions = events, []
        slim.fault = job.get('first_error')
        slim.stages = []
        for event in events:
            if event.get('kind') == 'stage_record':
                matching = [p['t_host_s'] for p in phases if p.get('step') in (event['stage'], event['stage'] + '_' + event['side'])
                            and p['t_host_s'] <= event['t_host_s']]
                slim.stages.append({'name': event['stage'], 'side': event['side'], 'result': event['result'],
                                    'start': max(matching) if matching else event['t_host_s'], 'end': event['t_host_s']})
        stages = slim.stage_results(now)
        phase = phases[-1].get('step') if phases else job.get('phase', UNKNOWN)
        required = {'PICK': {'close'}, 'LIFT': {'lift'},
                    'CARRY': {'handoff_approach_carry_right', 'handoff_approach_carry_left'},
                    'PREGRASP': {'handoff_approach_right_approach'}}
        completed = {s['name'] for s in slim.stages}
        failed = ('BUS_LOST', 'TARGET_NOT_REACHED_HOLDING', 'STOPPED_HOLDING', 'STOPPED_STATE_RECORDED')
        for group in stages:
            if group['name'] in required and group['status'] not in (*failed, 'NOT_REACHED') and not required[group['name']] <= completed:
                group.update(status='INCOMPLETE' if status['state'] in ('ENDED', 'FAULT') else 'RUNNING', ended_s=UNKNOWN)
        if phases and not str(phase).startswith('after_'):
            step = str(phase)
            active = ('PICK' if step in ('pregrasp', 'align', 'descend_a', 'descend', 'close') else
                      'LIFT' if step == 'lift' else 'CARRY' if step == 'handoff' or step.startswith('handoff_approach_carry_') else
                      'PREGRASP' if step.startswith('handoff_approach_right_approach') else
                      'OBSERVE' if step.startswith('handoff_observe_') else None)
            ended_phase = any(s['end'] >= phases[-1]['t_host_s'] and step in (s['name'], s['name'] + '_' + s['side']) for s in slim.stages)
            if active and not ended_phase:
                group = next(s for s in stages if s['name'] == active)
                if group['status'] not in failed:
                    group.update(status='INCOMPLETE' if status['state'] in ('ENDED', 'FAULT') else 'RUNNING',
                                 ended_s=UNKNOWN, detail='현재 phase: ' + str(phase) + '; ' + group['detail'])
        observed = next((e for e in reversed(events) if e.get('kind') == 'trail' and e.get('step') == 'handoff'), None)
        if observed:
            stages[-1].update(status=observed.get('result', UNKNOWN), detail=observed.get('why', UNKNOWN))
        if job.get('pending_view'):
            phase = 'AWAITING_OBSERVATION_MOVE_APPROVAL'
            stages[-1].update(status='AWAITING_APPROVAL', detail='현재 actual 기반 추가 비접촉 관측 이동 승인 대기')
        if job.get('first_error'):
            terminal = terminal_evidence(root)
            failed_name = (terminal.get('failed_stage') or {}).get('stage')
            if failed_name == 'close':
                phase = 'BLOCKED_NOT_COMPLETED_AFTER_CLOSE'
                stages[0].update(status=terminal['failed_stage']['result'], completion='BLOCKED_NOT_COMPLETED',
                                 detail='BLOCKED_NOT_COMPLETED · ' + job['first_error']['detail'])
            else:
                phase = 'STOPPED_NEEDS_OPERATOR'
                stages[-1].update(status='FAILED', detail=job['first_error']['detail'])
        state_file = root / 'STATE.json'
        if state_file.exists():
            mtime = (str(state_file), state_file.stat().st_mtime_ns)
            if self.state_mtime != mtime:
                try:
                    self.cached_state, self.state_mtime = read_json(state_file), mtime
                except (ValueError, OSError):
                    pass  # Native STATE may be in the middle of an existing non-atomic write.
        state = self.cached_state if self.state_mtime and self.state_mtime[0] == str(state_file) else {}
        state = state or {}
        collision = [{'stage': name, 'verdict': v.get('verdict', UNKNOWN)}
                     for name, stage in state.get('stages', {}).items() for v in stage.get('collision', [])]
        commands = [dict(cmd, joint=joint) for per_side in feed.get('commands', {}).values() for joint, cmd in per_side.items()]
        last = max(commands, key=lambda cmd: cmd['time_s'], default={})
        terminal = terminal_evidence(root)
        error = feed.get('first_error') or terminal.get('error') or job.get('first_error') or ({'gate': 'EXECUTOR_ERROR', 'detail': job['error']} if job.get('error') else None)
        for recorded in camera_errors:
            if error is None or finite(recorded['time_s']) and finite(error.get('time_s')) and recorded['time_s'] < error['time_s']:
                error = recorded
        return clean({'mode': 'LIVE', 'session': {'id': job['id'], 'label': '현재 실물 세션', 'outcome': status['state']},
            'time_s': now, 'start_s': now, 'end_s': now, 'current_stage': phase, 'cameras': cameras, 'joints': joints,
            'stage_results': stages, 'ik': {'status': 'RECORDED_PLAN' if state.get('plan', {}).get('ik') else UNKNOWN,
                                        'details': state.get('plan', {}).get('ik', {}), 'source': 'existing owner STATE.json'},
            'collision': {'status': 'CLEAR' if collision and all(c['verdict'] == 'CLEAR' for c in collision) else UNKNOWN,
                          'classification': 'MODEL_CANDIDATE_NOT_PHYSICAL_CERTIFICATION', 'details': collision},
            'command': {'ack': ('NOT_SENT' if terminal.get('command_state') == 'NOT_EXECUTED' else UNKNOWN)
                            if terminal.get('failed_stage') else last.get('ack', UNKNOWN), 'application': UNKNOWN,
                        'state': terminal.get('command_state') if terminal.get('failed_stage') else
                            'APPLICATION_CONFIRMED' if any(row.get('command_state') == 'APPLICATION_CONFIRMED' for row in joints) else
                            'APPLICATION_UNKNOWN' if last else 'NOT_EXECUTED',
                        'application_proof': [] if terminal.get('failed_stage') else sorted({row.get('application_proof') for row in joints if row.get('application_proof')}),
                        'terminal': terminal, 'last_attempt': None if terminal.get('failed_stage') else last,
                        'historical_last_command': last if terminal.get('failed_stage') and last else None,
                        'first_error': error, 'safe_shutdown': 'RECORDED_BOTH_ARMS_OFF' if status['state'] == 'ENDED' and self.controller.off_confirmed(job) else UNKNOWN,
                        'later_recovery': {'time_s': recovery_at, 'scope': recovery.get('scope'),
                            'all_left_off': True if recovery.get('all_off_confirmed') is True else UNKNOWN, 'right': 'both arms OFF confirmed' if recovery.get('all_off_confirmed') is True else UNKNOWN,
                            'source': str(recovery_path.relative_to(root))} if recovery else None},
            'warnings': ['현재 실행기 소유자의 파일 수신 화면입니다. 축별 실제 수신시각/경과 시간을 표시합니다.'],
            'sources': ['episode/frames.jsonl', 'CURRENT_FEEDBACK.json', 'episode/events.jsonl', 'STATE.json'],
            'controls': {'read_only': False, 'start_connected': True, 'stop_connected': True, 'device_access': False}})

    def media(self, job_id, relative):
        job = self.controller.selected(job_id)
        root = Path(job['session_out']).resolve()
        path = (root / relative).resolve()
        if (job_id, relative) not in self.allowed_media or not path.is_relative_to(root) or not path.is_file():
            raise ControlError('현재 owner 인덱스에 없는 원본 파일')
        return path
