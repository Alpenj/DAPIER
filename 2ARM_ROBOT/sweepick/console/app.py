"""Loopback console: files for display, explicit operator requests to the existing A owner."""
import math
from pathlib import Path
import secrets
import subprocess
import tempfile

import anyio
from fastapi import FastAPI, HTTPException, Request, Body
from fastapi.responses import FileResponse, Response, StreamingResponse
from starlette.concurrency import run_in_threadpool
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware

from replay import depth_png, execution_plan, session, ROOTS
from control import Controller, ControlError
from live import LiveView
from zipfile import ZipFile
from replay import read_json
from inspection import file_evidence

BASE = Path(__file__).resolve().parent
REVIEW = Path('/home/robot/sweepick_instructor_review_20261014')
BUNDLE = Path('/home/robot/Downloads/SWEEPICK_INSTRUCTOR_REVIEW_20261014.zip')
app = FastAPI(title='SWEEPICK Local Replay Console', docs_url=None, redoc_url=None, openapi_url=None)
app.add_middleware(TrustedHostMiddleware, allowed_hosts=['localhost', '127.0.0.1', '[::1]'])
app.mount('/static', StaticFiles(directory=BASE / 'static'), name='static')
app.mount('/instructor-materials', StaticFiles(directory=REVIEW), name='instructor_materials')
controller = Controller()
live = LiveView(controller)
ANALYSIS_REVIEWS = {
    'A16': {'title': 'A-16 · LEFT CLOSE REFUSED 원인 분석·수정', 'directory': BASE / 'review/a16_20261015',
            'files': ('A16_ROOT_CAUSE.json', 'A16_REGRESSION_DIFF.md', 'A16_FAKE_BUS_TEST.json',
                      'A16_FIX_REVIEW.md', 'A16_ALL_CHANGES.diff', 'CURRENT_FIX_SHA256.json', 'VALIDATION.json')},
    'A17': {'title': 'A-17 · CLOSE 계약·최초 Goal 검수',
            'directory': Path('/home/robot/sweepick_261013_receive_alignment_restore/review/a17_20261015'),
            'files': ('A17_CODE_AUDIT.md', 'A17_FIRST_GOAL_FAKE_BUS.json', 'A17_LEGACY_FAILURE_TRIAGE.md',
                      'A17_REAL_TEST_READINESS.md', 'NATIVE_TESTS.txt', 'INITIAL_SOURCE_SHA256.json', 'VALIDATION.json')},
}


@app.middleware('http')
async def local_operator_headers(request, call_next):
    response = await call_next(request)
    response.headers['Content-Security-Policy'] = "frame-ancestors 'none'"
    response.headers['X-Frame-Options'] = 'DENY'
    if request.url.path in ('/', '/instructor') or request.url.path.startswith('/static/'):
        response.headers['Cache-Control'] = 'no-cache'
    if request.url.path.startswith('/instructor-materials/') and request.url.path.endswith('.md'):
        response.headers['Content-Type'] = 'text/plain; charset=utf-8'
    return response


def operator_request(request):
    if request.headers.get('origin') != str(request.base_url).rstrip('/'):
        raise HTTPException(403, 'same-origin operator request required')
    if not secrets.compare_digest(request.headers.get('x-sweepick-token', ''), controller.csrf_token):
        raise HTTPException(403, 'operator request token required')


def action(method, *args, **kwargs):
    try:
        return method(*args, **kwargs)
    except ControlError as exc:
        raise HTTPException(409, str(exc))


@app.get('/api/control')
def control_status():
    return controller.status()


@app.get('/api/control/plan')
def next_session_plan():
    return {**execution_plan(), 'execution_connected': True,
            'approval_state': 'CONNECTED_AWAITING_NEW_SESSION_FIELD_APPROVAL'}


@app.get('/api/runs')
def recorded_runs():
    return {'runs': controller.runs()}


@app.get('/api/analyses')
def analyses():
    reviews = []
    for key, item in ANALYSIS_REVIEWS.items():
        base = item['directory']
        try:
            validation = read_json(base / 'VALIDATION.json')
        except (OSError, ValueError):
            validation = {}
        files = []
        for name in item['files']:
            path = base / name
            exists = path.is_file() and not path.is_symlink()
            files.append({'name': name, 'available': exists,
                          **(file_evidence(path, base) if exists else {}),
                          'url': f'/api/analyses/{key}/files/{name}' if exists else None})
        reviews.append({'id': key, 'title': item['title'], 'kind': 'SOFTWARE_REVIEW_NO_PHYSICAL_RUN',
                        'source_run_id': '20261015_092942_a4628c', 'reference_run_id': '20261014_155824_d4017e',
                        'verification': validation or 'UNKNOWN', 'files': files,
                        'physical_reproduction': 'NOT_EXECUTED_FOR_THIS_REVIEW', 'device_access': False})
    return {'analyses': reviews}


@app.get('/api/analyses/{review_id}/files/{name}')
def analysis_file(review_id: str, name: str):
    item = ANALYSIS_REVIEWS.get(review_id)
    if not item or name not in item['files']:
        raise HTTPException(404, 'unknown review file')
    path = item['directory'] / name
    if path.is_symlink() or not path.is_file():
        raise HTTPException(404, 'review file unavailable')
    return FileResponse(path, media_type='text/plain; charset=utf-8', filename=name)


@app.get('/api/runs/{run_id}/review')
def run_review(run_id: str):
    path = action(controller.review_file, run_id)
    return FileResponse(path, filename=run_id + '_AI_REVIEW.json')


@app.get('/api/runs/{run_id}/failure-packet')
def failure_packet(run_id: str):
    path = action(controller.failure_packet, run_id)
    return FileResponse(path, filename=run_id + '_FAILURE_PACKET.zip')


def inspection_file(run_id, name):
    directory = controller.run_directory(run_id)
    if directory is None:
        raise HTTPException(404, 'unknown run')
    status = controller.status()
    if status['owner_alive'] and (status.get('active') or {}).get('id') == run_id:
        raise HTTPException(409, '검수 자료는 기존 owner 종료 후 생성합니다.')
    # Serialize derived exports with the existing record index lock.
    with controller.records_lock:
        path = directory / name
        try:
            cached = read_json(directory / 'RUN_RECORD.json')
        except (OSError, ValueError):
            cached = {}
        if not path.is_file() or not isinstance(cached, dict) or cached.get('index_signature') != controller.record_stamp(directory):
            action(controller.review_file, run_id)
        if not path.is_file():
            raise HTTPException(409, '원본 검수 자료가 부족합니다.')
        return path


@app.get('/api/runs/{run_id}/inspection')
def run_inspection(run_id: str):
    return FileResponse(inspection_file(run_id, 'INSPECTION.json'), media_type='application/json')


@app.get('/api/runs/{run_id}/inspection-package')
def inspection_package(run_id: str):
    return FileResponse(inspection_file(run_id, 'INSPECTION_PACKAGE.zip'), filename=run_id + '_INSPECTION_PACKAGE.zip')


@app.post('/api/runs/{run_id}/data-review')
def data_review(run_id: str, request: Request, body: dict = Body(...)):
    operator_request(request)
    return action(controller.save_data_review, run_id, body)


def archive_stream(directory):
    with tempfile.TemporaryFile() as errors:
        process = subprocess.Popen(['/usr/bin/tar', '-C', str(directory.parent), '-cf', '-', '--', directory.name],
                                   stdout=subprocess.PIPE, stderr=errors)
        try:
            while chunk := process.stdout.read(64 * 1024):
                yield chunk
            if process.wait() != 0:
                errors.seek(0)
                raise RuntimeError('data archive failed: ' + errors.read().decode(errors='replace'))
        finally:
            process.stdout.close()
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


class ArchiveResponse(StreamingResponse):
    def __init__(self, directory, **kwargs):
        self.archive = archive_stream(directory)
        super().__init__(self.archive, **kwargs)

    async def __call__(self, scope, receive, send):
        try:
            await super().__call__(scope, receive, send)
        finally:
            with anyio.CancelScope(shield=True):
                await run_in_threadpool(self.archive.close)


@app.get('/api/runs/{run_id}/data')
def run_data(run_id: str):
    directory = controller.run_directory(run_id)
    if directory is None:
        raise HTTPException(404, 'unknown run')
    status = controller.status()
    if status['owner_alive'] and status['active']['id'] == run_id:
        raise HTTPException(409, '원본 데이터 묶음은 현재 owner 종료 후 다운로드할 수 있습니다.')
    return ArchiveResponse(directory, media_type='application/x-tar',
        headers={'Content-Disposition': f'attachment; filename="{run_id}_data.tar"'})


@app.post('/api/control/prepare')
def prepare(request: Request, body: dict = Body(...)):
    operator_request(request)
    return action(controller.prepare, body.get('source_session'))


@app.post('/api/control/start')
def start(request: Request, body: dict = Body(...)):
    operator_request(request)
    return action(controller.start, body.get('job_id'), field_ready=body.get('field_ready'),
                  mounts_unchanged=body.get('mounts_unchanged'), supervised=body.get('supervised'))


@app.post('/api/control/stop')
def stop(request: Request, body: dict = Body(...)):
    operator_request(request)
    return action(controller.stop, body.get('job_id'))


@app.post('/api/control/view-decision')
def view_decision(request: Request, body: dict = Body(...)):
    operator_request(request)
    return action(controller.view_decision, body.get('job_id'), body.get('request_sha256'), body.get('approved'))


@app.post('/api/control/confirm-recovery')
def confirm_recovery(request: Request, body: dict = Body(...)):
    operator_request(request)
    return action(controller.confirm_recovery, body.get('job_id'), power_isolated=body.get('power_isolated'),
                  workspace_reset=body.get('workspace_reset'))


@app.get('/api/control/snapshot')
def live_snapshot():
    return action(live.snapshot)


@app.get('/api/control/media/{job_id}/{relative:path}')
def live_media(job_id: str, relative: str, depth: bool = False):
    path = action(live.media, job_id, relative)
    if depth:
        try:
            return Response(depth_png(str(path))[0], media_type='image/png')
        except (OSError, ValueError):
            raise HTTPException(422, 'invalid owner depth file')
    return FileResponse(path)


def get_session(session_id):
    try:
        if session_id in ROOTS:
            return session(session_id)
        directory = controller.run_directory(session_id)
        if directory is None:
            raise HTTPException(404, 'unknown session')
        root = directory / 'session'
        paths = [root / name for name in ('episode/EPISODE.json', 'episode/events.jsonl', 'episode/frames.jsonl', 'episode/joints.jsonl', 'STATE.json')]
        revision = tuple((path.stat().st_mtime_ns, path.stat().st_size) if path.exists() else None for path in paths)
        archive = session(session_id, root, revision)
        archive.native_release_confirmed = controller.off_confirmed({'session_out': str(root)})
        return archive
    except (OSError, KeyError, ValueError, TypeError, AttributeError, IndexError):
        raise HTTPException(422, 'recorded session is incomplete or malformed')


@app.get('/')
def index():
    return FileResponse(BASE / 'static/index.html')


@app.get('/instructor')
def instructor():
    return FileResponse(BASE / 'static/instructor.html')


@app.get('/api/instructor')
def instructor_materials():
    record = read_json(REVIEW / 'SOURCE_SELECTION.json')
    try:
        with ZipFile(BUNDLE) as archive:
            record['source_matches'] = all(archive.read(member) == path.read_bytes() for member, path in (
                ('source/RUN_APPROVED_SESSION.py', Path(record['launcher'])),
                ('source/sweepick_full_chain(2).py', Path(record['launcher']).parent / 'revision/sweepick_full_chain.py')))
    except (OSError, ValueError, KeyError):
        record['source_matches'] = False
    return record


@app.get('/api/instructor/bundle')
def instructor_bundle():
    return FileResponse(BUNDLE, filename=BUNDLE.name)


@app.get('/api/sessions')
def sessions():
    catalog = [get_session(key).catalog() for key in ROOTS]
    for record in controller.runs():
        key = record['replay_session_id']
        if key is None or key in ROOTS:
            continue
        try:
            entry = get_session(key).catalog()
        except HTTPException:
            continue
        entry.update(category=record['category'], classification_scope='이번 승인 범위 결과 · 전체 인계 성공과 구분')
        catalog.append(entry)
    return {'sessions': catalog}


@app.get('/api/sessions/{session_id}/snapshot')
def snapshot(session_id: str, t: float | None = None):
    if t is not None and not math.isfinite(t):
        raise HTTPException(422, 'time must be finite')
    return get_session(session_id).snapshot(t)


@app.get('/api/sessions/{session_id}/rerun-plan')
def rerun_plan(session_id: str):
    plan = get_session(session_id).rerun_plan()
    plan['execution_connected'] = True
    plan['approval_state'] = 'CONNECTED_AWAITING_NEW_SESSION_FIELD_APPROVAL'
    return plan


@app.get('/api/media/{session_id}/{relative:path}')
def media(session_id: str, relative: str, depth: bool = False):
    archive = get_session(session_id)
    path = (archive.root / relative).resolve()
    if relative not in archive.media or not path.is_relative_to(archive.root) or not path.is_file():
        raise HTTPException(404, 'original indexed media unavailable')
    if depth:
        try:
            return Response(depth_png(str(path))[0], media_type='image/png')
        except (ValueError, OSError):
            raise HTTPException(422, 'invalid archived depth')
    if path.suffix.lower() == '.npy':
        raise HTTPException(404, 'raw depth is only served as a display image')
    return FileResponse(path)


if __name__ == '__main__':
    import uvicorn
    uvicorn.run(app, host='127.0.0.1', port=8765)
