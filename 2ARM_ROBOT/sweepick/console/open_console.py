"""Open the local operator UI; never start a robot session or access a device."""
import argparse
import fcntl
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import urlopen
import webbrowser

BASE = Path(__file__).resolve().parent
URL = 'http://localhost:8765'
LAUNCHER = '/home/robot/sweepick_261013_receive_alignment_restore/RUN_APPROVED_SESSION.py'


def console_ready():
    try:
        with urlopen(URL + '/api/instructor', timeout=2) as response:
            record = json.load(response)
    except HTTPError as error:
        raise RuntimeError(f'8765 포트가 응답하지만 시연 서버를 확인할 수 없습니다: HTTP {error.code}') from error
    except URLError:
        return False
    if record.get('launcher') != LAUNCHER:
        raise RuntimeError('8765 포트의 서버가 준비한 SWEEPICK 콘솔과 다릅니다. 기존 프로세스를 덮어쓰지 않습니다.')
    return True


def ensure_console():
    with (BASE / 'review/console_launch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if console_ready():
            return
        if not shutil.which('tmux'):
            raise RuntimeError('기존 실행 관리에 필요한 tmux가 없습니다.')
        alive = subprocess.run(['tmux', 'has-session', '-t', 'sweepick-console'], capture_output=True)
        if alive.returncode:
            command = 'exec ' + shlex.join([sys.executable, str(BASE / 'app.py')])
            command += ' > ' + shlex.quote(str(BASE / 'review/server.log')) + ' 2>&1'
            result = subprocess.run(['tmux', 'new-session', '-d', '-s', 'sweepick-console', command], capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError('콘솔 시작 실패: ' + result.stderr.strip())
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            if console_ready():
                return
            time.sleep(.2)
        raise RuntimeError(f'콘솔이 응답하지 않습니다. 기록: {BASE / "review/server.log"}')


def main():
    parser = argparse.ArgumentParser(description='로컬 시연 화면만 열기; 실물 START는 현장 승인 버튼에서 수행')
    parser.add_argument('--check', action='store_true', help='실행 파일/자료 존재만 확인, 서버와 로봇 시작 안 함')
    parser.add_argument('--no-browser', action='store_true', help='콘솔 서버 준비까지만 수행')
    args = parser.parse_args()
    try:
        required = [BASE / '.venv/bin/python', BASE / 'app.py', BASE / 'static/instructor.html', Path(LAUNCHER),
                    Path('/home/robot/sweepick_instructor_review_20261014/SOURCE_SELECTION.json'),
                    Path('/home/robot/Downloads/SWEEPICK_INSTRUCTOR_REVIEW_20261014.zip')]
        missing = [str(path) for path in required if not path.is_file()]
        if missing:
            raise RuntimeError('필요한 파일 없음: ' + ', '.join(missing))
        if args.check:
            print('PASS: 시연 화면·현재 A 실행기·검토 자료 존재. 서버/장치/실물 실행 안 함.')
            return 0
        ensure_console()
        if not args.no_browser:
            if not webbrowser.open(URL + '/instructor'):
                subprocess.run(['xdg-open', URL + '/instructor'], check=True)
        return 0
    except Exception as error:
        message = str(error)
        print(message, file=sys.stderr)
        if not args.check and not args.no_browser and shutil.which('zenity'):
            subprocess.run(['zenity', '--error', '--title=SWEEPICK 시연 준비', '--text=' + message])
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
