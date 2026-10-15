"""Check published bytes and syntax without importing any runtime module."""
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
index = json.loads((ROOT / 'SOURCE_INDEX.json').read_text())
for item in index['files']:
    path = (ROOT / item['file']).resolve()
    assert path.is_relative_to(ROOT) and path.is_file(), item['file']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == item['public_sha256'], item['file']

required = ('console/app.py','console/control.py','console/inspection.py','console/static/review.js',
            'console/static/field.js','console/test_console16.py','docs/WORKFLOW.md',
            'docs/console16/CONSOLE16_BACKEND_HANDOFF.md',
            'native/RUN_APPROVED_SESSION.py','native/revision/sweepick_move_a.py',
            'native/revision/sweepick_pick01.py','native/revision/sweepick_full_chain.py',
            'native/pin/sweepick_261008_receive_entry.py','docs/a17/A17_FIRST_GOAL_FAKE_BUS.json')
assert all((ROOT / path).is_file() for path in required)
secret = re.compile(r'gh[pousr]_[A-Za-z0-9]{20,}|AKIA[A-Z0-9]{16}|-----BEGIN (?:RSA |OPENSSH )?PRIVATE KEY-----')
for path in ROOT.rglob('*'):
    if not path.is_file() or '__pycache__' in path.parts:
        continue
    assert not path.is_symlink(), path
    assert path.suffix.lower() not in ('.zip','.tar','.mjb','.npy','.npz','.jpg','.png'), path
    assert path.name not in ('AGENTS.md','WORK_STATE.json','CONTROL.json','APPROVAL.json','WRIST_PAIRING.json'), path
    text = path.read_text()
    assert not secret.search(text), path
    assert all(name == 'robot' for name in re.findall(r'(?<![\w/-])/home/([A-Za-z_][A-Za-z0-9_-]*)', text)), path
    if path.suffix == '.py':
        compile(text, str(path), 'exec')
    if path.suffix == '.md':
        for link in re.findall(r'\]\(([^)]+)\)', text):
            if '://' not in link and not link.startswith('#'):
                assert (path.parent / link.split('#')[0]).exists(), (path, link)
print(json.dumps({'source_files':len(index['files']), 'sha256':'PASS', 'syntax':'PASS',
                  'public_boundary':'PASS', 'device_access':False}))
