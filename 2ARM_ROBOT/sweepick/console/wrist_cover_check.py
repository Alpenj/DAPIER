"""Operator cover test for the two wrist cameras (identical model, no USB serial).

    .venv/bin/python wrist_cover_check.py confirm     # aliases unchanged: confirm which camera is LEFT, record it
    .venv/bin/python wrist_cover_check.py discover    # after a port/hub change: find the cameras, propose udev lines

Opens only the two capture nodes, read-only, while no owner holds any robot device. The operator covers the LEFT wrist
lens; the camera whose image darkens must be the one mapped to LEFT, and the operator confirms the saved images.
confirm writes WRIST_IDENTITY_CONFIRMATION.json next to the native sources (the console refuses PREPARE/START without a
current one). discover writes a proposal only; it never changes udev, the plan or the pairing."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import control

VENDOR, PRODUCT = '090c', 'b371'


def capture(node, frames=8):
    import cv2
    cam = cv2.VideoCapture(node, cv2.CAP_V4L2)
    try:
        if not cam.isOpened():
            raise RuntimeError(f'{node} could not be opened')
        frame = None
        for _ in range(frames):
            ok, image = cam.read()
            if ok:
                frame = image
        if frame is None:
            raise RuntimeError(f'{node} delivered no frame')
        return frame
    finally:
        cam.release()


def capture_nodes(sysfs='/sys/class/video4linux'):
    """{ID_PATH-equivalent USB route: capture node} of every connected wrist-model camera (index 0 = capture)."""
    found = {}
    for entry in sorted(Path(sysfs).glob('video*')):
        if (entry / 'index').read_text().strip() != '0':
            continue
        device = Path(os.path.realpath(entry / 'device'))
        usb = device.parent
        if (usb / 'idVendor').exists() and (usb / 'idVendor').read_text().strip() == VENDOR and (usb / 'idProduct').read_text().strip() == PRODUCT:
            found[usb.name] = dict(node=f'/dev/{entry.name}', usb_sysfs=f'/sys/bus/usb/devices/{usb.name}', devnum=(usb / 'devnum').read_text().strip(),
                                   by_path=next((str(p) for p in Path('/dev/v4l/by-path').glob('*-video-index0') if os.path.realpath(p) == f'/dev/{entry.name}' and '-usbv2-' not in p.name), None))
    return found


def precheck(nodes, run=subprocess.run):
    if run(['tmux', 'has-session', '-t', control.MANAGED], capture_output=True).returncode == 0:
        raise SystemExit('a managed owner session exists; cover test not started')
    for node in nodes:
        r = run(['fuser', node], capture_output=True, text=True)
        if r.returncode not in (0, 1) or (r.returncode == 1 and (r.stderr or '').strip()) or r.stdout.strip():
            raise SystemExit(f'{node} is in use or its occupancy is unknown; cover test not started')


def cover_test(candidates, out, *, grab=capture, ask=input, say=print):
    """candidates: {label: node}. Returns (covered_label, evidence) or raises SystemExit. Human judgement decides;
    the brightness drop is only a cross-check that must agree (relative order, no fixed threshold)."""
    from PIL import Image
    out.mkdir(parents=True, exist_ok=True)
    base = {k: grab(v) for k, v in candidates.items()}
    ask('왼쪽 손목 카메라 렌즈만 손으로 가린 뒤 Enter (오른쪽은 가리지 않음): ')
    covered = {k: grab(v) for k, v in candidates.items()}
    files = {}
    for k in candidates:
        for tag, image in (('base', base[k]), ('covered', covered[k])):
            path = out / f'{k}_{tag}.png'; Image.fromarray(image[..., ::-1]).save(path); files[f'{k}_{tag}'] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    drop = {k: 1.0 - float(covered[k].mean()) / max(float(base[k].mean()), 1e-6) for k in candidates}
    darkest = max(drop, key=drop.get)
    say(f'저장한 영상: {out}  (밝기 감소 비율 {json.dumps({k: round(v, 3) for k, v in drop.items()})})')
    ordered = sorted(drop.values())
    if len(ordered) < 2 or ordered[-1] == ordered[-2]:
        raise SystemExit('두 카메라의 밝기 변화가 구분되지 않음; 기록하지 않음')
    answer = ask(f"영상을 확인하십시오. 가려진 카메라는 '{darkest}' 입니까? [y/N]: ").strip().lower()
    if answer != 'y':
        raise SystemExit('운영자가 확인하지 않음; 기록하지 않음')
    if ask('손을 뗐고 블록·케이블 간섭이 없습니까? [y/N]: ').strip().lower() != 'y':
        raise SystemExit('작업 공간 복원이 확인되지 않음; 기록하지 않음')
    return darkest, dict(images=files, brightness_drop=drop)


def confirm(args, *, grab=capture, ask=input, say=print, run=subprocess.run, now=lambda: datetime.now().astimezone().isoformat()):
    plan = json.loads((control.A / 'RECEIVE_FIELD_PLAN.json').read_text())
    pairing = json.loads((control.A / 'WRIST_PAIRING.json').read_text())['wrists']
    nodes = control.resolve_devices(plan, pairing)                      # aliases, routes and arm serials must already be consistent
    wrists = {n: nodes[n] for n in control.WRIST_NAMES}
    precheck(wrists.values(), run=run)
    covered, evidence = cover_test(wrists, Path(args.out), grab=grab, ask=ask, say=say)
    if covered != 'left_wrist':
        raise SystemExit(f'가려진 카메라가 LEFT가 아니라 {covered}로 연결돼 있음: 좌우 케이블이 바뀌었을 수 있음; 기록하지 않음 (discover 모드로 확인)')
    record = dict(schema='sweepick.wrist-identity-confirmation.v1', confirmed_at=now(), operator_source='local_operator_cover_test',
                  boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                  wrists={n: dict(capture_node=wrists[n], usb_sysfs=pairing[n]['connection']['usb_sysfs'],
                                  devnum=Path(pairing[n]['connection']['usb_sysfs'], 'devnum').read_text().strip(), by_path=pairing[n]['device']) for n in control.WRIST_NAMES},
                  evidence=evidence, motor_writes=0)
    target = control.A / control.IDENTITY; tmp = target.with_suffix('.tmp')
    tmp.write_text(json.dumps(record, indent=1)); tmp.replace(target)
    say(f'기록함: {target}'); return record


def discover(args, *, grab=capture, ask=input, say=print, run=subprocess.run, nodes=capture_nodes):
    found = nodes()
    if len(found) != 2:
        raise SystemExit(f'손목 카메라 Capture 노드가 2개가 아님: {sorted(found)}; 제안하지 않음')
    precheck([v['node'] for v in found.values()], run=run)
    covered, evidence = cover_test({usb: v['node'] for usb, v in found.items()}, Path(args.out), grab=grab, ask=ask, say=say)
    left = covered; right = next(u for u in found if u != left)
    def id_path(row):
        return Path(row['by_path']).name.replace('-video-index0', '') if row.get('by_path') else None
    proposal = dict(schema='sweepick.wrist-route-proposal.v1', left=dict(found[left], usb=left), right=dict(found[right], usb=right), evidence=evidence,
                    udev_lines=[f'SUBSYSTEM=="video4linux", ATTR{{index}}=="0", ATTRS{{idVendor}}=="{VENDOR}", ATTRS{{idProduct}}=="{PRODUCT}", ENV{{ID_PATH}}=="{id_path(found[left])}", SYMLINK+="dapier/left_wrist_rgb"',
                                f'SUBSYSTEM=="video4linux", ATTR{{index}}=="0", ATTRS{{idVendor}}=="{VENDOR}", ATTRS{{idProduct}}=="{PRODUCT}", ENV{{ID_PATH}}=="{id_path(found[right])}", SYMLINK+="dapier/right_wrist_rgb"'],
                    plan_device_routes=dict(left_wrist=found[left]['by_path'], right_wrist=found[right]['by_path']),
                    note='proposal only: udev, RECEIVE_FIELD_PLAN and WRIST_PAIRING are not changed by this tool; apply with operator approval, then run confirm')
    path = Path(args.out) / 'WRIST_ROUTE_PROPOSAL.json'; path.write_text(json.dumps(proposal, indent=1))
    say(f'제안 저장: {path}'); return proposal


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument('mode', choices=('confirm', 'discover'))
    ap.add_argument('--out', default=str(Path.home() / 'sweepick_console_runs' / ('wrist_cover_' + datetime.now().strftime('%Y%m%d_%H%M%S'))))
    args = ap.parse_args(argv)
    (confirm if args.mode == 'confirm' else discover)(args)


if __name__ == '__main__':
    sys.exit(main())
