"""Keep operator-confirmed physical wrist routes under the existing alias checks."""
import copy
import hashlib
import json
from pathlib import Path

NAMES = ('left_wrist', 'right_wrist')

def current_connection(row):
    device = Path(row['device'])
    usb = Path(row['connection']['usb_sysfs'])
    if not device.exists():
        raise RuntimeError(f'confirmed wrist route missing: {device}; no fallback')
    video_sysfs = Path('/sys/class/video4linux') / device.resolve().name / 'device'
    if usb.resolve() not in video_sysfs.resolve().parents:
        raise RuntimeError(f'{device}: current video route is not the confirmed USB route {usb}')
    return dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                usb_sysfs=str(usb), devnum=(usb / 'devnum').read_text().strip())

def binding_document(prior, plan, approval, *, read_connection=current_connection):
    if approval.get('unchanged_wrist_mount_and_routes') is not True:
        raise RuntimeError('same physical wrist mounting and routes are not operator-confirmed')
    doc = copy.deepcopy(prior)
    devices = []
    for name in NAMES:
        row = doc['wrists'][name]
        if row['device'] != plan['device_routes'][name] or not row.get('evidence'):
            raise RuntimeError(f'{name}: selected route differs from confirmed physical pairing')
        devices.append(str(Path(row['device']).resolve()))
        row['previous_connection'] = copy.deepcopy(row['connection'])
        row['connection'] = read_connection(row)
        row['binding_status'] = 'OPERATOR_CONFIRMED_SAME_PHYSICAL_MOUNT_AND_ROUTE_CURRENT_CONNECTION'
    if len(set(devices)) != len(NAMES):
        raise RuntimeError('both wrists resolve to the same camera')
    doc['refresh_basis'] = 'approved unchanged physical mounting/routes; current connection metadata only; no new extrinsic or unique-camera-serial claim'
    return doc

def bind_wrist_readers(loop, prepared, out, approval):
    prepared, out = Path(prepared), Path(out)
    source = (prepared / 'WRIST_PAIRING.json').read_bytes()
    plan = json.loads((prepared / 'RECEIVE_FIELD_PLAN.json').read_text())
    doc = binding_document(json.loads(source), plan, approval)
    doc['source_registry_sha256'] = hashlib.sha256(source).hexdigest()
    path = out / 'CAMERA_BINDING.json'
    path.write_text(json.dumps(doc, indent=2))
    loop.WRIST_PAIRING = path
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                source_kind='OPERATOR_PHYSICAL_PAIRING_AND_CURRENT_CONNECTION_METADATA',
                image_measurement=False, aliases={n: '/dev/dapier/' + n + '_rgb' for n in NAMES})
