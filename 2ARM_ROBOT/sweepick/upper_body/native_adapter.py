"""The REAL-side adapter of the upper-body mission — READ-ONLY part only.

What it does now (no device is opened, nothing is written):
  * evaluate_arms(): per-arm evidence from the live console's read-only status (HTTP GET), the /dev/dapier links (existence
    only, never opened), the stored REAL PICK records, and the mounted-model results the caller supplies. Everything it
    cannot show is None (UNKNOWN), never filled in.
What it refuses (no motion): pick / transport_pose / place / safe_end / hold_readback. Their REAL path must run through the
existing native Owner / stage / STOP / IK / collision / Torque-FAULT code under its own approval scope
(UPPER_BODY_PICK_TRANSPORT_PLACE), which does not exist in the native executor yet. Until it does, each returns
refused=NOT_WIRED with moved=False, so the mission ends ARM_NOT_READY and moves nothing."""
import json
import os
from pathlib import Path
from urllib.request import urlopen

NOT_WIRED = ('NOT_WIRED: the native executor has no UPPER_BODY_PICK_TRANSPORT_PLACE scope yet; '
             'motion must go through the existing Owner/stage/STOP/IK/collision path under its own approval')
LEFT_REAL_EVIDENCE = ('PICK02 run14 2026-10-08 (first unassisted pick/lift/put-back, desk)',
                      'console runs 2026-10-14..16 that reached ready_to_close / lift on the desk')


def console_status(url='http://localhost:8765', get=None):
    """The live console's /api/control as JSON (read-only GET). None when it cannot be read."""
    get = get or (lambda u: json.load(urlopen(u, timeout=3)))
    try:
        value = get(url.rstrip('/') + '/api/control')
        return value if isinstance(value, dict) else None
    except Exception:
        return None


class NativeArmAdapter:
    def __init__(self, *, console_url='http://localhost:8765', device_dir='/dev/dapier', get=None, exists=os.path.exists,
                 mounted=None, mount_verification=None):
        """mounted: {side: {approach_ik, collision_and_view, grasp_room, transport_pose, place_pose}} from the mounted-model
        checks for THIS observation (None: not run). mount_verification: the operator's record that the arms, wiring, power
        and wrist cameras were checked on the TurtleBot mount (None: not recorded)."""
        self.url, self.dev, self.get, self.exists = console_url, Path(device_dir), get, exists
        self.mounted, self.mount_verification = mounted or {}, mount_verification
        self.calls = []

    def _mechanical(self, side, console):
        links = {n: self.exists(str(self.dev / n)) for n in (f'{side}_arm', f'{side}_wrist_rgb')}
        if not all(links.values()):
            return dict(ok=False, why=f'device link missing: {[n for n, v in links.items() if not v]}', links=links)
        if console is None:
            return dict(ok=None, why='the live console status could not be read', links=links)
        if console.get('owner_alive'):
            return dict(ok=False, why='a session owner is running on the console', links=links)
        if console.get('state') == 'FAULT' or console.get('can_confirm_recovery') or console.get('block_reason'):
            return dict(ok=False, why=f"console: {console.get('state')} — {console.get('block_reason') or 'operator recovery pending'}", links=links)
        if not self.mount_verification:
            return dict(ok=None, why='no operator record that this arm, its wiring, power and wrist camera were checked on the TurtleBot mount', links=links)
        return dict(ok=True, why='links present, console idle, mount verification recorded', links=links, mount_verification=self.mount_verification)

    def evaluate_arms(self, observation):
        self.calls.append('evaluate_arms')
        console = console_status(self.url, self.get)
        out = {}
        for side in ('left', 'right'):
            cap = (dict(ok=True, eligible_for_REAL=True, evidence=list(LEFT_REAL_EVIDENCE), note='desk setup; the mount is checked under mechanical_state') if side == 'left' else
                   dict(ok=True, eligible_for_REAL=False, why='no independent RIGHT PICK on the real arm; RIGHT RECEIVE results are not a PICK'))
            rec = dict(verified_capability=cap, mechanical_state=self._mechanical(side, console))
            for name in ('approach_ik', 'collision_and_view', 'grasp_room', 'transport_pose', 'place_pose'):
                given = (self.mounted.get(side) or {}).get(name)
                rec[name] = given if isinstance(given, dict) else dict(ok=None, why=f'{name}: no mounted-model / planner result for this observation')
            out[side] = rec
        return out

    def _refuse(self, name):
        self.calls.append(name)
        return dict(refused=NOT_WIRED, moved=False)

    def pick(self, side, still_allowed):
        return self._refuse('pick')

    def transport_pose(self, side, still_allowed):
        return self._refuse('transport_pose')

    def place(self, side, still_allowed, target):
        return self._refuse('place')

    def safe_end(self, side):
        self.calls.append('safe_end')
        return dict(torque_state='UNKNOWN', holding_state='UNKNOWN', why=NOT_WIRED)

    def hold_readback(self, side):
        self.calls.append('hold_readback')
        return dict(readback_ok=False, gripper_hold='UNKNOWN', why=NOT_WIRED)
