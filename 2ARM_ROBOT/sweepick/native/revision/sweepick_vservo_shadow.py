"""Shadow-only interface between A's visual loop and B18/B19 (2026-10-15). Offline: reads what A recorded
(VSERVO_SHADOW_INPUT.jsonl + looks/<tag>/), never opens a device, never imports an executor, and accepts a B proposal
only as a non-executable record. A's control path does not read anything written here."""
import hashlib
import json
from pathlib import Path

INPUT = "VSERVO_SHADOW_INPUT.jsonl"
PROPOSALS = "VSERVO_SHADOW_PROPOSALS.jsonl"
INPUT_SCHEMA = "sweepick.vservo-shadow-input.v1"


def inputs(session_out):
    """The recorded shadow input lines of one session, in order. Malformed lines are reported, not skipped silently."""
    path = Path(session_out) / INPUT; rows, errors = [], []
    if not path.exists():
        return rows, [f"{INPUT} not present"]
    for number, line in enumerate(path.read_text().splitlines(), 1):
        try:
            row = json.loads(line)
            if row.get("schema") != INPUT_SCHEMA:
                raise ValueError(f"schema {row.get('schema')!r}")
            rows.append(row)
        except ValueError as exc:
            errors.append(f"{INPUT}:{number}: {exc}")
    return rows, errors


def observation(session_out, row, *, load_images=False):
    """B18-shaped observation for one recorded look. Views without a stored file are marked unavailable (right wrist is
    not recorded per look by A). q12 is A's recorded value; reference times are A's monotonic stamps, unchanged."""
    root = Path(session_out); views = {}
    for key, rel in (("left_wrist", row["files"].get("left_wrist")), ("top", row["files"].get("top")), ("right_wrist", None)):
        path = root / rel if rel else None
        available = bool(path and path.is_file())
        view = dict(available=available, file=rel if available else None,
                    source_sha256=hashlib.sha256(path.read_bytes()).hexdigest() if available else None)
        if load_images and available:
            import cv2
            view["image_bgr"] = cv2.imread(str(path))
        views[key] = view
    return dict(observation_identity=dict(run_id=str(root), observation_id=row["observation_identity"]["look_tag"]),
                source_kind="REAL_RECORDED" if row.get("q12") is not None else "UNKNOWN", q12=row.get("q12"),
                reference_times=dict(clock="time.monotonic", look=row.get("look_timing_monotonic_s"), loop=row.get("timing_monotonic_s")),
                views=views, a_feature=row.get("a_feature"), a_correction=row.get("a_correction"), executable=False)


def record_proposal(session_out, proposal):
    """Store a B proposal next to the inputs. Refused unless it is explicitly non-executable and names its input look."""
    if proposal.get("executable") is not False or proposal.get("motor_write_authorized") is not False:
        raise ValueError("a shadow proposal must state executable=false and motor_write_authorized=false")
    ident = proposal.get("observation_identity") or {}
    if not ident.get("observation_id"):
        raise ValueError("a shadow proposal must name the recorded look it was computed from")
    line = dict(proposal, role="SHADOW_PROPOSAL_RECORD_ONLY")
    with open(Path(session_out) / PROPOSALS, "a") as stream:
        stream.write(json.dumps(line, default=str) + "\n")
    return line
