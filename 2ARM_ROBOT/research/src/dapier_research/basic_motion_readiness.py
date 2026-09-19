"""Offline Monday LEFT-arm read-only contract report; no device or command API."""
import argparse
import hashlib
import json
from pathlib import Path

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def inspect_snapshot(snapshot, calibration, settings):
    if (snapshot.get("schema_version") != "dapier.dual-so101-smoke.v0.4"
        or snapshot.get("motion_requested_deg") != 0
        or snapshot.get("motion_enabled") is not False
        or snapshot.get("trusted_profile_verified") is not True
        or snapshot.get("controller_identity_revalidated", {}).get("left") is not True
        or snapshot.get("error") or snapshot.get("disconnect_errors")):
        raise ValueError("not a successful existing read-only LEFT snapshot")
    raw = snapshot["arms"]["left"]["position_raw_tick"]
    if set(raw) != set(JOINTS) or set(calibration) != set(JOINTS):
        raise ValueError("joint names/order contract mismatch")
    channels = []
    for motor_id, name in enumerate(JOINTS, 1):
        c, tick = calibration[name], raw[name]
        if (type(tick) is not int or not 0 <= tick <= 4095 or c["id"] != motor_id
            or c["drive_mode"] not in (0, 1)
            or not 0 <= c["range_min"] < c["range_max"] <= 4095):
            raise ValueError("invalid encoder/calibration contract")
        # Encoder normalization is not evidence of MJCF mechanical zero/sign.
        # Preserve raw readings; never manufacture current REAL pose from SIM q=0.
        ratio = (tick-c["range_min"])/(c["range_max"]-c["range_min"])
        if c["drive_mode"]: ratio = 1-ratio
        channels.append(dict(joint=name, motor_id=motor_id, raw_tick=tick,
            range_min=c["range_min"], range_max=c["range_max"], drive_mode=c["drive_mode"],
            recorded_homing_offset=c["homing_offset"], unclamped_encoder_ratio=ratio,
            within_calibrated_range=c["range_min"] <= tick <= c["range_max"]))
    args = settings.get("parameters", settings)
    # Report the installed profile; these values are not new motion limits.
    profile = {k: args.get(k) for k in ("rate-hz", "maximum-step-ticks", "keep-motion-settings")}
    return dict(mode="REAL BASIC READINESS / OFFLINE / NO MOTION", selected_arm="LEFT",
        device_identity="reader verified LEFT role; private identity omitted",
        snapshot_timestamp=snapshot.get("started_at"), snapshot_freshness="must capture again at attended execution",
        current_measured_q=[raw[n] for n in JOINTS], units="raw encoder ticks; NOT model radians",
        joint_order=list(JOINTS), channels=channels,
        model_sign_zero="UNVERIFIED: device offsets/range do not prove MJCF zero or physical direction",
        existing_profile=profile, profile_validated_side=settings.get("validated_side", "UNVERIFIED"),
        left_profile_reuse=settings.get("left_reuse", "UNVERIFIED"),
        interpolation="current teleop raw-goal per-cycle limiter; NOT a verified Cartesian/septic hardware executor",
        update="fresh measured position initializes existing limiter; existing protection settings retained",
        status="BLOCKED_REAL_TO_MODEL_SIGN_ZERO", target_q=None, return_q=None,
        required_next="confirm joint order/sign/mechanical zero at attended session, then measured q -> FK -> base +Z 0.020 m -> existing IK; inspect bounded direction before motion approval",
        camera_extrinsic_required=False, new_watchdog_required=False,
        hardware_execution=False, control_authorized=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("snapshot", "calibration", "settings", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    a=parser.parse_args()
    report=inspect_snapshot(*(json.loads(p.read_text()) for p in (a.snapshot,a.calibration,a.settings)))
    report["input_sha256"]={name:hashlib.sha256(getattr(a,name).read_bytes()).hexdigest()
                            for name in ("snapshot","calibration","settings")}
    with a.output.open("x") as f: json.dump(report,f,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2,allow_nan=False))

if __name__=="__main__":main()
