"""READ03: one common read-only capture. The vendor SDK helper owns the OS30A; wrist cameras and the arm buses are read
in the same window by sweepick_field_r6.capture; the SDK FrameSets are then bound into that sample. No motor / torque /
register write, no camera flash write.   usage: python -m sweepick.perception.sweepick_wrist_capture OUT_DIR [--framesets 60] [--full 5] [--duration 9]"""
import argparse, json, os, subprocess, sys, threading, time
from pathlib import Path
import numpy as np
from sweepick.perception import sweepick_feedback_observation as f6
from sweepick.perception import sweepick_os30a_observation as st

HELPER = Path.home() / "DAPIER/tjj-calibration-package/v2/camera/build/registered-capture-helper-read03"
SERIAL = "OS30AA2X2CG0180"
DEVICE = dict(usb="3438:0173 Etron HP-ASC-H201", serial=SERIAL, alias="/dev/dapier/workspace_rgbd")
ROIS = dict(workspace=[180, 120, 500, 400], board=[212, 200, 485, 375], cube_20260923=[306, 131, 346, 165])
ROI_NOTE = dict(workspace="table area between the two arms (pick / place region), chosen from the READ02 image", board="printed board as seen in READ02", cube_20260923="cube top face from the 2026-09-23 record; the cube was seen at about the same place in READ02")


def board_pose(bgr, K):
    import cv2
    board = cv2.aruco.CharucoBoard((10, 7), .024, .01728, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    mc, mi, _ = cv2.aruco.ArucoDetector(board.getDictionary()).detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    if mi is None:
        return None
    flat = [int(i) for i in mi.ravel()]
    keep = [k for k, i in enumerate(flat) if i < 35 and flat.count(i) == 1]
    if len(keep) < 6:
        return None
    op = dict(zip([int(i) for i in board.getIds().ravel()], board.getObjPoints()))
    obj = np.concatenate([np.asarray(op[flat[k]]).reshape(4, 3) for k in keep]).astype(np.float64)
    img = np.concatenate([np.asarray(mc[k]).reshape(4, 2) for k in keep]).astype(np.float64)
    ok, rv, tv = cv2.solvePnP(obj, img, K, np.zeros(5))
    pr, _ = cv2.projectPoints(obj, rv, tv, K, np.zeros(5))
    R, _ = cv2.Rodrigues(rv)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = tv.ravel()
    return dict(T=T.tolist(), markers=len(keep), rms_px=float(np.sqrt((np.linalg.norm(pr.reshape(-1, 2) - img, axis=1) ** 2).mean())))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--framesets", type=int, default=75); ap.add_argument("--full", type=int, default=5); ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--contract", default=str(Path.home() / "sweepick_261007_field_read02/field_contract.json")); ap.add_argument("--base-from-board-left"); a = ap.parse_args()
    out = Path(a.out); sdk = out / "camera_sdk"; sdk.mkdir(parents=True, exist_ok=False)
    contract = json.loads(Path(a.contract).read_text())
    contract["streams"]["top_color"].update(topic=None, device=None, v4l2=None, note="owned by the vendor SDK helper in this capture (no V4L2 reader on the camera); bound into the sample by sweepick_sdk_top.bind")
    contract["streams"]["top_depth"].update(topic=None, device=None, v4l2=None, note="SDK depth of the same FrameSet as top_color")
    contract["capture_note"] = "FIELD_READ03: SDK-owned top camera + wrist cameras + arm buses in one window; read-only"
    (out / "field_contract.json").write_text(json.dumps(contract, indent=1))
    fd_out = os.open(sdk / "read03.registered.bin", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    env = dict(os.environ, READ03_FRAMESETS=str(a.framesets), READ03_FULL_LAST=str(a.full))
    env.pop("DAPIER_OS30A_DIAGNOSTIC_PREFIX", None)
    limit = a.duration + 40
    cmd = ["timeout", "--signal=TERM", "--kill-after=2s", f"{limit:.0f}s", str(HELPER), str(fd_out), SERIAL, "--registered-point-cloud", str(sdk / "read03.trace.log")]
    t0 = time.time()
    proc = subprocess.Popen(cmd, pass_fds=(fd_out,), env=env, cwd=sdk, stderr=open(sdk / "read03.stderr.log", "w"), stdout=subprocess.DEVNULL)
    os.close(fd_out)
    # The SDK needs ~12 s before its first FrameSet (attempt 2). The common window opens when the first record is on
    # disk, so the wrist cameras and the buses are read while the SDK is delivering, not before.
    binf = sdk / "read03.registered.bin"
    while time.time() - t0 < 30 and proc.poll() is None and binf.stat().st_size < 140:
        time.sleep(0.02)
    sdk_first_record_after_s = round(time.time() - t0, 3)
    f6.capture(contract, out / "sample", duration_s=a.duration, samples=150, with_arms=True, ros=False)
    rc = proc.wait()
    trace = (sdk / "read03.trace.log").read_text() if (sdk / "read03.trace.log").exists() else ""
    exit_status = dict(returncode=rc, helper_started_host_s=t0, sdk_first_record_after_s=sdk_first_record_after_s, helper_ran_s=round(time.time() - t0, 2), payload_closed="output_closed=1" in trace, pauseStream_returned="pauseStream_end=1" in trace, closeStream_returned="closeStream_end=1" in trace,
                       meaning={0: "all records written and the stream closed", 124: "ended by the time limit (see which shutdown call did not return)", 137: "killed after the time limit", 12: "fewer FrameSets than asked, then closed"}.get(rc, "see stderr"))
    sample = out / "sample"
    rep = st.bind(sample, sdk / "read03.registered.bin", device=DEVICE, rois=ROIS, helper=dict(path=str(HELPER), sha256=f6.sha_file(HELPER), command=cmd, env=dict(READ03_FRAMESETS=a.framesets, READ03_FULL_LAST=a.full)), trace=trace.splitlines()[-8:], exit_status=exit_status)
    rep["roi_note"] = ROI_NOTE
    meta = json.loads((sample / "sample.json").read_text())
    if rep.get("full_indices"):
        raw = np.load(sample / "raw.npz")
        K = np.array(meta["streams"]["top_info"]["K"]).reshape(3, 3)
        bp = board_pose(raw["top_color"][-1], K)
        mount = dict(T_camera_from_board=dict(value=None if bp is None else bp["T"], verified=bp is not None, source=None if bp is None else f"ArUco markers of the printed board in the LAST FrameSet of this sample + device rectified K ({bp['markers']} markers, rms {bp['rms_px']:.2f} px); board: 24 mm squares, operator-confirmed 2026-09-18"),
                     T_base_from_board=dict(left=dict(value=json.loads(a.base_from_board_left) if a.base_from_board_left else None, verified=False, source="operator ruler measurement 2026-09-23 (left_motor1_datum: arm origin in board -34, 201, -40 mm; axes assumed aligned with the board). UNVERIFIED"), right=dict(value=None, verified=False, source="no record")),
                     note="stereo baseline of the camera (45 mm) is internal to the camera and is not used here")
        meta["contract"]["mount"] = mount
        rep["board_pose"] = bp
        (sample / "sample.json").write_text(json.dumps(meta, indent=2, default=str))
    (sample / "sdk_top.json").write_text(json.dumps(rep, indent=1, default=str))
    mi, _ = f6.model_input(sample)
    print(json.dumps(dict(exit=exit_status, records=rep["records"], partial=rep["partial"], full=rep.get("full_indices"), rate_hz=rep.get("rate_hz"), readiness={k: v["ok"] for k, v in mi["readiness"].items()}, skew=mi["pairing"].get("skew_s"),
                          geometry={k: (v.get("valid"), v.get("reason") or v.get("basis")) for k, v in mi["geometry"].items()}), indent=1, default=str))


if __name__ == "__main__":
    main()
