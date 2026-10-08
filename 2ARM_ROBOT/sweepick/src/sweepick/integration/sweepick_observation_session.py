"""Resident local runtime: observation -> existing owners (Arbiter / ACT Student) -> dispatch boundary -> goal ticks that
WOULD be written. The command sink is write-disabled: this program opens no motor port for writing and sends nothing.

One process loads the models once (laptop CUDA) and keeps consuming NEW observations:
  replay   observations rebuilt from stored captures (every stored top FrameSet with its own wrist frames and joint row)
  live     read-only devices (wrist cameras 320x240, both arm buses through the read-only wrapper, top image)
Each control period writes one JSON line: sequence, capture stamps, request / chunk / anchor ids, chunk index, owner, the
owner's raw target, the dispatch status and the ticks that would be written, and the time each part took.
The joint mapping is the CANDIDATE one: this is a timing / connection run, not a control or geometry verification."""
from sweepick.integration.sweepick_resource_paths import source_path
import argparse, hashlib, json, sys, time
from pathlib import Path
import numpy as np

RT = Path.home() / "DAPIER/tjj-runtime/v1"
TOOLS = Path(__file__).resolve().parents[1]
# The field tools come FIRST: a module that exists in both places must be the tool, never a copy inside the runtime package.
for p in (TOOLS, RT / "source/production_source_g000", RT / "factory/v1"):
    while str(p) in sys.path:
        sys.path.remove(str(p))
sys.path[:0] = [str(RT / "source/production_source_g000"), str(RT / "factory/v1")]
import mujoco, torch
from sweepick.control import sweepick_joint_command_mapping as rc
from sweepick.control import sweepick_command_application as rx
from sweepick.control import sweepick_trajectory_profile as tt
from sweepick.control import sweepick_joint_kinematics as sweepick_kin
from sweepick.perception import sweepick_observation_capture as f5
from sweepick.perception import sweepick_feedback_observation as f6

for _m, _source_name in ((rc, "sweepick_real_command.py"), (rx, "sweepick_real_exec.py"), (tt, "sweepick_trajectory.py"), (sweepick_kin, "sweepick_kin.py"), (f5, "sweepick_field.py"), (f6, "sweepick_field_r6.py")):                                           # refuse to run on a shadowed tool
    if Path(_m.__file__).resolve() != source_path(_source_name):
        raise ImportError(f"{_m.__name__} was imported from {_m.__file__}, not from the field tools")
IN = Path.home() / "sweepick_261007_commission/inputs"
CAL = Path.home() / ".config/dapier/lerobot-calibration"
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


class MirrorSensor:
    def __init__(self):
        self.current = {}


class Shim:
    t, _n = 0.0, 0
    def first_forward(self, student, context):
        return student.next_command(context)
    def step(self):
        return self._n


def to_model(img_bgr, gray3=False):
    rgb = f5.to_rgb(img_bgr, "bgr8")
    out, _, _ = f5.fit(rgb, None)
    if gray3:
        l = np.round(0.299 * out[..., 0] + 0.587 * out[..., 1] + 0.114 * out[..., 2]).astype(np.uint8)
        out = np.repeat(l[..., None], 3, axis=2)
    return np.ascontiguousarray(out)


def replay_observations(sample_dirs, mapping):
    """One observation per stored top FrameSet, each with the wrist frames and the joint row nearest to ITS stamp. Wrist
    frames that fail the stationary integrity check are not candidates; an observation without a usable frame inside one
    top period is yielded as unusable (it is not repaired from another instant)."""
    obs = []
    for sd in sample_dirs:
        sd = Path(sd); raw = np.load(sd / "raw.npz"); fb = json.loads((sd / "feedback.json").read_text())
        still = f6.arms_stationary(fb)
        use = {k: (f6.stream_integrity(raw[k])[0] if still and len(raw[k]) >= 3 else np.ones(len(raw[k]), bool)) for k in ("left_wrist", "right_wrist")}
        rows = {s: [r for r in fb["arms"][s]["rows"] if "values" in r["registers"].get("present_raw", {})] for s in ("left", "right")}
        tj = {s: np.array([(r["t_start"] + r["t_end"]) / 2 for r in rows[s]]) for s in rows}
        tt_ = raw["top_color_stamp"]; period = float(np.median(np.diff(tt_))) if len(tt_) > 1 else 0.04
        for i, t in enumerate(tt_):
            o = dict(source=str(sd), top_index=int(i), stamp_top=float(t), usable=True, why=[])
            for k in ("left_wrist", "right_wrist"):
                cand = np.nonzero(use[k])[0]
                if not len(cand):
                    o["usable"] = False; o["why"].append(f"{k}: no intact frame"); continue
                j = int(cand[np.argmin(np.abs(raw[k + "_stamp"][cand] - t))])
                o[k + "_index"], o["skew_" + k] = j, float(raw[k + "_stamp"][j] - t)
                if abs(o["skew_" + k]) > period:
                    o["usable"] = False; o["why"].append(f"{k}: nearest intact frame is {o['skew_' + k] * 1000:.0f} ms away (> one top period {period * 1000:.0f} ms)")
            ticks = {}
            for s in ("left", "right"):
                j = int(np.argmin(np.abs(tj[s] - t))); ticks[s] = rows[s][j]["registers"]["present_raw"]["values"]; o["skew_joints_" + s] = float(tj[s][j] - t)
                o["torque_" + s] = max(rows[s][j]["registers"]["torque_enable"]["values"].values())
            o["ticks"] = ticks
            if o["usable"]:
                o["left"], o["right"], o["top"] = to_model(raw["left_wrist"][o["left_wrist_index"]]), to_model(raw["right_wrist"][o["right_wrist_index"]]), to_model(raw["top_color"][i], gray3=True)
            o["depth_mm"] = raw["top_depth"][i] if "top_depth" in raw.files else None
            obs.append(o)
    return obs


def cadence_limit(intervals):
    """Freshness of a camera stream from ITS OWN measured cadence: a frame is not fresh once more than one whole frame
    interval has been missed, i.e. its age exceeds two of the stream's recent median intervals. No fixed time constant;
    None (not judged) until the stream has shown a few intervals. This rule refuses an OBSERVATION (provisional input
    rule); it is not a motor gate."""
    x = list(intervals)
    return None if len(x) < 5 else 2.0 * float(np.median(x))


class SourceUnavailable(RuntimeError):
    def __init__(self, text, devices):
        super().__init__(text); self.devices = devices


class SdkTop:
    """The OS30A through the vendor SDK helper as the ONLY owner of the camera (no V4L2 reader on it): image, depth, stamps and
    the factory rectification log of the SAME FrameSet, streamed over a pipe. Same record format and decoder constants as
    sweepick_sdk_top (READ03). Stamps kept apart: the SDK's own stamp, the helper's host-monotonic receipt, this process's receipt."""
    HELPER = Path.home() / "DAPIER/tjj-calibration-package/v2/camera/build/registered-capture-helper-live"
    SERIAL = "OS30AA2X2CG0180"

    def __init__(self, workdir, framesets):
        import os, struct, subprocess, threading
        from sweepick.perception import sweepick_os30a_observation as st
        self.st, self.struct, self.os = st, struct, os
        self.workdir = Path(workdir); self.workdir.mkdir(parents=True, exist_ok=True)
        self.stop_file = self.workdir / "SDK_STOP"
        r, w = os.pipe()
        env = dict(os.environ, READ03_FRAMESETS=str(framesets), READ03_FULL_LAST=str(framesets), READ03_NO_PC="1", READ03_STOP_FILE=str(self.stop_file))
        env.pop("DAPIER_OS30A_DIAGNOSTIC_PREFIX", None)
        self.cmd = [str(self.HELPER), str(w), self.SERIAL, "--registered-point-cloud", str(self.workdir / "sdk.trace.log")]
        self.proc = subprocess.Popen(self.cmd, pass_fds=(w,), env=env, cwd=self.workdir, stderr=open(self.workdir / "sdk.stderr.log", "w"), stdout=subprocess.DEVNULL)
        os.close(w)
        self.r, self.latest, self.count, self.error, self.K, self.rect_sha = os.fdopen(r, "rb", buffering=0), None, 0, None, None, None
        import collections
        self.lock, self.done, self.intervals = threading.Lock(), threading.Event(), collections.deque(maxlen=60)
        self.thread = threading.Thread(target=self._reader, daemon=True); self.thread.start()

    def _read(self, n):
        buf = bytearray()
        while len(buf) < n:
            b = self.r.read(n - len(buf))
            if not b:
                return None
            buf += b
        return bytes(buf)

    def _reader(self):
        st = self.st
        try:
            while True:
                head = self._read(140)
                if head is None:
                    break
                h = dict(zip(st.HEADER, self.struct.unpack("<21I7Q", head)))
                if h["magic"] != st.MAGIC:
                    self.error = "bad record magic"; break
                sizes = [h["color_transport_bytes"], h["color_bgr_bytes"], h["depth_bytes"], h["pc_rgb_bytes"], h["pc_rgb_bytes"], h["pc_xyz_bytes"], h["rectify_log_bytes"]]
                parts = [self._read(k) if k else b"" for k in sizes]
                if any(x is None for x in parts):
                    self.error = "truncated record"; break
                recv = time.monotonic()
                if self.K is None:
                    rl = st.rectify_log(parts[6]); P = rl["NewCamMat1"]
                    self.K, self.rect_sha = [P[0], 0.0, P[2], 0.0, P[5], P[6], 0.0, 0.0, 1.0], rl["calibration_fields_sha256"]
                fs = dict(n=self.count + 1, frame_number=[h["color_frame_number"], h["depth_frame_number"]], sdk_color_stamp_s=h["color_sdk_timestamp_ns"] / 1e9, sdk_depth_stamp_s=h["depth_sdk_timestamp_ns"] / 1e9,
                          helper_receipt_monotonic_s=h["host_monotonic_ns"] / 1e9, recv_monotonic_s=recv,
                          bgr=np.frombuffer(parts[1], np.uint8).reshape(h["color_height"], h["color_width"], 3) if sizes[1] else None, zd=np.frombuffer(parts[2], np.uint16).reshape(h["depth_height"], h["depth_width"]))
                with self.lock:
                    if self.latest is not None:
                        self.intervals.append(fs["helper_receipt_monotonic_s"] - self.latest["helper_receipt_monotonic_s"])
                    self.latest, self.count = fs, self.count + 1
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
        finally:
            self.done.set()

    def get(self):
        with self.lock:
            return self.latest

    def close(self, wait_s=12.0):
        """Ask the helper to end through its normal close path (stop file); the pipe is drained meanwhile. Only if it does not
        end in time is this process's own child terminated."""
        rec = dict(framesets_received=self.count, reader_error=self.error)
        try:
            self.stop_file.write_text("stop")
        except Exception:
            pass
        t0 = time.monotonic()
        while self.proc.poll() is None and time.monotonic() - t0 < wait_s:
            time.sleep(0.05)
        rec["ended_by"] = "itself (normal close path)" if self.proc.poll() is not None else "terminated by its parent after the wait"
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except Exception:
                self.proc.kill(); self.proc.wait(timeout=3)
        self.thread.join(timeout=3)
        try:
            self.r.close()
        except Exception:
            pass
        trace = (self.workdir / "sdk.trace.log").read_text() if (self.workdir / "sdk.trace.log").exists() else ""
        rec.update(returncode=self.proc.returncode, close_stream_returned="closeStream_end=1" in trace, reader_thread_alive=self.thread.is_alive(), framesets_received=self.count)
        return rec


class WristCam:
    def __init__(self, name, device, open_capture=None):
        import threading
        import collections
        self.name, self.device, self.stop, self.lock, self.latest, self.prev, self.n, self.info = name, device, threading.Event(), threading.Lock(), None, None, 0, {}
        self.intervals = collections.deque(maxlen=60)
        self.cap = (open_capture or self._open)(device)
        if not self.cap.isOpened():
            raise RuntimeError(f"{name}: {device} could not be opened")
        self.thread = threading.Thread(target=self._run, daemon=True); self.thread.start()

    def _open(self, device):
        import cv2
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV")); cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        # No CAP_PROP_BUFFERSIZE = 1: on these cameras it halved the delivered rate (12 / 10 instead of 25 / 20 frames per second, measured 2026-10-08)
        f = int(cap.get(cv2.CAP_PROP_FOURCC))
        self.info = dict(device=device, requested="YUYV 320x240", fourcc="".join(chr((f >> 8 * i) & 0xFF) for i in range(4)), width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), driver_fps=cap.get(cv2.CAP_PROP_FPS))
        return cap

    def _run(self):
        while not self.stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.005); continue
            with self.lock:
                t = time.monotonic()
                if self.latest is not None:
                    self.intervals.append(t - self.latest["recv_monotonic_s"])
                self.prev, self.n = self.latest, self.n + 1
                self.latest = dict(frame=frame, recv_monotonic_s=t, n=self.n)

    def get(self):
        with self.lock:
            return self.latest, self.prev

    def close(self):
        self.stop.set(); self.thread.join(timeout=3)
        try:
            self.cap.release()
        except Exception:
            pass
        return dict(frames_read=self.n, thread_alive=self.thread.is_alive())


def open_readonly_bus(side):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    b = f5.ReadOnlyBus(FeetechMotorsBus(port=f"/dev/dapier/{side}_arm", motors={n: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES) for i, n in enumerate(rc.JOINTS)}))
    b.connect()
    return b


class LiveSource:
    """Fresh observations from the real devices, read-only: SDK-owned top FrameSet (image + depth + K + stamps), two wrist
    cameras (320x240) and both arm buses through sweepick_field.ReadOnlyBus. It owns exactly what it opened and releases all of it
    on a failed start, on an exception and on a normal end. It never writes a register."""

    def __init__(self, lim, workdir, *, framesets=900, open_bus=open_readonly_bus, open_wrist=None, open_sdk=None, top_wait_s=25.0):
        self.lim, self.devices = lim, {k: dict(state="not-attempted") for k in ("left_arm", "right_arm", "left_wrist", "right_wrist", "top_sdk")}
        self.bus, self.cam, self.sdk, self.last_ticks, self.bus_reads, self.closed = {}, {}, None, None, 0, None
        try:
            for s in ("left", "right"):
                self.devices[f"{s}_arm"]["state"] = "opening"
                self.bus[s] = open_bus(s); self.devices[f"{s}_arm"]["state"] = "opened"
            for s in ("left", "right"):
                self.devices[f"{s}_wrist"]["state"] = "opening"
                self.cam[s] = (open_wrist or (lambda name, dev: WristCam(name, dev)))(f"{s}_wrist", f"/dev/dapier/{s}_wrist_rgb"); self.devices[f"{s}_wrist"].update(state="opened", info=getattr(self.cam[s], "info", None))
            self.devices["top_sdk"]["state"] = "opening"
            self.sdk = (open_sdk or (lambda: SdkTop(Path(workdir) / "sdk", framesets)))(); self.devices["top_sdk"]["state"] = "opened"
            t0 = time.monotonic()
            while time.monotonic() - t0 < top_wait_s and (self.sdk.get() is None or any(c.get()[0] is None for c in self.cam.values())):
                if self.sdk.done.is_set() and self.sdk.get() is None:
                    raise RuntimeError(f"the SDK helper ended before its first FrameSet ({self.sdk.error})")
                time.sleep(0.05)
            if self.sdk.get() is None:
                raise RuntimeError("no FrameSet from the SDK within the wait")
            self.ready_after_s = round(time.monotonic() - t0, 2)
        except BaseException as e:
            for k, v in self.devices.items():
                if v["state"] == "opening":
                    v.update(state="failed", error=f"{type(e).__name__}: {str(e).splitlines()[0][:160]}")
            closed = self.close()
            raise SourceUnavailable(f"{type(e).__name__}: {str(e).splitlines()[0][:200]}", dict(devices=self.devices, released=closed))

    def next(self):
        now, o = time.monotonic(), dict(source="live", usable=True, why=[], t_monotonic=None)
        t0 = time.perf_counter(); ticks = {}
        try:
            for s in ("left", "right"):
                ticks[s] = {n: float(v) for n, v in self.bus[s].sync_read("Present_Position", normalize=False).items()}
                o["torque_" + s] = max(self.bus[s].sync_read("Torque_Enable", normalize=False).values())
            self.bus_reads += 1
        except Exception as e:
            o["usable"] = False; o["why"].append(f"bus read failed: {type(e).__name__}: {str(e).splitlines()[0][:120]}"); ticks = None
        o["io_bus_ms"] = round(1000 * (time.perf_counter() - t0), 2); o["bus_seq"] = self.bus_reads; o["ticks"] = ticks; o["t_bus_monotonic"] = time.monotonic()
        still = self.last_ticks is not None and ticks is not None and all(abs(ticks[s][n] - self.last_ticks[s][n]) <= 1 for s in ticks for n in rc.JOINTS)
        if ticks is not None:
            self.last_ticks = ticks
        now = time.monotonic(); o["t_monotonic"] = now
        fs = self.sdk.get(); frames = {s: self.cam[s].get() for s in self.cam}
        o["frame_n"] = dict(top=fs["n"] if fs else None, **{f"{s}_wrist": (frames[s][0] or {}).get("n") for s in frames})
        o["age_ms"] = {}
        if fs is None:
            o["usable"] = False; o["why"].append("top: no FrameSet")
        else:
            o["age_ms"]["top_since_helper_receipt"] = round(1000 * (now - fs["helper_receipt_monotonic_s"]), 1); o["age_ms"]["top_since_this_process_received"] = round(1000 * (now - fs["recv_monotonic_s"]), 1)
            o["top"] = dict(frame_number=fs["frame_number"], sdk_color_stamp_s=fs["sdk_color_stamp_s"], sdk_depth_minus_color_ms=round(1000 * (fs["sdk_depth_stamp_s"] - fs["sdk_color_stamp_s"]), 3), depth_valid_fraction=float((fs["zd"] > 0).mean()))
            lim_top = cadence_limit(getattr(self.sdk, "intervals", ())); o.setdefault("freshness_limit_ms", {})["top"] = None if lim_top is None else round(1000 * lim_top, 1)
            if lim_top is not None and now - fs["helper_receipt_monotonic_s"] > lim_top:
                o["usable"] = False; o["why"].append(f"top: FrameSet is {o['age_ms']['top_since_helper_receipt']:.0f} ms old, more than two of its own frame intervals (stale)")
            if self.sdk.done.is_set():
                o["usable"] = False; o["why"].append("top: the SDK stream has ended")
        import cv2
        g = lambda f: cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (160, 120), interpolation=cv2.INTER_AREA).astype(np.float32)
        for s in frames:
            cur, prev = frames[s]
            if cur is None:
                o["usable"] = False; o["why"].append(f"{s}_wrist: no frame"); continue
            age = now - cur["recv_monotonic_s"]; o["age_ms"][f"{s}_wrist"] = round(1000 * age, 1)
            lim_w = cadence_limit(getattr(self.cam[s], "intervals", ())); o.setdefault("freshness_limit_ms", {})[f"{s}_wrist"] = None if lim_w is None else round(1000 * lim_w, 1)
            if lim_w is not None and age > lim_w:
                o["usable"] = False; o["why"].append(f"{s}_wrist: frame is {age * 1000:.0f} ms old, more than two of its own frame intervals (stale)")
            if prev is not None:
                diff = float(np.abs(g(cur["frame"]) - g(prev["frame"])).mean())
                o.setdefault("integrity", {})[f"{s}_wrist"] = dict(checked=bool(still), difference_to_previous=round(diff, 2))
                if still and diff > f6.INTEGRITY_MAX_NEIGHBOUR_DIFF:
                    o["usable"] = False; o["why"].append(f"{s}_wrist: differs from the previous frame by {diff:.1f} with the arms at rest (corrupted)")
        o["io_total_ms"] = round(1000 * (time.perf_counter() - t0), 2)
        if o["usable"]:
            t1 = time.perf_counter()
            o["left"], o["right"], o["top_img"] = to_model(frames["left"][0]["frame"]), to_model(frames["right"][0]["frame"]), to_model(fs["bgr"], gray3=True)
            o["preprocess_ms"] = round(1000 * (time.perf_counter() - t1), 2)
        o["depth_mm"], o["K"] = (fs["zd"] if fs else None), self.sdk.K
        return o

    def read_goal_registers(self):
        """Read-only: what the servos hold as Goal_Position / Torque_Enable (for the record of the real applied state)."""
        out = {}
        for s, b in self.bus.items():
            try:
                out[s] = dict(goal=dict(b.sync_read("Goal_Position", normalize=False)), torque=dict(b.sync_read("Torque_Enable", normalize=False)))
            except Exception as e:
                out[s] = dict(error=f"{type(e).__name__}: {str(e).splitlines()[0][:120]}")
        return out

    def close(self):
        if self.closed is not None:
            return self.closed
        rec = {}
        for s, c in self.cam.items():
            try:
                rec[f"{s}_wrist"] = c.close()
            except Exception as e:
                rec[f"{s}_wrist"] = dict(error=str(e))
        if self.sdk is not None:
            try:
                rec["top_sdk"] = self.sdk.close()
            except Exception as e:
                rec["top_sdk"] = dict(error=str(e))
        for s, b in self.bus.items():
            try:
                b.disconnect(); rec[f"{s}_arm"] = "disconnected (the wrapper passes disable_torque=False: no register is written)"
            except Exception as e:
                rec[f"{s}_arm"] = dict(error=str(e))
        self.closed = rec
        return rec


def real_area(depth_mm, K_image, T_camera_from_board, stamp, center_xy, radius_m=0.04):
    """The existing tjj_perception.observe_area on a REAL depth frame, in the board frame (z up), with the rule that a pixel
    without depth is 'not observed'. Returns the existing AreaObservation fields plus counts. Frame = the printed board:
    this is not a target in an arm frame (the camera-to-base transform is not verified)."""
    import tjj_perception as tp, tjj_task_manager as tm
    Kd = np.array(K_image, dtype=float).reshape(3, 3).copy(); Kd[:2] /= 2
    Twc = np.diag([1.0, -1.0, -1.0, 1.0]) @ np.linalg.inv(np.array(T_camera_from_board, dtype=float))
    reg = tm.Region("real", "board_z_up", tuple(center_xy), radius_m, 0.0)
    dm = np.asarray(depth_mm, dtype=np.float64) / 1000.0
    a = tp.observe_area(dm, Kd, Twc, stamp, reg, unseen_is_unknown=True)
    pts = tp.back_project(dm, Kd, Twc)
    inside = pts[np.hypot(pts[:, 0] - reg.center_xy[0], pts[:, 1] - reg.center_xy[1]) <= reg.radius_m]
    h = inside[:, 2] if len(inside) else np.zeros(0)
    return dict(state="OCCUPIED" if a.points_above_surface else ("UNKNOWN (not fully observed)" if not a.validity.valid else "CLEAR"), valid=a.validity.valid, clear=a.clear, reason=a.validity.reason, pixels_with_depth=int(len(inside)),
                pixels_without_depth=int(tp.unseen_on_surface(dm, Kd, Twc, reg)), points_above_8mm=int(a.points_above_surface), height_mm=dict(p50=float(np.percentile(h, 50) * 1000), p99=float(np.percentile(h, 99) * 1000), max=float(h.max() * 1000)) if len(h) else None)


def load_runtime():
    from dapier_act_policy import ACTPolicy
    from sim_data_factory import student_eval
    import tjj_mvp_r2 as r2m, tjj_mvp_r3 as r3, receive_candidate as rcand
    t = time.perf_counter()
    model = mujoco.MjModel.from_binary_path(str(RT / "model/dual_scene_desk_both.mjb")); data = mujoco.MjData(model)
    mapping = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", CAL, IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")
    lim = tt.load_limits()
    three = r2m.install_three_cameras()
    loaded = {}
    for name in ("START_3V", "RECEIVE_3V"):
        ck = RT / "checkpoints" / name / "step"
        pol = ACTPolicy.from_pretrained(ck, local_files_only=True, strict=True).cuda().eval()
        with np.load(ck.parent / "normalization.npz", allow_pickle=False) as z:
            stats = {k: {x: z[k + "_" + x].copy() for x in ("mean", "std")} for k in ("state", "action")}
        pol.check_normalization(stats)
        loaded[name] = (pol, stats, sha(ck / "model.safetensors"))
    return dict(model=model, data=data, mapping=mapping, lim=lim, three=three, loaded=loaded, student_eval=student_eval, r3=r3, prefix=rcand.PREFIX, load_s=round(time.perf_counter() - t, 2))


class Owner:
    """Which policy answers. Without task state the owner is START for the whole session. `branch_exercise_at` switches to
    RECEIVE at a step number ONLY to exercise that branch: it is not a pick-complete transition and is labelled as such."""
    def __init__(self, rt, sensor, branch_exercise_at=None):
        self.rt, self.sensor, self.at, self.name, self.arb, self.student = rt, sensor, branch_exercise_at, None, None, None

    def select(self, seq):
        want = "RECEIVE_3V" if (self.at is not None and seq >= self.at) else "START_3V"
        if want != self.name:
            if self.arb is not None:
                self.arb.close()                                              # the old chunk is dropped
            self.name = want; pol, st, _ = self.rt["loaded"][want]; pol.reset()
            self.student = self.rt["student_eval"].Student(pol, st, self.sensor, self.rt["prefix"], None, "project")
            self.arb = self.rt["r3"].Arbiter(Shim(), self.student, f"ACT:{want}")
        return self.arb, self.student

    @property
    def selection(self):
        return "fixed START (no task state is read)" if self.at is None else f"BRANCH EXERCISE: START, then RECEIVE from step {self.at} by step count; NOT a task transition"


def owner_step(rt, sensor, owner, seq, q, images):
    """One step of the existing owner on a mirror of the measured state. Returns (arb, student, command, new_inference, ms)."""
    import torch
    arb, student = owner.select(seq)
    sensor.current = dict(left_wrist_rgb=dict(rgb=images["left"]), right_wrist_rgb=dict(rgb=images["right"])); rt["three"].update(top=dict(rgb=images["top"]))
    sweepick_kin.set_q12(rt["model"], rt["data"], q, t=seq * rt["lim"]["dt"])
    t = time.perf_counter(); before = student.calls
    command, _ = arb.next_command(dict(model=rt["model"], data=rt["data"], control_dt=rt["lim"]["dt"]))
    new = student.calls != before
    if new:
        torch.cuda.synchronize()                                              # the forward has really finished before the clock is read
    return arb, student, command, new, 1000 * (time.perf_counter() - t)


def run_fixture(a, out, rt):
    """TEST FIXTURE: stored observations in turn, the arm assumed to follow the goal perfectly (the applier is given its own
    goal as the measurement). Logical period 20 ms, replayed as fast as it runs. Not a live result and not a feedback result."""
    import torch
    lim, mapping = rt["lim"], rt["mapping"]
    obs = replay_observations(a.replay, mapping)
    sensor = MirrorSensor(); owner = Owner(rt, sensor, a.branch_exercise_at)
    f = open(out / "FIXTURE_IDEAL_FOLLOWING.jsonl", "w")
    f.write(json.dumps(dict(schema="tjj.local-loop.v2", mode="FIXTURE_IDEAL_FOLLOWING", meaning="test fixture: stored observations cycled, logical dt 20 ms, the applier is fed its own goal as the measurement. SEND counts are not feedback results", device=torch.cuda.get_device_name(0),
                            owner_selection=owner.selection, observations=len(obs), usable=sum(o["usable"] for o in obs), sources=list(a.replay), commands_sent=0)) + "\n")
    first = next(o for o in obs if o["usable"]); app = rx.Applier(mapping, lim, first["ticks"]); status, forwards = {}, 0
    t0 = time.perf_counter()
    for seq in range(a.steps):
        o = obs[(seq + seq // rt["prefix"]) % len(obs)]
        ideal = {s: dict(app.goal[s]) for s in rx.SIDES}
        if not o["usable"]:
            d = app.dispatch(owner.select(seq)[0].name, None, ideal, stop=True, closed_loop=False); row = dict(seq=seq, consumed=False, why=o["why"])
        else:
            arb, student, command, new, ms = owner_step(rt, sensor, owner, seq, rc.ticks_to_q12(mapping, o["ticks"]).astype(np.float32), dict(left=o["left"], right=o["right"], top=o["top"]))
            forwards += new
            d = app.dispatch(arb.name, command, ideal, closed_loop=False); row = dict(seq=seq, consumed=True, owner=arb.name, request_id=student.calls, chunk_index=student.trace[-1]["selected_chunk_index"], new_inference=bool(new), owner_step_ms=round(ms, 2))
        status[d.status] = status.get(d.status, 0) + 1
        f.write(json.dumps(dict(row, measurement="SYNTHETIC (ideal following)", dispatch=d.status, planned_write=d.write, commands_sent=0), default=str) + "\n")
    summ = dict(summary=True, mode="FIXTURE_IDEAL_FOLLOWING", steps=a.steps, logical_s=a.steps * lim["dt"], wall_s=round(time.perf_counter() - t0, 2), forwards=forwards, dispatch_status_under_the_ideal_assumption=status, commands_sent=0)
    f.write(json.dumps(summ) + "\n"); f.close()
    return 0, summ


def run_live(a, out, rt, source_factory=None):
    """LIVE_READONLY: real fresh observations and real joint readback drive the owner. There is no writer: the 'planned write'
    is what WOULD be written, computed on a SYNTHETIC plant that is put back on the real measured position at every new
    chunk. The arm not following a command that was never sent is not a tracking failure and is not counted as one."""
    import torch
    lim, mapping = rt["lim"], rt["mapping"]
    rec = dict(schema="tjj.local-loop.v2", mode="LIVE_READONLY", device=torch.cuda.get_device_name(0), torch=torch.__version__, cuda=torch.version.cuda, models={k: v[2] for k, v in rt["loaded"].items()}, model_load_s=rt["load_s"],
               mapping="CANDIDATE q12 (verified=false): a connection / timing session, not a control or geometry verification", period_s=lim["dt"], prefix=rt["prefix"], writer="NONE: this program has no Goal_Position / Torque_Enable / base write path", commands_sent=0)
    path = out / "LIVE_READONLY.jsonl"
    src = None
    try:
        try:
            src = (source_factory or (lambda: LiveSource(lim, out, framesets=int((a.seconds + 20) * 34))))()
        except SourceUnavailable as e:
            rec.update(result="NOT RUN", reason=str(e), **e.devices)
            path.write_text(json.dumps(rec, default=str) + "\n")
            return 3, rec
        sensor = MirrorSensor(); owner = Owner(rt, sensor, a.branch_exercise_at)
        # ---- warm-up, kept apart from the session ---------------------------------------------------------
        w0 = time.perf_counter(); o = None
        while time.perf_counter() - w0 < 3.0:
            o = src.next()
            if o["usable"]:
                break
            time.sleep(0.02)
        warm = dict(usable_observation=bool(o and o["usable"]), why=(o or {}).get("why"))
        if o and o["usable"]:
            for name in (["START_3V"] + (["RECEIVE_3V"] if a.branch_exercise_at is not None else [])):
                pol, st, _ = rt["loaded"][name]; pol.reset()
                stu = rt["student_eval"].Student(pol, st, sensor, rt["prefix"], None, "project")
                sensor.current = dict(left_wrist_rgb=dict(rgb=o["left"]), right_wrist_rgb=dict(rgb=o["right"])); rt["three"].update(top=dict(rgb=o["top_img"]))
                sweepick_kin.set_q12(rt["model"], rt["data"], rc.ticks_to_q12(mapping, o["ticks"]).astype(np.float32), t=0.0)
                t = time.perf_counter(); stu.next_command(dict(model=rt["model"], data=rt["data"], control_dt=lim["dt"])); torch.cuda.synchronize()
                warm[name + "_ms"] = round(1000 * (time.perf_counter() - t), 1)
        before = src.read_goal_registers()
        rec.update(devices=src.devices, sources_ready_after_s=src.ready_after_s, top=dict(owner="vendor SDK helper (single owner; no V4L2 reader on the OS30A)", K_rectified=src.sdk.K, rectify_fields_sha256=src.sdk.rect_sha, depth="uint16 mm of the same FrameSet (0 = invalid)", helper=src.sdk.cmd[0]),
                   clocks=dict(loop_bus_wrist="this process, time.monotonic()", top_helper_receipt="helper CLOCK_MONOTONIC when it received the FrameSet (same clock as the loop)", top_sdk_stamp="SDK's own stamp (host realtime microseconds; exposure or reception not stated): recorded, never compared with the monotonic ones"),
                   warm_up=warm, owner_selection=owner.selection, real_goal_registers_before=before)
        f = open(path, "w"); f.write(json.dumps(rec, default=str) + "\n")
        app = None; anchor_seq = None; last_start = None; seq = 0
        T = dict(io=[], pre=[], owner=[], forward=[], dispatch=[], log=[], total=[], interval=[]); ages = {}; reasons = {}; consumed = 0; tops, wl, wr, tickset = [], [], [], set(); reuse = dict(top=0, left_wrist=0, right_wrist=0); forwards = []; owners = {}; planned = {}
        last_n = {}
        s0 = time.monotonic()
        while time.monotonic() - s0 < a.seconds:
            c0 = time.perf_counter(); m0 = time.monotonic()
            if last_start is not None:
                T["interval"].append(1000 * (m0 - last_start))
            last_start = m0
            o = src.next()
            row = dict(seq=seq, t_s=round(m0 - s0, 4), usable=o["usable"], why=o["why"], frame_n=o["frame_n"], bus_seq=o["bus_seq"], age_ms=o["age_ms"], freshness_limit_ms=o.get("freshness_limit_ms"), integrity=o.get("integrity"), torque=[o.get("torque_left"), o.get("torque_right")], top=o.get("top"),
                       measured_ticks=o["ticks"], measurement_source="REAL bus readback (read-only)", commands_sent=0)
            for k, v in o["age_ms"].items():
                ages.setdefault(k, []).append(v)
            t_owner = t_disp = 0.0
            if not o["usable"]:
                for w in o["why"]:
                    key = w.split(":")[0] + ": " + ("stale" if "stale" in w else "corrupted" if "corrupted" in w else w.split(":", 1)[1].strip()[:40])
                    reasons[key] = reasons.get(key, 0) + 1
                row.update(consumed=False, planned="none: the observation was not consumed")
            else:
                consumed += 1
                for k, lst in (("top", tops), ("left_wrist", wl), ("right_wrist", wr)):
                    n = o["frame_n"][k]
                    if last_n.get(k) == n:
                        reuse[k] += 1
                    last_n[k] = n; lst.append(n)
                tickset.add(tuple(o["ticks"][s][n] for s in rx.SIDES for n in rc.JOINTS))
                q = rc.ticks_to_q12(mapping, o["ticks"]).astype(np.float32)
                arb, student, command, new, t_owner = owner_step(rt, sensor, owner, seq, q, dict(left=o["left"], right=o["right"], top=o["top_img"]))
                owners[arb.name] = owners.get(arb.name, 0) + 1
                if arb.failure:
                    row.update(consumed=True, owner=arb.name, owner_failure=arb.failure, planned="none: the owner failed")
                else:
                    if new or app is None:                                    # SYNTHETIC plant back on the real measured position: nothing was sent, nothing moved
                        app = rx.Applier(mapping, lim, o["ticks"]); anchor_seq = seq
                        forwards.append(dict(seq=seq, t_s=row["t_s"], owner=arb.name, request_id=student.calls, anchor_frame_n=o["frame_n"], anchor_bus_seq=o["bus_seq"], owner_step_ms=round(t_owner, 2)))
                    t = time.perf_counter()
                    d = app.dispatch(arb.name, command, {s: dict(app.goal[s]) for s in rx.SIDES}, closed_loop=False)        # measurement of the SYNTHETIC plant only
                    t_disp = 1000 * (time.perf_counter() - t)
                    planned[d.status] = planned.get(d.status, 0) + 1
                    tr = student.trace[-1]
                    gap = max(abs(o["ticks"][s][n] - app.goal[s][n]) for s in rx.SIDES for n in rc.JOINTS)
                    row.update(consumed=True, owner=arb.name, owner_selected_by=owner.selection, request_id=student.calls, chunk_id=tr["chunk_generation_id"], anchor_seq=anchor_seq, chunk_index=tr["selected_chunk_index"], new_inference=bool(new),
                               raw_target_ticks={s: {n: round(d.raw_target[s][n], 1) for n in rc.JOINTS} for s in rx.SIDES} if d.raw_target else None,
                               planned=dict(plant="SYNTHETIC ideal-following, reset to the real measurement at each new chunk", status_on_the_synthetic_plant=d.status, reasons=d.reasons, would_write=d.write, sent=False),
                               real_vs_planned=dict(max_abs_ticks=round(gap, 1), note="the arm stays where it is because nothing is sent; this is not a tracking failure and no gate is applied to it"))
            t = time.perf_counter(); f.write(json.dumps(row, default=str) + "\n"); t_log = 1000 * (time.perf_counter() - t)
            total = 1000 * (time.perf_counter() - c0)
            T["io"].append(o.get("io_total_ms", 0.0)); T["pre"].append(o.get("preprocess_ms", 0.0)); T["owner"].append(t_owner); T["dispatch"].append(t_disp); T["log"].append(t_log); T["total"].append(total)
            if row.get("new_inference"):
                T["forward"].append(t_owner)
            seq += 1
            rest = lim["dt"] - (time.perf_counter() - c0)
            if rest > 0:
                time.sleep(rest)
        wall = time.monotonic() - s0
        after = src.read_goal_registers()
        st = lambda x: dict(median=round(float(np.median(x)), 2), p95=round(float(np.percentile(x, 95)), 2), max=round(float(np.max(x)), 2)) if len(x) else None
        budget = 1000 * lim["dt"]
        summ = dict(summary=True, mode="LIVE_READONLY", wall_s=round(wall, 2), cycles=seq, consumed=consumed, refused=seq - consumed, refusal_reasons=reasons,
                    unique=dict(top_framesets=len(set(tops)), left_wrist_frames=len(set(wl)), right_wrist_frames=len(set(wr)), bus_reads=src.bus_reads, distinct_joint_tick_vectors=len(tickset)),
                    reused_in_consecutive_consumed_cycles=reuse, reuse_note="cameras deliver about 30 frames per second and the loop runs at 50 Hz: a frame inside its validity time may be used twice; it is counted once as a unique frame",
                    age_ms={k: st(v) for k, v in ages.items()}, loop_interval_ms=st(T["interval"]),
                    timing_ms=dict(io_bus_and_frames=st(T["io"]), preprocess=st(T["pre"]), owner_step=st(T["owner"]), owner_step_with_forward=st(T["forward"]), dispatch=st(T["dispatch"]), log=st(T["log"]), cycle_total=st(T["total"]), budget=budget,
                                   cycles_over_budget_including_acquisition=int(sum(x > budget for x in T["total"])), cycles_over_budget_excluding_acquisition=int(sum((t_ - i_) > budget for t_, i_ in zip(T["total"], T["io"])))),
                    forwards=len(forwards), forward_list=forwards, owners=owners, owner_selection=owner.selection, planned_status_on_the_synthetic_plant=planned,
                    q12="CANDIDATE mapping; measurement REAL; plant for the planned commands SYNTHETIC", real_goal_registers_after=after, real_registers_unchanged=bool(before == after), writes=dict(goal=0, torque=0, base=0), tracking_with_the_real_arm="not evaluated: no command was sent")
        f.write(json.dumps(summ, default=str) + "\n"); f.close()
        return 0, summ
    except Exception as e:                                                    # a failure inside the session: recorded, then everything this program opened is released below
        import traceback
        tb = traceback.extract_tb(e.__traceback__)[-1]
        with open(path, "a") as g:
            g.write(json.dumps(dict(aborted=True, reason=f"{type(e).__name__}: {str(e).splitlines()[0][:200]}", at=f"{Path(tb.filename).name}:{tb.lineno}", commands_sent=0)) + "\n")
        return 4, dict(mode="LIVE_READONLY", result="ABORTED", reason=f"{type(e).__name__}: {str(e).splitlines()[0][:200]}", at=f"{Path(tb.filename).name}:{tb.lineno}", commands_sent=0)
    finally:
        if src is not None:
            released = src.close()
            with open(path, "a") as g:
                g.write(json.dumps(dict(released=released), default=str) + "\n")


def probe(out, lim, seconds=3.0):
    """ONE read-only attempt per device, each on its own, so that a failure of one does not hide the state of the others.
    Buses: handshake ping + one Present_Position read. Cameras: frames for a few seconds (rates, ages). Everything opened
    here is released here. No model, no command."""
    rec = dict(schema="tjj.live-probe.v1", devices={}, commands_sent=0)
    for s in ("left", "right"):
        d = dict(state="not-attempted")
        try:
            d["state"] = "opening"; b = open_readonly_bus(s); d["state"] = "opened"
            try:
                d["present"] = {n: float(v) for n, v in b.sync_read("Present_Position", normalize=False).items()}; d["torque"] = max(b.sync_read("Torque_Enable", normalize=False).values()); d["state"] = "read"
            finally:
                b.disconnect(); d["released"] = True
        except Exception as e:
            d.update(state="failed", error=f"{type(e).__name__}: {' | '.join(str(e).splitlines()[:8])[:400]}", cause="NOT determined here: the USB adapter answers, the motors did not answer the ping. Motor power off is one candidate (to be confirmed on site)")
        rec["devices"][f"{s}_arm"] = d
    cams, sdk = {}, None
    try:
        for s in ("left", "right"):
            d = dict(state="opening")
            try:
                cams[s] = WristCam(f"{s}_wrist", f"/dev/dapier/{s}_wrist_rgb"); d.update(state="opened", info=cams[s].info)
            except Exception as e:
                d.update(state="failed", error=f"{type(e).__name__}: {e}")
            rec["devices"][f"{s}_wrist"] = d
        d = dict(state="opening")
        try:
            sdk = SdkTop(Path(out) / "sdk_probe", framesets=int((seconds + 6) * 34)); d["state"] = "opened"
            t0 = time.monotonic()
            while time.monotonic() - t0 < 25 and sdk.get() is None and not sdk.done.is_set():
                time.sleep(0.05)
            d["first_frameset_after_s"] = round(time.monotonic() - t0, 2)
            if sdk.get() is None:
                d.update(state="failed", error=f"no FrameSet ({sdk.error})")
        except Exception as e:
            d.update(state="failed", error=f"{type(e).__name__}: {e}")
        rec["devices"]["top_sdk"] = d
        seen = dict(top=[], left=[], right=[]); age = dict(top_helper=[], top_recv=[], left=[], right=[]); t0 = time.monotonic(); valid = []
        while time.monotonic() - t0 < seconds:
            now = time.monotonic()
            fs = sdk.get() if sdk else None
            if fs:
                seen["top"].append(fs["n"]); age["top_helper"].append(1000 * (now - fs["helper_receipt_monotonic_s"])); age["top_recv"].append(1000 * (now - fs["recv_monotonic_s"])); valid.append(float((fs["zd"] > 0).mean()))
            for s, c in cams.items():
                cur, _ = c.get()
                if cur:
                    seen[s].append(cur["n"]); age[s].append(1000 * (now - cur["recv_monotonic_s"]))
            time.sleep(lim["dt"])
        st = lambda x: dict(median=round(float(np.median(x)), 1), p95=round(float(np.percentile(x, 95)), 1), max=round(float(np.max(x)), 1)) if len(x) else None
        rec["streams"] = dict(seconds=seconds, polls=int(seconds / lim["dt"]), unique_frames={k: len(set(v)) for k, v in seen.items()}, rate_hz={k: round((max(v) - min(v)) / seconds, 1) if v else None for k, v in seen.items()}, age_ms={k: st(v) for k, v in age.items()},
                              frame_interval_ms=dict(top=st([1000 * x for x in sdk.intervals]) if sdk else None, **{k: st([1000 * x for x in c.intervals]) for k, c in cams.items()}), top_depth_valid_fraction=st(valid), top_K=sdk.K if sdk else None, top_image_shape=list(sdk.get()["bgr"].shape) if sdk and sdk.get() else None)
        if sdk and sdk.get():
            rec["devices"]["top_sdk"]["state"] = "read"
        for s in cams:
            if seen[s]:
                rec["devices"][f"{s}_wrist"]["state"] = "read"
    finally:
        rec["released"] = {f"{s}_wrist": c.close() for s, c in cams.items()}
        if sdk is not None:
            rec["released"]["top_sdk"] = sdk.close()
    (Path(out) / "LIVE_PROBE.json").write_text(json.dumps(rec, indent=1, default=str))
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("live", "fixture", "probe")); ap.add_argument("out"); ap.add_argument("--replay", nargs="+"); ap.add_argument("--steps", type=int, default=300); ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--branch-exercise-at", type=int, default=None, help="switch START -> RECEIVE at this step: a branch exercise only")
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=False)
    if a.mode == "probe":
        rec = probe(out, tt.load_limits())
        print(json.dumps(rec, indent=1, default=str)); return 0
    rt = load_runtime()
    code, summ = (run_live if a.mode == "live" else run_fixture)(a, out, rt)
    print(json.dumps({k: v for k, v in summ.items() if k not in ("forward_list", "real_goal_registers_after", "real_goal_registers_before")}, indent=1, default=str))
    return code


if __name__ == "__main__":
    sys.exit(main())
