"""TJJ field tools R6, READ-ONLY: one path from the real feedback to the model's q12, with time pairing and separate
readiness levels. Nothing here sends a motor command, a torque enable / disable or a base cmd_vel. (sweepick_field.py of R5 is
unchanged; its decoders and the read-only bus wrapper are reused.)

  contract     the R6 field contract template
  capture      ROS topics + V4L2 wrist devices read AT THE SAME TIME (threads), optional bus feedback in the same window
  arms         bus feedback only: a time series of command / measured registers, one read window per sample
  model-input  raw sample + feedback series -> left / right / top + q12, paired by time; prints the readiness levels
  shadow       CUDA forward of the frozen policies on that input; sends nothing

raw tick --(lerobot calibration file: range_min / range_max, the driver's own formula)--> driver units (deg, 0..100)
         --(SI)--> rad / opening percent --(sim alignment of the contract: sign, offset, gripper closed / open)--> model q12
Each arrow is recorded with its source and fingerprint. Goal_Position (command) and Present_Position (measured) are two
channels and are never copied into each other. Out-of-range values are recorded and flagged, never clipped.
"""
import argparse
import hashlib
import json
import math
import threading
import time
from pathlib import Path

import numpy as np

from sweepick.perception import sweepick_observation_capture as f5

JOINTS, MODEL_W, MODEL_H = f5.JOINTS, f5.MODEL_W, f5.MODEL_H
REGISTERS = (("goal_raw", "Goal_Position", "command"), ("present_raw", "Present_Position", "measured"), ("velocity_raw", "Present_Velocity", "measured"), ("load_raw", "Present_Load", "measured"),
             ("current_raw", "Present_Current", "measured"), ("torque_enable", "Torque_Enable", "state"))
TICKS = 4096                      # STS3215 resolution (lerobot model_resolution_table); the driver divides by TICKS - 1


def template():
    c = f5.template()
    c["schema"] = "tjj.field-contract.v2"
    c["timing"] = dict(reference="top_color", max_skew_s=dict(wrist=None, joints=None, depth=None), max_clock_offset_s=None,
                       note="device settings. null = one frame period of the slower of the two streams, taken from the measured rates of the capture (recorded with the result). "
                            "Online control needs the measured latency of each stream, which is not known yet")
    c["depth"] = dict(registered_to_color=None, K=None, frame_id=None, T_color_from_depth=None,
                      note="registered_to_color: true only if the driver publishes depth aligned to the colour image (then the colour K applies). Otherwise give the depth K (9 numbers) and the "
                           "depth -> colour transform; without them the depth is kept raw and not projected")
    for side in ("left", "right"):
        g = c["q12"][side]["gripper"]
        g.clear()
        g.update(driver_percent_closed=None, driver_percent_open=None, verified=False)
        for j in c["q12"][side]["joints"]:
            j["note"] = "model angle [rad] = sign * driver angle [rad] + offset_rad"
    c["q12"]["unit_in"] = "driver units from the calibration file: arm joints in degrees about the calibrated mid range, gripper in percent of its calibrated range (not clipped)"
    c["q12"]["note"] = ("stage 1 (ticks -> driver units) uses the existing lerobot calibration files and is checked by the files' own ranges. stage 2 (driver -> SIM model joints) is the sim-to-real "
                        "alignment: sign, offset_rad per arm joint and the gripper's closed / open percent. It does not exist yet; null keeps q12 UNVERIFIED. Do not set verified without a measurement")
    return c


def sha_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def content_sha(joints):
    """SHA of the calibration CONTENT (the numbers that convert ticks), independent of the file's formatting or path."""
    return hashlib.sha256(json.dumps({n: {k: joints[n][k] for k in sorted(joints[n])} for n in sorted(joints)}, sort_keys=True).encode()).hexdigest()


METADATA = ("Model_Number", "Firmware_Major_Version", "Firmware_Minor_Version", "ID", "Baud_Rate", "Angular_Resolution", "Homing_Offset", "Operating_Mode", "Min_Position_Limit", "Max_Position_Limit",
            "Max_Torque_Limit", "Protection_Current", "Torque_Limit", "Present_Voltage", "Present_Temperature", "Status", "Moving", "Lock")


def read_metadata(bus):
    """Device registers read ONCE (identity, resolution / mode, limits, offsets). Unsupported ones are UNAVAILABLE."""
    out = {}
    for reg in METADATA:
        try:
            got = bus.sync_read(reg, normalize=False)
            out[reg] = dict(status="READ", values={n: float(got[n]) for n in JOINTS})
        except f5.ReadOnlyViolation:
            raise
        except Exception as e:
            out[reg] = dict(status="UNAVAILABLE", reason=f"{type(e).__name__}: {str(e)[:120]}")
    return out


def calibration_snapshot(meta, feedback):
    """The calibration bound to a sample at capture time, checked against its own recorded content SHA."""
    cap = (feedback or {}).get("calibration_at_capture") or meta.get("calibration_at_capture") or {}
    out = {}
    for side in ("left", "right"):
        c = cap.get(side) or {}
        if c.get("status") != "READ" or "joints" not in c:
            out[side] = dict(status="MISSING", reason="no calibration snapshot is bound to this sample for this arm" + (f" (capture said: {c.get('status')})" if c else ""))
            continue
        now = content_sha(c["joints"])
        rec = c.get("content_sha256")
        if rec is not None and rec != now:
            out[side] = dict(status="INVALID", reason="the snapshot's content does not match the content SHA recorded with it", recorded_content_sha256=rec, actual_content_sha256=now)
            continue
        out[side] = dict(c, content_sha256=now, source="calibration snapshot bound to the sample at capture", content_sha_recorded=rec is not None,
                         content_sha_note=None if rec is not None else "the sample (R6 format) recorded only the file SHA; the content SHA is computed now from the stored numbers")
    return out


# ----------------------------------------------------------------------------------------------
# stage 1: raw ticks -> driver units (the existing lerobot calibration)
# ----------------------------------------------------------------------------------------------

def load_calibration(contract):
    cfg, out = contract["lerobot"], {}
    for side in ("left", "right"):
        f = Path(cfg["calibration_dir"]).expanduser() / f"{cfg[f'{side}_id']}.json"
        if not f.exists():
            out[side] = dict(status="MISSING", file=str(f))
            continue
        data = json.loads(f.read_text())
        problems = [n for n in JOINTS if n not in data] + [n for n in JOINTS if n in data and data[n]["range_max"] == data[n]["range_min"]]
        ids = [data[n]["id"] for n in JOINTS if n in data]
        if problems or len(set(ids)) != len(ids):
            out[side] = dict(status="INVALID", file=str(f), reason=f"missing / degenerate joints {problems} or duplicate motor ids {ids}")
            continue
        out[side] = dict(status="READ", file=str(f), sha256=sha_file(f), content_sha256=content_sha({n: data[n] for n in JOINTS}), joints={n: data[n] for n in JOINTS}, motor_ids=ids, loaded=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                         formula="lerobot MotorsBus._normalize: arm joints deg = (tick - (min + max) / 2) * 360 / 4095; gripper percent = (tick - min) / (max - min) * 100; drive_mode inverts; NOT clipped here")
    return out


def driver_units(ticks, cal):
    """{name: tick} -> {name: dict(value, unit, raw, in_calibrated_range)} with the calibration of one arm."""
    out = {}
    for n in JOINTS:
        v = ticks.get(n)
        c = cal["joints"][n]
        if v is None or not math.isfinite(v):
            out[n] = dict(value=None, raw=v, unit=None, valid=False, reason="missing or non-finite tick")
            continue
        lo, hi, inv = c["range_min"], c["range_max"], bool(c.get("drive_mode"))
        if n == "gripper":
            p = (v - lo) / (hi - lo) * 100
            val, unit = (100 - p if inv else p), "percent of the calibrated gripper range"
        else:
            d = (v - (lo + hi) / 2) * 360 / (TICKS - 1)
            val, unit = (-d if inv else d), "deg about the calibrated mid range"
        out[n] = dict(value=float(val), raw=float(v), unit=unit, valid=True, in_calibrated_range=bool(lo <= v <= hi), motor_id=c["id"])
    return out


# ----------------------------------------------------------------------------------------------
# feedback time series (bus or JointState) in one schema
# ----------------------------------------------------------------------------------------------

def read_feedback_series(bus, *, samples, period_s, clock=time.time, stop=None, sleep=time.sleep):
    """One read window per sample: every register is read once inside [t_start, t_end]. The registers of a sample are
    read one after the other, not at one instant; the window says so."""
    rows = []
    for seq in range(samples):
        if stop is not None and stop.is_set():
            break
        row = dict(seq=seq, t_start=clock(), clock="host time.time()", registers={}, errors={})
        for key, reg, role in REGISTERS:
            try:
                got = bus.sync_read(reg, normalize=False)
                row["registers"][key] = dict(register=reg, role=role, values={n: float(got[n]) for n in JOINTS})
            except f5.ReadOnlyViolation:
                raise
            except Exception as e:
                row["errors"][key] = f"{type(e).__name__}: {e}"
                row["registers"][key] = dict(register=reg, role=role, status="UNAVAILABLE")
        row["t_end"] = clock()
        rows.append(row)
        wait = period_s - (row["t_end"] - row["t_start"])
        if wait > 0:
            sleep(wait)
    return rows


def feedback_document(source, rows_by_side, calibration, extra=None):
    doc = dict(schema="tjj.feedback.v2", source=source, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), calibration_at_capture=calibration, arms={}, commands_sent=dict(motor=0, torque_enable=0, base_cmd_vel=0), **(extra or {}))
    for side, rows in rows_by_side.items():
        if not rows:
            doc["arms"][side] = dict(status="NO DATA")
            continue
        t = [r["t_start"] for r in rows]
        avail = {key: sum("values" in r["registers"].get(key, {}) for r in rows) for key, _, _ in REGISTERS}
        goal = [r["registers"]["goal_raw"]["values"]["gripper"] for r in rows if "values" in r["registers"].get("goal_raw", {})]
        pres = [r["registers"]["present_raw"]["values"]["gripper"] for r in rows if "values" in r["registers"].get("present_raw", {})]
        doc["arms"][side] = dict(status="READ" if avail["present_raw"] else "NO DATA", samples=len(rows), rate_hz=f5.rate(t), read_window_s_median=round(float(np.median([r["t_end"] - r["t_start"] for r in rows])), 5),
                                 availability={k: ("READ" if v == len(rows) else "UNAVAILABLE" if v == 0 else f"PARTIAL {v}/{len(rows)}") for k, v in avail.items()},
                                 command_and_measured_are_separate_channels=bool(goal and pres), command_echo=None if not (goal and pres) or len(goal) != len(pres) else bool(len(goal) >= 5 and np.ptp(goal) > 0 and np.array_equal(goal, pres)),
                                 errors=sum(len(r["errors"]) for r in rows), rows=rows)
    return doc


def arms(contract, out, *, samples=50, period_s=0.02, open_bus=None):
    """Bus feedback of both followers as a time series (read-only wrapper). `open_bus(side, port)` is injectable for tests."""
    cal = load_calibration(contract)
    rows, metadata = {}, {}
    for side in ("left", "right"):
        port = contract["lerobot"].get(f"{side}_port")
        if not port or (open_bus is None and not Path(port).exists()):
            rows[side] = []
            metadata[side] = dict(status="NOT_CONNECTED" if port else "MISSING", port=port)
            continue
        if open_bus is None:
            from lerobot.motors import Motor, MotorNormMode
            from lerobot.motors.feetech import FeetechMotorsBus
            motors = {n: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES) for i, n in enumerate(JOINTS)}
            bus = f5.ReadOnlyBus(FeetechMotorsBus(port=port, motors=motors))
            bus.connect()
        else:
            bus = f5.ReadOnlyBus(open_bus(side, port))
        try:
            metadata[side] = read_metadata(bus)
            rows[side] = read_feedback_series(bus, samples=samples, period_s=period_s)
        finally:
            bus.disconnect()
    doc = feedback_document("lerobot Feetech bus through the read-only wrapper", rows, cal, dict(device_metadata=metadata))
    Path(out).mkdir(parents=True, exist_ok=True)
    (Path(out) / "feedback.json").write_text(json.dumps(doc, indent=1, default=str))
    return doc


def feedback_from_jointstate(sample_meta):
    """JointState rows of a capture in the feedback schema: measured position in rad as published; no command channel."""
    rows = {}
    for side in ("left", "right"):
        j = (sample_meta.get("joints") or {}).get(side) or {}
        rows[side] = [dict(seq=i, t_start=r["stamp_s"], t_end=r["stamp_s"], clock="ROS header stamp", errors={}, registers=dict(
            present_rad=dict(register="JointState.position", role="measured (as the driver publishes it; whether it is measured or an echo must be confirmed with the driver)", values=dict(zip(r["names"], r["position"]))),
            goal_raw=dict(register="(none in JointState)", role="command", status="UNAVAILABLE"))) for i, r in enumerate(j.get("rows") or [])]
    doc = dict(schema="tjj.feedback.v2", source="ROS JointState topics", arms={})
    for side, rs in rows.items():
        doc["arms"][side] = dict(status="READ" if rs else "NO DATA", samples=len(rs), rate_hz=f5.rate([r["t_start"] for r in rs]), availability=dict(present_rad="READ" if rs else "NO DATA", goal_raw="UNAVAILABLE"),
                                 command_and_measured_are_separate_channels=False, rows=rs)
    return doc


# ----------------------------------------------------------------------------------------------
# stage 2: driver units -> model q12 (sim alignment), from the feedback sample nearest to the reference time
# ----------------------------------------------------------------------------------------------

def q12_at(feedback, contract, calibration, t_ref):
    q, rec, verified, valid = [], {}, True, True
    for side in ("left", "right"):
        arm = (feedback.get("arms") or {}).get(side) or {}
        rows = [r for r in arm.get("rows") or [] if "values" in r["registers"].get("present_raw", {}) or "values" in r["registers"].get("present_rad", {})]
        if not rows:
            rec[side] = dict(status="MISSING", reason=arm.get("status", "no feedback rows"))
            valid = False
            continue
        r = min(rows, key=lambda x: abs((x["t_start"] + x["t_end"]) / 2 - t_ref))
        mp = contract["q12"][side]
        row = dict(seq=r["seq"], read_window=[r["t_start"], r["t_end"]], clock=r["clock"], skew_s=round((r["t_start"] + r["t_end"]) / 2 - t_ref, 4), source=feedback.get("source"))
        if "values" in r["registers"].get("present_raw", {}):
            cal = calibration.get(side) or {}
            if cal.get("status") != "READ":
                rec[side] = dict(row, status="MISSING", reason=f"calibration file {cal.get('status')}: raw ticks are not converted without it")
                valid = False
                continue
            drv = driver_units(r["registers"]["present_raw"]["values"], cal)
            row.update(raw_ticks=r["registers"]["present_raw"]["values"], driver=drv, goal_raw_ticks=r["registers"].get("goal_raw", {}).get("values"),
                       calibration=dict(used=cal.get("source"), file_at_capture=cal.get("file"), file_sha256=cal.get("sha256"), content_sha256=cal.get("content_sha256"), override=cal.get("override")))
            if not all(d["valid"] for d in drv.values()):
                rec[side] = dict(row, status="INVALID", reason="missing or non-finite tick")
                valid = False
                continue
            arm_rad = [math.radians(drv[n]["value"]) for n in JOINTS[:5]]
            grip_percent = drv["gripper"]["value"]
            row["out_of_calibrated_range"] = [n for n in JOINTS if not drv[n]["in_calibrated_range"]]
        else:
            vals = r["registers"]["present_rad"]["values"]
            names = contract["joints"][side].get("names") or list(JOINTS)
            if len(set(names)) != 6 or any(n not in vals for n in names) or not all(math.isfinite(vals[n]) for n in names):
                rec[side] = dict(row, status="INVALID", reason=f"joint names {names} not unique / not all present and finite in {sorted(vals)}")
                valid = False
                continue
            arm_rad, grip_percent = [vals[n] for n in names[:5]], None
            row.update(published=dict(zip(names, [vals[n] for n in names])), note="JointState gripper value: its unit is the driver's; give closed / open in the same unit")
            grip_percent = vals[names[5]]
        g = mp["gripper"]
        gc, go = g.get("driver_percent_closed"), g.get("driver_percent_open")
        if any(j["sign"] is None or j["offset_rad"] is None for j in mp["joints"]) or gc is None or go is None:
            rec[side] = dict(row, status="UNVERIFIED", reason="the sim alignment of the contract (sign / offset_rad / gripper closed-open) is empty: driver units are kept, no model q12 is made from guessed values",
                             driver_arm_rad=arm_rad, driver_gripper=grip_percent, empty=[j["name"] for j in mp["joints"] if j["sign"] is None or j["offset_rad"] is None] + (["gripper closed / open"] if gc is None or go is None else []))
            valid = False
            continue
        if go == gc or any(j["sign"] not in (1, -1, 1.0, -1.0) or not math.isfinite(j["offset_rad"]) for j in mp["joints"]):
            rec[side] = dict(row, status="INVALID", reason="sim alignment: gripper open equals closed, a sign is not +-1, or an offset is not finite")
            valid = False
            continue
        model = [j["sign"] * a + j["offset_rad"] for j, a in zip(mp["joints"], arm_rad)]
        opening = (grip_percent - gc) / (go - gc)                             # NOT clipped: an opening outside [0, 1] is a finding
        v = all(j["verified"] for j in mp["joints"]) and bool(g["verified"])
        if "present_raw" in r["registers"] and "values" in r["registers"]["present_raw"] and (calibration.get(side) or {}).get("override"):
            v = False                                                         # the alignment was verified with the capture calibration, not with a replacement
            row["verification_note"] = "NOT verified: converted with a calibration that replaces the capture snapshot"
        verified &= v
        flags = (["gripper opening outside [0, 1]"] if not 0.0 <= opening <= 1.0 else []) + [f"{n} outside its calibrated range" for n in row.get("out_of_calibrated_range", [])]
        if not all(math.isfinite(x) for x in model + [opening]):
            rec[side] = dict(row, status="INVALID", reason="non-finite model value")
            valid = False
            continue
        rec[side] = dict(row, status="MAPPED" if v else "MAPPED, alignment present but not marked verified", model=model + [opening], flags=flags)
        q += model + [opening]
    ok = valid and len(q) == 12
    return (np.array(q, dtype=np.float32) if ok else None), dict(rec, valid=bool(ok), verified=bool(ok and verified))


# ----------------------------------------------------------------------------------------------
# capture: topics and devices in the same time window
# ----------------------------------------------------------------------------------------------

def device_reader(device, frames, stop, clock=time.time, open_camera=None, settings=None, info=None):
    """Read one V4L2 device until `stop`. The stamp is the host time when the read returned (not a capture time).
    settings (from the contract's stream entry `v4l2`): fourcc / width / height requested from the UVC device, and
    raw=true to keep the frame bytes as delivered (no colour conversion) for streams whose format is not an image."""
    import cv2
    cap = open_camera(device) if open_camera else cv2.VideoCapture(device, cv2.CAP_V4L2)
    info = {} if info is None else info
    info.update(device=device, opened=bool(cap.isOpened()), requested=settings or {})
    if info["opened"] and settings and not open_camera:
        if settings.get("fourcc"):
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*settings["fourcc"]))
        if settings.get("width"):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings["width"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings["height"])
        if settings.get("raw"):
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    if info["opened"] and not open_camera:
        f = int(cap.get(cv2.CAP_PROP_FOURCC))
        info.update(fourcc="".join(chr((f >> 8 * i) & 0xFF) for i in range(4)), width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), driver_fps=cap.get(cv2.CAP_PROP_FPS))
    while info["opened"] and not stop.is_set():
        t0 = clock()
        ok, img = cap.read()
        if ok:
            frames.append(dict(data=img, stamp_s=clock(), read_started_s=t0))
    cap.release()
    return info


def capture(contract, out, *, duration_s=3.0, samples=30, with_arms=False, open_camera=None, open_bus=None, ros=True, warmup_limit_s=6.0):
    """ROS topics (R5's locked read-only node), the V4L2 wrist devices and (optionally) the bus feedback, all during the
    same window. Returns the sample folder."""
    out = Path(out)
    stop, dev_frames, threads, fb, bus_errors, bus_meta, dev_info = threading.Event(), {}, [], {}, {}, {}, {}
    for n, s in contract["streams"].items():
        if s.get("device") and not s.get("topic") and not n.endswith("_info"):
            dev_frames[n], dev_info[n] = [], {}
            th = threading.Thread(target=device_reader, args=(s["device"], dev_frames[n], stop), kwargs=dict(open_camera=open_camera, settings=s.get("v4l2"), info=dev_info[n]), daemon=True)
            threads.append(th)
    cal = load_calibration(contract)
    n_cam = len(threads)
    if with_arms:
        def bus_thread(side, port):
            try:
                bus_reader(side, port)
            except Exception as e:                                            # a bus that cannot be read is reported in the sample, not silently skipped
                bus_errors[side] = f"{type(e).__name__}: {e}"

        def bus_reader(side, port):
            if open_bus is None:
                from lerobot.motors import Motor, MotorNormMode
                from lerobot.motors.feetech import FeetechMotorsBus
                motors = {n: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES) for i, n in enumerate(JOINTS)}
                bus = f5.ReadOnlyBus(FeetechMotorsBus(port=port, motors=motors))
                bus.connect()
            else:
                bus = f5.ReadOnlyBus(open_bus(side, port))
            try:
                bus_meta[side] = read_metadata(bus)
                fb[side] = read_feedback_series(bus, samples=10 ** 6, period_s=0.02, stop=stop)
            finally:
                bus.disconnect()
        for side in ("left", "right"):
            port = contract["lerobot"].get(f"{side}_port")
            if port and open_bus is None and not Path(port).exists():
                bus_errors[side] = f"NOT_CONNECTED: {port} does not exist"
                continue
            if port:
                threads.append(threading.Thread(target=bus_thread, args=(side, contract["lerobot"][f"{side}_port"]), daemon=True))
    # READ02: the cameras start first; the common window opens when every device that opened has delivered a frame
    # (some need ~2 s for the first one), so a slow first frame does not empty the capture. Frames read before the
    # window are dropped; a device that never delivers is reported after `warmup_limit_s`, not waited for again.
    t_warm = time.time()
    for th in threads[:n_cam]:
        th.start()
    while time.time() - t_warm < warmup_limit_s and any(not dev_frames[n] and dev_info.get(n, {}).get("opened", True) for n in dev_frames):
        time.sleep(0.02)
    warmup = dict(waited_s=round(time.time() - t_warm, 3), limit_s=warmup_limit_s, first_frame_seen={n: bool(dev_frames[n]) for n in dev_frames})
    for n in dev_frames:
        del dev_frames[n][:-1]
    t_begin = time.time()
    for th in threads[n_cam:]:
        th.start()
    ros_contract = json.loads(json.dumps(contract))
    for n in dev_frames:
        ros_contract["streams"][n]["device"] = None                          # the R5 capture would read devices AFTER the topics; here they are read in parallel
    if ros:
        sample, facts, meta = f5.capture_ros(ros_contract, out, samples=samples, timeout_s=duration_s)
    else:
        time.sleep(duration_s)
        sample, facts, meta = f5.save_sample(out, {}, {}, dict(schema="tjj.field-sample.v1", source="devices / bus only", joints={}, tf=dict(status="MISSING", reason="no ROS"), contract=contract)), {}, json.loads((out / "sample.json").read_text())
        facts = meta.get("streams", {})
    stop.set()
    for th in threads:
        th.join(timeout=5.0)
    t_end = time.time()
    raw = dict(np.load(sample / "raw.npz")) if (sample / "raw.npz").exists() else {}
    meta = json.loads((sample / "sample.json").read_text())
    for n, rows in dev_frames.items():
        if not rows:
            exists = Path(contract["streams"][n]["device"]).exists()
            meta["streams"][n] = dict(status="NO DATA" if exists else "NOT_CONNECTED", device=contract["streams"][n]["device"], reason="no frame read from the device in the window" if exists else "the device node does not exist",
                                      v4l2=dev_info.get(n))
            continue
        keep = rows[-samples:]
        img = keep[0]["data"]
        if (contract["streams"][n].get("v4l2") or {}).get("raw"):                # bytes as delivered: kept, not interpreted
            raw[n + "_raw_uvc"], raw[n + "_raw_uvc_stamp"] = np.stack([np.asarray(r["data"]) for r in keep]), np.array([r["stamp_s"] for r in keep])
            meta["streams"][n] = dict(status="READ_RAW_UNVERIFIED", device=contract["streams"][n]["device"], frames=len(keep), rate_hz=f5.rate([r["stamp_s"] for r in rows]), v4l2=dev_info.get(n), array_shape=list(np.asarray(img).shape),
                                      dtype=str(np.asarray(img).dtype), stamp_source="HOST time when the read returned; NOT a capture time",
                                      note="frame bytes of the UVC stream as delivered (no colour conversion). The pixel format / unit of this stream is not decoded here: it is NOT used as depth")
            continue
        raw[n], raw[n + "_stamp"] = np.stack([r["data"] for r in keep]), np.array([r["stamp_s"] for r in keep])
        meta["streams"][n] = dict(f5.colour_facts(img, dict(encoding="bgr8", dtype=str(img.dtype), channels=img.shape[2], width=img.shape[1], height=img.shape[0])), status="READ", device=contract["streams"][n]["device"],
                                  frames=len(keep), rate_hz=f5.rate([r["stamp_s"] for r in rows]), v4l2=dev_info.get(n), stamp_source="HOST time when the read returned; NOT a capture time (latency of the device and driver not measured)",
                                  read_duration_s_median=round(float(np.median([r["stamp_s"] - r["read_started_s"] for r in rows])), 4))
    for n, s in meta["streams"].items():
        if s.get("status") == "READ" and "stamp_source" not in s and not n.endswith("_info"):
            s["stamp_source"] = "ROS header stamp of the message (the driver's time; its relation to the exposure is the driver's)"
    meta.update(schema="tjj.field-sample.v2", capture_window_host_s=[t_begin, t_end], warmup=warmup, contract=contract, calibration_at_capture=cal, concurrent=dict(devices=sorted(dev_frames), bus=sorted(fb), bus_errors=bus_errors, ros_topics=bool(ros)),
                arrays=sorted(raw))
    np.savez_compressed(sample / "raw.npz", **raw)
    (sample / "sample.json").write_text(json.dumps(meta, indent=2, default=str))
    if fb:
        (sample / "feedback.json").write_text(json.dumps(feedback_document("lerobot Feetech bus through the read-only wrapper, during the camera capture window", fb, cal, dict(capture_window_host_s=[t_begin, t_end], device_metadata=bus_meta)), indent=1, default=str))
    return sample


# ----------------------------------------------------------------------------------------------
# model input: pairing by time, separate readiness levels
# ----------------------------------------------------------------------------------------------

def valid_K(K, w, h):
    if K is None:
        return False, "no K"
    K = np.asarray(K, dtype=float).reshape(-1)
    if K.size != 9 or not np.isfinite(K).all():
        return False, "K is not 9 finite numbers"
    if K[0] <= 0 or K[4] <= 0:
        return False, f"focal length {K[0]}, {K[4]} is not positive"
    if not (0 <= K[2] <= w and 0 <= K[5] <= h):
        return False, f"principal point ({K[2]}, {K[5]}) outside the {w} x {h} image"
    return True, ""


INTEGRITY_MAX_NEIGHBOUR_DIFF = 2.5      # grey levels at 160 x 120. Stored streams of 2026-10-07: 327 intact stationary frames <= 1.64; corrupted frames 3 .. 63


def stream_integrity(frames):
    """Usable mask of a STATIONARY camera stream: a frame must agree with its previous or its next frame (mean absolute
    grey difference at 160 x 120). A transport-corrupted frame differs from both. Only meaningful while the camera does
    not move; the caller decides that from the joint feedback. Returns (mask, per-frame difference)."""
    import cv2
    if len(frames) < 3:
        return np.zeros(len(frames), dtype=bool), np.full(len(frames), np.nan)
    g = np.array([cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (160, 120), interpolation=cv2.INTER_AREA) for f in frames], dtype=np.float32)
    d = np.abs(np.diff(g, axis=0)).mean(axis=(1, 2))
    m = np.minimum(np.r_[np.inf, d], np.r_[d, np.inf])
    return m <= INTEGRITY_MAX_NEIGHBOUR_DIFF, m


def arms_stationary(feedback):
    """True if every arm with feedback kept every joint within 1 tick over the capture; None without feedback."""
    seen = False
    for arm in (feedback.get("arms") or {}).values():
        rows = [r["registers"]["present_raw"]["values"] for r in (arm.get("rows") or []) if "values" in r["registers"].get("present_raw", {})]
        if rows:
            seen = True
            if max(max(r[n] for r in rows) - min(r[n] for r in rows) for n in JOINTS) > 1:
                return False
    return True if seen else None


def model_input(sample_dir, calibration_override=None, reason=None):
    """calibration_override: a directory with replacement calibration files. It never changes the default artifact: the
    result is written as a separate derived file with the previous / new versions and the reason, and it is not verified."""
    sample_dir = Path(sample_dir)
    meta = json.loads((sample_dir / "sample.json").read_text())
    raw = np.load(sample_dir / "raw.npz")
    contract, s = meta["contract"], meta["streams"]
    fb_file = sample_dir / "feedback.json"
    feedback = json.loads(fb_file.read_text()) if fb_file.exists() else feedback_from_jointstate(meta)
    # R6.1: the conversion uses the calibration bound to THIS sample. A file of the same name on the machine that runs
    # the analysis is only compared, never used in its place.
    calibration = calibration_snapshot(meta, feedback)
    here = load_calibration(contract)
    comparison = {}
    for side in ("left", "right"):
        snap, loc = calibration[side], here.get(side) or {}
        comparison[side] = dict(snapshot_status=snap["status"], snapshot_content_sha256=snap.get("content_sha256"), file_on_this_machine=loc.get("status"), file_on_this_machine_content_sha256=loc.get("content_sha256"),
                                same_content=None if snap["status"] != "READ" or loc.get("status") != "READ" else snap["content_sha256"] == loc["content_sha256"], used="snapshot")
    override = None
    if calibration_override is not None:
        if not reason:
            raise ValueError("a calibration override needs a reason")
        c2 = json.loads(json.dumps(contract))
        c2["lerobot"]["calibration_dir"] = str(calibration_override)
        new = load_calibration(c2)
        override = dict(reason=reason, directory=str(calibration_override), previous={k: calibration[k].get("content_sha256") for k in calibration}, new={k: new[k].get("content_sha256") for k in new})
        for side in ("left", "right"):
            if new[side].get("status") == "READ":
                calibration[side] = dict(new[side], source="OVERRIDE calibration (not the capture snapshot)", override=True)
                comparison[side]["used"] = "OVERRIDE"
    names = dict(left="left_wrist", right="right_wrist", top="top_color")
    present = {k: n in raw.files and len(raw[n]) > 0 for k, n in names.items()}
    notes, arrays, views = [], {}, {}
    rate_of = lambda n: s.get(n, {}).get("rate_hz")
    lim = contract.get("timing", {}).get("max_skew_s", {})

    def limit(kind, a, b):
        if lim.get(kind) is not None:
            return float(lim[kind]), "contract timing.max_skew_s." + kind
        rates = [r for r in (rate_of(a), b) if r]
        return (None, "no measured rate: no limit can be derived") if not rates else (1.0 / min(rates), "one period of the slower stream (measured rate)")

    # ---- pairing: the top frame whose nearest wrist frames and feedback sample are closest in time ----
    pairing = dict(reference=None, method="nearest stamps to each top frame; stamps are what each source gives (see stamp_source); never the same array index")
    fb_t = {side: [((r["t_start"] + r["t_end"]) / 2) for r in (feedback["arms"].get(side) or {}).get("rows") or []] for side in ("left", "right")}
    # Wrist frames that arrived corrupted are not candidates. The check needs a stationary camera (see stream_integrity);
    # while the arms move it is NOT applied and says so. No person picks frames afterwards: if no usable frame lies within
    # the skew limit the input is not time aligned and nothing downstream runs on it.
    still = arms_stationary(feedback)
    integrity, usable = {}, {}
    for k in ("left", "right"):
        if not present[k]:
            continue
        if still and len(raw[names[k]]) >= 3:
            mask, diff = stream_integrity(raw[names[k]])
            usable[k] = mask
            integrity[k] = dict(checked=True, frames=int(len(mask)), unusable=int((~mask).sum()), unusable_indices=np.nonzero(~mask)[0].tolist()[:60], max_neighbour_difference=float(np.nanmax(diff[np.isfinite(diff)])) if np.isfinite(diff).any() else None,
                                rule=f"stationary stream: mean |grey difference| to the previous or next frame <= {INTEGRITY_MAX_NEIGHBOUR_DIFF} at 160 x 120")
        else:
            usable[k] = np.ones(len(raw[names[k]]), dtype=bool)
            integrity[k] = dict(checked=False, frames=int(len(raw[names[k]])), reason=("fewer than 3 frames in the stream: a frame cannot be compared with its neighbours" if still else "the arms were not stationary (or there is no joint feedback): the stationary-stream check does not apply") + "; frame integrity is NOT verified")
        if not usable[k].any():
            present[k] = False
            notes.append(f"{names[k]}: no usable frame (all {len(usable[k])} failed the integrity check)")
    if present["top"]:
        tt = raw["top_color_stamp"]
        best = None
        for i, t in enumerate(tt):
            sk = {}
            for k in ("left", "right"):
                if present[k]:
                    st = raw[names[k] + "_stamp"]
                    cand = np.nonzero(usable[k])[0]
                    j = int(cand[np.argmin(np.abs(st[cand] - t))])
                    sk[k] = (j, float(st[j] - t))
            for side in ("left", "right"):
                if fb_t[side]:
                    j = int(np.argmin(np.abs(np.array(fb_t[side]) - t)))
                    sk["joints_" + side] = (j, float(fb_t[side][j] - t))
            if "top_depth" in raw.files:
                st = raw["top_depth_stamp"]
                j = int(np.argmin(np.abs(st - t)))
                sk["depth"] = (j, float(st[j] - t))
            worst = max([abs(v[1]) for v in sk.values()] or [float("inf")])
            if best is None or worst < best[0]:
                best = (worst, i, sk, float(t))
        worst, ti, sk, t_ref = best
        limits = dict(wrist=limit("wrist", "top_color", min([r for r in (rate_of("left_wrist"), rate_of("right_wrist")) if r] or [None]) if any((rate_of("left_wrist"), rate_of("right_wrist"))) else None),
                      joints=limit("joints", "top_color", min([r for r in ((feedback["arms"].get(x) or {}).get("rate_hz") for x in ("left", "right")) if r] or [None]) if any((feedback["arms"].get(x) or {}).get("rate_hz") for x in ("left", "right")) else None),
                      depth=limit("depth", "top_color", rate_of("top_depth")))
        within = {}
        for k, (j, dt) in sk.items():
            kind = "wrist" if k in ("left", "right") else "joints" if k.startswith("joints") else "depth"
            L = limits[kind][0]
            within[k] = None if L is None else bool(abs(dt) <= L)
        clocks = sorted({s.get(names[k], {}).get("stamp_source", "?")[:40] for k in names if present[k]} | {(feedback.get("source") or "?")[:40]})
        pairing.update(reference="top_color", top_index=int(ti), reference_stamp_s=t_ref, skew_s={k: round(v[1], 4) for k, v in sk.items()}, index={k: v[0] for k, v in sk.items()},
                       limits_s={k: dict(limit=None if v[0] is None else round(v[0], 4), basis=v[1]) for k, v in limits.items()}, within_limit=within, stamp_sources=clocks,
                       mixed_clocks=len(clocks) > 1, note="host-read stamps and ROS header stamps are different clocks / different points of the pipeline: a small skew between them is not proof of simultaneous exposure")
    else:
        notes.append("top_color: " + s.get("top_color", {}).get("status", "MISSING"))
        t_ref, sk, within = None, {}, {}
    # ---- derived views -------------------------------------------------------------------------------
    for k in ("left", "right", "top"):
        if not present[k]:
            notes.append(f"{names[k]}: {s.get(names[k], {}).get('status', 'MISSING')}")
            continue
        idx = pairing["top_index"] if k == "top" else (sk[k][0] if k in sk else len(raw[names[k]]) - 1)     # no reference stream: the last frame, and the input is not time aligned
        rgb = f5.to_rgb(raw[names[k]][idx], s[names[k]]["encoding"])
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb.astype(np.float32) / 256.0, 0, 255).astype(np.uint8)
        K = (s.get("top_info") or {}).get("K") if k == "top" else (s.get(names[k] + "_info") or {}).get("K")
        img, K2, how = f5.fit(rgb, K)
        if k == "top":
            luma = np.round(0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]).astype(np.uint8)
            img = np.repeat(luma[..., None], 3, axis=2)
            how["derived"] = "gray3 for the policy (trained on a gray3 SIM top view); the raw stream is kept as sent"
            how["sensor_kind_from_pixels"] = s[names[k]].get("kind")
            how["sensor_kind_note"] = "equal channels suggest mono / IR but do not prove what the sensor is; confirm with the camera's documentation / driver settings"
        arrays[k] = img
        views[k] = dict(how, stamp_s=float(raw[names[k] + "_stamp"][idx]), index=int(idx), stamp_source=s[names[k]].get("stamp_source"), K_scaled=None if K2 is None else K2.round(4).tolist())
        if k == "top" and K2 is not None:
            arrays["top_K"] = K2
    # ---- geometry: K, T, depth registration ---------------------------------------------------------
    geometry = {}
    info = s.get("top_info") or {}
    okK, whyK = valid_K(info.get("K"), info.get("width") or 0, info.get("height") or 0) if info.get("status") == "READ" else (False, "top CameraInfo " + info.get("status", "MISSING"))
    geometry["top_K"] = dict(valid=okK, reason=whyK, distortion_model=info.get("distortion_model"), D=info.get("D"), note="the image is NOT undistorted here; with a non-zero D the scaled K is an approximation")
    tf = meta.get("tf", {})
    geometry["camera_to_base"] = dict(valid=tf.get("status") == "READ", status=tf.get("status", "MISSING"), reason=tf.get("reason"))
    mount = contract.get("mount")
    if tf.get("status") != "READ" and isinstance(mount, dict):
        # READ03: a mount snapshot (board pose in the camera + arm base on the board) is taken as the same kind of
        # metadata as a TF. p_camera = T_camera_from_board @ p_board  =>  T_base_from_camera = T_base_from_board @ inv(T_camera_from_board).
        cb, per = mount.get("T_camera_from_board") or {}, {}
        for side in ("left", "right"):
            bb = (mount.get("T_base_from_board") or {}).get(side) or {}
            if cb.get("value") is None or bb.get("value") is None:
                per[side] = dict(status="MISSING", reason="no board pose" if cb.get("value") is None else "no arm-base-on-board record for this arm")
                continue
            T = np.array(bb["value"], dtype=float) @ np.linalg.inv(np.array(cb["value"], dtype=float))
            per[side] = dict(status="VERIFIED" if (cb.get("verified") is True and bb.get("verified") is True) else "CANDIDATE_UNVERIFIED", T_base_from_camera=T.round(6).tolist(), camera_from_board_source=cb.get("source"), base_from_board_source=bb.get("source"))
        geometry["camera_to_base"] = dict(valid=all(v["status"] == "VERIFIED" for v in per.values()), status="MOUNT_SNAPSHOT", arms=per,
                                          reason=None if all(v["status"] == "VERIFIED" for v in per.values()) else "mount snapshot incomplete or unverified: " + ", ".join(f"{k} {v['status']}" for k, v in per.items()))
    dep, dc = dict(valid=False), contract.get("depth", {})
    if "top_depth" not in raw.files:
        dep["reason"] = "no depth stream: " + s.get("top_depth", {}).get("status", "MISSING")
    elif dc.get("registered_to_color") is True and okK:
        d = raw["top_depth"][sk["depth"][0]].astype(np.float32)
        d = d / 1000.0 if s["top_depth"].get("unit", "").startswith("millimetres") else d
        ratio = dc.get("pixel_ratio")
        ch, cw = raw["top_color"].shape[1:3]
        if d.shape[:2] != (ch, cw) and isinstance(ratio, int) and ratio > 1 and (d.shape[0] * ratio, d.shape[1] * ratio) == (ch, cw):
            # READ03: registered depth at 1 / ratio of the image size (declared by the adapter). Its K is the image K
            # divided by the ratio; it gets the same centre crop and resize as the image, computed on its own grid.
            Kd = np.array(info["K"], dtype=float).reshape(3, 3).copy()
            Kd[:2] /= ratio
            dfit, Kd2, howd = f5.fit(d, Kd, nearest=True)
            arrays["top_depth_m"], arrays["top_depth_valid"], arrays["top_depth_K"] = dfit, dfit > 0, Kd2
            dK = float(np.abs(Kd2 - arrays["top_K"]).max()) if "top_K" in arrays else None
            dep.update(valid=True, basis=f"adapter: depth registered to the image at 1/{ratio} size; depth K = image K / {ratio}; mm -> m once; same centre crop / resize on the depth grid (nearest)", unit_in=s["top_depth"].get("unit"), unit_out="metres (0 = invalid)",
                       fit=howd, K_scaled=Kd2.round(4).tolist(), max_abs_difference_to_image_K_scaled=dK, valid_fraction_model_view=float((dfit > 0).mean()), same_frameset=bool(sk["depth"][0] == pairing["top_index"]),
                       depth_minus_image_stamp_ms=round(1000 * sk["depth"][1], 3))
        elif d.shape[:2] != (ch, cw):
            dep["reason"] = f"declared registered to colour but the depth size {d.shape[:2]} differs from the colour size {raw['top_color'].shape[1:3]}: the colour K cannot be applied as it is"
        else:
            arrays["top_depth_m"], _, _ = f5.fit(d, None, nearest=True)
            dep.update(valid=True, basis="contract: the driver registers depth to colour; the colour K applies; same crop and scale as the colour image")
    elif dc.get("K") is not None and dc.get("T_color_from_depth") is not None:
        okD, whyD = valid_K(dc["K"], raw["top_depth"].shape[2], raw["top_depth"].shape[1])
        dep.update(valid=False, reason="depth K and extrinsic are given (" + ("valid K" if okD else whyD) + "): reprojection into the colour frame is not implemented in this tool; the raw depth and its own K are kept for a later step")
    else:
        dep["reason"] = ("the contract does not say that the depth is registered to colour and gives no depth K / extrinsic: the depth is kept raw and is not resized or projected with the colour K "
                         "(equal image sizes would not prove registration)")
    geometry["depth"] = dep
    q, qrec = q12_at(feedback, contract, calibration, t_ref) if t_ref is not None else (None, dict(valid=False, verified=False))
    if q is not None:
        arrays["q12"] = q
    time_ok = bool(within) and all(v is True for v in within.values()) and all(k in within for k in ("left", "right", "joints_left", "joints_right"))
    windows = [r["t_end"] - r["t_start"] for a_ in (feedback.get("arms") or {}).values() for r in (a_.get("rows") or [])]
    exposure_reasons = ([] if not pairing.get("mixed_clocks") else ["the streams are stamped by different clocks / at different points of their pipelines"]) + \
        [f"{k}: {v.get('stamp_source')}" for k, v in views.items() if "HOST" in str(v.get("stamp_source"))] + \
        ([f"joint registers are read one after the other inside a window of {1000 * float(np.median(windows)):.1f} ms (median)"] if windows and float(np.median(windows)) > 0 else [])
    exposure = dict(verified=False if (exposure_reasons or not time_ok) else None, reasons=exposure_reasons,
                    note="stamp agreement (time_aligned) is not exposure simultaneity. With the reasons listed the true capture instants are not known; None = nothing argues against it, still not measured")
    readiness = dict(
        arrays_present=dict(ok=all(present.values()), detail=present),
        policy_tensor_possible=dict(ok=bool(all(present.values()) and q is not None and np.isfinite(q).all()), detail="three views and a finite 12-vector" if q is not None else "no valid q12 (see q12)"),
        q_mapping_verified=dict(ok=bool(qrec.get("verified")), detail={k: v.get("status") for k, v in qrec.items() if isinstance(v, dict)}),
        time_aligned=dict(ok=time_ok, meaning="the stamps each source gives agree within the limits", detail=dict(skew_s=pairing.get("skew_s"), within_limit=within, mixed_clocks=pairing.get("mixed_clocks")),
                          exposure_simultaneity=exposure),
        geometry_valid=dict(ok=bool(geometry["top_K"]["valid"] and geometry["camera_to_base"]["valid"] and geometry["depth"]["valid"]), detail={k: (v["valid"], v.get("reason")) for k, v in geometry.items()}))
    ready_for = dict(shape_only_shadow=readiness["policy_tensor_possible"]["ok"],
                     policy_input_shadow=bool(readiness["policy_tensor_possible"]["ok"] and readiness["q_mapping_verified"]["ok"] and readiness["time_aligned"]["ok"]),
                     manipulation=False, manipulation_note="never set by this tool: it needs the geometry, the timing, the verified q map AND an on-site approval; this tool is read-only")
    out = dict(schema="tjj.model-input.v2", sample=str(sample_dir), readiness=readiness, ready_for=ready_for, pairing=pairing, views=views, geometry=geometry, q12=qrec, notes=notes,
               wrist_frame_integrity=integrity, arms_stationary=still,
               feedback=dict(source=feedback.get("source"), arms={k: {x: v.get(x) for x in ("status", "samples", "rate_hz", "availability", "command_and_measured_are_separate_channels", "command_echo")} for k, v in feedback.get("arms", {}).items()}),
               calibration_applied={k: {x: v.get(x) for x in ("status", "source", "file", "sha256", "content_sha256", "reason")} for k, v in calibration.items()}, calibration_comparison=comparison, calibration_override=override,
               order=contract["model"]["order"], colour_order="RGB", image_size=[MODEL_W, MODEL_H],
               is_fixture=bool(meta.get("fixture")))
    tag = "model_input_r6" if override is None else "model_input_r6_override_" + hashlib.sha256(json.dumps(override["new"], sort_keys=True).encode()).hexdigest()[:8]
    out["artifact"] = tag
    np.savez_compressed(sample_dir / f"{tag}.npz", **arrays)
    (sample_dir / f"{tag}.json").write_text(json.dumps(out, indent=2, default=str))
    return out, arrays


def shadow(sample_dir, *, shape_only=False):
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    info, arrays = model_input(sample_dir)
    sample_dir = Path(sample_dir)
    level = "policy_input_shadow" if info["ready_for"]["policy_input_shadow"] else ("shape_only_shadow" if shape_only and info["ready_for"]["shape_only_shadow"] else None)
    out = dict(schema="tjj.shadow.v2", sample=str(sample_dir), commands_sent=dict(motor=0, torque_enable=0, base_cmd_vel=0), readiness={k: v["ok"] for k, v in info["readiness"].items()}, level=level,
               meaning="input compatibility only: not a pick result, not a readiness for manipulation, not a safety approval")
    if level is None:
        out.update(status="NOT RUN", reason="the input is not ready for a policy-input shadow (q map verified + time aligned + tensors)" + ("" if shape_only else "; --shape-only runs shapes and devices if the tensors exist"))
    else:
        import torch
        if not torch.cuda.is_available():
            out.update(status="NOT RUN", reason="CUDA is not available on this machine: no CPU fallback")
        else:
            import receive_candidate as rc
            import tjj_alignment_r1 as al
            import tjj_mvp_r2 as r2m
            from dapier_act_policy import MEASURED_Q_KEY
            from sim_data_factory.r2_data import policy_observation
            m, q, rows = r2m.models(), arrays["q12"].astype(np.float32), {}
            for role, name in (("START", "START_3V"), ("RECEIVE", "RECEIVE_3V")):
                rc.MODELS[role] = Path(m[name]["path"])
                policy, stats, meta = rc.load_policy(role)
                policy.reset()
                policy.eval()
                batch = {k: v[None] for k, v in policy_observation(q, dict(left=arrays["left"], right=arrays["right"]), stats, device="cuda").items()}
                batch[al.TOP_KEY] = al.image_tensor(arrays["top"], device="cuda")[None]
                batch[MEASURED_Q_KEY] = torch.tensor(q, device="cuda")[None]
                t0 = time.perf_counter()
                with torch.inference_mode():
                    chunk = policy.predict_canonical_chunk(batch)
                torch.cuda.synchronize()
                c = chunk[0].cpu().numpy()
                rows[name] = dict(model_sha256=meta["model_sha256"], parameter_devices=sorted({str(p.device) for p in policy.parameters()}), input_devices=sorted({str(v.device) for v in batch.values()}), output_device=str(chunk.device),
                                  chunk_shape=list(c.shape), finite=bool(np.isfinite(c).all()), forward_s=round(time.perf_counter() - t0, 4), first_action=c[0].round(4).tolist())
                np.save(sample_dir / f"shadow_r6_chunk_{name}.npy", c)
            out.update(status="RAN (" + ("policy input" if level == "policy_input_shadow" else "SHAPE ONLY: q map unverified and / or time not aligned") + ")", device=torch.cuda.get_device_name(0), policies=rows)
    (sample_dir / "shadow_r6.json").write_text(json.dumps(out, indent=2, default=str))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)
    c = sub.add_parser("contract")
    c.add_argument("--out")
    x = sub.add_parser("capture")
    x.add_argument("--contract", required=True)
    x.add_argument("--out", required=True)
    x.add_argument("--duration", type=float, default=3.0)
    x.add_argument("--samples", type=int, default=30)
    x.add_argument("--with-arms", action="store_true", help="also read the bus feedback during the window (only if no other process owns the ports)")
    x.add_argument("--no-ros", action="store_true")
    x = sub.add_parser("arms")
    x.add_argument("--contract", required=True)
    x.add_argument("--out", required=True)
    x.add_argument("--samples", type=int, default=50)
    x.add_argument("--period", type=float, default=0.02)
    for name in ("model-input", "shadow"):
        x = sub.add_parser(name)
        x.add_argument("--sample", required=True)
        if name == "shadow":
            x.add_argument("--shape-only", action="store_true")
        else:
            x.add_argument("--calibration-override", help="directory with replacement calibration files: writes a SEPARATE derived artifact, never verified")
            x.add_argument("--reason", help="why the capture calibration is replaced (required with --calibration-override)")
    a = p.parse_args(argv)
    if a.command == "contract":
        text = json.dumps(template(), indent=2)
        (Path(a.out).parent.mkdir(parents=True, exist_ok=True), Path(a.out).write_text(text)) if a.out else print(text)
    elif a.command == "capture":
        sample = capture(json.loads(Path(a.contract).read_text()), a.out, duration_s=a.duration, samples=a.samples, with_arms=a.with_arms, ros=not a.no_ros)
        m = json.loads((sample / "sample.json").read_text())
        print(json.dumps(dict(sample=str(sample), streams={k: (v.get("status"), v.get("kind") or v.get("unit"), v.get("rate_hz"), (v.get("stamp_source") or "")[:30]) for k, v in m["streams"].items()}, concurrent=m["concurrent"],
                              feedback=(sample / "feedback.json").exists(), commands_sent=m.get("commands_sent")), indent=1, default=str))
    elif a.command == "arms":
        d = arms(json.loads(Path(a.contract).read_text()), a.out, samples=a.samples, period_s=a.period)
        print(json.dumps({k: {x: v.get(x) for x in ("status", "samples", "rate_hz", "read_window_s_median", "availability", "command_and_measured_are_separate_channels", "command_echo", "errors")} for k, v in d["arms"].items()}, indent=1))
    elif a.command == "model-input":
        out, _ = model_input(a.sample, a.calibration_override, a.reason)
        print(json.dumps(dict(artifact=out["artifact"], calibration=out["calibration_comparison"], exposure_simultaneity=out["readiness"]["time_aligned"]["exposure_simultaneity"]["verified"], readiness={k: v["ok"] for k, v in out["readiness"].items()}, ready_for=out["ready_for"], pairing={k: out["pairing"].get(k) for k in ("skew_s", "within_limit", "limits_s", "mixed_clocks")},
                              q12={k: (v.get("status"), v.get("reason", "")[:80], v.get("empty")) for k, v in out["q12"].items() if isinstance(v, dict)}, geometry={k: (v["valid"], (v.get("reason") or "")[:90]) for k, v in out["geometry"].items()}, notes=out["notes"]), indent=1, default=str))
    elif a.command == "shadow":
        out = shadow(a.sample, shape_only=a.shape_only)
        print(json.dumps({k: v for k, v in out.items() if k != "policies"} | dict(policies={k: {x: v[x] for x in ("parameter_devices", "output_device", "chunk_shape", "finite")} for k, v in out.get("policies", {}).items()}), indent=1, default=str))


if __name__ == "__main__":
    main()
