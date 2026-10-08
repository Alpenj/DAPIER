"""TJJ field tools, READ-ONLY: capture the real observation (OS30A colour + depth + CameraInfo, two wrist cameras, both
arms' joint and gripper feedback, TF), build the model input from it, and run the frozen ACT policies on it in SHADOW
(CUDA forward only). No motor command, no torque enable / disable, no base cmd_vel is sent by anything in this file.

  contract     print the field contract template (topics, frames, joint map) to fill in on the robot
  capture      ROS 2 (system python): subscribe, save a short raw sample + what the streams really are   [robot laptop]
  arms         lerobot bus (ACT venv): read positions / velocity / load / current of both followers        [robot laptop, needs approval]
  model-input  raw sample -> left / right / top (160x120) + q12, with K scaled the same way as the images
  shadow       model input -> START_3V / RECEIVE_3V forward on CUDA; writes the predicted chunk, sends nothing
  fixture      a raw sample made from a stored SIM frame (labelled SIM FIXTURE) to exercise the path without hardware

Raw streams are saved as they arrive; the model views are derived copies. A missing or stale K / T, an unmapped joint or
an unsupported feedback is written as MISSING / UNVERIFIED / UNAVAILABLE and makes the sample incomplete: nothing is
filled in from the simulator. A shadow result shows input compatibility only: it is not a pick result and not a safety
approval.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

MODEL_W, MODEL_H = 160, 120
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
WRITE_CALLS = ("write", "sync_write", "enable_torque", "disable_torque", "configure_motors", "configure", "reset_calibration", "write_calibration", "set_half_turn_homings", "setup_motor", "set_baudrate")


class ReadOnlyViolation(RuntimeError):
    pass


def template():
    stream = lambda topic, note, device=None: dict(topic=topic, device=device, note=note)
    joint = lambda n: dict(name=n, sign=None, offset_rad=None, verified=False)
    arm = lambda: dict(joints=[joint(n) for n in JOINTS[:5]], gripper=dict(feedback_open=None, feedback_closed=None, verified=False))
    return dict(
        schema="tjj.field-contract.v1",
        note="fill the null fields on the robot. Topic names marked 'suggested' come from the team's simulation bridge files and are NOT verified on the real robot",
        streams=dict(top_color=stream("/os30a/camera/color/image_raw", "suggested (gazebo_os30a_bridge.yaml)"), top_depth=stream("/os30a/camera/depth/image_raw", "suggested (gazebo_os30a_bridge.yaml)"),
                     top_info=stream(None, "sensor_msgs/CameraInfo of the OS30A colour stream; without it K is MISSING"),
                     left_wrist=stream(None, "a ROS topic, or the V4L2 device of the existing recorder (record.env: LEFT_WRIST_CAMERA); device not opened or verified here", "/dev/dapier/left_wrist_rgb"),
                     right_wrist=stream(None, "a ROS topic, or the V4L2 device of the existing recorder (record.env: RIGHT_WRIST_CAMERA); device not opened or verified here", "/dev/dapier/right_wrist_rgb"),
                     left_wrist_info=stream(None, "optional"), right_wrist_info=stream(None, "optional")),
        joints=dict(left=dict(topic=None, names=None, note="sensor_msgs/JointState of the left follower (6 names in the order of q12), or use `arms` for the lerobot bus"),
                    right=dict(topic=None, names=None, note="same for the right follower"), max_age_s=0.1),
        tf=dict(base_frame=None, top_camera_optical_frame=None, max_age_s=None, note="camera -> arm base transform; MISSING if the frames are not given or TF has no transform"),
        q12=dict(order="left 5 arm joints (rad) + left gripper (opening 0..1) + right 5 arm joints (rad) + right gripper (opening 0..1)", unit_in="rad from JointState, or lerobot degrees converted",
                 left=arm(), right=arm(), note="model angle = sign * feedback_rad + offset_rad; opening = (feedback - closed) / (open - closed). These come from the sim-to-real joint calibration, "
                                               "which is not done: until verified is true the q12 is UNVERIFIED"),
        lerobot=dict(left_port="/dev/dapier/left_arm", right_port="/dev/dapier/right_arm", calibration_dir="~/.config/dapier/lerobot-calibration", left_id="dapier_dual_follower_left",
                     right_id="dapier_dual_follower_right", note="ports as in the existing recorder's record.env (LEFT_FOLLOWER_PORT / RIGHT_FOLLOWER_PORT); not opened or verified here"),
        model=dict(image_size=[MODEL_W, MODEL_H], order=["left_wrist_rgb", "right_wrist_rgb", "os30a top gray3"], colour_order="RGB", top_view="gray3 made from the colour stream (the policies were trained on a gray3 SIM top view)"))


# ----------------------------------------------------------------------------------------------
# Raw sample: what the streams really are
# ----------------------------------------------------------------------------------------------

def decode_image(encoding, height, width, step, data):
    """sensor_msgs/Image -> numpy without cv_bridge. Returns (array as sent, facts)."""
    enc = encoding.lower()
    table = dict(rgb8=(np.uint8, 3), bgr8=(np.uint8, 3), rgba8=(np.uint8, 4), bgra8=(np.uint8, 4), mono8=(np.uint8, 1), mono16=(np.uint16, 1))
    table.update({"8uc1": (np.uint8, 1), "8uc3": (np.uint8, 3), "16uc1": (np.uint16, 1), "32fc1": (np.float32, 1)})
    if enc not in table:
        raise ValueError(f"unsupported image encoding {encoding!r}")
    dtype, ch = table[enc]
    row = np.frombuffer(bytes(data), dtype=np.uint8).reshape(height, step)[:, :width * ch * np.dtype(dtype).itemsize]
    img = row.copy().view(dtype).reshape(height, width, ch)
    return (img[..., 0] if ch == 1 else img), dict(encoding=encoding, dtype=np.dtype(dtype).name, channels=ch, width=width, height=height)


def colour_facts(img, facts):
    f = dict(facts)
    if img.ndim == 2:
        f["kind"] = "MONO (one channel): not a colour stream"
    else:
        same = bool(np.array_equal(img[..., 0], img[..., 1]) and np.array_equal(img[..., 1], img[..., 2]))
        f["kind"] = "GRAY3 (three equal channels: mono / IR sent as a colour image)" if same else "COLOUR (channels differ)"
        f["channel_means"] = [round(float(img[..., i].mean()), 2) for i in range(min(3, img.shape[2]))]
    return f


def depth_facts(img, facts):
    f = dict(facts)
    f["unit"] = {"16uc1": "millimetres (uint16, REP 118)", "mono16": "millimetres (uint16)", "32fc1": "metres (float32, REP 118)"}.get(facts["encoding"].lower(), "UNKNOWN")
    valid = img[np.isfinite(img) & (img > 0)] if img.size else img
    f.update(valid_share=round(float(valid.size / max(img.size, 1)), 4), min=float(valid.min()) if valid.size else None, max=float(valid.max()) if valid.size else None)
    return f


def rate(stamps):
    s = np.asarray(sorted(stamps), dtype=float)
    return None if len(s) < 3 else round(float(1.0 / np.median(np.diff(s))), 2)


def save_sample(out, frames, facts, meta):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    arrays = {}
    for name, rows in frames.items():
        if rows and isinstance(rows[0]["data"], np.ndarray):
            arrays[name] = np.stack([r["data"] for r in rows])
            arrays[name + "_stamp"] = np.array([r["stamp_s"] for r in rows])
    np.savez_compressed(out / "raw.npz", **arrays)
    (out / "sample.json").write_text(json.dumps(dict(meta, streams=facts, arrays=sorted(arrays)), indent=2, default=str))
    return out


# ----------------------------------------------------------------------------------------------
# ROS 2 capture (system python). The node cannot publish, call a service or send an action goal.
# ----------------------------------------------------------------------------------------------

def capture_ros(contract, out, *, samples=10, timeout_s=15.0, node_name="tjj_readonly_capture"):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, JointState

    class ReadOnlyNode(Node):
        locked = False

        def create_publisher(self, *a, **k):
            if self.locked:
                raise ReadOnlyViolation("read-only capture: this node does not publish")
            return super().create_publisher(*a, **k)

        def create_client(self, *a, **k):
            raise ReadOnlyViolation("read-only capture: this node calls no service")

    stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    own = rclpy.ok() is False
    if own:
        rclpy.init()
    node = ReadOnlyNode(node_name, start_parameter_services=False, enable_rosout=False)
    node.locked = True
    frames, facts, infos, joints, problems = {}, {}, {}, {}, []
    now = lambda: node.get_clock().now().nanoseconds * 1e-9

    def image_cb(name, depth):
        def cb(msg):
            try:
                img, f = decode_image(msg.encoding, msg.height, msg.width, msg.step, msg.data)
            except ValueError as e:
                facts[name] = dict(status="UNSUPPORTED", reason=str(e))
                return
            rows = frames.setdefault(name, [])
            if len(rows) < samples:
                rows.append(dict(data=img, stamp_s=stamp(msg.header), received_s=now(), frame_id=msg.header.frame_id))
            f = depth_facts(img, f) if depth else colour_facts(img, f)
            facts[name] = dict(f, status="READ", frame_id=msg.header.frame_id, topic=contract["streams"][name]["topic"], first_stamp_s=rows[0]["stamp_s"])
        return cb

    for name, s in contract["streams"].items():
        if not s.get("topic"):
            if not s.get("device"):
                facts[name] = dict(status="MISSING", reason="no topic and no device in the contract")
            continue
        if name.endswith("_info"):
            node.create_subscription(CameraInfo, s["topic"], lambda m, n=name: infos.setdefault(n, dict(K=[float(x) for x in m.k], D=[float(x) for x in m.d], width=m.width, height=m.height, frame_id=m.header.frame_id,
                                                                                                    distortion_model=m.distortion_model, stamp_s=stamp(m.header))), qos_profile_sensor_data)
        else:
            node.create_subscription(Image, s["topic"], image_cb(name, name.endswith("depth")), qos_profile_sensor_data)
    for side in ("left", "right"):
        j = contract["joints"][side]
        if not j.get("topic"):
            joints[side] = dict(status="MISSING", reason="no joint topic in the contract (use `arms` for the lerobot bus)")
            continue

        def jcb(msg, side=side):
            rows = frames.setdefault(f"joint_{side}", [])
            if len(rows) < 10 * samples:
                rows.append(dict(stamp_s=stamp(msg.header), received_s=now(), names=list(msg.name), position=list(msg.position), velocity=list(msg.velocity), effort=list(msg.effort), data=None))
        node.create_subscription(JointState, j["topic"], jcb, qos_profile_sensor_data)
    tf_buffer = None
    tfc = contract.get("tf", {})
    if tfc.get("base_frame") and tfc.get("top_camera_optical_frame"):
        from tf2_ros import Buffer, TransformListener
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node, spin_thread=False)                 # a subscriber to /tf and /tf_static
    t0 = time.monotonic()
    want = [n for n, s in contract["streams"].items() if s.get("topic") and not n.endswith("_info")]
    while time.monotonic() - t0 < timeout_s and not all(len(frames.get(n, [])) >= samples for n in want):
        rclpy.spin_once(node, timeout_sec=0.05)
    for n, s_ in contract["streams"].items():                                # V4L2 devices (the existing recorder's wrist cameras): opened for reading only
        if s_.get("topic") or not s_.get("device") or n.endswith("_info"):
            continue
        import cv2
        cap = cv2.VideoCapture(s_["device"])
        if not cap.isOpened():
            facts[n] = dict(status="NO DATA", device=s_["device"], reason="the device could not be opened")
            continue
        rows = []
        for _ in range(samples):
            ok, img = cap.read()
            if ok:
                rows.append(dict(data=img, stamp_s=time.time(), received_s=time.time(), frame_id=s_["device"]))
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        props = dict(driver_fps=cap.get(cv2.CAP_PROP_FPS), fourcc="".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4)))
        cap.release()
        if not rows:
            facts[n] = dict(status="NO DATA", device=s_["device"], reason="no frame read")
            continue
        frames[n] = rows
        img = rows[0]["data"]
        facts[n] = dict(colour_facts(img, dict(encoding="bgr8", dtype=str(img.dtype), channels=img.shape[2], width=img.shape[1], height=img.shape[0])), status="READ", device=s_["device"], frames=len(rows),
                        rate_hz=rate([r["stamp_s"] for r in rows]), **props, stamp_source="host time at read (the device gives no capture time through OpenCV): capture latency is not measured")
    for n in want:
        got = len(frames.get(n, []))
        if got == 0:
            facts[n] = dict(status="NO DATA", topic=contract["streams"][n]["topic"], reason=f"no message in {timeout_s} s")
        elif "frames" not in facts[n]:
            facts[n]["frames"] = got
            facts[n]["rate_hz"] = rate([r["stamp_s"] for r in frames[n]])
            facts[n]["stamp_minus_receive_s"] = round(float(np.median([r["stamp_s"] - r["received_s"] for r in frames[n]])), 4)
    for n, s in contract["streams"].items():
        if n.endswith("_info") and s.get("topic"):
            facts[n] = dict(infos[n], status="READ", topic=s["topic"]) if n in infos else dict(status="NO DATA", topic=s["topic"])
    for side in ("left", "right"):
        rows = frames.pop(f"joint_{side}", [])
        if rows:
            joints[side] = dict(status="READ", topic=contract["joints"][side]["topic"], messages=len(rows), rate_hz=rate([r["stamp_s"] for r in rows]), names=rows[-1]["names"], last=rows[-1],
                                has_velocity=bool(rows[-1]["velocity"]), has_effort=bool(rows[-1]["effort"]), rows=rows[-samples:],
                                note="JointState.position is feedback as the driver publishes it; whether it is measured or the commanded value must be confirmed with the driver")
        elif side not in joints:
            joints[side] = dict(status="NO DATA", topic=contract["joints"][side]["topic"])
    tf_row = dict(status="MISSING", reason="base_frame / top_camera_optical_frame not given in the contract")
    if tf_buffer is not None:
        try:
            from rclpy.time import Time
            t = tf_buffer.lookup_transform(tfc["base_frame"], tfc["top_camera_optical_frame"], Time())
            age = now() - (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
            tr, ro = t.transform.translation, t.transform.rotation
            static = t.header.stamp.sec == 0 and t.header.stamp.nanosec == 0
            tf_row = dict(status="READ", parent=tfc["base_frame"], child=tfc["top_camera_optical_frame"], translation=[tr.x, tr.y, tr.z], quaternion_xyzw=[ro.x, ro.y, ro.z, ro.w], static=static,
                          age_s=None if static else round(age, 3))
            if not static and tfc.get("max_age_s") is not None and age > tfc["max_age_s"]:
                tf_row["status"] = "STALE"
        except Exception as e:
            tf_row = dict(status="MISSING", reason=f"{type(e).__name__}: {e}")
    name, ns = node.get_name(), node.get_namespace()
    everything = [t for t, _ in node.get_publisher_names_and_types_by_node(name, ns)]
    infra = [t for t in everything if t in ("/parameter_events", "/rosout")]   # created by rclpy for every node; they carry no command
    published = [t for t in everything if t not in infra]
    graph = dict(node=name, topics_published_by_this_node=published, ros_infrastructure_topics_of_this_node=infra, service_clients=[s for s, _ in node.get_client_names_and_types_by_node(name, ns)],
                 cmd_vel_publishers_seen=sorted({p.node_name for p in node.get_publishers_info_by_topic("/cmd_vel")}),
                 read_only=not published, note="cmd_vel publishers listed here belong to other nodes of the graph (e.g. the team's navigation); this node has none")
    meta = dict(schema="tjj.field-sample.v1", source="ROS2 topics (read-only node)", captured=time.strftime("%Y-%m-%dT%H:%M:%S%z"), ros_time_s=now(), joints=joints, tf=tf_row, graph=graph,
                commands_sent=dict(motor=0, torque_enable=0, base_cmd_vel=0), contract=contract)
    out = save_sample(out, {k: v for k, v in frames.items()}, facts, meta)
    node.destroy_node()
    if own:
        rclpy.shutdown()
    return out, facts, meta


# ----------------------------------------------------------------------------------------------
# lerobot bus, read-only (both followers). Not run without hardware and approval.
# ----------------------------------------------------------------------------------------------

class ReadOnlyBus:
    """Forwards reads to a motor bus and refuses every call that writes a register."""

    def __init__(self, bus):
        object.__setattr__(self, "_bus", bus)
        object.__setattr__(self, "refused", [])

    def __getattr__(self, name):
        if name in WRITE_CALLS or name.startswith(("write", "set_", "enable", "disable", "configure", "reset")):
            def refuse(*a, **k):
                self.refused.append(name)
                raise ReadOnlyViolation(f"read-only adapter: {name}() is not sent to the motors")
            return refuse
        return getattr(self._bus, name)

    def connect(self, *a, **k):
        try:
            return self._bus.connect(*a, **k)
        except BaseException:
            if self._bus.port_handler.is_open:
                self._bus.port_handler.closePort()
            raise

    def disconnect(self, *a, **k):
        return self._bus.disconnect(disable_torque=False)                     # the default of lerobot would write Torque_Enable = 0


def read_arm(bus, names=JOINTS, repeats=5):
    """Feedback registers of one follower through a ReadOnlyBus. Unsupported registers are UNAVAILABLE, not zero."""
    out = dict(names=list(names))
    for key, reg in (("position_raw", "Present_Position"), ("goal_position_raw", "Goal_Position"), ("velocity_raw", "Present_Velocity"), ("load_raw", "Present_Load"), ("current_raw", "Present_Current"),
                     ("torque_enable", "Torque_Enable")):
        try:
            rows, stamps = [], []
            for _ in range(repeats):
                got = bus.sync_read(reg, normalize=False)
                rows.append([float(got[n]) for n in names])
                stamps.append(time.time())
            out[key] = dict(status="READ", register=reg, last=rows[-1], spread=[round(max(c) - min(c), 3) for c in zip(*rows)], rate_hz=rate(stamps), stamp_s=stamps[-1])
        except ReadOnlyViolation:
            raise
        except Exception as e:
            out[key] = dict(status="UNAVAILABLE", register=reg, reason=f"{type(e).__name__}: {e}")
    p, g = out["position_raw"], out["goal_position_raw"]
    out["command_and_feedback_are_separate_registers"] = bool(p["status"] == "READ" and g["status"] == "READ")
    out["load_note"] = ("Present_Load / Present_Current were read: their unit, sign and update rate on this firmware still have to be checked against the register table; "
                        "a load value alone does not prove a stable grasp") if out["load_raw"]["status"] == "READ" or out["current_raw"]["status"] == "READ" else "no load or current feedback: UNAVAILABLE"
    return out


def arms_lerobot(contract, out):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    cfg, rows = contract["lerobot"], {}
    for side in ("left", "right"):
        port = cfg.get(f"{side}_port")
        if not port:
            rows[side] = dict(status="MISSING", reason="no port in the contract")
            continue
        motors = {n: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES) for i, n in enumerate(JOINTS)}
        bus = ReadOnlyBus(FeetechMotorsBus(port=port, motors=motors))
        bus.connect()
        try:
            rows[side] = dict(read_arm(bus), status="READ", port=port, refused_calls=bus.refused)
        finally:
            bus.disconnect()
    Path(out).mkdir(parents=True, exist_ok=True)
    (Path(out) / "arms.json").write_text(json.dumps(dict(schema="tjj.field-arms.v1", source="lerobot Feetech bus, read-only wrapper", captured=time.strftime("%Y-%m-%dT%H:%M:%S%z"), arms=rows,
                                                         commands_sent=dict(motor=0, torque_enable=0, base_cmd_vel=0)), indent=2, default=str))
    return rows


# ----------------------------------------------------------------------------------------------
# Model input: derived views; K follows the same crop and scale as the image
# ----------------------------------------------------------------------------------------------

def to_rgb(img, encoding):
    e = encoding.lower()
    if img.ndim == 2:
        return np.repeat(img[..., None], 3, axis=2)
    if e.startswith("bgr"):
        return img[..., 2::-1].copy()
    return img[..., :3].copy()


def fit(img, K=None, *, nearest=False):
    """Centre crop to 4:3, then resize to 160x120. K is cropped and scaled with it. Returns (image, K or None, record)."""
    import cv2
    h, w = img.shape[:2]
    tw = min(w, int(round(h * MODEL_W / MODEL_H)))
    th = min(h, int(round(tw * MODEL_H / MODEL_W)))
    x0, y0 = (w - tw) // 2, (h - th) // 2
    crop = img[y0:y0 + th, x0:x0 + tw]
    sx, sy = MODEL_W / tw, MODEL_H / th
    out = cv2.resize(crop, (MODEL_W, MODEL_H), interpolation=cv2.INTER_NEAREST if nearest else cv2.INTER_AREA)
    K2 = None
    if K is not None:
        K2 = np.array(K, dtype=float).reshape(3, 3).copy()
        K2[0, 2], K2[1, 2] = (K2[0, 2] - x0) * sx, (K2[1, 2] - y0) * sy
        K2[0, 0], K2[1, 1] = K2[0, 0] * sx, K2[1, 1] * sy
    return out, K2, dict(source_size=[w, h], crop_xywh=[x0, y0, tw, th], scale=[sx, sy], interpolation="nearest" if nearest else "area")


def q12_from(joints, contract):
    """(q12 or None, per-side record). The joint map of the contract must be filled; otherwise UNVERIFIED / MISSING."""
    q, rec, ok, verified = [], {}, True, True
    for side in ("left", "right"):
        j, mp = joints.get(side) or {}, contract["q12"][side]
        if j.get("status") != "READ":
            rec[side] = dict(status="MISSING", reason=j.get("reason") or j.get("status") or "no joint feedback in the sample")
            ok = False
            continue
        names, pos = j["last"]["names"], j["last"]["position"]
        want = contract["joints"][side].get("names") or [x["name"] for x in mp["joints"]] + ["gripper"]
        if any(n not in names for n in want):
            rec[side] = dict(status="MISSING", reason=f"joint names {want} not all in {names}")
            ok = False
            continue
        p = [pos[names.index(n)] for n in want]
        g = mp["gripper"]
        if any(x["sign"] is None or x["offset_rad"] is None for x in mp["joints"]) or g["feedback_open"] is None or g["feedback_closed"] is None:
            rec[side] = dict(status="UNVERIFIED", reason="the joint map (sign / offset / gripper open-closed) of the contract is not filled: no q12 is made from guessed values", feedback=p)
            ok = False
            continue
        arm = [x["sign"] * p[i] + x["offset_rad"] for i, x in enumerate(mp["joints"])]
        opening = float(np.clip((p[5] - g["feedback_closed"]) / (g["feedback_open"] - g["feedback_closed"]), 0.0, 1.0))
        v = all(x["verified"] for x in mp["joints"]) and g["verified"]
        verified &= v
        rec[side] = dict(status="MAPPED" if v else "MAPPED (map present but not marked verified)", feedback=p, model=arm + [opening], stamp_s=j["last"]["stamp_s"])
        q += arm + [opening]
    return (np.array(q, dtype=np.float32) if ok else None), dict(rec, verified=bool(ok and verified))


def model_input(sample_dir, index=-1):
    sample_dir = Path(sample_dir)
    meta = json.loads((sample_dir / "sample.json").read_text())
    raw = np.load(sample_dir / "raw.npz")
    s, contract = meta["streams"], meta["contract"]
    rec, missing, arrays = {}, [], {}
    for name, key in (("left_wrist", "left"), ("right_wrist", "right"), ("top_color", "top")):
        if name not in raw.files:
            missing.append(f"{name}: {s.get(name, {}).get('status', 'MISSING')}")
            continue
        rgb = to_rgb(raw[name][index], s[name]["encoding"])
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb.astype(np.float32) / 256.0, 0, 255).astype(np.uint8)
        K = (s.get("top_info") or {}).get("K") if key == "top" else (s.get(f"{name}_info") or {}).get("K")
        img, K2, how = fit(rgb, K)
        if key == "top":
            luma = np.round(0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]).astype(np.uint8)
            img = np.repeat(luma[..., None], 3, axis=2)                       # the policies were trained on a gray3 top view
            how["derived"] = "gray3 from the colour stream (raw stream kept as sent)"
            if K2 is None:
                missing.append("top K: MISSING (no CameraInfo); depth geometry cannot be used")
            else:
                arrays["top_K"] = K2
        arrays[key] = img
        rec[key] = dict(how, source_kind=s[name].get("kind"), capture_stamp_s=float(raw[name + "_stamp"][index]), K_scaled=None if K2 is None else K2.round(4).tolist())
    if "top_depth" in raw.files:
        d = raw["top_depth"][index].astype(np.float32)
        if s["top_depth"].get("unit", "").startswith("millimetres"):
            d = d / 1000.0
        d2, _, how = fit(d, None, nearest=True)
        arrays["top_depth_m"] = d2
        rec["top_depth"] = dict(how, unit_in=s["top_depth"].get("unit"), unit_out="metres", capture_stamp_s=float(raw["top_depth_stamp"][index]),
                                same_crop_as_colour=bool(rec.get("top") and rec["top"]["source_size"] == how["source_size"]),
                                note="uses the colour K only if depth is registered to colour and has the same size; otherwise the depth K / extrinsics are needed (UNVERIFIED)")
    else:
        missing.append("top depth: " + s.get("top_depth", {}).get("status", "MISSING"))
    tf = meta.get("tf", {})
    if tf.get("status") != "READ":
        missing.append(f"camera -> base transform: {tf.get('status', 'MISSING')} ({tf.get('reason', '')})")
    q, qrec = q12_from(meta.get("joints", {}), contract)
    if q is None:
        missing.append("q12: " + "; ".join(f"{k}: {v['status']}" for k, v in qrec.items() if isinstance(v, dict)))
    else:
        arrays["q12"] = q
    stamps = [v["capture_stamp_s"] for v in rec.values()] + [v["stamp_s"] for v in qrec.values() if isinstance(v, dict) and "stamp_s" in v]
    out = dict(schema="tjj.model-input.v1", sample=str(sample_dir), index=index, views=rec, q12=qrec, tf=tf, complete=not missing, missing_or_unverified=missing,
               time=dict(stamps_s=stamps, spread_s=None if len(stamps) < 2 else round(max(stamps) - min(stamps), 4), note="capture times of the three views and the joint read; the allowed spread on the real robot is not measured yet"),
               order=contract["model"]["order"], colour_order="RGB", image_size=[MODEL_W, MODEL_H], is_fixture=bool(meta.get("fixture")))
    np.savez_compressed(sample_dir / "model_input.npz", **arrays)
    (sample_dir / "model_input.json").write_text(json.dumps(out, indent=2, default=str))
    return out, arrays


# ----------------------------------------------------------------------------------------------
# Shadow inference (ACT venv, CUDA). Sends nothing.
# ----------------------------------------------------------------------------------------------

def shadow(sample_dir, *, allow_unverified_q=False):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import torch
    import receive_candidate as rc
    import tjj_alignment_r1 as al
    import tjj_mvp_r2 as r2m
    from dapier_act_policy import MEASURED_Q_KEY
    from sim_data_factory.r2_data import policy_observation
    sample_dir = Path(sample_dir)
    info, arrays = model_input(sample_dir)
    out = dict(schema="tjj.shadow.v1", sample=str(sample_dir), is_fixture=info["is_fixture"], commands_sent=dict(motor=0, torque_enable=0, base_cmd_vel=0),
               meaning="input compatibility only: not a pick result, not a safety approval", complete_input=info["complete"], missing_or_unverified=info["missing_or_unverified"])
    need = [k for k in ("left", "right", "top", "q12") if k not in arrays]
    if need:
        out.update(status="NOT RUN", reason=f"model input incomplete: {need}; nothing is filled in from the simulator")
    elif not info["q12"]["verified"] and not allow_unverified_q:
        out.update(status="NOT RUN", reason="q12 map not verified; rerun with --allow-unverified-q to see shapes only (the output is then labelled UNVERIFIED)")
    elif not torch.cuda.is_available():
        out.update(status="NOT RUN", reason="CUDA is not available on this machine: no CPU fallback")
    else:
        m = r2m.models()
        q = arrays["q12"].astype(np.float32)
        rows = {}
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
            rows[name] = dict(model_sha256=meta["model_sha256"], parameter_devices=sorted({str(p.device) for p in policy.parameters()}), input_devices=sorted({str(v.device) for v in batch.values()}),
                              output_device=str(chunk.device), chunk_shape=list(c.shape), finite=bool(np.isfinite(c).all()), forward_s=round(time.perf_counter() - t0, 4), training_flag=bool(policy.training),
                              first_action=c[0].round(4).tolist(), first_action_minus_q=(c[0] - q).round(4).tolist(), input_keys=sorted(batch))
            np.save(sample_dir / f"shadow_chunk_{name}.npy", c)
        out.update(status="RAN" + ("" if info["q12"]["verified"] else " (q12 UNVERIFIED: shapes and devices only)"), device=torch.cuda.get_device_name(0), policies=rows,
                   command_sink="none: the predicted chunk is written to a file; no publisher, no serial port, no simulator is attached to this process")
    (sample_dir / "shadow.json").write_text(json.dumps(out, indent=2, default=str))
    return out


def fixture(out, seed=5035):
    """A raw sample in the field format made from one stored SIM frame (R4 hybrid run). SIM FIXTURE, not a capture."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import tjj_mvp_r4 as r4
    run = r4.R3_RUNS / f"hybrid_{seed}"
    with np.load(run / "episode.npz", allow_pickle=False) as a:
        q = a["q_state"][0].astype(float)
    cams = {c: np.load(run / "cameras" / f"{c}.npz") for c in r4.CAMERAS}
    top = cams[r4.TOP]
    frames = dict(left_wrist=[dict(data=cams["left_wrist_rgb"]["rgb"][0], stamp_s=0.0)], right_wrist=[dict(data=cams["right_wrist_rgb"]["rgb"][0], stamp_s=0.0)],
                  top_color=[dict(data=top["rgb"][0], stamp_s=0.0)], top_depth=[dict(data=top["depth_m"][0].astype(np.float32), stamp_s=0.0)])
    f = lambda img, enc: dict(colour_facts(img, dict(encoding=enc, dtype=str(img.dtype), channels=3, width=img.shape[1], height=img.shape[0])), status="READ", frames=1)
    facts = dict(left_wrist=f(frames["left_wrist"][0]["data"], "rgb8"), right_wrist=f(frames["right_wrist"][0]["data"], "rgb8"), top_color=f(frames["top_color"][0]["data"], "rgb8"),
                 top_depth=dict(depth_facts(frames["top_depth"][0]["data"], dict(encoding="32FC1", dtype="float32", channels=1, width=160, height=120)), status="READ", frames=1),
                 top_info=dict(status="READ", K=top["K"][0].ravel().tolist(), width=160, height=120, frame_id="SIM os30a"))
    contract = template()
    for side in ("left", "right"):
        for x in contract["q12"][side]["joints"]:
            x.update(sign=1.0, offset_rad=0.0, verified=True)                 # SIM joints are already model joints
        contract["q12"][side]["gripper"].update(feedback_open=1.0, feedback_closed=0.0, verified=True)
    joints = {side: dict(status="READ", last=dict(names=list(JOINTS), position=q[o:o + 6].tolist(), stamp_s=0.0)) for side, o in (("left", 0), ("right", 6))}
    meta = dict(schema="tjj.field-sample.v1", fixture="SIM FIXTURE: one stored frame of R3 hybrid_%d written in the field sample format; NOT a real capture" % seed, source=str(run), joints=joints,
                tf=dict(status="READ", parent="SIM world", child="SIM os30a optical", note="SIM transform of the stored frame"), contract=contract, commands_sent=dict(motor=0, torque_enable=0, base_cmd_vel=0))
    return save_sample(out, frames, facts, meta)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)
    c = sub.add_parser("contract")
    c.add_argument("--out")
    for name in ("capture", "arms"):
        x = sub.add_parser(name)
        x.add_argument("--contract", required=True)
        x.add_argument("--out", required=True)
        if name == "capture":
            x.add_argument("--samples", type=int, default=10)
            x.add_argument("--timeout", type=float, default=15.0)
    for name in ("model-input", "shadow"):
        x = sub.add_parser(name)
        x.add_argument("--sample", required=True)
        if name == "shadow":
            x.add_argument("--allow-unverified-q", action="store_true")
    f = sub.add_parser("fixture")
    f.add_argument("--out", required=True)
    a = p.parse_args(argv)
    if a.command == "contract":
        text = json.dumps(template(), indent=2)
        (Path(a.out).write_text(text) if a.out else print(text))
    elif a.command == "capture":
        out, facts, meta = capture_ros(json.loads(Path(a.contract).read_text()), a.out, samples=a.samples, timeout_s=a.timeout)
        print(json.dumps(dict(sample=str(out), streams={k: (v.get("status"), v.get("kind") or v.get("unit"), v.get("width"), v.get("height"), v.get("rate_hz")) for k, v in facts.items()},
                              joints={k: v.get("status") for k, v in meta["joints"].items()}, tf=meta["tf"]["status"], graph=meta["graph"], commands_sent=meta["commands_sent"]), indent=1, default=str))
    elif a.command == "arms":
        print(json.dumps(arms_lerobot(json.loads(Path(a.contract).read_text()), a.out), indent=1, default=str)[:3000])
    elif a.command == "model-input":
        out, _ = model_input(a.sample)
        print(json.dumps(dict(complete=out["complete"], missing_or_unverified=out["missing_or_unverified"], views={k: (v["source_size"], v["crop_xywh"], v.get("source_kind")) for k, v in out["views"].items()}), indent=1))
    elif a.command == "shadow":
        out = shadow(a.sample, allow_unverified_q=a.allow_unverified_q)
        print(json.dumps({k: out[k] for k in out if k != "policies"} | dict(policies={k: {x: v[x] for x in ("parameter_devices", "input_devices", "output_device", "chunk_shape", "finite", "forward_s")} for k, v in out.get("policies", {}).items()}), indent=1, default=str))
    elif a.command == "fixture":
        print(fixture(a.out))


if __name__ == "__main__":
    main()
