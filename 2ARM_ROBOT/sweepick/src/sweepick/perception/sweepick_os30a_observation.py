"""READ03: the OS30A through its vendor SDK as the top stream of a field sample.

decode   the helper's record file -> FrameSets (image, depth, point cloud, factory rectification log, stamps)
bind     put the FrameSets of ONE helper run into the sample captured in the same window (raw.npz + sample.json):
         top_color = rectified left image, top_depth = SDK depth of the SAME FrameSet, top_info = the device's
         rectified intrinsics. Nothing from another instant is attached; what the SDK did not give stays missing.
The SDK process is the only owner of the camera (no V4L2 reader on the same device at the same time)."""
import ctypes, hashlib, json, struct, time
from pathlib import Path
import numpy as np

HEADER = "magic protocol_version mode_id rectify_log_index color_width color_height depth_width depth_height pc_width pc_height color_transport_bytes color_bgr_bytes depth_bytes pc_rgb_bytes pc_xyz_bytes rectify_log_bytes color_data_format color_rgb_format color_frame_number depth_frame_number pc_frame_number color_sdk_timestamp_ns depth_sdk_timestamp_ns pc_sdk_timestamp_ns pc_color_source_timestamp_ns pc_depth_source_timestamp_ns host_monotonic_ns host_realtime_ns".split()
MAGIC = 0x48323033


class RectLog(ctypes.Structure):
    _fields_ = [("InImgWidth", ctypes.c_ushort), ("InImgHeight", ctypes.c_ushort), ("OutImgWidth", ctypes.c_ushort), ("OutImgHeight", ctypes.c_ushort), ("RECT_ScaleEnable", ctypes.c_int), ("RECT_CropEnable", ctypes.c_int),
                ("RECT_ScaleWidth", ctypes.c_ushort), ("RECT_ScaleHeight", ctypes.c_ushort), ("CamMat1", ctypes.c_float * 9), ("CamDist1", ctypes.c_float * 8), ("CamMat2", ctypes.c_float * 9), ("CamDist2", ctypes.c_float * 8),
                ("RotaMat", ctypes.c_float * 9), ("TranMat", ctypes.c_float * 3), ("LRotaMat", ctypes.c_float * 9), ("RRotaMat", ctypes.c_float * 9), ("NewCamMat1", ctypes.c_float * 12), ("NewCamMat2", ctypes.c_float * 12)]


def rectify_log(blob):
    r = RectLog.from_buffer_copy(blob[:ctypes.sizeof(RectLog)])
    d = {n: (list(map(float, getattr(r, n))) if hasattr(getattr(r, n), "__len__") else getattr(r, n)) for n, _ in RectLog._fields_}
    d["calibration_fields_sha256"] = hashlib.sha256(blob[:456]).hexdigest()         # the bytes after ReProjectMat hold non-numeric values that change between reads
    return d


def decode(path):
    """Complete records of the helper's output. A truncated last record is reported, not used."""
    b, o, out, partial = Path(path).read_bytes(), 0, [], None
    while o + 140 <= len(b):
        h = dict(zip(HEADER, struct.unpack("<21I7Q", b[o:o + 140])))
        if h["magic"] != MAGIC:
            partial = f"bad magic at byte {o}"
            break
        sizes = [h["color_transport_bytes"], h["color_bgr_bytes"], h["depth_bytes"], h["pc_rgb_bytes"], h["pc_rgb_bytes"], h["pc_xyz_bytes"], h["rectify_log_bytes"]]
        if o + 140 + sum(sizes) > len(b):
            partial = f"record at byte {o} is truncated ({len(b) - o} of {140 + sum(sizes)} bytes)"
            break
        p, parts = o + 140, []
        for n in sizes:
            parts.append(b[p:p + n])
            p += n
        rec = dict(header=h, zd=np.frombuffer(parts[2], np.uint16).reshape(h["depth_height"], h["depth_width"]), rectify=parts[6], full=h["color_bgr_bytes"] > 0)
        if rec["full"]:
            rec["bgr"] = np.frombuffer(parts[1], np.uint8).reshape(h["color_height"], h["color_width"], 3)
            rec["xyz"] = np.frombuffer(parts[5], np.float32).reshape(h["pc_height"], h["pc_width"], 3)
        out.append(rec)
        o = p
    return out, partial


def roi_stats(zd, roi):
    x0, y0, x1, y1 = roi
    w = zd[y0:y1, x0:x1]
    v = w[w > 0].astype(float)
    return dict(valid_fraction=float((w > 0).mean()), median_mm=float(np.median(v)) if v.size else None, p05_mm=float(np.percentile(v, 5)) if v.size else None, p95_mm=float(np.percentile(v, 95)) if v.size else None)


def bind(sample_dir, sdk_bin, *, device, rois, helper, trace=None, exit_status=None):
    """rois: {name: [x0, y0, x1, y1]} in DEPTH pixels (640 x 460)."""
    sample_dir = Path(sample_dir)
    recs, partial = decode(sdk_bin)
    meta = json.loads((sample_dir / "sample.json").read_text())
    if not (sample_dir / "sample_prebind.json").exists():
        (sample_dir / "sample_prebind.json").write_text(json.dumps(meta, indent=2, default=str))
    report = dict(schema="tjj.sdk-top.v1", sdk_bin=str(sdk_bin), sdk_bin_sha256=hashlib.sha256(Path(sdk_bin).read_bytes()).hexdigest(), records=len(recs), partial=partial, helper=helper, helper_exit=exit_status, trace=trace, device=device)
    if not recs:
        meta["streams"]["top_color"] = dict(status="NO DATA", reason="the SDK helper wrote no complete FrameSet", owner="vendor SDK")
        (sample_dir / "sample.json").write_text(json.dumps(meta, indent=2, default=str))
        (sample_dir / "sdk_top.json").write_text(json.dumps(report, indent=1, default=str))
        return report
    t = np.array([r["header"]["color_sdk_timestamp_ns"] for r in recs]) / 1e9
    series = [dict(index=i, frame_number=[r["header"][k] for k in ("color_frame_number", "depth_frame_number", "pc_frame_number")], color_stamp_s=float(t[i]), depth_minus_color_ms=(r["header"]["depth_sdk_timestamp_ns"] - r["header"]["color_sdk_timestamp_ns"]) / 1e6,
                   host_realtime_minus_color_ms=(r["header"]["host_realtime_ns"] - r["header"]["color_sdk_timestamp_ns"]) / 1e6, full=r["full"], valid_fraction_whole=float((r["zd"] > 0).mean()),
                   roi={n: roi_stats(r["zd"], q) for n, q in rois.items()}) for i, r in enumerate(recs)]
    full = [i for i, r in enumerate(recs) if r["full"]]
    rect = rectify_log(recs[full[-1] if full else -1]["rectify"])
    same_rect = len({hashlib.sha256(r["rectify"][:456]).hexdigest() for r in recs}) == 1
    P = rect["NewCamMat1"]
    K = [P[0], 0.0, P[2], 0.0, P[5], P[6], 0.0, 0.0, 1.0]
    h0 = recs[0]["header"]
    report.update(series=series, rois_depth_px=rois, full_indices=full, rate_hz=float((len(t) - 1) / (t[-1] - t[0])) if len(t) > 1 and t[-1] > t[0] else None, rectify_log=rect, rectify_log_same_in_all_records=same_rect,
                  stamp_meaning="SDK frame timestamp in the host's realtime clock (microseconds). The SDK does not state whether it is taken at exposure or on reception; it is NOT a hardware exposure stamp",
                  spectrum="UNDECIDED: the stream the SDK calls 'color' is monochrome (rectified left image); visible mono or IR is not stated by the SDK")
    if full:
        raw = dict(np.load(sample_dir / "raw.npz")) if (sample_dir / "raw.npz").exists() else {}
        raw.update(top_color=np.stack([recs[i]["bgr"] for i in full]), top_color_stamp=t[full], top_depth=np.stack([recs[i]["zd"] for i in full]), top_depth_stamp=np.array([recs[i]["header"]["depth_sdk_timestamp_ns"] for i in full]) / 1e9,
                   top_xyz_mm=np.stack([recs[i]["xyz"] for i in full]), top_valid=np.stack([recs[i]["zd"] > 0 for i in full]))
        np.savez_compressed(sample_dir / "raw.npz", **raw)
        img = recs[full[0]]["bgr"]
        mono = bool((img[..., 0] == img[..., 1]).all() and (img[..., 1] == img[..., 2]).all())
        src = dict(owner="vendor SDK (eYs3D), sole owner of the camera during the capture", device=device, mode="PID0173 mode 3 (L'+D): 1280x920 MJPG + 640x460 depth, FrameSet", frame_numbers=[series[i]["frame_number"] for i in full], stamp_source=report["stamp_meaning"])
        meta["streams"]["top_color"] = dict(src, status="READ", encoding="bgr8", dtype="uint8", channels=3, width=h0["color_width"], height=h0["color_height"], frames=len(full), rate_hz=report["rate_hz"],
                                            kind="GRAY3 (three equal channels)" if mono else "COLOUR (channels differ)", image="rectified left image (L') as delivered by the SDK", spectrum=report["spectrum"])
        meta["streams"]["top_depth"] = dict(src, status="READ", unit="millimetres (uint16; 0 = invalid)", encoding="16UC1", width=h0["depth_width"], height=h0["depth_height"], frames=len(full), rate_hz=report["rate_hz"],
                                            same_frameset_as="top_color (same index = same FrameSet)", valid_fraction=[series[i]["valid_fraction_whole"] for i in full])
        meta["streams"]["top_info"] = dict(status="READ", K=K, D=[0.0] * 5, distortion_model="none: the image is rectified by the device (raw-lens K / D are kept in sdk_top.json and are NOT this K)", width=h0["color_width"], height=h0["color_height"],
                                           frame_id="os30a_rectified_left_optical", source="factory rectification log read from the device by the SDK in this run (NewCamMat1)", calibration_fields_sha256=rect["calibration_fields_sha256"],
                                           bound_to=dict(device=device, mode="mode 3", resolution=[h0["color_width"], h0["color_height"]], rectification="device (L')"))
        meta["contract"].setdefault("depth", {}).update(registered_to_color=True, pixel_ratio=h0["color_width"] // h0["depth_width"], frame_id="os30a_rectified_left_optical",
                                                        note="SDK: the depth is computed on the rectified left image; depth pixel (u, v) = image pixel (ratio * u, ratio * v). Declared by the adapter from the SDK mode, checked against the printed board in READ02")
        meta["sdk_top"] = dict(file="sdk_top.json", bound=time.strftime("%Y-%m-%dT%H:%M:%S%z"), prebind="sample_prebind.json")
    else:
        meta["streams"]["top_color"] = dict(status="NO DATA", reason="no complete FrameSet with an image", owner="vendor SDK")
    (sample_dir / "sample.json").write_text(json.dumps(meta, indent=2, default=str))
    (sample_dir / "sdk_top.json").write_text(json.dumps(report, indent=1, default=str))
    return report
