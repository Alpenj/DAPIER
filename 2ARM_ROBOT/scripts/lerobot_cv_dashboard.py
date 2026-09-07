#!/usr/bin/env python3
"""Run LeRobot teleop/record with a local OpenCV camera dashboard."""

from __future__ import annotations

import sys
import os
import struct
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np


PANEL_SIZE = (320, 240)
CAMERAS = (
    ("left_wrist", "LEFT WRIST RGB"),
    ("top_h201_depth", "TOP H201 DEPTH (mm)"),
    ("right_wrist", "RIGHT WRIST RGB"),
)
WINDOW = "DAPIER cameras: LEFT | H201 | RIGHT"
HEADER = struct.Struct("<IIIIQ")
MAGIC = 0x48323031
STATUS = "STARTING"
RECORD_EVENTS: dict | None = None
LEFT_KEYS = {81, 65361, 2424832, 16777234}
RIGHT_KEYS = {83, 65363, 2555904, 16777236}


class H201DepthCamera:
    width = 640
    height = 460
    fps = 15
    use_rgb = False
    use_depth = True

    def __init__(self) -> None:
        self.process: subprocess.Popen | None = None
        self.thread: threading.Thread | None = None
        self.read_fd: int | None = None
        self.frame: np.ndarray | None = None
        self.timestamp = 0.0
        self.lock = threading.Lock()
        self.ready = threading.Event()

    @property
    def is_connected(self) -> bool:
        return self.process is not None and self.process.poll() is None and self.ready.is_set()

    @staticmethod
    def find_cameras() -> list[dict]:
        return []

    @staticmethod
    def _read_exact(fd: int, size: int) -> bytes | None:
        chunks = bytearray()
        while len(chunks) < size:
            chunk = os.read(fd, size - len(chunks))
            if not chunk:
                return None
            chunks.extend(chunk)
        return bytes(chunks)

    def _reader(self) -> None:
        assert self.read_fd is not None
        while True:
            raw_header = self._read_exact(self.read_fd, HEADER.size)
            if raw_header is None:
                return
            magic, width, height, payload_size, _timestamp_ns = HEADER.unpack(raw_header)
            if magic != MAGIC or (width, height, payload_size) != (self.width, self.height, width * height * 2):
                return
            payload = self._read_exact(self.read_fd, payload_size)
            if payload is None:
                return
            frame = np.frombuffer(payload, dtype="<u2").reshape(height, width, 1).copy()
            frame[frame > 10_000] = 0
            with self.lock:
                self.frame = frame
                self.timestamp = time.monotonic()
            self.ready.set()

    def connect(self, warmup: bool = True) -> None:
        helper = Path.home() / ".local/lib/dapier/h201_depth_stream"
        sdk_root = Path.home() / ".local/opt/eys3d_ros/dm_preview/eYs3D_wrapper"
        if not helper.exists():
            raise ConnectionError(f"missing H201 SDK helper: {helper}")
        read_fd, write_fd = os.pipe()
        self.read_fd = read_fd
        self.process = subprocess.Popen(
            [str(helper), str(write_fd)],
            cwd=sdk_root,
            pass_fds=(write_fd,),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        os.close(write_fd)
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        if warmup and not self.ready.wait(20):
            self.disconnect()
            raise ConnectionError("H201 SDK did not produce a depth frame within 20 seconds")

    def read(self) -> np.ndarray:
        return self.read_latest_depth()

    def async_read(self, timeout_ms: float = 200) -> np.ndarray:
        if not self.ready.wait(timeout_ms / 1000):
            raise TimeoutError("H201 depth frame timeout")
        return self.read_latest_depth()

    def read_latest_depth(self, max_age_ms: int = 500) -> np.ndarray:
        with self.lock:
            if self.frame is None:
                raise RuntimeError("H201 has no depth frame")
            if (time.monotonic() - self.timestamp) * 1000 > max_age_ms:
                raise TimeoutError("H201 depth frame is stale")
            return self.frame.copy()

    def disconnect(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        if self.thread is not None:
            self.thread.join(timeout=1)
        if self.read_fd is not None:
            try:
                os.close(self.read_fd)
            except OSError:
                pass
        self.process = None
        self.thread = None
        self.read_fd = None
        self.ready.clear()


def _panel(frame: np.ndarray | None, label: str) -> np.ndarray:
    if frame is None:
        panel = np.full((PANEL_SIZE[1], PANEL_SIZE[0], 3), 30, dtype=np.uint8)
        cv2.putText(panel, "NO FRAME", (105, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 80, 255), 2)
    else:
        image = np.asarray(frame)
        if image.dtype == np.uint16:
            depth = image.squeeze(-1) if image.ndim == 3 else image
            valid = depth > 0
            scaled = np.clip((depth.astype(np.float32) - 200) * (255 / 2800), 0, 255).astype(np.uint8)
            image = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
            image[~valid] = 0
        elif image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if image.shape[1] > image.shape[0] * 2:
            image = image[:, : image.shape[1] // 2]
        panel = cv2.resize(image, PANEL_SIZE, interpolation=cv2.INTER_AREA)
    cv2.rectangle(panel, (0, 0), (PANEL_SIZE[0], 38), (20, 20, 20), -1)
    cv2.putText(panel, label, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return panel


def compose_dashboard(observation: dict) -> np.ndarray:
    images = np.hstack([_panel(observation.get(key), label) for key, label in CAMERAS])
    footer = np.full((64, images.shape[1], 3), 24, dtype=np.uint8)
    cv2.putText(footer, f"STATUS: {STATUS}", (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (80, 230, 120), 2)
    cv2.putText(
        footer,
        "RIGHT: finish/save episode   LEFT: redo episode   ESC: save/stop",
        (12, 51),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (235, 235, 235),
        1,
    )
    return np.vstack((images, footer))


def _status_for_message(message: str) -> str:
    if message.startswith("Recording episode "):
        return f"RECORDING EPISODE {int(message.rsplit(' ', 1)[1]) + 1} (auto-save)"
    return {
        "Reset the environment": "RESETTING ENVIRONMENT",
        "Re-record episode": "DISCARDING AND RE-RECORDING EPISODE",
        "Stop recording": "FINALIZING / SAVING",
    }.get(message, message.upper())


def _apply_recording_key(key: int, events: dict) -> str | None:
    if key in RIGHT_KEYS:
        events["exit_early"] = True
        return "FINISHING EPISODE / RESET NEXT"
    if key in LEFT_KEYS:
        events["rerecord_episode"] = True
        events["exit_early"] = True
        return "DISCARDING AND RE-RECORDING EPISODE"
    if key in (27, ord("q")):
        events["stop_recording"] = True
        events["exit_early"] = True
        return "STOPPING / SAVING CURRENT EPISODE"
    return None


def init_visualization(*_args, **_kwargs) -> None:
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)


def log_visualization_data(_mode, *, observation: dict, **_kwargs) -> None:
    global STATUS
    cv2.imshow(WINDOW, compose_dashboard(observation))
    key = cv2.waitKeyEx(1)
    if RECORD_EVENTS is not None:
        status = _apply_recording_key(key, RECORD_EVENTS)
        if status is not None:
            STATUS = status
    elif key in (27, ord("q")):
        raise KeyboardInterrupt


def shutdown_visualization(*_args, **_kwargs) -> None:
    cv2.destroyAllWindows()


def main() -> int:
    if sys.argv[1:] == ["--self-test"]:
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        assert compose_dashboard({key: frame for key, _ in CAMERAS}).shape == (304, 960, 3)
        assert _status_for_message("Recording episode 0") == "RECORDING EPISODE 1 (auto-save)"
        assert _status_for_message("Stop recording") == "FINALIZING / SAVING"
        events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
        assert _apply_recording_key(2555904, events) == "FINISHING EPISODE / RESET NEXT"
        assert events["exit_early"] and not events["stop_recording"]
        events["exit_early"] = False
        assert _apply_recording_key(27, events) == "STOPPING / SAVING CURRENT EPISODE"
        assert events["exit_early"] and events["stop_recording"]
        return 0

    if len(sys.argv) < 2 or sys.argv[1] not in {"teleop", "record"}:
        raise SystemExit("usage: lerobot_cv_dashboard.py {teleop|record} [LeRobot arguments]")
    mode = sys.argv.pop(1)
    if mode == "teleop":
        import lerobot.scripts.lerobot_teleoperate as target
    else:
        import lerobot.scripts.lerobot_record as target

    make_robot = target.make_robot_from_config

    def make_robot_with_h201(config):
        robot = make_robot(config)
        camera = H201DepthCamera()
        robot.left_arm.cameras["top_h201"] = camera
        robot._top_level_cam_keys.update({"top_h201", "top_h201_depth"})
        robot.cameras["top_h201"] = camera
        return robot

    target.make_robot_from_config = make_robot_with_h201
    original_log_say = target.log_say

    def log_say_with_status(message, *args, **kwargs):
        global STATUS
        STATUS = _status_for_message(message)
        return original_log_say(message, *args, **kwargs)

    target.log_say = log_say_with_status
    if mode == "record":
        original_init_keyboard_listener = target.init_keyboard_listener

        def init_keyboard_listener_with_dashboard():
            global RECORD_EVENTS
            listener, RECORD_EVENTS = original_init_keyboard_listener()
            return listener, RECORD_EVENTS

        target.init_keyboard_listener = init_keyboard_listener_with_dashboard
    target.init_visualization = init_visualization
    target.log_visualization_data = log_visualization_data
    target.shutdown_visualization = shutdown_visualization
    target.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
