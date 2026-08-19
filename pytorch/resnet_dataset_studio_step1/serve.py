#!/usr/bin/env python3
"""로컬 전용 데이터 수집 웹앱 및 이미지 저장 API 서버."""

from __future__ import annotations

import argparse
import functools
import http.server
import json
import re
import threading
import time
import urllib.parse
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp"}
CLASS_NAME_PATTERN = re.compile(r"^[\w-]{1,50}$", re.UNICODE)
MAX_IMAGE_BYTES = 5 * 1024 * 1024


class DatasetServer(http.server.ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], output_root: Path) -> None:
        handler = functools.partial(AppHandler, directory=str(ROOT))
        super().__init__(server_address, handler)
        self.output_root = output_root
        self.capture_lock = threading.Lock()


class AppHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Pragma", "no-cache")
        super().end_headers()

    @property
    def dataset_server(self) -> DatasetServer:
        return self.server  # type: ignore[return-value]

    def send_json(self, status: int, payload: dict[str, object]) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def valid_class_name(self, name: str) -> bool:
        return bool(CLASS_NAME_PATTERN.fullmatch(name))

    def image_count(self, class_name: str) -> int:
        class_dir = self.dataset_server.output_root / class_name
        if not class_dir.is_dir():
            return 0
        return sum(
            path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
            for path in class_dir.iterdir()
        )

    def parse_json_body(self) -> dict[str, object]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 100_000:
            raise ValueError("잘못된 요청 크기")
        payload = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("JSON 객체가 필요합니다.")
        return payload

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        try:
            relative_output = self.dataset_server.output_root.relative_to(ROOT)
            protected_prefix = "/" + relative_output.as_posix().rstrip("/") + "/"
            if parsed.path.startswith(protected_prefix):
                self.send_json(404, {"error": "수집 이미지는 HTTP로 제공하지 않습니다."})
                return
        except ValueError:
            pass
        if parsed.path == "/api/config":
            self.send_json(
                200,
                {
                    "output": str(self.dataset_server.output_root),
                    "max_image_bytes": MAX_IMAGE_BYTES,
                },
            )
            return
        if parsed.path == "/api/state":
            query = urllib.parse.parse_qs(parsed.query)
            classes = [name for name in query.get("classes", [""])[0].split(",") if name]
            if not classes or any(not self.valid_class_name(name) for name in classes):
                self.send_json(400, {"error": "유효한 클래스 이름이 필요합니다."})
                return
            counts = {name: self.image_count(name) for name in classes}
            self.send_json(200, {"counts": counts})
            return
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        try:
            if parsed.path == "/api/classes":
                self.handle_classes()
            elif parsed.path == "/api/rename":
                self.handle_rename()
            elif parsed.path == "/api/capture":
                self.handle_capture(parsed.query)
            elif parsed.path == "/api/metadata":
                self.handle_metadata()
            else:
                self.send_json(404, {"error": "API를 찾을 수 없습니다."})
        except (ValueError, json.JSONDecodeError) as error:
            self.send_json(400, {"error": str(error)})
        except OSError as error:
            self.send_json(500, {"error": f"파일 저장 실패: {error}"})

    def handle_classes(self) -> None:
        payload = self.parse_json_body()
        classes = payload.get("classes")
        if (
            not isinstance(classes, list)
            or len(classes) != 4
            or len(set(classes)) != 4
            or any(not isinstance(name, str) or not self.valid_class_name(name) for name in classes)
        ):
            raise ValueError("서로 다른 클래스 이름 4개가 필요합니다.")
        for name in classes:
            (self.dataset_server.output_root / name).mkdir(parents=True, exist_ok=True)
        counts = {name: self.image_count(name) for name in classes}
        self.send_json(200, {"counts": counts, "output": str(self.dataset_server.output_root)})

    def handle_rename(self) -> None:
        payload = self.parse_json_body()
        old = payload.get("old")
        new = payload.get("new")
        if not isinstance(old, str) or not isinstance(new, str):
            raise ValueError("이전/새 클래스 이름이 필요합니다.")
        if not self.valid_class_name(old) or not self.valid_class_name(new):
            raise ValueError("유효하지 않은 클래스 이름입니다.")
        old_dir = self.dataset_server.output_root / old
        new_dir = self.dataset_server.output_root / new
        if new_dir.exists() and any(new_dir.iterdir()):
            self.send_json(409, {"error": "새 클래스 폴더에 이미 파일이 있습니다."})
            return
        if new_dir.exists():
            new_dir.rmdir()
        if old_dir.exists():
            old_dir.rename(new_dir)
        else:
            new_dir.mkdir(parents=True, exist_ok=True)
        self.send_json(200, {"renamed": True})

    def handle_capture(self, query_string: str) -> None:
        query = urllib.parse.parse_qs(query_string)
        class_name = query.get("class", [""])[0]
        target = int(query.get("target", ["1000"])[0])
        if not self.valid_class_name(class_name):
            raise ValueError("유효하지 않은 클래스 이름입니다.")
        if not 1 <= target <= 10_000:
            raise ValueError("목표 수량은 1~10,000이어야 합니다.")
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > MAX_IMAGE_BYTES:
            raise ValueError("JPEG 크기가 허용 범위를 벗어났습니다.")
        image = self.rfile.read(length)
        if not image.startswith(b"\xff\xd8") or not image.endswith(b"\xff\xd9"):
            raise ValueError("올바른 JPEG 파일이 아닙니다.")

        with self.dataset_server.capture_lock:
            class_dir = self.dataset_server.output_root / class_name
            class_dir.mkdir(parents=True, exist_ok=True)
            current_count = self.image_count(class_name)
            if current_count >= target:
                self.send_json(409, {"error": "이 클래스는 목표 수량에 도달했습니다."})
                return
            next_count = current_count + 1
            filename = f"{class_name}_{next_count:04d}_{time.time_ns()}.jpg"
            output_path = class_dir / filename
            temporary_path = output_path.with_suffix(".tmp")
            temporary_path.write_bytes(image)
            temporary_path.replace(output_path)

        self.send_json(201, {"count": next_count, "filename": filename})

    def handle_metadata(self) -> None:
        payload = self.parse_json_body()
        classes = payload.get("classes")
        target = payload.get("target_per_class")
        if (
            not isinstance(classes, list)
            or len(classes) != 4
            or len(set(classes)) != 4
            or any(
                not isinstance(name, str) or not self.valid_class_name(name)
                for name in classes
            )
        ):
            raise ValueError("클래스 목록이 필요합니다.")
        if not isinstance(target, int) or not 1 <= target <= 10_000:
            raise ValueError("목표 수량이 필요합니다.")
        counts = {name: self.image_count(name) for name in classes}
        metadata = {
            "updated_at_unix": time.time(),
            "target_per_class": target,
            "image_size": [224, 224],
            "classes": counts,
        }
        path = self.dataset_server.output_root / "collection_state.json"
        path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        self.send_json(200, {"saved": True, "counts": counts})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ResNet Dataset Studio 로컬 서버")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--open", action="store_true", help="브라우저 자동 열기")
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "collected_images",
        help="수집 이미지 저장 루트",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_root = args.output.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    server = DatasetServer((args.host, args.port), output_root)
    url = f"http://{args.host}:{args.port}"
    print(f"ResNet Dataset Studio: {url}")
    print(f"이미지 저장 위치: {output_root}")
    print("종료: Ctrl+C")
    if args.open:
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n서버를 종료합니다.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
