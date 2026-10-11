"""안정화 MiniVGG 개인화 체크포인트를 사용하는 실시간 웹캠 추론."""

# ruff: noqa: I001 -- conda 재실행이 OpenCV/PyTorch import보다 먼저여야 한다.

import argparse
import os
import sys
import time
from pathlib import Path


CONDA_PYTHON = Path.home() / "miniconda3/envs/lerobot-vision/bin/python"
if CONDA_PYTHON.exists() and Path(sys.executable).resolve() != CONDA_PYTHON.resolve():
    os.execv(str(CONDA_PYTHON), [str(CONDA_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])

import cv2
import numpy as np
import torch
from PIL import Image

from minivgg_common import build_eval_transform, load_stable_checkpoint


SCRIPT_DIR = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=SCRIPT_DIR / "miniVGGnet_stable.pth")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--threshold", type=float, default=0.65)
    parser.add_argument("--margin", type=float, default=0.12)
    parser.add_argument(
        "--ema",
        type=float,
        default=0.75,
        help="이전 프레임 확률의 비중(0=평활화 없음, 1에 가까울수록 안정적)",
    )
    parser.add_argument("--check-only", action="store_true", help="카메라 없이 모델 로드와 1회 추론만 검증")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not 0.0 <= args.ema < 1.0:
        raise ValueError("--ema는 0 이상 1 미만이어야 합니다.")
    if not 0.0 <= args.threshold <= 1.0:
        raise ValueError("--threshold는 0~1이어야 합니다.")
    if not 0.0 <= args.margin <= 1.0:
        raise ValueError("--margin은 0~1이어야 합니다.")


def main() -> None:
    args = parse_args()
    validate_args(args)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model, class_names, checkpoint = load_stable_checkpoint(args.model, device)
    transform = build_eval_transform(int(checkpoint.get("image_size", 64)))
    print(
        f"모델 로드 완료: {args.model}\n"
        f"device={device} classes={class_names} seed={checkpoint.get('seed')} "
        f"validation_accuracy={checkpoint.get('best_val_accuracy', 0.0) * 100:.2f}%"
    )

    if args.check_only:
        dummy = torch.zeros(1, 3, 64, 64, device=device)
        with torch.inference_mode():
            probabilities = model(dummy).softmax(dim=1)
        print(f"check-only 성공: output_shape={tuple(probabilities.shape)} sum={probabilities.sum().item():.4f}")
        return

    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        raise RuntimeError(
            f"카메라 {args.camera}를 열 수 없습니다. --camera 1처럼 다른 번호를 시도하세요."
        )

    ema_probabilities: np.ndarray | None = None
    previous_time = time.perf_counter()
    print("라이브캠 시작: 물체를 중앙 사각형 안에 놓고 q 또는 ESC로 종료하세요.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("카메라 프레임을 읽지 못했습니다.")

            height, width = frame.shape[:2]
            roi_size = min(360, int(min(height, width) * 0.70))
            x1 = (width - roi_size) // 2
            y1 = (height - roi_size) // 2
            x2, y2 = x1 + roi_size, y1 + roi_size
            roi = frame[y1:y2, x1:x2]

            rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
            tensor = transform(Image.fromarray(rgb)).unsqueeze(0).to(device)
            with torch.inference_mode():
                current = model(tensor).softmax(dim=1)[0].cpu().numpy()

            if ema_probabilities is None:
                ema_probabilities = current
            else:
                ema_probabilities = args.ema * ema_probabilities + (1.0 - args.ema) * current

            order = np.argsort(ema_probabilities)[::-1]
            top_index, second_index = int(order[0]), int(order[1])
            confidence = float(ema_probabilities[top_index])
            confidence_margin = confidence - float(ema_probabilities[second_index])
            confident = confidence >= args.threshold and confidence_margin >= args.margin
            label = class_names[top_index].upper() if confident else "UNCERTAIN"
            color = (0, 200, 0) if confident else (0, 165, 255)

            now = time.perf_counter()
            fps = 1.0 / max(now - previous_time, 1e-6)
            previous_time = now
            display = f"{label} {confidence * 100:.1f}%  FPS {fps:.1f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame,
                display,
                (x1, max(y1 - 12, 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                color,
                2,
            )
            cv2.putText(
                frame,
                "q / ESC: quit",
                (12, height - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
            )
            cv2.imshow("MiniVGG Stable Live Cam", frame)
            cv2.imshow("Model ROI", cv2.resize(roi, (192, 192)))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
