#!/usr/bin/env python3
"""ImageFolder 데이터셋의 수량, 손상, 해상도, 완전 중복을 검사한다."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from PIL import Image, UnidentifiedImageError
from torchvision.datasets import ImageFolder

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp", ".bmp"}


def inspect_dataset(root: Path, target: int, expected_classes: int) -> dict[str, Any]:
    class_dirs = sorted(
        path for path in root.iterdir() if path.is_dir() and not path.name.startswith(".")
    )
    counts: dict[str, int] = {}
    invalid: list[str] = []
    too_small: list[str] = []
    hashes: dict[str, list[str]] = {}

    for class_dir in class_dirs:
        images = sorted(
            path
            for path in class_dir.iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        )
        counts[class_dir.name] = len(images)
        for path in images:
            try:
                with Image.open(path) as image:
                    image.verify()
                with Image.open(path) as image:
                    if image.width < 64 or image.height < 64:
                        too_small.append(str(path))
                digest = hashlib.sha256(path.read_bytes()).hexdigest()
                hashes.setdefault(digest, []).append(str(path))
            except (OSError, UnidentifiedImageError):
                invalid.append(str(path))

    duplicates = [paths for paths in hashes.values() if len(paths) > 1]
    total = sum(counts.values())
    exact_target = bool(counts) and all(count == target for count in counts.values())
    valid = (
        len(class_dirs) == expected_classes
        and exact_target
        and not invalid
        and not too_small
        and not duplicates
    )

    imagefolder_classes: list[str] = []
    imagefolder_total = 0
    imagefolder_error = ""
    try:
        dataset = ImageFolder(root)
        imagefolder_classes = dataset.classes
        imagefolder_total = len(dataset)
    except (FileNotFoundError, RuntimeError) as error:
        imagefolder_error = str(error)
        valid = False

    return {
        "valid": valid,
        "root": str(root),
        "expected_classes": expected_classes,
        "target_per_class": target,
        "class_counts": counts,
        "total": total,
        "invalid_files": invalid,
        "too_small_files": too_small,
        "exact_duplicate_groups": duplicates,
        "imagefolder_classes": imagefolder_classes,
        "imagefolder_total": imagefolder_total,
        "imagefolder_error": imagefolder_error,
    }


def print_report(report: dict[str, Any]) -> None:
    print(f"데이터셋: {report['root']}")
    print(f"클래스: {len(report['class_counts'])} / {report['expected_classes']}")
    for name, count in report["class_counts"].items():
        mark = "OK" if count == report["target_per_class"] else "CHECK"
        print(f"  [{mark}] {name}: {count} / {report['target_per_class']}")
    print(f"전체 이미지: {report['total']}")
    print(f"손상 파일: {len(report['invalid_files'])}")
    print(f"64×64 미만: {len(report['too_small_files'])}")
    print(f"완전 중복 그룹: {len(report['exact_duplicate_groups'])}")
    print(f"ImageFolder: {report['imagefolder_classes']} ({report['imagefolder_total']}장)")
    print("결과:", "PASS" if report["valid"] else "FAIL")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ResNet ImageFolder 무결성 검사")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--target", type=int, default=1000)
    parser.add_argument("--expected-classes", type=int, default=4)
    parser.add_argument("--json", type=Path, help="JSON 결과 저장 경로")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.dataset.is_dir():
        print(f"데이터셋 폴더가 없습니다: {args.dataset}")
        return 2
    report = inspect_dataset(args.dataset.resolve(), args.target, args.expected_classes)
    print_report(report)
    if args.json:
        args.json.write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
