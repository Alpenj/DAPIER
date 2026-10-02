#!/usr/bin/env python3
"""Puppy URDF와 STL 메쉬를 찾아 SHA256으로 검증하고 이 폴더에 연결한다.

실습 5(so101)와 같은 방식이다. 9 MB짜리 STL 11개를 저장소에 또 복사하는 대신
교재 체크아웃을 찾아 `UPSTREAM_ASSETS.sha256`으로 검증한 뒤 `meshes/`
심볼릭 링크와 `build/puppy2.urdf` 사본을 만든다.

배포용 압축본처럼 `meshes/`와 `puppy2.urdf`가 이 폴더에 실물로 들어 있으면
그것을 그대로 쓴다.
"""

from __future__ import annotations

import hashlib
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
MANIFEST = os.path.join(HERE, "UPSTREAM_ASSETS.sha256")
BUILD = os.path.join(HERE, "build")
MESH_LINK = os.path.join(HERE, "meshes")
URDF_OUT = os.path.join(BUILD, "puppy2.urdf")

URDF_CANDIDATES = [
    # 배포용 압축본
    os.path.join(HERE, "puppy2.urdf"),
    # 교재 체크아웃
    os.path.join(REPO, "so101_imitation_learning", "105_MUJOCO_basic",
                 "108_puppypi_mujoco_load", "puppy2.urdf"),
]


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_manifest() -> dict[str, str]:
    wanted = {}
    with open(MANIFEST, encoding="utf-8") as handle:
        for line in handle:
            digest, _, path = line.strip().partition("  ")
            if path:
                wanted[path] = digest
    return wanted


def find_urdf() -> str:
    for path in URDF_CANDIDATES:
        if os.path.exists(path):
            return path
    raise SystemExit(
        "Puppy URDF를 찾지 못했다. 아래 중 한 곳에 있어야 한다:\n  "
        + "\n  ".join(URDF_CANDIDATES)
    )


def main() -> None:
    os.makedirs(BUILD, exist_ok=True)
    urdf = find_urdf()
    wanted = load_manifest()

    bundled = os.path.isdir(MESH_LINK) and not os.path.islink(MESH_LINK)
    mesh_dir = MESH_LINK if bundled else os.path.join(os.path.dirname(urdf), "meshes")

    print(f"[1/3] URDF: {urdf}")
    print(f"      메쉬:  {mesh_dir}")

    problems = []
    for rel, digest in sorted(wanted.items()):
        path = urdf if rel == "puppy2.urdf" else os.path.join(mesh_dir, os.path.basename(rel))
        if not os.path.exists(path):
            problems.append(f"  [없음]   {rel}")
        elif sha256(path) != digest:
            problems.append(f"  [불일치] {rel}")
    if problems:
        print("\n".join(problems))
        raise SystemExit("자산 검증 실패. 교재 체크아웃을 확인할 것.")
    print(f"[2/3] URDF 1개 + STL {len(wanted) - 1}개 SHA256 검증 통과")

    # meshdir은 URDF 파일 위치 기준이다. 사본이 build/ 안에 놓이므로 한 단계 위를
    # 보게 <mujoco> 블록을 넣어 준다. 원본 URDF에는 이 블록이 없다.
    with open(urdf, encoding="utf-8") as handle:
        text = handle.read()
    if "<mujoco>" not in text:
        text = text.replace(
            "<robot name=\"puppy\">",
            "<robot name=\"puppy\">\n  <mujoco>\n"
            "    <compiler meshdir=\"../meshes/\" strippath=\"true\" balanceinertia=\"true\"\n"
            "              fusestatic=\"false\" discardvisual=\"false\"/>\n"
            "  </mujoco>",
            1,
        )
    with open(URDF_OUT, "w", encoding="utf-8") as handle:
        handle.write(text)

    if bundled:
        print("[3/3] 동봉된 meshes/ 사용 (심볼릭 링크 만들지 않음)")
    else:
        if os.path.islink(MESH_LINK):
            os.remove(MESH_LINK)
        os.symlink(os.path.relpath(mesh_dir, HERE), MESH_LINK)
        print(f"[3/3] meshes -> {os.readlink(MESH_LINK)}")
    print("      build/puppy2.urdf 생성 완료")


if __name__ == "__main__":
    main()
