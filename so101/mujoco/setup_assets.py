#!/usr/bin/env python3
"""SO101 URDF와 STL 메쉬를 찾아 SHA256으로 검증하고 이 폴더에 연결한다.

이 저장소는 SO101 upstream 자산을 vendoring하지 않는다
(`so101/integrations/lerobot_v0_6_so101_mujoco/UPSTREAM_ASSETS.sha256` 참고).
16 MB짜리 STL 13개를 또 복사하는 대신, 이미 있는 체크아웃을 찾아
해시를 검증한 뒤 `meshes/` 심볼릭 링크와 `build/so101_new_calib.urdf`를 만든다.

검증에 쓰는 해시는 저장소가 이미 갖고 있는 위 매니페스트다. 즉
"교재가 준 STL"과 "LeRobot upstream STL"이 같은 파일인지도 같이 확인된다.
"""

from __future__ import annotations

import hashlib
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
# 저장소에서는 integrations/ 아래 매니페스트를 쓰고, 배포용 압축본에서는
# 같은 파일을 옆에 복사해 넣는다.
_MANIFEST_CANDIDATES = [
    os.path.join(HERE, "UPSTREAM_ASSETS.sha256"),
    os.path.join(REPO, "so101", "integrations", "lerobot_v0_6_so101_mujoco",
                 "UPSTREAM_ASSETS.sha256"),
]
MANIFEST = next((p for p in _MANIFEST_CANDIDATES if os.path.exists(p)),
                _MANIFEST_CANDIDATES[-1])
BUILD = os.path.join(HERE, "build")
MESH_LINK = os.path.join(HERE, "meshes")
URDF_OUT = os.path.join(BUILD, "so101_new_calib.urdf")

# URDF와 메쉬가 있을 만한 곳. 먼저 찾는 것을 쓴다.
URDF_CANDIDATES = [
    # 배포용 압축본: URDF와 meshes/가 이 폴더에 그대로 들어 있다.
    os.path.join(HERE, "so101_new_calib.urdf"),
    os.path.join(REPO, "so101_imitation_learning", "105_MUJOCO_basic",
                 "109_so101_mujoco_load", "so101_new_calib.urdf"),
    os.path.expanduser("~/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.urdf"),
]

# 교재 URDF의 해시. upstream 매니페스트에는 MJCF만 있고 URDF는 없어서 여기 적어둔다.
URDF_SHA256 = "1c7fde5d808c155441c3c832bd01b66cd3425397119d266a004d2360c0012fce"


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_manifest() -> dict[str, str]:  # noqa: D401
    """upstream 매니페스트에서 STL 파일명 -> sha256 맵을 만든다."""
    wanted = {}
    with open(MANIFEST, encoding="utf-8") as handle:
        for line in handle:
            digest, _, path = line.strip().partition("  ")
            if path.endswith(".stl"):
                wanted[os.path.basename(path)] = digest
    return wanted


def find_urdf() -> str:
    for path in URDF_CANDIDATES:
        if os.path.exists(path):
            return path
    raise SystemExit(
        "SO101 URDF를 찾지 못했다. 아래 중 한 곳에 있어야 한다:\n  "
        + "\n  ".join(URDF_CANDIDATES)
    )


def verify_meshes(mesh_dir: str, wanted: dict[str, str]) -> None:
    missing, bad = [], []
    for name, digest in sorted(wanted.items()):
        path = os.path.join(mesh_dir, name)
        if not os.path.exists(path):
            missing.append(name)
        elif sha256(path) != digest:
            bad.append(name)
    if missing or bad:
        for name in missing:
            print(f"  [없음]   {name}")
        for name in bad:
            print(f"  [불일치] {name}")
        raise SystemExit("메쉬 검증 실패. upstream 체크아웃을 확인할 것.")
    print(f"[2/3] STL {len(wanted)}개 SHA256 검증 통과 "
          f"(upstream 매니페스트와 완전 일치)")


def main() -> None:
    os.makedirs(BUILD, exist_ok=True)
    urdf = find_urdf()

    # meshes/가 이미 실물 폴더면(배포용 압축본) 그대로 쓰고, 심볼릭 링크를
    # 새로 만들지 않는다. 저장소에서는 upstream 체크아웃 옆의 폴더를 가리킨다.
    bundled = os.path.isdir(MESH_LINK) and not os.path.islink(MESH_LINK)
    if bundled:
        mesh_dir = MESH_LINK
    else:
        mesh_dir = os.path.join(os.path.dirname(urdf), "meshes")
        if not os.path.isdir(mesh_dir):
            mesh_dir = os.path.join(os.path.dirname(urdf), "assets")

    digest = sha256(urdf)
    print(f"[1/3] URDF: {urdf}")
    print(f"      메쉬:  {mesh_dir}")
    if digest != URDF_SHA256:
        raise SystemExit(
            f"URDF 해시가 다르다.\n  기대: {URDF_SHA256}\n  실제: {digest}\n"
            "다른 리비전의 SO101 URDF다. 관절 범위와 링크 오프셋이 달라질 수 있으니 "
            "README의 수치를 다시 측정할 것."
        )
    print(f"      URDF SHA256 검증 통과")
    verify_meshes(mesh_dir, load_manifest())

    # meshdir은 URDF 파일 위치 기준이다. 원본은 meshes/와 같은 폴더에 있지만
    # 사본은 build/ 안에 놓이므로 한 단계 위를 보게 고쳐 준다. (실습 3과 같은 함정)
    with open(urdf, encoding="utf-8") as handle:
        text = handle.read()
    text = text.replace('meshdir="meshes/"', 'meshdir="../meshes/"')
    with open(URDF_OUT, "w", encoding="utf-8") as handle:
        handle.write(text)
    if bundled:
        print(f"[3/3] 동봉된 meshes/ 사용 (심볼릭 링크 만들지 않음)")
        print(f"      build/so101_new_calib.urdf 복사 완료")
        return
    if os.path.islink(MESH_LINK):
        os.remove(MESH_LINK)
    os.symlink(os.path.relpath(mesh_dir, HERE), MESH_LINK)
    print(f"[3/3] meshes -> {os.readlink(MESH_LINK)}")
    print(f"      build/so101_new_calib.urdf 복사 완료")


if __name__ == "__main__":
    main()
