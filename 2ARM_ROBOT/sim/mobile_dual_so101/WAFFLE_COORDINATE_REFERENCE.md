# Waffle Pi 좌표 기준과 dual-tower 배치

이 문서는 ROBOTIS 공식 Jazzy URDF, 조립 STL과
`TB3_WAFFLE_PLATE-IPL-01` PDF/STEP을 기준으로 만든 제작 좌표표다. 단위는 별도
표기가 없으면 mm다. 사진 비례나 화면 눈대중으로 정한 값은 포함하지 않는다.

## `base_link` 축

공식 URDF에서 왼쪽 wheel은 `Y=+144`, 오른쪽 wheel은 `Y=-144`, 뒤 caster는
`X=-177`, 원래 전면 camera는 `X=+73`이다. 따라서 축은 다음과 같다.

```text
                    +Z 위
                     |
                     o------> +X 전방
                    /
                  +Y 좌측
```

- 원점: 좌우 wheel 축 중앙의 `base_link`
- `+X`: 로봇 전방
- `+Y`: 로봇 좌측
- `+Z`: 위
- 오른손 좌표계: `X × Y = Z`

MuJoCo에는 실제 `base_link`와 같은 방향을 가진 top-datum 표시를 넣었다. 주황색
원점은 `(-64, 0, 91.5)`, 빨강은 `+X`, 초록은 `+Y`, 파랑은 `+Z`다. site 이름은
`waffle_top_reference_origin`, `waffle_top_axis_x_forward`,
`waffle_top_axis_y_left`, `waffle_top_axis_z_up`이다.

## 공식 mesh에서 `base_link`로 변환

공식 URDF의 visual origin은 `(-0.064, 0, 0) m`이고 STL scale은 `0.001`이다.

```text
p_base_link[m] = 0.001 * p_mesh[mm] + (-0.064, 0, 0)
```

그래서 mesh `(64, 0, 0)`이 `base_link` 원점이고, top Waffle plate의 물리적
장착면은 `Z=91.5`다. URDF collision box 상단 `Z=94`는 단순 충돌 proxy이며
제작 장착면으로 사용하지 않는다.

공식 단품 STEP에서 Waffle plate 외형은 `128 × 64 × 9`다. compound M3 profile은
볼트 통과부와 nut 형상이 섞여 있으므로, 아래 값은 hole **중심 좌표**다. hole
지름을 새 출력물에 그대로 복사하기 전에는 fit coupon으로 확인한다.

## 선택한 deck 체결 중심

| mesh X | mesh Y | `base_link` X | `base_link` Y |
| ---: | ---: | ---: | ---: |
| -88 | -64 | -152 | -64 |
| -88 | +64 | -152 | +64 |
| 0 | -88 | -64 | -88 |
| 0 | +88 | -64 | +88 |
| +40 | -64 | -24 | -64 |
| +40 | +64 | -24 | +64 |

6개 중심은 공식 조립 STL의 최상단에서 열린 원형 피처로 추출했다. 실제 로봇에서
하부 nut 접근, 이미 사용 중인 support와 케이블 간섭은 조립 전에 다시 확인한다.

## tower layout 기준 좌표

| 대상 | `base_link` 중심 `(X, Y, Z)` | 근거/상태 |
| --- | --- | --- |
| top datum | `(-64, 0, 91.5)` | 공식 URDF transform + 조립 STL |
| common deck | `(-64, 0, 95.5)` | 바닥 Z=91.5, 두께 8 |
| deck 외형 | `192 × 256 × 8` | 6-hole pattern과 tower footprint 포함 |
| left tower/arm mount | `(-64, +100, 380)` | 좌측, camera 폭 여유 포함 |
| right tower/arm mount | `(-64, -100, 380)` | 우측 대칭 |
| depth camera | `(+25, 0, 450)` | 두 tower 정중앙, 외형은 아직 provisional |

팔 높이 380과 camera X/Z는 작업영역 및 시야 concept 값이다. Waffle 좌표에서
정확히 표현했지만 실물 치수 확정값은 아니다. 특히 camera model/체결홀/optical
origin과 팔 장착 flange는 실측 뒤 CAD revision에서 교체한다.

## 원본과 무결성

- TurtleBot3 source commit: `0c0be84e3f5c3194fb2adea8426a58a96060eab5`
- official Waffle Pi URDF SHA-256:
  `33c201d21492246e9eba8ecd7f1ca9ae4bd7c88a1da5bc360584002ba61ab9ce`
- official assembled base STL SHA-256:
  `706230121a287ff953c4c84d344014a147fc27615b0e95221aaf04cb0dca7e42`
- official plate PDF SHA-256:
  `f4568fa9e642e54998a2be0224c906b1dfec823d5bb9c10a8450833eb00e8105`
- official plate STEP SHA-256:
  `966d4e1a0236ef69674870bbb130ae7ce7fceb2168eb5eb7847e4b01c8a268d3`

출처:

- https://github.com/ROBOTIS-GIT/turtlebot3
- https://en.robotis.com/service/download.php?no=668
- https://en.robotis.com/service/download.php?no=669
- https://www.robotis.us/tb3-waffle-plate-ipl-01-8ea/

좌표 상수와 변환 함수는 [`waffle_reference.py`](waffle_reference.py), 회귀 테스트는
[`test/test_waffle_reference.py`](test/test_waffle_reference.py)에 있다.
