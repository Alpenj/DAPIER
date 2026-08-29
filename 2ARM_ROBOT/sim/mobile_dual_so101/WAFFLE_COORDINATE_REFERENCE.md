# Waffle Pi 좌표 기준과 중앙 STEP 지지대 배치

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

## 분할 `assem_base.step` layout 기준 좌표

| 대상 | `base_link` 중심 `(X, Y, Z)` | 근거/상태 |
| --- | --- | --- |
| top datum | `(-64, 0, 91.5)` | 공식 URDF transform + 조립 STL |
| STEP bottom contact | `(-64, 0, 91.5)` | 바닥면을 공식 Waffle 상판에 접촉 |
| STEP base 외형 | `160 × 180 × 25` | STEP에서 읽은 bottom footprint |
| STEP bottom hole 4개 | `(-64+/-77, +/-87, 91.5)` | 반지름 3; Waffle M3 후보와 불일치 |
| lower print part | `160 × 180 × 170` | assembly local Z=0~170 |
| source upper print part | `110.963 × 254 × 156` | 원본 보존, Z=160~316 |
| upper socket print part | `110.963 × 254 × 156` | 원본 유지, Z=160~316 |
| central column top | Z=`391.5` | assembly local Z=300 |
| left upper socket axis | `(-64, +127, 394.050896)` | radius 8.5 |
| right upper socket axis | `(-64, -127, 394.050896)` | 좌우 대칭 |
| left SO-101 arm frame | `(-64, +93.4, 387.686186)` | socket axis offset 역산 |
| right SO-101 arm frame | `(-64, -93.4, 387.686186)` | socket axis offset 역산 |
| depth camera body center | `(-64, 0, 550)` | 전용 mast, 27도 하향 |

원본 STEP과 두 STL은 수정 없이 사용한다. upper 좌우 소켓이 stock SO-101의
`base_so101_v2`만 대체한다. `base_motor_holder_so101_v1`,
`waveshare_mounting_plate_so101_v2`, base servo와 shoulder 이후 joint chain은 유지하며,
원본 socket 큰 홀 축과 SO-101 base frame 변환을 맞춘다. 팔 frame 좌표를
외부 장착홀 간격으로 해석하면 안 된다. STEP bottom hole 네 개는
공식 Waffle 체결 후보와 직접 일치하지 않으므로 adapter plate와 fit coupon이
필요하다. 카메라 model/체결홀/optical origin과 지지대 재료·질량은 실측 뒤 CAD와
동역학 모델에서 교체한다.

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
- teammate `assem_base.step` SHA-256:
  `f9f77f71a77f962aac3c7a3898bf5df7232b12982be3f16e3e2fc20d39c1bb3b`
- lower STL SHA-256:
  `f91c58b14bd9932757787d9fea1f72104bae537d38b003d2a430576fecc70076`
- upper STL SHA-256:
  `ed6218f3ba83459fc7c436416ef708130e2622a27df6772d4d62bf6a9d822bf7`
- SO-101 `base_so101_v2.stl` SHA-256:
  `bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d`

출처:

- https://github.com/ROBOTIS-GIT/turtlebot3
- https://en.robotis.com/service/download.php?no=668
- https://en.robotis.com/service/download.php?no=669
- https://www.robotis.us/tb3-waffle-plate-ipl-01-8ea/

좌표 상수와 변환 함수는 [`waffle_reference.py`](waffle_reference.py), 회귀 테스트는
[`test/test_waffle_reference.py`](test/test_waffle_reference.py)에 있다.
