# SO-101 interactive sim 조작 수정 기록

`record_id: DAPIER-2026-08-07-so101-interactive-controls`

## 오늘 확인하려던 것

MuJoCo 창에서 `3`을 누르면 로봇 일부가 사라지고, 팔을 cube 쪽으로 어떻게
돌려야 할지 알기 어렵고, 키를 누르는 동안 계속 움직이지 않는 문제를 직접
재현했다. 오늘은 하드웨어나 ROS 2 real backend를 열지 않고 LeRobot SO-101
MuJoCo viewer만 수정하고 있다.

## 직접 확인한 원인

처음에는 숫자키만 피하면 된다고 생각했지만 충분하지 않았다. 설치된 MuJoCo의
shortcut table을 읽어 보니 `0..4`는 geom group 표시를 바꾸고, 평문자도
`W=wireframe`, `A=auto-connect`, `S=shadow`, `F=contact force`,
`P=contact split`, `Q=camera` 등으로 이미 예약돼 있었다. 실제 viewer에서
plain `A`를 보내면 auto-connect flag가 바뀌고, `Shift+A`는 바뀌지 않는 것을
확인했다.

기존 cube도 로봇 기준 뒤쪽 `(-0.22, -0.10) m`에 있어 shoulder pan 범위를
벗어났다. home pose의 gripper는 대략 `x=+0.241 m`에 있는데 cube 방향은 약
`-159 deg`이고 shoulder pan limit는 약 `±110 deg`라서, 키 설명만 고쳐서는
집을 수 없는 장면이었다.

## 오늘 바꾼 것

- cube를 G1에서 검증한 reachable 위치
  `(0.254531, -0.002931, 0.075) m`에 놓고 raised support를 추가했다.
- green goal tray는 별도 reachable 위치에 남겨 lift와 place 목표를 섞지 않았다.
- G1과 같은 fixed/moving finger pad 두 개를 runtime model에 추가했다.
- damped-least-squares IK로 gripper 끝점을 world XYZ에서 조금씩 움직이는
  `CartesianJogController`를 추가했다.
- X11의 실제 key-down state를 30 Hz로 읽어 desktop key repeat와 무관하게
  key hold가 계속 적용되게 했다.
- 모든 로봇 명령은 `Shift+키`일 때만 처리해 MuJoCo plain-letter shortcut과
  분리했다.
- viewer 종료 전에 render thread가 끝날 때까지 기다려 짧은 GUI smoke test의
  종료 `segmentation fault`를 없앴다.

현재 viewer 조작은 다음과 같다.

| 입력 | 동작 |
|---|---|
| hold `Shift+W/S` | world X `+/-` |
| hold `Shift+A/D` | world Y `+/-` |
| hold `Shift+R/F` | world Z `+/-` |
| hold `Shift+O/L` | gripper open/close |
| `Shift+J/K`, hold `Shift+Up/Down` | 관절 선택, 선택 관절 연속 이동 |
| `Shift+G` | 검증된 open-gripper cube approach target |
| `Shift+P` | 300-frame padded pick-and-lift 재생 |
| `Shift+V` | wrist RGB로 cube 검출 후 pick-and-place |
| `Shift+C` | external/wrist viewer camera 전환 |
| `Shift+H`, `Shift+N`, `Shift+Q` | clear home, 새 episode, 종료 |

## 직접 실행해 본 검증

| 검증 | 결과 |
|---|---:|
| XYZ 10 mm IK 오차 | X `0.220 mm`, Y `0.026 mm`, Z `0.079 mm` |
| hold 입력 단위 검증 | 0.5 s X 요청 `40 mm`, target 이동 `38.29 mm` |
| key release 뒤 target drift | `0.0 mm` |
| 실제 GUI `Shift+W` 0.5 s | gripper X 약 `39 mm` 이동 |
| 전체 Shift chord shortcut 격리 | vis/render/geom flag 변화 각각 `0` |
| scripted physics | 300 frame, 30 Hz, 5000 substep, 10.0 s |
| final 30-frame hold | 최소 lift `40.90 mm`, bilateral pad `30/30`, support `0/30` |
| 실제 GUI `Shift+P` | cube final z `0.111 m`, exit code `0` |
| clean LeRobot v0.6.0 재구성 | upstream asset SHA-256 `14/14` |
| 재구성 환경 test | `15/15 PASS`, 기존 Gymnasium warning 1건 |

다음 명령으로 sim test를 다시 실행했다.

```bash
MUJOCO_GL=egl "$HOME/so101/lerobot/.venv/bin/python" -m pytest -q \
  "$HOME/so101/lerobot/tests/envs/test_so101_mujoco.py"
```

## 아직 확인하지 못한 부분

현재 scripted controller의 공식 G1 판정 구간은 frame `270..299`다. frame 299
뒤 동일 target으로 물리를 계속 진행해 보니 cube가 약 8 frame 안에 빠졌다.
그래서 interactive viewer는 `Shift+P`가 끝나면 검증된 frame 299에서 물리를
명시적으로 pause한다. 화면에서 계속 들고 있는 것처럼 보여도 장기 물리 hold
성공으로 기록하지 않는다. 수동 이동 chord를 누르면 physics가 다시 진행된다.

아직 place trajectory, 반복 seed 성공률, Wayland native 입력, learned policy,
실물 pad 마찰과 sim-to-real은 확인하지 않았다. 다음에는 pause 없이 장기 hold가
되는 접촉 geometry와 gripper controller를 별도 실험으로 확인할 예정이다.

## 같은 날 이어서 확인한 것

위 미확인 목록은 interactive control을 처음 검증한 시점의 상태다. 이어서 wrist
RGB camera, cube color detection과 green tray place trajectory를 추가했고,
`+-25 mm` randomization의 seed `0..29`에서 `30/30 PASS`를 확인했다. 이 후속
실험의 조건과 한계는
[`2026-08-07-so101-wrist-vision-pick-place.md`](2026-08-07-so101-wrist-vision-pick-place.md)에
분리해 기록했다. Wayland, learned policy와 sim-to-real은 여전히 확인하지 않았다.
