# Dual SO-101 leader imitation learning 실행 순서

실행 파일은 [`../scripts/dual_so101_il`](../scripts/dual_so101_il) 하나다. LeRobot 0.6의
`bi_so_leader`, `bi_so_follower`, `lerobot-record`, `lerobot-train --policy.type=act`를
그대로 호출한다. DAPIER가 별도 teleoperation framework를 다시 구현하지 않는다.

## 장치 정본

- left/right follower: 로컬 프로필의 `/dev/serial/by-id/...`
- left/right leader: `identify-leaders`로 물리 좌·우를 확인한 뒤 로컬 프로필에만 저장
- left wrist RGB: `/dev/dapier/left_wrist_rgb`, 320×240 YUYV 30 fps
- top H201 depth: eYs3D 공식 SDK `zdDepthVec`, 640×460 `uint16 mm`, 15 fps
- right wrist RGB: `/dev/dapier/right_wrist_rgb`, 320×240 YUYV 30 fps

H201은 OpenCV의 `/dev/video*` RGB/IR 프레임을 depth라고 부르지 않는다. DAPIER SDK helper가
eYs3D 공식 `zdDepthVec`를 직접 읽어 invalid sentinel `16384`를 `0`으로 바꾸고, 원본 거리값을
`observation.images.top_h201_depth`에 무손실 `uint16 mm`로 저장한다. 화면에서만 컬러맵으로
보인다. ACT는 양 손목 RGB와 12축 상태·행동을 학습하고, 이 raw depth는 박스 3D pose와 IK
목표 계산에 사용한다.

## 1. 로컬 프로필 생성

```bash
cd /home/dapier-jhj/DAPIER-vision-relative-manipulation-02
2ARM_ROBOT/scripts/dual_so101_il setup
```

생성 파일:

```text
2ARM_ROBOT/config/dual_so101_il.local.env
```

이 파일은 Git에서 제외된다.
네 팔의 현재 `/dev/serial/by-id/...` 경로를 follower/leader 항목에 넣는다.
물리 장비에도 같은 역할과 serial 끝 4자리를 `LF-xxxx`, `RF-xxxx`, `LL-xxxx`,
`RL-xxxx` 형식으로 표시한다. USB hub 포트나 `ttyACM*` 번호는 장치 정본으로 쓰지 않는다.

## 2. leader 좌·우 식별

```bash
2ARM_ROBOT/scripts/dual_so101_il identify-leaders
```

확인 문자열을 입력한 직후 5초 동안 **왼쪽 leader만** 여러 관절로 움직인다. 출력에서
`max_joint_span`이 큰 시리얼이 왼쪽 leader다. 그 결과를 로컬 프로필의
`LEFT_LEADER_PORT`, `RIGHT_LEADER_PORT`에 `/dev/serial/by-id/...` 전체 경로로 넣는다.

## 3. 장치와 calibration 상태 확인

대시보드는 `q`로 먼저 닫아 카메라를 해제한 뒤 실행한다.

```bash
2ARM_ROBOT/scripts/dual_so101_il status
```

네 serial과 세 camera가 `OK`여야 한다. 기존 단일팔
`so101_follower_main.json`/`so101_leader_main.json`은 새 양팔 ID로 간주하지 않는다.
연결할 때마다 calibration을 다시 하지 않는다. `teleop`과 `record`는 네 side-specific
calibration 중 하나라도 없으면 LeRobot의 자동 calibration을 시작하지 않고 즉시 중단한다.
저장 파일을 복구할 수 없거나 기구 조립·모터 교체·영점 변경이 있었을 때만 명시적으로
`calibrate-leaders` 또는 `calibrate-followers`를 실행한다.

calibration 정본은 지워질 수 있는 `~/.cache`가 아니라 로컬 프로필의
`CALIBRATION_DIR`(기본값 `~/.config/dapier/lerobot-calibration`)에 보관한다. 2026-09-04에는
두 follower EEPROM에 이미 저장된 offset·관절 범위를 LeRobot 공식 decoder로 읽어
side-specific JSON을 복구했고, 양쪽 모두 JSON과 EEPROM의 완전 일치를 확인했다. 이 복구는
토크·목표 위치·EEPROM을 쓰거나 팔을 다시 움직여 범위를 측정하지 않았다.

## 4. 양쪽 leader calibration

```bash
2ARM_ROBOT/scripts/dual_so101_il calibrate-leaders
```

화면 지시에 따라 각 leader의 중앙 자세와 전 관절 가동 범위를 기록한다.

## 5. 양쪽 follower calibration

양팔 ID의 calibration이 `MISSING`일 때만 실행한다.

```bash
2ARM_ROBOT/scripts/dual_so101_il calibrate-followers
```

## 6. 녹화 전 teleop 시험

 follower 두 팔 주위의 사람과 물체를 치우고 전원 차단 수단을 잡은 상태에서 실행한다.

```bash
2ARM_ROBOT/scripts/dual_so101_il teleop
```

leader를 아주 작게 움직여 물리 좌·우, 관절 방향, gripper 방향을 확인한다. 화면은 Rerun 없이
동일 observation을 `LEFT WRIST | TOP H201 DEPTH | RIGHT WRIST` 순서로 표시한다. wrapper는
한 control tick의 follower 변화량을 5도로 제한한다. 어느 축이 반대로 움직이거나 다른
팔이 반응하면 즉시 종료하고 calibration/leader mapping을 수정한다.

## 7. demonstration 녹화

```bash
2ARM_ROBOT/scripts/dual_so101_il record
```

기본 episode는 30초, reset 15초, 20회, 15 Hz다. 화면과 저장은 같은 observation을 사용하므로
카메라를 두 번 열지 않는다. 성공 시연만 남긴다. 동작 순서는
오른팔로 박스 날개 접근·열기 → 왼팔로 박스 내부 접근·신발 들어올리기다. base 이동은
이 dataset의 action에 포함하지 않는다.

녹화를 실행하면 별도 저장 버튼 없이 `30초 시연 → 15초 환경 복원 → episode 자동 저장 →
다음 episode 시작`을 20회 반복한다. 환경 복원 구간에는 박스·물체·leader/follower를 다음
시연의 시작 상태로 되돌린다. 카메라 창의 디스켓 아이콘은 dataset 저장이 아니라 화면 캡처다.

| 카메라 창 키 | 동작 |
| --- | --- |
| `→` | 현재 시연을 30초 전에 끝내고 복원·저장 단계로 이동 |
| `←` | 현재 시연을 버리고 복원 후 같은 episode 재녹화 |
| `Esc` 또는 `q` | 현재 시연을 저장하고 전체 녹화 정상 종료 |

창을 강제로 닫거나 터미널을 종료하지 않는다. 하단 `STATUS`가 `RECORDING EPISODE N`,
`RESETTING ENVIRONMENT`, `FINALIZING / SAVING` 중 무엇인지 확인한다.

## 8. ACT 학습

```bash
2ARM_ROBOT/scripts/dual_so101_il train
```

기본값은 CUDA, batch 8, 20,000 step이다. 결과는
`/home/dapier-jhj/dapier_training/dual-so101-box-shoe-act`에 저장된다. 학습 완료는 실물
성공 판정이 아니다. held-out loss 확인 후 MuJoCo replay, IK/충돌 gate, 저속 실물 rollout을
별도로 통과해야 한다.

2026-09-04 실물 smoke에서 150 frame/15 Hz episode를 다시 로드해 state/action 12D, 좌·우
RGB `(3,240,320)`, H201 depth `(1,460,640)`와 `depth_unit=mm`를 확인했다. 기본 ResNet18
ACT는 1채널 depth를 직접 받지 못하므로 launcher가 raw depth를 보존하면서 ACT 입력을 양 손목
RGB+state로 제한한다. 같은 dataset으로 CUDA ACT 1-step forward/backward/checkpoint 저장까지
통과했다.
