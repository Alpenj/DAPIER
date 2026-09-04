# Dual SO-101 leader imitation learning 실행 순서

실행 파일은 [`../scripts/dual_so101_il`](../scripts/dual_so101_il) 하나다. LeRobot 0.6의
`bi_so_leader`, `bi_so_follower`, `lerobot-record`, `lerobot-train --policy.type=act`를
그대로 호출한다. DAPIER가 별도 teleoperation framework를 다시 구현하지 않는다.

## 장치 정본

- left/right follower: 로컬 프로필의 `/dev/serial/by-id/...`
- left/right leader: `identify-leaders`로 물리 좌·우를 확인한 뒤 로컬 프로필에만 저장
- left wrist RGB: `/dev/dapier/left_wrist_rgb`, 320×240 YUYV 30 fps
- top H201 visual observation: `/dev/video6`, 1280×460 YUYV 15 fps
- right wrist RGB: `/dev/dapier/right_wrist_rgb`, 320×240 YUYV 30 fps

H201의 `/dev/video6`은 학습용 top-view 단안 영상이다. metric depth는 OpenCV 장치가
아니며 eYs3D SDK가 `/dev/video8` disparity를 변환해 제공한다. 따라서 첫 ACT 데이터에는
top mono + 양 손목 RGB를 기록하고, metric depth는 IK/박스 pose 계산 경로에 둔다.

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

leader를 아주 작게 움직여 물리 좌·우, 관절 방향, gripper 방향을 확인한다. wrapper는
한 control tick의 follower 변화량을 5도로 제한한다. 어느 축이 반대로 움직이거나 다른
팔이 반응하면 즉시 종료하고 calibration/leader mapping을 수정한다.

## 7. demonstration 녹화

```bash
2ARM_ROBOT/scripts/dual_so101_il record
```

기본 episode는 30초, reset 15초, 20회, 15 Hz다. 성공 시연만 남긴다. 동작 순서는
오른팔로 박스 날개 접근·열기 → 왼팔로 박스 내부 접근·신발 들어올리기다. base 이동은
이 dataset의 action에 포함하지 않는다.

## 8. ACT 학습

```bash
2ARM_ROBOT/scripts/dual_so101_il train
```

기본값은 CUDA, batch 8, 20,000 step이다. 결과는
`/home/dapier-jhj/dapier_training/dual-so101-box-shoe-act`에 저장된다. 학습 완료는 실물
성공 판정이 아니다. held-out loss 확인 후 MuJoCo replay, IK/충돌 gate, 저속 실물 rollout을
별도로 통과해야 한다.
