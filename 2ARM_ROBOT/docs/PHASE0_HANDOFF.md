# Phase 0 개발 인수인계

## 목표

ACT 학습 데이터의 형태와 품질 판정을 고정하고, 실측 하드웨어 프로필을 실제 recorder에
연결한다. Ubuntu ROS 2 교육 PC에서는 먼저 이 문서의 검증 명령을 통과시킨 뒤 recorder
adapter 개발을 이어간다.

## 확정 장비와 제약

| 항목 | 현재 값 |
|---|---|
| 양팔 | JDcobot200 두 대 / 팔당 STS3215 6개(관절 5 + 그리퍼 1) |
| 모바일 베이스 | TurtleBot3 Waffle Pi / XM430-W210-T |
| RGB-D | 라벨 AADJA1300GX / USB Orbbec Astra 계열, driver 미확인 |
| 연산 장치 | ASUS TUF Gaming A16 / RTX 5050 |
| 인원·기간 | 4명 / 핵심 개발 6주 |
| 추가 장비 예산 | 0원 |
| 현재 조립 | 양팔 USB 연결 확인, TurtleBot 실측 완료, 카메라 현재 분리 |

## 구현 완료

`src/shoe_sorting_data`는 다음 파일 계약을 사용한다.

```text
episode_000001/
├── episode_manifest.json
└── samples.jsonl
```

manifest에는 task, stream dimension/unit, robot/controller/calibration version,
operator/session/split, outcome, checksum이 들어간다. sample에는 episode
monotonic timestamp, state/action vector, RGB/Depth frame timestamp와 health가
들어간다.

quality gate는 다음을 hard error로 처리한다.

- sample/camera timestamp 역행·중복·과도한 gap
- camera 누락, frame drop, timestamp skew
- state/action stream 누락, 차원 오류, NaN/Inf
- 관절 상태의 비정상 급변
- 조작 중 base 명령 또는 측정 속도 발생
- checksum 불일치
- 실제 robot data의 미확정 calibration/config version
- 사람 검수가 `accepted`가 아닌 episode

GEN-1.5 조사에서 즉시 적용 가능한 축소판도 구현했다.

- one-shot perception exemplar: 임베딩 유사도와 margin이 낮으면 `abstain`
- typed skill exemplar: accepted episode만 등록하고 동일 controller contract만 검색
- leakage audit: object/session/span 중복은 평가 error로 처리

모든 exemplar 결과의 `control_authorized`는 `false`다. 실제 action은 ACT와
safety supervisor가 별도로 생성·승인해야 한다.

## 검증 기준

Ubuntu에서 다음 한 줄을 실행한다.

```bash
cd ~/DAPIER/2ARM_ROBOT
bash scripts/verify_ubuntu_ros2.sh
```

완료 조건:

- Python 단위 테스트 전부 통과
- `colcon build --symlink-install --packages-select shoe_sorting_data` 통과
- `ros2 run shoe_sorting_data shoe_episode --help` 실행 가능
- 합성 20 episode를 index했을 때 `indexed=20`, `usable=20`

## 실측으로 확정된 값과 남은 값

팔당 모터 6개가 응답했으며 데이터 계약은 `arm_dof=5`, `gripper_dof=1`로 분리한다.
TurtleBot stationary tolerance는 선속도 0.0025m/s, 각속도 0.0021rad/s다. 다음 항목은
아직 실기체 검증 없이 정본으로 확정하면 안 된다.

- 물리적 좌·우 arm label, 각 모터 역할과 회전 sign, 안전한 관절 limit
- 그리퍼 정규화 단위와 열린/닫힌 방향
- robot/controller/firmware/calibration version
- Astra RGB/Depth driver, timestamp source, intrinsics/extrinsics와 허용 skew
- Nav2 도킹 완료 및 base 정지를 증명할 실제 topic
- 양팔 작업 전류와 current/load-to-torque 보정

실물 driver가 실행된 교육 PC에서는 motion command를 보내기 전에 다음 read-only
snapshot을 먼저 수집한다.

```bash
bash scripts/capture_ros2_hardware_snapshot.sh \
  output/hardware_snapshots/first_connected
```

snapshot은 ROS graph와 selected metadata만 저장하고 이미지 pixel이나 제어
명령은 기록하지 않는다. 이 결과가 없으면 mock 이름·차원·주기를 실제 계약으로
승격하지 않는다.

## 다음 구현 단위

### 1. Mock ROS 2 recorder — 구현 완료

- 합성 `JointState`, 좌·우 action, base velocity, RGB/Depth metadata topic 발행
- approximate time synchronization 결과를 Phase 0 contract로 저장
- stop/abort를 outcome과 failure reason으로 기록
- 현재 quality validator를 그대로 호출

one-shot 검증 명령:

```bash
ros2 run shoe_sorting_data shoe_mock_demo \
  --output output/mock_episodes/episode_000001 \
  --samples 40
```

실제 장비 topic 이름과 joint order는 아직 연결하지 않았으며, 현재 publisher는
metadata-only `Image`와 합성 관절값을 사용한다.

### 2. ACT/LeRobot adapter

- manifest stream order를 보존해 observation/action tensor 생성
- train/validation split과 normalization statistics의 checksum 기록
- raw episode를 수정하지 않고 변환 receipt를 별도 생성

### 3. Offline evaluator

- ACT inference 없이도 identity/lag/noise policy로 evaluator 계약 검증
- skill별 0/0.5/1 progress, latency, retry, reject reason 기록
- 이후 ACT와 IDM/FDM/EMA ablation이 같은 evaluator를 공유

### 4. 실제 embedding/API adapter

- 현재 구현된 registry에 Astra Pro shoe crop의 실제 embedding을 입력
- 출력: `pair_id`, confidence, margin, abstention reason
- API 실패·낮은 confidence에서는 관절 명령을 만들지 않고 재관측 요청

## 안전 경계

LLM/VLM 출력은 관절값이 아니다. pair/slot/skill 선택만 허용하며 ACT 또는
scripted skill의 action도 safety supervisor, workspace limit, base-stop
interlock를 통과해야 한다.
