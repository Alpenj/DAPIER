# Phase 0 개발 인수인계

## 목표

실물 장비가 준비되기 전에 ACT 학습 데이터의 형태와 품질 판정을 고정한다.
Ubuntu ROS 2 교육 PC에서는 먼저 이 문서의 검증 명령을 통과시킨 뒤 recorder
adapter 개발을 이어간다.

## 확정 장비와 제약

| 항목 | 현재 값 |
|---|---|
| 양팔 | JDcobot300 |
| 모바일 베이스 | TurtleBot3 Waffle Pi / XM430-W210-T |
| RGB-D | Orbbec Astra Pro |
| 연산 장치 | ASUS TUF Gaming A16 / RTX 5050 |
| 인원·기간 | 4명 / 핵심 개발 6주 |
| 추가 장비 예산 | 0원 |
| 현재 조립 | 부분 조립 |

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

## 검증 기준

Ubuntu에서 다음 한 줄을 실행한다.

```bash
cd ~/DAPIER/2ARM_ROBOT
bash scripts/verify_ubuntu_ros2.sh
```

완료 조건:

- Python 단위 테스트 전부 통과
- `colcon build --symlink-install --packages-select shoe_sorting_data` 통과
- `shoe_episode --help` 실행 가능
- 합성 20 episode를 index했을 때 `indexed=20`, `usable=20`

## 아직 실물 검증되지 않은 값

합성 fixture의 `arm_dof=6`, `gripper_dof=1`은 기본 예시다. 다음 항목을
실기체 introspection 결과 없이 정본으로 확정하면 안 된다.

- 좌·우 joint name, order, sign, unit와 제어 주기
- gripper state/action dimension과 단위
- robot/controller/firmware/calibration version
- Astra Pro RGB/Depth timestamp source와 허용 skew
- Nav2 도킹 완료 및 base 정지를 증명할 실제 topic

## 다음 구현 단위

### 1. Mock ROS 2 recorder

- 합성 `JointState`, 좌·우 action, base velocity, RGB/Depth metadata topic 발행
- approximate time synchronization 결과를 Phase 0 contract로 저장
- stop/abort를 outcome과 failure reason으로 기록
- 현재 quality validator를 그대로 호출

### 2. ACT/LeRobot adapter

- manifest stream order를 보존해 observation/action tensor 생성
- train/validation split과 normalization statistics의 checksum 기록
- raw episode를 수정하지 않고 변환 receipt를 별도 생성

### 3. Offline evaluator

- ACT inference 없이도 identity/lag/noise policy로 evaluator 계약 검증
- skill별 0/0.5/1 progress, latency, retry, reject reason 기록
- 이후 ACT와 IDM/FDM/EMA ablation이 같은 evaluator를 공유

### 4. 신발 짝 추론 mock

- 입력: shoe crop/image ID와 후보 목록
- 출력: `pair_id`, left/right, confidence, allowed skill
- API 실패·낮은 confidence에서는 관절 명령을 만들지 않고 재관측 요청

## 안전 경계

LLM/VLM 출력은 관절값이 아니다. pair/slot/skill 선택만 허용하며 ACT 또는
scripted skill의 action도 safety supervisor, workspace limit, base-stop
interlock를 통과해야 한다.
