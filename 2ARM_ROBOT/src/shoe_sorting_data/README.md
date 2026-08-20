# Shoe Sorting Data Phase 0

실물 JDcobot300, TurtleBot3, Astra Pro가 없어도 먼저 개발할 수 있는
DYNA-lite 데이터 기반입니다. 이 패키지는 ACT 학습 코드를 넣기 전에
episode의 관측·행동 순서와 품질 기준부터 고정합니다.

현재 구현 범위:

- 좌·우 팔, 좌·우 그리퍼, 베이스 속도, RGB/Depth timestamp 계약
- 관절 차원과 그리퍼 차원을 manifest에 명시(실기체 확인 전 변경 가능)
- 합성 golden episode 생성
- timestamp gap, stream shape, camera health/skew/drop, joint jump, checksum 검사
- 조작 중 TurtleBot base의 명령 속도와 측정 속도 정지 interlock 검사
- 검수 대기 episode와 미확정 calibration/config를 학습 usable에서 제외
- SQLite manifest 생성과 train/validation, usable, success, shoe pair 질의
- one-shot perception exemplar의 유사도/margin 기반 match 또는 abstain
- accepted episode 기반 typed skill exemplar와 evaluation leakage audit

SQLite manifest는 원본이 아닌 파생 snapshot이다. provenance column이 추가된
버전으로 갱신한 뒤에는 `shoe_episode index`를 다시 실행해 DB를 재생성한다.

현재 코드는 합성 데이터와 계약만 검증합니다. 실제 로봇 동작, 카메라 영상,
캘리브레이션, ACT 성공을 검증했다는 의미가 아닙니다.

## 바로 실행

패키지 폴더에서 외부 라이브러리 설치 없이 실행할 수 있습니다.

```powershell
cd C:\Users\hjjeon\Documents\DAPIER\repo\2ARM_ROBOT\src\shoe_sorting_data
python -m unittest discover -s test -v

python -m shoe_sorting_data.cli generate `
  --root .\output\golden_episodes `
  --count 20

python -m shoe_sorting_data.cli validate `
  --manifest .\output\golden_episodes\episode_000001\episode_manifest.json

python -m shoe_sorting_data.cli index `
  --root .\output\golden_episodes `
  --db .\output\episode_manifest.sqlite3

python -m shoe_sorting_data.cli query `
  --db .\output\episode_manifest.sqlite3 `
  --usable true `
  --split validation
```

Ubuntu에서는 상위 `2ARM_ROBOT` 폴더를 ROS 2 workspace로 사용합니다.

```bash
cd ~/DAPIER/2ARM_ROBOT
colcon build --symlink-install --packages-select shoe_sorting_data
source install/setup.bash
shoe_episode --help
```

## Perception·skill exemplar

아래 기능은 GEN-1.5 모델을 실행하는 것이 아니다. 신발 짝 후보와 검증된
skill metadata만 반환하며 `control_authorized=false`를 유지한다.

```bash
shoe_episode pair-add \
  --registry output/pair_registry.json \
  --exemplar-id shoe_a_left \
  --pair-id pair_a \
  --object-instance-id shoe_a_left_object \
  --embedding 1,0,0 \
  --session-id session_train_a \
  --background-id background_train

shoe_episode pair-match \
  --registry output/pair_registry.json \
  --embedding 0.99,0.01,0

shoe_episode skill-register \
  --manifest output/golden_episodes/episode_000001/episode_manifest.json \
  --output output/exemplars/grid_pick/skill_exemplar.json \
  --exemplar-id grid_pick_001 \
  --precondition base_stopped \
  --precondition camera_fresh \
  --postcondition shoe_in_target \
  --timeout-ms 15000 \
  --tag grid

shoe_episode skill-retrieve \
  --root output/exemplars \
  --manifest output/golden_episodes/episode_000002/episode_manifest.json \
  --skill-id pair_and_place \
  --tag grid

shoe_episode exemplar-audit \
  --exemplar-root output/exemplars \
  --evaluation-root output/golden_episodes/episode_000002
```

## 고의 오류 fixture

quality gate가 실제로 실패하는지 다음 fixture로 확인할 수 있습니다.

```powershell
python -m shoe_sorting_data.cli generate `
  --root .\output\bad_base_motion `
  --count 1 `
  --fault base_motion
```

지원 fault: `base_motion`, `camera_frame_gap`, `camera_skew`,
`checksum_mismatch`, `dimension_mismatch`, `duplicate_timestamp`, `joint_jump`,
`missing_camera`, `sample_gap`.

## 실물 연결 전 확정할 값

`arm_dof=6`, `gripper_dof=1`은 합성 fixture의 기본값일 뿐입니다. 다음 값을
실기체 introspection과 캘리브레이션 결과로 확인한 뒤 manifest 생성기에
연결해야 합니다.

1. 좌·우 joint name, order, unit, sign
2. 그리퍼 명령·상태 차원과 단위
3. 실제 controller/software/calibration version
4. Astra Pro RGB/Depth frame timestamp와 허용 skew
5. Nav2 도킹 완료 후 base velocity가 0인지 판정하는 실제 신호

그 다음 구현 단계는 ROS 2 topic을 이 계약의 `samples.jsonl` 또는 향후
LeRobot 변환 입력으로 기록하는 recorder adapter입니다.
