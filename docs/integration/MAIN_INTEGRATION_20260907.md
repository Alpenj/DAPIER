# Dual SO-101 main 통합 후보 검증 기록

record_id: `DAPIER-2026-09-07-main-integration`

## 로컬 병합 전 재검증 — 2026-09-07

PR #53의 `8434591e9257a710fe53495318d00183d8cc6a1c`를 별도 worktree에서 읽고,
보고서에 남아 있던 두 거부 경계 문제를 직접 재현했다.

- 시작 상태가 infeasible이어도 뒤쪽이 clear이면 경로를 반환했다. 해당 예외를 제거해
  direct path와 graph connector 모두 모든 점의 constraint 검사를 통과해야 반환하도록 했다.
- checkpoint loader가 `weights_only=True` 호출의 TypeError 뒤에 안전 옵션 없이 재시도했다.
  재시도를 제거했다. 이 옵션을 지원하지 않는 PyTorch는 checkpoint를 읽지 못한 채 실패한다.

같은 신규 테스트가 수정 전에는 각각 실패하고, 수정 후에는 통과하는 것을 확인했다.
planner 테스트는 실제 nested segment를 NumPy 연산과 가짜 constraint 결과로 실행한다.
CuRobo collision geometry나 전체 SAPIEN trajectory 검증은 아니다.

| 로컬 검사 | 결과 |
|---|---|
| converter 회귀 16개 + planner 거부 회귀 1개 | 17개 PASS |
| native ACT: 안전 로드 거부, queue 경계, CPU 학습/checkpoint roundtrip | 3개 PASS |
| native ACT rollout/supervisor/mock bus | 6개 PASS |
| 기존 physical-motion 차단 및 static safety | 9개 PASS |
| 기존 XML model contract, HDF5 converter fixture | 각 PASS |
| `git diff --check` | PASS |

RoboTwin fixture는 기존 RoboTwin Python 3.10 환경, ACT는 기존 LeRobot Python 3.12 환경에서
CPU로 실행했다. 패키지를 설치하거나 기존 runtime·dataset·원본 worktree에 patch를 적용하지 않았다.
원격 CI는 최종 PR head에서 다시 확인한다. 병합 여부와 최종 SHA는 GitHub PR #53을 기준으로 한다.

### 확인한 경계와 미지원 범위

- config의 좌/우 5개 joint 이름과 `get_obs()`의 이름 기반 qpos 선택, converter의
  `left arm 5 → left gripper → right arm 5 → right gripper` 필드 연결을 대조했다.
  실제 IL과의 단위/action 의미 교차 검증은 미완료다. NPZ를 실제 record/train 경로에 자동으로
  연결하는 importer는 없으며, SIM/offline 결과로만 취급한다.
- native ACT 평가 함수는 bus를 주입하지 않는 dry-run이다. 명시적 승인 decision을 받은
  adapter의 rad→degree와 gripper→percent 변환은 fake bus로 확인했다. 기존 jog/wheel/teleop
  차단은 static suite로 확인했다. 별도 수동 IL 스크립트의 사용자 확인 절차는 실행하지 않았다.
- 외부 RoboTwin HEAD는 `45a853a9c87bd190e028db911137be01c2087d54`이고 local dirty 변경이 있다.
  읽기만 수행했다. runtime pin, asset provenance, 실제 SAPIEN/CuRobo 재실행은 검증하지 못했다.
  installer는 외부 경로를 명시하는 별도 수동 SIM 도구이며 CI가 실행하지 않는다.
- 9월 5일 handover 결과는 초기 constraint 예외 제거 전 기록이다. 수정 후 handover 성공,
  전 경로의 실제 collision 무발생 또는 실물 안전성을 인증하지 않는다.
- PR #40/#49/#51의 ancestry는 포함한다. 별도 C++ safety/bimanual/jitter/문서 PR과 로컬 WIP는
  이번 병합 범위가 아니며, 원본 브랜치·worktree를 정리하지 않는다.

## 초기 통합 후보 기록

아래는 로컬 추가 수정 전의 후보 생성 및 검증 기록이다.

- 후보: `pro/main-integration-so101-20260907-01`
- 시작 소스: `3ff180aff3eb312b91a2e2c6ea8e8d6b734148b3` (PR #51)
- 조회한 main: `c8e1eb0dc6345c6e38f1169e3e1e1b43a297e116`
- GitHub compare에서 main은 시작 소스의 조상이며, 소스 쪽 누적 커밋은 79개다.
- 후보는 시작 소스에서 분기했으며 기존 커밋을 squash/rebase하지 않았다.
- GitHub의 파일 단위 API로 후보에만 수정·신규 파일을 기록했다.

## 작업물 보존 범위

[REMOTE_REFS_20260907.json](REMOTE_REFS_20260907.json)에 후보 생성 전 원격 브랜치 50개의 이름과 full SHA를 기록했다.
이는 **ref 기록이며 Git 객체, LFS, 노트북 WIP, dataset 또는 runtime의 백업이 아니다.**
브랜치 목록을 읽었다는 사실도 모든 브랜치 소스를 전수검토했다는 뜻이 아니다.

원본 worktree와 외부 RoboTwin, 실제 IL dataset은 검토 컨테이너에 없다.
미푸시 커밋과 로컬 변경의 내용·보존 상태를 원격 조회만으로 인증하지 않는다.
해당 위치에 대한 쓰기, Git fetch/pull/reset/clean/stash, 하드웨어 접근은 하지 않았다.
기존 브랜치·PR 삭제, 강제 push, main ref 변경도 하지 않았다.

## 선별한 브랜치 관계

기준은 시작 소스 `3ff180a`다. 커밋 차이는 ancestry 기준이며 patch 동등성이나 코드 품질 판정이 아니다.

| 브랜치 | 확인한 tip | 관계 | 이번 처리 |
|---|---|---|---|
| main | `c8e1eb0` | 조상, 소스보다 79커밋 이전 | 그대로 유지 |
| pro/mujoco-shoe-mission-01 | `cbd8ce5` | 조상, 소스보다 64커밋 이전 | 기존 이력 포함 |
| pro/mujoco-shoe-followup-01 | `e58496f` | 조상, 소스보다 24커밋 이전 | 기존 이력 포함 |
| pro/cpp-safety-mock-bridge-01 | `84cb02a` | 분기됨, 해당 쪽 고유 39커밋 / 소스 쪽 79커밋 | 통합 보류, 원본 보존 |
| pro/lerobot-biso-contract | `29bc853` | 분기됨, 해당 쪽 고유 2커밋 / 소스 쪽 85커밋 | 통합 보류, 원본 보존 |
| pro/jitter-prep-01 | `607aece` | 분기됨, 해당 쪽 고유 1커밋 / 소스 쪽 98커밋 | 통합 보류, 원본 보존 |

나머지 브랜치는 tip inventory만 기록했다. 관련 구현이 다른 브랜치에 있다는 이유로 없다고 단정하거나, 검증 없이 일괄 병합하지 않는다.

## 이번 최소 수정

변경한 구현 파일은 `2ARM_ROBOT/robotwin/convert_episode.py` 하나다.

1. 첫 measured state를 velocity 검사에 포함한다. 첫 전이와 T=1 급변을 놓치지 않는다.
2. frequency에 finite·numeric scalar·positive integral int64 계약을 적용한다. `29.97`을 `29`로 조용히 바꾸지 않는다.
3. `action[t] == measured state[t+1]`의 기존 의미를 유지하고 비교의 relative tolerance를 0으로 명시한다.
4. 임시 NPZ를 완성한 뒤 같은 디렉터리의 hard link로 게시한다. 검사 이후 다른 writer가 생성한 출력을 덮어쓰지 않는다.
5. 기존 출력과 dangling symlink를 보존한다. CLI에서도 마지막 출력 symlink를 resolve하여 우회하지 않는다.

Hard link를 지원하지 않는 filesystem에서는 실패하며 replace로 fallback하지 않는다.
이 변경은 시스템 전원 상실에 대한 durability, 악의적인 디렉터리 교체, 전체 filesystem 보안을 인증하지 않는다.
Depth 단위 추정, 실제 calibration, arm 절대 limits를 임의로 추가하거나 실제 IL action 의미를 바꾸지 않았다.

## 직접 실행한 테스트

검토 컨테이너에서 기존 archive의 6개 source blob을 pinned Git blob SHA와 대조한 후 임시 baseline/candidate 사본을 사용했다.
소스 전체 repository를 로컬에서 빌드한 것은 아니다. 패키지를 새로 설치하지 않았다.

환경: Python 3.13.5, NumPy 2.3.5, OpenCV 4.13.0.92, h5py 3.15.1.
`PYTHONDONTWRITEBYTECODE=1`; tmp/cache와 출력은 검토용 임시 디렉터리로 격리했다.

| 실행 | exit code | 실제 결과 | 범위 |
|---|---:|---|---|
| 동일한 신규 회귀 테스트를 기존 converter에 실행 | 1 | `Ran 16 tests`; `FAILED (failures=11, errors=1)` | subtest 실패 건수 포함, 12개 독립 test method 실패라는 뜻은 아님 |
| 신규 회귀 테스트를 수정 candidate에 실행 | 0 | `Ran 16 tests`; `OK` | handcrafted HDF5와 output-preservation fixture |
| 기존 test_convert_episode.py | 0 | `PASS: RoboTwin HDF5 -> DAPIER Dual SO-101 canonical episode` | synthetic converter fixture |
| 기존 test_contract.py | 0 | `PASS: DAPIER RoboTwin 12-axis and mount contract` | synthetic XML fixture, SAPIEN 아님 |

신규 테스트는 concurrent output, API/CLI dangling symlink, first/single/final transition, invalid fps, 정상 RGB/depth schema 및 저장 실패 cleanup을 포함한다.
기존 XML fixture의 PASS를 active-joint 순서나 실제 URDF loader의 완전한 검증으로 확대하지 않는다.

테스트 진입점:

```bash
python -B 2ARM_ROBOT/robotwin/test_contract.py
python -B 2ARM_ROBOT/robotwin/test_convert_episode.py
python -B -m unittest discover -s 2ARM_ROBOT/robotwin -p test_merge_regressions.py -v
```

신규 `.github/workflows/robotwin-contract.yml`은 GitHub-hosted CPU runner에서 이 진입점을 실행한다.
CI 전용 dependency를 별도 runner에 설치하며 노트북의 venv 또는 RoboTwin을 변경하지 않는다.
Workflow 추가 자체는 CI 통과 증거가 아니다. 최종 PR head의 실제 check 결과를 별도로 확인해야 한다.

## 후속 실행 검증

실제 IL 교차 검증, runtime/asset 재현성, 변경 후 SAPIEN/CuRobo rollout, 실물 camera/calibration,
watchdog/E-stop과 양팔 동기 dispatch는 아직 확인하지 못했다. 이 항목은 위에 명시한
SIM/offline 통합 범위를 실제 학습·실물 실행으로 확대하기 전에 별도 검증한다.

## 병합·정리 원칙

위 통합 범위의 로컬 검사와 최종 PR head의 CI 성공을 확인한 뒤 사용자 병합 요청에 따라 진행한다.
병합 직전에 main/source/candidate SHA 변화를 다시 확인하고, 다른 작업자의 변경이 있으면 덮어쓰지 않는다.
현재와 같이 후속 브랜치가 기존 커밋을 공유하는 경우 저장소 정책이 허용하면 merge commit 방식으로 이력을 보존한다.
기존 브랜치/PR/worktree 정리는 별도 작업이며, 로컬 미푸시 변경이 확인되기 전에는 삭제하지 않는다.
