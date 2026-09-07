# Dual SO-101 main 통합 후보 검증 기록

record_id: `DAPIER-2026-09-07-main-integration`

## 상태

현재는 **Draft 통합 후보**다. `main` 병합 완료나 실물 실행 승인이 아니다.
기존 PR #40, #49, #51과 다른 개발 브랜치를 변경하거나 종료하지 않는다.

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

## 남은 병합 차단 및 별도 검증

- main과 누적 79커밋을 결합한 전체 CI/build 결과 확인. 로컬에서는 위 일부 fixture만 실행했다.
- RoboTwin planner의 초기 infeasible 구간 허용 예외, 전체 경로의 금지 collision gate 재검증.
- canonical 관절 이름·순서, 실제 IL 단위/action 의미·depth 계약과 importer 연결 검증.
- 외부 RoboTwin upstream·dirty 상태·asset provenance 및 실제 SAPIEN/CuRobo 실행 검증.
- checkpoint loading fallback과 실물 dispatch 경계의 보안 검증.
- 다른 개발 브랜치와 로컬 WIP의 필요한 변경 선별. 별도 C++ safety 코드를 이미 통합했다고 표시하지 않는다.

실물 camera/calibration, watchdog/E-stop, 양팔 동기 dispatch와 hardware acceptance는 미검증이다.
위 회귀 테스트는 실제 writer episode, task rollout, 실제 IL 교차 테스트 또는 hardware test가 아니다.

## 병합·정리 원칙

검증 완료 전 main에 병합하거나 auto-merge를 켜지 않는다.
병합 직전에 main/source/candidate SHA 변화를 다시 확인하고, 다른 작업자의 변경이 있으면 덮어쓰지 않는다.
현재와 같이 후속 브랜치가 기존 커밋을 공유하는 경우 저장소 정책이 허용하면 merge commit 방식으로 이력을 보존한다.
기존 브랜치/PR/worktree 정리는 별도 작업이며, 로컬 미푸시 변경이 확인되기 전에는 삭제하지 않는다.
