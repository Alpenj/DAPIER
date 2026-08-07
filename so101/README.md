# SO-101 작업 허브

`record_id: DAPIER-2026-08-07-so101-consolidation`

SO-101 관련 작업이 네 개의 비슷한 폴더에 흩어져 있어, 오늘 직접 Git 상태와
고유 파일을 대조한 뒤 이 문서를 정본 인덱스로 만들었다. 패키지 import와 ROS 2
workspace symlink를 깨뜨리지 않기 위해 이미 검증한 두 소스 디렉터리는 이름을
바꾸지 않았다. 외부 checkout에서 직접 만든 부분만 integration overlay나 patch로
가져왔다.

## DAPIER 안의 정본

| 구분 | 정본 | 현재 확인한 범위 |
|---|---|---|
| sim-first Gate | [`dapier_sim_first`](../dapier_sim_first/README.md) | G0 환경·계약과 G1 scripted pick-and-lift |
| DAPIER ROS 2 | [`so101_ros2`](../so101_ros2/README.md) | joint contract와 mock 안전 텔레옵 |
| 카지노 딜러 연결 순서 | [`SO101_CASINO_DEALER_RUNBOOK_KO.md`](../docs/SO101_CASINO_DEALER_RUNBOOK_KO.md) | 아직 끝나지 않은 실험 체크리스트 |
| sim-to-real 계약 | [`2026-08-07-so101-sim-to-real-foundation.md`](../project-planning/2026-08-07-so101-sim-to-real-foundation.md) | Gate별 증거 수준과 중단 조건 |
| LeRobot/ROS 2 분해 설계 | [`2026-08-06-dapier-lerobot-ros2-deconstruction-lab.md`](../project-planning/2026-08-06-dapier-lerobot-ros2-deconstruction-lab.md) | 별도 worktree에만 있던 설계 문서를 main에 통합 |

`dapier_sim_first`와 `so101_ros2`는 둘 다 DAPIER 소스지만 역할이 다르다.
앞쪽은 ROS 2와 실기체가 없는 MuJoCo 검증 경로이고, 뒤쪽은 ROS 2 message와
안전 경계를 학습하는 패키지다. 둘을 한 Python/colcon 패키지로 합치지 않는다.

## 가져온 실험 작업

| 위치 | 분류 | 사용 규칙 |
|---|---|---|
| [`integrations/lerobot_v0_6_so101_mujoco`](integrations/lerobot_v0_6_so101_mujoco/README.md) | LeRobot v0.6.0 실험 overlay | upstream checkout에 적용하는 보존본이며 현재 sim 정본은 아님 |
| [`integrations/feetech_ros2_driver`](integrations/feetech_ros2_driver/README.md) | 외부 ROS 2 driver 로컬 patch | hardware-facing 실험이므로 DAPIER mock 코어와 분리 |
| [`hardware_tools`](hardware_tools/README.md) | 물리 장비 보조 도구 | `read_only`와 `writes_hardware`를 디렉터리부터 분리 |
| [`records`](records/2026-08-07-so101-consolidation.md) | 통합 조사 기록 | 원본 위치, 크기, 해시와 오늘 검증 결과 |
| [`interactive sim 기록`](records/2026-08-07-so101-interactive-sim-controls.md) | LeRobot viewer 조작 검증 | Shift chord, 연속 XYZ, reachable cube와 scripted lift 결과 |

## 홈 디렉터리에 보이는 비슷한 폴더의 의미

| 로컬 위치 | 실제 역할 | DAPIER와의 관계 |
|---|---|---|
| `$HOME/so101` | LeRobot checkout, 7.7GB venv, 실험 dataset과 로컬 증거가 섞인 실행 작업장 | 고유 소스만 이 디렉터리의 integration/hardware tools로 가져옴 |
| `$HOME/so101_ros2_ws` | `build/install/log`가 생기는 colcon 작업장 | `src/dapier-so101-ros2`가 DAPIER의 `so101_ros2`를 가리키는 symlink |
| `$HOME/DAPIER/so101_ros2` | DAPIER가 직접 소유하는 ROS 2 정본 | 수정과 커밋은 여기에서만 함 |
| `$HOME/DAPIER-lerobot-ros2-lab` | 2026-08-06에 만든 별도 Git worktree | 유일했던 계획 문서는 main에 이력째 합쳤고 원 worktree는 보존 중 |

## Git에 복제하지 않은 것

다음 항목은 작업 결과를 잃은 것이 아니라, 소스와 섞지 않기 위해 로컬 실행
영역에 그대로 둔 것이다.

- LeRobot upstream 전체 checkout과 `.venv`
- colcon의 `build/`, `install/`, `log/`
- 캘리브레이션 JSON/JSONL과 카메라 이미지
- LeRobot Dataset v3 Parquet 및 실행 영상
- SO-ARM100에서 온 약 16MB STL/MJCF 원본 자산

원시 데이터와 실행 증거는 Git에 넣지 않고, 재현에 필요한 revision·metric·해시는
통합 기록에 남긴다. 기존 외부 폴더는 이번 작업에서 삭제하지 않았다. 새 정본을
한 번 더 사용해 본 뒤에만 중복 worktree나 오래된 실행 작업장의 정리를 판단한다.
