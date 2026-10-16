# C_OCT23_INTEGRATION_READINESS — 10/23 통합 준비 상태 (DAPIER C-2, 2026-10-16)

범위: Argon C-1(`6d24284`, `b509838`) 인수 → C-2 수정. A Native·Console·Motor Writer, B Visual Servo 계산기 미수정. 실물 Nav2 Goal·주행·Motor/Torque Write·Sweepick START 0회.

## 1. 이번에 끝난 것 (P0-1 ~ P0-5)

| 항목 | 상태 | 근거 |
|---|---|---|
| P0-1 Git 보존·원격 | DAPIER: C-1 `6d24284` 푸시 완료(`[new branch]`), C-2 코드 `6621b11` + 보고서 커밋을 `pro/sweepick-c2-argon-handoff-20261016`에 푸시. Navigation: **403**(Alpenj 계정 push 권한 없음, fork 없음) → 번들·패치로 보존 | `C_LOCAL_COMMITS_AND_REMOTE_PUSH.json` |
| P0-2 결과 분류 | FAULT > STOP > OBSERVATION_FAILED, 세 증거 보존. `owner_stop`만으로 STOP 아님. Console FAULT 상태는 OBSERVATION_FAILED로 강등 안 됨. 실물 기록 3건으로 확인 | `C_FAULT_CLASSIFICATION_FIX.json`, `tests/test_c2_fault_classification.py` |
| P0-2 복구 계약 | `confirm_operator_recovery(console_status=)`: Console의 `MANUAL_RECOVERY_CONFIRMATION` 기록(`active.operator_recovery`)과 `can_prepare`로만 해제, Boolean 단독 거부 | 같은 파일 |
| P0-3 TF·출처 | 하드코딩 `(0.073, 0, 0.084)` 제거(URDF 체인은 `(0.073, −0.011, 0.094)`, 실측 아님). tf2 lookup(메시지 시각)만 REAL. 출처 없으면 `POST_STOP_OBSERVATION_UNVERIFIED`. 같은 센서 프레임 재발행 거부 | `C_TF_AND_OBSERVATION_PROVENANCE_AUDIT.md` |
| P0-4 ROS 2 dry-run | Jazzy 확인(C-1 로드맵의 Humble 표기 오류). 구독 전용 어댑터(publisher 0, action/service client 0), 기존 타입·QoS·TF 재사용. 목 그래프 2모드 + 빈 실제 그래프 실행 | `C_ROS2_READONLY_DRYRUN.json` |
| P0-5 회귀 | 기존 13 mock + 5 Nav2/도킹 회귀 유지(test_04·test_12는 기본값 제거 API에 맞게 조정, test_12b 추가), 신규 39 → **57/57**, goal_bridge 25(3 skip) OK, `check_public_snapshot.py` PASS, Navigation 러너(`python test/test_sweepick_integration_e2e.py`) 57 OK | 이 문서 §4 |

요구된 추가 검증 9건 ↔ 시험:

| 요구 | 시험 | 결과 |
|---|---|---|
| BUS_LOST + owner_stop → FAULT | `test_bus_lost_with_owner_stop_is_fault_not_stop`, 실물 `20261015_151937` | FAULT |
| 실제 Operator STOP → STOP | `test_real_operator_stop_is_stop` (Console `stop_sent`) | STOP |
| 관측 실패 + 정상 복귀 → OBSERVATION_FAILED | `test_observation_failure_with_normal_return_is_observation_failed`, test_07 | OBSERVATION_FAILED |
| 관측 실패 + 하드웨어 FAULT → FAULT 우선, 관측 실패 보존 | `..._plus_hardware_fault_...`, `..._plus_console_fault_state_...`, 실물 `20261016_112020` | FAULT + `observation_failure` 보존 |
| 주행 직전 영상이 새 PoseStamped로 재발행 → 거부 | `test_republished_pose_from_same_sensor_frame_is_rejected`, 파이프라인 시험, 목 dry-run 재발행 모드 | 거부(REPUBLISHED / EMA / TOO_OLD) |
| TF 누락·과거 시각 변환 → 거부 | `test_tf_missing_is_rejected_not_defaulted`, `test_tf_from_the_past_is_rejected`, test_12b | 거부 |
| Base 이동 중 → 팔 작업 요청 금지 | `test_base_still_moving_forbids_arm_request`, 목 dry-run 0–4 s | 금지 |
| 이전 Sweepick FAULT 미복구 → 새 작업 금지 | `test_prior_unrecovered_fault_blocks_new_task`, `test_prior_sweepick_fault_unrecovered_forbids_new_task` | 금지 |
| Nav2 도착만으로 조작 성공 표시 금지 | test_03, `test_nav2_arrival_alone_is_not_manipulation_success`, dry-run `manipulation_succeeded=False` | 금지 |

## 2. Argon C-1 보고서에서 정정된 사실
- `C_NAV2_DOCKING_REGRESSION_REPORT.json`의 `navigation_base_commit` `3416c7cd82a1…`, `manipulation_base_commit` `42b06403cf89…`는 실제 SHA(`3416c7c65b7d…`, `42b06401e82c…`)와 앞 7자만 같고 나머지는 틀림. 실제 값은 `C_LOCAL_COMMITS_AND_REMOTE_PUSH.json`에 기록.
- `C_MOCK_E2E_REGRESSION_REPORT.json` "13/13 PASS"는 pydantic이 있는 환경에서만 성립(시스템 python3에서는 2개 에러). C-2는 `~/sweepick_261016_c2_argon_handoff/.venv`(system site-packages + pydantic)로 실행하고 README에 조건을 적음.
- 로드맵 C-2 항목 "ROS 2 Humble" → Jazzy.
- 로드맵 §1 "P1 결과 6분류 검증 완료" → 분류 결함 2건(D1, D2) 있었음(C-2에서 수정).
- 하드코딩 TF·합성 기본값(inlier 85/60, RMS 0.008/0.012)은 실물 증거로 쓸 수 없음(§P0-3).

## 3. 10/23까지 남은 blocking 항목 (C가 혼자 끝낼 수 없는 것)

| # | 항목 | 누구 | 왜 막히는가 |
|---|---|---|---|
| B1 | Navigation 원격 푸시 권한 또는 fork 결정 | 사용자 / shouttt1320 | 403. 번들·패치는 `~/sweepick_261016_c2_argon_handoff/navigation_preserve/` |
| B2 | 검출기 출처 사이드카 적용 여부 | Navigation 팀 | 적용 전에는 모든 실물 정지 후 관측이 `POST_STOP_OBSERVATION_UNVERIFIED`로 거부(의도된 안전 기본값). 패치 `proposals/C2_object_detector_3d_provenance_sidecar.patch`(미적용) |
| B3 | 정지 후 EMA 처리 결정 | Navigation 팀 | 현재 EMA는 항상 켜져 있어 정지 후 첫 샘플도 이동 중 값과 섞임. 리셋 또는 raw 샘플 병행 필요 |
| B4 | 실물 ROS 2 그래프 또는 rosbag 1회 녹화 | 현장 | 저장 ROS 자료가 없어 REAL dry-run 미실행. `run_ros2_readonly_dryrun.py --duration 30 --out …`(목 없이)만 돌리면 됨; 발행 0 |
| B5 | Console `status()` 읽기 경로를 C 프로세스에 연결 | A/Console 소유자 + C | 복구 계약은 Console `status()` JSON을 읽기만 함. 활성 Console의 파일·포트는 Codex A 소유(건드리지 않음) — 읽기 전용 노출 방법(HTTP status 또는 파일 사본) 합의 필요 |
| B6 | Base–Arm 물리 장착·외부 보정 | 현장 | `base_to_arm_tf_calibrated=False` 유지; `DIRECT_BASE_TO_ARM_METRIC_TF` 거부. 10/23 안에는 `INDEPENDENT_LOCAL_CAMERA_OBSERVATION`만 현실적 |
| B7 | 도킹 정지 자세 ↔ 조작 작업공간 정합(C-3) | C + 현장 | 실물 측정 전 미착수 |

## 4. 검증 기록
```
# DAPIER worktree, branch pro/sweepick-c2-argon-handoff-20261016
~/sweepick_261016_c2_argon_handoff/.venv/bin/python -m unittest discover -s 2ARM_ROBOT/sweepick/nav2_vslam_integration/tests -p "test_*.py"
  Ran 57 tests ... OK
~/sweepick_261016_c2_argon_handoff/.venv/bin/python -m unittest discover -s 2ARM_ROBOT/sweepick/goal_bridge -p "test_*.py"
  Ran 25 tests ... OK (skipped=3)
python3 2ARM_ROBOT/sweepick/tests/check_public_snapshot.py
  {"source_files": 238, "sha256": "PASS", "syntax": "PASS", "public_boundary": "PASS", "device_access": false}
# Navigation worktree, branch pro/sweepick-c2-argon-handoff-20261016 (c8cc836)
python test/test_sweepick_integration_e2e.py   # 모듈 형식(test.xxx)은 test/__init__.py가 없어 불가 — 기존 구조, 변경 안 함
  Ran 57 tests ... OK
git apply --check robot_ws/src/llm_semantic_docker/proposals/C2_object_detector_3d_provenance_sidecar.patch   # OK, 미적용
# ROS 2 Jazzy dry-run (publisher 0)
run_ros2_readonly_dryrun.py --mock-graph                 → REOBSERVE 거부 (OBSERVATION_TOO_OLD; EMA_FILTER_CARRYOVER)
run_ros2_readonly_dryrun.py --mock-graph --mock-fresh-frames → MANIPULATION_READY:ARM_TASK_NOT_REQUESTED_IN_DRYRUN
run_ros2_readonly_dryrun.py (실제 로컬 그래프, 노드 없음)     → NO_DATA
```

## 5. 하지 않은 것
- 자동 배포·병합 없음. main 및 A/B 통합 브랜치 미변경. `--force` 미사용.
- Navigation fork 생성 안 함. Navigation 저장소 로컬 main(3942b81, origin/main보다 앞섬) 미변경.
- `object_detector_3d.py`, `task_manager.py`, 도킹 노드, launch, config 미수정(제안 패치만).
- 활성 Sweepick Console·native 파일·장치 미접촉. 실물 기록은 `~/sweepick_console_runs/*/RUN_RECORD.json` 읽기만.
- 미추적 `2ARM_ROBOT/sweepick.zip`은 커밋·푸시하지 않음.

보고서 커밋: 이 파일을 담은 커밋이 브랜치 `pro/sweepick-c2-argon-handoff-20261016`의 HEAD이며, SHA는 인계 메시지에 기록.
