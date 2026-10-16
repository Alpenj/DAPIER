# DAPIER C — 2026년 10월 23일 마감 통합 로드맵 및 미검증 실물 조건 구분

- **담당**: Gemini 4 Argon / 독립 C팀
- **기능 구현 마감**: 2026-10-23
- **브랜치**: `pro/sweepick-c-vslam-nav2-integration-20261016`

---

## 1. 무장치 통합 검증 완료 범위 vs. 미검증 실물 조건 구분

본 C팀 통합 패키지는 `HardwareReadinessState` 계약을 통해 **무장치 소프트웨어 통합 검증 결과(`MOCK_DEVICE_FREE`)**와 **실물 로봇 준비 상태(`PHYSICAL_FIELD`)**를 엄격히 분리합니다.

| 항목 | 현재 상태 | 검증 근거 | 실물 실행 차단 게이트 |
|---|---|---|---|
| **P0 8단계 파이프라인 상태기계** | **검증 완료 (Mock/Offline)** | `VslamSweepickCoordinator` + 13개 E2E 시나리오 통과 | `validate_stage_transition`으로 순서 역전 및 단계 생략 차단 |
| **이동 전 물체 좌표 폐기 및 `REOBSERVE` 강제** | **검증 완료 (Mock/Offline)** | `BaseStopVerifier` + `PostStopReobserveGate` 통과 | `stopped_at_s` 이전 프레임, EMA 잔류값, 0.5s Persistence Latch, 180s Qwen BBox Latch 전부 차단 |
| **P1 Sweepick 승인 범위 및 결과 6분류 연결** | **검증 완료 (Mock/Offline)** | `SweepickExecutionAdapter` 통과 | 통합 계층의 자체 Motor/Torque 권한 생성 및 자동 START 영구 금지 |
| **Robot Base ↔ Arm Base 물리 장착 및 외부 보정 (`base_footprint ↔ arm_base`)** | **미검증 (`False`)** | Navigation 저장소 커밋 `80fe5b6`에서 미검증 URDF(`turtlebot3_dual_so101`) 제거 확인 | `base_to_arm_tf_calibrated=False` 상태에서 `DIRECT_BASE_TO_ARM_METRIC_TF` 요청 시 `UNVERIFIED_BASE_TO_ARM_TF`로 즉시 거부 |
| **Navigation 카메라 ↔ Sweepick 상단(OS30A) 카메라 시야/외부 보정** | **미검증 (`False`)** | 주행용 전면 카메라와 조작부 상단/손목 카메라는 별도 광학계로 운영됨 | `INDEPENDENT_LOCAL_CAMERA_OBSERVATION` 모드로만 인계(도착·정지 확인 후 Sweepick 자체 ArUco 보드/손목 카메라 관측 수행) |
| **양팔 손목 카메라 USB 경로 및 커버 테스트 (`WRIST_IDENTITY_CONFIRMATION.json`)** | **현장 실행 시 매회 확인 필요** | `console/control.py:resolve_devices` | 미확인 시 `DEVICE_ROUTE_OR_WRIST_IDENTITY_UNCONFIRMED` (`EXECUTION_REJECTED`) |

---

## 2. 10월 23일까지 필요한 단계별 통합 작업 일정

| 일정 | 단계 | 세부 작업 내용 | 담당 / 경계 |
|---|---|---|---|
| **10/16 (금) 완료** | **C-1: 계약 감사 및 P0/P1/P2 통합 코어 구현** | - Navigation 6개 파일 및 Sweepick Console/Native/Visual Servo 계약 감사 완료<br/>- `nav2_vslam_integration` 패키지 및 `sweepick_integration_bridge.py` 구현<br/>- Mock E2E 13종 및 Nav2/Docking 회귀시험 5종 + 기존 Goal Bridge 25종 통과 | C팀 독립 브랜치 (A·B팀 파일 수정 0건) |
| **10/17 ~ 10/18** | **C-2: ROS 2 Humble 라이브 토픽 어댑터 연결 (Read-Only Dry-Run)** | - `/odom`, `/cmd_vel`, `/docking/status`를 구독하여 실물 주행 없이 `BaseStopEvidence`를 생성하는 ROS 2 Subscriber 어댑터 연결<br/>- `/docking/target_object_pose` 수신 시 `object_detector_3d.py`의 EMA/Latch 여부를 구분하기 위한 프레임 시퀀스·타임스탬프 필터링 검증 | C팀 (모터/주행 명령 미발행) |
| **10/19 ~ 10/20** | **C-3: 도킹 정지 위치(Standoff) ↔ 조작부 작업 공간(Workspace) 정합 검증** | - 모바일 베이스에 듀얼 암 장착 시 `workplace1`/`desk` 도킹 완료 자세에서 조작 대상 블록이 Sweepick 상단 카메라(ArUco 보드) 및 왼손목 카메라 FOV(`0.05m < x < 0.45m, |y| < 0.25m`) 내부에 들어오는지 정적 배치 측정<br/>- 필요 시 `docking_stations.yaml`의 `standoff_distance` / `final_clearance` 파라미터만 조정 | C팀 + 현장 오퍼레이터 |
| **10/21 ~ 10/22** | **C-4:A/B팀 최신 승인본 SHA 동기화 및 감독하(Supervised) 단계별 연동 시험** | - A팀(`LEFT_START_TO_LIFT`, `FULL_CHAIN_RECEIVE_CONTACT`) 및 B팀(`visual_servo` B19/B20) 확정 SHA Manifest 반영<br/>- 오퍼레이터 감독하에 `DETECT → NAVIGATE → ARRIVED → BASE_STOPPED → REOBSERVE → MANIPULATION_READY` 도달 후 콘솔 `WAITING_FOR_OPERATOR_APPROVAL` 대기 및 수동 `START` 승인 연동 시험 | A·B·C 합동 (오퍼레이터 수동 승인 필수) |
| **10/23 (금) 마감** | **C-5: 최종 통합 회귀 검증 및 릴리스 게이트 마감** | - 정상 흐름 / 관측 실패 / 실행 실패(STOP·FAULT·거부) 전 시나리오 최종 리포트 및 아카이브 고정 | C팀 최종 산출물 인도 |

---

## 3. 실물 통합 전 필수 선결 조건 체크리스트 (Field Readiness Gates)

1. **`[필수-1]` 베이스 정지 후 진동 감쇠 시간 확인**: TurtleBot3 도킹 정지(`cmd_vel = 0`) 직후 듀얼 암 마운트의 잔류 진동이 감쇠될 때까지 최소 `min_settle_duration_s >= 0.5s` (권장 `0.8s ~ 1.0s`) 대기 후 `REOBSERVE` 및 Sweepick `observe`를 호출할 것.
2. **`[필수-2]` Base–Arm TF 직접 투영 금지 유지**: 물리적 치구 고정 및 ChArUco/ArUco 동시 관측 외부 보정(Extrinsic Calibration)이 완료·인증되기 전까지 `HardwareReadinessState.base_to_arm_tf_calibrated = False`를 유지하고, 반드시 `INDEPENDENT_LOCAL_CAMERA_OBSERVATION` 모드를 사용할 것.
3. **`[필수-3]` 단일 Owner 및 수동 START 승인 유지**: 통합 노드는 어떠한 경우에도 `/dev/dapier/*` 시리얼/카메라 장치를 직접 열거나 `APPROVAL.json`을 자동 승인하지 않으며, `WAITING_FOR_OPERATOR_APPROVAL` 상태에서 현장 오퍼레이터의 콘솔 승인을 기다릴 것.
