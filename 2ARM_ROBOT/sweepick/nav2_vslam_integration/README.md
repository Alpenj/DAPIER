# DAPIER C — VSLAM/NAV2 × Sweepick Visual Servo Integration (`nav2_vslam_integration`)

독립 C팀(Gemini 4 Argon)의 Navigation(`Dapier_project_visaul_slam`) × Manipulation(`DAPIER/2ARM_ROBOT/sweepick`) 통합 패키지입니다.

## 구성 파일

- [`contracts.py`](contracts.py): P0 8단계 파이프라인(`DETECT → NAVIGATE → ARRIVED → BASE_STOPPED → REOBSERVE → MANIPULATION_READY → ARM_TASK → RESULT`), P1 결과 분류(`WAITING_FOR_OPERATOR_APPROVAL`, `COMPLETED`, `PARTIAL_COMPLETED`, `OBSERVATION_FAILED`, `EXECUTION_REJECTED`, `STOP`, `FAULT`), TF2 좌표계 및 `HardwareReadinessState` 계약.
- [`reobserve_gate.py`](reobserve_gate.py): `BaseStopVerifier` 및 `PostStopReobserveGate` (이동 전 좌표 재사용, 주행 중 EMA 필터 이력, 0.5초 Persistence Latch, 180초 Qwen BBox Latch 차단).
- [`sweepick_adapter.py`](sweepick_adapter.py): 기존 Sweepick Console(`control.py`)의 승인된 실행 범위(`FULL_CHAIN_RECEIVE_CONTACT`, `LEFT_START_TO_LIFT`, `RIGHT_X_FINITE_METROLOGY`)를 무권한(`motor_write_authorized=False`)으로 호출·평가하는 어댑터.
- [`pipeline_coordinator.py`](pipeline_coordinator.py): Navigation과 Manipulation을 연결하고 `STOP`/`FAULT`/`EXECUTION_REJECTED` 시 후속 명령을 차단하며 Task Manager로 오류를 전파하는 상태 기계 코디네이터.
- [`navigation_bridge.py`](navigation_bridge.py): Navigation 저장소의 `command_validator.py`, `station_manager.py`, `semantic_waypoints.yaml`, `docking_profiles.yaml`, `docking_stations.yaml`, `vslam_navigation.launch.py` 광학 TF 체인을 직접 재사용하는 브리지.

## C-2 (2026-10-16, Argon C-1 인수) 추가 파일

- [`tf_provenance.py`](tf_provenance.py): `VerifiedTransform`(TF2 lookup 결과만 REAL 등급, 상수는 MOCK), `DetectionProvenance`(센서 시각·EMA/latch·anchor 출처·inlier·RMS), `SensorFrameLedger`(같은 센서 프레임 재발행 거부), `build_post_stop_observation`(출처·TF 미확인 시 `POST_STOP_OBSERVATION_UNVERIFIED` / `TF_UNAVAILABLE_AT_MESSAGE_TIME` 거부).
- [`ros2_readonly_adapter.py`](ros2_readonly_adapter.py): ROS 2 Jazzy 읽기 전용 어댑터. `ReadOnlyGraphState`(rclpy 불필요, 단위시험 대상) + `SweepickNavReadOnlyNode`(구독만, publisher·action client·service client 0). 기존 토픽 타입·QoS·TF 프레임 재사용.
- [`run_ros2_readonly_dryrun.py`](run_ros2_readonly_dryrun.py): dry-run 실행기(`--mock-graph`로 라벨된 MOCK 그래프 사용). 파이프라인은 MANIPULATION_READY까지만, ARM_TASK 요청 0.
- [`mock_ros2_graph_publisher.py`](mock_ros2_graph_publisher.py): 같은 토픽/타입/QoS의 MOCK 발행 노드(`sweepick_c2_mock_graph`). 로봇 네트워크에서 실행 금지.
- `sweepick_adapter.py`: 결과 분류 수정 — Motor/Bus FAULT > 운영자 STOP > 관측 실패 우선순위, 세 증거를 결과에 모두 보존(`hardware_fault`, `operator_stop`, `observation_failure`). `owner_stop` 이벤트만으로는 STOP이 아님.
- `pipeline_coordinator.py`: `confirm_operator_recovery(..., console_status=)` — Console 기원 차단은 Console의 `MANUAL_RECOVERY_CONFIRMATION` 기록(`active.operator_recovery`)과 `can_prepare`로만 해제, Boolean 단독 거부.
- `navigation_bridge.py`: 하드코딩 카메라 원점 기본값 제거(`camera_link_in_base_footprint_m` 필수 인자), 출처·TF 필수.
- 시험: `tests/test_c2_*.py` 3개 파일(실물 `~/sweepick_console_runs/*/RUN_RECORD.json` 읽기 전용 사례 포함).

## 산출물 보고서

1. [두 저장소 연결 가능성 및 인터페이스 감사](reports/C_VSLAM_SWEEPICK_INTERFACE_AUDIT.md)
2. [Mock End-to-End 회귀시험 결과 JSON](reports/C_MOCK_E2E_REGRESSION_REPORT.json)
3. [기존 주행·도킹 기능 회귀시험 결과 JSON](reports/C_NAV2_DOCKING_REGRESSION_REPORT.json)
4. [10월 23일 마감 통합 로드맵 및 미검증 실물 조건](reports/C_OCT23_INTEGRATION_ROADMAP_AND_HARDWARE_GATES.md)

## 테스트 실행 방법

```bash
# pydantic(Navigation command_validator)·numpy·yaml·rclpy(/opt/ros/jazzy)이 보이는 인터프리터로 실행
python -m unittest discover -s 2ARM_ROBOT/sweepick/nav2_vslam_integration/tests -p "test_*.py" -v
# ROS 2 Jazzy 읽기 전용 dry-run (MOCK 그래프)
python 2ARM_ROBOT/sweepick/nav2_vslam_integration/run_ros2_readonly_dryrun.py --duration 12 --mock-graph --out /tmp/dryrun.json
```
