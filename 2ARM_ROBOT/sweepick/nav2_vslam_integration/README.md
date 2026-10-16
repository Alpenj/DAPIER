# DAPIER C — VSLAM/NAV2 × Sweepick Visual Servo Integration (`nav2_vslam_integration`)

독립 C팀(Gemini 4 Argon)의 Navigation(`Dapier_project_visaul_slam`) × Manipulation(`DAPIER/2ARM_ROBOT/sweepick`) 통합 패키지입니다.

## 구성 파일

- [`contracts.py`](contracts.py): P0 8단계 파이프라인(`DETECT → NAVIGATE → ARRIVED → BASE_STOPPED → REOBSERVE → MANIPULATION_READY → ARM_TASK → RESULT`), P1 결과 분류(`WAITING_FOR_OPERATOR_APPROVAL`, `COMPLETED`, `PARTIAL_COMPLETED`, `OBSERVATION_FAILED`, `EXECUTION_REJECTED`, `STOP`, `FAULT`), TF2 좌표계 및 `HardwareReadinessState` 계약.
- [`reobserve_gate.py`](reobserve_gate.py): `BaseStopVerifier` 및 `PostStopReobserveGate` (이동 전 좌표 재사용, 주행 중 EMA 필터 이력, 0.5초 Persistence Latch, 180초 Qwen BBox Latch 차단).
- [`sweepick_adapter.py`](sweepick_adapter.py): 기존 Sweepick Console(`control.py`)의 승인된 실행 범위(`FULL_CHAIN_RECEIVE_CONTACT`, `LEFT_START_TO_LIFT`, `RIGHT_X_FINITE_METROLOGY`)를 무권한(`motor_write_authorized=False`)으로 호출·평가하는 어댑터.
- [`pipeline_coordinator.py`](pipeline_coordinator.py): Navigation과 Manipulation을 연결하고 `STOP`/`FAULT`/`EXECUTION_REJECTED` 시 후속 명령을 차단하며 Task Manager로 오류를 전파하는 상태 기계 코디네이터.
- [`navigation_bridge.py`](navigation_bridge.py): Navigation 저장소의 `command_validator.py`, `station_manager.py`, `semantic_waypoints.yaml`, `docking_profiles.yaml`, `docking_stations.yaml`, `vslam_navigation.launch.py` 광학 TF 체인을 직접 재사용하는 브리지.

## 산출물 보고서

1. [두 저장소 연결 가능성 및 인터페이스 감사](reports/C_VSLAM_SWEEPICK_INTERFACE_AUDIT.md)
2. [Mock End-to-End 회귀시험 결과 JSON](reports/C_MOCK_E2E_REGRESSION_REPORT.json)
3. [기존 주행·도킹 기능 회귀시험 결과 JSON](reports/C_NAV2_DOCKING_REGRESSION_REPORT.json)
4. [10월 23일 마감 통합 로드맵 및 미검증 실물 조건](reports/C_OCT23_INTEGRATION_ROADMAP_AND_HARDWARE_GATES.md)

## 테스트 실행 방법

```bash
python -m unittest discover -s 2ARM_ROBOT/sweepick/nav2_vslam_integration/tests -p "test_*.py" -v
```
