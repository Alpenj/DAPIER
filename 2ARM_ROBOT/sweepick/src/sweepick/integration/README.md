# sweepick integration

실제 session 진입점, snapshot adapter, 상태·소유권·applied ledger·후단 wrapper. 합성 입력의 통과와 실제 controller 적용·REAL 준비 상태를 구분한다.

| 파일 | 기존 대응 |
|---|---|
| [sweepick_controller_bindings.py](sweepick_controller_bindings.py) | sweepick_261008_concrete_bindings |
| [sweepick_handoff_snapshot_adapter.py](sweepick_handoff_snapshot_adapter.py) | sweepick_261008_receive_real_adapter |
| [sweepick_manipulation_handoff.py](sweepick_manipulation_handoff.py) | sweepick_261008_full_episode |
| [sweepick_manipulation_session.py](sweepick_manipulation_session.py) | sweepick_pick01 |
| [sweepick_observation_session.py](sweepick_observation_session.py) | sweepick_local_loop |
| [sweepick_receive_runtime.py](sweepick_receive_runtime.py) | sweepick_261008_receive_runtime |
| [sweepick_receive_session.py](sweepick_receive_session.py) | sweepick_261008_receive_session |
| [sweepick_replay_rig.py](sweepick_replay_rig.py) | sweepick_real_rig |
| [sweepick_resource_paths.py](sweepick_resource_paths.py) | 기능별 경로·source ledger 대응만 관리 |

공통 장치 writer는 `control/sweepick_trajectory_executor.py`의 기존 `run/grip`을 재사용한다. 이번 정리는 import·자산 경로와 이름만 변경한다. 시험·실행 범위는 [제품 README](../../../README.md)를 따른다.
