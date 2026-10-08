# sweepick manipulation

기존 teacher를 쓰는 오른 RECEIVE 계획·dispatch와 로컬 Owner/Release/Downstream 연결. 들린 pose와 원래 책상 pose를 구분하고, support 확인 전 왼 full release를 허용하지 않는 기존 계약을 유지한다.

| 파일 | 기존 대응 |
|---|---|
| [sweepick_bimanual_handover.py](sweepick_bimanual_handover.py) | sweepick_full_chain |
| [sweepick_right_receive_dispatch.py](sweepick_right_receive_dispatch.py) | sweepick_261008_receive_entry |
| [sweepick_right_receive_planner.py](sweepick_right_receive_planner.py) | sweepick_261008_receive_plan |

공통 장치 writer는 `control/sweepick_trajectory_executor.py`의 기존 `run/grip`을 재사용한다. 이번 정리는 import·자산 경로와 이름만 변경한다. 시험·실행 범위는 [제품 README](../../../README.md)를 따른다.
