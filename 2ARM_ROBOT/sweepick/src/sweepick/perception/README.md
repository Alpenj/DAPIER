# sweepick perception

OS30A·양손목 관측과 실제 feedback의 입력 준비. frame의 원래 stamp/sequence와 유효성을 보존하며 command와 measured를 구분한다. capture/reader 생성은 실제 장치를 열 수 있다.

| 파일 | 기존 대응 |
|---|---|
| [sweepick_feedback_observation.py](sweepick_feedback_observation.py) | sweepick_field_r6 |
| [sweepick_observation_capture.py](sweepick_observation_capture.py) | sweepick_field |
| [sweepick_observation_view.py](sweepick_observation_view.py) | sweepick_view |
| [sweepick_os30a_observation.py](sweepick_os30a_observation.py) | sweepick_sdk_top |
| [sweepick_wrist_capture.py](sweepick_wrist_capture.py) | sweepick_read03_capture |

공통 장치 writer는 `control/sweepick_trajectory_executor.py`의 기존 `run/grip`을 재사용한다. 이번 정리는 import·자산 경로와 이름만 변경한다. 시험·실행 범위는 [제품 README](../../../README.md)를 따른다.
