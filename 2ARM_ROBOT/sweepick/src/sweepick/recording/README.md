# sweepick recording

기존 3-view·양팔 측정값·명령 event의 연속 episode recorder. 센서 reader를 빌릴 수 있으며 성공·실물 support를 추론하여 채우지 않는다.

| 파일 | 기존 대응 |
|---|---|
| [sweepick_episode_recorder.py](sweepick_episode_recorder.py) | sweepick_episode |

공통 장치 writer는 `control/sweepick_trajectory_executor.py`의 기존 `run/grip`을 재사용한다. 이번 정리는 import·자산 경로와 이름만 변경한다. 시험·실행 범위는 [제품 README](../../../README.md)를 따른다.
