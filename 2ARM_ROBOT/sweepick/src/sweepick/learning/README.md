# sweepick 학습·추론 코드 위치

이 폴더에는 학습 구현을 복사하거나 새로 만들지 않는다. sweepick의 데이터·학습·추론 코드는 용도와 검증 범위가 다른 기존 경로에 있으며, 아래 링크가 그 경계를 기록한다.

| 구분 | 실제 기존 위치 | 범위 |
|---|---|---|
| 현재 ACT/HYBRID 연결부 | [`sweepick_observation_session.py`](../integration/sweepick_observation_session.py), [`sweepick_receive_runtime.py`](../integration/sweepick_receive_runtime.py) | `FACTORY_ROOT`로 지정하는 외부 runtime의 `dapier_act_policy`와 `sim_data_factory`를 불러오는 adapter다. 정책·학습기·checkpoint 자체는 이 공개 package에 포함하지 않는다. |
| LeRobot ACT 기준 학습 | [`train_act_baseline.py`](../../../../scripts/train_act_baseline.py) | 실제 녹화 episode로 수행한 기준 학습 스크립트다. 실행 조건과 당시 결과는 [`ACT_BASELINE_20260907.md`](../../../../docs/ACT_BASELINE_20260907.md)에 보존한다. |
| 보류 데이터 추론·평가 | [`evaluate_act_baseline.py`](../../../../scripts/evaluate_act_baseline.py) | 저장 checkpoint를 읽는 offline 평가다. 폐루프 실물 성공을 뜻하지 않는다. |
| SIM ACT 정책·폐루프 | [`dapier_act_policy.py`](../../../../sim/mobile_dual_so101/dapier_act_policy.py), [`parallel_tabletop_act.py`](../../../../sim/mobile_dual_so101/parallel_tabletop_act.py) | LeRobot 계열 정책과 MuJoCo 연결이다. SIM 결과를 REAL 장치 검증으로 승격하지 않는다. |
| 기존 native ACT | [`dapier_native_act.py`](../../../../src/shoe_sorting_data/shoe_sorting_data/dapier_native_act.py), [`native_act_rollout.py`](../../../../src/shoe_sorting_data/shoe_sorting_data/native_act_rollout.py) | 기존 ROS package `shoe_sorting_data`에 남아 있는 legacy 학습·추론 경로다. 공식 ACT checkpoint 호환이나 전체 양팔 실물 성공을 주장하지 않는다. |

외부 runtime 연결부, 공개 SIM 정책과 기존 `shoe_sorting_data` native ACT는 서로 대체할 수 있는 같은 구현이 아니다. 현재 통합 진입점의 기본 PICK 경로도 sensor-planned이며 ACT를 사용하지 않는다.

저장소에는 `scripts/train_dual_so101_act.sh`가 존재하지 않으므로 링크나 대체 파일을 만들지 않았다. 현재 공개된 전체 양팔 ACT 성공 결과도 없다. 학습을 다시 실행하거나 checkpoint·원시 episode를 이 구조 정리에 포함하지 않는다.
