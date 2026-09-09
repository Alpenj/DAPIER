# 문제 2 — 웹캠에 반응하는 가상 손

나는 문제 1의 실제 손 사진과 가상 관절 궤적을 연결해 RNN·LSTM·Transformer와 ViT·ACT/CVAE를 비교한다. [전체 요구사항·설계·실행 명령](../README.md)과 [실제 평가 결과](../RESULTS.md)를 함께 본다.

이 문서가 있는 `problem2_hand/`에서 실행한다.

```bash
python hand_exam.py serve --port 8766
```

<http://127.0.0.1:8766/hand>에서 학습 완료 모델을 고르고 카메라 시작 → 가상 손 시작을 누른다. 첫 선택은 가장 최신 실행과 교사 버전이 같은 완료 모델 중 검증 MSE가 가장 낮은 모델이며, 이미 선택한 모델은 상태 갱신 후에도 유지한다. z 샘플링을 켜면 N(0,I), 끄면 z=0을 사용한다. 분류 결과와 명령 8×10, 실제 갱신된 관절 상태를 구분해 표시한다. 이 화면은 기구학 실습이며 실물 장치나 물리 접촉 시뮬레이터가 아니다.

```bash
python hand_exam.py train --run runs/my-experiment --epochs 30
python evaluate_rollout.py --run runs/my-experiment
python test_hand.py
python test_rollout.py
node test_hand_ui.cjs
```

교사 데이터는 `make_episodes`로 재생성한다. 현재 버전은 `mixed-open-start-v2`이며, 초기의 임의 자세만 사용한 실행과 생성 조건이 다르다. 버전이 다른 실행의 MSE를 동일한 시험셋에서의 순수 성능 개선으로 비교하지 않는다.

`runs/<ID>`에는 source_manifest.json, meta.json, 모델별 학습 이력, train.log, 체크포인트, sequence_metrics.json, act_metrics.json, RESULTS.md, report.html이 저장된다. 폐루프 평가 명령은 rollout_metrics.json과 rollout_trajectories.png를 추가한다. 이 평가는 테스트 사진 120장을 고정한 상태에서 각 64틱을 실행한다. 0.2 rad RMSE 기준은 진단 기준이며 공식 합격 기준이나 게임 승률이 아니다. 처음부터 목표 근처인 바위 입력과 없음의 수치도 별도로 해석한다.

원시 사진과 가중치는 공개 저장소에 포함하지 않는다. 현재 교사 버전의 최종 학습·평가 완료 여부와 수치는 전체 [결과 기록](../RESULTS.md)에 근거를 함께 남긴다.
