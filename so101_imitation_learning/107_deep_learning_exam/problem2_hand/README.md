# 문제 2 — 웹캠에 반응하는 가상 손

문제 1에서 직접 촬영한 손 사진과 가상 관절 궤적을 연결해 RNN·LSTM·Transformer, ViT·ACT/CVAE를 구현하고 기존 v2 교사 실험을 수행했다. 이후 주먹·가위 렌더링과 상대 패 전환 반응의 문제를 확인해 그림의 기하, ACT 교사, 학습 시 잠재변수 사용 방식을 수정했다.

전체 [요구사항·설계·실행 명령](../README.md)과 [기존 평가 결과](../RESULTS.md)를 분리해 정리했다. 기존 실행 `cd9a368691de4b759ca03cfcb2876b31`의 30 Epoch 수치는 **mixed-open-start-v2 측정값**이며 최신 수정 성능으로 사용하지 않았다. v4 실행 `61dbebe471924c68ad73568ab479761c`의 20 Epoch 학습을 완료했고 `meta.json`의 `complete=true`를 확인했다. 최신 수치는 결과 문서의 해당 실행 ID에 연결했다. 학습 완료를 실제 웹캠 패 전환의 성공으로 표현하지 않았다.

## 데이터와 교사에서 변경한 내용

RNN·LSTM·Transformer의 비교용 데이터는 `make_episodes`의 **mixed-open-start-v2**를 그대로 보존했다. 72시점의 느린 궤적에서 과거 32시점 → 다음 상태를 학습하도록 구성했고, 초반 4시점의 값을 기억하는 별도 합성 진단도 유지했다.

ACT에는 v3에서 `make_action_episodes`를 추가했다. 각 사진을 **전관절 0·보·바위·가위·임의 자세의 5개 시작 상태**와 모두 짝지었다. 임의 자세는 관절마다 0~1.4 rad에서 뽑았고, 관절별 gain은 각 궤적마다 0.7~1.0에서 뽑아 고정했다.

```text
명령: a[t] = q[t] + gain × (goal − q[t])
상태: q[t+1] = q[t] + 0.35 × (a[t] − q[t])
없음: a[t] = q[t]
```

상대 가위에는 바위, 바위에는 보, 보에는 가위 자세를 교사 목표로 정했다. ACT 표본을 **0·2·6·20시점**에서 잘라 현재 이미지·현재 q·미래 8개 명령으로 구성했다. 사진 한 장에서 5 × 4 = 20개 표본이 나왔으며, 세션 단위 840/240/120장 분할을 보존해 ACT 표본은 16,800/4,800/2,400개가 됐다. 파생 표본 수를 독립 촬영 수로 해석하지 않았다.

v3 `c05774368ac741e6b1bc25b9ac3f0bc3`에서는 posterior 학습과 z=0 검증의 오차 차이를 확인한 뒤 ACT 15 Epoch에서 중단했다. v4에서는 학습용 posterior에서 μ·logvar와 재매개화 z를 만든 후 **표본마다 50% 확률로 z 전체를 0으로 교체**했다. 매 배치의 정확히 절반이 아니라 평균적으로 절반이다. 나머지 표본은 posterior z를 사용했고, KL은 전체 표본에 적용했다.

학습 손실은 재구성 MSE + 0.001 KL + 0.2 보조 분류 CE, Adam LR 0.0003·weight decay 0.0001로 유지했다. 검증·추론에는 미래 정답을 넣지 않고 z=0 또는 N(0,I) 표본을 사용하도록 구현했다. 메타데이터에 `teacher_version=feedback-prior-v4`, `sequence_teacher_version=mixed-open-start-v2`, `prior_zero_probability=0.5`를 기록했다.

v2와 v3/v4는 ACT 교사와 표본 구성이 달라 MSE를 동일한 시험셋의 성능 향상처럼 직접 비교하지 않았다. v3/v4 비교에서도 seed·데이터 분할·교사 생성 조건·학습 횟수·추론 z 조건을 함께 기록하도록 정리했다.

## 실행 방법

이 문서가 있는 `problem2_hand/`에서 실행한다.

```bash
python hand_exam.py serve --port 8766
```

<http://127.0.0.1:8766/hand>에서 완료 모델을 고르고 카메라 시작 → 가상 손 시작을 누른다. 첫 선택은 최신 완료 실행과 교사 버전이 같은 모델 중 검증 MSE가 가장 낮은 모델이다. 이미 선택한 모델은 상태 갱신 후에도 유지하므로 새 실험을 사용하려면 체크포인트를 확인한다. z 샘플링을 켜면 N(0,I), 끄면 z=0을 사용한다.

```bash
python hand_exam.py train --run runs/my-v4-experiment --epochs 20
# 위 학습이 완료된 뒤 실행
python evaluate_rollout.py --run runs/my-v4-experiment
python evaluate_transitions.py --run runs/my-v4-experiment
python test_hand.py
python test_rollout.py
node test_hand_ui.cjs
```

화면에는 분류 결과, 예측 명령 8×10, 갱신된 관절 상태를 구분해 표시했다. 8개 명령 중 앞 4개를 10Hz로 실행한 뒤 현재 q와 새 사진으로 다시 예측하도록 구현했다. 없음·신뢰도 70% 미만에서는 현재 자세를 유지하고, 정지·초기화 후 늦게 도착한 응답은 적용하지 않도록 처리했다. 실제 명령을 라벨별 정답 자세로 교체하지 않았다.

주먹·가위 그림은 두 링크의 cos 방향과 sin 깊이를 함께 투영하도록 수정했다. 검지·중지의 고정 벌어짐, 엄지의 굽힘면·겹침을 조정했고 정답 각도에서 실루엣을 확인했다. 라벨별 그림으로 바꾸는 대신 실제 q의 연속함수로 렌더링했다. 이 기하 검사는 최신 모델의 실시간 예측 성공과 구분했다.

## 결과 파일과 평가 범위

`runs/<ID>`에 source_manifest.json, meta.json, 모델별 학습 이력, train.log, 체크포인트, sequence_metrics.json, act_metrics.json, RESULTS.md, report.html을 저장하도록 구현했다. 완료 여부는 meta.json의 `complete`로 구분했다. 중단된 v3는 완료 결과로 취급하지 않았고, v4는 `complete=true`를 확인한 뒤 완료 실행으로 구분했다.

폐루프 평가 명령은 rollout_metrics.json과 rollout_trajectories.png를 추가한다. 기존 v2 평가는 테스트 사진 120장을 각각 고정한 상태에서 64틱씩 실행했다. 0.2 rad RMSE는 진단 기준으로 정했으며 공식 합격 기준이나 게임 승률로 사용하지 않았다. 바위 입력은 목표인 보 자세에 처음부터 가깝고 없음은 자세 유지가 목표라는 점도 구분했다. 이 기존 측정으로 v4의 패 전환 성능을 주장하지 않았다.

`evaluate_transitions.py`에는 테스트 사진 120장마다 주먹·보·가위 시작 자세를 적용해 360개 전환을 평가하도록 구현했다. 현재 q를 되먹임하며 앞 4개 명령 실행 후 재관찰하는 과정을 16회, 총 64틱 수행하도록 구성했다. 2초 내 도달은 20틱의 가상 시간 기준이고 처음부터 목표 근처인 사례는 별도로 제외한다. 각 전환 안에서는 사진을 고정하므로 실제 웹캠에서 연속으로 손을 바꾸는 시험과 구분했다. 결과는 `transition_metrics.json`과 `transition_summary.png`에 저장한다.

현재 화면은 가상 기구학 실습으로 구현했고 실물 장치나 물리 접촉 시뮬레이터에는 연결하지 않았다. 원시 사진·가중치는 공개 대상에서 제외했다. v4 학습 완료 후의 최신 수치는 전체 [결과 기록](../RESULTS.md)의 해당 실행 ID에서 확인하도록 연결했다. 실제 웹캠 패 전환 반응은 가상 평가와 별도로 확인할 대상으로 남겼다.
