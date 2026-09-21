# 웹캠 가위바위보와 가상 로봇 손 — 딥러닝 실습 기록

record_id: `DAPIER-2026-09-09-rps-virtual-hand`

직접 촬영한 가위·바위·보·없음 이미지 1,200장으로 MLP·CNN·ResNet18 동결·파인튜닝을 각각 10 Epoch 학습하고 평가했다. 문제 1 웹캠 게임도 직접 실행해 동작을 확인했다. 같은 이미지에 가상 손의 관절 궤적을 연결해 문제 2의 RNN·LSTM·Transformer와 ViT·ACT/CVAE를 구현하고 기존 v2 교사로 30 Epoch 실험을 수행했다.

이후 가상 손의 주먹·가위가 불분명하고 상대 패를 바꿔도 충분히 반응하지 않는 문제를 확인했다. 렌더링의 굽힘 깊이 표현을 고쳤고, ACT의 교사를 현재 관절 상태에 반응하는 방식으로 변경했다. 정답 청크를 본 잠재변수로만 학습하는 조건과 실제 z=0 추론 조건의 차이도 줄이도록 학습 경로를 수정했다. **v4 실행 `61dbebe471924c68ad73568ab479761c`의 20 Epoch 학습을 완료했고, `meta.json`의 `complete=true`를 확인했다. 최신 수치는 결과 문서에서 이 실행 ID의 기록을 참고한다. 학습 완료와 실제 웹캠에서의 패 전환 성공은 별도로 구분했다.**

기존 측정 수치와 해석은 [결과 및 수행 과정](RESULTS.md)에 기록했다. 그중 `cd9a368691de4b759ca03cfcb2876b31`의 문제 2 수치는 **기존 mixed-open-start-v2 실행**의 결과이며 최신 v4 성능을 뜻하지 않는다. 원시 손 사진과 가중치는 공개 대상에서 제외했다. 재현용 촬영 자료와 선택한 실행 기록은 비공개 제출 자료로 분리했다.

## 시험 요구별로 구현한 내용

| 문항 | 구현한 내용 | 코드·산출물 |
|---|---|---|
| 1-1 | Custom Dataset, train 전용 증강·정규화, 세션 단위 7:2:1 분할 | `problem1_rps/rps_exam.py`: HandDataset·make_manifest·image_transform |
| 1-2 | 직접 만든 MLP와 3개 Conv CNN, CrossEntropy·Adam, 각각 10 Epoch 학습 | MLP·CNN·train / 학습 곡선 |
| 1-3 | 사전학습 ResNet18의 fc를 4출력으로 교체, 백본 동결과 전체 파인튜닝 | build_model·make_optimizer·train_mode / 두 방식의 지표 |
| 1-4 | 검증 기준 가중치 선택 후 held-out Accuracy·혼동행렬·Latency/FPS 평가 | metrics.json·learning_curves.png |
| 2-1 | 10관절의 과거 32시점에서 다음 상태를 예측하는 RNN·LSTM 회귀 | `problem2_hand/hand_exam.py`: SequenceDataset·StatePredictor |
| 2-2 | 4-head Attention, sin/cos 위치 인코딩, Transformer encoder 2층·decoder 2층 | 다음 상태 MSE와 지연 기억 진단 |
| 2-3 | 64×64 입력의 8×8 패치 64개, Linear 투영·CLS·위치 임베딩 | PatchEmbedding·TinyViT, 총 65토큰 |
| 2-4 | 재매개화·KL·ViT·현재 상태로 8×10 미래 명령 출력, v4에서 z=0 학습 경로 추가 | ActionCVAE·make_action_episodes / action_chunks.png·rollout_trajectories.png |

문제 2의 출력을 엄지·검지·중지·약지·소지 각각 두 관절, 총 10개의 연속 각도 명령으로 구현했다. 손 모양 이름을 예측하는 보조 분류와 미래 액션 청크를 구분했다.

## 실행 환경과 재현 명령

Python 3.12.3, PyTorch 2.11.0+cu130, torchvision 0.26.0+cu130, Pillow 12.3.0, matplotlib 3.10.9, NVIDIA RTX 5050 Laptop 8GB에서 실행했다. 기존 Python 가상환경을 재사용했고 전역 패키지는 변경하지 않았다. ROS와 실물 로봇은 사용하지 않았다.

이 문서가 있는 `107_deep_learning_exam/`에서 의존성이 설치된 Python으로 실행한다. 새 환경의 의존성은 [requirements.txt](problem1_rps/requirements.txt)를 참고한다.

```bash
python problem2_hand/hand_exam.py serve --port 8766
```

- 문제 1 촬영·분류 게임: <http://127.0.0.1:8766/>
- 문제 2 가상 손: <http://127.0.0.1:8766/hand>
- 카메라는 화면의 시작 버튼으로 켠다. 서버는 Ctrl+C로 종료한다.

```bash
python problem1_rps/rps_exam.py train --data problem1_rps/data --run problem1_rps/runs/reproduce --epochs 10
python problem2_hand/hand_exam.py train --data problem1_rps/data --run problem2_hand/runs/reproduce-v4 --epochs 20
python problem2_hand/evaluate_rollout.py --run problem2_hand/runs/reproduce-v4
python problem2_hand/evaluate_transitions.py --run problem2_hand/runs/reproduce-v4
```

명령은 차례대로 실행하고, 폐루프 평가는 학습 완료 후 실행한다. `evaluate_rollout.py`는 전관절 0에서 시작하고, `evaluate_transitions.py`는 완성된 주먹·보·가위의 세 자세에서 새 상대 사진 조건으로 전환한다. 테스트 사진 120장 × 시작 자세 3개, 총 360개 가상 전환을 평가한다. 각 전환에서는 사진을 고정하므로 실제 웹캠의 연속 영상 시험과 구분한다. 기존 결과 폴더를 덮어쓰지 않으므로 재실행 때 새 경로를 지정한다. 브라우저에서 만든 32자리 실행 ID는 화면에서 선택할 수 있고, CLI 사용자 지정 이름의 보고서는 해당 폴더에서 직접 연다.

## 촬영 데이터 분할과 문제 1 실험

같은 회차의 연속 사진이 학습과 테스트에 섞이지 않도록 10회차 × 4종류 × 30장, 총 1,200장을 **회차 단위**로 학습 840장·검증 240장·테스트 120장에 분리했다. 클래스별 개수는 210·60·30장이었다. 동일한 파일 내용이 분할을 가로지르면 학습을 거부하도록 구현했다. 테스트는 한 회차뿐이므로 다른 사용자·장소에 대한 일반화는 아직 확인하지 못했다.

네 모델에 128×128 입력과 같은 정규화를 적용했다. CNN은 공간 구조를 보존하면서 MLP보다 적은 파라미터를 사용하도록 구성했다. 동결 ResNet은 백본 파라미터와 BatchNorm 통계를 함께 고정했다. 파인튜닝에는 백본 LR 0.0001·fc LR 0.001을 적용했고, 나머지 모델에는 Adam LR 0.001을 사용했다. weight decay는 모두 0.0001로 설정했다. 작은 입력·증강·학습률의 개선 효과를 대조 실험 없이 단정하지 않았다.

## 문제 2의 관절 상태와 교사 명령

상대 손 이미지는 실제 웹캠 사진을 사용했지만 **관절 상태와 정답 액션은 가상 교사로 생성했다.** 실제 로봇의 동기화 센서 기록은 사용하지 않았다. 사진 한 장을 각 에피소드의 고정 관측으로 두고 상태 `q[t]`와 명령 `a[t]`를 같은 시간축으로 생성했다. 이미지 라벨은 교사 생성과 보조 분류 학습에만 사용했으며 추론 입력에는 정답 라벨이나 미래 액션을 넣지 않았다.

관절 범위는 0~1.4 rad로 정했고, 10Hz 기구학 갱신을 다음 식으로 구현했다.

```text
q[t+1] = q[t] + 0.35 × (a[t] − q[t])
```

가위 입력에는 바위 자세, 바위 입력에는 보 자세, 보 입력에는 가위 자세를 목표로 정했다. 없음은 현재 자세를 유지하도록 구성했다. 목표 규칙은 **교사 정답 생성**에 사용했고 실제 추론 출력은 학습 모델이 만든 연속 명령을 그대로 사용하도록 유지했다.

### RNN·LSTM·Transformer: v2 시퀀스 교사 보존

세 모델의 비교 조건을 보존하기 위해 `make_episodes`의 **mixed-open-start-v2** 교사를 유지했다. 에피소드 절반은 전관절 0에서, 나머지는 임의 굽힘 자세에서 시작하도록 구성했다. 72시점 궤적에 36~64시점의 전이 시간, 부드러운 보간, 작은 중간 경로 변화를 넣었다.

일반 회귀 창은 시점 31·39·47·55·63에서 잘랐다. 각 창은 과거 32개 상태와 바로 다음 상태 정답으로 구성했다. 별도 장기기억 진단에서는 초반 4시점에만 기억할 값을 제시하고 뒤를 공통 중립값으로 채웠다. 32시점 입력과 마지막 8시점만 남긴 입력의 MSE를 분리해 보고하도록 구성했다. 짧은 입력 진단에는 정보 손실과 길이·분포 변화가 함께 있으므로 Transformer의 보편적 우월성을 주장하지 않았다.

### ACT v3: 현재 상태에서 목표로 접근하는 교사

기존의 느린 시간 보간 궤적을 매번 초반부터 흉내 내는 대신, 현재 관절 상태에서 새 목표를 향하도록 `make_action_episodes`를 추가했다. 사진마다 다음 **5개 시작 자세**를 모두 연결했다.

1. 전관절 0인 열린 자세
2. 보 자세: 전관절 0.05 rad
3. 바위 자세: 손가락마다 1.05 / 0.95 rad
4. 가위 자세: 검지·중지는 0.05 / 0.05 rad, 나머지는 1.05 / 0.95 rad
5. 각 관절을 0~1.4 rad에서 뽑은 임의 자세

각 궤적에서 관절별 gain을 0.7~1.0 범위로 한 번 뽑아 고정하고 아래 식으로 72시점의 명령·상태를 생성했다. 없음은 `a[t] = q[t]`로 처리했다.

```text
a[t] = q[t] + gain × (goal − q[t])
q[t+1] = q[t] + 0.35 × (a[t] − q[t])
```

ACT 표본은 전이 초반을 포함하도록 **0·2·6·20시점**에서 추출했다. 각 표본을 현재 사진·현재 q·그 시점부터의 미래 8개 명령으로 구성했다. 사진당 5개 시작 × 4개 시점 = 20개 표본을 만들었으며, 이미지의 기존 세션 분할은 유지했다. 따라서 ACT 표본 수는 학습 16,800개·검증 4,800개·테스트 2,400개다. 같은 사진에서 파생한 표본이므로 이를 새로운 독립 촬영 수로 세지 않았다.

### ACT v4: 실제 추론의 z=0 조건도 학습

v3 실행에서 posterior를 사용하는 학습 오차에 비해 z=0 검증 오차가 큰 문제를 확인했다. `c05774368ac741e6b1bc25b9ac3f0bc3`는 30 Epoch 계획 중 ACT 15 Epoch까지 진단한 뒤 중단했고, 완료 실험으로 취급하지 않았다.

v4에서는 먼저 정답 청크를 읽은 posterior가 μ·logvar를 만들고 `z = μ + exp(0.5 × logvar) × ε`로 표본을 생성하도록 유지했다. 그 뒤 **학습 표본마다 50% 확률로 z 벡터 전체를 0으로 바꿨다.** 평균적으로 절반이며 매 배치의 정확히 절반을 고정한 것은 아니다. 나머지는 posterior 표본을 사용했고 KL 손실은 전체 표본의 μ·logvar에 적용했다.

손실은 재구성 MSE + 0.001 KL + 0.2 보조 이미지 분류 CE, Adam LR은 0.0003, weight decay는 0.0001로 유지했다. 평가에서는 미래 정답 없이 z=0과 N(0,I) 표본을 각각 사용하도록 분리했다. 이 변경은 배포 조건을 학습에 포함하기 위한 수정이며, v4 최종 평가 전에는 반응성이나 정확도 개선을 확정하지 않았다.

메타데이터의 `teacher_version`은 `feedback-prior-v4`, `sequence_teacher_version`은 `mixed-open-start-v2`, `prior_zero_probability`는 0.5로 기록하도록 수정했다. v2와 v3/v4는 ACT 교사와 평가 표본이 다르므로 chunk MSE를 같은 시험셋의 순수 성능 향상처럼 직접 비교하지 않았다. v3와 v4를 비교할 때도 seed·분할·생성 조건·학습 횟수·추론 z 조건을 함께 확인하도록 기록했다.

## 화면에서 실행한 명령과 렌더링 수정

화면은 8개 명령 중 앞 4개를 실행한 뒤 바뀐 q와 새 사진으로 재관찰하도록 구현했다. 분류 신뢰도 70% 미만 또는 없음이면 현재 자세를 유지했고, 숨긴 탭·중지·오류에서는 진행 중 명령을 취소하도록 처리했다. 실제 명령을 목표 규칙으로 덮어쓰지는 않았다.

기존 그림에서는 굽힘의 깊이가 사라져 말단 링크가 한 직선 위에서 뒤집히는 문제가 있었다. 링크 길이를 유지한 채 cos 성분과 sin 깊이 성분을 비스듬히 투영하도록 바꿨다. 검지·중지의 고정 벌어짐과 엄지의 별도 굽힘면을 적용했고, 엄지를 마지막에 그려 손바닥 앞의 접힘이 보이도록 수정했다. **교사 정답 각도**로 주먹·가위 실루엣을 확인했으며, 모든 그림 좌표가 실제 q의 연속함수로 바뀌는 것도 검사했다. 이 검사는 모델이 실시간으로 올바른 각도를 예측했다는 증거와 구분했다.

원 ACT 전체를 재현했다고 표현하지 않았다. CNN 대신 직접 만든 ViT와 보조 분류기를 사용한 교육용 구성으로 기록했고 시간 중첩 앙상블은 추가하지 않았다.

## 검증 명령과 남은 확인

```bash
python problem1_rps/test_rps.py --smoke-training
node problem1_rps/test_ui.cjs
python problem2_hand/test_hand.py --smoke-training
python problem2_hand/test_rollout.py
node problem2_hand/test_hand_ui.cjs
```

합성 1 Epoch 실행 검사는 제출 성능에서 제외했다. 분할·시간 정렬·패치 순서·posterior gradient·정답 누출 방지·잘못된 HTTP 입력·관절 범위·중지 동작을 검사하도록 테스트를 구성했다. 최신 검사에는 5개 시작 자세의 피드백 교사, z=0/posterior 두 학습 경로, 주먹·가위의 정답 실루엣과 각도 연속성도 추가했다.

v4 학습 완료를 확인한 뒤 오프라인 청크 평가와 가상 자세 전환 평가를 분리해 기록하도록 정리했다. 최신 측정 수치는 [결과 및 수행 과정](RESULTS.md)의 v4 실행 기록을 따른다. 실제 웹캠에서 새 패를 보여준 뒤의 반응은 별도 확인 대상으로 남겼다. 실물 로봇 구동·다른 사용자와 장소의 일반화·사람 상대 정량 승률 실험은 수행하지 않았다.

## 참고한 공식 자료

- [torchvision 0.26 ResNet18](https://docs.pytorch.org/vision/0.26/models/generated/torchvision.models.resnet18.html)
- [PyTorch 전이학습 튜토리얼](https://docs.pytorch.org/tutorials/beginner/transfer_learning_tutorial.html)
- [PyTorch 2.11 Transformer](https://docs.pytorch.org/docs/2.11/generated/torch.nn.Transformer.html)
- [ViT 원 논문](https://arxiv.org/abs/2010.11929)
- [ACT 원 논문](https://arxiv.org/abs/2304.13705)
