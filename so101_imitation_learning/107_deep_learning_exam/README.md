# 웹캠 가위바위보와 가상 로봇 손 — 딥러닝 실습 기록

record_id: `DAPIER-2026-09-09-rps-virtual-hand`

나는 직접 촬영한 가위·바위·보·없음 이미지로 비전 모델 네 가지를 비교하고, 같은 이미지에 가상 손의 관절 궤적을 연결해 시퀀스 모델과 ViT·ACT/CVAE를 실습한다. 문제 1의 웹캠 게임은 직접 실행해 동작을 확인했다. 문제 2는 가상 관절 명령을 예측하고 기구학 식으로 화면의 손을 움직인다.

실제 수치와 해석은 [결과 및 수행 과정](RESULTS.md)에 기록한다. 원시 손 사진과 가중치는 공개 저장소에 포함하지 않는다. 아래 명령으로 직접 촬영·재학습할 수 있으며, 비공개 제출 묶음에는 재현용 촬영 자료와 선택한 실행 기록을 포함한다.

## 시험 요구를 어떻게 구현하는가

| 문항 | 내가 구현한 내용 | 확인할 코드·산출물 |
|---|---|---|
| 1-1 | Custom Dataset, train 전용 증강, 정규화, 세션 단위 7:2:1 분할 | `problem1_rps/rps_exam.py`: HandDataset·make_manifest·image_transform |
| 1-2 | 직접 만든 MLP, 3개 Conv CNN, CrossEntropy·Adam, 각각 10 Epoch | 같은 파일: MLP·CNN·train / 학습 곡선 |
| 1-3 | 사전학습 ResNet18의 fc를 4출력으로 교체, 백본 동결과 전체 파인튜닝 | build_model·make_optimizer·train_mode / 두 방식의 지표 |
| 1-4 | 검증 기준 체크포인트 선택, held-out Accuracy·혼동행렬·Latency/FPS | metrics.json·learning_curves.png |
| 2-1 | 10관절 연속 궤적에서 길이32 창을 8시점 간격으로 이동, 다음 상태 회귀 | `problem2_hand/hand_exam.py`: SequenceDataset·StatePredictor |
| 2-2 | 4-head Attention, sin/cos 위치 인코딩, Transformer encoder 2층·decoder 2층 | 다음 상태 MSE와 지연 기억 실험 |
| 2-3 | 64×64 입력을 8×8 패치 64개로 분할, Linear 투영·CLS·위치 임베딩 | PatchEmbedding·TinyViT, 총 65토큰 |
| 2-4 | 재매개화 z 샘플링·KL·ViT·현재 상태로 8×10 미래 관절 명령 예측 | ActionCVAE, action_chunks.png·rollout_trajectories.png |

가위바위보의 **다음 이름을 맞히는 분류**를 액션 청킹으로 부르지 않는다. 문제 2 출력은 엄지·검지·중지·약지·소지 각각 두 관절, 총 10개의 연속 각도 명령이다.

## 실행 환경과 명령

실제 실행 환경은 Python 3.12.3, PyTorch 2.11.0+cu130, torchvision 0.26.0+cu130, Pillow 12.3.0, matplotlib 3.10.9, NVIDIA RTX 5050 Laptop 8GB다. 기존 Python 가상환경을 재사용하고 전역 패키지를 변경하지 않았다. ROS와 실물 장치는 사용하지 않는다.

이 문서가 있는 `107_deep_learning_exam/`에서 의존성이 설치된 Python으로 실행한다. 새 환경의 의존성은 [requirements.txt](problem1_rps/requirements.txt)를 참고한다.

```bash
python problem2_hand/hand_exam.py serve --port 8766
```

- 문제 1 촬영·분류 게임: <http://127.0.0.1:8766/>
- 문제 2 가상 손: <http://127.0.0.1:8766/hand>
- 카메라는 화면의 시작 버튼으로 켠다. 서버는 Ctrl+C로 종료한다.

```bash
python problem1_rps/rps_exam.py train --data problem1_rps/data --run problem1_rps/runs/reproduce --epochs 10
python problem2_hand/hand_exam.py train --data problem1_rps/data --run problem2_hand/runs/reproduce --epochs 30
python problem2_hand/evaluate_rollout.py --run problem2_hand/runs/reproduce
```

명령은 차례대로 실행한다. 기존 결과 폴더는 덮어쓰지 않으므로 재실행 때 새 경로를 지정한다. 브라우저가 생성한 32자리 실행 ID의 결과는 화면에서도 선택할 수 있다. CLI 사용자 지정 이름의 보고서는 해당 폴더에서 직접 연다.

## 내가 데이터와 평가를 구성한 이유

같은 회차의 연속 사진을 무작위로 나누면 배경과 거의 같은 프레임을 암기한 결과가 테스트에 섞일 수 있다. 나는 10회차 × 4종류 × 30장, 총 1,200장을 **회차 단위**로 학습 840장·검증 240장·테스트 120장에 나눈다. 클래스별 개수는 210·60·30장이다. 파일 내용이 같은 이미지가 분할을 가로지르면 학습을 거부한다. 다만 테스트는 한 회차뿐이므로 다른 사용자·장소에 대한 일반화는 아직 확인하지 못했다.

문제 1의 네 모델은 128×128 입력과 같은 정규화를 사용한다. CNN은 공간 구조를 보존하면서 MLP보다 적은 파라미터를 쓴다. 동결 ResNet은 백본 파라미터와 BatchNorm 통계를 함께 고정한다. 파인튜닝은 사전학습 특징을 크게 훼손하지 않도록 백본 LR 0.0001, fc LR 0.001을 사용한다. 나머지는 Adam LR 0.001, weight decay 0.0001이다. 작은 입력·증강·학습률의 개선 효과는 대조 실험 없이 단정하지 않는다.

문제 2의 상대 손 이미지는 실제 웹캠 사진이지만 **관절 상태와 정답 액션은 가상 교사로 생성한 값**이다. 실제 로봇의 동기화 센서 기록은 아니다. 하나의 사진을 각 에피소드의 고정 관측으로 두고, 같은 시간축의 상태 q[t]와 목표 명령 a[t]를 짝짓는다. 이미지의 라벨은 교사 생성과 보조 분류 학습에 사용하지만, 추론에는 정답 라벨이나 미래 액션을 넣지 않는다.

관절 범위는 0~1.4 rad, 상태 갱신은 10Hz에서 `q[t+1] = q[t] + 0.35 × (a[t] − q[t])`다. 가위에는 바위, 바위에는 보, 보에는 가위 자세가 목표이며 없음은 현재 자세를 유지한다. 72시점 궤적에는 부드러운 전이·36~64시점의 속도 차이·작은 중간 경로 변화를 넣었다. 초기 구현의 임의 시작 자세와 화면의 전관절 0 시작이 달랐던 문제를 발견해, 수정된 교사에는 절반의 에피소드를 전관절 0에서 시작하도록 포함한다.

RNN·LSTM·Transformer는 동일한 과거 상태 창과 다음 상태 정답을 쓴다. 움직임만으로 장기 의존성이 충분히 필요하지 않을 수 있어, 초반 4시점에만 기억할 값을 주고 뒤를 중립값으로 채운 별도 합성 과제를 함께 학습한다. 32시점 입력과 마지막 8시점만 남긴 입력의 MSE를 따로 보고한다. 뒤의 비교에는 입력 길이·분포 변화도 있으므로 Transformer의 보편적 우월성으로 해석하지 않는다.

ACT/CVAE는 학습 중에만 posterior가 정답 청크를 보고 μ·logvar를 계산하며 `z = μ + exp(0.5 × logvar) × ε`로 샘플링한다. 평가에서는 정답 없이 z=0과 N(0,I) 샘플을 각각 사용한다. 손실은 재구성 MSE + 0.001 KL + 0.2 보조 이미지 분류 CE다. 학습률은 0.0003이다. 원 ACT의 재현 전체가 아니라, CNN 대신 직접 만든 ViT를 사용하는 교육용 구성이다. 시간 중첩 앙상블은 사용하지 않는다.

화면은 8개의 명령을 예측하고 앞 4개를 실행한 뒤 재관찰한다. 분류 신뢰도 70% 미만 또는 없음이면 현재 자세를 유지한다. 신뢰도는 보장된 정확도가 아니며, 목표 규칙으로 예측 액션을 교체하지 않는다. 숨긴 탭·중지·오류는 진행 중 명령을 취소한다.

## 검증

```bash
python problem1_rps/test_rps.py --smoke-training
node problem1_rps/test_ui.cjs
python problem2_hand/test_hand.py --smoke-training
python problem2_hand/test_rollout.py
node problem2_hand/test_hand_ui.cjs
```

합성 1 Epoch 검사는 학습·저장·평가 경로의 실행 검사이며 제출 성능으로 사용하지 않는다. CPU 검사는 분할, 시간 정렬, 패치 순서, posterior gradient와 평가 누출 방지, 잘못된 HTTP 입력, 관절 범위, 중지 동작을 확인한다. 실제 수치는 별도로 촬영 데이터 학습에서 얻는다. 실물 로봇 구동·다른 환경의 일반화·사람 상대 승률 실험은 수행하지 않았다.

## 참고한 공식 자료

- [torchvision 0.26 ResNet18](https://docs.pytorch.org/vision/0.26/models/generated/torchvision.models.resnet18.html)
- [PyTorch 전이학습 튜토리얼](https://docs.pytorch.org/tutorials/beginner/transfer_learning_tutorial.html)
- [PyTorch 2.11 Transformer](https://docs.pytorch.org/docs/2.11/generated/torch.nn.Transformer.html)
- [ViT 원 논문](https://arxiv.org/abs/2010.11929)
- [ACT 원 논문](https://arxiv.org/abs/2304.13705)
