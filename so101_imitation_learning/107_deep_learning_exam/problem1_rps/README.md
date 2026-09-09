# 문제 1 — 웹캠 가위바위보 실습

record_id: `DAPIER-2026-09-09-rps-exam`

나는 노트북 카메라로 가위·바위·보·없음을 촬영하고 MLP·CNN·ResNet 전이학습을 비교한다. 손 모양 분류기를 학습한 뒤 PC가 이기는 패를 표시한다. PC의 대응 자체는 규칙이다.

2026-09-09에 실제 손 사진 1,200장 수집과 네 모델의 10 Epoch 학습·평가를 완료했다. 실행 ID는 `3cb779884c1c4bfcbabb0fd00ab94afd`이며, `runs/<실행 ID>/RESULTS.md`와 `report.html`에 실제 지표가 기록되어 있다. 같은 날 웹캠 게임을 직접 실행해 동작을 확인했다. 이 동작 확인은 정량적인 라이브 인식 정확도나 승률 측정을 뜻하지 않는다. 합성 검사는 제출 성능으로 사용하지 않는다.

## 실제 측정 결과

| 모델 | 검증 Accuracy | Test Accuracy | Test Loss | forward ms/장 | FPS |
|---|---:|---:|---:|---:|---:|
| MLP | 61.25% | 60.00% | 0.8227 | 0.206 | 4851.7 |
| CNN | 88.75% | 74.17% | 0.5257 | 0.227 | 4402.1 |
| ResNet18 동결 | 80.83% | 60.83% | 0.9781 | 1.474 | 678.3 |
| ResNet18 파인튜닝 | 96.67% | 79.17% | 1.0379 | 1.504 | 664.8 |

검증 Accuracy 기준으로 선택한 게임 모델은 ResNet18 파인튜닝이다. 테스트 정확도는 79.17%이지만 다른 사용자·장소에 대한 일반화나 웹캠 게임의 정량 정확도는 아직 별도 평가하지 않았다. 파인튜닝의 Test Loss가 CNN보다 큰 점도 실제 결과 그대로 남긴다. 속도는 batch 1·GPU 동기화·준비 추론 10회 후 50회 측정한 순수 forward 값이며 카메라 FPS가 아니다. 전체 [결과 및 수행 과정](../RESULTS.md)에 해석을 모은다.

## 실행

기존 Python 가상환경을 재사용했다. 확인한 환경은 Python 3.12.3, PyTorch 2.11.0+cu130, torchvision 0.26.0+cu130, Pillow 12.3.0, matplotlib 3.10.9와 RTX 5050 Laptop 8GB다. ROS·실물 로봇은 사용하지 않는다.

```bash
# requirements.txt의 패키지가 설치된 Python 환경에서
python ../problem2_hand/hand_exam.py serve --port 8766
```

이 문서가 있는 `problem1_rps/`에서 위 명령으로 두 문제의 통합 서버를 실행하고 브라우저에서 <http://127.0.0.1:8766>을 연다. 문제 2 가상 손은 화면 상단 링크 또는 <http://127.0.0.1:8766/hand>로 이동한다. **카메라 시작** 버튼을 눌렀을 때만 카메라를 켠다. 서버 종료는 Ctrl+C이며 SIGTERM도 학습 자식 프로세스를 정리한다. 새 환경에서는 `requirements.txt`를 참고하고 GPU용 PyTorch는 공식 배포판에서 호환 wheel을 선택한다. 이번 작업에서는 전역 Python과 기존 패키지를 변경하지 않았다.

## 내가 촬영하는 순서

1. 카메라를 켜고 장치 목록에서 노트북 카메라를 선택한다. 초록 사각형에 손 전체를 넣는다.
2. 새 세션을 만든다. 가위·바위·보·없음을 각각 30장씩 촬영한다.
3. 라벨 버튼을 누르면 3초 준비 뒤 0.25초 간격으로 촬영한다. 같은 패를 유지하면서 위치와 각도를 조금씩 바꾼다.
4. 없음은 인식 영역에서 손을 완전히 빼고 촬영한다. 애매한 손 모양을 없음으로 섞지 않는다.
5. 네 종류를 다 촬영한 후 다음 세션에서 거리·조명·배경을 바꾼다. 특정 클래스에만 특정 배경을 사용하지 않는다.
6. 총 **10세션 × 4종류 × 30장 = 1,200장**을 수집한다. 중지·새로고침 후에도 저장한 장수부터 이어간다.

촬영과 게임에서 영상의 짧은 변 70%인 중앙 정사각형을 거울 모드 224×224 JPEG로 처리한다. 전체 프레임과 오디오는 저장하지 않는다. 원시 이미지는 로컬 `data/`에 두고 Git에 올리지 않는다.

세션 단위 7:2:1 분할은 학습/검증/테스트 **840/240/120장**, 클래스별 **210/60/30장**이다. 같은 세션의 연속 프레임이 다른 분할로 섞이지 않게 했다. 서로 다른 분할에 같은 파일 내용이 있으면 학습을 거부한다. 비슷한 이미지까지 완전히 걸러내는 기능은 아니므로 촬영 조건도 바꿔야 한다.

## 요구사항과 구현

| 문항 | 구현·선택 이유 | 결과 파일 |
|---|---|---|
| 1-1 | HandDataset, 세션 분할, train 전용 RandomCrop/HorizontalFlip/ColorJitter, Normalize | manifest.json, augmentation.png |
| 1-2 | MLP, Conv2d 3개 CNN, CrossEntropyLoss·Adam, 10 Epoch 이상 | 모델 코드, *_history.json, train.log |
| 1-3 | 사전학습 ResNet18의 fc를 4출력으로 교체. 동결 시 백본·BN 통계 고정, 파인튜닝은 전체 학습 | 두 체크포인트, 학습 가능한 파라미터 수 |
| 1-4 | 검증 기준 가중치 선택 후 Test Accuracy·혼동행렬·곡선·batch 1 속도 평가 | metrics.json, learning_curves.png, *_confusion.png, sample_predictions.png, report.html |

**네 모델 학습 시작** 버튼으로 모델을 순차 학습한다. 기본은 모델마다 10 Epoch다. CLI로도 실행할 수 있다.

```bash
python rps_exam.py train --data data --run runs/my-first-experiment --epochs 10
```

CLI 사용자 지정 실행 폴더는 보고서를 직접 열어 확인한다. 브라우저는 자체 생성한 실행 ID 폴더를 표시한다. 실행 기록이 있는 폴더를 덮어쓰지 않는다.

네 모델 모두 입력 128×128과 ImageNet 정규화를 사용한다. ResNet의 표준 224px보다 작은 입력을 사용하는 연산량 절충을 보고서에 명시한다. MLP/CNN/동결 ResNet의 Adam LR은 0.001, 파인튜닝은 백본 0.0001·fc 0.001, weight decay는 0.0001이다. 증강·설정의 개선 효과는 실제 대조 실험 전에는 주장하지 않는다.

검증 Accuracy가 가장 높은 Epoch, 동률이면 검증 Loss가 낮은 가중치를 선택한다. 게임의 기본 모델도 검증 Accuracy 기준이다. 테스트 결과로 모델을 선택하지 않는다.

## 게임과 속도 측정

- 가위 → PC 바위, 바위 → PC 보, 보 → PC 가위.
- 없음 → PC 대기. 불확실하거나 변화 중 → 판단 중.
- 신뢰도 70% 이상·1위와 2위 확률 차이 15% 이상인 같은 라벨을 3프레임 이상·0.7초 유지하면 확정한다.
- 다음 라운드로 결과를 지운다. 카메라·통신 오류나 중지 시 이전 예측을 지운다.

안정화 기준은 시작 설정이다. 높은 softmax가 실제 손 없음 검출을 보장하지 않는다. PC 승패를 인식 정확도로 계산하지 않고 테스트 정답·혼동행렬과 실제 카메라 관찰로 확인한다.

결과표 Latency는 batch 1의 순수 forward 평균이며 GPU 동기화를 수행한다. FPS=1000/Latency(ms)다. `pipeline_ms`는 전처리·장치 전송·forward·확률 반환을 포함하며, 둘 다 카메라·HTTP 시간은 제외한다. 게임 화면은 모델 시간·왕복 응답·갱신 FPS를 구분한다.

## 검증과 남은 일

```bash
python -m py_compile rps_exam.py test_rps.py
python test_rps.py --smoke-training
node test_ui.cjs
```

직접 실행해 확인한 항목은 회차 분할·중복 검출·Dataset 경로 검사·4모델 출력과 gradient·동결 BN 유지·촬영 API·잘못된 입력 거부·합성 1 Epoch 학습/저장/평가/보고서 생성이다. 저장 실패 후 남은 임시 파일의 재시도도 검사했다. 합성 가중치는 실제 게임에 사용할 수 없다.

UI 검사에서는 가위·바위·보 대응, 안정된 없음, 낮은 신뢰도에서 이전 판정 초기화, 3프레임·700ms 조건, 촬영과 추론의 동일한 거울 ROI를 확인했다. 실제 브라우저 1440px·390px 화면도 카메라를 자동 호출하지 않는 상태로 검사했고 모바일 가로 넘침을 수정했다.

실제 10세션 촬영과 네 모델 학습·평가, 사용자의 웹캠 게임 동작 확인은 완료했다. 남은 검증 범위는 다른 사용자·배경에서의 일반화와 별도 정답을 기록한 라이브 정확도다. 실제 제출에는 학생명 파일명을 적용하고 HTML 보고서를 브라우저 인쇄로 PDF 저장한다. 전체 작업 완료 후 실제 수행 과정을 Notion·GitHub에 정리할 예정이다. 현재 업로드 완료 상태가 아니다.

## 참고

- [torchvision 0.26 ResNet18](https://docs.pytorch.org/vision/0.26/models/generated/torchvision.models.resnet18.html)
- [PyTorch 전이학습 튜토리얼](https://docs.pytorch.org/tutorials/beginner/transfer_learning_tutorial.html)
- 기존 resnet_dataset_studio_step1의 촬영·로컬 저장 패턴과 101_pytorch_basic의 CNN 학습 흐름을 참고했다.
