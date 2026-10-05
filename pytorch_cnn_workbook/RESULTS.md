# CIFAR-10 CNN 실습 결과

`record_id`: `DAPIER-2026-08-18-cifar10-cnn-workbook`

나는 PDF의 다섯 과제를 통합 스크립트가 아닌 `project1_augmentation.py`부터 `project5_error_analysis.py`까지 과제별 파일로 구현하고 직접 실행했다. 공정 비교에 필요한 모델·데이터 코드만 `cifar10_common.py`로 공유했다.

## 실행 환경

- Python 3.10.20
- PyTorch 2.13.0+cu130, torchvision 0.28.0+cu130, CUDA 13.0
- NVIDIA GeForce RTX 5050 Laptop GPU
- OpenCV 5.0.0, Matplotlib 3.10.9, scikit-learn 1.7.2
- seaborn 0.13.2, pandas 2.3.3
- 공통 seed 42, batch size 64, Adam lr 0.001

## P1 — 데이터 증강과 역정규화

train에만 `RandomCrop(32, padding=4)`, `RandomHorizontalFlip`, `ColorJitter(0.2, 0.2, 0.2)`를 적용하고 test에는 적용하지 않았다. `Normalize(0.5, 0.5)` 뒤 train batch 범위는 [-1, 1]이었고, `x * 0.5 + 0.5`로 역정규화하면 [0, 1]로 정확히 돌아왔다. 역정규화 뒤 표시 범위를 벗어난 값은 train/test 모두 0개였다.

## P2 — Forward hook 피처맵

baseline의 첫 test 이미지(cat)를 cat으로 맞혔다. post-ReLU activation은 다음과 같았다.

| 레이어 | shape | 0 활성 비율 |
|---|---:|---:|
| conv1 | 1×32×32×32 | 45.86% |
| conv2 | 1×32×32×32 | 74.47% |
| conv3 | 1×64×16×16 | 66.71% |
| conv4 | 1×64×16×16 | 80.84% |

깊은 conv4에서 활성 희소성이 가장 컸다. 모든 handle을 `finally`에서 제거했고 실행 후 남은 hook은 0개였다.

## P3 — BN/Dropout 공정 절제 실험

세 조건 모두 같은 seed·데이터·epoch·optimizer를 사용해 30 epoch를 실행했다.

| 조건 | 최저 test loss epoch | 해당 test acc | 최종 test acc | 과적합 감지 |
|---|---:|---:|---:|---:|
| BN + Dropout | 30 | 85.06% | 85.06% | 감지 안 됨 |
| No BN | 29 | 79.43% | 78.55% | 27 epoch |
| No Dropout | 25 | 87.04% | 86.79% | 16 epoch |

BatchNorm을 빼면 baseline보다 해당 시점 정확도가 5.63%p 낮아져 수렴과 일반화에 불리했다. Dropout을 빼면 최고 정확도는 올랐지만 16 epoch부터 test loss 상승이 반복됐고, 최종 train 90.72% 대 test 86.79%로 3.93%p 간격이 생겼다. 따라서 내 다음 실험은 BN을 유지하고 conv/fc dropout 0.3/0.5를 0.1~0.3 범위로 낮추는 것이다.

## P4 — TensorBoard 검증

`baseline_bn_dropout`, `no_bn`, `no_dropout` 세 run을 실제 이벤트 파일에서 다시 읽었다. 각 run에 다음 scalar 7개와 histogram 2개가 존재했다.

- scalar: `loss/train`, `loss/test`, `accuracy/train`, `accuracy/test`, `lr`, `gradients/conv1_norm`, `gradients/conv4_norm`
- histogram: `weights/conv1`, `weights/conv4`

`project4_tensorboard.py --serve`로 검증 후 서버까지 열 수 있다.

## P5 — 혼동 행렬과 고신뢰 오답

baseline의 CIFAR-10 test 10,000장 정확도는 85.06%였다. recall이 가장 낮은 클래스는 cat(61.8%)이었고, 가장 큰 혼동은 cat→dog 172장(17.2%)이었다. 다음은 dog→cat 64장(6.4%), cat→frog 57장(5.7%), bird→frog 54장(5.4%), bird→deer 51장(5.1%) 순이었다.

서로 다른 정답→예측 쌍에서 confidence가 가장 높은 오답 5개도 저장했다. 최고 오답은 cat→frog였고 confidence 99.95%였다. 이 결과는 단순 정확도 외에 동물 클래스의 질감·윤곽 혼동과 과신 문제를 함께 개선해야 한다는 뜻이다.

## 정확도를 더 높이기 위한 다음 실험

1. BatchNorm은 유지하고 Dropout 비율을 낮춰 검증한다.
2. AdamW의 weight decay와 cosine learning-rate schedule을 함께 탐색한다.
3. RandomErasing 또는 MixUp/CutMix를 추가하되 test transform은 고정한다.
4. 같은 평가 계약으로 작은 ResNet을 비교한다.

이 개선안은 PDF의 P3 공정 비교를 오염시키지 않도록 별도 실험 파일에서 다룬다.

## 검증

- `ruff format --check`: 통과
- `ruff check`: 통과
- `pytest`: 4 passed
- P1~P5: 실제 CUDA 실행 완료
- 공개 결과: PNG 12개, CSV 6개, JSON 5개

모델 체크포인트와 TensorBoard 이벤트는 재생성 가능한 대용량 결과이므로 Git에서 제외했다.
