# MiniVGG 하이퍼파라미터 안정화 실험

JD 교육 코드의 MiniVGG 구조를 바꾸지 않고 학습 조건만 조정한 개인 실습이다.
원본 파일은 수정하지 않았고, 원본과 이 사본의 `state_dict` 이름·shape 및
총 파라미터 `8,457,635`개가 같음을 직접 검사했다.

## 실험 질문

이미지를 더 추가할 수 없을 때 높은 학습률과 매번 달라지는 분할에서 생기는
결과 변동을 줄일 수 있는가?

## 통제한 것과 바꾼 것

| 항목 | JD 원본 | 개인화 실험 |
|---|---:|---:|
| CNN 구조와 dropout | 원본 MiniVGG, 0.3/0.4/0.5 | 동일 |
| 입력 크기 | 64×64 | 동일 |
| learning rate | 0.001 | 0.0003 |
| optimizer | Adam | AdamW |
| weight decay | 없음 | 0.0001 |
| loss | CrossEntropy | class weight + label smoothing 0.05 |
| augmentation | 회전 15°, jitter 0.2 | crop 0.85~1.0, 회전 8°, jitter 완화 |
| 데이터 평가 | 전체를 train으로 사용 | 고정 stratified train 232 / validation 58 |
| 안정화 | 없음 | gradient clip 1.0, ReduceLROnPlateau |
| 종료/저장 | 30 epoch 최종 모델 | patience 8, best validation loss |
| 재현성 | seed 미고정 | split seed 2026, train seed 42/43/44 |

## 실제 실행 결과

| seed | best epoch | validation accuracy | macro recall |
|---:|---:|---:|---:|
| 42 | 10 | 86.21% | 88.29% |
| 43 | 9 | 87.93% | 88.31% |
| 44 | 10 | 87.93% | 89.48% |

- 3-seed validation accuracy: **87.36% ± 0.81%**
- 선택 체크포인트: seed 43, epoch 9, validation loss 0.4168
- 선택 모델 class recall: cans 91.67%, cups 92.31%, pets 80.95%

`results/training_curves.png`에서 train/validation loss와 accuracy를 함께 확인한다.
원본의 91.38%는 validation이 아닌 train accuracy이므로 위 수치와 직접 비교하지
않는다.

## 실행

```bash
conda activate lerobot-vision
cd so101/pytorch_basic/minivgg_hparam

# images/cans, images/cups, images/pets 구조를 가진 로컬 폴더를 지정한다.
python train_hparam.py \
  --data "$DATA_DIR" \
  --seeds 42 43 44 \
  --no-show

python live_cam.py
```

VS Code가 시스템 Python을 선택해도 로컬에 `lerobot-vision` 환경이 있으면 해당
환경으로 한 번 다시 실행한다. 라이브캠은 중앙 ROI의 확률을 EMA로 평활화하고,
confidence와 top-2 margin이 부족하면 `UNCERTAIN`으로 표시한다.

## 해석과 한계

고정 validation과 여러 seed 덕분에 결과 변동을 숫자로 보고 best epoch를 선택할
수 있게 됐다. 하지만 validation도 같은 이미지 모음에서 분리됐기 때문에 상품
사진과 노트북 카메라 사이의 domain shift를 측정하지 못한다. 따라서 87.93%를
라이브캠 정확도라고 부르지 않는다. 이번 과제의 결론은 CNN 구조 변경이나
ResNet 교체가 아니라, 동일 MiniVGG에서 하이퍼파라미터와 평가 절차를 통제한
결과다.

이미지 원본과 33MB 체크포인트는 저장소에 올리지 않는다.
