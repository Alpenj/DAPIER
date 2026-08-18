# 2026-08-18 MiniVGG 하이퍼파라미터 안정화와 라이브캠

`record_id`: `DAPIER-2026-08-18-minivgg-hparam-stability`

## 오늘 확인한 문제

JD 원본 MiniVGG를 직접 실행했을 때 epoch별 loss와 train accuracy가 크게
움직였다. 원본은 290장을 모두 train에 사용하고 seed와 validation을 두지 않아
91.38%가 일반화 정확도인지 확인할 수 없었다.

처음에는 모델 크기를 줄이는 구조 변경도 시도했지만, 과제 목표가
하이퍼파라미터 개선이라는 점과 맞지 않아 결과에서 제외했다. 최종 공개 코드는
원본과 동일한 8,457,635개 파라미터, 동일한 계층 순서와 dropout을 유지한다.
원본 두 Python 파일은 Git HEAD와 차이가 없음을 확인했다.

## 바꾼 학습 조건

- learning rate를 0.001에서 0.0003으로 낮췄다.
- Adam 대신 AdamW와 weight decay 0.0001을 적용했다.
- augmentation의 회전과 색 변화 폭을 줄이고 작은 random crop을 넣었다.
- 클래스 비율을 유지한 train 232 / validation 58 고정 분할을 만들었다.
- class weight, label smoothing 0.05, gradient clipping 1.0을 적용했다.
- validation loss가 멈추면 learning rate를 절반으로 줄이고 patience 8에서
  조기 종료한다.
- seed 42, 43, 44를 같은 분할에서 반복했다.

## 실제 결과

best validation accuracy는 seed별 86.21%, 87.93%, 87.93%였고 평균과
표준편차는 87.36% ± 0.81%였다. validation loss가 가장 낮은 seed 43의
epoch 9를 체크포인트로 선택했다. 해당 class recall은 cans 91.67%, cups
92.31%, pets 80.95%다.

학습 곡선은 train과 validation을 함께 저장했다. 라이브캠에는 프레임별
softmax를 바로 표시하지 않고 EMA, confidence threshold, top-2 margin을
사용했다.

## 남은 한계

라이브캠 인식은 validation 수치보다 분명히 낮았다. 학습 이미지가 상품 사진과
흰 배경 중심이라 노트북 카메라의 배경·거리·조명 변화가 validation에 들어 있지
않기 때문이다. 데이터 추가 금지 조건에서 이 한계를 숫자로 없앴다고 주장하지
않는다. 이번 실습에서 검증한 것은 동일 MiniVGG 구조에서 학습 조건을 고정하면
seed 변동을 측정하고 best epoch를 재현 가능하게 선택할 수 있다는 점이다.
