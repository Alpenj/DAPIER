# CIFAR-10 CNN 인터랙티브 실습 기록

`record_id`: `DAPIER-2026-08-18-cifar10-cnn-workbook`

오늘 수업에서 받은 `Pytorch CNN 실습.pdf`의 다섯 프로젝트를 과제별 실행 파일로 각각 옮긴다. `cifar10_common.py`는 공정한 비교를 위해 모델·데이터 계약만 공유하며, 통합 실행 파일은 두지 않는다. 이 폴더에는 직접 실행한 코드와 공개 가능한 CSV·PNG·JSON 결과만 남긴다. CIFAR-10 원본 데이터, 모델 체크포인트, TensorBoard 이벤트 파일은 용량과 재현성 경계를 명확히 하기 위해 Git에 넣지 않는다.

## 실습 범위

1. P1: train 증강과 test 비증강 입력을 역정규화해 비교한다.
2. P2: 제거 가능한 forward hook으로 conv1~conv4의 post-ReLU 피처맵을 저장한다.
3. P3: 같은 seed·epoch·데이터 조건에서 BN+Dropout, No BN, No Dropout을 비교한다.
4. P4: 세 실험의 loss, accuracy, lr, gradient norm, weight histogram을 TensorBoard에 기록한다.
5. P5: 행 정규화 혼동 행렬과 서로 다른 클래스쌍의 고신뢰 오답 5개를 분석한다.

## 환경

```bash
conda activate lerobot-vision
python -m pip install -r requirements.txt
```

실행 시 사용한 환경은 Python 3.10, RTX 5050 Laptop GPU, CUDA용 PyTorch다. 정확한 버전과 실제 결과는 실행 후 `RESULTS.md`와 `artifacts/reports/`에 기록한다.

## 과제별 실행

```bash
conda activate lerobot-vision
cd ~/DAPIER/.local-workspaces/cifar10-workbook/pytorch_cnn_workbook
python project1_augmentation.py --data-dir ~/DAPIER/so101_imitation_learning/101_pytorch_basic/data
python project3_ablation.py --data-dir ~/DAPIER/so101_imitation_learning/101_pytorch_basic/data --epochs 30
python project2_feature_maps.py --data-dir ~/DAPIER/so101_imitation_learning/101_pytorch_basic/data
python project4_tensorboard.py
python project5_error_analysis.py --data-dir ~/DAPIER/so101_imitation_learning/101_pytorch_basic/data
```

P2와 P5는 학습된 baseline 체크포인트를 읽으므로 P3 뒤에 실행한다. P4는 P3가 기록한 세 실험의 TensorBoard 태그가 실제로 존재하는지 독립적으로 검증한다.

## VS Code에서 실행

이 폴더 자체를 VS Code workspace로 열면 `.vscode/settings.json`이 `lerobot-vision` conda 환경을 선택하고 이 PyTorch 프로젝트에서만 ROS의 Python 3.12 경로 혼입을 차단한다. 우측 상단의 Python 실행 버튼으로 과제 파일을 바로 실행할 수 있다. 기존 수업용 CIFAR-10 데이터가 있으면 인자 없이 자동으로 재사용한다.

왼쪽 `실행 및 디버그`에서 다음 구성을 각각 선택할 수도 있다.

- 과제 1 - 증강 시각화
- 과제 2 - 피처맵
- 과제 3 - 절제 실험 (30 epoch)
- 과제 4 - TensorBoard 검증
- 과제 5 - 오류 분석

과제 4 실행 뒤 VS Code 안에서 보려면 `Ctrl+Shift+P` → `Simple Browser: Show`를 고르고 `http://127.0.0.1:6006`을 입력한다. P1·P2·P3·P5의 PNG 결과는 `artifacts/figures/`에서 클릭하면 VS Code 이미지 탭으로 열린다.

TensorBoard 이벤트는 `artifacts/tensorboard/`에 생성된다.

```bash
tensorboard --logdir=artifacts/tensorboard --port=6006
```

포트가 이미 사용 중이면 다른 포트를 고른다.

```bash
tensorboard --logdir=artifacts/tensorboard --port=6007
```

## 검증

```bash
python -m pytest -q tests
ruff check cifar10_common.py project*.py tests
```

## 실행 완료 범위

- P1~P5와 30 epoch×3 절제 실험을 RTX 5050 Laptop GPU에서 직접 실행했다. 측정값과 해석은 `RESULTS.md`에 기록했다.
- PDF의 추가 도전 과제인 Grad-CAM, saliency map, mixed precision은 핵심 P1~P5 완료 후 별도 실험으로 다룬다.
