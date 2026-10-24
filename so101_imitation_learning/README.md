# SO-101 Imitation Learning — 학습용 포크

PyTorch 기초에서 시퀀스 모델, ACT, SO-101 제어와 MuJoCo 실습으로 이어지는 학습 저장소입니다.

**원본:** [JD-edu/so101_imitation_learning](https://github.com/JD-edu/so101_imitation_learning) · **관리:** [전형주 / Alpenj](https://github.com/Alpenj) · **포트폴리오:** [Physical AI Portfolio](https://julianjeonresume.netlify.app/)

> 이 저장소는 교육용 원본의 **fork**입니다. 원본 코드·모델·예제 출력물을 개인의 독자 구현 또는 직접 측정한 성능으로 소개하지 않습니다. 개인 기여는 이 포크의 변경 이력과 별도의 재현 기록으로 구분합니다.

## 처음 보는 분께

- **학습 범위:** 아래 디렉터리 지도에서 PyTorch → Transformer → ACT → 로봇 제어 → 시뮬레이션 순서를 확인할 수 있습니다.
- **ACT 코드 읽기:** [`103_ACT_step_by_step/`](103_ACT_step_by_step/)에서 단계별 예제를 비교합니다.
- **개인 학습 설명:** [Physical AI Lab](https://github.com/Alpenj/physical-ai-lab)에서 개념·실험 질문·확인 범위를 읽을 수 있습니다.
- **기여 확인:** [커밋 이력](https://github.com/Alpenj/so101_imitation_learning/commits/main/)을 원본과 구분해 확인합니다. 문서 정리 커밋을 모델 구현 성과로 계산하지 않습니다.

## 전체 디렉터리 지도

| 순서 | 경로 | 다루는 범위 | 읽을 때 확인할 것 |
|---|---|---|---|
| 1 | [`101_pytorch_basic/`](101_pytorch_basic/) | PyTorch 기초 예제 | 입력·정답·모델 출력의 shape, 학습과 평가 구분 |
| 2 | [`102_RNN_transformer/`](102_RNN_transformer/) | RNN·Transformer 예제 | 시간축, 시퀀스 입력, 예측 대상 |
| 3 | [`103_ACT_step_by_step/`](103_ACT_step_by_step/) | ACT 단계별 구성 | action chunk, inference, receding horizon, temporal ensemble, CVAE, Transformer |
| 4 | [`103_so101_control_caibration/`](103_so101_control_caibration/) | SO-101 제어·캘리브레이션 | 장치 연결, 관절 순서, 단위, 제한과 정지 절차 |
| 5 | [`104_so101_imitation_learning/`](104_so101_imitation_learning/) | SO-101 모방학습 예제 | 관측·행동·데이터·학습·실행의 연결 |
| 6 | [`105_MUJOCO_basic/`](105_MUJOCO_basic/) | MuJoCo 기초 | 모델 자산, 좌표계, 시뮬레이션 조건 |
| 7 | [`106_so101_MUJOCO_imitation_learning/`](106_so101_MUJOCO_imitation_learning/) | SO-101 MuJoCo 모방학습 | 실제 장비와 시뮬레이션의 경계 |

`103` 번호가 중복되고 `caibration`이라는 기존 철자가 있지만, 원본과의 비교 및 코드·수업 자료의 경로 호환성을 위해 이름을 유지합니다. `.vscode/`는 편집기 설정이며 학습 단계가 아닙니다.

## ACT 예제를 읽는 순서

| 파일 | 비교할 주제 |
|---|---|
| [`101_act_pytorch.py`](103_ACT_step_by_step/101_act_pytorch.py) | 시작 예제의 데이터·모델·학습 흐름 |
| [`102_act_inference.py`](103_ACT_step_by_step/102_act_inference.py) | 학습과 추론 경계 |
| [`103_act_receding_horizon.py`](103_ACT_step_by_step/103_act_receding_horizon.py) | 예측한 행동 중 실행할 구간 |
| [`104_act_temporal_ensemble.py`](103_ACT_step_by_step/104_act_temporal_ensemble.py) | 겹치는 시점의 행동 예측 결합 |
| [`105_act_CVAE.py`](103_ACT_step_by_step/105_act_CVAE.py) | 잠재변수를 사용하는 구성 |
| [`106_act_transformer.py`](103_ACT_step_by_step/106_act_transformer.py), [`106_act_transformer_infer.py`](103_ACT_step_by_step/106_act_transformer_infer.py) | Transformer 예제 비교 |

각 파일 이름은 학습 동선입니다. 논문 전체 구현과의 동등성 또는 실물 로봇 성능을 보증하지 않습니다. `*_output/`, `experimental/`과 기존 모델·GIF는 원본 학습 자료와 함께 보존합니다.

## 실행 전 확인

이 저장소 전체를 하나의 실행 명령으로 시작하지 않습니다. 먼저 학습할 폴더를 선택하고 해당 스크립트의 의존성, 데이터 경로, 모델 파일, 장치 접근 여부를 확인합니다. 현재 루트에는 모든 실습을 포괄하는 고정 환경·통합 테스트 명령이 정리되어 있지 않습니다.

코드와 변경 이력만 확인하는 시작 방법:

```bash
git clone https://github.com/Alpenj/so101_imitation_learning.git
cd so101_imitation_learning
git log -5 --oneline
```

실물 제어·캘리브레이션 예제는 읽기만 하여도 충분합니다. 실행할 때에는 장치별 안내, 전원·관절 범위·정지 방법을 먼저 확인하고, 학습 예제를 실행했다는 이유만으로 실물 동작 준비가 완료됐다고 판단하지 않습니다.

## 재현 기록 기준

개인 실험 결과를 추가할 때에는 다음 항목을 함께 남깁니다.

| 항목 | 기록 내용 |
|---|---|
| 기준 코드 | 이 저장소 commit과 비교 대상 원본 commit |
| 실행 환경 | OS, Python, 주요 라이브러리, GPU 또는 장치 |
| 입력과 명령 | 데이터 출처·분할·seed, 실행한 파일과 인자 |
| 실제 결과 | 측정값·로그·출력물과 실행 날짜 |
| 비교와 한계 | 수정 전후 차이, 실패 사례, 아직 확인하지 않은 조건 |

예제 파일이 존재하는 것, 학습이 끝나는 것, 정책이 과제를 성공하는 것, 실제 로봇이 성공하는 것은 서로 다른 확인 단계입니다.

## 출처와 이용 범위

원본 및 개별 코드·모델·자산의 권리 표기를 유지합니다. 2026-09-05 루트 점검에서는 별도의 LICENSE 파일을 확인하지 못했습니다. 이 문서 정리는 새 라이선스 부여나 원본 코드의 권리 변경을 의미하지 않습니다.

문서 구조 점검: 2026-09-05. 이번 정리는 README와 탐색 경로에 한정되며 학습·시뮬레이션·실물 테스트를 새로 실행한 결과가 아닙니다.
