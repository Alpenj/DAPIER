# 신발 정리 양팔로봇 학습 전략 — ACT first

## 결정

이번 프로젝트의 필수 학습 경로는 **ACT**다. VLA는 ACT 다음의 자동 승급 단계가
아니며, 일정·GPU·데이터가 실제로 남을 때만 실행하는 선택 심화다.

> JD-edu 코드로 ACT 학습과 rollout을 재현하고, Receding Horizon과 Temporal
> Ensembling의 성공률·지연·jitter·복구 결과를 먼저 확보한다. 그 결과가 고정된
> 뒤에만 SmolVLA pretrained checkpoint의 작은 fine-tuning 비교를 검토한다.

## 왜 이 순서인가

SO-101에서 VLA의 비용은 GPU 학습시간만이 아니다. pretrained model 다운로드와
메모리, image-language-action dataset 변환, 12차원 양팔 action 순서와 정규화,
checkpoint 관리, 느린 inference와 servo/control 주기 연결, 실패 원인 분리까지 모두
새로운 변수가 된다. 언어 지시와 여러 task가 부족하면 VLA의 장점도 제대로 비교하기
어렵다.

현재 저장소에는 이미 다음 ACT 자산이 있다.

- finalized episode→RGB-D/action window→train-only normalization 계약
- DAPIER-native CVAE+Transformer ACT train/infer 경로
- held-out action-chunk offline evaluator
- checkpoint SHA와 reset generation을 포함한 supervisor dry-run
- policy proposal과 실제 executed action을 분리하는 fail-closed 계약

따라서 새 대형 pipeline을 여는 것보다 이 경로를 실제 신발 task에서 닫는 것이 먼저다.

## 현재 MuJoCo 병렬학습 readiness

- clearance upper와 SO-101 단일 베이스를 쓰는 tower 모델이 `ShoeTaskEnv` 기본값이다.
- observation은 21차원 state, action은 12차원 양팔 actuator target이다.
- 4개 spawn process의 독립 MuJoCo rollout과 finite state를 확인했다.
- 4,000 transition에서 1 worker 577.3/s, 4 workers 910.5/s로 약 1.58배였다.
- 짧은 batch에서는 시작 비용이 더 크므로 persistent worker와 긴 episode가 필요하다.
- 현재 기본 floor shoe는 shoulder 거리 약 0.497 m로 0.40 m envelope 밖이다.

따라서 state-based behavior cloning용 병렬 trajectory 생성은 가능하지만, 현재 위치의
floor-shoe 성공 demonstration을 바로 생성할 수 있다는 뜻은 아니다. 기구 도달성을 먼저
수정하고, image ACT에는 RGB-D episode writer와 normalization adapter를 추가해야 한다.

## 필수 실험 순서

| 순서 | 실험 | 최소 산출물 | 다음 단계 gate |
| ---: | --- | --- | --- |
| 1 | JD-edu 코드 기반 ACT 학습 재현 | dataset revision, config, checkpoint SHA, loss curve | 같은 입력으로 재실행 가능 |
| 2 | ACT 보수적 rollout | `n_action_steps=1`, ensemble off, supervisor trace | stale action 없이 평가 완료 |
| 3 | Receding Horizon 비교 | 같은 checkpoint·task에서 horizon/query 설정만 변경 | latency와 action age 기록 |
| 4 | Temporal Ensembling 비교 | per-step requery, ensemble 설정과 weight 기록 | jitter·성공률·개입 비교 |
| 5 | 외란·복구 평가 | shoe pose, 조명, grasp slip 등 고정 scenario matrix | 실패 원인이 분류됨 |
| 6 | 선택: SmolVLA fine-tuning | timebox된 단일 task 비교 보고서 | ACT 일정을 침범하지 않음 |

Receding Horizon과 Temporal Ensembling은 동시에 여러 변수를 바꾸지 않는다. 동일한
dataset split, checkpoint family, observation/action contract, task seed와 safety
profile을 유지하고 rollout 방식만 바꾼다.

## 공통 측정값

- dataset episode 수·시간·성공/실패 비율과 revision hash
- 학습 wall-clock, peak VRAM, checkpoint 크기와 best-step 선택 근거
- inference p50/p95 latency, policy query rate와 action age
- task success, cycle time, regrasp/retry, 사람 개입과 safe-stop
- measured joint velocity·acceleration·jerk와 supervisor reject 사유
- target action과 executed action의 차이, stale chunk/reset 사건

offline loss만으로 smoother 또는 성공률 향상을 주장하지 않는다. jitter 개선은 동일
실물 또는 동역학 조건의 measured motion과 task outcome으로 판단한다.

## SmolVLA 진입 gate

아래가 모두 충족될 때만 별도 timebox를 연다.

1. ACT 학습·checkpoint 선택·rollout을 재현할 수 있다.
2. ACT의 Receding Horizon과 Temporal Ensembling 비교표가 완성됐다.
3. 신발 task의 observation/action/normalization과 evaluation seed가 고정됐다.
4. language instruction과 평가 계약이 준비됐다. 단일 task에서도 pretrained 표현의
   이득을 비교할 수 있으므로 복수 task를 억지로 추가하지 않는다.
5. GPU 저장공간·VRAM·실험시간 상한과 중단 조건을 회의에서 승인했다.
6. VLA가 지연되거나 실패해도 A/B demo 일정이 영향을 받지 않는다.

허용 범위는 pretrained SmolVLA의 단일 신발 task 소규모 fine-tuning과 ACT 대비
offline/closed-loop 비교다. from-scratch pretraining, VLA에 맞춘 전체 데이터 재수집,
실물 safety gate 우회는 범위 밖이다. SmolVLA 참고 논문:
<https://arxiv.org/abs/2506.01844>.

π0.5, GR00T, Gemini Robotics 계열은 모델 구조, 데이터 규모, embodiment adaptation,
latency와 공개성만 조사표로 정리한다. 별도 자원 승인 전에는 구현 목표·완료조건·필수
의존성으로 넣지 않는다.

## 2026-09-09 PGripper 연구 근거와 실행 방향 보완

record_id: DAPIER-2026-09-09-pgripper-research-alignment

나는 2024–2026년 연구와 Hugging Face 공식 LeRobot 문서·모델 카드를 확인하고,
현재 PGripper 실험을 다음처럼 보완한다. 논문의 다른 로봇·작업 성공률을 우리 장비의
예상 성공률로 옮기지 않는다. 아래 단계는 개발 계획이며 구현·실물 검증 완료가 아니다.

### 직접 확인한 현재 한계

- 기존 ACT는 60분 동안 holdout imitation L1이 감소했지만 전체 작업 2회는 실패했다.
  같은 가중치에서 실행 chunk를 25/50 step으로 바꾸면 집기·들림까지 진행했다.
  따라서 학습 시간 부족으로 단정하지 않고 실행 주기와 관측 분포도 검사한다.
- 원래 MuJoCo 성공 시연은 6개, RoboTwin/SAPIEN 성공 시연은 10개다.
  RoboTwin의 블록 변화는 XY ±1mm·yaw ±1°로 작다. 수만 프레임이어도 다양한
  상황을 경험한 수만 개의 독립 시연은 아니다. expert 성공과 학습 정책 성공을 구분한다.
- PPO는 ACT가 아니라 expert 기준 궤적의 residual을 4초 구간에서 학습했다.
  이는 ACT+RL 통합 정책의 개선 근거가 아니며, 전체 작업 성공도 보장하지 않는다.
- 새 변환기는 원본 HDF5 해시·관측/명령·RGB를 검사하고 ACT 사본만 만든다.
  SAPIEN 8/MuJoCo 4개를 학습, SAPIEN 2/MuJoCo 2개를 검증에 둔다.
  500Hz의 20개 명령 중 끝 명령을 25Hz label로 쓰며 불완전한 마지막 구간은 제외한다.
  원래 첫 명령 label도 별도 보존한다. 이 선택은 개선 가설이지 검증된 정답이 아니다.
  SAPIEN 물리 상태를 MuJoCo PPO snapshot처럼 사용하지 않는다.
- 진행 중인 15분 mixed-data warm-start는 데이터 출처와 label 시점을 함께 바꾼
  탐색 실험이다. 개선되더라도 어느 변경 때문인지 분리해서 주장할 수 없다.

### 참고한 연구와 채택할 부분

| 근거 | 해당 연구/문서가 보여준 것 | 우리 적용 판단 |
| --- | --- | --- |
| [Mobile ALOHA, 2024](https://arxiv.org/abs/2401.02117) | 작업별 50개 시연과 기존 ALOHA 데이터 co-training으로 양팔 이동 조작 학습 | ACT/BC는 유효한 기준선. 같은 궤적 반복보다 작업 변화를 확보한다. |
| [RialTo, RSS 2024](https://real-to-sim-to-real.github.io/RialTo/) | 실측 장면의 digital twin, SIM RL, 실물 시연과의 distillation으로 강건성 개선 | 실물 보정은 마지막 확인만이 아니라 SIM 가정을 정하는 입력이다. |
| [RoboTwin 2.0, 2025](https://arxiv.org/abs/2506.18088) | 다양한 synthetic 시연, 구조적 domain randomization, 일부 실물 시연을 통한 전이 평가 | RoboTwin은 데이터 생성기로 유지하고 위치·외관·물리 변화를 명시적으로 설계한다. |
| [SmolVLA, 2025](https://huggingface.co/blog/smolvla) | 공개 데이터로 사전학습한 소형 VLA의 SO100/101 실험 | 사전학습 모델은 비교 후보. PGripper 양팔과 바로 호환된다고 가정하지 않는다. |
| [RoboPocket, 2026 preprint](https://arxiv.org/abs/2603.05504) | 예측의 약한 구간을 찾아 교정 데이터를 수집하는 반복으로 데이터 효율 개선 | 실패 상태에서 시작하는 복구 시연을 추가한다. 휴대폰 시스템 자체를 새로 구현할 필요는 없다. |
| [RL Tokens, 2026 연구 공개](https://www.pi.website/research/rlt) | 기존 VLA와 연결된 작은 RL 정책으로 정밀 접촉 구간 개선 | RL은 정상적인 기본 정책에 실제로 연결하고 병목 구간에 한정해 비교한다. 이 기법을 구현했다고 부르지 않는다. |

[HF ACT 안내](https://huggingface.co/docs/lerobot/act)는 ACT를 가벼운 시작점으로,
[SmolVLA 안내](https://huggingface.co/docs/lerobot/smolvla)는 약 50개 시연을 시작점으로
설명한다. 후자의 예시는 물체 위치 5곳×10회이며, 같은 연구의 25개 시연은 부족했다.
이는 모든 작업의 최소 표본 수를 증명한 규칙이 아니다.
[HIL 데이터 수집 안내](https://huggingface.co/docs/lerobot/hil_data_collection)는
실패 직전 개입과 복구 데이터를 다시 학습하는 루프를 제공한다.
[HIL-SERL 안내](https://huggingface.co/docs/lerobot/hilserl)는 시연·개입·성공 판정과
off-policy SAC를 결합하며, 처음에는 5–10초 작업을 권한다. 현재 PPO 실험과 다르다.

### 다음 실행 순서와 판정 기준

1. **제어 계약 검증부터:** 전문가 명령을 정책과 같은 25Hz hold, 500Hz rate limit,
   같은 초기 상태와 물리 판정기로 재생한다. 이 oracle 재생이 실패하면 neural 학습보다
   시간 정렬·제어 방식을 먼저 수정한다. 촬영 시점, 원래/실제 명령, 통계, 좌우 순서를 고정한다.
2. **원인 분리:** 원래 MuJoCo만/혼합 데이터와 interval-first/interval-end label을
   같은 초기 checkpoint·학습 step 예산·개발 seed로 비교한다. 필요한 최소 조합부터 실행한다.
   25/50 action-step 비교는 관측 수신 25Hz와 신경망 재계획 1/0.5Hz가 다름을 기록한다.
   긴 chunk가 진행을 돕더라도 외란 대응 지연을 함께 측정한다.
3. **시연 확장:** 먼저 도달 가능하고 충돌 없는 5개 정도의 작업 위치/배치에서 각 10회,
   약 50개 성공 시연을 시작 목표로 삼는다. 이후 실패 분포에 따라 늘린다.
   자세·시각·마찰·모터 지연 변화는 실측 범위를 기준으로 단계적으로 넓힌다.
   모든 랜덤화를 한꺼번에 크게 넣어 기구적으로 불가능한 작업을 만들지 않는다.
4. **교정 루프:** 집기, recipient 정렬/파지, 놓기의 실패 상태를 보존하고 그 상태에서
   전문가가 복구한 observation→action을 새로 수집한다. 실패한 정책 명령을 정답으로
   재사용하지 않는다. SIM에서 먼저 시행하고 실물 교정은 별도 현장 승인 후에만 한다.
5. **평가 고정:** 학습/개발/최종 test를 에피소드뿐 아니라 위치·장면 seed로 구분한다.
   개발 시 전체/단계별 성공, 낙하, 침투, 개입, 수행 시간, 지연을 기록한다.
   설정을 고른 뒤 미사용 20회 이상 최종 test를 시행한다. 잠정 18/20은 탐색 gate일 뿐
   실물 90% 성공 보증이나 통계적으로 확정된 성공률이 아니다. test로 튜닝하면 새 test를 둔다.
6. **RL은 병목에 연결:** grasp 유지·recipient 정렬 등 짧은 구간의 성공/실패부터 정의한다.
   ACT 단독과 ACT+bounded residual을 같은 초기 분포·전체 작업 지표로 비교한다.
   task success 없는 reference 추종 보상만 계속 학습하지 않는다. 필요 시 off-policy
   demonstration/replay 기반 기법을 검토하며 실물 자율 탐색은 지금 시작하지 않는다.
7. **실물 전환:** 장착 형상·카메라 내부/외부 파라미터·관절/그리퍼 범위·지연을 확인하고
   SIM에 반영한다. 상부 H201 metric depth는 인식/좌표변환용이며, 현재 두 손목 ACT가
   깊이를 학습한다고 쓰지 않는다. 새 물체 위치 작업에서 가시성이 부족하면 top 관측을
   별도 비교한다. 이후 소량 실물 시연, shadow, 승인된 저속 rollout 순서로 진행한다.

### 모델·자원 결정

ACT는 주 기준선으로 유지한다. 다만 ACT가 반드시 성공해야만 SmolVLA 비교를 허용하는
것은 아니다. 데이터/평가 계약이 고정되고 자원 예산이 맞으면 단일 task에서도 비교한다.
[SmolVLA 모델 카드](https://huggingface.co/lerobot/smolvla_base)는 task-specific
fine-tuning용 base 모델이며 완성된 우리 로봇 정책이 아니다.

[HF 메모리 안내](https://huggingface.co/docs/lerobot/hardware_guide)의 예시 조건
(batch 8, AdamW)은 ACT 약 2–6GB, SmolVLA 약 10–16GB다. 현재 8GB GPU에는
ACT가 우선이며, SmolVLA는 작은 batch/encoder freeze 또는
[PEFT](https://huggingface.co/docs/lerobot/peft_training)를 별도 측정해야 한다.
이 표는 근삿값이고 우리 해상도·양팔 환경에서의 VRAM 실측이 아니다.
대형 VLA, Isaac 이전, 원격 유료 GPU, 공개 dataset 업로드는 지금 추가하지 않는다.
관련성 있는 HF dataset도 형상·단위·카메라·action 의미·라이선스를 검사하기 전 섞지 않는다.

실행 원칙은 **시연 → 학습 → 실제 정책 rollout → 실패 상태의 교정 시연 → 재학습**이다.
동일 데이터에 대한 무제한 gradient update를 성공 전략으로 삼지 않는다. 같은 실패가
두 번 지속되면 시간만 늘리지 않고 데이터·관측·제어·목적함수 중 검증할 가설을 바꾼다.

## 회의/Notion에 넣을 문구

> **필수 경로: ACT. 선택 심화: VLA.** 먼저 JD-edu 기반 ACT 학습과 안전한 rollout을
> 재현하고 Receding Horizon·Temporal Ensembling의 성공률, latency, jitter와 복구를
> 비교한다. SmolVLA는 ACT 기준선과 데이터 계약이 고정되고 시간·GPU 예산이 맞을 때만
> pretrained model의 소규모 fine-tuning으로 시도한다. π0.5·GR00T·Gemini 계열은
> 이번 구현 범위가 아니라 최신 연구 조사 대상으로 관리한다.

## 안전 경계

ACT와 VLA 모두 policy 출력은 command가 아니라 proposal이다. 실제 실행은 독립
supervisor의 checkpoint identity, observation/action freshness, joint/rate/workspace,
base stationary, E-stop과 사람 승인 gate를 모두 통과한 경우에만 가능하다. 첫 실물
rollout은 가장 단순한 ACT 설정에서 시작하고, VLA를 실물 첫 진입 경로로 사용하지 않는다.
