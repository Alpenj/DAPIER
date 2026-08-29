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
4. VLA용 language instruction과 복수 task가 실제 연구 질문을 만들 만큼 준비됐다.
5. GPU 저장공간·VRAM·실험시간 상한과 중단 조건을 회의에서 승인했다.
6. VLA가 지연되거나 실패해도 A/B demo 일정이 영향을 받지 않는다.

허용 범위는 pretrained SmolVLA의 단일 신발 task 소규모 fine-tuning과 ACT 대비
offline/closed-loop 비교다. from-scratch pretraining, VLA에 맞춘 전체 데이터 재수집,
실물 safety gate 우회는 범위 밖이다. SmolVLA 참고 논문:
<https://arxiv.org/abs/2506.01844>.

π0.5, GR00T, Gemini Robotics 계열은 모델 구조, 데이터 규모, embodiment adaptation,
latency와 공개성만 조사표로 정리한다. 별도 자원 승인 전에는 구현 목표·완료조건·필수
의존성으로 넣지 않는다.

## 회의/Notion에 넣을 문구

> **필수 경로: ACT. 선택 심화: VLA.** 먼저 JD-edu 기반 ACT 학습과 안전한 rollout을
> 재현하고 Receding Horizon·Temporal Ensembling의 성공률, latency, jitter와 복구를
> 비교한다. SmolVLA는 ACT 결과가 고정되고 시간·GPU·복수 task 데이터가 남을 때만
> pretrained model의 소규모 fine-tuning으로 시도한다. π0.5·GR00T·Gemini 계열은
> 이번 구현 범위가 아니라 최신 연구 조사 대상으로 관리한다.

## 안전 경계

ACT와 VLA 모두 policy 출력은 command가 아니라 proposal이다. 실제 실행은 독립
supervisor의 checkpoint identity, observation/action freshness, joint/rate/workspace,
base stationary, E-stop과 사람 승인 gate를 모두 통과한 경우에만 가능하다. 첫 실물
rollout은 가장 단순한 ACT 설정에서 시작하고, VLA를 실물 첫 진입 경로로 사용하지 않는다.
