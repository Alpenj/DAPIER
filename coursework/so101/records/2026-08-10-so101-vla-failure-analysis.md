# SO-101 wrist-only VLA 실패 분석과 일반화 개선 기록

`record_id: DAPIER-2026-08-10-so101-vla-failure-analysis`

## 결론부터

처음 wrist-only SmolVLA의 미사용 seed 성공률은 `2/10 (20%)`였다. rollout
영상, 환경 reset, action chunk와 학습 dataset을 나눠 조사한 결과 가장 큰 계약
오류는 IK teacher와 VLA evaluator의 초기 관절 자세가 달랐던 점이었다. 이를
맞추면 같은 checkpoint가 `6/10 (60%)`까지 올라갔다.

실행 horizon을 25 step으로 줄인 seed `800..809` 결과는 `8/10`이었지만,
그 seed를 설정 선택에 사용했으므로 최종 결과로 채택하지 않았다. 완전히 새 seed
`900..919`에서는 `8/20 (40%)`여서 30개 demonstration만으로는 공간
일반화가 부족하다는 사실을 확인했다.

그래서 새 IK success 60개를 더 수집해 wrist-only로 변환하고 10,000 update를
추가했다. 균형 validation seed `1100..1109`는 `8/10 (80%)`, 처음 보는
seed `1200..1219`는 `14/20 (70%)`였다. 20% 기준선보다 크게 나아졌지만
80% release 기준에는 두 번 부족해 마지막 5,000 update를 진행했다.

> 마지막 5,000 update checkpoint는 같은 validation에서 `5/10 (50%)`로
> 악화돼 선택하지 않았다. validation `8/10`이었던 +10k checkpoint를 고정해
> 새 seed `1400..1419`를 평가한 결과도 `14/20 (70%)`였다. 서로 다른
> 미사용 20회 집합 두 곳에서 70%가 재현됐으므로 개선은 확인했지만
> `vla_success_threshold_met=false`로 종료했다.

여기서 성공률은 MuJoCo의 XY ±25 mm cube randomization과 wrist-only 관측에서
측정한 simulation 수치다. 실물 카메라, 모터, 카드와 양팔 주행 성공률이 아니다.

## 실패를 어떻게 확인했나

실패 episode 영상은 대부분 approach와 grasp에서 갈렸다. 성공 영상은 gripper가
cube를 들어 tray로 옮겼지만, 실패 영상은 cube를 밀거나 비껴 잡은 뒤 빈 gripper로
transfer 동작을 계속했다. 실패의 max reward는 대체로 0.55 아래였고 성공은
1.58 이상이어서 reward threshold의 애매함보다 실제 grasp 분기 문제로 판단했다.

30개 expert dataset의 660 frame을 phase별로 다시 나눴다.

- observe: frame 0..29
- leave observe: 30..89
- approach: 90..189
- close: 190..269
- grasp: 270..299
- lift와 hold: 300..449
- transfer와 hold: 450..559
- release와 settle: 560..659

16개 대표 frame을 30 episode에서 검사했을 때 blue cube는 모두 wrist 영상에
보였다. training XY 범위도 대략 X -24.89..+23.76 mm,
Y -22.83..+24.48 mm였고 첫 평가점과 가장 가까운 training point 거리는
2.47..10.69 mm였다. 따라서 “물체가 손목 영상에 안 보임”이나 단순 범위 이탈만을
주원인으로 보지 않았다.

## 같은 checkpoint로 분리한 원인

아래 실험은 모두 cube XY ±25 mm, episode 최대 700 step 조건이다. seed
`800..809`는 설정을 비교한 뒤 validation으로 분류했고, seed `900..919`만
그 단계의 최종 미사용 평가로 사용했다.

| 실험 | reset pose | action 실행 horizon | seed | 결과 | 평균 max reward |
|---|---|---:|---:|---:|---:|
| 원래 evaluator | 기본 home | 50 | 800..809 | 2/10, 20% | 0.522494 |
| teacher reset 정렬 | pick-clear | 50 | 800..809 | 6/10, 60% | 1.206332 |
| 잦은 재계획 | pick-clear | 10 | 800..809 | 3/10, 30% | 0.725947 |
| 중간 horizon | pick-clear | 25 | 800..809 | 8/10, 80% | 1.400977 |
| 미사용 seed 확인 | pick-clear | 25 | 900..919 | 8/20, 40% | 0.860158 |

IK collector는 매 episode를
`[0, -45, 17.5, 90, 0, 100]`에서 시작했다. 반면 당시 표준 evaluator의
기본 home은 `[0, -35, 55, 35, 0, 100]`이어서 첫 관측부터 elbow가 37.5도,
wrist가 55도 차이 났다. 학습에서 본 적 없는 초기 상태를 policy 성능 문제로
섞어 측정한 것이 첫 번째 원인이었다.

10 step은 stochastic action chunk를 너무 자주 다시 뽑아 trajectory 일관성이
깨졌다. 50 step은 오차를 오래 실행했고, 25 step이 이 checkpoint에서는 더
안정적이었다. 다만 25를 고른 seed에서 얻은 80%는 validation 결과일 뿐이다.

## 데이터 일반화 보강

기존 미사용 seed `900..919`를 방향별로 보면 Y가 음수인 절반은 `2/10`,
양수인 절반은 `6/10`이었다. 기존 30개 dataset이 양쪽을 15개씩 포함했어도
SmolVLA가 작은 공간 차이를 안정적으로 일반화하지 못했다.

새 seed `1000..1059`에서 IK teacher를 다시 실행했다.

| 항목 | 결과 |
|---|---:|
| 새 IK success | 60/60 |
| 새 frame | 39,600 |
| top RGB XY 오차 평균 | 0.808 mm |
| top RGB XY 오차 최대 | 1.474 mm |
| 음의 X / 음의 Y sample | 37/60 / 35/60 |
| student 관측 | wrist RGB + 6축 state |
| student action | 다음 6축 action |

student 변환은 `observation.images.top`만 제거하고 episode/frame 수,
wrist/state/action, reward/success와 teacher contract hash를 확인했다. 기존
10,000-update checkpoint에서 새 60개 dataset으로 10,000 update, batch 4를
추가해 40,000 sample, 약 1.01 dataset epoch를 학습했다. 마지막 logged loss는
`0.019`였다.

| 새 checkpoint 평가 | 역할 | seed | 결과 | 평균 max reward |
|---|---|---:|---:|---:|
| +10k, action 25 | 균형 validation | 1100..1109 | 8/10, 80% | 1.423054 |
| +10k, action 25 | 미사용 final | 1200..1219 | 14/20, 70% | 1.313702 |
| +15k, action 25 | checkpoint validation | 1100..1109 | 5/10, 50% | 0.980495 |
| 선택 +10k, action 25 | 새 미사용 final | 1400..1419 | 14/20, 70% | 1.304290 |

70% 평가에서는 이전 음의 Y 영역이 `9/12`까지 개선됐다. 남은 6개 실패 중
5개는 음의 X에 있었다. 새 dataset에도 음의 X가 37개 있어 데이터 범위를 다시
바꾸지 않고 마지막 반 epoch에 해당하는 5,000 update만 더 실행했다. 이 마지막
iteration의 logged loss는 `0.014`까지 내려갔지만 validation 성공률은
`50%`로 떨어졌다. loss 감소와 task 성공률을 같은 것으로 보지 않고 +10k
checkpoint로 되돌렸다. 고정한 모델의 새 seed `1400..1419` 결과는 평균
sum reward `151.594174`, `14/20 (70%)`였다.

## 코드에서 닫은 재현성 구멍

- `SO101MujocoEnvConfig.home_action`을 노출하고 teacher의 pick-clear pose와
  같은 값을 기본값으로 연결했다.
- 직접 environment를 만들 때도 collector와 같은 XY ±25 mm가 기본이 되도록
  `cube_xy_randomization=0.025`로 맞췄다. ±40 mm는 별도 stress test 범위다.
- wrist VLA 평가 command builder는 `n_action_steps=25`, home action과
  cube randomization을 명시한다.
- evidence sidecar는 성공률뿐 아니라 evaluation action steps, home action과
  XY randomization까지 저장한다.
- action steps에는 bool이나 float가 들어오지 못하도록 positive integer 검증과
  회귀 test를 추가했다.

## 검증과 경계

| 검증 | 결과 |
|---|---:|
| runtime Ruff | PASS |
| runtime SO-101 MuJoCo test | 33/33 PASS |
| clean v0.6.0 patch apply | PASS |
| clean-room upstream asset hash | 14/14 PASS |
| clean-room SO-101 MuJoCo test | 33/33 PASS |
| physical rollout | 0회, 미실행 |

실물 follower ID 2 통신·전압 문제, wrist camera, stable serial-by-id와 실측
extrinsic이 해결되지 않았으므로 motor command를 보내지 않았다. 이번 결과는
simulation learned rollout의 실패 분석과 개선까지만 포함한다.

## AI 도움을 받은 방식

내가 IK teacher와 VLA student를 분리하고 실제 장비를 움직이지 않는 안전 범위,
80% release 기준과 최종 seed 분리 원칙을 정했다. AI는 eval JSON과 영상을
비교하고 phase·공간 방향별 실패를 계산하며, reset/action 계약을 코드와 test로
옮기고 반복 명령과 문서를 정리하는 일을 조금씩 도왔다. 학습 완료를 성공으로
바꾸어 쓰지 않고, validation과 final 수치를 분리하는 판단은 끝까지 유지했다.
