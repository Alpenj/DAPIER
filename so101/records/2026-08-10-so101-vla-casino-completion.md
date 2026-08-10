# SO-101 IK→VLA와 카지노 one-card 최신화 기록

`record_id: DAPIER-2026-08-10-so101-vla-casino`

## 이번에 끝내려 한 범위

SO-101 simulation의 top+wrist IK teacher와 wrist-only VLA student를 코드·데이터·
평가 결과로 분리하고, 카지노 양팔 작업은 실제 장비 전에 한 장 카드의 역할 분할과
bounded 기구학 기준선을 완성했다. 소스와 문서는 Git에 남기고 dataset, checkpoint,
영상과 receipt는 로컬 evidence 디렉터리에 분리했다.

여기서 완료는 software slice가 실행되고 수치가 기록됐다는 뜻이다. 연결되지 않은
카메라·serial·모터, 실제 카드와 두 번째 팔의 동작을 성공한 것처럼 포함하지 않는다.

## 실물 조립·캘리브레이션 이력의 경계

이전 장비 실습에서는 leader/follower 조립 후 raw tick을 읽고 calibration을 만들었고,
팔로워 목표 변화량을 5도로 제한한 저속 teleoperation에서 약 59~60 Hz 통신을 확인한
기록이 있다. 이 경험으로 joint 순서, motor ID, 전압, torque와 calibration을 먼저
확인해야 한다는 런북을 만들었다.

그러나 2026-08-04 후속 점검에서 follower ID 2가 torque-on 상태에서 통신을 잃었고,
교체 후보 모터에는 voltage protection 문제가 있었다. 따라서 이전 저속 teleop 기록을
현재 hardware-ready 또는 실 주행 완료로 승격하지 않는다.

2026-08-10 read-only inventory 결과도 다음 세 이유로 `blocked`다.

- 예상 SO-101 32×32 wrist camera가 없고 ASUS 내장 FHD/IR camera만 보임
- 안정적인 `/dev/serial/by-id` robot port가 없음
- CAD mount profile의 physical alignment가 아직 `false`

점검기는 device node를 열지 않았고 motor command와 physical rollout도 0회다.

## top+wrist IK expert 데이터

privileged top RGB로 cube XY를 추정해 IK를 실행하되, 같은 frame에 wrist RGB,
measured state와 action을 기록했다. seed `400..429`에서 `30/30` episode가
sim success 조건을 만족했고 총 `19,800` frame을 얻었다.

| 항목 | 결과 |
|---|---:|
| IK episode | `30/30` |
| frame | `19,800` |
| top RGB XY 오차 평균 | `0.817 mm` |
| top RGB XY 오차 최대 | `1.577 mm` |

student 변환에서는 `observation.images.top`만 제거했다. wrist image,
`observation.state`, action과 teacher contract SHA-256은 유지했다.
`top,wrist → ik_expert`, `wrist-only → vla` 외 조합은 fail-closed이며,
wrist-only 실행이 IK로 자동 fallback하지 않는다.

## wrist-only SmolVLA 학습과 평가

RTX 5050 8GB에서 LeRobot SmolVLA와 pretrained SmolVLM2-500M을 사용했다.
학습량을 늘릴 때마다 동일한 held-out seed `800..809` 10회를 평가했다.

| checkpoint | 학습 범위 | 마지막 loss | held-out 결과 | 평균 max reward |
|---|---:|---:|---:|---:|
| pipeline smoke | 1 update | - | `0/1` | - |
| 1k | 1,000 update, batch 1 | `0.250` | `0/10` | `0.238817` |
| 5k | 5,000 update, batch 4 | `0.037` | `2/10` | `0.525192` |
| final fine-tune | 누적 10,000 update, batch 4 | `0.021` | `2/10` | `0.522494` |

최종 checkpoint는 `40,000` sample, dataset frame 기준 약 `2.020202 epoch`를
봤다. 최종 평균 sum reward는 `143.967292`다. 학습과 평가 실행은 완료했지만
성공률 `20%`로 release 기준 `80%`를 통과하지 못했다. sidecar는 이를
`vla_trained=true`, `vla_evaluated=true`,
`vla_success_threshold_met=false`로 각각 기록하고 physical rollout은 false로
유지한다.

## 카지노 one-card 양팔 역할 기준선

왼팔은 deck을 안정화하고 오른팔은 pick/place를 수행하도록 상태와 역할을 분리했다.
각 Cartesian delta는 최대 `0.02 m`로 제한하고 vacuum attachment, card table
clearance와 target radius를 검사한다.

seed `1000..1099`의 결정론적 3D 기구학 실행은 `100/100`, 평균
`32.92` step이었다. `casino_dealer` unit test `20/20`과 3인 블랙잭
planner JSON 생성도 통과했다.

이 모델에는 rigid/deformable contact physics, 실제 card vision, suction/gripper,
충돌 센서, learned policy와 physical arm이 없다. 따라서 CardBench G6나 실물
카지노 딜 성공률이 아니라 task-level software baseline이다.

## 검증 결과

| 검증 | 결과 |
|---|---:|
| SO-101 MuJoCo test | `33/33 PASS` |
| 카지노 test | `20/20 PASS` |
| IK expert | `30/30`, `19,800 frames` |
| 최종 VLA held-out | `2/10 (20%)`, 기준 미달 |
| 카지노 one-card 기구학 | `100/100` |
| physical wrist gate | `BLOCKED`, 비구동 |

## AI가 도운 부분

프로젝트 방향, SO-101에서 시작해 카지노 양팔로 확장하는 순서와 실제 장비의
안전 판단은 내가 정했다. AI는 명령어와 파일 구조 정리, 반복적인 test와 CLI
작성, training/evaluation 로그 비교, provenance와 실패 기준을 문서로 남기는
일을 조금씩 도왔다. 물리 조립·배선·전압·모터 상태와 실제 episode 성공 여부는
사람이 장비 앞에서 직접 확인해야 한다.

## 다음 물리 세션의 첫 gate

1. follower ID 2 통신과 정격 전압 문제를 해결한다.
2. wrist camera와 stable serial-by-id가 실제로 보이는지 read-only audit을 다시 한다.
3. 정지한 arm에서 intrinsics, image rotation과 gripper-to-optical extrinsic을 측정한다.
4. torque-off calibration 확인 뒤 5도 제한 단일 관절 jog부터 사람이 검수한다.
5. real episode와 learned rollout은 위 네 gate가 통과한 뒤 별도 evidence로 기록한다.

현재 완료하지 않은 것은 실물 wrist camera calibration, 실제 카드 episode,
성공하는 VLA, sim-to-real과 실 주행이다.
