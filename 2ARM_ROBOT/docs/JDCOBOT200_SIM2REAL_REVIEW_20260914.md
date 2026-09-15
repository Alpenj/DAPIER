# JDcobot200 코드를 SO-101 양팔에 차용하며 확인한 것

record_id: DAPIER-2026-09-14-jdcobot200-sim2real-review

나는 강사님이 제공한 레포를 clone해서 읽고, 우리 양팔 PGripper의 합성 데이터와
ACT 입력 경로에 필요한 검증을 적용했다. 원본 레포에 새 브랜치를 만들거나 변경을
push하지 않았다. 원본 로봇의 관절각·보정 파일·모델·서보 드라이버를 SO-101에 덮어쓰지 않았다.

검토 기준은 [`8b51972`](https://github.com/JD-edu/jdcobot200_imitation_learning/tree/8b51972f874171c6ef5136b2c008278e88b2e3ce)이다.
112개 Python 파일 전체를 실행 없이 AST 검사하고, 중복 파일을 구분한 뒤 실제 제어,
FK/IK, 합성 생성, 변환, 학습, 추론의 핵심 경로를 추적했다. 원본의 실물 코드는 실행하지 않았다.

## 먼저 정리한 결론

- **차용:** 합성 원본 보존, 변환 후 실제 학습 입력 재로딩, phase별 검사,
  명시적 제어 주기, 초기 자세·물체 위치를 변화시키되 실패 후보를 구분하는 접근.
- **이미 있어 재사용:** 양팔 5-DOF IK, 부드러운 관절 궤적, 충돌 검사,
  접촉·들림·인계·안착 순서 판정, 학습 전용 정규화, ACT chunk padding.
- **그대로 차용하지 않음:** JDcobot200 tick/영점/부호, 단일팔 6축 배열,
  다음 관측을 command라고 섞는 label, 물체 weld 또는 외부 spring force.

이번 새 SO-101 SIM expert는 seed 4100–4103의 **4/4 시연에서 전체 인계·배치**를
통과했다. 25 Hz target, 500 Hz command, 1 kHz physics 조건이다. 이것은 좁은 고정
장면의 scripted expert 결과이며 학습된 ACT의 성공률이나 실물 성공률이 아니다.

## 레포를 읽는 순서

| 경로 | 역할 | 우리 프로젝트에서 읽을 이유 |
| --- | --- | --- |
| `1_jdcobot200_hardware_control` | torque, 영점, 한계, 그리퍼, homing | tick·보정·실물 상태의 관계 |
| `2_*`–`5_*`, `jdcobot200_urdf`, `MUJOCO_simulation` | URDF/MJCF, FK, 위치·속도 제어, 초기 예제 | 모델과 motor command가 같은 것이 아님을 확인 |
| `6_jdcobot200_FK`, `7_FK_safe_zone` | 실측 tick→FK, 경로 전체 clearance 검사 | 목표 한 점이 아니라 시작점부터 검사 |
| `8_jdcobot200_IK`, `9_jdcobot200_pick_and_place` | differential IK와 단계별 작업 | 5자유도에서 가능한 목표 제약 |
| `9_robot_teaching_data_collection` | 실제 서보 1–6 티칭·JSON 재생 | 그리퍼를 포함한 실물 경로 |
| `10_create_virtual_dataset`, `11_lerobot_dataset` | randomized expert→NPZ→LeRobot | 상태·명령·시간 의미 추적 |
| `12_data_recording_using_synth_data`, `13_dual_cam_full_cources` | 검사·ACT·단일/dual camera 추론 | 데이터와 모델 설정의 연결, weld 의존성 |
| `13_full_IL_sequence_with_no_weld` | 접촉 시작 조건·보조 힘·dual-camera ACT | 최신 구현의 개선점과 남은 sim-to-real 차이 |

`SO101MinkSolver`라는 클래스 이름이 있어도 실제 모델은 JDcobot200이다.
이름이나 같은 STS 계열 모터만으로 링크 길이·축 방향·TCP가 호환되지 않는다.
`depth_cam` 자료는 별도의 카메라 bring-up 참고이며, 최신 ACT 입력은 front/wrist RGB다.
우리 H201 metric depth 경로를 대체하지 않는다.

## 실물 파지 코드는 있는가?

실물 그리퍼 제어는 있다. 다만 다음 세 가지를 나눠 읽는다.

1. [`218.all_servo_homing_with_gripper.py`](https://github.com/JD-edu/jdcobot200_imitation_learning/blob/8b51972f874171c6ef5136b2c008278e88b2e3ce/1_jdcobot200_hardware_control/218.all_servo_homing_with_gripper.py)는
   실제 6번 그리퍼를 포함해 서보에 목표를 보낸다. homing은 물체 파지 성공 판정이 아니다.
2. [`116_cobot_teaching_cali.py`](https://github.com/JD-edu/jdcobot200_imitation_learning/blob/8b51972f874171c6ef5136b2c008278e88b2e3ce/9_robot_teaching_data_collection/116_cobot_teaching_cali.py)는
   1–6번 실제 위치를 읽고 티칭 포인트를 저장·재생한다. 그리퍼를 여닫는 시퀀스를
   만들 수 있지만, 영상·접촉·물체 들림을 기록하는 ACT episode recorder와는 다르다.
3. [`sim2real_pick_n_place.py`](https://github.com/JD-edu/jdcobot200_imitation_learning/blob/8b51972f874171c6ef5136b2c008278e88b2e3ce/jdcobot200_urdf/sim2real_pick_n_place.py)는
   실제 서보 목표를 전송하지만 그리퍼 파지는 코드에 **더미 동작**으로 표시되어 있다.
   `move_robot_with_profile()`의 실패 반환을 호출부가 확인하지 않는 점도 있다.

따라서 “실물 코드가 없다”가 아니라 **실물 제어·티칭과 최신 ACT SIM 추론이 별도
경로**라는 판단이다. 검토한 버전에서 최신 ACT checkpoint를 실물에 연결하고 물체
파지를 검증하는 완결된 실행 경로는 확인하지 못했다. 강사님의 별도 실험 여부를
이 코드만으로 부정하는 것은 아니다.

티칭 UI의 연결 함수가 모든 motor feedback을 먼저 모으고 현재 위치를 Goal로 쓴 뒤
torque를 켜는 순서는 참고할 만하다. 반면 별도 `torque_on()`은 위치 읽기가 실패해도
torque enable을 진행할 수 있다. 부분 함수만 보고 안전성을 전체로 확대하지 않는다.
우리 실물 동작에는 기존 현장 승인·장치 확인·오류 중단을 유지한다.

## 합성 데이터에서 특히 주의한 차이

### 1. `no_weld`와 순수 접촉 파지는 다르다

최신 [생성기](https://github.com/JD-edu/jdcobot200_imitation_learning/blob/8b51972f874171c6ef5136b2c008278e88b2e3ce/13_full_IL_sequence_with_no_weld/generate_synthetic_trajectories.py)는
양쪽 pad 접촉 뒤 물체의 gripper-local offset을 저장하고,
`data.xfrc_applied`에 spring-damper 힘을 넣는다. gain은 180/4, 힘 상한은 3 N이다.
해당 scene의 물체 질량은 0.01 kg이므로 3 N 상한은 물체 중력의 약 30.6배다.
항상 3 N을 쓴다는 뜻은 아니지만 실물에는 없는 도움일 수 있다.

또한 offset이 설정된 뒤에는 접촉이 사라져도 open/release 조건 전까지 힘이 유지될 수 있다.
최신 [추론기](https://github.com/JD-edu/jdcobot200_imitation_learning/blob/8b51972f874171c6ef5136b2c008278e88b2e3ce/13_full_IL_sequence_with_no_weld/infer_lerobot_act_mujoco.py)는
그리퍼 open만이 아니라 목표 거리·높이·연속 확인 조건으로 힘을 해제한다.
README의 “open에서 즉시 해제” 설명과 생성/추론 동작을 함께 확인해야 한다.

우리 적용은 이 힘을 가져오는 것이 아니라, 공통 물리 step에서 `xfrc_applied`,
`qfrc_applied`, active block connect/weld를 검사하는 것이다. 그리퍼 내부 joint
연동 equality는 물체 부착과 구분한다. 외력 교란 실험은 이 순수 접촉 기준과 별도로
설계해야 하며, 이 검사만으로 모든 물리 모델의 정확성이 증명되지는 않는다.

### 2. `action`이라는 이름만 같고 학습 목표는 다르다

원본 [변환기](https://github.com/JD-edu/jdcobot200_imitation_learning/blob/8b51972f874171c6ef5136b2c008278e88b2e3ce/13_full_IL_sequence_with_no_weld/convert_to_lerobot.py)의
`make_next_actions()`는 저장된 expert `action` 대신 **다음 시점의 측정 joint state**를
학습 label로 만든다. 50 Hz 자료를 stride 5로 줄인 뒤 이동시키므로 다음 상태는
20 ms가 아니라 100 ms 뒤다. 마지막에는 마지막 state를 반복한다.

이는 가능한 학습 설계지만, 실제 actuator target과 동일하지 않다. servo lag, 접촉,
rate limit이 있으면 requested target, sent command, measured next state가 달라진다.
우리는 `mj_step` 이전의 관측과 requested target을 결합하고 실제 보낸 command도
별도로 저장한다. SAPIEN interval-first/end label은 출처를 명시하며 묵시적으로 섞지 않는다.

| 항목 | 강사님 최신 예제 | 현재 SO-101 양팔 계약 |
| --- | --- | --- |
| 관절 배열 | arm 5 + gripper 1 | left 6 → right 6, 총 12 |
| 그리퍼 | 해당 MJCF joint 단위 | 학습 경계 0..1, simulator 제어는 별도 변환 |
| 영상 | front RGB + wrist RGB | left wrist RGB + right wrist RGB |
| depth | 해당 ACT 입력에 없음 | H201 원본 metric depth는 인식/IK용 별도 경로 |
| 관측 주기 | 기본 10 Hz | 25 Hz |
| 50-action chunk | 5초 | 2초 |
| 학습 label | downsample 이후 next measured state | 명시한 command source |
| 파지 근거 | contact 시작 + 보조 힘 | 양 pad force·들림·연속 유지·인계·지지·해제 |

### 3. 데이터가 정상이어도 실행 주기가 달라지면 실패할 수 있다

원본 IK는 한 제어 주기를 `iterations`로 나눠 적분한다. 이 점은 올바른 참고다.
원본의 `round(dt / timestep)` 방식은 나누어떨어지지 않는 임의 설정에서 실제 시간이
달라질 수 있다. 우리 경로는 target/command/physics 주기가 정확히 맞는지 검사한다.

`n_action_steps=50`이면 관측을 매번 만들더라도 ACT의 새 chunk 요청은 매번 발생하지
않을 수 있다. 10 Hz에서 이는 최대 5초 실행 구간이다. 우리 기준은 우선 1-step 실행,
이후 같은 checkpoint와 scene에서 horizon만 바꿔 비교하는 것이다.

### 4. 최종 위치만 맞는 것은 pick-and-place 증거가 아니다

원본 최신 추론의 최종 `success`는 XY 오차와 Z 높이만 결합한다. report에 접촉과
grasp/release 항목이 있어도 최종 boolean이 그것을 모두 요구하는 것은 아니다.
밀어서 목표에 보낸 경우나 도움 힘이 남은 경우를 구분해야 한다.

우리 기존 `TaskProgress`의 접촉→들림→수신측 파지→독립 유지→테이블 지지→해제·안착
순서 검증을 유지했다. partial pick을 full handover 성공으로 승격하지 않는다.

### 5. randomization은 개수가 아니라 변화 범위로 읽는다

원본은 초기 관절, 물체 XY, waypoint, 속도를 변화시키고 실패 후보를 재시도한다.
단, 실패 원본을 버리고 성공만 남기면 어떤 구간이 어려웠는지 잃을 수 있다.
우리는 실패 archive를 보존하고 seed 기반 분할을 먼저 정한 뒤 성공 후보를 선별한다.
탈락 후 train/holdout을 다시 섞지 않는다.

이번 우리 4개 시연은 초기 팔 관절 ±0.5°만 바꾸고 물체·치구는 고정했다.
수천 frame을 수천 개의 독립 장면으로 세지 않는다. 접힌 실물 초기 자세, 물체 위치 변화,
손목 영상 차이, 마찰·하중 차이에 대한 일반화는 여전히 별도 실험이다.
고정 waypoint를 쓰는 현 expert에 물체만 무작위로 옮기는 기능을 덧붙이지 않았다.

## 적용한 변경과 검증 범위

- [`pgripper_dataset.py`](../sim/mobile_dual_so101/pgripper_dataset.py): 읽기 전용
  NPZ/JPEG audit. 12축·단위·25 Hz 연속 시각·label 출처·dense requested/sent 정렬,
  archive hash·분할·성공 metadata·양 손목 RGB 전체 decode를 검사한다.
- [`pgripper_learning.py`](../sim/mobile_dual_so101/pgripper_learning.py):
  `action_sent`, 제어 range/주기, archive hash와 action 이름을 기록한다.
  생성 후와 학습 전에 audit를 실행한다. 성공 시연이 양쪽 분할에 남아야 한다.
- [`pgripper_execution.py`](../sim/mobile_dual_so101/pgripper_execution.py):
  expert와 evaluator의 공통 step에서 외부 보조 힘·물체 부착을 거부한다.
- [`test_pgripper_dataset.py`](../sim/mobile_dual_so101/test/test_pgripper_dataset.py):
  정상 입력뿐 아니라 오른팔 단위 오류, 잘못된 label, 이미지 손상, 분할 중복,
  보조 힘과 부착의 부정 대조군을 실행한다.

새 dependency나 별도 학습 프레임워크는 추가하지 않았다. 기존 합성 archive의 없는
증거를 자동 생성하지 않는다. 오래된 파일이 canonical success 또는 명령 정렬 근거를
갖추지 못했다면 원본을 보존한 채 재검증해야 한다. 검증기는 metadata와 무결성을
확인하는 것이지 성공 metadata를 물리 재생 없이 독립 증명하는 도구는 아니다.
RGB shape 검사도 실제 카메라 identity·intrinsics/extrinsics 검증을 대신하지 않는다.

## 이번 직접 실행 결과

환경: Python 3.12.3, NumPy 2.2.6, MuJoCo 3.8.1, EGL. 실제 serial/카메라/모터 접근 0건.
비식별 수치와 해시는 [검증 요약 JSON](evidence/JDCOBOT200_ADOPTION_20260914.json)에 함께 남겼다.

| seed | frame 수 | SIM 시간(s) | 전체 expert task |
| ---: | ---: | ---: | --- |
| 4100 | 2,103 | 84.108 | PASS |
| 4101 | 2,111 | 84.416 | PASS |
| 4102 | 2,105 | 84.196 | PASS |
| 4103 | 2,113 | 84.514 | PASS |

- train: episode 0/1/2, **6,319 frame**; holdout: episode 3, **2,113 frame**.
- 전체 **8,432 frame / 손목 JPEG 16,864개** decode와 dense command 정렬 검사 PASS.
- 최대 기록 penetration은 약 **0.0733 mm**. 모델·물리 설정에 한정된 SIM 수치다.
- 원본 합성 데이터는 공개 GitHub에 올리지 않고 로컬에 보존한다.
- 데이터 manifest SHA-256: `7d3856c903ab2a3eff2c578399bc7385b15be6b468ad856711d132a07df98e14`.
- PGripper 회귀 **24/24**, evaluator 회귀 **6/6 PASS**.
- 실제 두 손목 영상과 12축 상태로 CUDA ACT **1 optimizer step**, checkpoint 저장 PASS.
  CPU에서 checkpoint 재로딩 후 holdout 관측만 넣어 finite `(1, 12)` action을 확인했다.
- holdout normalized L1은 **0.9242 → 1.0065**로 개선되지 않았다. 연결 smoke의
  성공을 성능 개선으로 바꾸어 쓰지 않으며, 이번 checkpoint의 task success는 미검증이다.
- 위 수치는 로컬 검증이다. 기존 PR #56의 MuJoCo CI는 이번 변경 전부터 실패 상태였으며
  CI 전체 통과·merge-ready를 주장하지 않는다. 기존 CI 설정·의존성 변경은 이 검토에
  임의로 섞지 않았다.

## 추가 학습용 실습 순서

현재 프로젝트 루트에서 실행한다. `SIM_PYTHON`은 LeRobot ACT와 MuJoCo가 설치된
검증 환경의 Python이며 `DAPIER_SO101_MJCF`는 그 환경의 SO-101 모델이다.
이번 실행 환경과 다른 MuJoCo 버전에서 같은 접촉 결과를 보장하지 않는다.

```bash
export SIM_PYTHON=/path/to/lerobot-env/bin/python
export DAPIER_SO101_MJCF=/path/to/so101_new_calib.xml
export MUJOCO_GL=egl

# 먼저 실패를 잡아내는 작은 검사
"$SIM_PYTHON" -m unittest discover \
  -s 2ARM_ROBOT/sim/mobile_dual_so101/test -p 'test_pgripper_dataset.py' -v

# 새 경로에만 생성한다. 기존 데이터에 overwrite하지 않는다.
"$SIM_PYTHON" 2ARM_ROBOT/sim/mobile_dual_so101/pgripper_learning.py collect \
  --output /path/to/new-private-sim-dataset --episodes 4 --workers 2 \
  --seed 4100 --policy-target-hz 25 --physics-substeps 2

"$SIM_PYTHON" 2ARM_ROBOT/sim/mobile_dual_so101/pgripper_dataset.py \
  --dataset /path/to/new-private-sim-dataset

# 이것은 성능 학습이 아니라 데이터→ACT→저장 연결 확인이다.
"$SIM_PYTHON" 2ARM_ROBOT/sim/mobile_dual_so101/pgripper_learning.py train \
  --dataset /path/to/new-private-sim-dataset --output /path/to/new-act-smoke \
  --minutes 1 --max-steps 1 --seed 4100
```

학습 질문:

1. `state[t]`, requested `action[t]`, `action_sent[t]`는 어떤 시점의 값인가?
   → 관측은 물리 step 전, 뒤의 두 값은 요청 목표와 실제 보낼 목표다. 같다고 가정하지 않는다.
2. 왜 next state를 그대로 actuator command라고 쓰면 결과가 달라지는가?
   → servo lag·rate limit·접촉이 목표와 실제 위치 사이를 바꾸기 때문이다.
3. `uses_weld=false`만으로 실물 파지 전이가 입증되는가?
   → 외부 힘·pose 갱신·물리 설정과 실제 접촉 유지도 확인해야 한다.
4. 50 frame chunk가 두 시스템에서 같은 시간인가?
   → 10 Hz는 5초, 25 Hz는 2초다.
5. 데이터 검사 PASS 다음 무엇을 확인하는가?
   → 같은 clock·제한의 expert replay, ACT closed-loop 평가, 실패 구간 교정 시연이다.
6. 현재 4/4로 일반화 성공률을 추정할 수 있는가?
   → 아니다. 고정 치구의 작은 초기 자세 변화에 대한 연결 검증이다.

다음 실험은 물체 위치 변화와 초기 자세 변화를 한 번에 섞지 않고 하나씩 확대한다.
장면 변화가 생기면 양팔 IK·충돌·카메라 시야를 함께 다시 확인한다.
실물은 현재 사용자 결정에 따른 별도 현장 승인 뒤에만 실행한다.

## 원본 결과와 출처를 읽을 때

강사님의 [진행상황 정리](https://github.com/JD-edu/jdcobot200_imitation_learning/blob/8b51972f874171c6ef5136b2c008278e88b2e3ce/13_full_IL_sequence_with_no_weld/%EC%A7%84%ED%96%89%EC%83%81%ED%99%A9_%EC%A0%95%EB%A6%AC.txt)는
1 episode 생성, CPU ACT 1-step, 2-step SIM 추론까지를 smoke로 기록하며,
50-episode/20,000-step 학습은 아직 실행하지 않았다고 명시한다.
그 정리는 해당 버전의 보고이며 강사님의 모든 후속 실험 현황을 뜻하지 않는다.

기존 사용 허가 기록은 [JDcobot200 자산 고지](../sim/jdcobot200_dual/THIRD_PARTY_NOTICE.md)에
유지한다. 이번에는 원본 모델·개인 보정값을 추가 반입하지 않고 검증 아이디어를
우리 기존 SO-101 경로에 맞춰 구현했다. Notion 원문·장치 식별자·원시 데이터는 공개하지 않는다.
