# 실물 전 Jitter 감소 준비: Temporal Ensembling

이 기록은 ROBOTIS OMY 영상과 공식 기술 문서를 보고 DAPIER 양팔 ACT 경로에
적용할 수 있는 부분을 hardware-free 상태에서 먼저 구현한 결과다. 영상의 자동
음성 자막은 약 2분 34초 동안 대부분 [music]이고 1분 59초 부근의 Hey 외에는
기술 설명이 없다. 따라서 아래 내용은 음성 전사가 아니라 화면 오버레이와
ROBOTIS 기술 문서를 시간대별로 대조한 요약이다. 영상 전문이나 자막을 저장소에
복제하지 않는다.

## 영상에서 확인한 내용

| 구간 | 화면 내용 | 확인한 수치/관찰 |
| --- | --- | --- |
| 00:00–00:25 | Next-Generation Policies on OMY, LeRobot 연결 구조 | ROS 2–Zenoh bridge가 observation/action을 변환 |
| 00:30–01:00 | VLA-JEPA, FastWAM, MolmoAct2, GR00T N1.7 비교 | 동일한 pick-and-organize 설정과 90 teleop episodes, 3 cameras |
| 01:00–01:20 | 데이터 환경과 모델별 rollout | wrist camera와 외부 카메라, joint-space chunk 정책 |
| 01:21–01:43 | GR00T N1.7, Chunk | 76 ms/chunk, Plate 1 100/100, Plate 2 100/100, Plate 3 50/40, Very Jittery와 joint velocity 진동 |
| 01:44–02:00 | GR00T N1.7, Ensemble | 같은 76 ms/chunk, Plate 3 60/60, Smoother, hold 구간의 joint velocity 감소 |
| 02:00 이후 | 성공률·지연 표와 확장 구조 | overlapping action predictions의 지수 가중 평균으로 소개 |

첨부 이미지의 요약은 큰 방향에서 맞다. 다만 “latency를 극복한다”는 표현은
정확히 나눠서 봐야 한다. Temporal Ensembling은 모델 추론 시간을 줄이지 않는다.
여러 시점에서 예측한 겹치는 joint target을 결합해 re-plan 경계의 command step과
실물 응답 지연 때문에 reach가 얕아지는 현상을 완화한다. 추론이 control tick보다
느린 경우에는 별도로 asynchronous inference 또는 Real-Time Chunking과 buffer
underrun 측정이 필요하다.

## 이번 구현

shoe_sorting_data.temporal_ensemble은 ROS 2, serial, LeRobot, Torch를 import하지
않는 순수 Python 모듈이다.

- TemporalEnsembler: oldest-first ACT 가중치 exp(-c*i)로 겹치는 chunk를 결합
- 입력 gate: 고정 chunk/action shape, finite 값, monotonic timestamp, 최대 source age
- disagreement gate: 동일 timestep을 가리키는 chunk 간 joint별 차이가 한계를 넘으면 거부
- transactional reject: stale/NaN/disagreement 입력이 들어와도 기존 buffer를 바꾸지 않음
- command_stream_metrics: command delta, velocity, acceleration, jerk의 RMS/P95/max
- shoe_jitter_prep: deterministic synthetic A/B report 생성
- 모든 결과: control_authorized=false, hardware_execution=NOT_ATTEMPTED

positive c는 오래된 prediction을 더 무겁게 두고, c=0은 균등 평균이다. 음수는
새 chunk를 더 우선해 이 준비 단계의 안전한 기본 방향과 반대이므로 거부한다.
ensemble 출력은 joint limit이나 rate limit을 대신하지 않으며, 이후에도 반드시
독립 SafetySupervisor의 reject-only gate를 통과해야 한다.

    cd 2ARM_ROBOT/src/shoe_sorting_data
    PYTHONPATH=. python -m shoe_sorting_data.temporal_ensemble \
      --output /tmp/dapier_jitter_smoke.json --fps 30
    PYTHONPATH=. python -m unittest test.test_temporal_ensemble -v

## 실물 전에 필요한 A/B 순서

1. **Recorded replay**: 같은 held-out observation과 checkpoint로 raw newest-chunk와
   ensemble command를 생성한다. observation timestamp, inference start/end,
   chunk ID, source frame, contributor 수, disagreement를 함께 기록한다.
2. **MuJoCo**: 동일 seed·초기 pose·target에서 raw/ensemble을 반복한다. policy
   command와 measured joint를 분리해 기록하고 task success뿐 아니라 hold velocity,
   acceleration, jerk, tracking lag, safety reject를 비교한다.
3. **Parameter sweep**: c=0, 0.01을 시작점으로 작은 범위만 비교한다. chunk size,
   query frequency, action execution horizon을 한 번에 바꾸지 않는다. 고정 c는
   반응성과 grasp depth의 trade-off가 있으므로 결과 없이 큰 값으로 올리지 않는다.
4. **Camera/time audit**: RGB와 depth의 timestamp skew, frame age, dropped frame,
   inference latency, command publish 주기를 먼저 측정한다. 카메라 위치 변경은 같은
   dataset/checkpoint A/B와 섞지 않는다.
5. **Approved physical canary**: 별도 현장 승인 후에도 base 고정, 낮은 속도/effort,
   작은 workspace, 한 번의 bounded rollout, E-stop과 watchdog 준비 상태에서만 한다.
   raw와 ensemble 각각의 exact command와 abort threshold는 실행 직전에 지정한다.

## 실물 canary의 사전 중단 조건

아래 값은 실제 profile과 simulator baseline을 얻은 뒤 수치로 채운다. 비어 있는
상태에서는 실물 실행을 승인하지 않는다.

- joint별 lower/upper limit, max delta/step, max velocity/effort
- observation/feedback/proposal max age
- RGB–depth 최대 timestamp skew와 허용 dropped-frame 비율
- chunk disagreement limit과 buffer underrun 허용 횟수
- measured tracking error, acceleration, jerk의 abort threshold
- E-stop, watchdog, communication-loss fail-safe의 현장 검증 결과
- 정확한 checkpoint SHA-256, hardware profile SHA-256, calibration identity

Temporal Ensembling이 줄이는 것은 policy command seam이다. servo gain, backlash,
전원, bus timing, calibration, camera timestamp 불일치로 생기는 jitter는 이 평균만으로
해결되지 않는다. raw command는 부드럽지만 measured joint가 떨리면 controller/기구
경로를 먼저 조사하고, raw command부터 튀면 policy scheduling/ensemble 경로를 본다.

## 근거

- [ROBOTIS 영상: AI MANIPULATOR #13](https://www.youtube.com/watch?v=lZhRQvXYnh8)
- [ROBOTIS OMY 공식 기술 문서](https://docs.robotis.com/docs/systems/omy/resources/technical_story/vla_lerobotnative/)
- [LeRobot ACTTemporalEnsembler 공식 구현](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py)
- [ACT/ALOHA 원 논문](https://www.roboticsproceedings.org/rss19/p016.pdf)

현재 근거는 공개 영상·문서와 synthetic/offline 검증이다. DAPIER 실물에서 jitter가
줄었다는 결과는 아직 없다.
