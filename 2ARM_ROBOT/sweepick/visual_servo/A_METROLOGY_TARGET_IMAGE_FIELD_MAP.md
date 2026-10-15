# A metrology → B Target Image 필드 대응

A 기준은 `0cb3ad4141ea2dcab970daed30235ed0f6e84567`, B 계산기 기준은 `29cdf06a457274553cbcb85f76231ff43fc2fa3c`이다. 나는 A의 고정 소스에서 저장 필드를 대조하고, A 모듈을 실행하지 않는 파일 어댑터 [metrology_bridge.py](metrology_bridge.py)를 추가했다. 실제 A T/F/V 실행 기록은 확보하지 못했다. 회귀의 양성 입력은 명시적인 `TEST_FIXTURE`이며 실측 성공을 뜻하지 않는다.

| A 원본 | B 입력/어댑터 결과 | 확인 및 한계 |
|---|---|---|
| `RIGHT_X_METROLOGY.json`: schema, scope, source_kind, endpoints, moves, config, status | `bridge_metrology`의 session root, 외부 run_id, 원본별 SHA manifest | `sweepick.right-x-metrology.v1` / `RIGHT_X_FINITE_METROLOGY`; T/F/V 순서와 `verify_from=F`, `ENDPOINTS_RECORDED`를 구분한다. 고정 소스 커밋은 스키마 근거이며 실제 실행 코드의 신원은 `UNKNOWN`이다. |
| Endpoint `decision_record` → 해당 폴더 `OBSERVATION.json` | `snapshot.record` 원본 bytes/SHA; observation_id는 해당 폴더명 | `stage=RIGHT_X_T/F/V`, source_kind를 대조한다. 경로는 지정 session 안에 있어야 하며 모든 읽기에 외부 pinned SHA가 필요하다. SHA는 내용 식별이며 운영자 확인이나 진본 인증을 대신하지 않는다. |
| Endpoint `actual.ticks` 양팔 6축, `actual.q12` | 원본 `actual_feedback.ticks`, `observation.present_ticks`, `q12`, `observation.q12` → 기존 `snapshot_observation` | 양팔 Present_Position, q12, native freshness, usable/snapshot 상태와 버스 identity를 교차 확인한다. q12는 A가 기록한 rad이며 이 어댑터가 tick→rad 보정이나 물리 FK를 인증하지 않는다. |
| Endpoint `actual.goal/torque/status`, bus_seq, t_bus_monotonic, cycle_start | `actual`에 원본 보존; native feedback_stamp/seq도 대조 | 목표와 실측을 섞지 않는다. 서로 다른 시계의 숫자를 빼서 노출 시각/지연을 만들지 않는다. |
| Endpoint `opening.right_present_tick/right_goal_tick` 및 왼쪽 값 | `opening`, `measured_opening_tick`; B current/reference context는 각각 해당 프레임의 actual opening | 실제 gripper raw tick을 대조한다. jaw gap mm는 `null/UNKNOWN`; 특정 물체용 opening 범위나 새로운 motion gate는 없다. |
| Endpoint `cameras`, OBS `camera_frames`, `files.*.rgb` | 각 카메라 metadata와 RGB 원본 SHA; Right RGB bytes/name → B snapshot | right_wrist의 n, fresh, reader_valid, recv_monotonic_s를 기존 B가 확인한다. top/left RGB는 context 출처로 보존하며 Right 계산을 위한 가짜 Pad/현재 관측으로 바꾸지 않는다. Receipt는 exposure가 아니며 exposure는 `UNKNOWN`이다. |
| Endpoint F/V `executor`, move `result/writes/folder` → `folder/run/move.json` | `stage_motion`의 result/writes/nominal target/last Present_Position/row t | side, result, writes와 명령 target ARM5를 교차 확인한다. Stage 마지막 표본과 이후 Endpoint read는 별도 표본이다. tick 차이는 진단으로만 기록하며 일치 cap을 두지 않는다. Stage row clock의 정확한 기준은 `UNKNOWN`; T 이전 approach Stage를 추정하지 않는다. |
| 이미 목표 tick인 A의 zero-command observation-view: `observation_view.settling_evidence`, `STATIC_VIEW_READS.json` | `stage_motion.status=STATIC_READ_SOURCE_BOUND`, 마지막 native actual read와 nominal target | `move.json`이 생기지 않는 정상 A 경로도 읽는다. 원본 reads와 endpoint/move view 상태 및 target을 대조한다. 누락된 writes는 `null` 그대로 두며 0을 만들어 넣지 않는다. 실제 엔코더 변화가 0이면 새 이동 응답 기울기를 만들지 않는다. |
| Endpoint `command.commanded_tool_displacement_m/target_ticks` 및 command FK | `command`; move candidate/forward target, Stage trajectory target과 대조 | 명령 metre, rounded goal ticks, 실측 ticks를 구분한다. 명령량을 실제 물리 X(mm)로 대체하지 않는다. |
| `right_tool_pose_fk`, `fk_role`, `actual_fk_from_previous/actual_fk_from_T` | `candidate_right_tool_pose_fk`, `candidate_fk_*`, candidate rotation/off-axis 기록 | actual ticks에 후보 매핑 FK를 적용한 값이다. `verified=false`의 물리적 불확실성은 유지하며 `MEASURED_PHYSICAL_X_RESPONSE`를 생성하지 않는다. |
| A에는 FIELD target의 운영자 물리 확인이 없음 | 선택적 외부 `reference`만 기존 `TargetImageServoProposal`로 전달 | T나 기존 REAL 사진에서 reference를 자동 생성하지 않는다. `current_evidence_by_role`은 각 T/F/V의 별도 원본 binding을 가져야 한다. Reference 확인 내용을 current에 복사하지 않는다. |

출처 누락·불일치는 `UNKNOWN` 및 구체적 missing_evidence로 남긴다. 원본 버스/카메라 freshness 필드의 저장 당시 의미를 보존하며, 아카이브가 지금 실물 write에 fresh하다는 뜻으로 사용하지 않는다. 어댑터는 A Native, Console, Motor Writer, calibration, 기존 B/PAD/LEFT 구현을 변경하지 않는다.

T→F는 fit 후보이며 F→V는 fit에서 제외한 응답 관측이다. V의 Right capture identity가 T/F와 같으면 독립 검증으로 인정하지 않는다. 실제 Right ARM5 tick 변화 벡터와 실제 q12 변화, 영상 centroid Δu를 별도로 기록한다. 명령량과 후보 FK 변화는 나란히 보존한다.

별도 엔코더 진단의 좌표는 T→F의 Right ARM5 raw tick 벡터를 정규화한 방향이다. ARM 순서는 `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll`이며, 좌표 단위는 **Euclidean raw ARM5 tick-norm**, 기울기는 **pixel / encoder tick-norm**이다. F→V를 같은 방향에 투영한 값, 그 밖의 성분과 영상 응답 부호를 보고한다. V에 fit 방향 밖의 성분이 있으면 `OFF_BASIS_CONFOUNDED_DIAGNOSTIC_ONLY`로 표시한다. 이 값은 tool X나 px/mm Jacobian이 아니며 REAL Jacobian 입력으로 사용할 수 없다. `validated=false`, `observable_for_control=false`를 유지한다.

가림/클리핑, object·feature·mount·camera mode/shape·opening context 변화, 왼팔 실제 변화, 후보 FK 회전, apparent component area 변화, frame/feedback 순서 불확실성이 있으면 1D 진단의 혼입 요인으로 표시한다. 면적 변화만으로 scale과 occlusion의 원인을 확정하지 않는다. 회전 0이나 일정한 면적도 물리적인 순수 X 이동을 보증하지 않는다. 이 진단 분류는 actuator HARD gate가 아니다.

현재 정지 상태 detector jitter의 독립 측정 근거가 없다. 움직이는 T/F/V 세 장으로 noise bound를 만들지 않으므로, 작은 Δu를 포함해 제어 관측 가능성은 `UNRESOLVED`이다. `detector_jitter_px=null`, `UNKNOWN_NO_STATIONARY_BOUND`를 반환한다. 새 noise/dwell/current/travel 임계값을 만들지 않는다.

Reference의 물리 확인, 각 current의 실제 출처와 context, 검증된 물리 X 응답·fit 독립 관측을 가진 REAL Jacobian은 각각 별도 근거다. 현재 확보한 REAL 자료에는 검증된 목표가 없어 Target Image Reference를 생성하지 않았고, 계산 Correction은 `null`, `REAL_J_VALIDATED=false`이다. 엔코더 진단으로 이 빈 근거를 채우지 않는다. 어댑터와 계산 결과는 실행·motor write 권한을 갖지 않는다.

반환값의 최상위 `Correction/REAL_J_VALIDATED`는 어댑터가 생성한 명령·물리 J가 없음을 나타낸다. 각 Endpoint의 `B_output`은 별도 외부 reference/current evidence/J를 받은 기존 계산기의 실제 반환값을 보존한다. 현재 회귀 입력에서는 이 두 위치 모두 Correction이 `null`이다.

검사는 [test_metrology_bridge.py](test_metrology_bridge.py), 재현 실행기는 [run_metrology_bridge.py](run_metrology_bridge.py), 결과는 [A_METROLOGY_TARGET_IMAGE_REGRESSION.json](reports/A_METROLOGY_TARGET_IMAGE_REGRESSION.json)에 둔다. 기존 B20의 57개 검사는 원래 실행기로 별도 확인하고 원본 receipt를 새 회귀에 연결한다. 새 회귀 실행기는 해당 57개를 중복 실행하지 않는다.

```bash
python -B visual_servo/run_metrology_bridge.py \
  --baseline-report /private/b20/TARGET_IMAGE_CONSUMER_REGRESSION.json \
  --output /private/bridge-regression
```

위 명령은 sweepick 디렉터리에서 기존 NumPy/OpenCV가 설치된 Python 환경으로 실행한다. 실제 장치·학습·활성 배포를 실행하지 않으며 B21을 시작하지 않는다.
