# Gripper 초기 범위 진입·CLOSE 검수 — 2026-10-15

## 판단

**초기 위치 보정의 추종 오차가 CLOSE 무부하 기준값에 섞이는 소프트웨어 결함을 재현해 수정했다.** 3497·3501 ticks는 수동 개방으로 가능한 실제 위치이며 하드웨어 고장이나 Calibration 오류의 증거로 취급하지 않는다. 저장된 상한 3431 ticks와 물리적 개방 한계는 별개다.

실물 장치 접근, Motor Write, Torque Write는 모두 0회다. 아래 ACK·이동·접촉 결과는 Fake Bus 결과다. **최근 ID6 `0x20`의 원인은 `OVERLOAD_ROOT_CAUSE_UNKNOWN`이며 실물 CLOSE/LIFT 성공을 선언하지 않는다.**

## 비교 조건과 결과

| CLOSE 시작 actual | 최초 Goal | 초기 보정 | 수정 전 고정 Fake 물체 접촉 잔차 | 수정 후 |
|---:|---:|---:|---:|---:|
| 3344 | 3344 | 없음 | 26.310 | 28.266 |
| 3431 | 3431 | 없음 | 25.361 | 27.316 |
| 3497 | 3431 | 66 ticks | 66.430 | 27.316 |
| 3501 | 3431 | 70 ticks | 68.386 | 27.316 |

위 잔차는 동일한 Fake 정지 물체 3075 ticks, 추가 seed 지연 0초, Torque OFF 시작을 비교한 값이며 접촉력 측정값이 아니다. 4개 actual × Torque OFF/ON × seed 추종 지연 0/0.12/0.32/0.40초 × 물체 유무의 **64조건을 수정 전후 각각 실행**했다. 빈 그리퍼의 잘못된 접촉 기록은 8조건에서 0조건으로 줄었다. 수정 후 모든 Goal은 기존 설정 범위 안이었다. 별도로 첫 CLOSE 이동 자체가 지연되는 24조건도 확인했다. 범위 내 actual에서 seed가 현재 위치와 같아지는 경우도 포함한다.

PREGRASP → ALIGN → DESCEND_A → DESCEND → CLOSE는 기존 `pick.stage()`와 실행기를 Fake Bus에 연결해 4개 시작값 모두 확인했다. 팔 단계 동안 gripper actual은 유지되고 gripper Goal/Torque write는 없었다. CLOSE의 전체 sweep은 최초 actual부터 포함했다. 충돌 결과는 Fake CLEAR, 관측 입력은 fixture이므로 실물 충돌 검사·연속 CLI 운영 검증으로 해석하지 않는다.

Torque ON 상태로 새 PREGRASP를 시작하는 경우에는 4개 actual 모두 기존 `torque_state` 소유권 조건으로 write 0회 거부된다. 초기 범위 초과만의 거부가 아니다. 이미 팔 단계가 완료된 CLOSE의 gripper Torque OFF/ON 시작은 별도로 검증했다. 기존 소유권 조건을 해제하지 않았다.

## 확인한 원인과 실제 변경

기존 `grip()`은 최초 Goal을 3431로 보정한 직후 실제 위치가 따라왔는지 구분하지 않고 `Jaw.update()`를 호출했다. 실제 초기 추종 오차가 첫 free-stroke 표본의 중앙값에 들어가 이후 접촉 기준을 높였다. ACK 및 Goal 레지스터 일치는 actual 도달 확인과 다르다. 기존 소스에 올바른 fixture를 사용한 비교에서도 해당 회귀 조건이 실패했다.

실제 제어 변경은 `sweepick_move_a.grip()`의 초기 Goal 적용과 접촉 관측 연결에 한정했다.

1. 필요한 최초 유효 Goal을 기존 writer로 쓰고 Goal readback 및 Status를 확인한다. 이미 Torque ON인 경우에도 오래된 Goal을 그대로 두지 않는다. Torque OFF 시작의 기존 ON 순서는 유지한다.
2. 범위 진입 중에는 확인된 seed Goal을 유지하고 해당 표본을 접촉 기준에 넣지 않는다. 기존 gripper 무부하 잔차 기준으로 실제 추종을 확인한 뒤 기존 Torque 소유권과 arm 1-tick 인증 바인딩을 다시 확인한다. 여기서 scene 재검사를 추가 호출하지 않는다.
3. seed를 지나쳐 실제 위치가 더 닫힌 경우 한 번의 fresh-actual hold/readback으로 시작점을 맞춘다. 모든 Goal은 기존 범위 내이며 실패 Goal 재전송은 없다.
4. 첫 실제 CLOSE 추종이 확인될 때까지 접촉 표본을 사용하지 않는다. 기존 motion profile로 시작하고 명령만 앞서는 동안 추가 목표 누적을 멈춘다. 이후 기존 Jaw 접촉·프리로드 절차를 사용한다.
5. 초기 추종을 확인하지 못하면 `GRIP_UNAVAILABLE_HOLDING`과 미확인 상태를 기록한다. CONTACT_CANDIDATE나 성공으로 표시하지 않고 자동 재시도·복귀·OFF를 하지 않는다.
6. 범위 진입 중 STOP/오류의 fresh actual이 아직 설정 범위 밖이면 이를 Goal로 재전송하지 않는다. 기존 arm 실행기의 같은 범위 보존 원칙을 적용했다. 마지막 유효 Goal이 남을 수 있으므로 `motion_stop_confirmed=UNKNOWN`, 운영자 개입 필요를 기록한다. STOP 요청 자체를 바꾸거나 물리 정지 완료로 표시하지 않는다.

무진행 판단은 기존 progress profile의 측정 기반 1-tick 해상도·17-tick free-position lead·0.321초 진행 관찰을 재사용한다. 새로운 숫자나 물리 HARD gate를 추가하지 않았다. 새 구간의 추종 미확인은 작업 결과 UNAVAILABLE이며 servo fault/bus loss/STOP과 구분한다. 기존 Jaw·프리로드, Calibration·EEPROM, motion profile, 1-tick 바인딩과 기존 오류 중단 조건은 변경하지 않았다.

기존 `test_sweepick_grip.Jaws` fixture는 새 Goal write가 있어야만 움직였다. 이미 기록된 Goal이 Torque ON 후에도 적용되도록 fixture의 기존 추종 모델을 연결했다. 물체·지연·부하 모델과 기존 assertion은 유지했다. 새 writer/Owner/카메라 reader는 없다.

## ID6 Overload: 사실과 가설

비교 원본은 `20261015_112218_f0d866/session/close/run1/move.json`이며 SHA256은 `08adbfb6c61f34efffe255ed1471a1055b709fd23616a2936d30faac12f3e36b`다. 원본을 변경하지 않았다.

- CLOSE 시작 actual은 3501, gripper Torque 0, Status 0, Goal 레지스터 0이었다. 3497은 이 실행 시작값으로 대체하지 않았다.
- 최초 정상 표본에는 Goal 레지스터 3431, actual 3501, Torque 1, Moving 1, Status 0이 기록돼 있다. 초기 잔차는 약 70→70→68→61→49→38→31→27 ticks로 줄었다. 당시 개별 최초 seed/Torque wire ACK 시각은 기록에 없어 확정하지 않는다.
- 접촉 후보 시각 `1792031026.2508862`: actual 3004, 내부 Goal 2934.253, 잔차 69.747, load raw 300, current raw 44였다. CLOSE 최종 읽기는 actual 3000, Goal 2934, Torque 1, Status 0이었다.
- `CURRENT_FEEDBACK.json` 최초 오류 시각 `1792031028.225814`에는 ID6 `0x20 Overload`가 기록됐다. 접촉 후보 약 1.975초 뒤이며 LIFT 완료 기록은 없다. 이후 handshake의 ID6 무응답을 영구 장치 고장으로 해석하지 않는다.

초기 기준값 오염이 더 큰 접촉 잔차와 부하에 기여했을 가능성은 있다. 그러나 **Overload를 일으킨 원인으로는 확정하지 않는다.** 10월 14일 비교 실행은 actual 3344, 접촉 load/current raw 116/7이지만 배치·접촉 조건이 같은 통제 실험이 아니다. 힘 단위가 검증되지 않은 raw load/current, 전원·온도·실제 패드 접촉·오류 직전 연속 상태의 부족 때문에 원인 연결은 남아 있다.

## 검증 및 보존

- 신규 회귀 111개 + 기존 A16/A17·allowlist·명령 순서 36개 + 기존 grip 7개 = **154 PASS**.
- 기존 trim 시험 1개는 수정 전 소스에서도 같은 `KeyError: provisional`이 재현됐다. 이번 초기 CLOSE 경로와 별개이며 숨기거나 모든 native 시험이 통과했다고 표시하지 않는다.
- 초기 ACK 유실·readback 유실·Status 오류, arm 바인딩 2-tick 변경, Torque/Goal 변경, 1-tick 흔들림, overshoot/rebound, STOP 및 이미 닫힌 빈 그리퍼를 포함한다.
- 무한 대기, 초기 구간의 CONTACT_CANDIDATE, 불명 적용 상태 이후 CLOSE Goal 진행, 범위 밖 복구 Goal이 발생하지 않는지 확인했다.
- 기존 `sweepick_pick01.py`, `sweepick_grasp.py`, Jaw/progress profile, allowlist와 원본 기록 SHA를 보존했다. 당시 A17 Source SHA는 역사 기록으로 유지한다.

실행 명령·결과는 `GRIPPER_INITIAL_RANGE_TEST_REPORT.json`, 64조건의 최초 Goal/ACK/readback·결과는 `GRIPPER_INITIAL_RANGE_FAKE_BUS.json`, 제어 diff는 `GRIPPER_INITIAL_RANGE_FIX.diff`, 이번 원본 SHA는 `SOURCE_SHA256.json`에 있다. 공개 사본 SHA와 실제 원본 SHA의 차이는 게시본 `SOURCE_INDEX.json`에서 구분한다.

## 남은 한계

최초 3497/3501→3431의 66/70-tick 이동 자체는 유효 Goal 범위를 지키면서 남는다. 그 이동의 실제 힘·과도 부하와 이번 수정 후 실제 접촉은 측정하지 않았다. 실제 START/CLOSE/LIFT를 실행하지 않았고, 실물 성공·Overload 해소·물체 지지·정상 회수·Torque OFF·safe shutdown을 확인한 것으로 보고하지 않는다. 후속 실물 시험은 기존 현장 승인 및 Owner 계약에 따른 별도 작업이다.
