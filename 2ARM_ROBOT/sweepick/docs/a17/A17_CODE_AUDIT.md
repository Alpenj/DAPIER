# A-17 CLOSE 수정 소스 검수

판정: A16 CLOSE certificate 재검사 계약은 무장치 검증에서 유지됐다. 실물 성공 판정은 아니다. 이번 A17 작업은 제어 소스를 수정하지 않았다.

## 확인한 실행본

- A16_ALL_CHANGES.diff와 CURRENT_FIX_SHA256.json: `/home/robot/sweepick_local_robot_console/review/a16_20261015/`
- 실제 import: `/home/robot/sweepick_261013_receive_alignment_restore/revision/`
- `sweepick_move_a.py`: `9fb399b4042faeda37ddcaf7eff1ac8077270582144e014c4d38be4b45519240`
- `sweepick_pick01.py`: `5fe4f24021f91ca6af13aa171d9b8273f8ccb14abda0208955431e848696e143`

## 재검사와 거부

`sweepick_pick01.stage(close)`의 `closing_check()`는 같은 `check` closure, 같은 물체/desk/right-arm 조건으로 실제 시작 6축을 넣어 전체 closing sweep을 생성한다. 원래 인증은 sweep.CHECK.json에, 한 번의 재검사는 sweep_recheck.CHECK.json에 보존된다.

`sweepick_move_a.grip()`는 CLEAR, arm 5축 기존 abs(delta)<=1 tick, 기존 sweep 포함 조건을 검사한다. 처음부터 유효하면 callback을 호출하지 않는다. 불일치할 때만 callback을 한 번 호출하며 반복문/자동 retry는 없다. callback 후 actual을 다시 읽고 Status, torque ownership, 동일한 1-tick binding을 다시 검사한다. 이 허용값은 기존 계약이며 새로 추가하거나 늘리지 않았다.

재검사 중 arm이 추가 2 tick 이동하거나, CONTACT/UNKNOWN/잘못된 인증/예외/Status 오류/Torque 변경/Bus loss/interrupt가 발생하면 0 Goal/0 Torque write로 거부된다. STOP이 callback 직후 도착하는 경우도 첫 write 전에 차단된다. Bus 상태를 읽을 수 없으면 torque는 UNKNOWN이며 자동 OFF나 실패 Goal 재전송을 하지 않는다. `rechecked=true`는 callback 시도 완료 기록이고 성공 판정은 별도 `matched`와 최종 result를 함께 봐야 한다.

## 3501 actual과 최초 Goal

3501은 수동 개방으로 가능한 시작 actual로 사용했다. 고장이나 Calibration 오류의 증거로 취급하지 않는다. 3431은 저장된 software Calibration/현재 코드가 대조하는 configured register 상한이며, 물리적 개방 한계라는 주장은 하지 않는다.

원래 CLOSE 거부는 wrist certificate 2837, actual 2835의 -2 tick 차이였다. `certificate_binding / measured -2 tick / 기존 abs(delta)<=1 tick / 1 tick 초과 / PROVISIONAL / 같은 scene 재검사 후 다시 binding`이다. Gripper opening과 별개다.

기존 `cmd=min(max(fresh_actual, lo), hi)`에 의해 CASE_A 최초 Goal은 3344, CASE_B는 3431이다. Fake Bus에서 Readback도 각각 동일했고 이후 모든 Goal이 2001..3431에 있었다. 따라서 범위 밖 Goal 수정은 필요하지 않아 제어 코드를 변경하지 않았다. 전체 write 순서·ACK·첫 readback은 A17_FIRST_GOAL_FAKE_BUS.json에 있다. fake contact는 실물 파지 증거가 아니다.

`sweep_doc(3501,2018)`는 시작 3501을 포함해 1484개의 nominal sample을 만든다. 3501→3431 구간도 그 전체 sweep에 들어간다. `range_entry_checked=false`는 trajectory.command_start를 위한 **별도** range-entry segment가 없다는 뜻이다. EEPROM 확인이나 전체 closing collision 검사를 생략했다는 뜻이 아니다. object_check는 command_start가 있을 때만 별도 entry를 만든다.

첫 seed는 3501→3431의 명령 차이가 있다. 이후 Jaw의 v/a profile을 이 최초 seed 구간의 실물 속도 검증으로 확대 해석하지 않는다. 또한 기존 자료에 Goal write 자체가 torque를 켠 관측이 있으므로 명시적 Torque_Enable보다 앞선 seed도 실물 동작으로 취급해야 한다. 모델의 sweep 검사와 실물 속도/접촉 검증은 별개다.

## 무장치 검증과 한계

36 passed: 기존 A16 15건 + A17 8건 + HARD allowlist 9건 + postwrite order 4건. 기존 writer/Mapping/Calibration/STOP 소스 변경 없음. test_a17_first_goal.py는 gripper Torque OFF/ON 시작, 두 opening, 재검사 실패, 재검사 직후 STOP을 다룬다.

사용 인터프리터는 기존 로봇 venv Python 3.12다. 첫 pytest 시작은 자동 로딩된 ROS launch_testing의 lark 의존성 누락으로 수집 전에 실패했다. 이번 검사에 불필요한 외부 pytest plugin 자동 로딩만 명령 단위로 비활성화했으며 패키지·시스템 설정을 바꾸지 않았다.

미검증: 현재 하드웨어 actual/전원/Torque/EEPROM 실측, 현재 작업대 모델 대응, 실제 3-view 관측, 실제 CLOSE/LIFT. 신규 세션 및 장치 write 0회.
