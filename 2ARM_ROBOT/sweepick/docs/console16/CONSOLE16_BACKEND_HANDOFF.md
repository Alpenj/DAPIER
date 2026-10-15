# 제한 실행 백엔드 추가 작업

현재 LEFT_START_TO_LIFT는 BACKEND_CHANGE_REQUIRED다. 아래 변경만 다음 백엔드 작업 범위다. 기존 writer, mapping, calibration, collision/STOP 조건을 유지한다. 웹 후보 구현은 native 코드를 변경하지 않았다.

1. revision/sweepick_pick01.py session()에 명시적 실행 경계 입력을 추가한다. 기존 stage를 재사용하고 held_after_lift 판정 직후(현재 1402–1410행), CARRY/handoff/회수 진입 전에 제한 종료 상태를 기록할 수 있어야 한다. CLOSE 거부/불명 시 LIFT 금지는 유지한다. fresh 6축/Goal/Torque/Status, 인증 원본·재검사 시각, Episode id와 물체 보유 근거를 함께 기록한다.
2. run_session()에 이 경계를 전달하고 Episode outcome 및 recorder lifecycle을 정의한다. full_chain=False 전환은 대안이 아니다. 현재 그 모드는 자동 lower/open/retreat/home을 포함한다.
3. RUN_APPROVED_SESSION.py main()의 run_session 호출을 승인된 실행 범위에 따라 분기하도록 하고 validate_launch()/source manifest/승인 결합도 함께 갱신한다. 기존 인자 생략 시 full_chain=True, RECEIVE_CONTACT 동작을 보존한다. 현재 웹 execution_scope 메타데이터만으로 native 실행 범위를 바꾸지 않는다.
4. LIFT 보유 종료 시 process를 끝내는 방식 또는 동일 Owner+resident Episode로 감독 대기하는 방식 중 한 가지의 계약을 먼저 확정한다. 전자는 ENDED_HOLDING_OPERATOR_REQUIRED 같은 명시적 결과, normal_return=false, torque_off=UNKNOWN/실측 값, operator_required 및 포트/reader 종료 근거가 필요하다. 후자는 Owner 유지·STOP·continuation 소유권·recorder 종료의 명시적 인터페이스가 필요하다. 자동 RELEASE/HOME/OFF는 추가하지 않는다.
5. 기존 full-chain Owner를 재사용한다면 sweepick_full_chain.SCOPES/build/Scoped에 경계를 반영하되 제한 범위가 오른 receive 준비를 선행조건으로 요구하지 않게 한다. 새 IK·writer는 만들지 않는다.
6. Controller.status()/records.run_record()가 위 제한 terminal을 FAULT/정상 HOME과 구분할 수 있도록 원본 terminal 필드를 전달한다. 현재 off_confirmed 기준을 완화하지 않는다. 서버 operation_contract()의 제한 supported/can_prepare는 실제 계약·Fake 회귀 통과 이후에만 활성화한다.

검증: 기존 Full-chain 불변, CLOSE REFUSED→LIFT 없음, LIFT 뒤 다음 stage 없음, STOP/예외 후 추가 Goal 없음, 단일 Owner/lease, 승인 범위 결합, HOLDING≠HOME/OFF/safe_shutdown, 원본 시각/SHA 보존. 무장치 통과 후 별도 현장 승인을 받아 fresh actual/3-view와 작업대 배치를 확인한다. 기존 승인이나 과거 관절 목표를 재사용하지 않는다.
