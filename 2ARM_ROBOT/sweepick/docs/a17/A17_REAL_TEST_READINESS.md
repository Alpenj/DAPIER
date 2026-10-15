# A-17 제한 실물 시험 준비 상태

**무장치 CLOSE 검수 PASS / 현재 웹 START로 START→CLOSE→LIFT 한정 실행은 NOT_READY. 실물 실행 승인 대기.**

현재 RUN_APPROVED_SESSION.py는 full_chain=True, scope=RECEIVE_CONTACT를 고정한다. 웹 START도 이 launcher를 호출한다. LIFT에서 종료하는 옵션이 없으므로 이 버튼으로 A17 시험을 시작하지 않는다. 일반 session은 lower/open/retreat/home도 이어서 실행하므로 왼 RELEASE 금지 범위를 충족하지 않는다. 임의의 새 controller·launcher·자동 STOP 타이머는 만들지 않았다.

기존 단일-stage CLI(`sweepick_pick01.py stage`)는 한 stage 뒤 HOLDING으로 종료하고 bus를 disable_torque=False로 해제한다. 이 기존 기능으로 fresh observe/plan 및 pregrasp·align·descend의 기존 관측 게이트를 거쳐 close, lift만 호출하는 감독 운영은 가능하다. 그러나 이것은 현재 웹 owner가 연속 실행하는 통합 세션과 같은 운영 방식이 아니다. 기존 단일-stage 방식 채택과 그 기록/종료 계약을 먼저 현장에서 합의해야 하며, 이 문서는 즉시 실행 가능한 START 승인이 아니다.

## 승인 전에 확정할 묶음

1. 기존 owner가 종료됐는지, 이전 정상 회수·전원·Torque가 실제로 어떤 상태인지 확인한다. 10/15 실패 run의 자세나 물체 보유를 가정하지 않는다. 10/14 별도 OFF 기록을 현재 상태로 재사용하지 않는다.
2. 현재 actual과 기존 정상 설정의 OS30A RGB-D/좌우 wrist RGB를 사용한다. 작업대·블록 배치와 기존 collision 모델의 대응, 팔·블록 지지 및 케이블 여유를 현장에서 확인한다. 과거 인증/목표는 live 입력으로 사용하지 않는다.
3. 동작 범위는 기존 START의 준비·관측·접근/descend → CLOSE → 확인 → LIFT → 보유 확인까지다. 오른팔 접근/INSERT/CLOSE, 하중 이전, 왼 RELEASE, ACT, 자동 전체 재시작은 제외한다.
4. LIFT 후 HOLDING은 Torque ON일 수 있다. 정상 HOME/OFF가 아니다. 물체를 파지한 상태에서 시험을 끝내려면 감독자가 물체와 팔을 지지하는 종료 방식을 함께 승인해야 한다. 자동 RELEASE/HOME/전체 OFF는 추가하지 않는다.
5. 최초 무응답/Status 오류/적용 불명확 시 후속 Goal 및 LIFT를 중단한다. 실패 Goal을 재전송하지 않는다. 기존 제한 사후 조회로 확인할 수 있는 상태만 기록한다. 지원 상태를 모르면 자동 회수하지 않으며, 운영자 지지·전원 차단과 필요 시 **사전에 승인한** 제한 OFF만 따른다. 확인하지 못한 torque는 UNKNOWN이다.

## 현장 수집할 원본 근거

- 원래 sweep.CHECK.json, 재검사 발생 시 sweep_recheck.CHECK.json, CLEAR/CONTACT와 같은 scene 조건
- grip move.json certificate_binding.initial.actual와 재검사 후 actual, 각 feedback 시각; 실제 6축 전후 값
- 재검사 호출 wall/monotonic 시작·끝의 실제 소요시간. 현재 certificate_binding은 정확한 callback duration을 따로 기록하지 않으므로 그 값은 현장 관찰 기록이 없으면 UNKNOWN으로 남긴다.
- 최초 Goal 및 그 readback/ACK, gripper contact candidate, 그 시각의 실제 wrist/OS30A 영상
- LIFT result/actual/그립 feedback와 held_after_lift 영상 근거
- 종료 시 actual/Torque/Status/Moving, 정상 회수 여부와 별도 운영자 개입

CLOSE REFUSED는 원본 refused/stopped_by와 write 0회 여부를 보존하며 성공 후보로 분류하지 않는다. 영상 누락은 물체 이탈 확정으로 바꾸지 않는다.

현재 실물 장치는 열지 않았다. 따라서 현장 상태·3-view·작업대 대응은 NOT_CHECKED이며 실물 READY를 선언하지 않는다. 실물 실행을 하지 않았으므로 A17_REAL_RUN_REVIEW.json 및 실물 증거 ZIP은 생성하지 않았다.
