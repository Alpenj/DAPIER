# Console-16 제한 실물 시험 계약

상태: **BACKEND_CHANGE_REQUIRED**. 웹 준비·검수와 무장치 시험만 수행한다. 실물 START, 모터/Torque/설정 write, 새 카메라 reader, Owner 생성은 수행하지 않았다.

| 실행 범위 | 백엔드 지원 | 웹 동작 |
|---|---|---|
| FULL_CHAIN_RECEIVE_CONTACT | 기존 launcher 경로 유지 | 기존 fresh 계획·소스 manifest·현장 승인·단일 Owner·STOP 계약으로 준비. 현재 FAULT/복구 미확인 상태에서는 준비 불가 |
| LEFT_START_TO_LIFT | BACKEND_CHANGE_REQUIRED | 기본 선택 후보. 준비/START 비활성, API 직접 요청과 저장 job 위조도 거부 |

제한 범위는 기존 현재 관측·START/pregrasp·접근·정렬·DESCEND·CLOSE·LIFT까지다. CARRY, RIGHT PREGRASP/INSERT/CLOSE, 하중 이전, LEFT RELEASE, ACT, 자동 LOWER/OPEN/HOME/OFF는 제외한다. 아직 이 경계를 실행하는 native 종료 계약이 없다. UI 옵션을 바꾸어도 백엔드 지원으로 승격되지 않는다.

FULL_CHAIN_RECEIVE_CONTACT는 프로젝트의 전체 인계 성공을 뜻하지 않는다. 기존 launcher의 RECEIVE_CONTACT 구현을 사용하되 현재 승인 범위는 LEFT PICK/LIFT/CARRY → RIGHT PREGRASP → 비접촉 관측 → 조건이 충족된 기존 put-back/home/OFF다. 추가 관측 이동은 기존 별도 승인 계약을 유지한다.

GET /api/control/scopes는 기존 control 상태, 기존 포트 점유 조회, 디스크상의 실행 소스 SHA만 읽는다. device_access=false이며 포트를 열지 않는다. 현재 관측·Calibration 실측은 NOT_CHECKED다. 저장 영상·과거 HELD_CONFIRMED를 현재 실측으로 사용하지 않는다. FULL backend 지원 여부와 현재 can_prepare, 전체 미션 준비 여부를 별도로 표시한다.

실행 중/종료 후 패널은 기존 owner가 제공한 control 및 snapshot을 사용한다. snapshot에는 run id, MODE, 시각을 표시하고 저장 replay를 LIVE로 승격하지 않는다. 원본 controller 상태·단계, 마지막 명령·실제 feedback·ACK/readback, 카메라 freshness, STOP, 최초 오류와 정상 회수·Torque OFF·safe_shutdown의 기존 구분을 재사용한다. 미측정은 UNKNOWN이다.

LIFT HOLDING은 물체 보유의 실시간 확정·정상 HOME·RELEASE·safe_shutdown을 뜻하지 않는다. STOP 요청도 회수 완료가 아니다. 현재 지원되지 않는 제한 경로에는 성공한 모터 시험 결과가 없으며 FakeRunner 결과는 실물 성공이 아니다. 실패 후 기존 수동 복구 기록과 연결하되 추가 자동 명령을 만들지 않는다.

기존 Inspection Panel 및 소형 ZIP Export를 재사용한다. 원본 세션·과거 Source SHA는 변경하지 않고 파생 검수 문서에 출처·시각·SHA를 남긴다.
