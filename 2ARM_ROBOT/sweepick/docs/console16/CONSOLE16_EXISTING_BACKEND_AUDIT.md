# 기존 백엔드 경로 감사

분석은 저장 소스·로그와 무장치 시험에 한정했다. 아래 경로는 native 디렉터리를 기준으로 하며 말미 SOURCE_SHA256.json이 감사 시점 파일을 고정한다.

| 구간 | 원본 함수·근거 | 계약 |
|---|---|---|
| 웹 START | console/control.py Controller.prepare/start | source snapshot+manifest, 이번 현장 승인, 단일 tmux/포트 owner 확인 후 기존 launcher만 호출 |
| launcher | RUN_APPROVED_SESSION.py main 314–315행 | p1.run_session(execute=True,record=True,full_chain=True,scope='RECEIVE_CONTACT') 고정 |
| scope | revision/sweepick_full_chain.py SCOPES/build 3072행 이후 | FULL_TASK/SUPPORT_OBSERVATION/RECEIVE_APPROACH/RECEIVE_CONTACT만 지원 |
| 세션 | revision/sweepick_pick01.py session 1251행, run_session 1489행 | LIFT 뒤 handoff와 lower/open/retreat/home을 포함. full_chain=False도 put-back 경로 |
| stage CLI | sweepick_pick01.py stage 958행, stage_request 716행 | lift 요청은 있으나 상태·source hash·allowed·최근 관측을 각 stage마다 검사. CLI 반복은 연속 Owner 계약과 다름 |
| 충돌/현재값 | stage 988–1001, 1066–1083행 | 관측 블록 scene cache, start_recheck/trim_check를 기존 native executor로 전달. A16/A17의 동일 scene 1회 재검사·기존 1tick binding 유지 |
| 관측 유효성 | look 630–651행, full-chain Owner.observation 1073–1102행 | fresh frame 및 현재 관절 조회, 정지 확인. 과거 STATE의 성공을 현재 관측으로 대체하지 않음 |
| 포트 lease | pick01.recorded_stage 1204–1223행, Episode.lease 98–118행 | recorder background 포트 양도→stage writer→polling 복귀. 새 writer/reader 없음 |
| 종료 | run_session finally 1511–1524행, Episode.close/release | 세션 종료 시 recorder/camera reader 닫음. stage CLI 하나의 종료와 resident Episode/Owner 연속 유지가 같지 않음 |
| STOP/예외 | full-chain Owner.segment 1707행, on_stop 2873행, 예외 924행 | STOP/상태 불명 뒤 추가 segment/정상 회수 명령 금지. holding/unknown 보존, 임의 OFF 없음 |
| 정상 회수 | pick01.session 1462–1486행, Owner recovery 2547행 | fresh actual/frame·현재 hold Goal·Torque/Status 및 보유 근거 조건하에 기존 경로. 조건 소실 시 운영자 개입 |

Torque Enable write와 포트 닫기는 별개다. 현재 after-lift 조기 process exit는 정상 HOME/회수/OFF를 증명하지 못한다. Controller.off_confirmed는 양팔 마지막 RETURNED_RELEASED와 실행기 torque_released 및 6축 실제 종료 레지스터를 요구한다. 단순 process 종료는 FAULT가 될 수 있다. 따라서 제한 시험을 stage CLI 여러 번 실행하거나 full_chain=False로 우회하지 않는다.

10월 14일 관측 실패: events.jsonl 303행 observation_return_check, host 1791961753.7144125; left frame n=6266, checked_monotonic=28695.791521693, recv_monotonic=28695.523576045. 경과 267.945648ms, reader freshness 64.082262ms. 115회 연속 PASS 뒤 FAIL이며 직전 8회도 PASS다. 기존 메시지의 support/bus 표현은 보존하되 실제 분류는 CAMERA_STALE다. register_identity={}이며 그 검사에 대응하는 bus 시각/sequence가 없어 BUS_ERROR로 확정하지 않는다. detector 미실행이므로 물체 이탈 UNKNOWN. 실패 전 3초 export 구간 관절 샘플 0개, 이후 2개다. reader_error=null은 freshness 통과 증거가 아니다.

기존 Console-15 Inspection Panel을 확장해 위 차이만 표시한다. 승인 표시·완료 단계 보존, A16/A17 분석 문서, 별도 OFF와 정상 회수 구분은 재작성하지 않았다.
