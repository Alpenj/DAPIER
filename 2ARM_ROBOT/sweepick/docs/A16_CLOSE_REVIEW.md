# A-16 CLOSE REFUSED 검토

비교 기준은 `20261014_155824_d4017e`, 분석 대상은 `20261015_092942_a4628c`다. 전날 성공한 단계와 최신 실패를 전체 인계 성공률로 합치지 않는다.

확인한 직접 원인은 `certificate_binding`이었다. 검사 때 wrist_flex는 2837 tick, CLOSE 시작 actual은 2835 tick으로 2 tick 달랐다. 기존 허용 1 tick을 1 tick 초과했다. 원본 상위 종료 3, 하위 CLOSE 종료 2/REFUSED를 구분한다. 해당 CLOSE의 Goal Position 및 Torque Enable write는 모두 0회였다. after_close를 완료 단계로 해석하지 않는다.

3344와 3501이라는 gripper 시작 차이는 이 refusal의 직접 원인이 아니다. 3501은 수동 개방으로 가능한 actual이며 설정 상한 3431이 물리 개방 한계라는 주장은 하지 않는다.

수정은 `sweepick_move_a.grip(start_recheck=...)`와 `sweepick_pick01.stage(close)`의 same-scene callback 연결이다. 불일치할 때 한 번 재검사하고, 이후 actual에 같은 1-tick 조건을 재적용한다. collision 판정·Calibration·writer·STOP 조건은 완화하지 않았다. 검사 실패나 재검사 중 오류는 write 0회로 끝난다. Bus 상태를 알 수 없으면 Torque UNKNOWN을 유지한다.

현재 코드와 회귀시험은 [코드 지도](CODE_MAP.md), 추가 검수는 [A-17 보고서](a17/A17_CODE_AUDIT.md)에 연결했다. 세션 원본·장치 설정은 로컬에 보존하며 공개 폴더에 포함하지 않는다. 수정 이후 실물 재현은 미실행이다.
