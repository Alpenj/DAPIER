# A-17 구형 실패 9건 영향 분류

`review/a16_20261015/BASELINE_RELATED_TESTS.txt`와 `RELATED_TESTS.txt`의 9건은 A16 전후 동일하다(33 pass/9 fail → 44 pass/9 fail). 단순히 테스트 이름의 CLOSE/LIFT 포함 여부로 무관하다고 처리하지 않고, 실제 중단 위치·주입 경로·현재 live callback을 대조했다. 장치와 제어 소스를 수정하지 않았고 전체 구형 suite를 다시 반복하지 않았다.

| # | 시험 | 실제 실패 원인 | A17 실제 경로에 대한 판단 |
|---|---|---|---|
| 1 | `test_sweepick_grip::test_at_model_level_a_trim_goes_through_the_same_validation_as_its_stage` | model trim에 fresh `other_arm_read` 미주입. validation 전 `check_error`로 끝나 `provisional` 키가 없는데 구형 assertion이 읽음 | START/LIFT 공통 run 경로와 관련은 있음. 실제 p1.stage는 current_right_ticks reader, full-chain Owner는 read_other_actual을 주입함. 명시적 fake right callback으로 CONTACT 거부/CLEAR 적용 확인. live 결함 증거 없음 |
| 2 | `test_sweepick_pick02_actual::test_the_whole_session_with_a_flickering_readback_and_a_second_stop_from_the_measured_error` | injected left bus에 explicit right reader가 없어 descend_a BUS_LOST. callback 추가 후에도 매 read ±1 tick을 계속 토글하는 persistent flicker가 재검사 work budget을 소모 | 기존 실제 경로는 fresh right callback을 제공함. 지속 토글 fixture를 일회 transient로 간주한 기대와 불일치. A16 CLOSE 회귀 아님 |
| 3 | `test_a_real_lateral_error_is_removed_by_the_visual_loop_and_the_block_stays_where_it_was_observed` | 첫 error -2.781789mm는 당시 pad 기반 allowance 2.996165mm 안이므로 첫 row에는 correction 없음. 후속 01a에서 -3.194982mm > 3.129788mm가 되어 +3.9937mm correction | 현재 관측 기반 허용조건과 구형 첫-row assertion 불일치. 정상 trajectory를 막는 새 gate 또는 CLOSE 결함 아님 |
| 4 | `test_a_block_disturbed_by_the_last_descent_is_not_closed_on_and_the_arm_backs_up_despite_the_flicker[turned]` | #2와 동일한 explicit other_arm_read 누락으로 descend_a에서 BUS_LOST | CLOSE 이전 fixture 중단. 실제 reader 주입 경로와 다름 |
| 5 | 같은 시험 `[shifted]` | #4와 동일 | CLOSE 이전 fixture 중단 |
| 6 | 같은 시험 `[not readable]` | #4와 동일 | CLOSE 이전 fixture 중단 |
| 7 | `test_a_block_turned_at_the_first_stop_ends_the_descent_there` | slope -1°, reference 4.7°, 차이 5.7°를 과거 yaw HARD 의미로 기대. 현재 camera가 wrist와 함께 회전하는 image slope는 DIAGNOSTIC_ONLY이며 visibility/reference를 판정 | 현재 명시된 yaw 정책과 구형 assertion drift. 해당 slope만으로 동작을 거부하도록 되돌리지 않음 |
| 8 | `test_a_request_refused_before_any_write_is_kept_as_a_refusal_and_does_not_close_the_way_home` | intended CONTACT checker 전에 explicit other_arm_read 누락으로 BUS_LOST | in-memory fake right callback을 주입하면 intended REFUSED 및 Goal/Torque 0회에 도달. live REFUSED 소실의 증거 아님 |
| 9 | `test_the_whole_session_on_an_arm_with_the_real_servo_behaviour` | fixture가 가리키는 run10 SCENE 파일 부재로 동작 전 FileNotFoundError | 환경/과거 fixture 누락. 실제 장치 동작이나 A16 callback의 실패 증거 아님 |

## 코드 근거

- `sweepick_move_a.py:289` prewrite 변경 후 other-arm 재조회, `:611` trim의 fresh-other-arm 계약 및 `:620` check_error 기록
- `sweepick_pick01.py:1069` real current_right_ticks / injected 경로의 missing_injected_other_actual. injected 경로는 실물 장치를 대신 열지 않도록 명시적으로 거부한다.
- `sweepick_full_chain.py:1033` read_other_actual, `:1717` 기존 run에 callback 전달
- `test_sweepick_pick02_actual.py:18` persistent flicker, `:74` left fake만 주입하는 cell
- `sweepick_pick01.py:851` 실제 pad room/frame uncertainty allowance, `:1341` 범위 안이면 ALIGNED
- `sweepick_pick01.py:822` current_wrist_alignment의 DIAGNOSTIC_ONLY, `:890` cross-height image drift 진단

상기 callback 보완은 조사 중 메모리 안의 Fake Bus에만 적용했다. 운영 파일·기존 test 원본을 통과시키기 위해 바꾸지 않았다. 공용 run_kw에 무조건 other_arm_read를 넣으면 grip에는 해당 인자가 없어 TypeError가 되므로 그런 전역 수정을 하지 않는다.

## 결론과 미해결 범위

9건을 모두 실제 경로와 무관한 것으로 뭉뚱그리지 않는다. #1/#2/#4/#5/#6/#8은 현재의 shared START/LIFT executor와 접점은 있지만, 저장된 실패는 fake reader 주입 계약 위반이다. 실제 source의 reader 공급 경로를 확인했으며 이번 분석에서 수정해야 할 live 결함은 발견되지 않았다. #3/#7은 assertion 정책 drift, #9는 fixture 파일 누락이다.

구형 9건은 여전히 기존 suite에서 실패한 기록으로 남는다. 이것을 전체 suite green 또는 실물 안정성 확인으로 보고하지 않는다. A17에 직접 필요한 36개 무장치 시험은 통과했다. 실제 관측·bus·모델 대응은 별도 승인 후 현장에서만 검증할 수 있다.
