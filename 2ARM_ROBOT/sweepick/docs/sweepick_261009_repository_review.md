# sweepick 구조·명명 정리 검토 — 2026-10-09

## 기준 소스와 적용 범위

GitHub 기준은 `Alpenj/DAPIER`의 `main@0238bbbcfa2346ce3c97e126e6e26f0467566c37`이다. 공개 main에는 최신 native 현장 코드가 없었다. 별도 clone의 `chore/sweepick-repository-layout`에서 현재 로컬 통합 담당자가 작업하지 않는다고 확인한 뒤 소스를 읽기 전용으로 고정했다. 원래 로컬 repo는 `feat/2arm-robot-phase0@5913b034fb16f089331add93789881e63b68489c`와 dirty/untracked 파일을 그대로 보존한다. 소스 revision은 개별 파일 SHA로 기록하며 원래 repo HEAD를 현장 파일의 revision으로 대신 쓰지 않는다.

work6에서는 `workstation6`/`dgu`와 독립 RECEIVE 경로만 읽었다. 원래 DAPIER는 `.git` 없는 snapshot이었다. git init·파일 이동·원격 원본 수정은 없었다. 로컬 active source와 새 작업본의 쓰기 범위는 분리했다.

| RECEIVE 소스 | 현재 로컬 고정본과 work6 | 채택 |
|---|---|---|
| `concrete_bindings` | SHA 다름 | 현재 로컬 entrypoint가 실제 사용하는 고정본 |
| `full_episode` | SHA 다름 | 현재 로컬 entrypoint가 실제 사용하는 고정본 |
| `receive_entry` | SHA 일치 | 현재 로컬 entrypoint가 실제 사용하는 고정본 |
| `receive_plan` | SHA 일치 | 현재 로컬 entrypoint가 실제 사용하는 고정본 |
| `receive_real_adapter` | SHA 일치 | 현재 로컬 entrypoint가 실제 사용하는 고정본 |
| `receive_runtime` | SHA 일치 | 현재 로컬 entrypoint가 실제 사용하는 고정본 |
| `receive_session` | SHA 일치 | 현재 로컬 entrypoint가 실제 사용하는 고정본 |

5개 RECEIVE 입력/계획/세션 소스는 같고 `concrete_bindings`·`full_episode`는 다르다. 서버 사본으로 최신 로컬 파일을 덮어쓰지 않았다. 현재 `run_session/build`의 `scope="full" / "observe"`, `right_lift_m` 계약을 유지한다. 이전 후보의 `execution_scope="FULL_TASK|SUPPORT_OBSERVATION"` 계약을 섞지 않았다. 자세한 old/new SHA는 [manifest](sweepick_261009_repository_source_manifest.json)에 있다.

## 실제 tree

`src/`의 기존 이름 파일은 호환 alias이며 운영 구현은 `src/sweepick/<기능>/` 한 곳에 있다. `learning/README.md`는 실제 기존 학습 코드를 연결한다. 없는 학습기·controller 파일을 만들지 않았다. 아래는 이번 PR의 제품 파일만이며 build/cache/private 환경은 제외했다.

```text
2ARM_ROBOT/sweepick/
  README.md
  docs/sweepick_261009_repository_path_map.csv
  docs/sweepick_261009_repository_source_manifest.json
  pyproject.toml
  src/sweepick/__init__.py
  src/sweepick/config/sweepick_HARD_GATE_ALLOWLIST.json
  src/sweepick/config/sweepick_PROGRESS_EVIDENCE.json
  src/sweepick/config/sweepick_jaw_profile_left.json
  src/sweepick/config/sweepick_motion_profile.json
  src/sweepick/config/sweepick_progress_profile.json
  src/sweepick/control/README.md
  src/sweepick/control/__init__.py
  src/sweepick/control/sweepick_collision_preview.py
  src/sweepick/control/sweepick_command_application.py
  src/sweepick/control/sweepick_execution_gates.py
  src/sweepick/control/sweepick_gripper_contact_hold.py
  src/sweepick/control/sweepick_gripper_contact_inspection.py
  src/sweepick/control/sweepick_joint_command_mapping.py
  src/sweepick/control/sweepick_joint_kinematics.py
  src/sweepick/control/sweepick_motion_progress.py
  src/sweepick/control/sweepick_servo_access.py
  src/sweepick/control/sweepick_trajectory_executor.py
  src/sweepick/control/sweepick_trajectory_profile.py
  src/sweepick/integration/README.md
  src/sweepick/integration/__init__.py
  src/sweepick/integration/sweepick_controller_bindings.py
  src/sweepick/integration/sweepick_handoff_snapshot_adapter.py
  src/sweepick/integration/sweepick_manipulation_handoff.py
  src/sweepick/integration/sweepick_manipulation_session.py
  src/sweepick/integration/sweepick_observation_session.py
  src/sweepick/integration/sweepick_receive_runtime.py
  src/sweepick/integration/sweepick_receive_session.py
  src/sweepick/integration/sweepick_replay_rig.py
  src/sweepick/integration/sweepick_resource_paths.py
  src/sweepick/learning/README.md
  src/sweepick/manipulation/README.md
  src/sweepick/manipulation/__init__.py
  src/sweepick/manipulation/sweepick_bimanual_handover.py
  src/sweepick/manipulation/sweepick_right_receive_dispatch.py
  src/sweepick/manipulation/sweepick_right_receive_planner.py
  src/sweepick/perception/README.md
  src/sweepick/perception/__init__.py
  src/sweepick/perception/sweepick_feedback_observation.py
  src/sweepick/perception/sweepick_observation_capture.py
  src/sweepick/perception/sweepick_observation_view.py
  src/sweepick/perception/sweepick_os30a_observation.py
  src/sweepick/perception/sweepick_wrist_capture.py
  src/sweepick/recording/README.md
  src/sweepick/recording/__init__.py
  src/sweepick/recording/sweepick_episode_recorder.py
  src/sweepick_261008_concrete_bindings.py
  src/sweepick_261008_full_episode.py
  src/sweepick_261008_receive_entry.py
  src/sweepick_261008_receive_plan.py
  src/sweepick_261008_receive_real_adapter.py
  src/sweepick_261008_receive_runtime.py
  src/sweepick_261008_receive_session.py
  src/sweepick_collision_cert.py
  src/sweepick_commission.py
  src/sweepick_contact01.py
  src/sweepick_episode.py
  src/sweepick_field.py
  src/sweepick_field_r6.py
  src/sweepick_full_chain.py
  src/sweepick_gates.py
  src/sweepick_grasp.py
  src/sweepick_kin.py
  src/sweepick_local_loop.py
  src/sweepick_move_a.py
  src/sweepick_pick01.py
  src/sweepick_progress.py
  src/sweepick_read03_capture.py
  src/sweepick_real_command.py
  src/sweepick_real_exec.py
  src/sweepick_real_rig.py
  src/sweepick_sdk_top.py
  src/sweepick_trajectory.py
  src/sweepick_view.py
  tests/sweepick_261008_control_fake_bus.py
  tests/sweepick_261008_manipulation_chain_fixture.py
  tests/test_sweepick_bimanual_handover.py
  tests/test_sweepick_command_application.py
  tests/test_sweepick_episode_recorder.py
  tests/test_sweepick_execution_gates.py
  tests/test_sweepick_feedback_observation.py
  tests/test_sweepick_gripper_contact_inspection.py
  tests/test_sweepick_gripper_execution.py
  tests/test_sweepick_gripper_recovery.py
  tests/test_sweepick_joint_command_mapping.py
  tests/test_sweepick_manipulation_session.py
  tests/test_sweepick_observation_session.py
  tests/test_sweepick_repository_paths.py
  tests/test_sweepick_trajectory_executor.py
  tests/test_sweepick_trajectory_profile.py
  docs/sweepick_261009_repository_review.md
```

## 변경과 보존 경계

- native 운영 모듈 28개를 기능별 안정된 이름으로 정리하고 기존 import/CLI 이름 28개를 얇은 호환 경계로 유지했다. 양팔 executor·servo·Jaw는 복제하지 않았다.
- 기존 시험·chain fixture의 import와 소스/자산 참조를 맞췄다. 보관된 commissioning writer와 폐기된 cap은 공개 runtime에 가져오지 않았다. 그 시험에서 사용하는 기존 fake bus만 단일 fixture로 재사용했다. 원래 보관본·시험은 제자리 보존했다.
- source ledger의 이전 key는 실제 canonical 파일로 해석한다. 새 기록은 실제 이동한 source SHA를 기록하며 과거 기록을 새 SHA로 덮지 않는다. collision subprocess는 설치된 module을 호출한다.
- `SWEEPICK_RECEIVE_DIR`가 지정되면 입력·planner·gear helper가 같은 고정본을 사용한다. 기본 경로는 이 branch에 포함된 canonical RECEIVE다. 관측 모듈의 source identity 검사도 실제 canonical 파일을 확인한다.
- profile·근거 JSON 5개는 byte-identical이다. 기존 필수 scene JSON 2개도 원본과 SHA가 같다. 이번에 빠진 작은 자산이 없으므로 별도 dataset/model 묶음을 복원하지 않았다.
- `2ARM_ROBOT` root, 기존 `shoe_sorting_data`·research·SIM·vendor package, 교육 실습과 외부 이동 저장소는 이동하지 않았다. root rename은 CI·ROS resource·상대경로 약 160곳의 결합 때문에 보류했다.
- main에는 AGENTS.md가 없어 공개 product README에 명명 원칙을 기록했고, 이 독립 clone의 로컬 AGENTS.md는 upload에서 제외했다. 활성 원본 AGENTS.md는 수정하지 않았다.

[이관표 CSV](sweepick_261009_repository_path_map.csv)에 old_path/new_path/역할/상태/source_revision/entrypoint/변경/new_sha256를 기록했다. 원래 native source는 main에 없었으므로 Git diff에는 신규 canonical 소스로 나타난다. 원본 SHA와 경로 변경 차이는 이관표·manifest로 검토한다. 제어 함수/class의 import/docstring을 제외한 AST를 고정본과 대조하며, 경로·source hash·동적 loader만 바뀐 함수는 별도로 기록한다.

## 검증 기록

| 검사 | 실제 결과 |
|---|---|
| 경로 변경 전 기존 무장치 기준 | 34 PASS, 5 deselected |
| 경로 변경 후 같은 기준 | 34 PASS, 5 deselected |
| 추가 경로·alias·자산·고정 AST·override 검사 | 5 PASS (fresh 설치 검사에 포함) |
| 설치 package에서 구조/gate | 13 PASS (override 추가 전) |
| 새·옛 CLI help | session/executor/adapter 3쌍, 총 6개 exit 0 |
| 지원 현장 환경에서 RECEIVE import | 3쌍 동일 module 확인; callback/controller 실행 없음 |
| 원본 소스 freeze 뒤 재대조 | 읽은 53개 파일 drift 없음 |
| fresh 가상환경의 CI 동일 설치/시험 | 14 PASS, 0 skip; 선언 의존성 + pytest만 설치 |

기준 명령은 기존 프로젝트 Python 3.12 환경에서 다음 네 파일을 실행했다. 경로 이동 검증을 위해 같은 34개만 전후로 대조했다. 이미 완료된 owner/STOP 반례 4개와 원본에만 보존한 archive-symbol 검사 1개는 제외했다. 새 skip을 넣지 않았다.

```bash
python -m pytest -p no:cacheprovider -q test_sweepick_joint_command_mapping.py test_sweepick_execution_gates.py test_sweepick_trajectory_profile.py test_sweepick_command_application.py -k "not no_travel_cap_is_left and not target_reversal_and_owner_change and not stop_request and not after_a_stop"
```

fresh 환경에서 CI가 확인하는 명령은 다음과 같다.

```bash
python -m pip install ./2ARM_ROBOT/sweepick pytest
python -m pytest -p no:cacheprovider -q 2ARM_ROBOT/sweepick/tests/test_sweepick_repository_paths.py 2ARM_ROBOT/sweepick/tests/test_sweepick_execution_gates.py
```

처음 alias 검사에서 viewer가 일반 library import가 아니라 즉시 argv를 읽고 HTTP 서버를 시작하는 기존 script라는 점을 확인했다. viewer 동작을 바꾸지 않고 script-only로 명시했다. 독립 검토에서 찾은 clean 환경 MuJoCo 의존성과 외부 planner의 gear 참조를 수정하고 해당 path 검사를 추가했다.

## 미실행과 현장 병합 주의

저장 SIM 두 성공 사례, 완료된 RELEASE owner/STOP 반례, run14 replay, 왼 C 측정, ACT 학습/forward, scene physics step, camera/bus/motor/torque/settings write는 실행하지 않았다. 이번 시험은 구조·무장치 연결이며 합성 support를 실물 근거로 보고하지 않는다. `real_execution_ready=false`, `motor_commands_sent=0`을 유지한다.

`tjj-runtime/v1`의 teacher·IK·collision·ACT는 main SIM과 다른 API/알고리즘이다. `sim_data_factory`, 정책 source/정규화, compiled scene, upstream SO-101 MJCF/mesh와 센서 SDK는 외부 runtime 의존성으로 기록하고 통째로 업로드하지 않았다. runtime이 없는 환경의 전체 조작·recording fixture는 NOT_RUN이며, skip 통과로 숨기지 않는다. `tabletop_replay.json`·desk JSON은 실제 branch에 포함된 자산이다. 기존 license/provenance와 vendor 경계는 보존한다.

로컬 통합 담당자가 다음 적용을 맡는다. 최신 source SHA와 이관표가 달라졌으면 함수/API 단위로 대조하고 최신 로컬 동작 위에 path/reference 변경만 병합한다. 새 package를 실행 중 세션에 교체하지 않는다. 기존 기록의 SHA·checkpoint를 변환하지 않는다. 현장 writer/support/오른 REAL profile/held pose의 실제 근거가 없는 부분은 MISSING이며 이 PR은 그 값을 만들지 않는다.

기존 Git 이력의 10 MiB 초과 파일 13개는 교육 데이터 등을 포함한 목록만 확인했고 삭제·LFS 전환·history rewrite는 하지 않았다. 신규 공개 대상은 소스·문서·작은 JSON뿐이며 raw dataset·optimizer·checkpoint·설치 환경·private 설정·작업 대화는 제외했다.

이 제출은 draft PR이다. main merge·현장 runtime 적용은 별도 절차이며 이번 작업에서 수행하지 않는다.
