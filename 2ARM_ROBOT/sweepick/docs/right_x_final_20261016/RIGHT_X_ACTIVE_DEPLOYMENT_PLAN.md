# RIGHT_X_ACTIVE_DEPLOYMENT_PLAN — 활성 배포 사전 점검 (2026-10-16)

상태: **계획만 세웠다. 실행하지 않았다.** 배포할지와 언제 할지는 사용자가 정한다. 이 계획을 따르더라도 실물 START 승인이 되지는 않는다. 현장 사진과 실제 비접촉 방향 확인이 아직 없다.

## 1. 배포 후 기대 SHA

**Native**
- 활성 폴더 `/home/robot/sweepick_261013_receive_alignment_restore`에 덮어쓴다.
- 이 폴더를 다른 경로로 바꾸면 안 된다. `PREPARED_SOURCE_MANIFEST.json`이 이 폴더의 절대 경로를 담고 있기 때문이다.

| 파일 | 지금 | 배포 후 |
|---|---|---|
| `RUN_APPROVED_SESSION.py` | `9578a1ee…` | `46def840…` |
| `revision/sweepick_full_chain.py` | `d97aa60f…` | `19e7ca35…` |
| `revision/right_x_metrology.py` | 없음 | `dd126c28…` |
| `revision/test_right_x_metrology.py`, `test_right_x_launch.py` | 없음 | 시험 파일(SHA는 감사 JSON) |
| 나머지 바인딩 파일 9개 (`confirmed_wrist_routes.py`, `pick01`, `move_a`, `grasp`, `episode`, profile 2개, `console_feedback.py`, `current_view_candidate.py`) | 그대로 | 그대로 |

**Console**
- 라이브 Console `/home/robot/sweepick_local_robot_console`에 덮어쓴다.

| 파일 | 지금 | 배포 후 |
|---|---|---|
| `control.py` | `c90beb6e…` | `73ee3bf1…` |
| `app.py` | `83ee66be…` | `0d2b0829…` |
| `records.py` | `976a32a5…` | `297a98f6…` |
| `static/app.js` | `108be397…` | `9f4da897…` |
| `static/field.js` | `6bde2946…` | `b892ec45…` |
| `static/index.html` | `8f3f8e89…` | `56be64b9…` |
| `static/style.css` | `c669c60b…` | `42e5b026…` |
| `CONSOLE17_TESTED_NATIVE_SHA256.json` | `6b07fb4d…` | `0e945546…` |
| `RIGHT_X_TESTED_NATIVE_SHA256.json` | 없음 | `73046bad…` |
| `live.py`, `inspection.py`, `replay.py` | 그대로 | 그대로 |

전체 64자리 SHA는 `RIGHT_X_ALL_SCOPES_SHA_AUDIT.json`의 `deploy_files`에 있다. 공개 저장소 사본은 홈 경로를 바꿔 SHA가 다를 수 있다. 원본과 공개본 SHA는 `SOURCE_INDEX.json`에 함께 기록했다.

## 2. 순서 (사용자가 배포를 결정한 뒤)

1. **사전 확인.** 라이브 Console이 PREPARED이고 owner 없음, `tmux sweepick-console` 외 장치 owner 없음, 디스크 여유를 확인한다(아래 4절).
2. **기존 PREPARED 작업 보존.** `20261015_175106_f5aabc`는 START하지 않는다. 폴더를 그대로 두고 기록으로 남긴다.
   - 기록 파일 SHA(10/16 점검): `MANIFEST.json` `0dbdaf61…`, `APPROVAL.json` `16ffa1e2…`, `JOB.json` `b7a223a5…`, `SOURCE_SNAPSHOT.json` `6b55badb…`
   - 이 Manifest는 옛 launcher(`9578a1ee`)에 묶여 있다. 배포 후 START 검사에서 SHA 불일치로 거부되는 것이 맞는 동작이다.
3. **백업.** 덮어쓸 파일을 `sweepick_2610xx_right_x_apply_backup/`에 복사하고 SHA를 기록한다.
4. **덮어쓰기.**
   - Native는 1절의 5개 파일을 활성 폴더에 넣는다. `review/right_x_metrology_20261016/`와 `review_tools/`는 선택이다.
   - Console은 1절 파일과 `test_*.py` 4개를 넣는다.
   - Native와 Console을 **같은 시점에** 바꾼다. 한쪽만 바꾸면 LEFT_START_TO_LIFT나 RIGHT_X가 닫힌다.
5. **배포 후 무장치 확인.**
   - Native: 활성 폴더에서 pytest(변수 unset, 순차). 기대값은 578개 ID에 38 failed / 1 error / 31 skipped / 508 passed이고 후보와 같아야 한다.
   - Console: `unittest discover`. 기대값은 175개 실행, skip 2개(계약이 없는 활성 Native를 가정한 시험)다.
   - `control.limited_backend()`와 `control.right_x_backend()`가 둘 다 True여야 한다.
6. **라이브 Console 재시작.** tmux `sweepick-console` 창에서 재시작하고 포트 8765를 쓴다.
   - 재시작하면 이전 PREPARED는 CONTROL_FINAL로 보존된다.
   - 새 작업은 운영자가 새 Manifest로 준비한다. 다음 세션을 자동으로 준비하지 않는다.
7. **되돌리기.** 3번 백업을 같은 경로로 되돌리고 Console을 재시작한다. 이전 SHA로 돌아왔는지 감사 스크립트로 확인한다.

## 3. 다시 개발하지 않는 것

- 장치 udev와 카메라 점유 검사 보완은 이미 끝났다(`resolve_devices`, `wrist_cover_check.py`).
- 손목 가림 시험은 USB 경로가 바뀔 때만 다시 한다.

## 4. 다음 실제 Episode를 기록할 수 있는가: 디스크

- 10/16 점검: `/`에 11G 여유(233G 중 96% 사용).
- 기존 실행 기록 폴더 크기(`du -sh`): 실행한 작업은 0.5–1.8 GB, 가장 큰 것은 `20261015_140844_3cc05c` 1.8 GB다. 전체는 5.4 GB다.
- 가장 큰 기록을 기준으로 잡으면 11 GB는 약 6개분이다. 다만 시스템 전체가 96% 찬 상태라 다른 작업도 이 공간을 쓴다.
- 계산상으로는 다음 Episode 1개를 기록할 수 있다. 하지만 여유가 적으므로 사용자의 별도 디스크 점검 결과가 오면 그 수치로 다시 판단한다.
- RIGHT_X 세션은 F/V마다 운영자 결정을 기다리는 만큼 길어지므로 기록이 더 클 수 있다. 실제 크기는 측정하지 않았다.

## 5. 실물 START 전 남은 조건 (이 계획 밖)

- RIGHT_X F/V의 방향·시작 자세·비접촉 공간 현장 확인. 각 요청마다 운영자가 직접 확인한다.
- 현장 사진.
- 운영자 입력 5개는 매번 운영자가 정한다. ±5/10/20 mm는 권장값이 아니다.
- **LIVE_READY가 아니다.** SHA가 같거나 `can_prepare=true`여도 물리 안전 승인이 아니다.
