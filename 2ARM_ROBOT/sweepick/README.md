# Sweepick — 현재 실행 소스 검토

record_id: `DAPIER-2026-10-15-sweepick-source-review`

SO-101 양팔의 실제 관측·집기·오른팔 접근을 기존 실행기로 연결하고, 실패 원인과 종료 상태를 저장 기록으로 대조하고 있다. 이 폴더는 2026-10-15 현재 로컬 실행 소스를 읽기 위한 공개 사본이다. 저장소 밖에 흩어져 있던 콘솔, launcher, native executor, 선택된 receive 모듈과 프로젝트 support 코드를 한곳에 모았다.

**검토 시작점: [코드 호출 지도](docs/CODE_MAP.md), [A-16 원인·수정](docs/A16_CLOSE_REVIEW.md), [A-17 검수](docs/a17/A17_CODE_AUDIT.md), [10/15 ID6 과부하 진단](docs/ID6_OVERLOAD_20261015.md).**

| 확인할 내용 | 파일 |
|---|---|
| 웹 START와 승인·단일 Owner | [console/control.py](console/control.py), [console/app.py](console/app.py) |
| 실제 launcher와 source/import 검증 | [native/RUN_APPROVED_SESSION.py](native/RUN_APPROVED_SESSION.py) |
| START·정렬·DESCEND·CLOSE·LIFT | [native/revision/sweepick_pick01.py](native/revision/sweepick_pick01.py) |
| Owner·receive 연결·STOP | [native/revision/sweepick_full_chain.py](native/revision/sweepick_full_chain.py) |
| Goal/ACK/readback·gripper 바인딩 | [native/revision/sweepick_move_a.py](native/revision/sweepick_move_a.py) |
| 선택된 receive 구현 | [native/pin](native/pin) |
| 저장 run 검수·소형 Export | [console/inspection.py](console/inspection.py), [console/records.py](console/records.py), [console/static/review.js](console/static/review.js) |
| 원본과 공개 사본의 파일 SHA | [SOURCE_INDEX.json](SOURCE_INDEX.json) |
| 의존성·공개하지 않은 입력 | [DEPENDENCIES.md](docs/DEPENDENCIES.md) |

## 현재 확인한 범위

- `20261014_155824_d4017e`: LEFT PICK/CLOSE/LIFT/CARRY와 RIGHT PREGRASP는 완료했다. 추가 관측 승인 대기 중 왼손목 영상 freshness 근거를 잃어 중단했다. 정상 회수는 수행하지 않았고, 이후 별도 사용자 요청 Torque OFF 확인과 구분한다.
- `20261015_092942_a4628c`: CLOSE 인증의 wrist_flex 2837→2835 tick 불일치로 REFUSED됐다. 이 CLOSE의 Goal/Torque write는 0회였다.
- A-16 수정: 같은 scene의 closing sweep을 한 번 재검사하고, 이후 fresh actual에 기존 1-tick 바인딩을 다시 적용한다. 정상 인증에는 재검사를 추가하지 않는다.
- A-17 무장치 시험: gripper actual 3344에서는 최초 Goal 3344, actual 3501에서는 최초 Goal 3431을 보냈고 Fake Bus readback이 일치했다. 3501은 수동 개방으로 가능한 actual이며 고장 증거로 취급하지 않는다.
- `20261015_112218_f0d866`: A-16/A-17 검수 이후 실물 LEFT CLOSE는 `GRIP_CONTACT_HOLDING`까지 진행했다. LIFT는 미도달이며, 직후 LEFT Gripper ID6에서 `0x20 Overload error` 및 handshake 시 ID6 누락이 기록돼 Run이 예외 종료됐다. 실제 물체 파지·모터 손상·정상 회수는 확인되지 않았다. [별도 진단 지침](docs/ID6_OVERLOAD_20261015.md) 참고.
- 현재 웹 launcher는 `full_chain=True`, `scope='RECEIVE_CONTACT'`를 고정하므로 LIFT 한정 종료를 지원한다고 표시하지 않는다. 관련 후속 작업은 별도 검수 및 실물 승인 후 진행한다.

## 공개 사본과 실제 실행본

`SOURCE_INDEX.json`의 `source_sha256`는 게시 시점 실제 로컬 원본, `public_sha256`는 이 브랜치 파일의 SHA다. 사용자 홈 경로, 고유 장치 식별값과 불필요한 작성자 표기는 공개 사본에서 정리했다. 기존 API/함수명은 호출 관계 비교를 위해 유지했다. 로컬 실행 소스·기존 원본 기록·이전 Source SHA는 변경하지 않았다.

실물 영상, 세션 전체, 장치별 EEPROM/Calibration, camera pairing/extrinsic, 승인 파일, 계정 설정, 모델 바이너리는 이 폴더에 없다. 동작 profile JSON은 출처와 분류가 있는 기존 알고리즘 파라미터이며 새 물리 한계를 선언하지 않는다. 이 폴더를 장비에 덮어쓰거나 과거 승인·tick을 재생하는 배포 절차로 사용하지 않는다.

## 무장치 확인

아래 검사는 표준 라이브러리로 파일 SHA·Python 구문·공개 대상만 확인하며 실행기 모듈을 import하지 않는다.

```bash
python3 2ARM_ROBOT/sweepick/tests/check_public_snapshot.py
```

로컬 실제 코드에서 확인한 시험 범위와 구형 실패는 [VALIDATION.md](docs/VALIDATION.md)에 구분했다. 하드웨어 의존성을 생략한 이 공개 폴더만으로 3-view 실물 기록이나 장비 검증이 재현된다고 주장하지 않는다.
