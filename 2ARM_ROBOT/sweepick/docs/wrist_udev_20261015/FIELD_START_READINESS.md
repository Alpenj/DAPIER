# FIELD_START_READINESS — 손목 udev·점유·USB 변경 대응 (2026-10-15)

**판정: START 불가. 운영자 손목 카메라 가림 시험 1회가 필요하다.** 실물 START, Motor/Torque write는 하지 않았고, 이번 작업에서 카메라도 열지 않았다.

| 항목 | 상태 |
|---|---|
| udev 손목 규칙 | 정정 완료(SHA `146b2ce4…`, 원본 `7844964a…` 백업). LEFT 1-1.3 → `left_wrist_rgb` = video10, RIGHT 1-1.1 → `right_wrist_rgb` = video8 |
| 다른 `/dev/dapier` 링크 | 변경 없음 |
| Native SHA | 10/10 일치 |
| 콘솔 점유 검사 | 실제 노드 기준이다. 링크·경로·팔 Serial·재연결 이상은 거부한다 |
| 현재 콘솔 | 점유 `UNKNOWN`, `can_prepare=false`. 사유: 10/13 가림 시험 이후 카메라가 재연결됨(devnum 31→34, 33→32) |
| PREPARED | `20261015_175106_f5aabc`(미승인). 가림 시험 기록 전에는 START하면 거부된다. 이전 `172337_7b36aa`는 시작하지 않았고 기록은 보존했다 |

## USB 위치·허브를 바꿀 때

| 장치 | 묶는 기준 | 위치·허브 변경 시 |
|---|---|---|
| 양팔 | 어댑터 고유 Serial | 그대로 맞게 잡힌다. 콘솔이 팔 링크의 Serial을 페어링 기록과 다시 대조한다 |
| 상단 카메라 | 고유 Serial | 그대로 맞게 잡힌다 |
| 손목 카메라 2대 | Serial이 없어 USB 경로로 묶음 | 링크가 사라지거나 경로가 어긋난다 → PREPARE/START 거부(안전하게 멈춤) |
| 같은 두 포트끼리 손목 카메라 교체 | 소프트웨어로 구분 불가 | 재연결(devnum·부팅 변경)이 감지되면 가림 시험 전까지 거부 |

- 같은 포트에 그대로 다시 꽂았거나 재부팅한 경우: `wrist_cover_check.py confirm`
- 포트나 허브를 바꾼 경우:
  1. `wrist_cover_check.py discover`로 두 카메라를 찾고 가림 시험을 한다. udev 줄과 계획 경로는 제안 파일로만 나온다.
  2. 운영자가 확인한 뒤 udev와 계획을 반영한다.
  3. `confirm`을 실행한다.

## START 전 남은 것

1. 운영자 가림 시험(`confirm`) 1회
2. 팔·물체 배치·지지, 감독자, STOP 준비
3. 사전 승인 옵션은 선택 사항이다. 처음에는 수동 승인을 권장한다

SHA 일치, `can_prepare=true`, 점유 `{}`는 실물 안전 승인이 아니다.
