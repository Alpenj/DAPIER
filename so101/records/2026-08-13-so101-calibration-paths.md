# 2026-08-13 SO-101 실물 연결과 이중 캘리브레이션 경로

## 결론

새로 조립한 SO-101 leader/follower는 모터 ID 1~6 응답만으로 실기 준비가
완료되지 않는다. 조립 각도와 기계 범위가 기존 JSON과 다를 수 있으므로
teleoperation 전에 장비별 calibration을 새로 기록한다.

기본 경로는 LeRobot 공식 계약을 유지하는 안전 래퍼다. 별도 감사·복구 경로는
LeRobot를 import하지 않고 FEETECH `scservo_sdk`로 동일한 STS3215
레지스터 계약을 재현한다. 두 경로를 동시에 실행하지 않는다.

## 2026-08-13 확인 증거

- 서로 다른 두 USB serial board가 OS에서 각각 안정적인 by-id 경로로 인식됐다.
- 현재 Linux 계정은 `dialout` 그룹이어서 두 포트를 열 수 있다.
- 양쪽 bus에서 `shoulder_pan=1`부터 `gripper=6`까지 여섯 모터가 응답했다.
- 기존 follower/leader JSON은 모두 로드됐지만 새 조립에 유효하다는 증거는
  아니므로 재사용 판정을 보류했다.
- 기존 JSON 두 파일은 로컬 evidence 경로에 백업했다. 실제 serial ID와
  calibration JSON은 Git에 넣지 않는다.

## 첫 follower 시도와 실패 분석

첫 안전 calibration에서 torque-off 검증은 6축 모두 통과했다. 중앙 homing
단계 뒤 range recorder가 한 번만 읽고 즉시 종료되어 다섯 축 모두
`MIN=POS=MAX=2047`이 됐다. LeRobot는 같은 min/max를 감지해
`ValueError`로 저장을 거부했다.

이 실패는 위치 센서 고정으로 판정하지 않는다. 출력상 기록 루프가 반복되지
않았으므로 Enter가 너무 빨리 들어간 사례다. 기존 JSON과 사전 백업은
수정되지 않았지만, 중앙 homing offset은 servo에 기록됐으므로 완료 전
teleoperation을 금지하고 calibration을 다시 수행한다.

## 보강한 LeRobot 경로

`calibrate_follower_safe.py`는 기존 torque-off interlock에 다음을 추가했다.

1. 관절별 최소 span을 충족하지 못하면 저장하지 않는다.
2. 진행 중인 `SPAN`을 80열 이내의 `P/L/E/F/G` 한 줄에서 갱신해,
   좁은 터미널에서도 줄바꿈 출력 폭주를 막는다.
3. Enter를 누르면 즉시 판정하고, 미달이면 부족한 축을 표시한 뒤 실패 종료한다.
4. `q` 또는 Ctrl+C로 즉시 안전 취소할 수 있다.
5. 모든 축이 기준을 넘은 뒤 Enter를 눌러야 공식 LeRobot 저장 단계로 간다.
6. 성공·실패·취소 모두 마지막 torque-off를 시도한다.

최소 span은 정상 SO-101 전체 범위보다 작게 잡아 조기 Enter와 미이동 축만
차단한다. 숫자를 맞추기 위해 기계적 stop을 억지로 밀어서는 안 된다.

## LeRobot 미사용 경로

`calibrate_sts3215_direct.py`는 다음 계약을 독립 구현한다.

| 기능 | STS3215 주소 | 동작 |
|---|---:|---|
| 최소/최대 위치 | 9 / 11 | 기록한 안전 범위 저장 |
| homing offset | 31 | 현재 중앙을 2047로 맞추는 sign-magnitude 값 |
| operating mode | 33 | position mode 0 |
| torque | 40 | 쓰기 전후 0 확인 |
| EEPROM lock | 55 | 쓰기 동안 unlock |
| present position | 56 | range recorder 입력 |

안전 경계:

- 기본 `inspect`는 읽기 전용이다.
- `calibrate`와 `restore`는
  `--confirm WRITE_STS3215_CALIBRATION` 없이는 실행되지 않는다.
- 모델 번호 777과 ID 1~6을 모두 확인한다.
- 쓰기 전 레지스터 snapshot을 원자적으로 백업한다.
- range span gate와 wrist roll 0..4095 계약을 검증한다.
- EEPROM을 다시 읽어 요청값과 일치한 경우에만 최종 JSON과 receipt를 쓴다.
- 소스와 테스트가 `import lerobot` / `from lerobot` 부재를 검사한다.

## 선택 기준

| 상황 | 선택 |
|---|---|
| 정상 설치에서 leader/follower 최초 설정 | LeRobot 경로 |
| 기존 LeRobot dataset/policy와 바로 연결 | LeRobot 경로 |
| LeRobot import/CLI 문제와 servo 문제 분리 | 직접 SDK `inspect` |
| EEPROM과 JSON 계약 독립 재현 | 직접 SDK |
| 사전 snapshot 복원 | 직접 SDK `restore` |

직접 SDK 출력은 LeRobot-compatible JSON이지만, 독립 경로 성공만으로
teleoperation 준비 완료가 되지는 않는다. 두 장비 JSON schema 검사, 현재
normalized pose 비교, 유사 자세 정렬, 작은 `max_relative_target`의
방향 확인을 순서대로 통과해야 한다.
