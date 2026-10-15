# 2026-10-15 LEFT Gripper ID 6 과부하 — 진단 기록과 읽기 전용 점검

## 증거와 현재 판정

- 현장 Run: `20261015_112218_f0d866` (`RIGHT_RECEIVE_NONCONTACT_OBSERVATION`). 이번 작업은 종료 상태 `FAULT`, 정상 회수 및 마지막 LEFT Torque 상태는 미확인.
- LEFT CLOSE는 `GRIP_CONTACT_HOLDING`을 기록했다. 이것은 **물체 파지·LIFT 성공 증명 아님**. LIFT는 실행되지 않았다.
- 종료 근처 관측: `Present_Position` read의 ID 6 상태 응답 `0x20 [RxPacketError] Overload error`. 이후 LEFT 버스 handshake에서 ID 1~5는 발견됐고 ID 6은 `Missing motor IDs`로 나타남.
- 원인 불명: 기계적 구속/패드 걸림, 접촉부 부하, 연결/전원, 상태 오류를 모터 누락으로 처리하는 SDK 중 무엇이 지속 문제인지 아직 확인하지 못했다. 단선·소손 확정 금지.
- 승인된 1회 실물 Run이 **다음 Run의 자동 승인으로 이어지지 않는다.**

## 현장 확인 순서

1. **재실행하지 않는다.** LEFT 팔/물체를 지지하고, 남은 동작·토크 상태를 과거 기록으로 추측하지 않는다. 기존 owner 종료와 작업 공간을 확인한다.
2. 안전하게 장치 전원을 분리한 뒤 그리퍼 패드·기구 간섭, 파지 중 걸림, ID 6의 3핀 케이블/커넥터, 눌림·당김, 발열·냄새·손상을 육안 확인한다. 전원 인가 중 탈착하거나 Torque ON으로 의심되는 관절을 강제로 돌리지 않는다.
3. 걸림/손상이 없고 현재 장치가 지지·감독되는 경우에만 **별도 읽기 전용 승인**으로 단발성 ID 6 진단을 수행한다. 기존 실행기와 다른 프로세스가 동일 `/dev/dapier/left_arm`을 열지 못하도록 한다. 모터/토크/EEPROM/ID에 대한 WRITE 금지.
4. 우선 ID 6 raw Ping 결과의 **통신 상태와 servo error byte를 구분**한다. 필요한 경우 같은 버스 ID 5를 대조군으로 조회한다. `FeetechMotorsBus.connect()`의 6축 handshake 실패만으로 ID 6의 물리적 무응답을 확정하지 않는다.
5. 모터가 응답하면 원시 `Servo_Status(65)`, `Present_Position(56)`, `Present_Load(60)`, `Present_Voltage(62)`, `Present_Temperature(63)`, `Present_Current(69)`, `Torque_Enable(40)`를 시각·통신 결과·servo error byte와 함께 저장한다. 장착된 STS3215 변형/전원 사양을 우선 확인한다.
6. 응답이 없다면 더 이상 위치 명령을 보내지 않고, 전원 격리 후 커넥터·케이블·전원 공급 경로를 조사한다. 오류가 재현되면 기존 하드웨어 점검 절차로 이관한다.
7. 현재의 실물 Torque OFF/회수 증거를 별도로 남긴다. `safe_shutdown=UNKNOWN`을 `HOME` 또는 `OFF`로 임의 변경하지 않는다.

## 데이터·소프트웨어 주의사항

- 이 Run의 `CLOSE`와 `LIFT`는 구분한다. CONTACT 후보에서 물체 보유로 라벨 승격하지 않는다.
- `AI_REVIEW.first_error.time_s=0` 때문에 오류 시점 주변 Export가 첫 프레임으로 선택된 현상은 별도 대시보드 파생 검수 결함이다. 실제 최초 모터 오류 시각은 원본 피드백/episode 기록에서 확인하고, 원본을 수정하지 않는다.
- 앞으로 A / B / Console 수정본은 이 브랜치의 하위 `native/`, `console/`, `tests/`, `docs/`로 나누되 로컬 실행본과 공개용 사본의 SHA/차이를 `SOURCE_INDEX.json` 등에 구분한다.
- 이 **공개** 브랜치에 raw Calibration/EEPROM, 장치 ID, 승인 비밀, 전체 REAL 영상, 대용량 세션, 모델 바이너리 및 절대 로컬 private path를 커밋하지 않는다. 승인·Owner·STOP/회수 경로를 변경하지 않는다.

## 후속 READ-ONLY 조회 결과 — 2026-10-15

이전 진단의 SDK import 문제(`from scservo_sdk import sms_sts`)를 수정하고, 설치된 `scservo_sdk`의 `PortHandler`/`PacketHandler(0)`으로 LEFT ID6을 직접 읽었다. 사용자가 전달한 원시 출력은 아래와 같다.

| 항목 | 원시값 | 해석 |
|---|---:|---|
| `ping()` model | 777 | 기존 장착 STS3215 모델 코드와 일치 |
| `ping()` comm | 0 | 정상 통신 |
| `ping()` error | `0x00` | 응답 오류 비트 없음 |
| `Servo_Status` (65) | 0 | 조회 시점의 과부하/상태 플래그 없음 |
| `Present_Position` (56) | 3497 ticks | 저장된 software Calibration 상한 3431보다 66 ticks 개방된 실제 위치; 사용자 수동 개방 가능성과 부합 |
| `Present_Load` (60) | 0 | Torque OFF 상태의 부하 피드백 |
| `Present_Voltage` (62) | 124 | STS3215 계열 환산 12.4 V (0.1 V/LSB) |
| `Present_Temperature` (63) | 37 | 37 °C |
| `Present_Current` (69) | 0 | Torque OFF 상태의 전류 피드백 |
| `Torque_Enable` (40) | 0 | 해당 ID6 조회 시점 토크 비활성 |

`port.closePort()`로 종료됐으며 조회 코드는 **Ping 및 READ만 포함**했고 Goal/Torque/EEPROM Write를 호출하지 않았다.

**판정: `ID6_RESPONSIVE_STATUS_CLEAR_TORQUE_OFF_AT_READ`.** 직전 오류 시점의 `0x20` 기록은 유효하며 이번 정상 응답으로 과거 오류가 사라지는 것은 아니다. 다만 이 시점에서는 ID6의 영구 고장/상시 무응답/지속 과부하라는 가설을 지지하지 않는다. 상태가 해제된 물리적 계기(부하 해소, 전원 재투입, 프로토콜의 status handling 등)와 이전 `safe_shutdown=UNKNOWN`은 해결되지 않았다. LEFT Arm 나머지 5축의 현재 Torque/OFF, 전체 HOME, 로봇 회수는 이번 단일 ID6 READ로 확인할 수 없다.

재현을 위한 새 실물 CLOSE/Grip test, EEPROM 초기화, 과부하 보호값 완화, Torque 강제 ON은 이 결과로 승인되지 않는다. 기존 단계별 운영·현장 승인 하에서만 후속 실험을 계획한다.

참고: STS3215 레지스터 주소/전압 환산 https://github.com/hybotix/STS3215-Tutorial/blob/main/REGISTER_REFERENCE.md

## 문서 근거

- 프로젝트: 2026-10-15 제출 Run 원본 `owner.log`, `FAILURE_PACKET`, `AI_REVIEW`, `INSPECTION`.
- Feetech STS3215 제품 사양: https://files.seeedstudio.com/products/Feetech/108090023_STS3215-C001_Datasheet.pdf (해당 제품 변형의 과부하 보호 설명).
- Feetech/LeRobot 모터 버스: https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/motors_bus.py (`ping()`은 상태 응답 오류에서도 `None`을 반환할 수 있음).

**이 문서는 기록과 진단 계획이며, 실물 장치 구동 또는 진단 실행 승인이 아니다.**
