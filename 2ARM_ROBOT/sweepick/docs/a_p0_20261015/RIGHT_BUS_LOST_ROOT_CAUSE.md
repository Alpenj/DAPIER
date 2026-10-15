# RIGHT BUS_LOST — 20261015_151937_81d00c

**판정: BUS_LOST_CAUSE_UNCONFIRMED.** 저장된 기록만으로는 OS 수준 원인을 확정할 수 없다. 장치에는 접근하지 않았다. 이 세션은 콘솔에서 FAULT 상태이고, 실물 회수 확인이 없다.

## 확인된 사실

| 항목 | 값 | 근거 |
|---|---|---|
| 실패 동작 | `contact_right_return_initial`(오른팔 최종 Return)의 **계획 단계**. 실행기 `run(mode="return")` plan 호출에서 Bus open 실패. 모터 쓰기 0 | `handoff/08_contact_right_return_initial_right/plan/move.json` result REFUSED |
| 포트 | `/dev/dapier/right_arm` | 같은 파일의 `port` |
| 열려던 주체 | Owner 프로세스 안의 실행기 Bus(`sweepick_commission.real_bus`, 배타 open). Recorder Reader가 아니다 | `stage()` → `mv.run(open_bus=real_bus)` |
| 시각 | plan 시작 15:23:43(초 단위 기록). `owner_stop` t_host 1792045423.1345 (15:23:43.135) | move.json `started`, `episode/events.jsonl` |
| 직전 동작 | `contact_right_back_to_carry` HOLDING, 1.19 s 전에 종료 | events `segment` |
| SDK 반환값 | 없음. 패킷 송수신 전 포트 open 단계에서 실패 | lerobot `MotorsBus._connect` |
| 기록된 오류 | `the bus could not be opened: ConnectionError: ` (내용 없음) | move.json `refused.detail` |
| OS 오류 | **기록 없음** | 아래 참고 |
| USB 커널 로그 | 15:00–15:34:59 사이 USB/ttyACM 이벤트 없음. 오른 어댑터(usb 1-1.2)는 **15:34:59**에 disconnect·재등록. 실패 11분 뒤이며 회수 조치와 시각이 겹친다(원인과의 관계 미확인) | `journalctl -k` |
| Recorder | 실패 시점에 오른팔 포트를 빌려준 상태였다. 마지막 오른팔 행은 −135.6 s, 다음 행은 +0.5 s. Recorder 오류 0 | `episode/joints.jsonl`, `EPISODE.json errors` |
| 외부 점유 | 실행기 run()은 open 전에 `fuser` 점유를 확인한다. 점유가 있었으면 `port_owned`로 거부됐을 것이다. 기록된 결과는 bus_lost였다 | `sweepick_move_a.run` |

## 오류 내용이 비어 있는 이유 (코드로 확인)

lerobot `_connect()`는 `openPort()` 실패, `OSError`, `serial.SerialException`을 `ConnectionError("\nCould not connect on port '…'…")`로 감싼다. 메시지가 줄바꿈으로 시작한다. 실행기는 `str(e).splitlines()[0]`, 즉 빈 첫 줄만 기록하고 `__cause__`(errno)는 남기지 않는다. 그래서 EBUSY·EIO·ENOENT 중 무엇이었는지 기록에 없다. `Refuse`(배타 open 소유권 확인 실패)는 `ConnectionError`가 아니어서 이번 경로가 아니다.

## Owner의 반복 Bus Open/Disconnect

`Owner.read_registers()`는 읽을 때마다 양팔 포트를 각각 열고 닫는다. 기록으로 셀 수 있는 것은 아래와 같다. 전체 open 횟수 카운터는 없어 정확한 총수는 UNKNOWN이다.

- 승인 대기 6.574 s 동안 `observation_return_check` 17회. 포트 open/close 34회, 약 0.39 s 간격으로 읽기 1회
- 세션 전체에서 `observation_return_check` 21회(42회 open/close). 여기에 handoff segment 8개 각각의 시작 읽기, plan/run 실행기 open, other-arm 읽기가 더해진다

이 반복 open/close와 15:23:43 실패의 인과는 확인되지 않았다. 직전 읽기를 닫은 직후 다시 열 때 OS 수준에서 실패했을 가능성은 가설로만 남긴다.

## Elbow 미도달과의 분리

observe_0_0/0_1의 elbow +26/+23은 14:08 세션과 같은 '수용된 정착 미도달'이다. 이후 back_001·back_000·back_to_carry 3개 이동은 HOLDING으로 끝났다. Bus open 실패는 그 다음 Return 계획에서 별도로 발생했다.

## 하지 않은 것

자동 재접속, 재이동, Torque 변경, 오류 무시는 구현하지 않았다. `sweepick_move_a.py`는 그대로다.

**다음 원인 확인용 최소 변경 제안 (미적용):** open 실패 기록에 예외 사슬(`__cause__`의 type·errno·strerror)을 함께 남긴다. 동작은 바꾸지 않고 기록만 늘린다. move_a 변경이라 별도 승인이 필요하다.
