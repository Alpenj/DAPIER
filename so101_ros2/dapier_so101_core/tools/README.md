# Bounded pregrasp / wrist executor

record_id: DAPIER-2026-09-23-native-pregrasp-feedback

현재 장치 없는 연결 검증까지 완료했다. 실물 구동·접촉·들림·HOLD 수용은 미검증이다.

호출은 `execute_bounded_pregrasp.py` → `bounded_pregrasp_plan()` → 이 디렉터리의
`bounded_pregrasp_main.cpp` → `execute_pregrasp()` → transport의 `send()/read()`다.
손목 영상 오차는 기존 `wrist_correction_intent()`의 제안으로 들어오며, FK/축/경로를
검사한 후보와 관절 목표가 일치해야 같은 진입점에 연결된다. Python은 모터 SDK를 호출하지 않는다.

실제 transport는 로컬 Feetech native `SerialPort/CommunicationProtocol`을 재사용한다.
ROS plugin의 고정 midpoint·설정 변경·deactivate torque-off는 호출하지 않는다.
속도 register 46, Goal_Position 42, Torque_Enable=1 register 40만 쓰는 경로다.
calibration/EEPROM/보호 register 쓰기와 정상 종료의 자동 torque-off는 없다.
외부 SDK가 C++20을 요구하므로 이 standalone build도 C++20을 사용한다.

장치 없는 확인(저장소 루트):

```bash
bash so101_ros2/dapier_so101_core/tools/build_bounded_pregrasp /tmp/bounded_pregrasp_native
python so101/hardware_tools/motion/check_native_pregrasp.py /tmp/bounded_pregrasp_native /tmp/native-pregrasp-evidence-new
```

위 Python은 프로젝트 가상환경에서 실행한다. 이미 있는 증거 디렉터리를 덮어쓰지 않는다.
실제 launcher/C++ loop를 통과하고 transport만 지연 plant로 바꾼다. 정지 plant는 timeout,
손목 제안은 `WRIST_ALIGN_REACHED_HOLDING`으로 끝난다. `task_success`는 항상 false다.
`bounded_pregrasp_smoke.cpp`는 stale/cancel/부분 arming/양자화 경계도 검사한다.

명목 quintic 이동 시간과 실제 정착 시간은 다르다. 저장된 후보를 지연 MOCK으로
실행했을 때 공통 진행률 제한으로 경로 이탈은 해소됐지만, 기존 `명목 시간 + 1초`
예산에서는 종점에 도달하지 못했다. 후보 launcher의 `--maximum-duration-s`로
최대 60초 안에서 실행 예산을 명시할 수 있다. 기본값은 그대로이며 이미 작성된
plan의 시간은 이 옵션으로 바꿀 수 없다. 명시한 시간은 승인할 plan에 저장되고,
타임아웃 시 자동 연장·재시도하지 않는다. MOCK에서 정한 시간은 실물 응답 근거가 아니다.
같은 저장 후보의 별도 40초 MOCK 실행은 기록 구간 약 24.07초에 관절 종점에 도달했다
(최대 관절 잔차 0.000424 rad). 원래 16.97초 실패는 그대로 보존했다.
이 결과는 `PREGRASP_REACHED_HOLDING`이며 Cartesian endpoint·실물 task 성공은 아니다.
옵션 전달·잘못된 예산·stale 입력 거부는 `test_real_sensor_ik_adapter.py`와
`test_pregrasp_refusal.py`에서 장치 없이 검사한다.

실물 실행에는 별도 현장 승인, 실제 source/binary/profile/plan 고정, 최신 target/readback,
물리적으로 확인한 매핑, 위치 0.5 mm·수직축 2° 및 tracking envelope 전체의 clearance
30 mm 증거가 필요하다. 후보의 단일 경로 clearance를 envelope 증거로 승격하지 않는다.
토크가 꺼진 PREGRASP와 토크를 유지하는 WRIST_ALIGN 진입을 구분하고 실제 register를 확인한다.

현재 남은 연결 조건: 실제 모드에 맞는 RGB/depth 정합과 metric target, 현재 양팔 상태가
모델 범위 안에 있는 경로, tracking envelope 증거, 실물 feedback의 Cartesian endpoint 검사,
실제 wrist frame/관절 시각 연결, 접촉·들림·3.000초 HOLD·지지 종료 검증.
외부 SDK 한 호출 내부의 재시도/OS drain은 즉시 취소를 보장하지 않는다. 호출 사이의 취소·시간
검사와 별도 프로세스 종료 제한은 독립 하드웨어 watchdog 또는 graceful stop의 증명이 아니다.

## 관측 기반 HOLD 진입점 — 장치 없는 검증

기존 native 실행기에서 `phase=HOLD`, `initial_torque_enabled=true`, `start_rad=goal_rad`인
정지 계획을 처리한다. 같은 C++ 명령 제한·transport·실측 feedback 경로를 사용한다.
관절이 도착한 뒤 독립 물체 관측으로 연속 3.000초를 확인하며, 관절 도착이나 CLOSE 전송은
물체 파지 근거로 사용하지 않는다. `observed_hold_verified=true`라도 지지 종료가 별도이므로
전체 `task_success`는 false다. 자동 torque-off는 하지 않는다.

계획의 `hold_observation`은 `path`, `run_id`, `object_id`, `calibration_revision`,
`producer_sha256`, `boot_id`, `observer_physically_verified`를 고정한다. producer는 관측 JSON을
임시 파일 작성 후 rename으로 갱신하고, 참조하는 원본 프레임은 보존해야 한다.
관측 계약 `dapier.block-hold-observation.v1`은 위 식별자와 다음 필드를 요구한다.

- `source_kind`: `mock` 또는 `hardware`; 실제 transport와 일치해야 한다.
- `sequence`, `captured_monotonic_ns`: 같은 부팅의 증가하는 취득 번호·시각. 취득 후 나이는
  250ms 이하이며 관측 간격도 250ms를 넘으면 중단한다. HOLD 이전 시간은 유지 시간에 포함하지 않는다.
- `frame_path`, `frame_sha256`: 보존한 원본 파일과 SHA. 같은 bytes를 새 번호/시각으로 다시 제출하면
  중단한다. 완전히 같은 영상의 새로운 취득인지 구분할 수 없는 경우도 통과 근거로 쓰지 않는다.
- `bilateral_grasp_verified`, `external_support`: 실제 관측에서 양쪽 jaw 파지를 확인하고 외부 지지가
  없음을 확인한 결과. 명령값이나 모터 위치만으로 true/false를 만들어서는 안 된다.
- `bottom_clearance_lower_bound_m`: 원래 지지면에서 물체 바닥까지 들린 거리의 보수적 하한.
  불확실성을 반영한 값이 0.030m 이상이어야 한다. camera Z나 물체 중심 높이로 대체하지 않는다.

실제 관측 producer의 위 의미는 아직 실물 검증하지 않았다. 실물 모드는 기존 물리 매핑·센서·경로
검사에 더해 검증된 observer와 **`VISIBLE_LEFT_OBSERVED_HOLD`** 승인을 요구한다.
이 문자열은 실행안의 승인 범위를 정의할 뿐이며 이 문서가 장치 실행 승인은 아니다.
아래 LIFT 연결은 MOCK에서 검사했다. 실제 목표의 접촉 경로 준비와 지지 종료는 아직 미완료다.

```bash
python so101/hardware_tools/motion/check_native_pregrasp.py /tmp/bounded_pregrasp_native /tmp/observed-hold-evidence-new hold
```

위 검사는 명시적 MOCK 원본·관측을 생성하며 카메라를 열지 않는다. 실제 launcher→native 실행에서
정상 HOLD, 외부 지지 발생, 관측 정지, 원본 재사용을 확인했다. C++ smoke는 마지막 관측 처리 중
관절 feedback이 만료되는 경우와 기존 PREGRASP 경로·오류 중단도 검사한다. 신규 증거 파일을 사용하고
이미 통과한 전체 IK/physics 회귀는 반복하지 않았다.

### 관측 기반 CLOSE → GRASP_CONFIRM (장치 없는 검증)

기존 native 루프에 `phase=CLOSE`를 연결했다. 팔 5개 관절의 목표는 시작값과 같아야 하며,
gripper 목표만 감소한다. 모든 명령은 기존 ControlIntent 검증·속도/관절 제한·path envelope·
transport·measured feedback을 통과한다. `grasp_observation`은 HOLD와 같은 run/object/calibration/
producer/boot/raw SHA binding을 쓰고 schema는 `dapier.block-grasp-observation.v1`이다.
`bilateral_grasp_verified`는 true/false/null을 구분한다. null은 불확실하므로 전송 전에 거부한다.

첫 positive 관측은 측정된 집게 간격에서 닫힘을 멈춘다. 이후 서로 다른 fresh frame에서 파지를
재확인하고 관절이 안정되면 `GRASP_CONFIRMED_HOLDING`을 반환한다. 완전 닫힘 목표 도달이나
실제 들림을 뜻하지 않는다. `phase_completed=true`, `observed_grasp_verified=true`여도
`reached_joint_endpoint=false`, `task_success=false`다. 판정 변경/유실/오래된 관측/원본 재사용은
추가 닫힘을 중단한다. preflight 관측 이력도 실행 진입에 보존한다. 정상 종료에서 torque-off하지 않는다.

`wrist_servo_adapter.block_grasp_observation_from_wrist()`는 기존 native wrist 캡처 JSON과
원본 PNG/NPY, 선택적 frame-bound detection mask를 읽어 기존 centroid 처리를 재사용한다.
취득 시각·boot·원본 SHA를 보존하며 visibility와 파지 판정을 구분한다. 현재 RGB 특징만으로는
양쪽 접촉·들림·외부 지지를 판정할 수 없으므로 해당 필드는 null이다. 이 producer의 출력을
native reader까지 넣어 unknown이 motion 없이 거부되는 것을 검사한다. 저장 영상 replay를
fresh 관측으로 바꾸지 않는다.

```bash
python so101/hardware_tools/motion/check_native_pregrasp.py /tmp/bounded_pregrasp_native /tmp/observed-close-evidence-new close
```

이 검사는 장치를 열지 않는 동일 launcher/native 경로다. CLOSE의 **실물 실행은 코드에서 차단**한다.
기존 PREGRASP의 일반 30mm certificate는 jaw/block 접촉 경로 검증을 대신할 수 없고, 실물 observer도
미검증이기 때문이다. 별도 승인 문자열이나 true 플래그만으로 이 차단을 우회할 수 없다.
LIFT의 actual-target IK/접촉 경로와 지지 종료는 후속 작업으로 남아 있다.
이 단계는 물체를 집어 들었다는 실물 성공이 아니다.

### GRASP_CONFIRM → LIFT → 기존 observed HOLD 연결

`phase=LIFT` 계획은 `grasp_confirmation: {path, sha256}`으로 직전 native CLOSE trace를 참조한다.
같은 run/object/calibration/producer/boot, profile와 transport인지 확인하고, 실제로 기록된
`GRASP_CONFIRMED_HOLDING`의 실측 자세와 시작값을 대조한다. 이전 fully-closed 목표를 시작값으로
사용하지 않는다. 기존 raw frame의 재사용과 이전 sequence/시각은 단계 경계에서도 거부한다.

집게 goal은 start aperture와 같아야 하며 팔 관절만 이동한다. 실측 초기 aperture의 0.001rad 초과
차이는 거부하고, 허용 오차 내에서도 명령 경로는 승인된 aperture에 고정한다. 실측값은 수정하지 않는다.
`hold_observation`의 기존 원본·시간·metric 계약을 LIFT 중에도 읽고 파지 유실/미확정/stale를
거부한다. 지지면 위에 아직 놓인 이동 초기에는 외부 지지가 있을 수 있다.

관절 종점 도달 후 기존 `ObservedBlockHold`가 unsupported bottom clearance >=30mm를 검사하고
그때부터 연속3.000초를 센다. 이동 시간은 포함하지 않는다. 동일 native 실행/transport 안에서
`LIFT_TRAVEL → HOLD_OBSERVING → HOLD_REACHED_HOLDING`으로 이어진다. 종료 시 torque를 유지하며
`task_success=false`다. 객체를 지지면에 되놓는 정상 종료는 별도 연결이 필요하다.

```bash
python so101/hardware_tools/motion/check_native_pregrasp.py /tmp/bounded_pregrasp_native /tmp/lift-evidence-new lift
```

검사는 먼저 native CLOSE를 실행한 결과에 다음 LIFT 계획을 묶는다. CLI 단계별 mock transport의
초기값은 이전 실측 결과이며, C++ smoke는 같은 lagged plant를 CLOSE부터 HOLD까지 계속 사용한다.
프레임은 명시적 synthetic MOCK 관측이다. 실제 블록 LIFT의 IK/경로 검사나 물리 마찰/접촉 검증이 아니다.
단일 실패 연결을 다시 검사할 때 마지막 인자로 `lift_old_grasp_frame` 등 해당 case를 선택할 수 있다.
CLOSE와 LIFT의 실물 실행 차단은 유지한다. 실물 observer가 미확정인 값을 true로 바꾸지 않는다.
