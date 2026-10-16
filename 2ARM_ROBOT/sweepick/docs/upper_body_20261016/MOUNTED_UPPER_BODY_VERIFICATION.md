# sweepick 상부(TurtleBot 탑재 팔) 바닥 집기 검증 — 2026-10-16

근거 JSON: `MOUNTED_UPPER_BODY_VERIFICATION.json` (같은 폴더). 모두 **OFFLINE** 결과다. 저장된 REAL 프레임은 읽기만 했고, 버스는 가짜이며 모델로 계산했다.

`executable=false` · `motor_write_authorized=false` · `FIELD_READY=false` · `REAL_VERIFIED=false`

## 운영 경계
- 이 작업에서 모터를 움직이려고 장치를 연 횟수 0. Goal/Torque/EEPROM 쓰기 0. 이 작업이 낸 START 0. 활성 native/Console 배포 0. main merge 0.
- 읽기 전용 관절 읽기는 사용자 승인 아래 ReadOnlyBus로 했다(쓰기 호출 0).
- run 20261016_164052_9b3c13은 운영자가 라이브 콘솔(책상 보정 native)에서 시작한 실행이다. 이 기록은 읽기만 했다.

## 탑재 셋업 (`MOUNTED_FLOOR`, 후보본 `native/revision/sweepick_setup.py`)
- **월드 = 로봇 base frame.**
  - 탑 카메라 위치·pitch는 A mobile scene 값을 쓰고, 화면 안 회전은 보정했다(화면 위 = 로봇 전방).
  - 확인 근거: 관측한 보드가 −12 mm 높이에서 수평으로 나오고(법선 기울기 0.9°), 모델 팔을 실제 탑 영상에 투영하면 실제 팔 위에 겹친다.
- **왼팔 장착 보정 `(0.036, −0.016, −0.016)` m.**
  - 실제 팔·프레임을 옮긴 것이 아니다. 실제 팔이 이미 있는 자리에 **모델을** 맞춘 것이다.
  - 근거: run 164052 실제 탑 depth에서 왼팔 점 중 6 mm 이내 비율이 home 0.33→0.77, pre-grasp 0.66→0.88로 올랐다(서로 다른 두 자세).
  - 오른팔은 이미 0.80이라 그대로 두었다.
- **책상 보드 체인 `T_DATUM_FROM_BOARD`가 run 164052 pre-grasp 빗나감의 원인이다.** 탑재 상태에서는 책상의 보드–팔 관계가 성립하지 않는다.
- run 164052의 블록 위치는 로봇 좌표 (0.325, 0.032)였다. 왼팔 도달 범위 밖이다.

## 후보 칸 단계표 (왼팔, 바닥, 실제 탑재 시작 tick)
| 중심 (m) | yaw | TEACHER_IK_ONLY | A_PLAN_PICK_PASS | NATIVE_STAGE_CERTIFIED | pad 여유(한쪽) | 탑 화면 안 | 손목 모델 카메라 시야 | 5 cm 이웃 plan 통과 |
|---|---|---|---|---|---|---|---|---|
| (0.10, 0.30) | 0 / 45 | ✓ / ✓ | ✓ / ✓ | ✓ / ✓ | 5.27 / 5.32 mm | ✓ | ✓ (≤0.6°) | 8/8 |
| (0.20, 0.05) | 0 / 45 | ✓ / ✓ | ✓ / ✓ | ✓ / ✓ | 5.23 / 5.41 mm | ✓ | ✓ (≤0.7°) | 8/8 |
| (0.15, 0.20) | 0 / 45 | ✓ / ✓ | ✓ / ✓ | ✓ / ✓ | 5.37 / 5.26 mm | ✓ | ✓ (≤0.6°) | 7/8 |

- NATIVE_STAGE_CERTIFIED는 `stage(execute=False)` 결과다. 가짜 버스에서 실제 `sweepick_collision_cert.py`를 돌렸고, pregrasp·descend_a·descend·close sweep·lift가 모두 CLEAR였다.
- **가정:**
  - 관측 gate는 SYNTHETIC이고, close 결과는 GRIP_CONTACT_HOLDING으로 가정했다.
  - 손목 카메라는 모델 카메라다. 방향을 실제 손목 영상과 대조하지 않았다(USER_UNVERIFIED).
  - 왼팔 장착 보정은 run 하나·자세 두 개에서 얻었다.
  - 바꾼 것은 블록 위치뿐이다.
- TEACHER_IK_ONLY는 work6 grid 결과다. 장착 보정 이전 scene에서 책상 home을 시작점으로 썼다.
- 오른팔: A `plan_pick`가 왼팔 전용이라 N/A다.

## 회귀
- **native, `SWEEPICK_SETUP=DESK`:** 공통 541개 ID 결과가 활성본과 같다(471 pass / 38 fail / 1 error / 31 skip). 새 `test_mounted_floor` 6개는 통과했다.
- **native, 기본값 MOUNTED:** 책상 fixture에 묶인 시험 11개의 결과가 바뀐다(10개 실패로 바뀜, 1개 통과로 바뀜). 기존 묶음은 `SWEEPICK_SETUP=DESK`로 돌려야 한다.
- **upper_body 계약:** 35개 통과(반례 시험 7개 포함).
- **변이 시험:** 새 조건 7개를 하나씩 끄면 매번 시험이 하나씩 실패했다. 소스는 원래대로 되돌렸다.

## 계약 수정 (`upper_body/sweepick_upper_body.py`)
1. **stamp:** 유한값만 받는다(Infinity·NaN·bool 거부). `after()`는 유한 stamp와 비어 있지 않은 공통 clock_id를 요구한다.
2. **place 분리:** 단계를 다음 순서로 나눴다.
   - PLACE_PLAN/AREA_VERIFIED
   - approach
   - 정지 후 RELEASE 직전 영역 재확인
   - RELEASE
   - 분리와 착지 관측
   - RETREAT 확인
   - SAFE_END verified
   - ARM_DISPOSAL_CONFIRMED는 이 모든 단계를 거친 뒤에만 낸다.
3. **hold_evidence:** 손목 영상에 다음이 모두 있어야 한다.
   - 64-hex sha
   - 같은 owner·같은 target
   - 자세 확보 이후의 read
   - 이전에 보지 않은 sha
4. **select_arm:** 두 팔이 모두 가능할 때는 비용 정의와 출처가 하나로 같아야 자동으로 고른다. 아니면 `BOTH_ELIGIBLE_NO_COMPARABLE_COST`(운영자 선택)를 낸다.

## 구현됨 / 아직 연결 안 됨
- **구현됨(후보본, 비활성):**
  - MOUNTED_FLOOR 셋업
  - 탑재 바닥 scene(mjb는 로컬)
  - 바닥 analyse/support_region
  - MOUNTED에서 full_chain 거부
  - 단계표 스크립트 `upper_body/mounted/mounted_candidates.py`
- **연결 안 됨:**
  - 탑재 상태 REAL 집기
  - native UPPER_BODY_PICK_TRANSPORT_PLACE scope(adapter는 아직 NOT_WIRED)
  - 주행 중 운반 자세(진동·파지력·센서 가림)
  - 오른팔 A plan_pick
  - 선반 변형
  - base 팀 합의(topic·clock·정지 증거)
- **현장에서 필요한 것:**
  - 실제 블록 크기·질량(40 mm A 블록이 아닐 경우)
  - 위치 2의 폐기함·영역 형상
