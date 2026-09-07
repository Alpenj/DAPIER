import serial
import time
import json
from pathlib import Path
from motor_control import MiniFeetechDriver



def calibrate_single_motor_shoulder_pan(
    port="/dev/ttyUSB0",
    motor_id=1,
    out_json_path="./shoulder_pan_calibration_leader.json",
):
    """
    LeRobot 스타일을 교육용으로 단순화한 단일 모터 캘리브레이션.
    - homing_offset: 중앙 자세에서 pos를 half-turn(2047)로 맞추도록 offset 계산
    - range_min/max: 양 끝에 가져다 놓고 Enter로 기록(자동 스트리밍 대신 단순화)
    """
    driver = MiniFeetechDriver(port=port)

    # STS3215가 0~4095라고 가정(12bit)
    MAX_RES = 4095
    HALF_TURN = MAX_RES // 2  # 2047

    print("\n[0] 토크 OFF (손으로 관절을 움직이기 위해)")
    driver.set_torque(motor_id, False)
    time.sleep(0.1)

    # [중요] 이전 설정값이 영향을 주지 않도록 오프셋과 리미트를 초기화
    print("  - 이전 설정 초기화 중...")
    driver.set_homing_offset(motor_id, 0)
    driver.set_position_limits(motor_id, 0, 4095)
    time.sleep(0.2) # 모터 내부 반영 시간 대기

    # ---- Step 1: homing offset ----
    input("\n[1] shoulder_pan(모터 1)을 가능한 '중앙' 위치에 두고 ENTER")
    pos_center = driver.get_position(motor_id)
    if pos_center is None:
        raise RuntimeError("현재 위치를 읽지 못했습니다.")

    # LeRobot(Feetech) 개념: homing_offset = current_pos - HALF_TURN
    homing_offset = int(pos_center) - int(HALF_TURN)

    print(f"  - 현재 pos = {pos_center}")
    print(f"  - HALF_TURN = {HALF_TURN}")
    print(f"  - 계산된 homing_offset = {homing_offset}")

    print("\n  -> 모터에 Homing_Offset 기록")
    driver.set_homing_offset(motor_id, homing_offset)
    time.sleep(0.05)

    # ---- Step 2: range min/max ----
    input("\n[2] shoulder_pan을 '왼쪽 끝(최소)'까지 천천히 돌리고 ENTER")
    pos_min = driver.get_position(motor_id)
    if pos_min is None:
        raise RuntimeError("최소 위치를 읽지 못했습니다.")

    input("\n[3] shoulder_pan을 '오른쪽 끝(최대)'까지 천천히 돌리고 ENTER")
    pos_max = driver.get_position(motor_id)
    if pos_max is None:
        raise RuntimeError("최대 위치를 읽지 못했습니다.")

    # 안전하게 min/max 정렬
    range_min = int(min(pos_min, pos_max))
    range_max = int(max(pos_min, pos_max))

    print(f"\n  - 기록된 raw min = {range_min}")
    print(f"  - 기록된 raw max = {range_max}")

    if range_min == range_max:
        raise ValueError("min과 max가 같습니다. 관절을 충분히 움직였는지 확인하세요.")

    print("\n  -> 모터에 Min/Max_Position_Limit 기록")
    driver.set_position_limits(motor_id, range_min, range_max)

    # ---- Save JSON ----
    calib = {
        "shoulder_pan": {
            "id": motor_id,
            "drive_mode": 0,
            "homing_offset": homing_offset,
            "range_min": range_min,
            "range_max": range_max,
        }
    }

    Path(out_json_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_json_path, "w", encoding="utf-8") as f:
        json.dump(calib, f, indent=4, ensure_ascii=False)

    print(f"\n[완료] 캘리브레이션 JSON 저장: {out_json_path}")
    print("[팁] 다음 단계: 이 값을 이용해 raw↔정규화 매핑(-100~100 또는 degrees)을 계산할 수 있습니다.")

    return calib


if __name__ == "__main__":
    calibrate_single_motor_shoulder_pan(
        port="/dev/ttyUSB0",
        motor_id=1,
        out_json_path="./shoulder_pan_calibration_leader.json",
    )
