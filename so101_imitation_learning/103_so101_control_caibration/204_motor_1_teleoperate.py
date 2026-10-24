import json
import time
from motor_control import MiniFeetechDriver


def clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def raw_to_norm_range_m100_100(raw, cal):
    """
    LeRobot RANGE_M100_100에 해당하는 선형 정규화.
    - raw는 Present_Position에서 읽은 값
    - cal: {range_min, range_max, drive_mode}
    """
    mn = cal["range_min"]
    mx = cal["range_max"]
    if mx == mn:
        raise ValueError("range_max == range_min")

    raw_b = clip(raw, mn, mx)
    norm = (((raw_b - mn) / (mx - mn)) * 200.0) - 100.0

    if cal.get("drive_mode", 0):
        norm = -norm
    return norm


def norm_to_raw_range_m100_100(norm, cal):
    """
    LeRobot RANGE_M100_100 역변환.
    """
    mn = cal["range_min"]
    mx = cal["range_max"]
    if mx == mn:
        raise ValueError("range_max == range_min")

    # drive_mode면 부호 반전
    if cal.get("drive_mode", 0):
        norm = -norm

    norm_b = clip(norm, -100.0, 100.0)
    raw = ((norm_b + 100.0) / 200.0) * (mx - mn) + mn
    return int(raw)


def teleop_motor1_shoulder_pan(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    leader_json="shoulder_pan_calibration_leader.json",
    follower_json="shoulder_pan_calibration_follower.json",
    hz=50,
):
    # --- load calibration ---
    leader_cal = json.load(open(leader_json, "r", encoding="utf-8"))["shoulder_pan"]
    follower_cal = json.load(open(follower_json, "r", encoding="utf-8"))["shoulder_pan"]

    leader_id = int(leader_cal["id"])
    follower_id = int(follower_cal["id"])

    leader = MiniFeetechDriver(port=leader_port)
    follower = MiniFeetechDriver(port=follower_port)

    # --- torque setup ---
    # leader: torque off (human moves)
    leader.set_torque(leader_id, False)
    # follower: torque on (robot follows)
    follower.set_torque(follower_id, True)

    dt = 1.0 / hz
    print("\n[Teleop] 시작: leader 모터1을 움직이면 follower 모터1이 따라옵니다. Ctrl+C로 종료\n")

    try:
        while True:
            raw_leader = leader.get_position(leader_id)
            if raw_leader is None:
                continue

            # 1) leader raw -> norm (-100~100)
            norm = raw_to_norm_range_m100_100(raw_leader, leader_cal)

            # 2) norm -> follower raw
            raw_follower_goal = norm_to_raw_range_m100_100(norm, follower_cal)

            # 3) send to follower
            follower.set_position(follower_id, raw_follower_goal)

            # (교육용) 간단 프린트
            print(
                f"leader_raw={raw_leader:4d}  norm={norm:7.2f}  follower_goal_raw={raw_follower_goal:4d}",
                end="\r",
                flush=True,
            )
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n\n[Teleop] 종료")
    finally:
        # 안전: follower torque off
        follower.set_torque(follower_id, False)


if __name__ == "__main__":
    teleop_motor1_shoulder_pan(
        leader_port="/dev/ttyACM0",      # 리더 포트
        follower_port="/dev/ttyACM1",    # 팔로워 포트
        leader_json="shoulder_pan_calibration_leader.json",
        follower_json="shoulder_pan_calibration_follower.json",
        hz=50,
    )
