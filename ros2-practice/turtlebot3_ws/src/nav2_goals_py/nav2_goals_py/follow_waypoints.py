#!/usr/bin/env python3
"""
여러 Waypoint를 순서대로 이동하는 TurtleBot3 제어 예제
Nav2 SimpleCommander의 followWaypoints()를 사용합니다.
"""

import math
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


def create_pose_stamped(navigator: BasicNavigator,
                        x: float, y: float, yaw_deg: float) -> PoseStamped:
    """(x, y, yaw_도) → PoseStamped 변환 헬퍼"""
    yaw_rad = math.radians(yaw_deg)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Nav2 활성화 대기
    navigator.waitUntilNav2Active()
    print("[INFO] Nav2 활성화 완료. Waypoint 순차 이동을 시작합니다.")

    # -------------------------------------------------------
    # Waypoint 목록 정의
    # (x좌표, y좌표, 방향각_도) 형식으로 원하는 지점을 추가하세요.
    # 지도 좌표는 map.yaml의 origin을 기준으로 한 미터 단위입니다.
    # -------------------------------------------------------
    waypoint_list = [
        (0.5,  0.5,   0.0),   # 지점 1: (0.5, 0.5), 동쪽
        (2.0,  0.5,  90.0),   # 지점 2: (2.0, 0.5), 북쪽
        (2.0,  2.0, 180.0),   # 지점 3: (2.0, 2.0), 서쪽
        (0.5,  2.0, 270.0),   # 지점 4: (0.5, 2.0), 남쪽
        (0.0,  0.0,   0.0),   # 지점 5: 원점으로 복귀
    ]

    # PoseStamped 리스트 생성
    waypoints = []
    for idx, (x, y, yaw) in enumerate(waypoint_list):
        pose = create_pose_stamped(navigator, x, y, yaw)
        waypoints.append(pose)
        print(f"  Waypoint {idx + 1}: x={x}, y={y}, yaw={yaw}°")

    # Waypoint 순차 이동 명령
    navigator.followWaypoints(waypoints)

    # 완료 대기 및 피드백 출력
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            current_wp = feedback.current_waypoint
            print(f"[FEEDBACK] 현재 이동 중인 Waypoint: {current_wp + 1}/{len(waypoints)}")

    # 결과 확인
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("[SUCCESS] 모든 Waypoint 방문 완료!")
    elif result == TaskResult.CANCELED:
        print("[WARN] Waypoint 이동이 취소되었습니다.")
    elif result == TaskResult.FAILED:
        print("[ERROR] Waypoint 이동 중 오류가 발생했습니다.")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
