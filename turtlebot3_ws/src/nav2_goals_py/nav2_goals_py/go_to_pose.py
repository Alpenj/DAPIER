#!/usr/bin/env python3
"""
단일 목표 지점으로 TurtleBot3를 이동시키는 예제
Nav2 SimpleCommander API를 사용합니다.
"""

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
# NOTE: tf_transformations is unused below (quaternion math is done by hand
# with math.sin/cos) and its import currently crashes on this machine due to
# a transforms3d / numpy 2.0 incompatibility, so it's omitted here.


def create_pose_stamped(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    """
    지도 좌표 (x, y, yaw_도)를 PoseStamped 메시지로 변환하는 헬퍼 함수.
    yaw는 도(degree) 단위로 입력받아 내부적으로 쿼터니언으로 변환합니다.
    """
    import math
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    # yaw(도)를 쿼터니언으로 변환
    yaw_rad = math.radians(yaw)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

    return pose


def main():
    rclpy.init()

    # BasicNavigator 인스턴스 생성
    navigator = BasicNavigator()

    # Nav2가 완전히 활성화될 때까지 대기
    navigator.waitUntilNav2Active()
    print("[INFO] Nav2 활성화 완료.")

    # 목표 지점 설정: x=2.0, y=1.0, 방향=90도(북쪽)
    goal_pose = create_pose_stamped(navigator, x=2.0, y=1.0, yaw=90.0)

    print(f"[INFO] 목표 지점으로 이동 시작: x=2.0, y=1.0, yaw=90도")
    navigator.goToPose(goal_pose)

    # 이동 완료를 기다리며 진행 상태를 주기적으로 출력
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            distance = feedback.distance_remaining
            print(f"[FEEDBACK] 남은 거리: {distance:.2f} m")

    # 결과 확인
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("[SUCCESS] 목표 지점에 도달했습니다!")
    elif result == TaskResult.CANCELED:
        print("[WARN] 이동 태스크가 취소되었습니다.")
    elif result == TaskResult.FAILED:
        print("[ERROR] 목표 지점 이동에 실패했습니다.")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
