---
title: "한 번에 실행하는 대신, SO-101 ROS 2 제어 경계를 직접 나누기 시작했다"
description: "LeRobot 원클릭 실행을 비교 기준으로 남기고, 관절 계약과 안전 텔레옵을 직접 구현한 첫 기록."
date: 2026-08-04
status: draft
series: Physical AI Lab
entry: 002
tags: [SO-101, ROS 2, ros2_control, teleoperation, learning-log]
---

# 한 번에 실행하는 대신, SO-101 ROS 2 제어 경계를 직접 나누기 시작했다

이 글은 완성된 로봇 시스템 소개가 아니다. SO-101 장비를 교체하기로 한
상태에서 실제 모터 쓰기는 멈추고, 하드웨어 없이 확인할 수 있는 관절 계약과
안전 텔레옵부터 분리해 본 기록이다.

## 01. 동작시키는 것과 이해하는 것은 같은 일이 아니었다

LeRobot 명령을 사용하면 calibration부터 teleoperation과 recording까지 빠르게
연결할 수 있다. baseline을 확인할 때는 큰 장점이다. 하지만 내 학습 목표에서
명령 한 번으로 전체가 실행되는 흐름은 관절 순서, 단위 변환, 포트 소유권,
명령 제한이 어디에서 결정되는지 놓치기 쉬웠다.

그래서 LeRobot을 없애는 것이 아니라 역할을 줄이기로 했다. 실시간 로봇 실행은
ROS 2 모듈로 직접 구성하고, 데이터셋 변환과 정책 학습에서만 LeRobot을 다시
연결하는 구조다.

> 이번 단계의 목표는 팔을 움직이는 데 성공했다고 쓰는 것이 아니라, 팔을
> 움직이기 전에 어떤 조건을 코드로 확인해야 하는지 고정하는 것이다.

## 02. 첫 경계는 공통 계약과 ROS 입출력이다

현재 나눈 흐름은 다음과 같다.

~~~text
joint contract / calibration / limits
                 ↓
leader JointState → safe teleop → follower JointTrajectory
                 ↓
          future hardware driver
                 ↓
             SO-101 bus
~~~

dapier_so101_core에는 여섯 관절 이름과 순서, motor ID, radian limit, 최대
속도를 넣었다. 이 코드는 ROS 노드가 아니어서 실제 팔이나 ROS daemon 없이
테스트할 수 있다.

dapier_so101_teleop에는 JointState 수신, 최신 데이터 확인, 명시적 enable,
trajectory 발행을 넣었다. 모터 register와 calibration EEPROM은 다루지 않는다.
앞으로 만들 hardware driver만 시리얼 포트를 소유하게 하기 위해서다.

## 03. 모듈을 나눈 이유는 코드 수보다 실패 위치를 보기 위해서다

첫째, 관절 배열 순서를 메시지가 들어온 순서 그대로 믿지 않는다. name을
기준으로 다시 정렬해 항상 shoulder_pan부터 gripper까지 같은 순서를 만든다.

둘째, calibration 예제 파일은 verified: false다. 장비별로 측정하지 않은 값을
실수로 실제 팔에 쓰지 못하도록 기본 loader가 거부한다.

셋째, teleop 노드는 실행 직후 명령을 보내지 않는다. 리더와 팔로워 상태가
모두 최신이고, 두 자세가 충분히 가깝고, 운영자가 enable을 요청해야만 명령을
발행한다.

넷째, 첫 command는 리더 자세가 아니라 현재 팔로워 자세에서 시작한다. 이후
한 주기에 이동할 수 있는 양을 max_velocity × dt로 제한한다.

이 분리는 코드를 멋있게 보이게 하기 위한 것이 아니라 잘못된 관절 순서,
오래된 state, 큰 첫 점프, 검증되지 않은 calibration을 서로 다른 위치에서
찾기 위한 것이다.

## 04. 오늘 실제로 확인한 범위

완료로 표시할 수 있는 것은 다음뿐이다.

- ROS 2 Jazzy에서 core와 teleop 두 패키지 빌드 성공
- 관절 계약과 calibration 변환 GTest 7개 통과
- state가 없을 때 enable 요청 거부
- 합성한 leader/follower JointState가 정렬됐을 때 enable 성공
- 고정된 관절 순서의 JointTrajectory 발행
- disable 요청과 노드 정상 종료

실제 모터, 실제 ros2_control controller, torque, leader-follower 움직임은
검증하지 않았다. 특히 현재 disable은 새 명령을 멈추는 것이지 물리적인
torque OFF가 아니다.

## 05. 다음 장비에서 확인할 순서

1. motor ID 1~6을 torque OFF 상태로 반복해서 읽는다.
2. 장치 serial ID와 calibration 파일을 묶는다.
3. raw tick과 ROS radian 방향을 관절별로 비교한다.
4. ros2_control configure/read까지만 검증한다.
5. 현재 위치로 command를 초기화하고 제한된 write를 연다.
6. watchdog과 torque OFF를 확인한 뒤 저속 teleop을 시작한다.
7. ROS topic을 rosbag2/MCAP episode로 기록한다.
8. 파생 데이터만 LeRobotDataset으로 변환한다.

## 기록 경계

이번 글의 구현은 DAPIER의 so101_ros2 디렉터리에 남긴다. 기존
legalaspro/so101-ros-physical-ai는 구조를 비교한 참고 프로젝트이며, 이번
모듈을 그 저장소에 기여했다고 쓰지 않는다. 실제 장비에서 확인하지 않은
내용도 완료 결과로 쓰지 않는다.

## 참고

- [ROS 2 Control: Writing a Hardware Component](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
- [ROS 2 Control: Controller Manager](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html)
- [ROS 2 rosbag2](https://github.com/ros2/rosbag2)
- [Hugging Face LeRobotDataset v3](https://huggingface.co/docs/lerobot/main/lerobot-dataset-v3)
- [DAPIER SO-101 ADR 0001](../adr/0001-own-ros2-runtime.md)
