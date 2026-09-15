# DAPIER

DAPIER Physical AI 교육에서 실습한 코드와 개인 로봇 프로젝트를 모아 둔 저장소다.
Python·PyTorch 기초부터 ROS 2, MuJoCo, 모방학습, 실물 로봇 제어까지 다룬다.
현재는 SO-101 양팔과 TurtleBot3를 이용한 물체 조작을 개발하고 있다.

## 주요 폴더

| 폴더 | 내용 |
|---|---|
| [2ARM_ROBOT](2ARM_ROBOT/README.md) | SO-101 양팔 시뮬레이션·학습·실물 연결 |
| [so101](so101/README.md) · [so101_ros2](so101_ros2/README.md) | SO-101 실습, LeRobot 연동, ROS 2 제어 |
| [so101_imitation_learning](so101_imitation_learning/) | PyTorch·모방학습 교육 코드 |
| [deepThinkCar_mini](deepThinkCar_mini/) | 자율주행 자동차 실습 |
| [turtlebot3_ws](turtlebot3_ws/README.md) · [ros_dd_ws](ros_dd_ws/README.md) | 이동로봇 시뮬레이션, SLAM·내비게이션 |
| [jdcobot100_sim](jdcobot100_sim/README.md) · [ros_arm](ros_arm/README.md) | 로봇암 시뮬레이션과 Arduino 제어 |
| [dapier_sim_first](dapier_sim_first/README.md) · [casino_dealer](casino_dealer/README.md) | 한 팔 조작과 카드 딜러 실험 |

## 참고

학습과 개발이 진행 중인 저장소다. 실행 환경과 사용법은 각 폴더의 README를 참고한다.
시뮬레이션 결과와 실물 검증은 구분하며, 실물 장비에 예제 설정을 그대로 적용하지 않는다.

외부 코드·자산의 출처와 라이선스는 해당 폴더에 남겨 두었다.
별도 fork를 이 저장소로 옮긴 내역은 [통합 기록](docs/FORK_MIGRATION_20260907_KO.md)에서 확인할 수 있다.
