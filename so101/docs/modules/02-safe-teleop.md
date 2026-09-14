# 모듈 학습 02: dapier_so101_teleop

## 이 모듈에서 배우려는 것

리더의 JointState를 팔로워 명령으로 그대로 복사하는 것만으로는 안전한
teleoperation이 되지 않는다. 언제 명령을 시작할 수 있고, 입력이 끊기면
무엇을 해야 하며, 첫 명령이 얼마나 변할 수 있는지를 별도로 정의해야 한다.

## 먼저 읽을 파일

1. [safe_teleop.yaml](../../dapier_so101_teleop/config/safe_teleop.yaml)
2. [safe_leader_follower.cpp](../../dapier_so101_teleop/src/safe_leader_follower.cpp)
3. [safe_teleop.launch.py](../../dapier_so101_teleop/launch/safe_teleop.launch.py)

## 입력과 출력

입력:

- /leader/joint_states
- /follower/joint_states
- /dapier_so101/teleop/enable 서비스

출력:

- /follower/trajectory_controller/joint_trajectory
- /dapier_so101/teleop/enabled

이 모듈은 시리얼 포트를 열지 않고 torque를 변경하지 않는다.

## 시작을 자동으로 하지 않는 이유

노드가 실행된 순간 리더와 팔로워 자세가 다르면 첫 command가 큰 이동을 만들
수 있다. 그래서 다음 조건을 모두 만족할 때만 enable 요청을 받아들인다.

1. leader state를 수신했다.
2. follower state를 수신했다.
3. 두 state가 stale_timeout 안에 들어온 최신 값이다.
4. 두 state가 hard limit를 크게 벗어나지 않는다.
5. 가장 큰 관절 차이가 max_start_error_rad 이하이다.
6. 운영자가 SetBool service에 true를 보냈다.

enable 시 last_command를 리더가 아니라 현재 follower state로 시작한다. 첫
제어 주기부터 core의 max_velocity 제한을 적용하기 위해서다.

## 시간은 왜 steady clock으로 확인하는가

message header 시간은 발행자 설정이나 simulation time에 따라 멈추거나 점프할
수 있다. 이 노드가 알고 싶은 것은 로컬 프로세스가 마지막 메시지를 받은 뒤
실제로 얼마나 지났는가다. 그래서 freshness에는 steady clock을 사용한다.

## QoS 선택

JointState는 SensorDataQoS로 구독한다. 모든 과거 샘플을 처리하는 것보다 가장
최근 상태를 계속 받는 것이 중요하기 때문이다. 명령과 enabled 상태는 reliable
QoS를 사용한다. enabled 상태는 transient local로 발행해 늦게 들어온 관찰자도
현재 상태를 받을 수 있게 했다.

## disable의 정확한 의미

현재 disable은 새 trajectory를 더 이상 발행하지 않는다는 뜻이다. 실제
컨트롤러가 마지막 위치를 유지할 수 있으므로 이것은 물리적 E-stop이나 torque
OFF와 같지 않다. hardware 계층에서 watchdog과 torque OFF를 구현하기 전에는
이를 비상 정지라고 부르면 안 된다.

## 모의 검증에서 확인한 것

- state 없이 enable 요청 → success false
- leader 0.10 rad, follower 0.00 rad → 허용 오차 내에서 enable 성공
- 고정된 여섯 관절 순서로 JointTrajectory 발행
- time_from_start 0.04초 확인
- disable 요청 → enabled false 및 정상 종료

실제 controller와 모터는 이 검증에 포함되지 않았다.

## 직접 해볼 실습

1. 두 가짜 JointState 차이를 0.40 rad로 만들어 enable이 거부되는지 본다.
2. enable 후 leader publisher만 끄고 0.25초 뒤 enabled가 false가 되는지 본다.
3. leader JointState의 관절 배열 순서를 섞어도 출력 순서가 고정되는지 본다.
4. 한 관절 이름을 빼서 invalid joint state 경고를 확인한다.
5. publish_rate_hz와 max_velocity를 이용해 1초 동안 가능한 최대 이동을
   계산하고 topic echo 결과와 비교한다.

## 다음 구현에서 확인할 것

- controller가 disable 후 마지막 위치를 어떻게 유지하는지
- hardware watchdog의 timeout을 teleop timeout과 어떻게 계층화할지
- 서비스 enable 외에 물리 스위치나 dead-man input을 어떻게 넣을지
- 양팔에서 좌우 각각의 상태와 enable을 분리할지
