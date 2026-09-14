# 모듈 학습 01: dapier_so101_core

## 이 모듈에서 배우려는 것

로봇 코드에서 가장 먼저 고정해야 하는 것은 알고리즘보다 데이터 계약이다.
SO-101의 여섯 관절 이름, 순서, motor ID, 단위, 허용 범위를 한곳에서 정의하고
다른 모듈이 이를 공유하도록 만든다.

이 패키지는 rclcpp에 의존하지 않는다. 따라서 ROS graph와 하드웨어가 없는
상태에서도 일반 C++ 테스트로 계산을 검증할 수 있다.

## 먼저 읽을 파일

1. [so101_joint_contract.yaml](../../dapier_so101_core/config/so101_joint_contract.yaml)
2. [joint_model.hpp](../../dapier_so101_core/include/dapier_so101_core/joint_model.hpp)
3. [joint_model.cpp](../../dapier_so101_core/src/joint_model.cpp)
4. [test_joint_model.cpp](../../dapier_so101_core/test/test_joint_model.cpp)

## 핵심 타입

### JointSpec

관절 하나의 이름, motor ID, 최소·최대 위치, 최대 속도를 가진다. 위치는 radian,
속도는 radian/second다. 이 값은 향후 URDF와 hardware interface가 공유해야
한다.

### JointModel

다음 불변조건을 검사한다.

- 관절 이름은 중복될 수 없다.
- motor ID는 1~253이며 중복될 수 없다.
- 모든 실수 값은 finite여야 한다.
- lower limit는 upper limit보다 작아야 한다.
- max velocity는 0보다 커야 한다.

reorder 함수는 JointState의 배열 순서를 믿지 않고 이름으로 다시 정렬한다.
ROS 메시지에서 name과 position은 같은 인덱스를 공유하지만 발행자마다 배열
순서는 달라질 수 있기 때문이다.

limit 함수는 두 단계를 수행한다.

1. 목표 위치를 hard limit 안으로 clamp한다.
2. 이전 명령에서 한 주기 동안 이동 가능한 거리로 다시 제한한다.

계산은 다음과 같다.

~~~text
max_delta = max_velocity × dt
command = clamp(target, previous - max_delta, previous + max_delta)
~~~

예를 들어 최대 속도가 0.5 rad/s이고 주기가 0.02초라면 한 번의 명령에서
움직일 수 있는 최대 변화는 0.01 rad다.

### CalibrationEntry

raw tick 범위와 ROS 위치 범위 사이를 선형 변환한다.

~~~text
ratio = (raw - raw_min) / (raw_max - raw_min)
position = position_min + ratio × (position_max - position_min)
~~~

inverted가 true면 ratio를 1 - ratio로 뒤집는다. 범위 밖 입력은 먼저 clamp한다.

homing_offset은 장치 설정을 기록하기 위한 값이다. 현재 변환은 모터가 보고한
raw_min~raw_max를 기준으로 하며 EEPROM 쓰기는 수행하지 않는다.

### CalibrationSet의 verified

calibration.example.yaml은 verified: false로 배포된다. 기본 loader는 이 파일을
거부한다. 예제 값을 실수로 실제 하드웨어에 사용하는 일을 막기 위한 작은
fail-closed 장치다.

## 테스트가 증명하는 범위

- 여섯 관절 계약을 YAML에서 읽을 수 있다.
- 중복 이름과 ID를 거부한다.
- 섞인 JointState 순서를 올바르게 복원한다.
- 누락된 관절과 NaN을 거부한다.
- 위치와 속도 제한을 동시에 적용한다.
- calibration을 양방향으로 변환한다.
- 검증되지 않은 calibration 파일을 기본적으로 거부한다.

테스트는 패킷 통신, 모터 방향, 실제 관절 각도를 증명하지 않는다.

## 직접 해볼 실습

1. test_joint_model.cpp의 target을 바꾸고 max_delta를 손으로 계산한다.
2. 한 관절의 이름을 중복시켜 어떤 예외가 나오는지 확인한다.
3. calibration.example.yaml에서 inverted를 true로 바꾸고 양 끝의 결과를
   종이에 계산한다. 단, verified는 false로 유지한다.
4. max_velocity를 절반으로 바꾼 뒤 테스트의 예상값을 수정해 통과시킨다.
5. infinity 입력을 추가하여 왜 finite 검사가 필요한지 확인한다.

## 다음 질문

- URDF limit와 실제 calibration endpoint가 다를 때 어느 값이 hard limit가
  되어야 하는가?
- gripper를 radian으로 표현할지 0~1 비율로 표현할지 어디에서 변환할 것인가?
- calibration 파일을 장치 serial ID와 어떻게 묶어 다른 팔의 파일 사용을
  막을 것인가?
