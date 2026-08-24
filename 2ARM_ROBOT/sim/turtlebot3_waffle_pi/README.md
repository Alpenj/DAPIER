# TurtleBot3 Waffle Pi MuJoCo 기준 모델

ROBOTIS 공식 Jazzy turtlebot3_description의 Waffle Pi URDF와 필요한 mesh를 DAPIER에서
재현 가능한 형태로 보존한다. 원본은 upstream 아래에 그대로 두고, MuJoCo 입력용 파생 URDF만
상위 폴더에 분리한다.

## 현재 범위

- base_footprint와 base_link를 보존한다.
- 좌우 wheel joint 2개를 보존한다.
- Waffle Pi base, 좌우 tire, LDS visual mesh를 포함한다.
- base collision, wheel collision, caster collision을 포함한다.
- camera_link와 RGB optical frame을 보존한다.
- 바퀴 actuator는 추가하지 않는다.
- base는 world에 고정한 stationary manipulation 기준이다.

MuJoCo 변환본은 원본 xacro를 확장한 뒤 두 가지만 변경했다.

1. package URI를 이 폴더에서 재현 가능한 상대 mesh 경로로 변경했다.
2. fusestatic=false를 지정해 base_link와 sensor frame을 보존했다.

## 검증

    cd ~/DAPIER/2ARM_ROBOT/sim/turtlebot3_waffle_pi
    ~/DAPIER/so101_imitation_learning/.venv/bin/python waffle_pi_model.py --smoke-steps 1000
    ~/DAPIER/so101_imitation_learning/.venv/bin/python -m unittest discover -s test -v

화면 확인:

    ~/DAPIER/so101_imitation_learning/.venv/bin/python waffle_pi_model.py --viewer

현재 모델에는 wheel actuator나 ROS 2 publisher가 없으므로 이 명령으로 실물 TurtleBot이
움직이지 않는다.

## 아직 포함하지 않은 것

- 실제 이동 베이스용 wheel actuator와 differential-drive control
- 배터리, 양팔 브래킷, 전원 분배 장치의 질량과 관성
- AADJA1300GX depth camera의 실제 외형과 optical transform
- 바닥 마찰과 wheel slip 실측 파라미터

원본과 라이선스 정보는 THIRD_PARTY_NOTICE.md를 본다.
