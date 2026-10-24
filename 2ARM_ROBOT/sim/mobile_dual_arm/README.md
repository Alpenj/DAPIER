# Waffle Pi + JDcobot200 양팔 MuJoCo 조합 모델

> **Legacy reference:** 현재 조합은 Waffle Pi + SO-101 두 팔이며
> `sim/mobile_dual_so101`을 사용한다. 이 폴더는 초기 JDcobot 회귀용이다.

공식 TurtleBot3 Waffle Pi base_link 아래에 DAPIER JDcobot200 좌우 팔을 장착한 stationary
manipulation 기준 모델이다.

## 모델 계약

- wheel joint: 2개
- arm과 gripper joint: 14개
- 전체 nq와 nv: 16
- actuator: 양팔 12개
- wheel actuator: 0개
- gripper equality: 2개
- base: world 고정
- 실물 hardware execution 경로: 없음

기본 양팔 장착값은 시각화용 임시값이다.

- x: 0.02 m
- 좌우 중심 간격: 0.18 m
- z: 0.20 m
- 좌우 yaw: 0 rad

이 값은 실제 TurtleBot 상판이나 브래킷을 측정한 값이 아니다. collision 판정과 sim-to-real
학습 전에 반드시 실측값으로 교체한다.

## 실행

    cd ~/DAPIER/2ARM_ROBOT/sim/mobile_dual_arm
    ~/DAPIER/so101_imitation_learning/.venv/bin/python mobile_dual_model.py --smoke-steps 1000
    ~/DAPIER/so101_imitation_learning/.venv/bin/python -m unittest discover -s test -v

화면 확인:

    ~/DAPIER/so101_imitation_learning/.venv/bin/python mobile_dual_model.py --viewer

장착값을 임시로 바꿔 비교할 수 있다.

    ~/DAPIER/so101_imitation_learning/.venv/bin/python mobile_dual_model.py \
      --arm-mount-x-m 0.02 \
      --arm-mount-separation-m 0.18 \
      --arm-mount-height-m 0.20 \
      --viewer

## 다음 단계

1. 상판 기준 두 arm base 중심의 x, y, z와 yaw를 측정한다.
2. 실제 브래킷과 payload 질량을 추가한다.
3. 상판, 팔, 카메라의 primitive collision을 실측 치수로 보정한다.
4. 신발 primitive와 free joint를 추가해 그리퍼 접촉을 검증한다.
5. 이동과 조작을 분리한 상태에서만 wheel actuator 모델을 별도 추가한다.
