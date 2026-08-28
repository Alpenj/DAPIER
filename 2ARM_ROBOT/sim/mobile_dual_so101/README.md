# Waffle Pi + SO-101 양팔 MuJoCo 모델

JDcobot200 코드를 삭제하지 않고 별도 경로에 만든 SO-101 양팔 기준 모델이다. 팔 하나는
`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`
6개 actuator를 가지며, 좌우를 합친 action은 12차원이다.

이 코드는 MuJoCo 안에서만 동작한다. serial port를 열지 않으며 ROS 2 publisher, 모터
command, hardware dispatch 경로가 없다.

실물 계획에 맞춰 TurtleBot3 LiDAR는 기본적으로 제거한다. 원본 형상과 비교할 때만
`--include-lidar`를 사용한다.

원본 Waffle Pi의 작은 `camera_link`도 제거했다. 전면 중앙에는 AADJA1300GX가
Orbbec Astra 계열로 관측된 사실을 바탕으로 Astra 공식 보수 외형
(165 x 48 x 40 mm), 질량 310 g을 둔다. 정확한 외형과 optical origin을 실측하지
않았으므로 현재 카메라 형상은 확정 CAD가 아니다. 광학 중심은 base 기준
`(0.120, 0, 0.200) m`, 아래쪽 10도로 배치했다.

## 선택형 dual-tower layout

기존 `printed-torso`를 기본값으로 보존하고, 사용자가 제시한 두 개의 독립 세로
스탠드 형상은 `--mount-layout tower`로 선택한다. 현재 provisional 기준은 다음과
같다.

- arm mount: X=-0.060 m, Z=0.380 m, 좌우 간격 0.200 m
- 공통 deck: Waffle 상판 local Z=0.094 m에 직접 접촉
- depth camera: 두 tower 정중앙 `(0.025, 0, 0.450) m`
- camera down tilt: 35도
- 중심 광선의 바닥 교차점: 로봇 전방 약 0.68 m
- tower/crossbar/deck 가정 질량: 1.20 kg

    ~/DAPIER/so101_imitation_learning/.venv/bin/python \
      mobile_dual_so101.py --mount-layout tower \
      --arm-mount-height-m 0.38 --smoke-steps 1000

`--arm-mount-x-m`을 생략하면 tower에는 -0.060 m, 기존 printed torso에는 +0.020 m가
각각 적용된다. 카메라 외함은 두 tower 사이에 좌우 각 5.5 mm의 nominal 여유를 두며,
실제 enclosure와 bracket 공차를 측정하기 전에는 이 값을 제작 치수로 확정하지 않는다.

2026-08-28에 official Waffle mesh와 provisional 질량으로 4,096 endpoint corner 및
무작위 1,500자세를 계산했다. home 자세 COM의 휠-캐스터 지지다각형 여유는 약
+32 mm였지만, 전체 관절 범위에는 무부하에서도 약 -13 mm의 전도 자세가 남았다.
따라서 이 결과는 tower 안전 인증이 아니다. 주행 중에는 낮은 transport pose를 쓰고,
작업 pose 허용영역과 base 정지 interlock은 별도로 제한해야 한다.

제작용 CAD/STL/G-code에 필요한 실측값과 출력 순서는
[`TOWER_FABRICATION.md`](TOWER_FABRICATION.md)에 분리했다.

## 외부 모델 자산

약 16 MB인 SO-101 원본 XML/STL은 이 폴더에 중복 복사하지 않는다. 기본값은 기존
LeRobot 작업공간의 다음 모델을 사용한다.

    ~/DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/
      so101_mujoco/assets/so101_new_calib.xml

다른 checkout에서는 XML과 인접한 `assets/*.stl`을 준비하고 환경변수로 지정한다.

    export DAPIER_SO101_MJCF=/absolute/path/to/so101_new_calib.xml

원본 자산 출처와 해시는
`~/DAPIER/so101/integrations/lerobot_v0_6_so101_mujoco/`에 기록되어 있다. 로더는
파일 존재뿐 아니라 6관절·6 actuator 이름 계약도 검사하고 다르면 중단한다.
실행 보고서는 사용한 XML의 실제 SHA-256과 기록된 upstream SHA-256의 일치 여부를
함께 출력한다. 현재 로컬 source XML은 shoulder 범위 등 실험 수정 이력이 있어 기록된
원본 해시와 일치하지 않으며, 이 코드는 해당 파일을 자동 수정하지 않는다.

## 사람형 어깨 배치

두 SO-101 holder를 Waffle Pi 좌우에서 모두 pitch +90도로 세운다. 그 상태에서
holder의 로컬 X축을 기준으로 왼팔은 -90도, 오른팔은 +90도 twist해 양팔이 몸통
중앙이 아니라 각자의 바깥쪽으로 뻗는 사람의 왼팔·오른팔 형상으로 만든다. 같은 팔 두
개처럼 보이지 않게 좌우를 대칭으로 배치한다. shoulder-pan에 가짜 90도
offset을 넣지 않으며 gripperframe의 접근축은 계속 world -Z를 향한다.

SO-101은 짧으므로 예전에 JDcobot 화면 확인에 쓴 0.55 m 높이를 재사용하면 바닥의
신발에 닿지 않는다. 실제 브래킷을 측정하기 전에는 높이를 확정할 수 없으므로 CLI에서
`--arm-mount-height-m`을 필수로 받는다. 현재 모델에서는 0.30 m일 때 screenshot
pose의 gripperframe이 바닥 위 약 59 mm에 있고 팔의 추가 충돌이 없어 provisional
시각화 기준값으로 사용한다.

## 실행

    cd ~/DAPIER/2ARM_ROBOT/sim/mobile_dual_so101
    ~/DAPIER/so101_imitation_learning/.venv/bin/python \
      mobile_dual_so101.py --arm-mount-height-m 0.30 --smoke-steps 1000

화면 확인:

    ~/DAPIER/so101_imitation_learning/.venv/bin/python \
      mobile_dual_so101.py --arm-mount-height-m 0.30 --viewer

MuJoCo `Control` 슬라이더로 양팔 자세를 직접 잡으려면:

    ~/DAPIER/so101_imitation_learning/.venv/bin/python \
      mobile_dual_so101.py --arm-mount-height-m 0.30 --pose-editor

`--save-pose /tmp/new-pose.json`을 함께 주면 기존 파일은 덮어쓰지 않고 새 JSON에만
저장한다. 이 값은 자동으로 실물에 전송되지 않는다.

## 실물 좌우 매핑

`/dev/ttyACM0` 같은 번호는 재연결 순서에 따라 바뀌므로 사용하지 않는다. 좌우 팔을
하나씩 식별한 뒤 `/dev/serial/by-id/...` 고유 경로를 각각 기록해야 한다. 같은
SO-101 모델이어도 두 팔의 encoder zero/range calibration은 독립적으로 유지한다.

현재 단계에서 확정하지 않은 항목은 실측 mount xyz, 좌우 USB ID 매핑, 각 팔의
calibration, 실물 torque/velocity, 신발 파지 trajectory와 sim-to-real이다.

TurtleBot3 위에 실제로 고정하는 구조 초안과 실측 체크리스트는
[`MOUNTING_CONCEPT.md`](MOUNTING_CONCEPT.md)에 기록했다. 완성된 SO-101 전체를
90도로 돌리면 원래의 바닥 체결면도 수직이 된다. 따라서 수평 선반 위에 얹는 형상은
사용하지 않고, 원본 base를 닫힌 중앙 torsion box의 좌우 수직판에 직접 through-bolt로
체결한다. 8 mm 분할 deck의 하단은 Waffle 상판 local Z=0.094 m에 바로 닿으며 중간
공중 간격을 두지 않는다. 외곽 outrigger/caster를 추가하는 안은 주행성과 제작성이
나빠 채택하지 않았다. 실제 Waffle M3 및 SO-101 base 구멍 좌표는 아직 실측하지
않았으므로 현재 형상은 제작 도면이 아니다.

## 양팔 충돌 가드와 구조 검증

목표 자세만 검사하지 않고 현재 자세에서 목표까지 기본 2도 간격으로 보간해 좌우 팔과
전면 카메라 사이의 최소 거리를 검사한다. 30 mm 미만이면 fail-closed로
거부하며 이 명령에는 하드웨어 전송 경로가 없다.

    ~/DAPIER/so101_imitation_learning/.venv/bin/python collision_guard.py

정역학 표본 검증은 4,096개 관절 끝점 조합과 한 팔/반대 팔/양팔 비대칭 무작위 자세,
0.5 kg nominal 및 1.0 kg proof payload를 계산한다.

    ~/DAPIER/so101_imitation_learning/.venv/bin/python design_validation.py \
      --random-samples 10000 --collision-samples 2000

연속 관절공간의 모든 실수를 완전 열거한 결과가 아니며, 실물 질량·관성·브레이크 거리와
출력물 강성이 측정되기 전에는 제조 안전 인증이나 실물 제어 승인으로 사용하지 않는다.

## 상태 기반 신발 task

SO-101 관절명과 gripper frame을 사용하는 별도 21차원 ground-truth observation
환경이다. 0.30 m 높이와 0.18 m 간격은 중앙 신발이 팔 길이 0.40 m 이내에 들어가는지
검사하기 위한 값일 뿐 실측 브래킷 수치가 아니다.

    ~/DAPIER/so101_imitation_learning/.venv/bin/python shoe_task.py \
      --smoke-steps 200

현재는 primitive 신발, stationary base, state observation까지만 구현했다. 파지 성공
trajectory, RGB/depth perception, domain randomization과 실물 실행은 아직 검증하지
않았다.
