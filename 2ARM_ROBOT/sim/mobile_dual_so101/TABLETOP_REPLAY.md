# 책상형 양팔: 실제 녹화의 MuJoCo 재생

record_id: DAPIER-2026-09-07-tabletop-episode-replay

나는 실제 LeRobot-v3 녹화 한 에피소드를 장치 연결 없이 재생한다. 사진 두 장의
책상 고정형 배치를 사용하며 TurtleBot 이동형 배치를 그대로 가져오지 않는다.
카메라 중심선에서 좌우 베이스까지 **수평 0.15 m씩**, 베이스 간격 0.30 m는
사용자가 확인한 값이다. 책상 면을 z=0, 양팔 베이스를 (0,+0.15,0), (0,-0.15,0)로
놓는다. 블록은 사용자가 알려준 한 변 4cm·약 20g의 정육면체다. 질량은 근사값이며,
나머지 치수와 장착 방향, 블록 초기 XY·마찰은 `tabletop_replay.json`의 추정치다.

## 실행

프로젝트 루트에서 기존 MuJoCo/LeRobot Python 환경을 사용한다. 새 패키지를
설치하거나 하드웨어 설정을 읽으러 장치에 접속하지 않는다. 현재 schema v2는
설치된 LeRobot fork의 SO-101 CAD 손가락 접촉면과 손목 카메라 profile을 재사용한다.
확인한 fork commit은 `30da8e687a6dfc617fcd94afc367ac7071c376ce`다.

```bash
/home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/.venv/bin/python \
  2ARM_ROBOT/sim/mobile_dual_so101/replay_recorded_episode.py --self-test

MUJOCO_GL=egl /home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/.venv/bin/python \
  2ARM_ROBOT/sim/mobile_dual_so101/replay_recorded_episode.py \
  --dataset /home/dapier-jhj/DAPIER/2ARM_ROBOT/recording/episodes/20260907 \
  --episode 0 \
  --model /home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml \
  --calibration-dir /home/dapier-jhj/DAPIER/2ARM_ROBOT/recording/config/record-ready-ejyqts0w/calibration \
  --output /home/dapier-jhj/DAPIER/2ARM_ROBOT/recording/simulation/20260907/episode-000-physics-4cm
```

출력 폴더가 이미 있으면 덮어쓰지 않고 거부한다. 다른 에피소드는 `--episode`와
별도의 출력 경로를 지정한다. 현재 지원 입력은 확인된 팔 degree / 그리퍼 0~100
계약이다. 다른 녹화기의 단위를 값 범위만으로 추측하지 않는다.

## 결과의 의미

- 영상 왼쪽: `observation.state`를 qpos로 설정한 **운동학적 재생**이다.
  블록은 초기 위치에 표시되며 원본 영상의 물체 궤적을 재구성한 화면이 아니다.
- 영상 오른쪽: 첫 측정 상태에서 초기화하고 `action[t]`를 실제 `data.ctrl`에
  전달한 **물리 재생**이다. 다음 관측 시점에 `state[t+1]`과 비교하며 중간에
  측정 자세로 다시 덮어쓰지 않는다. 마지막 액션도 1주기 적용한다.
  블록은 free joint를 가진 강체로 중력·접촉에 따라 움직인다. 그리퍼에 붙이는
  weld나 매 프레임 물체 pose를 갱신하는 코드는 없다.
- 기존 12축 순서와 그리퍼 변환을 재사용한다. leader→follower shoulder-pan
  보정은 저장된 action에 이미 반영돼 있으므로 다시 더하지 않는다.
- 15 Hz 녹화는 1/510초 물리 스텝 34개로 정확한 주기를 맞춘다. 녹화 timestamp는
  frame_index/fps이므로 실제 카메라 노출·전송 지연을 재현했다는 뜻은 아니다.
- `report.json`은 원본 dataset 전체와 제공한 calibration JSON의 전후 SHA-256,
  단위 변환 왕복 오차, 관절 RMSE, 모든 물리 substep의 최대 접촉 겹침을 기록한다.
- `joint_trace.npz`는 측정 상태·전송 액션·시뮬레이션 상태를 별도 배열로 저장한다.
  이 파일에서는 **그리퍼도 MuJoCo 관절 rad**이며 학습용 gripper 0~1과 다르다.
  `simulated_block_position_m`은 물리 엔진에서 나온 블록 위치다. 양면 접촉 프레임
  수와 최대 높이는 보고하지만 이것만으로 grasp 성공을 선언하지 않는다.

관절 영점/부호는 현재 기존 모델의 관례를 적용한 후보이며 실측으로 확정하지
않았다. 첫 자세의 접촉 겹침이나 추종 오차를 없애려고 실제 calibration 파일,
기록 데이터, 모델 관절 한계를 바꾸지 않는다. `ready_for_policy_evaluation=false`를
유지한다. 이 재생은 학습 정책의 실행이나 실제 로봇 성공 검증이 아니다.

## 2026-09-08 · 고정 블록 오류 수정과 대조 검사

첫 구현 `a907b09`의 schema v1은 블록을 충돌 없는 고정 참고물로 넣었다. 따라서
실제 녹화에 블록 들기가 담겨 있어도 그 장면에서는 파지가 불가능했다. 녹화 실패나
실물 전원 OFF가 원인이 아니었다. v2에서는 4cm 자유 강체와 접촉을 적용하고,
속이 빈 손가락 mesh의 볼록 껍질 대신 기존 CAD 기반 접촉면을 재사용했다.
양손목 카메라도 모델에 추가했지만 실제 렌즈·장착 보정을 완료한 것은 아니다.

```bash
MUJOCO_GL=egl /home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/.venv/bin/python \
  2ARM_ROBOT/sim/mobile_dual_so101/test_tabletop_physics.py \
  --model /home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml \
  --dataset /home/dapier-jhj/DAPIER/2ARM_ROBOT/recording/episodes/20260907
```

이전 결과의 정정:

- 자유 블록 낙하·책상 지지·weld 없음·양손목 카메라 존재 검사가 통과했다.
- 블록을 **처음부터 손가락 사이에 둔 접촉 fixture**에서는 1,500 step 중 1,491 step에
  양면 접촉을 유지하고 0.051418m 올라갔다는 과거 50g·부드러운 접촉 설정의 기록이 있다.
  그러나 frame 600의 이미 닫힌 집게에 큐브를 넣어 **초기 6.468mm 관통**이 있었다.
  따라서 이 결과와 접촉 제거 시 -0.070683m 하강을 정상 파지의 근거로 사용하지 않는다.
  현재 테스트에서는 이 잘못된 초기 배치를 검출하는 negative 검사로만 남긴다.
- 실제 에피소드 전체를 추정 XY로
  재생한 첫 v2 실행은 양면 접촉이 좌우 모두 0프레임이었다. 첫 자세의 접촉 겹침도
  최대 17.08mm로 남아 있다. 원본 데이터·모터 보정은 수정하지 않았다.

블록 초기 위치, 모델과 실물의 관절/베이스 기준, 손목 카메라 영상 방향·내외부
파라미터를 맞춘 뒤에만 폐루프 성공률을 평가한다. 현재 XY를 그리퍼가 지나가는
위치로 옮겨 성공 수치만 높이거나, 관통을 숨기려고 관절 보정값을 바꾸지 않는다.

## 2026-09-08 · 영상 + IK로 실제 바닥 집기 대조

`vision_tabletop_pick.py`는 학습 정책이 아닌 **SIM 전용 scripted baseline**이다.
가상 metric depth로 중심을 추정하고 5축 IK와 7차 관절 경로를 적용한다.
3mm 옆으로 비켜 내려온 뒤 큐브 높이에서 옆으로 접근해 고정 손가락이 윗면을 치지 않게 했다.
15Hz마다 0.00025rad씩 닫고 양쪽 가상 패드 힘이 각각 1N 이상으로 5주기 유지되면
lift를 허용한다. 최대 270주기 안에 접촉이 잡히지 않으면 lift 명령을 보내지 않는다.
1N은 접촉 확인 기준이며 이동·유지 중 일정한 힘을 제어한다는 뜻은 아니다.

확인한 환경은 MuJoCo 3.8.1, 1/510초, elliptic cone, Newton, tolerance 1e-10,
impratio 1, NoSlip 0이다. 기존 pyramidal 설정에서는 추가 약 3초 유지 중 2.30mm가
미끄러져 2mm 검사에 실패했다. elliptic의 초기 0.5N 설정도 이동 중 놓쳐 실패했다.
1N 기준과 작은 닫기 증분을 함께 적용한 최종 회귀 검사는 다음과 같다.

- 바닥 집기·3초 유지 통과: 최종 중심 67.10mm, 약 47.1mm 상승.
- 접근 단계 패드 정상력 0N, 전체 최대 2.145N, 최대 접촉 겹침 0.0518mm, 경고 0.
- 처음부터 패드 접촉을 끄면 접촉 확인에 실패하고 lift 명령을 보내지 않는다.
- 실제로 잡은 상태를 초기 조건으로 복제해 1,500 step 더 유지하면 양면 접촉 1,500회,
  높이 변화 -0.646mm다. 동일 상태에서 접촉만 끄면 0회, -47.108mm로 떨어진다.
- 모델·scene·제어 코드 해시와 실제 엔진/solver 설정을 결과에 남긴다. 3.3.7에서의
  별도 통과 결과를 3.8.1 결과로 섞지 않는다.

로컬 근거는 `recording/simulation/20260908/tabletop-contact-final/`의
`physics-regression.json`, `contact-on/report.json`, `contact-off/report.json`이다.
회귀 검사 출력 경로는 `--output`으로 지정할 수 있고 기존 결과를 덮어쓰지 않는다.
GUI는 같은 Python에서 `vision_tabletop_pick.py --model <위 MJCF> --output <새 폴더> --viewer`로 연다.
열린 viewer는 코드 변경을 자동 반영하지 않으므로 다시 실행해야 한다.

마찰 계수·접촉 강성은 실측 전이며 현재 geom 설정은 큐브-책상 접촉에도 적용된다.
모터 외함 충돌은 유지하지만 손가락 몸통 전체의 충돌 형상은 아직 완전하지 않다.
15Hz 목표의 sample-and-hold와 들어 올릴 때 약 0.199m/s 순간 속도도 남아 있어
모든 덜컹거림·충돌 안전이 해결됐다고 하지 않는다. 가상 힘 센서는 실물에 연결되지 않는다.
[MuJoCo의 마찰 원뿔·미끄러짐 설명](https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip)을
참고했으며 이 통과는 ACT·양팔 전달·실물 성공이나 물성 보정의 증명이 아니다.

## RGB-D 보정은 다음 단계

관절 재생 자체에는 렌즈 보정이 필수는 아니지만 RGB-D→3D 목표와 visual sim-to-real에는
내부·외부 보정이 필요하다. H201의 현재 depth 모드(640×460)에 맞는 공장 보정값을
먼저 확인하고, rectified 영상에 왜곡 보정을 중복 적용하지 않는다. depth mm 단위와
거리 오차도 실측한다. 현재 녹화에는 top RGB 없이 top depth와 양손목 RGB가 있다.

외부 보정은 상부 카메라→좌/우 베이스, 손목 카메라→각 그리퍼로 나눈다. 15cm 간격과
일반 사진 두 장만으로 높이·회전·렌즈 왜곡을 모두 정밀하게 추정할 수 없다. 알려진
치수의 보정판과 여러 자세의 동기화 관측이 필요하다. 손목 RGB 카메라의 내부 보정도
각각 확인한다. 모터 calibration과 카메라 calibration은 별도 파일로 유지한다.

참고: [OpenCV camera/hand-eye calibration](https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html),
[eYs3D 공식 드라이버](https://github.com/eYs3D/eys3d-ros).
