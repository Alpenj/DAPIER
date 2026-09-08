# 책상형 양팔: 실제 녹화의 MuJoCo 재생

record_id: DAPIER-2026-09-07-tabletop-episode-replay

나는 실제 LeRobot-v3 녹화 한 에피소드를 장치 연결 없이 재생한다. 사진 두 장의
책상 고정형 배치를 사용하며 TurtleBot 이동형 배치를 그대로 가져오지 않는다.
카메라 중심선에서 좌우 베이스까지 **수평 0.15 m씩**, 베이스 간격 0.30 m는
사용자가 확인한 값이다. 책상 면을 z=0, 양팔 베이스를 (0,+0.15,0), (0,-0.15,0)로
놓는다. 블록은 사용자가 확인한 한 변 4cm의 정육면체다. 나머지 치수와 장착 방향,
블록 초기 XY·무게·마찰은 `tabletop_replay.json`의 추정치다.

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
/home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/.venv/bin/python \
  2ARM_ROBOT/sim/mobile_dual_so101/test_tabletop_physics.py \
  --model /home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml \
  --dataset /home/dapier-jhj/DAPIER/2ARM_ROBOT/recording/episodes/20260907
```

직접 실행한 결과:

- 자유 블록 낙하·책상 지지·weld 없음·양손목 카메라 존재 검사가 통과했다.
- 블록을 **처음부터 손가락 사이에 둔 접촉 fixture**에서는 1,500 step 중 1,491 step에
  양면 접촉을 유지하고 0.051418m 올라갔다. 초기 배치 이후 물체 pose는 쓰지 않았다.
- 같은 초기 조건에서 손가락 접촉만 끄면 양면 접촉은 0 step, 높이 변화는 -0.070683m였다.
  강제 부착으로 만들어진 성공이 아니라는 대조 검사다. MuJoCo 경고도 없었다.
- 이것은 책상에서 집는 정책의 성공 검사가 아니다. 실제 에피소드 전체를 추정 XY로
  재생한 첫 v2 실행은 양면 접촉이 좌우 모두 0프레임이었다. 첫 자세의 접촉 겹침도
  최대 17.08mm로 남아 있다. 원본 데이터·모터 보정은 수정하지 않았다.

블록 초기 위치, 모델과 실물의 관절/베이스 기준, 손목 카메라 영상 방향·내외부
파라미터를 맞춘 뒤에만 폐루프 성공률을 평가한다. 현재 XY를 그리퍼가 지나가는
위치로 옮겨 성공 수치만 높이거나, 관통을 숨기려고 관절 보정값을 바꾸지 않는다.

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
