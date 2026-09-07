# 책상형 양팔: 실제 녹화의 MuJoCo 재생

record_id: DAPIER-2026-09-07-tabletop-episode-replay

나는 실제 LeRobot-v3 녹화 한 에피소드를 장치 연결 없이 재생한다. 사진 두 장의
책상 고정형 배치를 사용하며 TurtleBot 이동형 배치를 그대로 가져오지 않는다.
카메라 중심선에서 좌우 베이스까지 **수평 0.15 m씩**, 베이스 간격 0.30 m는
사용자가 확인한 값이다. 책상 면을 z=0, 양팔 베이스를 (0,+0.15,0), (0,-0.15,0)로
놓는다. 나머지 치수와 장착 방향은 `tabletop_replay.json`의 추정치다.

## 실행

프로젝트 루트에서 기존 MuJoCo/LeRobot Python 환경을 사용한다. 새 패키지를
설치하거나 하드웨어 설정을 읽으러 장치에 접속하지 않는다.

```bash
/home/dapier-jhj/DAPIER/so101_imitation_learning/.venv/bin/python \
  2ARM_ROBOT/sim/mobile_dual_so101/replay_recorded_episode.py --self-test

MUJOCO_GL=egl /home/dapier-jhj/DAPIER/so101_imitation_learning/.venv/bin/python \
  2ARM_ROBOT/sim/mobile_dual_so101/replay_recorded_episode.py \
  --dataset /home/dapier-jhj/DAPIER/2ARM_ROBOT/recording/episodes/20260907 \
  --episode 0 \
  --model /home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml \
  --calibration-dir /home/dapier-jhj/DAPIER/2ARM_ROBOT/recording/config/record-ready-ejyqts0w/calibration \
  --output /home/dapier-jhj/DAPIER/2ARM_ROBOT/recording/simulation/20260907/episode-000
```

출력 폴더가 이미 있으면 덮어쓰지 않고 거부한다. 다른 에피소드는 `--episode`와
별도의 출력 경로를 지정한다. 현재 지원 입력은 확인된 팔 degree / 그리퍼 0~100
계약이다. 다른 녹화기의 단위를 값 범위만으로 추측하지 않는다.

## 결과의 의미

- 영상 왼쪽: `observation.state`를 qpos로 설정한 **운동학적 재생**이다.
- 영상 오른쪽: 첫 측정 상태에서 초기화하고 `action[t]`를 실제 `data.ctrl`에
  전달한 **물리 재생**이다. 다음 관측 시점에 `state[t+1]`과 비교하며 중간에
  측정 자세로 다시 덮어쓰지 않는다. 마지막 액션도 1주기 적용한다.
- 기존 12축 순서와 그리퍼 변환을 재사용한다. leader→follower shoulder-pan
  보정은 저장된 action에 이미 반영돼 있으므로 다시 더하지 않는다.
- 15 Hz 녹화는 1/510초 물리 스텝 34개로 정확한 주기를 맞춘다. 녹화 timestamp는
  frame_index/fps이므로 실제 카메라 노출·전송 지연을 재현했다는 뜻은 아니다.
- `report.json`은 원본 dataset 전체와 제공한 calibration JSON의 전후 SHA-256,
  단위 변환 왕복 오차, 관절 RMSE, 모든 물리 substep의 최대 접촉 겹침을 기록한다.
- `joint_trace.npz`는 측정 상태·전송 액션·시뮬레이션 상태를 별도 배열로 저장한다.
  이 파일에서는 **그리퍼도 MuJoCo 관절 rad**이며 학습용 gripper 0~1과 다르다.

관절 영점/부호는 현재 기존 모델의 관례를 적용한 후보이며 실측으로 확정하지
않았다. 첫 자세의 접촉 겹침이나 추종 오차를 없애려고 실제 calibration 파일,
기록 데이터, 모델 관절 한계를 바꾸지 않는다. `ready_for_policy_evaluation=false`를
유지한다. 빨간 블록은 사진의 위치를 나타내는 정적 시각 참고물로, 물체 궤적이나
잡기 성공을 재구성하지 않는다. 이 재생은 학습이나 실제 로봇 성공 검증이 아니다.

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
