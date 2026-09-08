# 손목 카메라·양팔 좌표 보정 준비

record_id: DAPIER-2026-09-08-camera-alignment-preparation

나는 기존 조작 녹화와 모터 보정을 보존하고, 실물 영상과 MuJoCo의 좌표를 맞추는
단계를 준비한다. 이 문서와 보정판 생성은 **보정 완료나 실물 실행 승인**이 아니다.
저장된 데이터의 재생·학습에는 실물 전원이 필요하지 않다.

## 준비한 보정판

기존 OpenCV 4.13의 ChArUco 패턴을 사용한다. A4 가로, 10×7칸, 한 칸 25mm,
marker 18mm, DICT_4X4_50, legacy pattern OFF다. 35개 marker ID와 54개 내부
corner ID를 검출하는 self-test를 실행한다. PDF와 PNG는 254 DPI(10px/mm),
실제 패턴 250×175mm이며 100mm 확인선을 포함한다.

```bash
python 2ARM_ROBOT/scripts/prepare_camera_board.py --self-test
python 2ARM_ROBOT/scripts/prepare_camera_board.py --output /path/to/new-board-folder
```

기존 MuJoCo/LeRobot Python 환경을 사용한다. 파일을 덮어쓰지 않으며 카메라·모터를
열지 않는다. board.json에 패턴 정의와 출력 파일 해시를 기록한다.

출력 시 **실제 크기/100%**로 지정하고 페이지에 맞추기를 끈다. 인쇄 후 확인선이
실제 100mm인지 자로 확인하고 평평한 판에 고정한다. 화면에 표시된 크기나 PDF의
페이지 크기만으로 실제 인쇄 치수가 맞았다고 판단하지 않는다.

## 다음 측정 순서

1. 양손목 RGB의 실제 녹화 해상도·영상 회전·초점 설정을 유지한다. 로봇과 카메라는
   고정하고 보정판을 다양한 위치·거리·기울기로 보여 내부 파라미터와 왜곡을 구한다.
   같은 정면 사진만 반복해서 얻은 작은 오차를 충분한 보정으로 해석하지 않는다.
2. 손목 camera→gripper와 상부 depth camera→좌·우 base 변환을 각각 확인한다.
   손목 외부 보정에는 고정된 기준판과 여러 팔 자세의 동기화된 joint state가 필요하다.
   이미 저장된 leader→follower offset을 모델에 다시 더하지 않는다.
3. 보정판 기준으로 책상 면과 4cm 블록의 초기 pose를 측정해 동일한 좌표계로 옮긴다.
   관절 영점·방향은 알려진 자세에서 확인한다. 파지 성공 수치에 맞춰 보정을 조정하지 않는다.
4. 관절/영상 정합과 접촉을 확인한 뒤 실제 ACT의 새 카메라 관측 → 행동 → 물리 변화 →
   재관측 폐루프를 CPU 병렬 환경에서 평가한다. 그 결과로 GPU 병렬 학습 필요성을 판단한다.

H201은 RGB용 보정판 결과를 depth intrinsics에 대입하지 않는다. 640×460 metric
depth 모드에 맞는 공장 보정과 거리 단위를 먼저 확인하고, rectified 영상에 왜곡 보정을
중복 적용하지 않는다. 실제 장치를 여는 작업은 정확한 장치·명령·조건을 제시하고
현장 승인을 받은 별도 단계에서만 수행한다. 전원 공급은 그 승인 자체가 아니다.

sim-to-real은 카메라·단위·관절 범위·제어 주기 계약, 관측 지연·분실 시 정지,
충돌·속도·명령 제한과 shadow inference를 확인한 뒤 승인된 저속 실물 검증으로 넘긴다.
보정·원본 영상·serial 같은 개인 장치 자료는 공개 저장소에 올리지 않는다.

참고: [OpenCV 4.13 ChArUco calibration](https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html).
