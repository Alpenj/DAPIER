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

## 사진으로 내부 보정 후보 계산하기 — 장치 접근 없음

나는 먼저 내부 보정으로 픽셀과 카메라 시선의 관계를 구한다. `K`의 초점거리
`fx, fy`와 영상 중심 `cx, cy`, 렌즈 왜곡을 모르면 영상의 점을 올바른 방향으로
해석하기 어렵다. 이것은 **카메라가 로봇 어디에 달렸는지** 구하는 외부 보정과 다르다.
보정판 생성기의 정의를 공유하고 OpenCV의 corner 검출 → ID별 3D/2D 대응 →
`calibrateCameraExtended`를 사용한다. 새로운 라이브러리는 추가하지 않았다.

촬영은 별도 현장 승인 후 진행한다. 팔과 카메라를 고정한 상태에서 보정판만 움직여
카메라마다 20장 이상을 목표로 한다. 화면 중앙·네 모서리, 여러 거리와 기울기를
포함하고 흐림·반사·판의 휘어짐을 피한다. 모든 사진에서 판 전체가 보일 필요는 없지만
이 도구는 최소 12개의 일직선이 아닌 corner를 요구한다. 좌우 사진은 폴더를 나누고
같은 카메라의 해상도·초점·crop·회전·raw/rectified 설정을 유지한다. 현재 녹화의
320×240 모드도 실제 촬영 설정과 대조해야 하며 도구는 자동 resize하지 않는다.
보정 계산에 쓰지 않을 추가 사진도 따로 남겨 후속 검증에 사용한다.

다음은 경로와 촬영 메모를 실제 값으로 바꾼 뒤 실행할 **오프라인 예시**다.
기존 OpenCV 4.13 환경의 Python을 사용한다.

```bash
python 2ARM_ROBOT/scripts/calibrate_camera_images.py --self-test
python 2ARM_ROBOT/scripts/calibrate_camera_images.py \
  --images /path/to/left-wrist-images --camera left_wrist \
  --capture-notes '실제 장치 식별·해상도·초점·raw/rectified·crop·회전 설정을 기록' \
  --output /path/to/new-left-intrinsics-candidate
```

오른쪽은 별도 사진 폴더와 새 출력 폴더, `--camera right_wrist`를 사용한다.
도구는 PNG/JPEG 10~300장을 받아 중복 픽셀·부족한 corner를 제외한 최소 10개
view를 요구하고, 혼합 해상도와 16-bit depth 입력은 거부한다. 입력 해시·view별 오차·
`K`·왜곡을 `intrinsics-candidate.json`에 기록한다. 기존 출력 폴더는 덮어쓰지 않는다.

직접 합성 사진 20장으로 실행한 결과 재투영 RMS는 **0.358px**이고, 알려진
`fx=600, fy=610`에 대해 약 `597.3, 607.4`를 얻었다. 중복·사진 부족·혼합 해상도·
depth 입력 거부와 원본 해시 보존, 기존 출력 거부도 self-test에서 확인했다.
이는 **합성 입력으로 계산 흐름을 검증한 결과**이며 실제 카메라 정확도가 아니다.

RMS 1px 초과, 첫 view 대비 법선 변화 15도 미만, corner 분포의 폭/높이 50% 미만,
초점거리 상대 표준편차 10% 초과 또는 영상 밖 중심점이면 경고와 종료 코드 2를 낸다.
이 기준은 촬영 품질의 휴리스틱이고, 종료 코드 0도 실물 검증 통과를 뜻하지 않는다.
출력은 항상 `candidate_only=true`다. view별 pose는 **board→camera**이며
camera→gripper/base가 아니다. 인쇄 실측·장치 identity·미사용 사진 검증·손목 외부
보정·H201 depth 보정은 아직 남아 있고, 런타임 보정 파일에는 자동 적용하지 않는다.

참고: [OpenCV 4.13 ChArUco calibration](https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html).
