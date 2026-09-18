# REAL RGB-D → shadow PREGRASP 준비

record_id: DAPIER-2026-09-18-real-sensor-pregrasp-preparation

**REAL READ-ONLY / R2 PARTIAL / R3 BLOCKED — shadow 및 실물 동작 성공 아님.**

## 실제 취득과 장치 정정

사용자는 Astra가 없고 연결된 HP-ASC-H201(3438:0173)이 OS30A라고 확인했다. 앞선 OpenNI2 2.3.0.86 enumeration=0/exit4는 **잘못 선택한 Orbbec SDK 경로의 비검출**이며 OS30A 미연결의 증거로 쓰지 않는다. 카메라 open 전 종료한 첫 실패는 로컬 근거에 보존했다.

사용자 확인 뒤 해당 USB 제품에 속한 camera node를 대조하여 실제 영상을 취득했다. 기존 설치 eYs3D h201_depth_stream helper로 원시 거리도 읽었다. 이 과정은 camera-only이며 모터/serial open·snapshot·torque/register·hardware motion·approval 요청은 없다.

| 관측 | 실제 확인 | 남은 한계 |
|---|---|---|
| camera video | unchanged 1280×460×3 좌우 packed stereo | 두 640×460 crop은 진단용이며 calibrated optical frame 정의 미검증 |
| image channels | 최대 채널 차이2, 거의 grayscale | red 색 segmentation 검증 불가 |
| SDK zdDepthVec | uint16 640×460, 공식 SDK 정의 mm | nonzero47412 pixels, 범위193..16384mm; nonzero는 physical-valid 아님, invalid/saturation 해석 미검증 |
| 취득 timing | video와 depth를 순차 취득 | timestamp-matched pair 및 registration 미검증; 3D target 융합에 사용하지 않음 |

개인 device ID·보정·원시 영상·overlay·local capture adapter는 공개하지 않는다. 실제 top/arm q/shadow PREGRASP는 생성하지 않았다.

## 보정판: 기존 ChArUco 경로를 재사용

실제 영상은 marker가 포함된 ChArUco다. 기존 prepare_camera_board.py의 DICT_4X4_50/10×7 후보 정의로 왼쪽19 markers/14 ChArUco corners, 오른쪽10 markers/6 corners를 검출했다. Block이 board 상부를 가린다. 사용자가 제공한 원본 PDF의 caption은10×7/nominal25mm square/18mm marker다. PDF에서35marker IDs0..34와54corner IDs0..53을 검출해 기존 정의의 layout 대응을 확인했다. 실제 print의 square/XY 치수 검증은 남는다. Block을 치운 새 영상은 left20markers/20corners, right7markers/4corners였다. Corner수가 부족한 오른쪽에서 origin/axes를 외삽한 값은 신뢰 가능한 board frame으로 쓰지 않는다.

사용자가 먼저 실측한 **95mm는 'Measure the line exactly100mm' 기준선**이지 square 길이가 아니다. 이 기준선의 배율95/100=.95로 얻은23.75mm는 당시 균일 print-scale 추정이다. 이후 가로5칸과 세로5칸을 각각120mm로 실측했으므로 최신 square 후보는 **24.0mm**, nominal25mm 대비 XY scale=.96이다. 두 scale 추정은 약1.05% 차이가 나며 측정 오차·flatness는 아직 정량화하지 않았다. 최신 다중칸 실측을 후보 치수로 사용하되 이전 기준선 측정도 별도 revision에 보존한다. 실제 기준선을96mm로 측정했다고 바꾸거나24mm를 검증된 보정 정확도로 주장하지 않는다.

기존 calibrate_camera_images.py/prepare_camera_board.py를 변경 없이 재사용한다. Intrinsics는 unchanged camera configuration의 여러 자세 관측으로 추정하고, single-view board pose와 구분한다. 기존 도구는 최소10개의 usable distinct views를 요구하며 candidate_only와 physical calibration 미검증 상태를 유지한다. 현재 도구의 measured_line_mm 입력은 실제100mm 기준선 측정 전용이다. 최신24mm square를 반영하려고 이를96mm로 입력하는 것은 측정 provenance를 바꾸므로 하지 않는다. 후속 교정에서는 board object points에24mm 후보를 명시하고 동일 left640×460 crop의 설정과 실제 다중칸 measurement revision을 함께 기록해야 한다. 이번 한 장으로 calibration/PnP를 실행하지 않았다.

## camera/board/arm 방향과 최소 helper

OpenCV calibration/solvePnP의 rvec,tvec는 **T_camera_from_board**다. Camera→board는 inverse이고 측정한 board→arm transform과의 chain은 다음이다.

**T_arm_from_camera = T_arm_from_board × inverse(T_camera_from_board)**

[camera_board_transform.py](src/dapier_research/camera_board_transform.py)는 기존 OpenCV pose의 matrix 변환과 이 chain만 제공한다. 별도의 측정 board→arm transform/frame/revision과 caller verification이 없으면 거부한다. Verification flag는 evidence attestation이며 보정을 인증하지 않는다. 결과는 candidate_only, independently_validated=false, authorization=false다.

검증 frame에는 board origin/+X/+Y/+Z와 left-arm-base origin/axes가 필요하다. Board origin은 기존 CharucoBoard 전체 grid의 object-coordinate 원점이다. 첫 inner corner는 nominal(square,square,0)이며 origin과 다르다. 도안 후보의 영상 X/Y와 extrapolated origin은 2D homography로만 표시했고 3D축/PnP로 주장하지 않는다. ChArUco IDs는 board 대응을 제공하지만 board→arm 관계를 대신 측정하지 않는다. Factory RGB/stereo K·distortion/rectification·depth projection·mode/table revision도 물리외부보정과 별개다. Stream 초기화 전 SDK getRectifyLogData(0)는 null, loadstatus0의 getter는 zero였다. 동일 기존 depth stream의 scale-down11bit640×460/15fps/index0 초기화 및1frame 뒤에는 유효 nonzero factory CamMat/P/Q가 반환됐다. 따라서 이전zero를 실제 factory 값으로 해석하지 않는다. Factory log의 packed input/output은2560×920인데 취득video는1280×460이다. 두 축half-scale 대응은 가설이며 crop/pixel-centre/raw-vs-rectified 관계 검증전K를 자동축소/채택하지 않았다. Frame ZDtable buffer는0bytes라 rawtable·invalid-depth해석·RGB/depth정합은 미확정이다. 실제계수·SDKrevision/checksum은 비공개 로컬에 보존한다. 설치 SDK의 mode DB에서 해당 제품은 Hypatia2이고 depth-only640×460/15fps/rectification index0는 mode5에 명시돼 있다. 예제 callback도640×460 depth에 scale-down11bit를 선택한다. 그러나 현재 UVC packed1280×460 YUYV 영상은 DB의 packed2560×920 raw-YUYV 또는 MJPEG rectified 모드와 정확한 대응이 확인되지 않았다. 설치 구현이 binary-only라 resampling/crop/pixel-centre 및 raw CamMat와 rectified P 중 어느 것이 실제 crop에 맞는지 입증하지 못했다. 따라서 factory K를.5배로 축소하거나 이 영상의 metric PnP를 실행하지 않았다.

## 검증 결과

- 기존 vision_target focused8 PASS/.006s (MOCK).
- 기존 camera calibration self-test PASS:20 synthetic views, RMS.358px,95/100 scale가 K를 유지하고 board→camera translation을.95배로 만드는 회귀 및 negative 입력 거부.
- 최소 transform chain focused3 PASS/.028s: 회전/이동/inverse/점 chain, 미측정 revision/frame 거부, invalid transform/pose 거부.
- Fullsuite/CI는 실행하지 않았다. 이 결과는 실물 카메라 정확도나 task 성공의 증명이 아니다.

## 다음 실제 측정

1. Board의 원본 도안과 여러 square 길이/XY scale/flatness를 확인하고, block을 가리지 않는 여러 board view를 수집한다. 동일 camera side/resolution/crop/rotation/focus/raw-or-rectified 설정을 유지한다.
2. Intrinsics 및 factory rectification/depth projection 관계와 mode revision을 확인한다. 이미 rectified 영상에 distortion을 중복 적용하지 않는다. RGB/depth pairing과 registration을 독립 검증한다.
3. Board→left-arm-base 관계를 고정 기준점/실측 board pose로 측정하고 unused points에서 확인한다. 실제 base revision과 CAD의 고정 datum feature를 먼저 대조해야 한다. CAD arm origin을 tabletop/foot-plane/foot-centre/mount-hole-centre로 동일시하지 않는다. 확인한 CAD base origin은 모델 최저면보다2.4mm 위지만 이 관계는 실물 장착면·XY datum·rotation·extrinsic의 실측이 아니다. Shoulder axis/발바닥만으로 in-plane yaw/sign이 정해지지 않으며 SIM offset을 REAL 변환으로 복사하지 않는다. Camera-only board pose만으로 arm transform을 만들지 않는다.
4. 실제 block top을 관측·변환하고 gravity/up와 uncertainty를 확인한 뒤 top 위20mm의 shadow TCP를 만든다. Mask centroid/nominal40mm/SIM pose로 top을 대체하지 않는다.
5. Calibrated 비접촉 orientation·TCP/hand envelope·양팔 mapping·IK/path/collision을 검증한다. Python은 shadow만 만들며 C++ 실물 실행 경계와 총괄의 별도 승인에 남긴다.

현재 R2는 실제 영상/SDK 원시거리 취득과 marker overlay까지이며 registered metric observation은 미완료다. R3 camera→arm은 BLOCKED. Arm q/shadow/실물 동작 미실행.
