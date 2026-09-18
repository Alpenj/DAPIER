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

사용자가 먼저 실측한 **95mm는 'Measure the line exactly100mm' 기준선**이지 square 길이가 아니다. 이 기준선의 배율95/100=.95로 얻은23.75mm는 당시 균일 print-scale 추정이다. 이후 가로5칸과 세로5칸을 각각120mm로 실측했으므로 최신 사용자 확정 square는 **24.0mm**, nominal25mm 대비 XY scale=.96이다. 두 scale 추정은 약1.05% 차이가 나며 측정 오차·flatness는 아직 정량화하지 않았다. 최신 다중칸 실측을 후보 치수로 사용하되 이전 기준선 측정도 별도 revision에 보존한다. 실제 기준선을96mm로 측정했다고 바꾸거나24mm를 검증된 보정 정확도로 주장하지 않는다.

기존 prepare_camera_board.py의 nominal layout은 유지하고 calibrate_camera_images.py에 실제 square 간격 입력만 추가했다. Intrinsics는 unchanged camera configuration의 여러 자세 관측으로 추정하고, single-view board pose와 구분한다. 기존 도구는 최소10개의 usable distinct views를 요구하며 candidate_only와 physical calibration 미검증 상태를 유지한다. measured_line_mm은 실제100mm 기준선 측정 전용으로 유지한다. 새 --square-length-mm24.0와 --measurement-revision 경로는 현재 square 간격을 직접 기록하며 두 입력은 동시에 허용하지 않는다. 이를 반영하려고 기준선을96mm로 입력하거나 두 측정치를 평균내지 않는다. 동일 left640×460 crop의 여러 영상에서 empirical K/distortion을 추정한다. Raw/rectified 모드 명칭이 미확정이어도 정확히 같은 capture mapping에서 fitted distortion을 한 번만 적용하고 별도 view에서 재투영을 확인할 수 있다. Factory K나 depth projection과의 호환을 뜻하지 않는다. 앞선 한 장으로 calibration/PnP를 실행하지 않았다. 현재 camera-only GUI는 SPACE로 training view, H로 별도 holdout view를 저장하며12개 이상 non-collinear corners와 영상평면 자세 차이를 확인한다. Repeated/유사 자세는 rejected 이미지와 이유를 따로 보존한다. 이 diversity 기준은 acquisition heuristic이며 각도·물리 정확도 증명이 아니다. 모터·깊이 취득·camera 설정 쓰기 없이 unchanged packed1280×460의 left640×460 crop을 사용한다. Camera 설정을 쓰지 않았다는 사실은 focus가 고정됐다는 증명이 아니므로 focus/autofocus 상태는 별도 한계로 남긴다.

## camera/board/arm 방향과 최소 helper

OpenCV calibration/solvePnP의 rvec,tvec는 **T_camera_from_board**다. Camera→board는 inverse이고 측정한 board→arm transform과의 chain은 다음이다.

**T_arm_from_camera = T_arm_from_board × inverse(T_camera_from_board)**

[camera_board_transform.py](src/dapier_research/camera_board_transform.py)는 기존 OpenCV pose의 matrix 변환과 이 chain만 제공한다. 별도의 측정 board→arm transform/frame/revision과 caller verification이 없으면 거부한다. Verification flag는 evidence attestation이며 보정을 인증하지 않는다. 결과는 candidate_only, independently_validated=false, authorization=false다.

검증 frame에는 board origin/+X/+Y/+Z와 left-arm-base origin/axes가 필요하다. Board origin은 기존 CharucoBoard 전체 grid의 object-coordinate 원점이다. 첫 inner corner는 nominal(square,square,0)이며 origin과 다르다. 도안 후보의 영상 X/Y와 extrapolated origin은 2D homography로만 표시했고 3D축/PnP로 주장하지 않는다. ChArUco IDs는 board 대응을 제공하지만 board→arm 관계를 대신 측정하지 않는다. Factory RGB/stereo K·distortion/rectification·depth projection·mode/table revision도 물리외부보정과 별개다. Stream 초기화 전 SDK getRectifyLogData(0)는 null, loadstatus0의 getter는 zero였다. 동일 기존 depth stream의 scale-down11bit640×460/15fps/index0 초기화 및1frame 뒤에는 유효 nonzero factory CamMat/P/Q가 반환됐다. 따라서 이전zero를 실제 factory 값으로 해석하지 않는다. Factory log의 packed input/output은2560×920인데 취득video는1280×460이다. 두 축half-scale 대응은 가설이며 crop/pixel-centre/raw-vs-rectified 관계 검증전K를 자동축소/채택하지 않았다. Frame ZDtable buffer는0bytes라 rawtable·invalid-depth해석·RGB/depth정합은 미확정이다. 실제계수·SDKrevision/checksum은 비공개 로컬에 보존한다. 설치 SDK의 mode DB에서 해당 제품은 Hypatia2이고 depth-only640×460/15fps/rectification index0는 mode5에 명시돼 있다. 예제 callback도640×460 depth에 scale-down11bit를 선택한다. 그러나 현재 UVC packed1280×460 YUYV 영상은 DB의 packed2560×920 raw-YUYV 또는 MJPEG rectified 모드와 정확한 대응이 확인되지 않았다. 설치 구현이 binary-only라 resampling/crop/pixel-centre 및 raw CamMat와 rectified P 중 어느 것이 실제 crop에 맞는지 입증하지 못했다. 따라서 factory K를.5배로 축소하거나 이 영상의 metric PnP를 실행하지 않았다.

## 검증 결과

- 기존 vision_target focused8 PASS/.006s (MOCK).
- 기존 camera calibration self-test PASS:20 synthetic views, RMS.358px,95/100 scale가 K를 유지하고 board→camera translation을.95배로 만드는 회귀 및 negative 입력 거부.
- 실측24mm 직접 입력 self-test PASS: 기존95mm 기준선과 각각 독립 provenance를 유지하고 K 불변/translation scale, 저장된 measurement revision, invalid/ambiguous/미기록 입력 거부를 검증했다.
- 최소 transform chain focused3 PASS/.028s: 회전/이동/inverse/점 chain, 미측정 revision/frame 거부, invalid transform/pose 거부.
- Fullsuite/CI는 실행하지 않았다. 이 결과는 실물 카메라 정확도나 task 성공의 증명이 아니다.

## 다음 실제 측정

1. Board의 원본 도안과 여러 square 길이/XY scale/flatness를 확인하고, block을 가리지 않는 여러 board view를 수집한다. 동일 camera side/resolution/crop/rotation/focus/raw-or-rectified 설정을 유지한다.
2. Intrinsics 및 factory rectification/depth projection 관계와 mode revision을 확인한다. 이미 rectified 영상에 distortion을 중복 적용하지 않는다. RGB/depth pairing과 registration을 독립 검증한다.
3. Board→left-arm-base 관계를 고정 기준점/실측 board pose로 측정하고 unused points에서 확인한다. 실제 base revision과 CAD의 고정 datum feature를 먼저 대조해야 한다. CAD arm origin을 tabletop/foot-plane/foot-centre/mount-hole-centre로 동일시하지 않는다. 확인한 CAD base origin은 모델 최저면보다2.4mm 위지만 이 관계는 실물 장착면·XY datum·rotation·extrinsic의 실측이 아니다. Shoulder axis/발바닥만으로 in-plane yaw/sign이 정해지지 않으며 SIM offset을 REAL 변환으로 복사하지 않는다. Camera-only board pose만으로 arm transform을 만들지 않는다.
4. 실제 block top을 관측·변환하고 gravity/up와 uncertainty를 확인한 뒤 top 위20mm의 shadow TCP를 만든다. Mask centroid/nominal40mm/SIM pose로 top을 대체하지 않는다.
5. Calibrated 비접촉 orientation·TCP/hand envelope·양팔 mapping·IK/path/collision을 검증한다. Python은 shadow만 만들며 C++ 실물 실행 경계와 총괄의 별도 승인에 남긴다.

현재 R2는 실제 영상/SDK 원시거리 취득과 marker overlay까지이며 registered metric observation은 미완료다. R3 camera→arm은 BLOCKED. Arm q/shadow/실물 동작 미실행.

## 실제 intrinsic 수집 R1 체크포인트

사용자가 SPACE/H/Q로 서로 다른 training15개와 holdout3개를 수집했고, rejected2개는 corner9개 부족과 유사 영상평면 자세로 분리했다. 기존 최소12 non-collinear corner 기준을 재사용했다.15px projected-quad 차이는 중복 수집을 줄이는 휴리스틱이며 calibration 품질 문턱이 아니다. 저장20개 원본 hash는 모두 다르고 manifest와 일치했다. Camera-only GUI는 Q 후 정상 release됐다.

직접24.0mm 입력의 최초 fit RMS는1.178301px로 **기존1px 경고 기준을 넘었다(exit2)**. Holdout에서 각 pose만 fit한 RMS는1.252592/1.080057/1.107578px이며 독립 metric 정확도 증명이 아니다. Principal point/coverage/focal uncertainty에 대한 기존 경고는 없었다.54corner 전체 보드2개 영상의 RMS1.929/2.391px가 컸고, 나머지13개는.344–.772px였다. Worst 영상의 손으로 잡은 종이 곡률은 평면 가정 위반의 후보 근거지만 blur/lens/평탄성 중 원인을 확정하지 않는다. 원본을 임의 삭제하거나 모델/기준을 늘리지 않았다. 사용자 승인에 따라 종이를 단단한 판에 고정한 별도 데이터셋으로 재수집 중이며 첫 결과를 보존한다.

첫 수집 R1은 intrinsic 후보와 quality warning을 남겼다. Runtime 적용·metric PASS·camera→arm 보정·arm q·shadow·실물 동작 성공으로 보고하지 않는다.

## 고정판 intrinsic 재수집 결과

사용자가 종이를 단단한 판에 고정해 별도 dataset-r2에서 training15/holdout3/rejected1을 수집했다. 이 dataset 이름은 외부보정 단계와 구분한다.19개 원본은 hash가 모두 다르고 manifest와 일치하며, training과 holdout 원본 hash는 겹치지 않는다. Camera GUI는 정상 종료/release됐다. API readback은1280×460/YUYV/15fps, focus/autofocus=-1이므로 unknown이다. 설정을 변경하지 않은 사실과 focus 고정은 구분한다.

동일 모델·기준·24.0mm 간격의 fit RMS는 **.495353px**, 기존 quality warning은 없다. Per-view RMS는.179–.866px, held-out pose-fit RMS는.508803/.463835/.387512px였다. Focal relative standard deviation은1.417/1.425%, normal span은38.697deg, corner coverage X/Y는64.24/62.77%다. 원본 삭제·선별fit·모델 증가·기준 완화 없이 수치/재투영 후보 검사를 통과했다. R1보다 개선됐지만 poses도 달라 평탄성 단독 인과를 증명하지 않는다. Intrinsics 계수/원시 영상/개인 revision은 로컬에만 보존한다. 이 후보는 해당 left640×460 mapping에 한 번만 적용하며 depth registration·독립 metric 정확도·camera→arm 보정·runtime 채택을 의미하지 않는다.

다음 board→camera 관측은 고정 완료가 확인된 새로운 한 장으로 수행한다. Handheld training/holdout pose를 고정 board extrinsic으로 사용하지 않는다. 기존 whole-grid 원점을 유지한 채 사용자가 선택한 ID0 inner corner를 별도 datum으로 정하면 grid 좌표는(24,24,0)mm이고 T_camera_from_ID0=T_camera_from_grid×Trans(.024,.024,0)다. 실제 visible corner임을 확인해야 하며 board +Z=X×Y를 gravity up으로 가정하지 않는다. Board→left-arm-base의 실측/검증 관계는 여전히 필요하다.


## 고정 보드 측정 단계의 현재 한계
새 고정 위치는19corners/PnP RMS.227px이며 실제검출ID15를 선택했다.
ID0는 안 보여 원점으로 외삽하지 않았다. 선택점의 grid좌표(168,48,0)mm와
T_camera_from_grid / T_camera_from_selected_innercorner를 구분한다.
설치base 형상/책상직접밀착은 확인했으나 CAD virtual datum을 현장 측정점으로
아직 대응하지 못했다. 밑판 전체와 board가 함께 보이는 사진의 실제 feature를
표시해 실측할 예정이다. Camera→arm 숫자나 shadow/hardware motion은 생성하지 않았다.
