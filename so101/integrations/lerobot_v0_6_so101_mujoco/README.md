# LeRobot v0.6.0 SO-101 MuJoCo 실험 보존본

2026-07-31에 Hugging Face LeRobot checkout 안에서 만든 SO-101 MuJoCo 환경을
upstream 전체와 분리해 보존한다. 이 코드는 LeRobot 내부 경로를 전제로 하므로
독립 DAPIER Python 패키지가 아니다. 현재 G0/G1 정본은
[`dapier_sim_first`](../../../dapier_sim_first/README.md)다.

## 고정한 기준

- LeRobot remote: `https://github.com/huggingface/lerobot.git`
- base revision: `30da8e687a6dfc617fcd94afc367ac7071c376ce` (`v0.6.0`)
- license: `LICENSE.apache-2.0.txt`
- 로컬 custom Python/XML/example/test: `overlay/`
- upstream tracked 파일 변경: `lerobot-v0.6.0-tracked.patch`
- SO-ARM100 원본 model/mesh 기대 해시: `UPSTREAM_ASSETS.sha256`

`uv.lock`에는 SO-101 extra 추가 외에도 당시 uv가 다시 해석한 platform marker
변경이 100줄 이상 섞여 있었다. 그 lockfile은 고유 구현으로 보존하지 않았다.
새 checkout에 overlay를 적용할 때 현재 uv로 lock을 다시 만들고 diff를 별도로
검토한다.

## overlay에 포함한 것

- `src/lerobot/envs/so101_mujoco/*.py`
- `assets/pick_cube.xml`, camera profile JSON과 provenance 문서
- top+wrist IK expert / wrist-only VLA fail-closed routing과 dataset sidecar
- `examples/so101_mujoco/`
- `tests/envs/test_so101_mujoco.py`

약 16MB의 STL과 원본 `so101_new_calib.xml`은 직접 만든 코드가 아니어서 중복
커밋하지 않았다. wrist camera는 원본 MJCF를 수정하지 않고 environment load 때
JSON profile에 따라 `gripper` body에 추가한다. foundation 문서에 고정한
SO-ARM100 revision에서 자산을 준비한 뒤 LeRobot checkout 루트에서 다음 해시를
확인해야 한다.

```bash
sha256sum -c \
  "$DAPIER_ROOT/so101/integrations/lerobot_v0_6_so101_mujoco/UPSTREAM_ASSETS.sha256"
```

## 깨끗한 checkout에 재구성하는 순서

```bash
export DAPIER_ROOT="${DAPIER_ROOT:-$HOME/DAPIER}"
export LEROBOT_ROOT=/path/to/clean/lerobot
export OVERLAY_ROOT="$DAPIER_ROOT/so101/integrations/lerobot_v0_6_so101_mujoco"

git -C "$LEROBOT_ROOT" checkout --detach \
  30da8e687a6dfc617fcd94afc367ac7071c376ce
git -C "$LEROBOT_ROOT" apply "$OVERLAY_ROOT/lerobot-v0.6.0-tracked.patch"
cp -a "$OVERLAY_ROOT/overlay/." "$LEROBOT_ROOT/"
```

그다음 SO-ARM100 자산을 위 해시와 같은 경로에 준비하고 lockfile을 갱신한다.
이 절차는 source 복원 절차이지 실행 성공 주장이나 hardware 제어 절차가 아니다.

2026-08-07에는 임시 디렉터리에 base revision을 풀고 patch와 overlay를 다시
적용했다. interactive control 수정 전에는 10/10, 수정 후에는 asset 해시
14/14와 `tests/envs/test_so101_mujoco.py` 15/15가 통과했다. Gymnasium의
비대칭·비정규화 action space 권고 warning 1건은 남아 있다.

## 2026-08-07 interactive viewer 수정

직접 실행해 보니 기존 `1..6` 관절 선택은 MuJoCo의 geom group 표시 단축키와
겹쳤다. 문자키도 `W=wireframe`, `A=auto-connect`, `F=contact force`,
`P=contact split`, `Q=camera`처럼 대부분 뷰어 단축키였다. 그래서 로봇 입력은
모두 `Shift+키` chord일 때만 처리한다. 실제 뷰어에서 사용하는 chord 전체를
보내고 visualization, rendering, geom-group flag 변화가 각각 0건인지 확인했다.

X11 key state를 읽어 OS key repeat에 기대지 않으므로 `Shift+W`처럼 키를 누르고
있는 동안 30 Hz로 계속 움직인다. 0.5초 `Shift+W` 검증에서는 gripper 끝점 X가
약 39 mm 이동했고 key release 뒤 추가 이동은 없었다. `Shift+W/S`,
`Shift+A/D`, `Shift+R/F`는 world XYZ, `Shift+O/L`은 gripper,
`Shift+Up/Down`은 선택 관절을 움직인다. `Shift+G`는 cube approach,
`Shift+P`는 검증된 390-frame padded pick-and-lift를 재생한다.

interactive scene은 G1에서 직접 확인한 reachable cube 위치와 별도 green goal
tray를 둔다. 이번 후속 수정에서는 `Shift+P`를 390 frame으로 다시 맞췄고,
마지막 30 frame 동안 양쪽 pad 접촉과 support 비접촉을 직접 검사한다. viewer는
자동 동작 끝에서 물리를 pause해 결과를 관찰하게 하며, 수동 이동 chord를 누르면
다시 진행한다.

같은 날 후속 실습에서 gripper에 wrist RGB camera와 비충돌 mount를 추가했다.
`Shift+C`로 external/wrist view를 바꾸고, `Shift+V`로 reset부터 RGB 검출,
pick, green tray place까지 실행한다. 검은 판처럼 보이던 finger contact geom은
이번에 보이는 형상과 충돌 형상을 하나로 합쳤다. 양쪽 검은 rubber proxy는 각각
`24 x 16 x 6 mm`이고 접촉면을 CAD fingertip의 측정 평면에 맞췄다. 기존처럼
보이지 않는 큰 collision envelope가 cube를 받치지 않는다.

upstream fingertip collision mesh는 MuJoCo에서 오목한 CAD가 아니라 convex hull로
계산되어 빈 공간에서도 cube를 밀었다. `gripper`와 `moving_jaw_so101_v1`의 그
collision 사본만 비활성화하고, 화면에 보이는 CAD mesh와 위 rubber proxy를
남겼다. 회색 시작 tray의 높은 벽에도 cube가 걸렸기 때문에 visual과 collision을
함께 낮은 rim으로 바꿨다. 보이는 벽만 낮추거나 보이지 않는 통과 영역을 만들지는
않았다.

planner는 wrist RGB의 blue mask, 현재 camera calibration, 알려진 cube top plane만
사용한다. cube body pose, depth와 segmentation id는 사용하지 않는다. `+-25 mm`
cube randomization에서 seed `101..130`을 직접 실행한 결과 `30/30`이 tray success
조건을 만족했고, RGB XY 추정 오차는 평균 `3.08 mm`, 최대 `6.55 mm`였다. 현재
SO-101 MuJoCo test는 `24/24 PASS`이며 Gymnasium warning 1건은 남아 있다. 자세한
기록은
[`2026-08-07-so101-wrist-vision-pick-place.md`](../../records/2026-08-07-so101-wrist-vision-pick-place.md)에
남겼다. base revision을 임시 디렉터리에 다시 checkout하고 검증된 upstream asset
`14/14`, tracked patch와 overlay를 차례로 적용한 재구성 환경에서도 `24/24`가
통과했다.

keyboard 실행의 cube randomization 기본값은 `+-25 mm`다. `--seed 101`이면 시작
scene은 101을 쓰고 첫 `Shift+V` 또는 `Shift+N` reset은 102, 다음 reset은 103처럼
하나의 seed sequence를 공유한다. 따라서 첫 `Shift+V`가 시작 위치를 반복하던
문제도 남지 않는다. 고정 scene이 필요할 때만 `--cube-randomization 0`을 준다.

## 2026-08-07 CAD camera profile과 IK/VLA 분기

앞선 wrist RGB 실습에서 임의로 둔 `(0.05, -0.07, 0.04) m` 카메라는 현재
기준값이 아니다. 공식 SO-101에는 모든 카메라에 공통인 URDF extrinsic이 없고
카메라별 mount CAD가 여러 개라서, 이번에는 TheRobotStudio의 integrated 32×32
UVC module mount를 명시적으로 선택했다. 해당 STL의 camera-board mounting face와
공식 wrist-roll mesh pose에서 다음 `gripper` local profile을 계산했다.

- position: `(0.0025, -0.072057361, 0.004150235) m`
- camera X/Y axes: `(1, 0, 0)`, `(0, 0.906307787, 0.422618262)`
- look direction: `(0, 0.422618262, -0.906307787)`
- profile id: `therobotstudio_integrated_32x32_mount_surface_v1`

실제 lens offset, 장착 방향, image rotation과 FOV는 아직 측정하지 못했으므로
JSON에 `physical_alignment=false`를 유지한다. 이 값은 실제 영상과 동일하다는
주장이 아니라 CAD 장착면이 gripper와 함께 움직이는 방식의 기준이다.

기본 policy camera set은 `top,wrist`다. 이때 `Shift+V`는 팔을 top camera 관찰
자세로 옮긴 뒤 top RGB만으로 cube XY를 구하고 IK expert trajectory를 만든다.
동시에 wrist RGB, measured state와 commanded action을 기록해 student 데이터로
남긴다. recorder는 `Shift+V` 자동 동작 frame만 받고 다른 수동 frame은 섞지
않으며 `meta/dapier_control_route.json`에 teacher/student 계약을 쓴다.

`wrist-only`는 IK로 자동 fallback하지 않는다. 반드시 `--input policy`와 VLA
checkpoint를 요구하고 표준 `lerobot_eval`에 위임한다. IK expert 데이터에서
`observation.images.top`만 제거하는 `lerobot_edit_dataset`, wrist-only 데이터로
SmolVLA를 학습하는 `lerobot_train`, wrist-only 평가 명령 builder를 함께 테스트했다.

최신 상태에서 정적 검사, `31/31` MuJoCo test, seed `0..9`의 top-RGB IK
pick-and-place `10/10`이 통과했다. top RGB XY 오차는 평균 `0.670 mm`, 최대
`0.939 mm`였다. clean v0.6.0 checkout에 patch와 overlay를 다시 적용하고 원본
asset 해시 `14/14` 및 같은 `31/31` test도 확인했다.

후속 end-to-end smoke에서는 headless collector로 seed `200..202`의 successful IK
episode `3/3`, 총 `1,980` frame을 실제 LeRobot dataset으로 기록했다. generic
`remove_feature`가 추가 sidecar를 복사하지 않는 것을 발견해 episode/frame 수와
feature를 검사한 뒤 provenance를 다시 쓰는 wrapper를 추가했다. wrist student에는
top image가 없고 wrist/state/action이 남는다.

RTX 5050 8GB에서 SmolVLA extra와 pretrained SmolVLM2-500M을 사용해 batch 1,
1-step training checkpoint를 만들고, 그 checkpoint로 wrist-only 5-step evaluator를
실행했다. 첫 evaluator에서는 flat image key 때문에 정책이 영상을 찾지 못했고,
environment observation을 LeRobot 표준 nested `pixels` 구조로 고친 뒤 rollout이
끝까지 실행됐다. smoke success는 `0/1`이다. 따라서 VLA train/inference **배관은
실행 검증**, learned pick policy 성능은 **미검증**으로 분리한다. full training과
physical camera alignment도 아직 확인하지 않았다.

### 2026-08-10 bounded wrist-only 학습과 held-out 평가

seed `400..429`에서 IK expert `30/30`, 총 `19,800` frame을 새로 수집했다.
top RGB XY 오차는 평균 `0.817 mm`, 최대 `1.577 mm`였다. 변환 후 student에는
`observation.images.top`이 없고 wrist/state/action과 teacher contract hash가
남아 있다.

RTX 5050 8GB에서 pretrained SmolVLM2-500M 기반 SmolVLA를 batch 4로 두 단계
5,000 update씩 이어서 학습했다. 최종 checkpoint 기준 optimizer update는
`10,000`, sample은 `40,000`, 데이터 기준 약 `2.02 epoch`이며 마지막
logged loss는 `0.021`이다.

학습에 쓰지 않은 seed `800..809`, episode당 700 step으로 wrist-only 평가한
결과는 `2/10 (20%)`였다. 평균 max reward는 `0.5224936`, 평균 sum reward는
`143.9673`이다. `record_wrist_vla_evidence.py`가 LeRobot `eval_info.json`을
읽어 student sidecar에 이 수치를 복사한다. 학습·평가 완료와 정책 성능 통과를
분리해 `vla_trained=true`, `vla_evaluated=true`,
`vla_success_threshold_met=false`로 기록했다.

같은 날 새 read-only hardware gate와 회귀 test를 추가했다. 실제 inventory에는
ASUS 내장 FHD/IR video node만 있고 `/dev/serial/by-id` 장치가 없었다. 선택한
32×32 wrist profile도 계속 `physical_alignment=false`다. receipt는 세 이유로
`blocked`이며 device open, motor command, physical rollout은 모두 false다.
새 gate를 포함한 SO-101 MuJoCo test는 `33/33 PASS`다.

따라서 full training과 held-out evaluation은 완료했지만 80% sim 성능 기준,
실물 camera calibration, sim-to-real과 실제 주행은 완료하지 않았다.

## 결과를 섞지 않는 규칙

이 overlay로 2026-08-06 만든 `so101_mujoco_joint_sweep`은 5 episode, 450
frame이지만 `next.success`는 0/450이었다. 따라서 물체 집기 성공 데이터가 아니다.
2026-08-07의 G1 PASS 정본은 `dapier_sim_first` Gate에서 나온 결과다. 현재
interactive overlay는 그 task adaptation을 사람이 조작해 볼 수 있게 옮긴 것이며,
새로운 learned policy나 장기 hold 성공 결과로 합치지 않는다.
