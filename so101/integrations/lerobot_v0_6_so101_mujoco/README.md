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
- `assets/pick_cube.xml`과 provenance 문서
- `examples/so101_mujoco/`
- `tests/envs/test_so101_mujoco.py`

약 16MB의 STL과 원본 `so101_new_calib.xml`은 직접 만든 코드가 아니어서 중복
커밋하지 않았다. foundation 문서에 고정한 SO-ARM100 revision에서 자산을 준비한
뒤 LeRobot checkout 루트에서 다음 해시를 확인해야 한다.

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
`Shift+P`는 검증된 300-frame padded pick-and-lift를 재생한다.

interactive scene은 G1에서 직접 확인한 reachable cube 위치, raised support와
finger pad 조건을 사용하고 별도 green goal tray를 둔다. `Shift+P` 실제 GUI
재생에서 cube 최종 z `0.111 m`, process exit `0`을 확인했다. 다만 frame 299
뒤에도 물리를 계속 진행하면 현재 pad 모델은 약 8 frame 안에 cube를 놓친다.
이를 장기 hold 성공으로 쓰지 않고, viewer는 검증된 frame 299에서 물리를
명시적으로 pause해 결과를 관찰하게 했다. 수동 이동 chord를 누르면 물리가 다시
진행된다.

## 결과를 섞지 않는 규칙

이 overlay로 2026-08-06 만든 `so101_mujoco_joint_sweep`은 5 episode, 450
frame이지만 `next.success`는 0/450이었다. 따라서 물체 집기 성공 데이터가 아니다.
2026-08-07의 G1 PASS 정본은 `dapier_sim_first` Gate에서 나온 결과다. 현재
interactive overlay는 그 task adaptation을 사람이 조작해 볼 수 있게 옮긴 것이며,
새로운 learned policy나 장기 hold 성공 결과로 합치지 않는다.
