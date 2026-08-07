# SO-101 MuJoCo

This example runs an SO-101 digital twin without a leader arm or a connected follower. Its public action
and state contract matches the calibrated LeRobot arm: five joints in degrees followed by a gripper value
from 0 (closed) to 100 (open).

## Install

From the LeRobot repository root:

```bash
uv sync --extra so101_mujoco --extra test
```

For a headless Linux machine, select EGL before rendering:

```bash
export MUJOCO_GL=egl
```

## Run without recording

```bash
uv run python examples/so101_mujoco/run_joint_sweep.py --episodes 1 --steps 90
```

This deterministic sweep is a data-pipeline diagnostic, not a pick-and-place expert.

## Teleoperate without physical hardware

The interactive recorder uses top and wrist cameras by default. In this expert route, top RGB drives the
IK planner while wrist RGB is recorded as the future student's observation. It keeps only successful
episodes, so aborted attempts do not enter the training set.

```bash
export SO101_SIM_DATASET_ROOT="<NEW_OUTPUT_PATH_ON_THIS_PC>"
uv run python examples/so101_mujoco/teleoperate.py \
  --input keyboard \
  --camera-set expert \
  --control-mode ik_expert \
  --episodes 20 \
  --record-root "$SO101_SIM_DATASET_ROOT" \
  --repo-id local/so101_mujoco_teleop
```

The controls are also shown in the viewer. They deliberately avoid number keys because MuJoCo reserves
`0` through `4` for geometry-group visibility, and they require `Shift` so MuJoCo does not consume its
plain-letter rendering shortcuts at the same time.

- Hold `Shift+W`/`Shift+S`, `Shift+A`/`Shift+D`, or `Shift+R`/`Shift+F` for continuous
  world X, Y, or Z gripper motion.
- Hold `Shift+O`/`Shift+L` to open or close the gripper.
- Press `Shift+J`/`Shift+K` to select the previous or next joint, then hold
  `Shift+Up`/`Shift+Down` for direct joint motion.
- Press `Shift+G` for the known open-gripper cube-approach target.
- Press `Shift+P` to replay the verified 390-frame CAD-aligned padded pick-and-lift demonstration. The
  interactive viewer pauses physics at verified frame 389 so the result remains inspectable; any manual
  motion command resumes physics.
- Press `Shift+V` to reset to the next randomized scene, move to the observation pose, detect the blue
  cube from top RGB, then run the verified IK pick-and-place plan. Press `Shift+C` to cycle through the
  external, top, and wrist views.
- Press `Shift+H` for the safe home target, `Shift+N` for a new episode, and `Shift+Q` to quit.

The `Shift` chord is required because MuJoCo assigns plain letters such as `W`, `A`, `F`, `P`, and `Q`
to rendering and visualization toggles. The hold behavior uses X11 key-state polling after a key press
has arrived in the focused MuJoCo window; it does not depend on desktop key-repeat timing. The default
interactive cube randomization is `+-0.025 m` in XY. Pass `--cube-randomization 0` only for a fixed
layout. Every `Shift+V` and `Shift+N` reset consumes the next seed from the same sequence, so the first
automatic run does not repeat the initial layout. Use `--save-mode=all` only when failed demonstrations
are intentionally needed.

## Optionally drive MuJoCo with the calibrated leader

This mode reads the leader through LeRobot's `SO101Leader`; it does not connect to or command a follower.

```bash
uv sync --extra so101_mujoco --extra feetech
uv run python examples/so101_mujoco/teleoperate.py \
  --input leader \
  --leader-port "<LEADER_PORT_ON_THIS_PC>" \
  --leader-id "<LEADER_CALIBRATION_ID_ON_THIS_PC>" \
  --episodes 20 \
  --record-root "$SO101_SIM_DATASET_ROOT" \
  --repo-id local/so101_mujoco_leader_teleop
```

The existing leader calibration is loaded by its ID. If LeRobot reports that the motor values and file
do not match, stop and inspect the calibration instead of starting a new calibration casually.

## Record a diagnostic LeRobot dataset

Choose an output path on the current machine; no user-specific path is assumed:

```bash
export SO101_SIM_DATASET_ROOT="<OUTPUT_PATH_ON_THIS_PC>"
uv run python examples/so101_mujoco/run_joint_sweep.py \
  --episodes 5 \
  --steps 90 \
  --record-root "$SO101_SIM_DATASET_ROOT"
```

The diagnostic joint-sweep dataset contains `observation.state`, `observation.images.front`, and `action`
with the same six-axis order used by the real SO-101. The IK expert recorder instead contains
`observation.images.top` and `observation.images.wrist`, and writes
`meta/dapier_control_route.json`. Images are stored as individual frames in this baseline so it does not
depend on a video encoder.

## Train a wrist-only VLA from top-camera IK demonstrations

The routing is fail-closed:

- `top,wrist` selects the IK expert. Top RGB is privileged teacher input; wrist RGB, measured state, and
  commanded action are recorded together.
- `wrist` selects VLA inference. It cannot silently call IK or consume the top image.

First collect successful episodes with `Shift+V`. Other keyboard motion is not written into this expert
dataset:

```bash
export SO101_IK_DATASET_ROOT="<NEW_IK_DATASET_PATH_ON_THIS_PC>"
uv run python examples/so101_mujoco/teleoperate.py \
  --input keyboard \
  --camera-set expert \
  --control-mode ik_expert \
  --episodes 100 \
  --record-root "$SO101_IK_DATASET_ROOT" \
  --repo-id local/so101_ik_teacher
```

Make a distinct student dataset by removing only the privileged top image:

```bash
export SO101_WRIST_DATASET_ROOT="<NEW_WRIST_DATASET_PATH_ON_THIS_PC>"
uv run python -m lerobot.scripts.lerobot_edit_dataset \
  --repo_id=local/so101_ik_teacher \
  --root="$SO101_IK_DATASET_ROOT" \
  --new_repo_id=local/so101_wrist_student \
  --new_root="$SO101_WRIST_DATASET_ROOT" \
  --operation.type=remove_feature \
  --operation.feature_names='["observation.images.top"]'
```

Then train LeRobot's maintained SmolVLA implementation. Choose a new output directory; the values below
are an initial run configuration, not a completed training result:

```bash
export SO101_WRIST_VLA_OUTPUT="<NEW_SMOLVLA_OUTPUT_PATH_ON_THIS_PC>"
uv run python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/so101_wrist_student \
  --dataset.root="$SO101_WRIST_DATASET_ROOT" \
  --policy.type=smolvla \
  --policy.push_to_hub=false \
  --wandb.enable=false \
  --output_dir="$SO101_WRIST_VLA_OUTPUT" \
  --job_name=so101_wrist_smolvla \
  --steps=100000 \
  --batch_size=8 \
  --seed=23
```

Evaluate a saved `pretrained_model` directory with wrist images only:

```bash
uv run python examples/so101_mujoco/teleoperate.py \
  --input policy \
  --camera-set wrist-only \
  --control-mode vla \
  --policy-path "<SMOLVLA_PRETRAINED_MODEL_DIR>" \
  --episodes 10 \
  --steps 1800 \
  --output-dir "<NEW_WRIST_ONLY_EVAL_PATH>"
```

This command delegates policy rollout to LeRobot's standard evaluator. The dataset derivation, training,
and evaluation command lines have been parser-checked locally, but no expert dataset or VLA checkpoint has
been trained in this slice.

## Camera profile boundary

The wrist camera pose is derived from TheRobotStudio's integrated 32x32 UVC module mount surface and is
attached to the fixed `gripper` body, not the moving jaw. `assets/camera_profiles.json` records its source
revision and hash. The physical lens center, module orientation, image rotation, intrinsics, and FOV have
not been measured on this PC, so the profile deliberately reports `physical_alignment=false`. Calibrate
those fields against the installed module before treating simulation pixels as real-camera-equivalent.

## Train the official LeRobot ACT policy

Use a dataset containing successful task demonstrations, not the diagnostic joint sweep:

```bash
export SO101_ACT_OUTPUT="<ACT_OUTPUT_PATH_ON_THIS_PC>"
uv run lerobot-train \
  --dataset.repo_id=local/so101_mujoco_teleop \
  --dataset.root="$SO101_SIM_DATASET_ROOT" \
  --policy.type=act \
  --policy.push_to_hub=false \
  --output_dir="$SO101_ACT_OUTPUT" \
  --job_name=so101_mujoco_act \
  --batch_size=64 \
  --steps=100000
```

This uses LeRobot's maintained ACT implementation instead of the simplified classroom model.

## Check a real calibration before sim-to-real work

Pass paths from the machine that owns the files:

```bash
uv run python examples/so101_mujoco/check_calibration_contract.py \
  "<LEADER_CALIBRATION_JSON_ON_THIS_PC>" \
  "<FOLLOWER_CALIBRATION_JSON_ON_THIS_PC>"
```

The checker validates joint names/order, motor IDs, required fields, tick bounds, and minimum calibrated
span. It never guesses calibration paths.

## Scope and next tasks

The current slice covers model loading, joint- and Cartesian-position control, deterministic reset, a
reachable cube/support/goal-tray scene, visible CAD-plane finger contact proxies, top and wrist RGB
rendering, a Gymnasium/LeRobot adapter, keyboard/leader simulation teleoperation, success-only IK expert
selection, and diagnostic dataset recording. The scripted `P` path has been verified for padded
pick-and-lift. The latest top-RGB IK route placed `10/10` randomized seeds in the square goal tray with
mean XY estimation error `0.670 mm` and maximum `0.939 mm`. This is not a learned policy,
physical-camera/gripper validation, or sim-to-real evidence.

The next vertical slices are:

1. add in-motion RGB re-detection and measure recovery from calibration/noise errors;
2. collect successful expert episodes and train/evaluate the ACT baseline;
3. domain randomization for friction, backlash, latency, lighting, and camera pose;
4. a 12-action bimanual scene made from two SO-101 instances;
5. bimanual handover, dish-rack loading, towel folding, then deformable garment folding.

Rigid-object handover and dish loading come before cloth. Cloth needs a flexible-body model, stable
two-gripper contacts, and task-specific metrics for corner alignment and fold quality.
