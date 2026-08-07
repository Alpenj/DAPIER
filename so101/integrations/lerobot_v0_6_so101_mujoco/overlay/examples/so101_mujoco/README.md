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

The interactive recorder uses front and wrist cameras by default. It keeps only successful episodes, so
aborted attempts do not enter the training set.

```bash
export SO101_SIM_DATASET_ROOT="<NEW_OUTPUT_PATH_ON_THIS_PC>"
uv run python examples/so101_mujoco/teleoperate.py \
  --input keyboard \
  --episodes 20 \
  --record-root "$SO101_SIM_DATASET_ROOT" \
  --repo-id local/so101_mujoco_teleop
```

In the MuJoCo viewer, press `1` through `6` to select a joint, use `Up`/`Down` to jog it, `Home` to
return the target to the safe home pose, `N` to discard the current attempt and start another, and
`Q` or `Esc` to quit. Use `--save-mode=all` only when failed demonstrations are intentionally needed.

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

The diagnostic dataset contains `observation.state`, `observation.images.front`, and `action` with the same
six-axis order used by the real SO-101. The interactive dataset additionally contains
`observation.images.wrist`. Images are stored as individual frames in this baseline so it does not depend
on a video encoder.

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

The current slice covers model loading, joint-position control, deterministic reset, a cube/tray scene,
front and wrist RGB rendering, a Gymnasium/LeRobot adapter, keyboard/leader simulation teleoperation,
success-only demonstration selection, and diagnostic dataset recording. It does not yet claim successful
autonomous grasping.

The next vertical slices are:

1. scripted IK reach, grasp, lift, and place with measured success rate;
2. collect successful expert episodes and train/evaluate the ACT baseline;
3. domain randomization for friction, backlash, latency, lighting, and camera pose;
4. a 12-action bimanual scene made from two SO-101 instances;
5. bimanual handover, dish-rack loading, towel folding, then deformable garment folding.

Rigid-object handover and dish loading come before cloth. Cloth needs a flexible-body model, stable
two-gripper contacts, and task-specific metrics for corner alignment and fold quality.
