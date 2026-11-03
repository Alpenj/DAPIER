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

For a reproducible headless batch, collect successful IK episodes directly. This refuses to overwrite an
existing root and marks the sidecar verified only after every requested episode succeeds:

```bash
export SO101_IK_DATASET_ROOT="<NEW_IK_DATASET_PATH_ON_THIS_PC>"
MUJOCO_GL=egl uv run python examples/so101_mujoco/collect_ik_expert.py \
  --root "$SO101_IK_DATASET_ROOT" \
  --episodes 100 \
  --repo-id local/so101_ik_teacher
```

The interactive recorder above remains useful for visual inspection: press `Shift+V` once per episode.
Other keyboard motion is not written into its expert dataset.

Make a distinct student dataset by removing only the privileged top image. The wrapper calls LeRobot's
standard editor, verifies that episode/frame counts are unchanged, and restores the DAPIER provenance
sidecar that the generic editor does not copy:

```bash
export SO101_WRIST_DATASET_ROOT="<NEW_WRIST_DATASET_PATH_ON_THIS_PC>"
uv run python examples/so101_mujoco/prepare_wrist_student.py \
  --teacher-root "$SO101_IK_DATASET_ROOT" \
  --teacher-repo-id local/so101_ik_teacher \
  --student-root "$SO101_WRIST_DATASET_ROOT" \
  --student-repo-id local/so101_wrist_student
```

Then train LeRobot's maintained SmolVLA implementation. Choose a new output directory; the values below
are an initial run configuration, not a completed training result:

```bash
export SO101_WRIST_VLA_OUTPUT="<NEW_SMOLVLA_OUTPUT_PATH_ON_THIS_PC>"
uv run python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/so101_wrist_student \
  --dataset.root="$SO101_WRIST_DATASET_ROOT" \
  --policy.type=smolvla \
  --policy.load_vlm_weights=true \
  --policy.push_to_hub=false \
  --wandb.enable=false \
  --output_dir="$SO101_WRIST_VLA_OUTPUT" \
  --job_name=so101_wrist_smolvla \
  --steps=100000 \
  --batch_size=1 \
  --num_workers=0 \
  --persistent_workers=false \
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

With a viewer, this command runs the interactive intervention loop. Press `Space` to transfer authority
from VLA to the keyboard, correct the pose with the same `Shift+W/S`, `Shift+A/D`, `Shift+R/F`,
`Shift+O/L`, and selected-joint chords documented above, then press `Enter` to resume VLA. Resume clears
the policy's remaining action-chunk queue so pre-intervention commands cannot execute afterward.
`Shift+C` cycles external/top/wrist views, `Shift+N` ends the attempt, and `Shift+Q` quits. Every frame is
written under `<OUTPUT>/interventions/episode_NNNN/` as source-labeled JSONL plus wrist PNG, including
requested and actually applied actions. The manifest labels this as conversion-required evidence; it is
not silently presented as a ready-to-train LeRobot dataset.

Add `--no-viewer` to delegate an unattended policy rollout to LeRobot's standard evaluator instead. The
same headless route accepts `--parallel-envs N`. It keeps one policy on the GPU, batches observations from
`N` asynchronous MuJoCo workers, writes one action trace per worker, and emits
`parallel_rollout_manifest.json`. On the RTX 5050 8 GB test, four workers did not reduce wall time for four
episodes because one failed worker ran to the 700-step horizon; use one worker for reproducible performance
comparison and parallel workers for bounded failure harvesting. Parallel rollout is experience generation,
not an optimizer update or a ready-to-train dataset.

```bash
uv run python examples/so101_mujoco/teleoperate.py \
  --input policy --camera-set wrist-only --control-mode vla --no-viewer \
  --policy-path "<SMOLVLA_PRETRAINED_MODEL_DIR>" \
  --episodes 8 --parallel-envs 4 --steps 700 --seed 1800 \
  --output-dir "<NEW_PARALLEL_EVAL_PATH>"
```

The unattended subprocess defaults to EGL unless `MUJOCO_GL` is already set. Trace rows include the reset
seed, reward, success, termination, truncation, and `episode_done`. The manifest resolves each evaluator
episode to its worker file and local `trace_episode_index` by seed, including runs where the number of
episodes exceeds the number of workers; select that local episode through its first `episode_done=true` row.

The 2026-08-10 bounded run on the
current 8 GB GPU collected 30 successful IK episodes (19,800 frames, seeds 400 through 429), then removed
the top image while preserving wrist/state/action and the teacher-contract hash. Top-RGB XY error was
0.817 mm on average and 1.577 mm at maximum.

The final wrist checkpoint used two consecutive 5,000-update, batch-4 stages: 10,000 optimizer updates,
40,000 samples seen, or approximately 2.02 passes over the recorded frames. A held-out wrist-only
evaluation used seeds 800 through 809, 700 steps per episode, and scored `2/10 (20%)`. Average maximum
reward was 0.5224936 and average summed reward was 143.9673. This is completed bounded training and
evaluation, but it does not meet the 80% task threshold and is not a successful sim-to-real policy.

The wrist-only route now blends the first three frames of each 25-action chunk, applies per-axis target
slew limits derived from the verified v2 IK demonstrations, and holds gripper changes below one percentage
point. The generic environment keeps smoothing disabled unless the VLA route requests it. Evaluation also
writes `action_trace.jsonl` with raw/applied actions plus radian command and synchronous MuJoCo readback.
Summarize a completed trace with:

```bash
uv run python examples/so101_mujoco/analyze_action_trace.py \
  <WRIST_ONLY_EVAL_PATH>/action_trace.jsonl \
  --episode-length 700
```

On 2026-08-11, a same-seed five-episode A/B retained `4/5` success while reducing the maximum
shoulder-pan chunk-boundary jump from `9.405°` to `1.750°`. A new 20-episode set scored `16/20`; the two
earlier held-out sets remained `14/20` each, so this is evidence for smoother execution rather than a claim
of stable 80% generalization or physical readiness.

The successful seed-1600 replay also exposed a simulation-fidelity defect: the 50 mm cube is commanded
with a learned gripper target near 27%, where the measured pad gap is only 39.1 mm. Contact penetration in
that rollout reached 8.3 mm. A 35% target gives a 49.3 mm free gap, but clamping the existing checkpoint to
that value made the replay fail. Correcting this requires changing the IK teacher grasp target, collecting
new demonstrations, and retraining; changing only renderer geometry would hide the error.

The 2026-08-11 correction run added explicit named pad/cube contact pairs, a 1 mm fail-closed penetration
gate, and bilateral-contact evidence. A 35% gripper target with a -15 mm grasp offset passed 60/60
pre-collection seeds; the accepted 30-episode teacher dataset contains 19,800 frames and 0.309 mm maximum
penetration. Its wrist-only student was verified to contain no top image.

Neither a 10,000-update new-from-base student nor a 5,000-update adaptation of the selected v2 policy
completed its held-out set. The control run retained the original selected v2 checkpoint and changed only
the contact physics: it completed 11/20 unseen episodes at seeds 2100 through 2119 while staying at or
below 0.663 mm penetration. Keep that original checkpoint selected, treat both corrected-grasp students as
failure-analysis artifacts, and keep the 80% release gate closed. Action traces now also contain cube,
gripper, and tray positions for direct lift/transfer diagnosis.

A follow-up failure analysis found that the evaluator reset pose did not match the IK collector. Matching
the teacher's `[0,-45,17.5,90,0,100]` pose raised the same checkpoint from 20% to 60%. A 25-step action
execution horizon scored 80% on the configuration-selection seeds but only 40% on a fresh 20-episode set,
so that 80% result is validation rather than a final claim.

The v2 run added 60 successful IK episodes (39,600 frames, seeds 1000 through 1059) and fine-tuned the
selected checkpoint for 10,000 updates at batch 4. It scored 80% on validation seeds 1100 through 1109
and 70% on each of two distinct unseen 20-episode sets, seeds 1200 through 1219 and 1400 through 1419.
Another 5,000 updates reduced validation success to 50%, so the earlier checkpoint remains selected.
The verified bounded result is therefore 70%, still below the 80% release threshold.

After evaluation, copy metrics from LeRobot's `eval_info.json` into the student provenance sidecar
without retyping them:

~~~bash
uv run python examples/so101_mujoco/record_wrist_vla_evidence.py \
  --student-root "<WRIST_STUDENT_DATASET>" \
  --checkpoint "<PRETRAINED_MODEL_DIR>" \
  --evaluation-output "<LEROBOT_EVAL_OUTPUT>" \
  --training-updates 10000 \
  --batch-size 4 \
  --dataset-episodes 30 \
  --dataset-frames 19800 \
  --training-seed-start 400 \
  --training-seed-end 429 \
  --evaluation-seed-start 800 \
  --evaluation-action-steps 25 \
  --evaluation-home-action 0 -45 17.5 90 0 100 \
  --evaluation-cube-xy-randomization-m 0.025
~~~

The sidecar records `vla_trained=true` and `vla_evaluated=true` separately from
`vla_success_threshold_met=false`. It also stores the evaluation reset pose, action execution horizon,
and cube XY range so a tuned rollout cannot be compared against a different task distribution without
leaving evidence. Physical-rollout and camera-alignment claims remain false.

## Camera profile boundary

The wrist camera pose is derived from TheRobotStudio's integrated 32x32 UVC module mount surface and is
attached to the fixed `gripper` body, not the moving jaw. `assets/camera_profiles.json` records its source
revision and hash. The physical lens center, module orientation, image rotation, intrinsics, and FOV have
not been measured on this PC, so the profile deliberately reports `physical_alignment=false`. Calibrate
those fields against the installed module before treating simulation pixels as real-camera-equivalent.

Run the read-only physical gate before any real-camera session:

~~~bash
uv run python examples/so101_mujoco/audit_physical_wrist_gate.py \
  --expected-camera-name "<OPERATOR_CONFIRMED_CAMERA_NAME_PART>" \
  --output "<NEW_RECEIPT_PATH>"
~~~

The audit only lists video and stable serial nodes. It never opens a device or sends a motor command, and
it exits with status 2 when the expected wrist camera, stable serial path, or physically verified camera
profile is missing. A ready receipt still requires operator validation and never authorizes motion.

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
