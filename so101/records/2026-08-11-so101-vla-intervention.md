# SO-101 VLA human intervention and grasp-penetration analysis

## Why I added intervention

The wrist-only VLA could be watched in MuJoCo but could not be corrected by a
person during rollout. I used AI assistance to inspect the evaluator boundary,
implement a small authority state machine, add source-labeled evidence, and run
the executable checks. No physical motor or camera was opened.

The viewer route now starts under policy authority. `Space` transfers authority
to the keyboard and anchors its target to the measured six-axis state. Manual
Cartesian, gripper, and selected-joint controls then own the requested action.
`Enter` restores policy authority and calls `policy.reset()`, which drops the
unconsumed part of the old 25-action chunk before new inference.

The unattended `--no-viewer` route still uses LeRobot's standard evaluator.

## Evidence contract

Each intervention attempt writes an episode directory containing:

- `episode.json` with seed, task, camera and joint order;
- `events.jsonl` with policy/human source, intervention segment, measured state,
  requested action, filter-applied action, last policy action, reward and done;
- one wrist RGB PNG per recorded frame;
- `manifest.json` with frame counts, intervention counts, success and termination.

The manifest deliberately reports
`evidence_only_requires_dataset_conversion`. I have not claimed that these
files are already a native LeRobot training dataset.

I ran a six-step CUDA smoke with the selected 10,000-update SmolVLA checkpoint.
The scripted authority sequence was
`policy, human, human, policy, policy, policy`. The saved manifest reported one
intervention segment, two human frames and six total frames; all six wrist PNGs
and all applied-action fields were present. The complete SO-101 MuJoCo source
test suite passed 38/38 with the existing Gymnasium action-space warning.

## Why the fingers visibly enter the cube

I replayed successful episode 0 from the filtered seed-1600 evaluation and
measured MuJoCo contact distances instead of judging the screenshot alone. The
finger-pad/cube contact had maximum penetration of 8.314 mm and median
penetration of 3.258 mm.

The root cause is primarily the grasp command rather than the camera. The cube
is 50 mm wide. At the teacher's closed-gripper target near 27%, the simulated
pad gap is only 39.1 mm. At 35%, the free gap is 49.3 mm. Contact-stiffness and
gripper-force sweeps did not remove the penetration, while a 35% runtime clamp
made the selected policy miss the placement. The current checkpoint therefore
depends on an over-closed teacher convention.

I did not move collision geometry merely to make the replay look cleaner. The
correct follow-up is to change the IK teacher grasp target, recollect successful
demonstrations, train a new wrist-only student, and compare both penetration and
held-out task success. Human intervention evidence can help identify and retain
the corrected grasp portion, but it still needs an explicit dataset conversion
and retraining step.

## Physical boundary

This work is simulation-only. A 35% simulated gap is not a hardware torque or
contact limit. Physical assembly, calibration, current limits, emergency stop,
low-speed joint checks, and camera alignment remain separate gates before any
real rollout.
