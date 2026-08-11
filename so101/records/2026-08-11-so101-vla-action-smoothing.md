# SO-101 wrist-only VLA action smoothing and RCS trace

## Why I ran this follow-up

The selected wrist-only SmolVLA checkpoint could complete the MuJoCo task, but
its recorded rollouts visibly shook. I treated that as a control-interface
problem separate from policy success: the model emits a 25-action chunk, and a
new chunk can begin with a discontinuous target even when the previous chunk
was locally smooth.

I used AI assistance to inspect the action path, summarize demonstration
deltas, implement the bounded filter and tests, and compare traces. I kept the
acceptance decision tied to executable rollouts and saved evidence rather than
to the visual impression alone.

## Change

The VLA evaluation route now enables three controls while the generic MuJoCo
environment keeps them opt-in:

- blend the first 3 frames of every 25-frame action chunk from the last applied
  target;
- limit per-frame target changes to
  `[1.75, 0.65, 0.30, 0.35, 0.12, 5.50]` in the LeRobot SO-101 convention;
- hold gripper changes smaller than 1 percentage point.

The first five limits are degrees per 30 Hz simulation frame and the last is
gripper percentage points per frame. They came from the maximum per-frame
changes measured in the verified 60-episode v2 IK teacher dataset, rounded up
slightly. They are simulation rollout stability ceilings, not physical motor
safety limits.

Every step can also write a JSONL trace containing the raw, bounded and applied
action, filter decisions, the commanded joint positions in radians, and the
synchronous post-action MuJoCo readback. The joint order, absolute-target
semantics, radians and contract hash follow the concepts adopted in DAPIER's
Robot Control Stack concept work. The trace keeps a globally increasing RCS
timestamp plus a reset-local episode timestamp.

RCS did not itself make the motion smoother. The chunk blend, slew limiter and
gripper deadband changed the motion. RCS concepts made the before/after result
measurable under one declared contract and prepared the simulation trace for a
later physical-readback comparison.

## Same-seed A/B

I evaluated the same checkpoint, reset pose, 25-action execution horizon,
700-step limit, ±25 mm cube randomization and seeds starting at 1500. Both the
unfiltered and filtered five-episode sets succeeded in 4/5 episodes.

Selected absolute per-frame target-delta results:

| Metric | Baseline | Filtered |
| --- | ---: | ---: |
| shoulder-pan chunk-boundary p95 | 4.268° | 1.357° |
| shoulder-pan chunk-boundary max | 9.405° | 1.750° |
| gripper chunk-boundary max | 12.887 points | 4.282 points |
| shoulder-pan command/sim p95 error | 0.00770 rad | 0.00691 rad |
| shoulder-pan command/sim max error | 0.11585 rad | 0.02226 rad |

The filtered run therefore reduced the discontinuity without lowering the
small A/B success count.

## New-seed regression

I then ran 20 new episodes starting at seed 1600. The filtered route completed
16/20 episodes before the 700-step limit, or 80% for this set. Its applied
per-frame target maxima were exactly bounded at
`[1.75, 0.65, 0.30, 0.35, 0.12, 5.50]`. The chunk-boundary shoulder-pan p95 was
1.411° and maximum was 1.750°; gripper chunk-boundary p95 was 1.889 points and
maximum was 3.607 points.

This one set reaches the 80% target, but the same checkpoint previously scored
14/20 on each of two other held-out sets. I therefore retain the broader claim:
the v2 policy is materially better than the original 20% baseline, while
generalization has not been demonstrated as a stable ≥80% result. The new
evidence does support enabling smoothing on the wrist-only VLA simulation
route.

The evaluator printed 80% after all 20 batches and the action trace contains
all 20 episodes. A terminal X11 `BadWindow` occurred during process cleanup,
after rollout completion, so that run did not persist `eval_info.json`. I used
the trace frame counts only because this environment terminates immediately on
success; four traces reached the full 700-step limit and sixteen ended early.
The source-level test suite passed 36/36 with the existing Gymnasium action
space warning.

## Reproduce the trace summary

`teleoperate.py --input policy --camera-set wrist-only --control-mode vla`
enables the filter and writes `action_trace.jsonl` under the requested output
directory. From a LeRobot checkout with the DAPIER overlay applied:

```bash
python examples/so101_mujoco/analyze_action_trace.py \
  <EVAL_OUTPUT>/action_trace.jsonl \
  --episode-length 700
```

The analyzer rejects timestamps that are not strictly increasing across the
trace.

## Physical boundary

No physical motor command was issued in this work. Mechanical assembly can
continue, but the simulation limits above must not be reused as hardware
safety limits. Before torque-on I still need a stable serial identity, camera
intrinsic/extrinsic alignment, read-only joint inventory and zero-offset check,
low-speed single-joint verification, an emergency-stop path, and a separately
approved physical trace. Only then can command, simulation and physical
readback be evaluated together with explicit thresholds.
