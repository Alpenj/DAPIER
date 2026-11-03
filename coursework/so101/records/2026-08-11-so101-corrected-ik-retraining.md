# SO-101 corrected IK demonstrations and wrist-only VLA retraining

## Why I reopened the grasp

The successful SO-101 MuJoCo replay still showed the finger geometry entering
the 50 mm cube. I measured the contact rather than treating it as a rendering
artifact. The old 27% gripper target produced about 8.3 mm of penetration, so I
kept the physical-readiness claim closed and rebuilt the simulation teacher.

I used AI assistance to inspect the MuJoCo contacts, run bounded parameter
sweeps, verify datasets, drive the training commands, and summarize the action
traces. I retained executable gates and saved artifacts as the source of the
result; no physical robot command was issued.

## Contact and teacher correction

Closing the gripper less was not enough by itself: a 35% command avoided the
deep intersection but initially failed to lift. The successful corrected
combination is:

- gripper target: 35%;
- grasp target Z offset: -15 mm;
- explicit pad/cube contact friction: `[1.6, 1.6, 0.02, 0.001, 0.001]`;
- direct-format contact stiffness/damping: `[-200000, -400]`;
- accepted pad penetration: at most 1 mm, with bilateral pad contact required.

The five-value friction array explicitly supplies both tangential axes before
torsional and rolling terms. An earlier trial accidentally left the second
tangential coefficient at 0.02 and failed to lift. Friction 1.2 passed the
initial sweep but failed seed 2027; 1.6 was the smallest tested value that
passed the follow-up. A 60-seed pre-collection sweep at seeds 2000 through 2059
then completed 60/60 with 0.310 mm maximum penetration.

The environment now creates named collision pairs between each visible finger
pad and the cube. Environment info and the RCS-compatible action trace report
bilateral contact and maximum penetration. The trace also records cube,
gripper, and tray positions so lift and transfer can be diagnosed without
inferring them from video alone.

## Fail-closed dataset collection

The collector refuses an episode when task success is false, no bilateral
contact occurred, or penetration exceeded 1 mm. A first 30-episode attempt at
friction 1.2 collected 27 episodes and then failed seed 2027. I preserved that
partial dataset as rejected evidence and did not write a completed validation
manifest for it.

The accepted teacher dataset used seeds 2000 through 2029 and contains 30/30
successful episodes and 19,800 frames. Its mean top-RGB XY error is 0.764 mm,
maximum XY error is 1.245 mm, and maximum observed pad penetration is 0.309 mm.
The validation sidecar records the teacher and contact parameters and states
`simulation_only=true` and `physical_grasp_verified=false`.

I derived the student dataset by removing only
`observation.images.top`. Wrist RGB, measured state, action, task, and the
teacher-contract hash remain. Reloading the derived LeRobot dataset confirmed
30 episodes, 19,800 frames, and no privileged top image.

## First corrected student result

I trained a new SmolVLA student for 10,000 updates at batch 4. Four persistent
data-loader workers reduced measured data time from about 0.252 s/update to
0.003 s/update; the run completed in 21 minutes 32 seconds, saw 40,000 samples
(about 2.02 passes), and ended at loss 0.017 with 1.69 GB reported GPU memory.

The new-from-base checkpoint did not reproduce the full long-horizon sequence.
It scored 0/20 on unseen seeds 2100 through 2119 and 0/5 on five training seeds.
The unseen run made bilateral contact for 804 frames and stayed below 0.509 mm
penetration, but never completed the lift-and-place task. Disabling action
smoothing on the five training seeds also scored 0/5, ruling out the existing
slew filter as the primary cause.

A time-aligned teacher/policy comparison and rollout montage showed that the
policy approached and touched the cube, then executed later arm motion without
retaining the cube. It also skipped or phase-shifted the initial observation
motion. This is a sequence-learning failure, not a return of the original deep
penetration defect.

## Existing-policy correction pass

I initialized a bounded 5,000-update pass from the previously selected v2 wrist
policy, which had learned the overall approach/lift/transfer structure. It saw
20,000 corrected samples, about 1.01 dataset passes, and ended at loss 0.024.
This checkpoint recovered partial lifts up to 30 mm but still scored 0/10 on
unseen seeds 2100 through 2109. It is retained as a failure-analysis artifact,
not selected for use.

The decisive control was the unmodified selected v2 policy under the corrected
contact physics. On the same seed range it scored 5/10, and another unseen ten
episodes at seeds 2110 through 2119 scored 6/10. The combined result is 11/20
(55%), with 0.663 mm maximum pad penetration. This is below the 80% release
gate, but it preserves task ability while reducing the previously measured
8.3 mm simulation penetration below 1 mm.

I therefore keep the original v2 checkpoint as the selected simulation policy
and select the new contact model, not either 35%-grasp retraining checkpoint.
The result also changes the interpretation of the old 27% action: with stiff,
visible contact proxies it behaves as a position-servo preload request while
the joint is physically blocked by the cube, rather than allowing geometry to
pass through it. Whether that preload is acceptable for the real servo still
requires a current-limited physical test; simulation does not authorize it.

## Physical boundary

All evidence here is from MuJoCo. The 1 mm gate is a simulation-quality gate,
not a guarantee that the printed fingers, servo calibration, backlash, camera,
or cube friction match the real assembly. Before torque-on I still need stable
serial identities, camera calibration, read-only joint inventory and zero
checks, an emergency-stop path, low-speed single-joint verification, and a new
physical demonstration dataset. Corrected simulation IK data can bootstrap the
student, but it does not replace real corrective demonstrations.
