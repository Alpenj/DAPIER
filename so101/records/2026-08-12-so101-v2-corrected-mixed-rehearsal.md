# SO-101 v2 + corrected IK mixed rehearsal

This run contains one SO-101 arm and one task. References to two-pad contact
mean the two pads of its single gripper, not two robot arms. Dataset task
metadata contains only `Pick up the blue cube and place it in the green tray.`
No casino or dual-arm data was included.

The corrected IK collector completed seeds `3000..3099`: 100/100 successful
episodes, 66,000 frames, 0.843 mm mean and 1.500 mm maximum top-RGB XY error,
and 0.342 mm maximum pad penetration. Removing only the top image retained 100
wrist-only episodes.

The existing v2 dataset has 60 episodes and 39,600 frames. Both sources have
identical 30 FPS state, action, reward, success, done, and 120 x 160 wrist-image
contracts. Their merged rehearsal dataset reopened with 160 episodes, 105,600
frames, and no top image. The mix is 37.5% existing v2 and 62.5% corrected IK.

Training initialized from selected v2. A 10,000-update pass at peak LR `1e-5`
completed in 20:41 with loss 0.035. A 15,000-update continuation at peak LR
`5e-6` completed in 32:25 with loss 0.028. Together they processed 100,000
samples, about 0.95 merged-dataset passes.

On unseen seeds `3200..3209`, the unchanged v2 baseline scored 5/10. Mixed
checkpoints at 2,500, 5,000, 7,500, 10,000, 15,000, 20,000, and 25,000 total
updates scored 0/10, 1/10, 3/10, 3/10, 2/10, 2/10, and 2/10. No mixed model
was promoted; selected simulation policy remains original v2.

Mixed traces stayed below about 0.58 mm penetration. The regression was
two-pad contact retention and long-horizon phase execution, not renewed
geometry penetration. I used AI assistance for contract checks, bounded
commands, checkpoint comparisons, and trace summaries. No physical command
was issued. Simulation data does not replace real calibration, current limits,
emergency stop checks, printed-finger validation, or real demonstrations.
