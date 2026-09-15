# Mobile dual-arm agent rules

These rules add to the repository root `AGENTS.md` for this directory.

- PGripper learning follows execution → failure analysis → corrective demonstrations →
  retraining, not unlimited updates on unchanged data. Before further training, check
  expert replay under the policy's actual control clock and safety limits. Preserve
  failures, keep evaluation data separate, and read the current decisions in
  `sim/mobile_dual_so101/LEARNING_STRATEGY_ACT_FIRST_KO.md`.

- MuJoCo-only models, offline data validation, static analysis, and tests that cannot
  connect to ROS or serial devices are allowed by default.
- Do not run hardware teleop, motor jog, torque control, EEPROM/register writes,
  hardware snapshot scripts, or actuator publishers without the root hardware approval
  gate. Read-only snapshots still require approval when they contact physical devices.
- Treat `/cmd_vel`, JointTrajectory, JointState command publishers, and any unknown ROS
  graph as physical until its SIM/MOCK isolation is demonstrated.
- Never reuse committed example serial IDs or calibration files as live device identity.
- Do not add a self-hosted runner or make CI discover attached hardware.

## Code Review Rules

- Flag new mobile-base or arm command paths that lack an explicit human gate,
  bounded motion, or a short execution time limit. Apply the root distinction between
  attended manual recording and autonomous operation; do not reintroduce a mandatory
  independent watchdog/communication-loss stop verification gate for manual recording.
- Simulation entrypoints must remain usable without importing or opening hardware APIs.
