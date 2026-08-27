# Mobile dual-arm agent rules

These rules add to the repository root `AGENTS.md` for this directory.

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

- Flag new mobile-base or arm command paths that lack an explicit human gate, bounded
  motion, timeout/watchdog, and fail-safe stop behavior.
- Simulation entrypoints must remain usable without importing or opening hardware APIs.
