# Mobile dual-arm agent rules

These rules add to the repository root `AGENTS.md` for this directory.

- MuJoCo-only models, offline data validation, static analysis, and tests that cannot
  connect to ROS or serial devices are allowed by default.
- New hardware-agnostic Python research code belongs in `2ARM_ROBOT/research/`.
  It may create versioned control intents, but it must not import low-level motor or
  serial SDKs, open device paths, authorize hardware, or publish direct actuator commands.
- Real-time command validation, limits, watchdogs, safe stop, and hardware I/O belong in
  the C++ packages under `so101_ros2/`. Python may not bypass those gates.
- The cross-language boundary is `contracts/research_realtime_control_v1.json`.
  Change the JSON first and regenerate the C++ header instead of editing generated values.
- Do not run hardware teleop, motor jog, torque control, EEPROM/register writes,
  hardware snapshot scripts, or actuator publishers without the root hardware approval
  gate. Read-only snapshots still require approval when they contact physical devices.
- Treat `/cmd_vel`, JointTrajectory, JointState command publishers, and any unknown ROS
  graph as physical until its SIM/MOCK isolation is demonstrated.
- Never reuse committed example serial IDs or calibration files as live device identity.
- TurtleBot3 public configuration may contain `TB3_USER=user`, but must never contain a
  password, token, private key, or automated password-input mechanism. Use a Git-ignored
  local env file and SSH key authentication.
- Do not add a self-hosted runner or make CI discover attached hardware.

## Code Review Rules

- Flag new mobile-base or arm command paths that lack an explicit human gate, bounded
  motion, timeout/watchdog, and fail-safe stop behavior.
- Simulation entrypoints must remain usable without importing or opening hardware APIs.
- Flag Python research changes that acquire direct hardware dispatch responsibility or
  C++ real-time changes that embed/launch Python inside the control loop.
