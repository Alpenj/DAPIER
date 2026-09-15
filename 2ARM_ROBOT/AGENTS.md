# Mobile dual-arm agent rules

These rules add to the repository root `AGENTS.md` for this directory.

- MuJoCo-only models, offline data validation, static analysis, and tests that cannot
  connect to ROS or serial devices are allowed by default.
- New hardware-agnostic Python research code belongs in `2ARM_ROBOT/research/`.
  It may create versioned control intents, but it must not import low-level motor or
  serial SDKs, open device paths, authorize hardware, or publish direct actuator commands.
- Runtime perception and planning must not use MuJoCo object body poses, geom IDs, or
  segmentation labels to create grasp/IK targets. Use rendered sensor observations and
  transforms that have a physical calibration/TF equivalent. Simulator truth is limited
  to reset, reward, offline labels, and test-only error measurement.
- Visual SLAM, localization quality, map/session identity, reset generation, and TF adapter
  logic belong in the C++ localization packages under `so101_ros2/dapier_localization_*`.
  Localization code must not publish actuator commands, open motor/serial devices, or
  authorize hardware. Its language-neutral boundary is
  `contracts/localization_runtime_v1.json`.
- Real-time command validation, limits, watchdogs, safe stop, and hardware I/O belong in
  the C++ control packages under `so101_ros2/dapier_so101_*`. Python and localization
  packages may not bypass those gates.
- The Python-to-control boundary is `contracts/research_realtime_control_v1.json`.
  Change contract JSON first and regenerate the corresponding C++ header instead of
  editing generated values.
- A localization `relocalized`, `lost`, or map/reset-generation change must invalidate
  stale navigation goals, shoe targets, and action chunks before motion resumes.
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
- Flag Python research changes that acquire direct hardware dispatch responsibility,
  localization changes that acquire actuator authority, or C++ real-time changes that
  embed/launch Python or Visual SLAM inside the bounded command loop.
- Flag any runtime IK/grasp target derived from simulator truth rather than RGB/RGB-D
  perception, calibrated transforms, and an explicit confidence/uncertainty gate.
- Flag localization consumers that ignore tracking state, receiver-local TTL, covariance,
  or map/reset generation when accepting navigation or manipulation motion.
