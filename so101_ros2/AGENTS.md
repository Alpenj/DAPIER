# SO-101 ROS 2 agent rules

These rules add to the repository root `AGENTS.md` for this directory.

- Core math, message-contract tests, mock ROS graphs, and builds without a hardware
  interface are allowed by default.
- Do not enable teleoperation or publish trajectories, JointState commands, torque
  requests, or actuator commands to a graph that can reach physical motors without the
  root hardware approval gate.
- Do not open a serial port or implement/run a hardware interface against a real device
  without the same gate. An enable service is not an E-stop or torque-off guarantee.
- Require an explicit SIM/MOCK namespace or isolated domain before automated ROS tests;
  treat an unknown ROS domain as HW.
- Do not add a self-hosted runner or hardware discovery to CI.

## Code Review Rules

- Flag ROS publishers, services, controllers, or future hardware interfaces that can
  move a robot without freshness checks, bounded commands, watchdog behavior, and an
  explicit human gate.
- Mock tests must not fall back to the host ROS graph or attached serial devices.
