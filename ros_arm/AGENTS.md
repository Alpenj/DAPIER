# ROS arm agent rules

These rules add to the repository root `AGENTS.md` for this directory.

- RViz, URDF, Gazebo, static analysis, and tests that cannot open a serial device are
  allowed by default.
- Do not launch `ros_arm_bridge`, open an Arduino/USB serial port, publish a physical
  joint command, or upload firmware without the root hardware approval gate.
- Treat opening an Arduino serial connection as a physical action because it resets the
  board and the bridge can transmit servo commands.
- Do not run `sudo`, change device permissions, or select `/dev/ttyUSB*` automatically.
- Keep simulation publishers isolated from a live ROS graph; an ambiguous graph is HW.

## Code Review Rules

- Flag serial open/write or ROS command paths that can move SG90/MG90 servos without an
  explicit human gate, bounded joint limits, and a safe no-command startup state.
- Tests must mock `serial.Serial` and ROS publishers that could reach hardware.
