# SO-101 hardware tools agent rules

These rules add to the repository root `AGENTS.md` for this directory.

- Source review, schema validation, fixtures, mocks, and unit tests that never open a
  device are allowed by default.
- Files under `read_only/` still count as physical access when they open a serial port.
  Do not run them without the root hardware approval gate.
- Do not run anything under `writes_hardware/`, including inspect modes that connect to
  a bus, without the root hardware approval gate.
- Never enable or disable torque, unlock EEPROM, change homing offsets, operating mode,
  position limits, calibration, or restore data on an agent's own initiative.
- Do not use a discovered `/dev/tty*` path, serial ID, or calibration file as implicit
  authorization. Keep device identity and personal calibration out of Git and logs.
- Hardware execution must be interactive, name the exact role and device profile, and
  stop on an unexpected motor ID, model, voltage, temperature, load, or torque state.

## Code Review Rules

- Flag any path that can reach `Bus.write`, torque control, EEPROM unlock/write, or
  calibration replacement without both an exact confirmation string and a human gate.
- Tests for write-capable code must replace the bus/serial layer with a fake.
