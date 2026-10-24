# dapier_safety_core

Hardware-agnostic C++ safety controller placed between Python/SLAM proposals and
any ROS or physical command sink.

## Enforced gates

- receiver-local intent TTL and monotonic sequence
- explicit operator enable and E-stop health
- measured-state freshness and command watchdog
- localization `Proceed` decision
- hard joint position, velocity and bounded-horizon delta limits
- base linear/angular limits
- settled-base interlock for arm motion
- carry-pose and verified-grasp interlocks for base transport
- collision-clear interlock
- latched safe stop with disabled explicit reset

The library emits a typed `SafeCommand`. It does not publish ROS messages, open
serial ports, or execute hardware. `RecordingMockCommandSink` accepts only
commands marked dispatchable and rejects any object claiming hardware execution.

The future hardware sink must still own torque-off behavior, device identity,
read-back verification, and the local human commissioning gate.
