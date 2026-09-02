# dapier_safety_bridge

ROS 2 adapter that combines sensor-derived localization, measured robot state,
task interlocks and a Python/research `ControlIntent`, then publishes only a
sanitized `/dapier/safe_command`.

The node does **not** publish `/cmd_vel`, joint trajectories, ros2_control
commands, serial data or motor registers. A separately reviewed downstream sink
must consume `SafeCommand` after local commissioning.

## Topics

### Subscriptions

| Topic | Type | Meaning |
|---|---|---|
| `/dapier/research_intent` | `dapier_interfaces/ResearchControlIntent` | non-authorizing arm/base/hold proposal |
| `/dapier/localization_estimate` | `dapier_interfaces/LocalizationEstimate` | sensor-derived pose, quality and identity |
| `/dapier/replan_acknowledgement` | `dapier_interfaces/ReplanAcknowledgement` | exact plan reset after loss/relocalization |
| `/joint_states` | `sensor_msgs/JointState` | measured dual-arm state |
| `/odom` | `nav_msgs/Odometry` | measured base velocity |
| `/dapier/interlocks` | `dapier_interfaces/InterlockState` | collision, carry, grasp and task-phase facts |
| `/dapier/operator_enable` | `std_msgs/Bool` | explicit operator motion enable |
| `/dapier/estop_healthy` | `std_msgs/Bool` | read-back of external E-stop health |

### Publications

| Topic | Type | Meaning |
|---|---|---|
| `/dapier/safe_command` | `dapier_interfaces/SafeCommand` | sanitized command or hold/safe-stop result |
| `/dapier/safety_status` | `dapier_interfaces/SafetyStatus` | observable bridge state and rejection reason |

### Service

`/dapier/acknowledge_safe_stop` (`std_srvs/Trigger`) clears a latched software
safe stop only when operator motion is disabled and E-stop health is true.
Operator enable must then be asserted again.

## Build and start

```bash
cd ~/DAPIER/so101_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to dapier_safety_bridge
source install/setup.bash
ros2 launch dapier_safety_bridge safety_bridge.launch.py
```

The initial state is fail-closed. No motion command can pass until fresh joint
state, odometry, interlocks, localization, replan acknowledgement when required,
E-stop health and operator enable are all present.

## Hardware boundary

`SafeCommand.hardware_execution` is always false in this node. The later
hardware sink must verify device identity, calibration, applied command
read-back, controller watchdog behavior and E-stop/torque-off semantics locally.
