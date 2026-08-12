# scripts

Standalone helper scripts written while working through the TurtleBot3 SLAM/Nav2
exercise (2026-08-10), not part of any ROS2 package.

## gentle_explorer.py

Reactive waypoint explorer used to drive the robot around `turtlebot3_world`
during SLAM mapping (Part 1), as a stand-in for keyboard teleop. Publishes
`geometry_msgs/msg/TwistStamped` on `/cmd_vel`, steers toward a fixed list of
waypoints while avoiding obstacles via `/scan`, and reports `/map` coverage
every 10s.

Tuned to avoid the odometry drift / map "ghosting" that sharp turns and
reverse-while-spinning recovery maneuvers caused in earlier iterations: lower
speeds, a larger obstacle safety margin, rate-limited (smoothed) velocity
commands, and a gentle in-place-turn recovery instead of reverse+spin.

```bash
source /opt/ros/jazzy/setup.bash
source ~/DAPIER/turtlebot3_ws/install/setup.bash
python3 gentle_explorer.py 320   # run for 320 seconds
```

Requires Gazebo + Cartographer already running (see the main learning log for
the full launch sequence).

## Physical Burger teleop

The Jetson Nano runs TurtleBot3 Humble in Docker on `192.168.0.253`, while this
PC runs Jazzy. Use the dedicated wrapper so the exam-time
`ROS_LOCALHOST_ONLY=1` setting is overridden only for this terminal and the
robot receives plain `geometry_msgs/msg/Twist` rather than Jazzy's default
`TwistStamped`.

Keep the wheels clear on the first run, then execute:

```bash
tb3-ready
tb3-teleop
```

Routine commands run `tb3-ready` automatically. It checks the same typed live
signals concurrently without restarting the ROS daemon: one physical endpoint,
monotonic odometry, live TF, at least 11.1V, and enabled Dynamixel torque.
SLAM/Nav2 mode also requires fresh valid LiDAR scans. It returns as soon as the
minimum live samples arrive. `tb3-teleop` does not wait for LiDAR because manual
driving does not use it.

Use the slower `tb3-check` after power cycling, a motor LED/alarm, DDS endpoint
confusion, or any failed `tb3-ready`. It resets ROS discovery and performs a
longer sequential audit. `tb3-restart` intentionally keeps this full check.
A powered-off, stale, low-voltage, or torque-off robot cannot pass either gate.

If the short commands are missing after a fresh clone, install their symlinks
once and build the tracked real-robot package:

```bash
~/DAPIER/turtlebot3_ws/scripts/install_tb3_commands.sh
cd ~/DAPIER/turtlebot3_ws
colcon build --symlink-install --packages-select dapier_turtlebot3_real
```

The teleop uses the Burger specification while keeping mapping headroom.
`w/x` changes linear speed in `0.02 m/s` steps up to `+/-0.18 m/s`;
`a/d` changes steering by `0.15 rad/s` while the robot is moving, up to
`1.10 rad/s`; and `r` straightens without stopping. In-place rotation is
limited separately to `1.50 rad/s`. At the `0.22 m/s` sport limit, steering
automatically lowers body speed just enough to turn. Combined commands keep
each estimated wheel at or below `0.22 m/s` and the inner wheel at or above
20% of the outer wheel. `s` or Space stops, and Ctrl-C sends three zero
commands before exiting.

The default is mapping mode. Use `tb3-teleop --sport` to raise only the
linear limit to the Burger's official `0.22 m/s` maximum. Sport mode is for
open-floor driving, not for building a clean map.

## Physical Burger SLAM and map save

Use three terminals. The first runs Cartographer and RViz, the second drives
the robot, and the third checks and saves the map:

```bash
# Terminal 1
tb3-slam

# Terminal 2
tb3-teleop

# Terminal 3, after some slow driving
tb3-slam-check
tb3-map-save my_room
```

Maps are saved as `~/maps/<name>.yaml` and `~/maps/<name>.pgm`. Existing
maps are never overwritten. Save before pressing Ctrl-C in the SLAM terminal.
If only the OpenCR or motor power was cycled while the Jetson stayed on, stop
SLAM and run `tb3-restart` once before starting `tb3-slam` again.

After saving, stop teleop and SLAM, then start localization and navigation:

```bash
tb3-nav my_room
```

The tracked `dapier_turtlebot3_real` package uses a typed physical-only Nav2
YAML: all command nodes use plain `Twist` for the Humble robot and autonomous
linear motion is limited to `0.18 m/s`. The requested map path is injected
into `map_server`; the upstream example `map.yaml` is not used. In RViz, set
**2D Pose Estimate**, run `tb3-nav-check my_room`, and only then set **Nav2
Goal**. The check reruns the live robot/LiDAR gates, verifies that `map_server`
loaded that exact YAML, that the live map dimensions/resolution match its PGM
pair, and that Nav2/TF plus `/navigate_to_pose` are active.

Before clicking **Nav2 Goal**, run the following in one more terminal:

```bash
tb3-nav-watch
```

It ignores status retained from old goals, locks the next RViz goal UUID, prints
its transitions, and exits successfully only for `SUCCEEDED`. `CANCELED`,
`ABORTED`, no action server, and timeout all return a nonzero exit status.

For an ordered tour, repeat `--pose X Y YAW_DEG` for every map-frame target.
The command intentionally plans without moving unless `--execute` is present:

```bash
# Read-only route validation: catches occupied, unknown, or disconnected poses.
tb3-waypoints --pose 1.0 0.0 0 --pose 2.0 0.0 180

# Physical execution: reruns the robot gate and the same full-route plan first.
tb3-waypoints --execute --pose 1.0 0.0 0 --pose 2.0 0.0 180
```

This two-stage syntax exists so copying or checking coordinates cannot move the
robot accidentally. Execution succeeds only when the FollowWaypoints action
returns `SUCCEEDED`, error code zero, and an empty missed-waypoint list.
