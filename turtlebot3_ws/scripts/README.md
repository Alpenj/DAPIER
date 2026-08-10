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
