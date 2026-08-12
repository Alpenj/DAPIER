#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH=$(readlink -f -- "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)
# shellcheck source=tb3_real_env.sh
source "$SCRIPT_DIR/tb3_real_env.sh"
set -u

echo "[1/8] Jetson reachability: $TURTLEBOT3_NANO_IP"
if ! ping -c 1 -W 2 "$TURTLEBOT3_NANO_IP" >/dev/null; then
  echo "ERROR: Jetson Nano is not reachable at $TURTLEBOT3_NANO_IP" >&2
  exit 1
fi

echo "[2/8] Resetting the ROS 2 daemon for domain $ROS_DOMAIN_ID / CycloneDDS"
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true
if ! timeout 10 ros2 daemon start >/dev/null; then
  echo "ERROR: ROS 2 daemon did not start within 10 seconds." >&2
  exit 1
fi

echo "[3/8] Waiting for /cmd_vel subscriber"
subscriber_count=0
plain_twist_endpoint=0
for _ in {1..60}; do
  topic_info=$(timeout 3 ros2 topic info /cmd_vel -v 2>/dev/null || true)
  subscriber_count=$(awk '/Subscription count:/ {print $3}' <<<"$topic_info")
  if rg -q '^Topic type: geometry_msgs/msg/Twist$' <<<"$topic_info"; then
    plain_twist_endpoint=1
  fi
  if [[ "${subscriber_count:-0}" -eq 1 && "$plain_twist_endpoint" -eq 1 ]]; then
    break
  fi
  sleep 1
done
if [[ "${subscriber_count:-0}" -ne 1 ]]; then
  echo "ERROR: /cmd_vel must have exactly one robot subscriber; found ${subscriber_count:-0}." >&2
  exit 1
fi

if [[ "$plain_twist_endpoint" -ne 1 ]]; then
  echo "ERROR: /cmd_vel has no geometry_msgs/msg/Twist endpoint for the Humble robot." >&2
  exit 1
fi

echo "[4/8] Waiting for unique, monotonic live odometry"
odom_publishers=0
odom_endpoint=0
for _ in {1..60}; do
  odom_info=$(timeout 3 ros2 topic info /odom -v 2>/dev/null || true)
  odom_publishers=$(awk '/Publisher count:/ {print $3}' <<<"$odom_info")
  if rg -q '^Topic type: nav_msgs/msg/Odometry$' <<<"$odom_info"; then
    odom_endpoint=1
  fi
  if [[ "${odom_publishers:-0}" -eq 1 && "$odom_endpoint" -eq 1 ]]; then
    break
  fi
  sleep 1
done
if [[ "${odom_publishers:-0}" -ne 1 ]]; then
  echo "ERROR: /odom must have exactly one publisher; found ${odom_publishers:-0}." >&2
  exit 1
fi
if [[ "$odom_endpoint" -ne 1 ]]; then
  echo "ERROR: /odom has no nav_msgs/msg/Odometry publisher." >&2
  exit 1
fi

if ! odom_audit=$(python3 "$SCRIPT_DIR/tb3_odom_audit.py" \
  --seconds 4 --min-samples 20 --strict 2>&1); then
  echo "$odom_audit" >&2
  echo "ERROR: /odom timestamp audit failed." >&2
  exit 1
fi
if ! rg -q '^samples=[0-9]+ .* regressions=0 ' <<<"$odom_audit"; then
  echo "$odom_audit" >&2
  echo "ERROR: /odom is not safe for Cartographer." >&2
  exit 1
fi
rg '^samples=' <<<"$odom_audit"

echo "[5/8] Waiting for odom -> base_footprint TF"
if ! tf_audit=$(python3 "$SCRIPT_DIR/tb3_tf_probe.py" odom base_footprint \
  --seconds 30 --min-updates 2 2>&1); then
  echo "$tf_audit" >&2
  echo "ERROR: odom -> base_footprint TF is not live." >&2
  exit 1
fi
if ! rg -q '^target=odom source=base_footprint updates=([2-9]|[1-9][0-9]+) regressions=0 invalid=0$' <<<"$tf_audit"; then
  echo "$tf_audit" >&2
  echo "ERROR: unexpected odom -> base_footprint TF audit output." >&2
  exit 1
fi
rg '^target=' <<<"$tf_audit"

echo "[6/8] Checking multiple OpenCR battery samples"
if ! opencr_audit=$(python3 "$SCRIPT_DIR/tb3_opencr_probe.py" \
  --seconds 10 --min-samples 3 --min-voltage 11.1 2>&1); then
  echo "$opencr_audit" >&2
  echo "ERROR: OpenCR battery/torque audit failed; do not drive the robot." >&2
  exit 1
fi
if ! rg -q '^battery_samples=([3-9]|[1-9][0-9]+) torque_samples=([3-9]|[1-9][0-9]+) min_voltage=[0-9]+\.[0-9]{3} torque=true$' <<<"$opencr_audit"; then
  echo "$opencr_audit" >&2
  echo "ERROR: unexpected OpenCR audit output." >&2
  exit 1
fi
rg '^battery_samples=' <<<"$opencr_audit"
battery_voltage=$(awk -F'[ =]' '/^battery_samples=/ {print $6}' <<<"$opencr_audit")

echo "[7/8] Dynamixel torque stayed enabled across the same samples"

echo "[8/8] Checking live LiDAR scans"
scan_info=$(timeout 3 ros2 topic info /scan -v 2>&1 || true)
scan_publishers=$(awk '/Publisher count:/ {print $3}' <<<"$scan_info")
if [[ "${scan_publishers:-0}" -ne 1 ]] || \
  ! rg -q '^Topic type: sensor_msgs/msg/LaserScan$' <<<"$scan_info"; then
  echo "$scan_info" >&2
  echo "ERROR: /scan must have exactly one sensor_msgs/msg/LaserScan publisher." >&2
  exit 1
fi
if ! scan_audit=$(python3 "$SCRIPT_DIR/tb3_scan_probe.py" \
  --seconds 4 --min-samples 10 2>&1); then
  echo "$scan_audit" >&2
  echo "ERROR: LiDAR scan audit failed." >&2
  exit 1
fi
if ! rg -q '^scans=[0-9]+ duplicates=0 regressions=0 invalid=0 finite_points=[1-9][0-9]*$' <<<"$scan_audit"; then
  echo "$scan_audit" >&2
  echo "ERROR: LiDAR scans are not fresh and valid." >&2
  exit 1
fi
rg '^scans=' <<<"$scan_audit"

echo "OK: robot link, Twist, odometry, TF, battery (minimum ${battery_voltage}V), motor torque, and LiDAR are healthy."
