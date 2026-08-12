#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH=$(readlink -f -- "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)
# shellcheck source=tb3_real_env.sh
source "$SCRIPT_DIR/tb3_real_env.sh"
set -u

if [[ "$#" -ne 1 ]]; then
  echo "Usage: tb3-nav-check <map_name|absolute_map_yaml>" >&2
  exit 2
fi
if [[ "$1" = /* ]]; then
  expected_yaml="$1"
else
  expected_yaml="$HOME/maps/${1%.yaml}.yaml"
fi
if [[ ! -s "$expected_yaml" ]]; then
  echo "ERROR: expected map YAML not found: $expected_yaml" >&2
  exit 1
fi
expected_yaml=$(readlink -f -- "$expected_yaml")
if ! expected_info=$(python3 "$SCRIPT_DIR/tb3_map_validate.py" --machine "$expected_yaml"); then
  exit 1
fi
expected_width=$(awk -F= '$1 == "width" {print $2}' <<<"$expected_info")
expected_height=$(awk -F= '$1 == "height" {print $2}' <<<"$expected_info")
expected_resolution=$(awk -F= '$1 == "resolution" {print $2}' <<<"$expected_info")

echo "[preflight] Rechecking the live robot, odometry, motor power, and LiDAR"
"$SCRIPT_DIR/tb3_real_check.sh"

required_nodes=(
  map_server
  amcl
  controller_server
  planner_server
  behavior_server
  bt_navigator
  velocity_smoother
  collision_monitor
)

echo "[1/6] Checking Nav2 lifecycle nodes"
inactive_nodes=()
for node in "${required_nodes[@]}"; do
  state=$(timeout 6 ros2 lifecycle get "/$node" 2>&1 || true)
  if ! rg -q '^active ' <<<"$state"; then
    inactive_nodes+=("$node")
  fi
done
if [[ "${#inactive_nodes[@]}" -gt 0 ]]; then
  echo "INFO: resuming partially active Nav2 nodes: ${inactive_nodes[*]}"
  # The navigation lifecycle can pause while the global costmap waits for the
  # first AMCL map->odom transform.  Resume only after proving that the user has
  # supplied a live 2D Pose Estimate; otherwise activation would hide the real
  # localization error.
  if ! initial_tf=$(python3 "$SCRIPT_DIR/tb3_tf_probe.py" map odom \
    --seconds 8 --min-updates 1 2>&1); then
    echo "$initial_tf" >&2
    echo "ERROR: set 2D Pose Estimate before resuming Nav2." >&2
    exit 1
  fi
  resume_result=$(timeout 30 ros2 service call \
    /lifecycle_manager_navigation/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes '{command: 2}' 2>&1 || true)
  if ! rg -q 'success=True' <<<"$resume_result"; then
    echo "$resume_result" >&2
    echo "ERROR: Nav2 lifecycle RESUME failed." >&2
    exit 1
  fi
  sleep 1
fi
for node in "${required_nodes[@]}"; do
  state=$(timeout 6 ros2 lifecycle get "/$node" 2>&1 || true)
  if ! rg -q '^active ' <<<"$state"; then
    echo "ERROR: /$node is not active after lifecycle check: $state" >&2
    exit 1
  fi
done

echo "[2/6] Checking the loaded map"
loaded_parameter=$(timeout 6 ros2 param get /map_server yaml_filename 2>&1 || true)
loaded_yaml=${loaded_parameter#String value is: }
if [[ "$loaded_yaml" == "$loaded_parameter" || ! -e "$loaded_yaml" ]]; then
  echo "ERROR: cannot read /map_server yaml_filename: $loaded_parameter" >&2
  exit 1
fi
loaded_yaml=$(readlink -f -- "$loaded_yaml")
if [[ "$loaded_yaml" != "$expected_yaml" ]]; then
  echo "ERROR: map_server loaded '$loaded_yaml', expected '$expected_yaml'." >&2
  exit 1
fi
map_info=$(timeout 6 ros2 topic echo /map --once --field info 2>&1 || true)
width=$(awk '/^width:/ {print $2; exit}' <<<"$map_info")
height=$(awk '/^height:/ {print $2; exit}' <<<"$map_info")
resolution=$(awk '/^resolution:/ {print $2; exit}' <<<"$map_info")
if [[ -z "$width" || -z "$height" || -z "$resolution" ]]; then
  echo "$map_info" >&2
  echo "ERROR: no valid /map metadata arrived." >&2
  exit 1
fi
if [[ "$width" != "$expected_width" || "$height" != "$expected_height" ]]; then
  echo "ERROR: /map is ${width}x${height}, expected ${expected_width}x${expected_height}." >&2
  exit 1
fi
if ! awk -v actual="$resolution" -v expected="$expected_resolution" \
  'BEGIN {delta=actual-expected; if (delta < 0) delta=-delta; exit !(delta < 1e-9)}'; then
  echo "ERROR: /map resolution is $resolution, expected $expected_resolution." >&2
  exit 1
fi

echo "[3/6] Checking plain Twist and physical speed parameters"
cmd_info=$(timeout 6 ros2 topic info /cmd_vel -v 2>&1 || true)
if ! rg -q '^Type: geometry_msgs/msg/Twist$' <<<"$cmd_info"; then
  echo "$cmd_info" >&2
  echo "ERROR: /cmd_vel is not geometry_msgs/msg/Twist." >&2
  exit 1
fi
cmd_subscribers=$(awk '/Subscription count:/ {print $3}' <<<"$cmd_info")
if [[ "${cmd_subscribers:-0}" -ne 1 ]]; then
  echo "ERROR: /cmd_vel must have exactly one robot subscriber; found ${cmd_subscribers:-0}." >&2
  exit 1
fi
for node in controller_server behavior_server velocity_smoother collision_monitor; do
  value=$(timeout 6 ros2 param get "/$node" enable_stamped_cmd_vel 2>&1 || true)
  if ! rg -q 'False$' <<<"$value"; then
    echo "ERROR: /$node enable_stamped_cmd_vel is not False: $value" >&2
    exit 1
  fi
done
max_velocity=$(timeout 6 ros2 param get /velocity_smoother max_velocity 2>&1 || true)
if ! rg -q '\[0\.18, 0\.0, 1\.0\]' <<<"$max_velocity"; then
  echo "ERROR: unexpected velocity_smoother max_velocity: $max_velocity" >&2
  exit 1
fi

echo "[4/6] Checking map -> odom after 2D Pose Estimate"
if ! map_tf=$(python3 "$SCRIPT_DIR/tb3_tf_probe.py" map odom \
  --seconds 30 --min-updates 2 2>&1); then
  echo "$map_tf" >&2
  echo "ERROR: map -> odom is missing. Set 2D Pose Estimate in RViz first." >&2
  exit 1
fi
if ! rg -q '^target=map source=odom updates=([2-9]|[1-9][0-9]+) regressions=0 invalid=0$' <<<"$map_tf"; then
  echo "$map_tf" >&2
  echo "ERROR: unexpected map -> odom TF audit output." >&2
  exit 1
fi

echo "[5/6] Rechecking odom -> base_footprint"
if ! odom_tf=$(python3 "$SCRIPT_DIR/tb3_tf_probe.py" odom base_footprint \
  --seconds 30 --min-updates 2 2>&1); then
  echo "$odom_tf" >&2
  echo "ERROR: odom -> base_footprint is missing." >&2
  exit 1
fi
if ! rg -q '^target=odom source=base_footprint updates=([2-9]|[1-9][0-9]+) regressions=0 invalid=0$' <<<"$odom_tf"; then
  echo "$odom_tf" >&2
  echo "ERROR: unexpected odom -> base_footprint TF audit output." >&2
  exit 1
fi

echo "[6/6] Checking the NavigateToPose action server"
action_info=$(timeout 6 ros2 action info /navigate_to_pose 2>&1 || true)
if ! rg -q '^Action servers: 1$' <<<"$action_info"; then
  echo "$action_info" >&2
  echo "ERROR: /navigate_to_pose must have exactly one action server." >&2
  exit 1
fi

echo "OK: Nav2 is active; map=${width}x${height} @ ${resolution}m; robot sensors, Twist, speed limits, TF, and navigation action are valid."
