#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH=$(readlink -f -- "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)
# shellcheck source=tb3_real_env.sh
source "$SCRIPT_DIR/tb3_real_env.sh"
set -u

if ! timeout 6 ros2 node list 2>/dev/null | awk '$0 == "/cartographer_node" {found=1} END {exit !found}'; then
  echo "ERROR: /cartographer_node is not running. Restart tb3-slam." >&2
  exit 1
fi

tmp_dir=$(mktemp -d)
trap 'rm -f "$tmp_dir/map.info" "$tmp_dir/map.err" "$tmp_dir/tf.out"; rmdir "$tmp_dir"' EXIT

map_topic_info=$(timeout 6 ros2 topic info /map 2>/dev/null || true)
map_publishers=$(awk '/Publisher count:/ {print $3}' <<<"$map_topic_info")
if [[ "${map_publishers:-0}" -lt 1 ]]; then
  echo "ERROR: /map has no publisher. Start tb3-slam first." >&2
  exit 1
fi

if ! timeout 8 ros2 topic echo --once /map --field info >"$tmp_dir/map.info" 2>"$tmp_dir/map.err"; then
  cat "$tmp_dir/map.err" >&2
  echo "ERROR: /map exists, but no OccupancyGrid sample arrived." >&2
  exit 1
fi

timeout 5 ros2 run tf2_ros tf2_echo map odom -r 2 >"$tmp_dir/tf.out" 2>&1 || true
if ! rg -q '^- Translation:' "$tmp_dir/tf.out"; then
  echo "ERROR: map -> odom transform is not available." >&2
  exit 1
fi

resolution=$(awk '/^resolution:/ {print $2; exit}' "$tmp_dir/map.info")
width=$(awk '/^width:/ {print $2; exit}' "$tmp_dir/map.info")
height=$(awk '/^height:/ {print $2; exit}' "$tmp_dir/map.info")

echo "OK: SLAM is publishing /map and map -> odom."
echo "Map: ${width} x ${height} cells, ${resolution} m/cell"
