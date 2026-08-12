#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH=$(readlink -f -- "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)
# shellcheck source=tb3_real_env.sh
source "$SCRIPT_DIR/tb3_real_env.sh"
set -u

if [[ "$#" -gt 1 ]]; then
  echo "Usage: tb3-map-save [map_name]" >&2
  exit 2
fi

map_name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
if [[ ! "$map_name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "ERROR: map name may contain only letters, digits, dot, underscore, and hyphen." >&2
  exit 2
fi

map_dir="$HOME/maps"
map_prefix="$map_dir/$map_name"
if [[ -e "$map_prefix.yaml" || -e "$map_prefix.pgm" ]]; then
  echo "ERROR: map already exists: $map_prefix.{yaml,pgm}" >&2
  echo "Choose another name; existing maps are never overwritten." >&2
  exit 1
fi

map_topic_info=$(timeout 6 ros2 topic info /map 2>/dev/null || true)
map_publishers=$(awk '/Publisher count:/ {print $3}' <<<"$map_topic_info")
if [[ "${map_publishers:-0}" -lt 1 ]]; then
  echo "ERROR: /map has no publisher. Start tb3-slam first." >&2
  exit 1
fi

mkdir -p "$map_dir"
save_log=$(mktemp)
save_complete=false
cleanup() {
  rm -f "$save_log"
  if [[ "$save_complete" != true ]]; then
    rm -f -- "$map_prefix.yaml" "$map_prefix.pgm"
  fi
}
trap cleanup EXIT

if ! timeout 30 ros2 run nav2_map_server map_saver_cli -f "$map_prefix" >"$save_log" 2>&1; then
  cat "$save_log" >&2
  echo "ERROR: map_saver_cli failed." >&2
  exit 1
fi

if [[ ! -s "$map_prefix.yaml" || ! -s "$map_prefix.pgm" ]]; then
  cat "$save_log" >&2
  echo "ERROR: map saver returned, but one or more output files are missing." >&2
  exit 1
fi

if ! python3 "$SCRIPT_DIR/tb3_map_validate.py" "$map_prefix.yaml"; then
  cat "$save_log" >&2
  echo "ERROR: saved map failed structural validation; partial output was removed." >&2
  exit 1
fi
save_complete=true

echo "OK: map saved"
echo "  $map_prefix.yaml"
echo "  $map_prefix.pgm"
