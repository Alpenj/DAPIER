#!/usr/bin/env bash
set -euo pipefail
umask 077

readonly CONFIRMATION="VISIBLE_ROS2_SNAPSHOT_READONLY"

usage() {
  echo "Usage: bash scripts/capture_ros2_hardware_snapshot.sh OUTPUT_DIRECTORY --confirm $CONFIRMATION" >&2
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 3 || "$2" != "--confirm" || "$3" != "$CONFIRMATION" ]]; then
  usage
  exit 2
fi
[[ -t 0 && -t 1 ]] || {
  echo "ERROR: read-only ROS 2 access requires an interactive TTY" >&2
  exit 2
}

script_dir="$(cd -P -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -e "$1" || -L "$1" ]]; then
  echo "ERROR: refusing existing output path or symlink: $1" >&2
  exit 1
fi
raw_output_root="${script_dir}/../output"
if [[ -L "${raw_output_root}" ]]; then
  echo "ERROR: repository output root must not be a symlink" >&2
  exit 1
fi
output_root="$(realpath -m -- "${raw_output_root}")"
snapshot_dir="$(realpath -m -- "$1")"
case "$snapshot_dir/" in
  "$output_root/"*) ;;
  *) echo "ERROR: snapshot must be under $output_root" >&2; exit 1 ;;
esac
if [[ -e "${snapshot_dir}" || -L "${snapshot_dir}" ]]; then
  echo "ERROR: refusing to overwrite: ${snapshot_dir}" >&2
  exit 1
fi
snapshot_parent="$(dirname -- "${snapshot_dir}")"
mkdir -p -- "${snapshot_parent}"
snapshot_parent="$(realpath -e -- "${snapshot_parent}")"
case "${snapshot_parent}/" in
  "${output_root}/"*) ;;
  *) echo "ERROR: snapshot parent escaped ${output_root}" >&2; exit 1 ;;
esac
private_dir="$(mktemp -d "${snapshot_parent}/.ros2-snapshot.XXXXXX")"
cleanup() {
  rm -rf -- "${private_dir}"
}
trap cleanup EXIT

if ! command -v ros2 >/dev/null 2>&1; then
  for setup_file in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
    if [[ -f "${setup_file}" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "${setup_file}"
      set -u
      break
    fi
  done
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 is not available; source the installed ROS 2 environment first." >&2
  exit 1
fi

mkdir -- "${private_dir}/samples"
printf '%s\n' "${ROS_DISTRO:-unknown}" > "${private_dir}/ros_distro.txt"
date --utc --iso-8601=seconds > "${private_dir}/captured_at_utc.txt"

ros2 node list > "${private_dir}/nodes.txt"
ros2 topic list -t > "${private_dir}/topics.txt"
ros2 service list -t > "${private_dir}/services.txt"
ros2 action list -t > "${private_dir}/actions.txt"
ros2 doctor --report > "${private_dir}/doctor.txt" 2>&1 || true

: > "${private_dir}/node_info.txt"
while IFS= read -r node_name; do
  [[ -z "${node_name}" ]] && continue
  {
    printf '===== %s =====\n' "${node_name}"
    ros2 node info "${node_name}" || true
    printf '\n'
  } >> "${private_dir}/node_info.txt" 2>&1
done < "${private_dir}/nodes.txt"

: > "${private_dir}/topic_info.txt"
candidate_topic_count=0
sampled_topic_count=0
while IFS= read -r topic_line; do
  [[ -z "${topic_line}" ]] && continue
  topic_name="${topic_line%% *}"
  topic_type="${topic_line#*[}"
  topic_type="${topic_type%]}"
  case "${topic_name}" in
    /parameter_events|/rosout)
      continue
      ;;
  esac
  candidate_topic_count=$((candidate_topic_count + 1))
  {
    printf '===== %s %s =====\n' "${topic_name}" "${topic_type}"
    ros2 topic info -v "${topic_name}" || true
    printf '\n'
  } >> "${private_dir}/topic_info.txt" 2>&1

  safe_name="${topic_name//\//_}"
  sample_path="${private_dir}/samples/${safe_name}.txt"
  case "${topic_type}" in
    *sensor_msgs/msg/Image*|*sensor_msgs/msg/CompressedImage*)
      if ros2 topic echo "${topic_name}" --field header --once --timeout 3 \
        > "${sample_path}" 2>&1; then
        sampled_topic_count=$((sampled_topic_count + 1))
      fi
      ;;
    *sensor_msgs/msg/JointState*|*sensor_msgs/msg/CameraInfo*|*geometry_msgs/msg/Twist*|*nav_msgs/msg/Odometry*)
      if ros2 topic echo "${topic_name}" --once --timeout 3 \
        > "${sample_path}" 2>&1; then
        sampled_topic_count=$((sampled_topic_count + 1))
      fi
      ;;
  esac
done < "${private_dir}/topics.txt"

{
  printf 'ros_distro=%s\n' "${ROS_DISTRO:-unknown}"
  printf 'node_count=%s\n' "$(wc -l < "${private_dir}/nodes.txt")"
  printf 'candidate_topic_count=%s\n' "${candidate_topic_count}"
  printf 'sampled_topic_count=%s\n' "${sampled_topic_count}"
  printf 'camera_payload_captured=false\n'
  printf 'motion_commands_sent=false\n'
  printf 'hardware_execution=false\n'
} > "${private_dir}/summary.txt"

if [[ -e "${snapshot_dir}" || -L "${snapshot_dir}" ]]; then
  echo "ERROR: refusing output path created during capture: ${snapshot_dir}" >&2
  exit 1
fi
mv -Tn -- "${private_dir}" "${snapshot_dir}"
if [[ -e "${private_dir}" ]]; then
  echo "ERROR: atomic no-clobber finalize failed: ${snapshot_dir}" >&2
  exit 1
fi
trap - EXIT

if [[ ${candidate_topic_count} -eq 0 ]]; then
  echo "NO_CANDIDATE_TOPICS: no running robot or sensor topics were discovered." >&2
  echo "Snapshot: ${snapshot_dir}" >&2
  exit 2
fi

echo "PASS: read-only ROS 2 hardware snapshot captured at ${snapshot_dir}"
