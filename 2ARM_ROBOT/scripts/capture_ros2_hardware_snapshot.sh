#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: bash scripts/capture_ros2_hardware_snapshot.sh OUTPUT_DIRECTORY" >&2
  exit 2
fi

snapshot_dir="$1"
if [[ -e "${snapshot_dir}" && ! -d "${snapshot_dir}" ]]; then
  echo "ERROR: output path exists and is not a directory: ${snapshot_dir}" >&2
  exit 1
fi
if [[ -d "${snapshot_dir}" && -n "$(ls -A -- "${snapshot_dir}")" ]]; then
  echo "ERROR: output directory is not empty; refusing to overwrite: ${snapshot_dir}" >&2
  exit 1
fi

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

mkdir -p -- "${snapshot_dir}/samples"
printf '%s\n' "${ROS_DISTRO:-unknown}" > "${snapshot_dir}/ros_distro.txt"
date --utc --iso-8601=seconds > "${snapshot_dir}/captured_at_utc.txt"

ros2 node list > "${snapshot_dir}/nodes.txt"
ros2 topic list -t > "${snapshot_dir}/topics.txt"
ros2 service list -t > "${snapshot_dir}/services.txt"
ros2 action list -t > "${snapshot_dir}/actions.txt"
ros2 doctor --report > "${snapshot_dir}/doctor.txt" 2>&1 || true

: > "${snapshot_dir}/node_info.txt"
while IFS= read -r node_name; do
  [[ -z "${node_name}" ]] && continue
  {
    printf '===== %s =====\n' "${node_name}"
    ros2 node info "${node_name}" || true
    printf '\n'
  } >> "${snapshot_dir}/node_info.txt" 2>&1
done < "${snapshot_dir}/nodes.txt"

: > "${snapshot_dir}/topic_info.txt"
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
  } >> "${snapshot_dir}/topic_info.txt" 2>&1

  safe_name="${topic_name//\//_}"
  sample_path="${snapshot_dir}/samples/${safe_name}.txt"
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
done < "${snapshot_dir}/topics.txt"

{
  printf 'ros_distro=%s\n' "${ROS_DISTRO:-unknown}"
  printf 'node_count=%s\n' "$(wc -l < "${snapshot_dir}/nodes.txt")"
  printf 'candidate_topic_count=%s\n' "${candidate_topic_count}"
  printf 'sampled_topic_count=%s\n' "${sampled_topic_count}"
  printf 'camera_payload_captured=false\n'
  printf 'motion_commands_sent=false\n'
} > "${snapshot_dir}/summary.txt"

if [[ ${candidate_topic_count} -eq 0 ]]; then
  echo "NO_CANDIDATE_TOPICS: no running robot or sensor topics were discovered." >&2
  echo "Snapshot: ${snapshot_dir}" >&2
  exit 2
fi

echo "PASS: read-only ROS 2 hardware snapshot captured at ${snapshot_dir}"
