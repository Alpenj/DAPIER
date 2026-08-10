#!/usr/bin/env bash
set -eo pipefail

ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
workspace_root="${SO101_ROS2_WS:-${HOME}/so101_ros2_ws}"

source "${ros_setup}"
source "${workspace_root}/install/setup.bash"

set -u

export ROS_DOMAIN_ID="${SO101_ROS_DOMAIN_ID:-42}"

SO101_HARDWARE_TYPE="${SO101_HARDWARE_TYPE:-mock}"
SO101_USB_PORT="${SO101_USB_PORT:-}"

if [[ "${SO101_HARDWARE_TYPE}" != "real" && "${SO101_HARDWARE_TYPE}" != "mock" ]]; then
  printf 'ERROR: SO101_HARDWARE_TYPE must be real or mock (got: %s)\n' "${SO101_HARDWARE_TYPE}" >&2
  exit 2
fi

if [[ "${SO101_HARDWARE_TYPE}" == "real" && -z "${SO101_USB_PORT}" ]]; then
  printf 'ERROR: SO101_USB_PORT must be set explicitly for real hardware.\n' >&2
  exit 3
fi

if [[ "${SO101_HARDWARE_TYPE}" == "real" && ! -e "${SO101_USB_PORT}" ]]; then
  printf 'ERROR: serial device not found: %s\n' "${SO101_USB_PORT}" >&2
  exit 4
fi

if [[ "${SO101_HARDWARE_TYPE}" == "real" && ( ! -r "${SO101_USB_PORT}" || ! -w "${SO101_USB_PORT}" ) ]]; then
  printf 'ERROR: serial device is not readable and writable: %s\n' "${SO101_USB_PORT}" >&2
  exit 5
fi

if pgrep -x rviz2 >/dev/null; then
  printf 'ERROR: an existing rviz2 process is still running; close it before starting SO101.\n' >&2
  exit 6
fi

printf 'SO101 follower: hardware=%s port=%s domain=%s rviz=false joint_config=none (servo EEPROM retained)\n' \
  "${SO101_HARDWARE_TYPE}" "${SO101_USB_PORT}" "${ROS_DOMAIN_ID}"

exec ros2 launch so101_bringup follower.launch.py \
  "hardware_type:=${SO101_HARDWARE_TYPE}" \
  "usb_port:=${SO101_USB_PORT}" \
  use_rviz:=false
