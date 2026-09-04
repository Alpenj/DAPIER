#!/usr/bin/env bash
set -euo pipefail
umask 077

readonly CONFIRMATION="VISIBLE_INSTALL_DAPIER_UDEV_RULES"

usage() {
  echo "Usage: $0 --confirm $CONFIRMATION" >&2
}
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 2 || "${1:-}" != "--confirm" || "${2:-}" != "$CONFIRMATION" ]]; then
  usage
  exit 2
fi
[[ -t 0 && -t 1 ]] || {
  echo "hardware alias installation requires an interactive TTY" >&2
  exit 2
}
script_dir="$(cd -P -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(realpath -e -- "${script_dir}/..")"
readonly rules_file="${repo_root}/config/99-dapier-hardware.rules"

validate_rules_file() {
  [[ -e "${rules_file}" && ! -L "${rules_file}" && -f "${rules_file}" ]] || return 1
  [[ "$(realpath -e -- "${rules_file}")" == "${rules_file}" ]] || return 1
  [[ "$(stat -c '%u' -- "${rules_file}")" == "$(id -u)" ]] || return 1
  mode="$(stat -c '%a' -- "${rules_file}")"
  mode_value=$((8#${mode}))
  (( (mode_value & 0022) == 0 ))
}

if ! validate_rules_file; then
  echo "canonical rules must be a current-user, non-symlink regular file without group/world writes: ${rules_file}" >&2
  exit 1
fi

source_identity="$(stat -c '%d:%i:%s' -- "${rules_file}")"
source_digest_before="$(sha256sum -- "${rules_file}" | awk '{print $1}')"
private_dir="$(mktemp -d /tmp/dapier-udev-rules.XXXXXX)"
readonly private_dir
staged_rules="${private_dir}/99-dapier-hardware.rules"
readonly staged_rules
cleanup() {
  rm -f -- "${staged_rules}"
  rmdir -- "${private_dir}"
}
trap cleanup EXIT

cp -- "${rules_file}" "${staged_rules}"
chmod 0400 "${staged_rules}"
source_digest_after="$(sha256sum -- "${rules_file}" | awk '{print $1}')"
staged_digest="$(sha256sum -- "${staged_rules}" | awk '{print $1}')"
if ! validate_rules_file \
  || [[ "$(stat -c '%d:%i:%s' -- "${rules_file}")" != "${source_identity}" ]] \
  || [[ "${source_digest_before}" != "${source_digest_after}" ]] \
  || [[ "${source_digest_before}" != "${staged_digest}" ]]; then
  echo "canonical rules changed during private staging; refusing installation" >&2
  exit 1
fi

sudo install -m 0644 "${staged_rules}" /etc/udev/rules.d/99-dapier-hardware.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
cleanup
trap - EXIT

for role in left_arm right_arm left_wrist_rgb right_wrist_rgb workspace_rgbd front_slam_rgbd; do
  path="/dev/dapier/${role}"
  if [[ -e "${path}" ]]; then
    printf '%-18s -> %s\n' "${role}" "$(readlink -f "${path}")"
  else
    printf 'WARNING: %-9s is not connected\n' "${role}" >&2
  fi
done
