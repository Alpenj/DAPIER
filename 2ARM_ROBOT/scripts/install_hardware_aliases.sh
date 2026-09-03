#!/usr/bin/env bash
set -euo pipefail

rules_file="${1:-${HOME}/.config/dapier/99-dapier-hardware.rules}"
if [[ ! -f "${rules_file}" ]]; then
  echo "missing local hardware rules: ${rules_file}" >&2
  exit 1
fi

sudo install -m 0644 "${rules_file}" /etc/udev/rules.d/99-dapier-hardware.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

for role in left_arm right_arm left_wrist_rgb right_wrist_rgb workspace_rgbd; do
  path="/dev/dapier/${role}"
  if [[ -e "${path}" ]]; then
    printf '%-18s -> %s\n' "${role}" "$(readlink -f "${path}")"
  else
    printf 'WARNING: %-9s is not connected\n' "${role}" >&2
  fi
done
