#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
credential_file="${XDG_CONFIG_HOME:-${HOME}/.config}/onshape-to-robot/env"
exporter="${ONSHAPE_TO_ROBOT_BIN:-${HOME}/.local/bin/onshape-to-robot}"

if [[ -f "${credential_file}" ]]; then
    # shellcheck disable=SC1090
    source "${credential_file}"
fi

: "${ONSHAPE_API:?ONSHAPE_API is not set}"
: "${ONSHAPE_ACCESS_KEY:?ONSHAPE_ACCESS_KEY is not set}"
: "${ONSHAPE_SECRET_KEY:?ONSHAPE_SECRET_KEY is not set}"

"${exporter}" --safe "${script_dir}/config.json"
"${exporter}" --safe "${script_dir}/config.mujoco.json"
python3 "${script_dir}/normalize_joint_names.py" \
    "${script_dir}/jdcobot100.urdf" \
    "${script_dir}/jdcobot100.xml"
