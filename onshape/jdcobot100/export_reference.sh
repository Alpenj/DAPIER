#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
reference_dir="${script_dir}/reference"
credential_file="${XDG_CONFIG_HOME:-${HOME}/.config}/onshape-to-robot/env"
exporter="${ONSHAPE_TO_ROBOT_BIN:-${HOME}/.local/bin/onshape-to-robot}"

if [[ -f "${credential_file}" ]]; then
    # shellcheck disable=SC1090
    source "${credential_file}"
fi

: "${ONSHAPE_API:?ONSHAPE_API is not set}"
: "${ONSHAPE_ACCESS_KEY:?ONSHAPE_ACCESS_KEY is not set}"
: "${ONSHAPE_SECRET_KEY:?ONSHAPE_SECRET_KEY is not set}"

"${exporter}" --safe "${reference_dir}/config.json"
"${exporter}" --safe "${reference_dir}/config.mujoco.json"
python3 "${script_dir}/normalize_joint_names.py" \
    "${reference_dir}/jdcobot100.urdf" \
    "${reference_dir}/jdcobot100.xml"
