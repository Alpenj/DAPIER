#!/usr/bin/env bash
set -euo pipefail
task_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$task_directory/.venv/bin/python" "$task_directory/open_console.py" "$@"
