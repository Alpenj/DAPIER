#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: $(basename "$0") [legacy teleop arguments]"
  echo "Physical motion is currently disabled."
  exit 0
fi

echo "BLOCKED: physical motion is disabled until local safety integration is complete." >&2
exit 2
