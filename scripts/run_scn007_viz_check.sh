#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

PASS_LINE='PASS: SCN-007 two-drone RViz2/noVNC view works locally'

cleanup() {
  local status=$?
  trap - EXIT
  if ! make down >/dev/null 2>&1; then
    echo "ERROR: SCN-007 cleanup failed" >&2
    status=1
  fi
  exit "${status}"
}

on_signal() {
  exit "$1"
}

trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

export SCENARIO="${SCENARIO:-mountain_two_drones}"
export CAMERA_ENABLED="${CAMERA_ENABLED:-1}"

make up-two-drone-viz

make down
remaining="$(docker compose --profile baseline --profile mujoco --profile viz ps --status running --services)"
if [[ -n "${remaining}" ]]; then
  echo "ERROR: services still running after SCN-007 cleanup: ${remaining}" >&2
  exit 1
fi

trap - EXIT INT TERM
printf '%s\n' "${PASS_LINE}"
