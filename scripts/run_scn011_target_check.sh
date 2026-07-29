#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

PASS_LINE='PASS: SCN-011 dual-SITL target follows the configured terrain-parallel ping-pong route'

cleanup() {
  local status=$?
  trap - EXIT
  if [[ "${status}" -ne 0 ]]; then
    docker compose logs --tail 300 --no-color sitl-target sitl-chaser physics ros2 >&2 || true
  fi
  if ! make down >/dev/null 2>&1; then
    echo "ERROR: SCN-011 cleanup failed" >&2
    status=1
  fi
  exit "${status}"
}

on_signal() { exit "$1"; }
trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

export SCENARIO=mountain_two_drones
export DUAL_SITL=1
export CAMERA_ENABLED=0

make down
docker compose --profile dual-sitl up -d --force-recreate \
  physics ros2 sitl-target sitl-chaser
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; cd /workspace; python3 scripts/dual_sitl_target_probe.py --timeout-s 300'

make down
remaining="$(docker compose --profile baseline --profile mujoco --profile viz --profile dual-sitl ps --status running --services)"
if [[ -n "${remaining}" ]]; then
  echo "ERROR: services still running after SCN-011 cleanup: ${remaining}" >&2
  exit 1
fi

trap - EXIT INT TERM
printf '%s\n' "${PASS_LINE}"
