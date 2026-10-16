#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
PASS_LINE='PASS: SCN-009 target straight terrain-parallel flight, chaser takeoff, intercept, and formation hold verified'

cleanup() {
  local status=$?
  trap - EXIT
  if [[ "${status}" -ne 0 ]]; then
    docker compose logs --tail 250 --no-color ros2 physics sitl-json >&2 || true
  fi
  if ! make down >/dev/null 2>&1; then
    echo 'ERROR: SCN-009 cleanup failed' >&2
    status=1
  fi
  exit "${status}"
}
on_signal() { exit "$1"; }
trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

export SCENARIO=mountain_two_drones
export DUAL_SITL=0
export CAMERA_ENABLED="${CAMERA_ENABLED:-1}"
make down
docker compose --profile mujoco up -d physics ros2 sitl-json

ready=0
for _ in $(seq 1 60); do
  if docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; ros2 topic info /scenario/state' >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 2
done
if [[ "${ready}" != 1 ]]; then
  echo 'ERROR: scenario controller did not become ready' >&2
  exit 1
fi

docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; cd /workspace; python3 scripts/e2e_mountain_two_drone.py'
make down
remaining="$(docker compose --profile baseline --profile mujoco --profile viz ps --status running --services)"
if [[ -n "${remaining}" ]]; then
  echo "ERROR: services still running after SCN-009 cleanup: ${remaining}" >&2
  exit 1
fi
trap - EXIT INT TERM
printf '%s\n' "${PASS_LINE}"
