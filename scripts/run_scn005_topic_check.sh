#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

PASS_LINE='PASS: SCN-005 target and chaser telemetry, TF, aliases, and markers publish correctly'

cleanup() {
  local status=$?
  trap - EXIT
  if ! make down >/dev/null 2>&1; then
    echo "ERROR: SCN-005 cleanup failed" >&2
    status=1
  fi
  exit "${status}"
}

on_signal() {
  exit "$1"
}

dump_logs() {
  docker compose logs --no-color physics ros2 sitl-json >&2 || true
}

trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

make down
SCENARIO="${SCENARIO:-mountain_two_drones}"
if [[ "${SCENARIO}" != "mountain_two_drones" ]]; then
  echo "ERROR: SCN-005 requires SCENARIO=mountain_two_drones" >&2
  exit 2
fi
export SCENARIO
# Dual image publication belongs to SCN-006.  The current single-camera path
# intentionally remains disabled for this telemetry-only stack.
export CAMERA_ENABLED=0

docker compose --profile mujoco up -d physics ros2 sitl-json

startup_deadline=$((SECONDS + 90))
while (( SECONDS < startup_deadline )); do
  running="$(docker compose --profile mujoco ps --status running --services)"
  if grep -qx physics <<<"${running}" \
      && grep -qx ros2 <<<"${running}" \
      && grep -qx sitl-json <<<"${running}"; then
    break
  fi
  sleep 2
done

running="$(docker compose --profile mujoco ps --status running --services)"
if ! grep -qx physics <<<"${running}" \
    || ! grep -qx ros2 <<<"${running}" \
    || ! grep -qx sitl-json <<<"${running}"; then
  echo "ERROR: SCN-005 services did not become ready within 90 seconds" >&2
  dump_logs
  exit 1
fi

# Ensure the marker subscriptions are genuinely late relative to publisher
# startup; the probe then requires transient-local endpoint QoS and delivery.
sleep 5
if ! docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash; if [[ -f /workspace/install/setup.bash ]]; then source /workspace/install/setup.bash; fi; cd /workspace; python3 scripts/two_drone_topic_probe.py --timeout-s 60'; then
  dump_logs
  exit 1
fi

# Cleanup is part of acceptance.  It must succeed before the only PASS line.
make down
remaining="$(docker compose --profile baseline --profile mujoco --profile viz ps --status running --services)"
if [[ -n "${remaining}" ]]; then
  echo "ERROR: services still running after SCN-005 cleanup: ${remaining}" >&2
  exit 1
fi

trap - EXIT INT TERM
printf '%s\n' "${PASS_LINE}"
