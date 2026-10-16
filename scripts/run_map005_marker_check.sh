#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

cleanup() {
  make down >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

make down
SCENARIO=mountain_map make up-map-viz

deadline=$((SECONDS + 120))
while (( SECONDS < deadline )); do
  running="$(docker compose --profile mujoco --profile viz ps --status running --services)"
  if grep -qx physics <<<"${running}" && grep -qx ros2 <<<"${running}" && grep -qx viz <<<"${running}"; then
    break
  fi
  sleep 2
done

running="$(docker compose --profile mujoco --profile viz ps --status running --services)"
grep -qx physics <<<"${running}" || { echo "physics did not start" >&2; exit 1; }
grep -qx ros2 <<<"${running}" || { echo "ros2 did not start" >&2; exit 1; }
grep -qx viz <<<"${running}" || { echo "viz did not start" >&2; exit 1; }

docker compose exec -T viz bash -lc \
  'test "$SCENARIO" = mountain_map && test "$RVIZ_CONFIG" = /workspace/config/mountain_map.rviz'

# Give the physics loop time to publish before creating the late subscriber.
sleep 5
docker compose exec -T physics bash -lc \
  'source /opt/ros/humble/setup.bash && cd /workspace && python3 scripts/map_marker_probe.py'
