#!/usr/bin/env bash
set -euo pipefail

cleanup() {
  docker compose --profile mujoco down >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

make down
docker compose run --rm physics pytest -q tests/test_terrain_map.py

read -r spawn_x spawn_y spawn_z spawn_heading < <(
  docker compose run --rm physics python3 scripts/terrain_probe.py --fields
)
test -n "${spawn_x}" && test -n "${spawn_y}" && test -n "${spawn_z}" && test -n "${spawn_heading}"

SCENARIO=mountain_map SIM_RESET_AFTER_FRAMES=1000 docker compose up -d ros2
probe_log=/workspace/artifacts/test-logs/map004-spawn-probe.log
docker compose exec -T ros2 bash -lc "mkdir -p /workspace/artifacts/test-logs; source /opt/ros/humble/setup.bash; nohup python3 scripts/ros_ground_truth_spawn_probe.py --x '${spawn_x}' --y '${spawn_y}' --z '${spawn_z}' --heading '${spawn_heading}' --timeout 45 > '${probe_log}' 2>&1 </dev/null &"

ready_deadline=$((SECONDS + 20))
until grep -q '^READY:' artifacts/test-logs/map004-spawn-probe.log 2>/dev/null; do
  if (( SECONDS >= ready_deadline )); then
    cat artifacts/test-logs/map004-spawn-probe.log 2>/dev/null || true
    echo "ERROR: ground-truth probe did not become ready" >&2
    exit 1
  fi
  sleep 1
done

SCENARIO=mountain_map SIM_RESET_AFTER_FRAMES=1000 docker compose --profile mujoco up -d physics sitl-json
probe_deadline=$((SECONDS + 45))
until grep -q '^PASS:' artifacts/test-logs/map004-spawn-probe.log 2>/dev/null; do
  if (( SECONDS >= probe_deadline )); then
    cat artifacts/test-logs/map004-spawn-probe.log 2>/dev/null || true
    docker compose logs physics sitl-json
    echo "ERROR: bounded MAP-004 live probe timed out" >&2
    exit 1
  fi
  sleep 1
done
cat artifacts/test-logs/map004-spawn-probe.log
echo "PASS: MAP-004 terrain-aware startup and reset spawn are verified"
