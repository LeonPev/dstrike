#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

cleanup() {
  make down >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

make down

docker compose config --quiet

make generate-map
docker compose run --rm physics bash -lc \
  "cd /workspace && python3 scripts/map_smoke_test.py"
docker compose run --rm physics bash -lc \
  "cd /workspace && SCENARIO=mountain_map python3 scripts/mujoco_terrain_collision_probe.py"

SCENARIO=mountain_map bash scripts/run_map004_spawn_check.sh
SCENARIO=mountain_map bash scripts/run_map005_marker_check.sh

SCENARIO=landing_pad make test-e2e
docker compose run --rm physics bash -lc \
  "cd /workspace && pytest tests/test_scenarios.py tests/test_mujoco_quad.py tests/test_ros_visualization.py tests/test_terrain_map.py"

echo "PASS: MAP-006 map generation, physics terrain, safe spawn, markers, regressions, and cleanup are verified"
