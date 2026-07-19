# dstrike

Docker Desktop for Mac drone simulation stack. It runs ArduPilot SITL, MuJoCo
physics, a ROS 2 MAVLink bridge, and RViz2/noVNC inside Docker.

No host ROS 2, ArduPilot, MuJoCo, RViz2, Python package, or alternate container
runtime install is required.

## Requirements

- macOS with Docker Desktop for Mac
- Docker Engine and Docker Compose plugin bundled with Docker Desktop
- Make, a shell, and a browser

## Quick Start

```bash
cp .env.example .env
make build
make up-viz
```

Open the browser viewer:

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

Stop everything:

```bash
make down
```

## Smoke Test

Run the full headless takeoff test:

```bash
make test-e2e
```

Expected success line:

```text
E2E PASS: heartbeat, arm, takeoff, odom altitude threshold reached
```

`make test-e2e` resets the Compose stack, starts the MuJoCo/SITL/ROS 2 services,
runs the takeoff test, and shuts the stack down. Run `make up-viz` again when
you want the browser viewer.

## What Runs

| Service | Purpose | Ports |
| --- | --- | --- |
| `sitl-json` | ArduCopter SITL in JSON external-physics mode | Internal UDP to `physics:9002` and `ros2:14550` |
| `physics` | MuJoCo physics, scenario markers, ground-truth odometry, camera publishing | Internal UDP `9002` |
| `ros2` | ROS 2 MAVLink bridge and command topics | Internal UDP `14550` |
| `viz` | RViz2 desktop streamed through noVNC | Host `127.0.0.1:6080` to container `6080/tcp` |

Only noVNC is published to the Mac host by default.

## Expected Topics

```text
/drone/heartbeat
/drone/odom
/sim/ground_truth/odom
/tf
/sim/scenario_markers
/sim/drone_markers
/sim/terrain_markers
/sim/nature_markers
/sim/camera/front/image_raw
/sim/camera/front/camera_info
```

Check the live topic set:

```bash
make topic-list
```

## Common Commands

```bash
make help
make preflight
make build
make up-viz
make test-e2e
make topic-list
make topic-hz TOPIC=/drone/odom
make bag-record DURATION=10
make logs LOG_FLAGS=
make down
```

Artifacts are written under:

```text
artifacts/ardupilot/
artifacts/rosbags/
artifacts/test-logs/
```

## Mountain terrain map

The mountain terrain work is specified in the
[Mountain terrain map plan](plans/mountain_terrain_map_plan.md). The map viewer uses
the committed deterministic heightfield, publishes the decimated surface plus
all generated trees and rocks with transient-local ROS 2 durability, and opens
an RViz view scaled for the 300 m map.

Generate the assets, run the automated marker check, or open the browser view:

```bash
make generate-map  # MAP-002
docker compose run --rm physics bash -lc "cd /workspace && SCENARIO=mountain_map python3 scripts/mujoco_terrain_collision_probe.py"  # MAP-003
SCENARIO=mountain_map bash scripts/run_map005_marker_check.sh
make up-map-viz
make test-map      # MAP-006 bounded full workflow
```

`make test-map` validates Compose, deterministic generation, both scenario
loads, MuJoCo terrain collision, safe startup/reset pose, marker content, the
landing-pad E2E flight, and the required regression suites. It tears down the
Compose stack before starting and on success, failure, or interruption. After a
failure, inspect the command output (and `docker compose logs` when a live check
failed), fix the cause, and rerun `make test-map`; no manual cleanup is required.
The landing-pad E2E check tears down visualization, so start `make up-map-viz`
again for the manual RViz signoff.

For manual MAP-005 signoff, open
`http://localhost:6080/vnc.html?autoconnect=true&resize=scale`, then confirm
RViz shows the green terrain surface, tree and rock markers, scenario markers,
the drone inside its yellow map-scale locator ring, and the live `Front Camera`
panel. The mountain spawn includes a finite yellow launch pad so the drone rests
level instead of balancing directly on a heightfield triangle.
Two yellow circles near the spawn are expected: the stationary 3 m-diameter
launch pad and the 5 m-diameter locator attached to the moving drone.

To verify real flight while watching RViz, leave the browser open and run:

```bash
make down
make up-map-viz
docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash && python3 scripts/mountain_vertical_flight_probe.py'
make down
```

The bounded probe waits for ArduPilot local-position readiness, then arms,
takes off relative to the launch height, holds for 20 seconds, commands a
4-second ascent and descent, lands on the pad, and disarms. It prints
`PASS: mountain vertical takeoff, hover, up/down, and landing verified` only
after every phase passes. All flight thresholds and timeouts are available via
`python3 scripts/mountain_vertical_flight_probe.py --help` inside the `ros2`
container.

## Configuration

Edit `.env` after copying `.env.example`.

| Variable | Default | Meaning |
| --- | --- | --- |
| `NOVNC_PORT` | `6080` | Browser viewer port on `127.0.0.1` |
| `VNC_GEOMETRY` | `1440x900x24` | Virtual desktop size matched to the saved RViz layout |
| `SCENARIO` | `landing_pad` | Scenario: `empty_world`, `landing_pad`, `obstacle_course`, or `mountain_map` |
| `CAMERA_ENABLED` | `true` | Publish the MuJoCo front camera topics |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | `640` / `480` | Camera image size |
| `CAMERA_FPS` | `10` | Camera publish rate |
| `ROS_DOMAIN_ID` | `42` | ROS 2 domain used inside containers |
| `ARDUPILOT_REF` | pinned SHA in `.env.example` | ArduPilot commit baked into the image |

Restart after config changes:

```bash
make down
make up-viz
```

## Troubleshooting

Browser does not open:

```bash
docker compose ps
docker compose logs viz
```

RViz2 opens but has no data:

```bash
make topic-list
docker compose logs ros2 physics viz
```

No heartbeat or takeoff fails:

```bash
docker compose logs sitl-json ros2
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /drone/heartbeat --once'
```

No JSON physics frames:

```bash
docker compose logs sitl-json physics
```

Camera topic missing:

```bash
grep -E "CAMERA_ENABLED|CAMERA_TOPIC|CAMERA_NAME" .env
docker compose logs physics
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/image_probe.py /sim/camera/front/image_raw'
```

Docker feels too slow or unstable:

```bash
make preflight
docker stats
```

Increase Docker Desktop CPU or memory limits, then restart the stack.
